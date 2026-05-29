import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import os
import time
import numpy as np

# 조교님 코드가 있는 경로를 파이썬 시스템 패스에 추가
sys.path.append(os.path.expanduser('~/Documents/양태헌교수님학연생/arm_lab_control/kitech_v1'))
from motor_control.cia402 import Cia402Protocol, Cia402Mode, Cia402Controlword, Cia402Object
from motor_control.can_bus import SocketCanBus

class RealWelconPositionDriver(Node):
    def __init__(self):
        super().__init__('real_welcon_position_driver')
        
        # 1. ROS2 설정 (GUI에서 target_joints를 발행하면 이를 구독)
        self.target_sub = self.create_subscription(JointState, 'target_joints', self.target_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 2. 제어 변수 정의
        self.joint_names = ['joint1', 'joint3', 'joint5']
        self.target_positions = [0.0] * 3  
        self.current_positions = [0.0] * 3 
        self.current_velocities = [0.0] * 3
        self.applied_voltages = [0.0] * 3
        self.error_integral = [0.0] * 3  # I 제어를 위한 오차 누적분
        self.status_words = [0] * 3       # 드라이버 상태 모니터링
        
        # 2.1 조인트 오프셋 설정 (Calibration 값 반영)
        # 로봇을 일자로 폈을 때 터미널 로그의 'C' 값을 여기에 입력하세요.
        # 여기에 Raw 값을 넣으면, Current Position(C)이 0.0으로 정렬됩니다.
        # -955/740 = -1.2905, -337/740 = -0.4554
        self.joint_offsets = [-1.2905, 0.0, -0.4554]
        self.raw_positions = [0.0] * 3 # 오프셋 적용 전 값 모니터링용

        # 2.2 조인트 방향 설정 (1.0: 정방향, -1.0: 역방향)
        # RViz에서 로봇이 반대로 움직인다면 해당 조인트를 -1.0으로 바꾸세요.
        self.joint_directions = [1.0, 1.0, 1.0]

        # 2.2 조인트 안전 한계 설정 (단위: Radian, 예: -1.57 ~ 1.57)
        self.joint_min_limits = [-0.5, -1.57, -1.57]
        self.joint_max_limits = [0.425, 1.57, 1.57]

        # Node ID 매핑 (Joint 1: Node 1, Joint 3: Node 2, Joint 5: Node 3)
        self.node_to_idx = {1: 0, 2: 1, 3: 2}

        # 3. 하드웨어 상수 및 제어 게인 설정
        self.COUNTS_PER_RADIAN = 740.0  
        self.VELOCITY_COUNTS_PER_RADIAN = 740.0 

        # 역기전력 계수 (Back-EMF)
        GEAR_RATIO = 406.4
        self.K_emf = 8.632 * GEAR_RATIO  # mV/(rad/s)
        
        # 제어 게인 (원하는 각도 정렬을 위해 Kp를 충분히 확보)
        # 각 조인트의 무게와 마찰력에 따라 차등 적용 (Index 0: joint1, 1: joint3, 2: joint5)
        self.Kp_list = [30000.0, 35000.0, 15000.0]
        self.Ki_list = [500.0, 800.0, 200.0]             # 조인트 3 게인 하향 조정
        self.Kd_list = [800.0, 1000.0, 500.0]             # 진동 방지를 위해 Kd도 함께 상향
        self.stiction_offset_list = [6500.0, 6500.0, 4800.0] # 조인트 3 전압 하향 조정
        self.i_limit = 2000.0                             # I항 최대 전압 제한 (mV)

        self.voltage_limit = 9800.0     # 9.8V 제한
        self.ERROR_THRESH = 0.001        # 목표 각도 도달 허용 오차 (약 0.057도)

        # 4. Welcon 및 CAN 설정
        self.protocol = Cia402Protocol()
        try:
            self.bus = SocketCanBus(channel='can0', receive_timeout=0.0)
            self.bus.open()
            self.get_logger().info("Position Controller connected to can0")
        except Exception as e:
            self.get_logger().error(f"CAN Connection Failed: {e}")
            return

        # 5. 하드웨어 초기화
        self.real_axes = [(1, 1), (2, 1), (3, 1)]
        self.init_hardware()

        # 50Hz 제어 루프 (20ms)
        self.log_counter = 0 
        self.timer = self.create_timer(0.02, self.control_loop)

    def init_hardware(self):
        self.get_logger().info("Initializing Hardware for Position Alignment...")
        self.bus.send(self.protocol.make_nmt_start(0))
        time.sleep(0.1)
        
        for node_id, axis in self.real_axes:
            # Voltage Mode (-11) 설정
            self.bus.send(self.protocol.make_axis_mode_sdo(node_id, axis, -11))
            time.sleep(0.02)
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, ctrl))
                time.sleep(0.02)
        self.get_logger().info("Hardware Ready.")

    def target_callback(self, msg: JointState):
        """GUI(joint_state_publisher_gui 등)에서 보낸 목표 각도를 업데이트"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                # 안전을 위해 목표 각도를 제한 범위 내로 클램핑
                clamped_target = max(self.joint_min_limits[i], min(self.joint_max_limits[i], msg.position[idx]))
                self.target_positions[i] = clamped_target

    def update_feedback(self):
        """SDO를 통해 위치와 속도 데이터를 모두 읽어옴"""
        for node_id, axis in self.real_axes:
            pos_obj = Cia402Object(0x6064 if axis == 1 else 0x6864)
            vel_obj = Cia402Object(0x606c if axis == 1 else 0x686c)
            status_obj = Cia402Object(0x6041 if axis == 1 else 0x6841)
            self.bus.send(self.protocol.make_sdo_read(node_id, pos_obj))
            self.bus.send(self.protocol.make_sdo_read(node_id, vel_obj))
            self.bus.send(self.protocol.make_sdo_read(node_id, status_obj))

        # CAN 버스에서 응답을 수집 (시간을 약간 더 확보)
        timeout_end = time.monotonic() + 0.008  # 8ms 동안 버퍼 확인
        while time.monotonic() < timeout_end:
            frame = self.bus.recv(timeout=0.001)
            if frame is None: continue
            
            sdo_res = self.protocol.parse_sdo_response(frame)
            if sdo_res and sdo_res.value is not None and sdo_res.node_id in self.node_to_idx:
                idx = self.node_to_idx[sdo_res.node_id]
                val = sdo_res.value
                
                if sdo_res.abort_code:
                    self.get_logger().error(f"Node {sdo_res.node_id} SDO Error: {hex(sdo_res.abort_code)}")

                if val > 0x7FFFFFFF: val -= 0x100000000
                
                # 위치 데이터 업데이트
                if sdo_res.index in [0x6064, 0x6864]:
                    raw_pos = (val / self.COUNTS_PER_RADIAN)
                    self.raw_positions[idx] = raw_pos
                    # 방향과 오프셋을 적용하여 실제 각도 계산
                    self.current_positions[idx] = (raw_pos - self.joint_offsets[idx]) * self.joint_directions[idx]
                # 속도 데이터 업데이트 (이 부분이 있어야 Kd 제어가 작동함)
                elif sdo_res.index in [0x606c, 0x686c]:
                    if val > 0x7FFFFFFF: val -= 0x100000000
                    self.current_velocities[idx] = val / self.VELOCITY_COUNTS_PER_RADIAN
                # 상태 워드(Statusword) 읽기 추가
                elif sdo_res.index in [0x6041, 0x6841]:
                    self.status_words[idx] = val
                    # 비트 3(Fault) 확인
                    if val & 0x08:
                        self.get_logger().error(f"Joint {idx+1} (Node {sdo_res.node_id}) is in FAULT state!")

    def control_loop(self):
        self.update_feedback()
        dt = 0.02 # 50Hz
        
        for i, (node_id, axis) in enumerate(self.real_axes):
            # --- 자동 상태 복구 로직 추가 ---
            # Fault(bit 3)이 떴거나 Operation Enabled(bit 2)가 아니면 재활성화
            status = self.status_words[i]
            if (status & 0x08) or not (status & 0x04):
                if self.log_counter == 0:
                    self.get_logger().warn(f"Joint {node_id} stat {hex(status)}: Resetting/Enabling...")
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.FAULT_RESET))
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.ENABLE_OPERATION))

            # 목표값이 배열 범위를 벗어나지 않도록 안전장치
            if i >= len(self.target_positions): break
            
            error = self.target_positions[i] - self.current_positions[i]
            
            # PID 제어 + Back-EMF 보상
            v_pd = (self.Kp_list[i] * error) - (self.Kd_list[i] * self.current_velocities[i])
            v_emf = self.K_emf * self.current_velocities[i]
            
            if abs(error) > self.ERROR_THRESH:
                # 오차 누적 (I항)
                self.error_integral[i] += error * dt
                # Anti-windup (I항 제한)
                self.error_integral[i] = max(-self.i_limit/self.Ki_list[i], min(self.i_limit/self.Ki_list[i], self.error_integral[i]))
                v_i = self.Ki_list[i] * self.error_integral[i]

                # 정지 마찰력을 이겨내고 목표 위치로 밀어줌
                v_stiction = self.stiction_offset_list[i] * np.sign(error)
                total_voltage = v_pd + v_i + v_emf + v_stiction
            else:
                # 목표 범위 내에 들어오면 전압을 차단하여 떨림(Hunting) 방지
                total_voltage = 0.0
                self.error_integral[i] = 0.0 # 정지 시 적분 초기화

            # 물리적 한계 도달 시 해당 방향 전압 차단 (2차 안전 장치)
            if (self.current_positions[i] >= self.joint_max_limits[i] and total_voltage > 0) or \
               (self.current_positions[i] <= self.joint_min_limits[i] and total_voltage < 0):
                total_voltage = 0.0
            
            clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
            self.applied_voltages[i] = clamped_voltage
            
            self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(node_id, int(clamped_voltage), axis))

        # 디버깅을 위한 터미널 출력 (10번 루프에 한 번씩, 약 5Hz)
        self.log_counter += 1
        if self.log_counter >= 10:
            pos_info = " | ".join([f"{n}: T={t:.3f}, C={c:.3f} (Raw={r:.3f})" for n, t, c, r in zip(self.joint_names, self.target_positions, self.current_positions, self.raw_positions)])
            volt_info = " | ".join([f"{n}: {v:.0f}mV" for n, v in zip(self.joint_names, self.applied_voltages)])
            stat_info = " | ".join([f"{n}: {hex(s)}" for n, s in zip(self.joint_names, self.status_words)])
            self.get_logger().info(f"\n[POS] {pos_info}\n[VOLT] {volt_info}\n[STAT] {stat_info}")
            self.log_counter = 0

        # RViz2 동기화를 위해 joint_states 발행
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        self.joint_pub.publish(msg)

    def destroy_node(self):
        for node_id, axis in self.real_axes:
            self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealWelconPositionDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()