import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys
import os
import time
import numpy as np

# 조교님 코드가 있는 경로를 파이썬 시스템 패스에 추가
sys.path.append(os.path.expanduser('~/Documents/양태헌교수님학연생/arm_lab_control/kitech_v1'))
from motor_control.cia402 import Cia402Protocol, Cia402Mode, Cia402Controlword, Cia402Object
from motor_control.can_bus import SocketCanBus

class RealWelconPDDriver(Node):
    def __init__(self):
        super().__init__('real_welcon_pd_driver')
        
        # 1. ROS2 설정 (실제 조인트 1, 3, 5만 사용)
        self.target_sub = self.create_subscription(JointState, 'target_joints', self.target_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.perf_pub = self.create_publisher(Float64MultiArray, '/joint_performance', 10)
        
        # 2. 제어 변수 정의
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.target_positions = [0.0] * 4  
        self.current_positions = [0.0] * 4 
        self.current_velocities = [0.0] * 4
        self.applied_voltages = [0.0] * 4
        self.actual_currents = [0.0] * 4  # 실제 흐르는 전류 모니터링 (mA)
        self.error_integral = [0.0] * 4  # I 제어를 위한 오차 누적분
        self.status_words = [0] * 4       # 드라이버 상태 모니터링용
        self.raw_positions = [0.0] * 4    # 엔코더 원시 값 저장 변수 추가

        # 자동 정렬 모드 플래그 및 목표치 (Raw Count 기준)
        self.alignment_complete = True    # 사용자 요청에 따라 일단 True로 설정하여 GUI 제어 우선
        self.align_targets_raw = [0.0] * 4 
        
        # 2.1 조인트 오프셋 설정 (Calibration)
        self.joint_offsets = [0.0] * 4

        # Node-Axis 조합 매핑 인덱스용 딕셔너리 생성
        # joint1 -> Node 3 Axis 1, joint2 -> Node 2 Axis 1, joint3 -> Node 1 Axis 1, joint4 -> Node 3 Axis 2
        self.node_axis_to_idx = {(3, 1): 0, (2, 1): 1, (1, 1): 2, (3, 2): 3}

        # 3. 하드웨어 상수 설정
        self.COUNTS_PER_RADIAN = 740.0  
        self.VELOCITY_COUNTS_PER_RADIAN = 740.0 

        # [FAULHABER 1506N012SR 및 기어비 기반 파라미터]
        # KE = 0.904 mV/rpm -> 0.008632 V/(rad/s)
        GEAR_RATIO = 406.4
        self.K_emf = 8.632 * GEAR_RATIO  # 약 3508.0 mV/(rad/s)
        
        # 4. 제어 게인 및 안전 제한값
        # 조인트 3(index 1)은 고장이므로 모든 게인을 0으로 설정하여 보호
        self.Kp_list = [30000.0, 30000.0, 20000.0, 20000.0]
        self.Ki_list = [400.0, 400.0, 300.0, 300.0]
        self.Kd_list = [3000.0, 3000.0, 1800.0, 1800.0]
        self.stiction_offset_list = [5500.0, 5500.0, 2500.0, 2500.0]
        self.i_limit = 1200.0                             # I항 최대 누적 제한 (mV)

        self.CURRENT_LIMIT_MA = 800.0    # 800mA 이상 시 보호를 위해 차단
        self.voltage_limit = 8000.0     # 안전 한계 8V
        self.ERROR_THRESH = 0.005        # 오실레이션 방지를 위해 데드밴드 소폭 확대

        # 5. Welcon 및 CAN 설정
        self.protocol = Cia402Protocol()
        try:
            self.bus = SocketCanBus(channel='can0', receive_timeout=0.0)
            self.bus.open()
            self.get_logger().info("Connected to can0 for HIL Control")
        except Exception as e:
            self.get_logger().error(f"CAN Connection Failed: {e}")
            return

        # 6. 하드웨어 초기화 (Node ID 1, 2, 3의 Axis 1, 2 활성화)
        self.real_axes = [(3, 1), (2, 1), (1, 1), (3, 2)]
        self.init_hardware()

        # 50Hz 제어 루프
        self.log_counter = 0 
        self.timer = self.create_timer(0.02, self.control_loop)

    def init_hardware(self):
        """모터 드라이버 NMT Start 및 Enable Operation 설정"""
        self.get_logger().info("Initializing Welcon Nodes (Joint 1, 2, 3, 4)...")
        self.bus.send(self.protocol.make_nmt_start(0))
        time.sleep(0.1)
        
        for node_id, axis in self.real_axes:
            # Voltage Mode (-11) 설정
            self.bus.send(self.protocol.make_axis_mode_sdo(node_id, axis, -11))
            time.sleep(0.02)
            
            # CiA402 상태 기어 전환
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, ctrl))
                time.sleep(0.02)
                
        self.get_logger().info("All axes are ENABLED and ready for PD control.")

    def target_callback(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.target_positions[i] = msg.position[idx]

    def update_encoder_feedback(self):
        """SDO 요청 및 수신 버퍼 파싱을 통해 현재 위치/속도 업데이트"""
        for node_id, axis in self.real_axes:
            pos_idx = 0x6064 if axis == 1 else 0x6864
            vel_idx = 0x606c if axis == 1 else 0x686c
            status_idx = 0x6041 if axis == 1 else 0x6841
            curr_idx = 0x6078 if axis == 1 else 0x6878
            self.bus.send(self.protocol.make_sdo_read(node_id, Cia402Object(pos_idx)))
            self.bus.send(self.protocol.make_sdo_read(node_id, Cia402Object(vel_idx)))
            self.bus.send(self.protocol.make_sdo_read(node_id, Cia402Object(status_idx)))
            self.bus.send(self.protocol.make_sdo_read(node_id, Cia402Object(curr_idx)))

        # SDO 응답을 충분히 기다려 데이터 누락 방지
        timeout_end = time.monotonic() + 0.020
        while time.monotonic() < timeout_end:
            frame = self.bus.recv(timeout=0.001)
            if frame is None:
                break
            
            # TPDO 파싱 (단일 축 드라이버용)
            feedback = self.protocol.parse_feedback(frame)
            if feedback:
                node_id = feedback.node_id
                axis = 1 # TPDO는 기본적으로 Axis 1로 처리
                if (node_id, axis) in self.node_axis_to_idx:
                    idx = self.node_axis_to_idx[(node_id, axis)]
                    if feedback.position_raw is not None:
                        self.current_positions[idx] = (feedback.position_raw / self.COUNTS_PER_RADIAN) - self.joint_offsets[idx]
                    if feedback.velocity_raw is not None:
                        self.current_velocities[idx] = feedback.velocity_raw / self.VELOCITY_COUNTS_PER_RADIAN
        
            # SDO 응답 파싱
            sdo_res = self.protocol.parse_sdo_response(frame)
            if sdo_res and sdo_res.value is not None:
                node_id = sdo_res.node_id
                axis = 1 if sdo_res.index in [0x6064, 0x606c, 0x6041, 0x6078] else 2 if sdo_res.index in [0x6864, 0x686c, 0x6841, 0x6878] else None
                if axis is not None and (node_id, axis) in self.node_axis_to_idx:
                    idx = self.node_axis_to_idx[(node_id, axis)]
                    val = sdo_res.value
                    if val > 0x7FFFFFFF: val -= 0x100000000

                    if sdo_res.index in [0x6064, 0x6864]:
                        self.raw_positions[idx] = val
                        self.current_positions[idx] = (val / self.COUNTS_PER_RADIAN) - self.joint_offsets[idx]
                    elif sdo_res.index in [0x606c, 0x686c]:
                        self.current_velocities[idx] = val / self.VELOCITY_COUNTS_PER_RADIAN
                    elif sdo_res.index in [0x6041, 0x6841]:
                        self.status_words[idx] = val
                    elif sdo_res.index in [0x6078, 0x6878]:
                        self.actual_currents[idx] = float(val)

    def control_loop(self):
        """PD + Back-EMF + Stiction Compensation 제어 루프"""
        self.update_encoder_feedback()
        dt = 0.02
        
        perf_data = [] # PlotJuggler용 통합 데이터 리스트
        
        for i, (node_id, axis) in enumerate(self.real_axes):
            # --- 자동 상태 복구 로직 ---
            # Statusword의 비트 3(Fault)이 1이거나, Operation Enabled 상태가 아니면 다시 활성화 시도
            status = self.status_words[i]
            if (status & 0x08): # Fault 상태인 경우
                if self.log_counter % 10 == 0:
                    self.get_logger().warn(f"Joint {self.joint_names[i]} (Node {node_id} Ax {axis}) FAULT (Stat: {hex(status)}). Resetting...")
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.FAULT_RESET))
                continue
            elif not (status & 0x04): # Operation Enabled가 아닌 경우
                if self.log_counter % 10 == 0:
                    self.get_logger().warn(f"Joint {self.joint_names[i]} (Node {node_id} Ax {axis}) Disabled (Stat: {hex(status)}). Enabling...")
                # Shutdown(0x06) -> Switch On(0x07) -> Enable Operation(0x0F) 순차 실행 필요 시
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.SHUTDOWN))
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.SWITCH_ON))
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.ENABLE_OPERATION))
                continue

            # 1. 목표값 선택 (정렬 완료 플래그에 따라 결정)
            if not self.alignment_complete:
                target = self.align_targets_raw[i] / self.COUNTS_PER_RADIAN
                current = self.raw_positions[i]
            else:
                target = self.target_positions[i]
                current = self.current_positions[i]

            error = target - current
            
            # 2. PID 피드백 계산
            v_pd = (self.Kp_list[i] * error) - (self.Kd_list[i] * self.current_velocities[i])
            
            # 3. 역기전력(Back-EMF) 피드포워드
            v_emf = self.K_emf * self.current_velocities[i]
            
            # 4. 정지 마찰력 보상 및 데드밴드 적용
            if abs(error) > self.ERROR_THRESH:
                # I항 계산 및 Anti-windup
                self.error_integral[i] += error * dt
                
                # ZeroDivisionError 방지: Ki가 0보다 클 때만 적분 제한 및 연산 수행
                if self.Ki_list[i] > 0:
                    i_limit_val = self.i_limit / self.Ki_list[i]
                    self.error_integral[i] = max(-i_limit_val, min(i_limit_val, self.error_integral[i]))
                    v_i = self.Ki_list[i] * self.error_integral[i]
                else:
                    v_i = 0.0

                v_stiction = self.stiction_offset_list[i] * np.sign(error)
                total_voltage = v_pd + v_i + v_emf + v_stiction
            else:
                # 오차 범위 내에서는 모터를 정지시켜 헌팅 방지
                total_voltage = 0.0
                self.error_integral[i] = 0.0
            
            # 5. 전압 클리핑 및 전송
            clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
            self.applied_voltages[i] = clamped_voltage
            
            volt_frame = self.protocol.make_q_axis_voltage_mv_sdo(
                node_id=node_id, 
                voltage_mv=int(clamped_voltage), 
                axis=axis
            )
            self.bus.send(volt_frame)
            
            # 성능 지표 데이터 수집 (Target, Actual, Error, Voltage)
            perf_data.extend([target, current, error, clamped_voltage])

        # 성능 지표 토픽 발행
        self.publish_performance(perf_data)

        # 로깅 및 ROS2 메시지 발행
        self.log_counter += 1
        if self.log_counter % 10 == 0:
            pos_str = ", ".join([f"{n}: {p:.3f} (Raw: {int(r)})" for n, p, r in zip(self.joint_names, self.current_positions, self.raw_positions)])
            volt_str = ", ".join([f"{n}: {v:.0f}mV" for n, v in zip(self.joint_names, self.applied_voltages)])
            stat_str = ", ".join([f"{n}: {hex(s)}" for n, s in zip(self.joint_names, self.status_words)])
            curr_str = ", ".join([f"{n}: {c:.0f}" for n, c in zip(self.joint_names, self.actual_currents)])
            self.get_logger().info(f"\n[POS] {pos_str}\n[VOLT] {volt_str}\n[STAT] {stat_str}\n[CURR] {curr_str}")
            self.log_counter = 0

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        msg.effort = self.applied_voltages  # 전압값을 effort 필드에 할당하여 모니터링 가능하게 함
        self.joint_pub.publish(msg)

    def publish_performance(self, data_list):
        """PlotJuggler에서 쉽게 확인하기 위해 리스트 데이터를 MultiArray로 발행"""
        if not data_list: return
        msg = Float64MultiArray()
        msg.data = data_list
        self.perf_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Stopping motors...")
        for node_id, axis in self.real_axes:
            try:
                self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.DISABLE_OPERATION))
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = RealWelconPDDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()