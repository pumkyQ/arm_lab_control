import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import os
import time

# 조교님 코드가 있는 경로를 파이썬 시스템 패스에 추가
sys.path.append(os.path.expanduser('~/Documents/양태헌교수님학연생/arm_lab_control/kitech_v1'))
from motor_control.cia402 import Cia402Protocol, Cia402Mode, Cia402Controlword, Cia402Object
from motor_control.can_bus import SocketCanBus

class RealWelconDriver(Node):
    def __init__(self):
        super().__init__('real_welcon_driver')
        
        # 1. ROS2 설정 (우리가 사용할 실제 조인트 1, 3, 5만 선언)
        self.target_sub = self.create_subscription(JointState, 'target_joints', self.target_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 2. 제어 변수 정의 (URDF와 1:1 매칭)
        self.joint_names = ['joint1', 'joint3', 'joint5']
        self.target_positions = [0.0] * 3  
        self.current_positions = [0.0] * 3 
        self.current_velocities = [0.0] * 3
        self.applied_voltages = [0.0] * 3
        
        # 2.1 조인트 오프셋 설정 (Calibration)
        # 손가락을 일자로 폈을 때 터미널에 나오는 값을 여기에 입력하면 해당 위치가 0이 됩니다.
        # 예: joint1이 -0.010으로 나오면 여기에 -0.010을 입력
        self.joint_offsets = [-0.010, 0.007, -0.005]

        # Node ID 매핑 인덱스용 딕셔너리 생성 (수신된 CAN 패킷의 Node ID 구별용)
        # joint1 -> Node 1, joint3 -> Node 2, joint5 -> Node 3
        self.node_to_idx = {1: 0, 2: 1, 3: 2}

        # 3. 엔코더 단위 변환 상수 (★ 로봇 핑거 하드웨어 사양에 맞춰 수정 필요)
        # 예시: 모터 엔코더 펄스 수와 기어 감속비를 감안하여 1 Radian당 발생하는 펄스 양
        self.COUNTS_PER_RADIAN = 100000.0  
        self.VELOCITY_COUNTS_PER_RADIAN = 100000.0 # 속도 단위가 pulse/s 라면 위치 상수와 동일

        # 4. 실제 제어 게인 및 마찰 오프셋 설정
        self.Kp = 12000.0   
        self.Kd = 400.0     
        self.voltage_limit = 9500.0     # 9.5V 안전 한계선
        self.stiction_offset = 4500.0   # 기어 마찰력을 이기기 위한 4.5V 밀어주기 전압

        # 5. Welcon 드라이버 프로토콜 초기화
        self.protocol = Cia402Protocol()
        
        # 6. 실제 SocketCAN (can0) 연결
        try:
            # 50Hz 제어 루프 진입 전 버퍼에 쌓인 모든 데이터를 받기 위해 
            # 기본 receive_timeout은 0.0으로 설정 (Non-blocking 느낌)
            self.bus = SocketCanBus(channel='can0', receive_timeout=0.0)
            self.bus.open()
            self.get_logger().info("Successfully connected to REAL CAN (can0)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect can0: {e}. Is PEAK-USB connected?")
            return

        # 7. 하드웨어 초기화 (NMT Start + Enable) - 실제 구동할 3개 포트만 활성화
        self.real_axes = [(1,1), (2,1), (3,1)]
        self.init_hardware()

        # 50Hz (20ms) 주기로 실시간 제어 루프 실행
        self.log_counter = 0 # 터미널 출력 조절용 카운터
        self.timer = self.create_timer(0.02, self.control_loop)

    def init_hardware(self):
        """실제 모터 드라이버들을 켜고 구동 가능 상태(Enable)로 만듭니다."""
        self.get_logger().info("Initializing Welcon Hardware (Joint 1, 3, 5)...")
        # NMT Start
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)
        
        for node_id, axis in self.real_axes:
            mode_frame = self.protocol.make_axis_mode_sdo(node_id, axis, -11)
            self.bus.send(mode_frame)
            time.sleep(0.02)
            
            for label, ctrl in (
                ("fault reset", Cia402Controlword.FAULT_RESET),
                ("shutdown", Cia402Controlword.SHUTDOWN),
                ("switch on", Cia402Controlword.SWITCH_ON),
                ("enable operation", Cia402Controlword.ENABLE_OPERATION)
            ):
                frame = self.protocol.make_axis_controlword_sdo(node_id, axis, ctrl)
                self.bus.send(frame)
                time.sleep(0.02)
                
        self.get_logger().info("Joint 1, 3, 5 Active axes enabled in Voltage Mode!")

    def target_callback(self, msg: JointState):
        """GUI 조작 시 목표 각도 업데이트"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.target_positions[i] = msg.position[idx]

    def update_encoder_feedback(self):
        """CAN 버스로부터 현재 쌓여있는 모든 피드백(TPDO)을 읽어와서 각 축의 엔코더 정보를 업데이트합니다."""
        received_any = False

        # 1. TPDO가 자동으로 오지 않으므로, SDO를 통해 현재 위치(0x6064)를 직접 요청합니다.
        for node_id, axis in self.real_axes:
            pos_idx = 0x6064 if axis == 1 else 0x6864
            read_frame = self.protocol.make_sdo_read(node_id, Cia402Object(pos_idx))
            self.bus.send(read_frame)

        debug_ids = set()

        while True:
            frame = self.bus.recv(timeout=0.0) # 비차단(Non-blocking)으로 버퍼 비우기
            if frame is None:
                break # 더 이상 읽을 패킷이 없으면 루프 탈출
            
            # 디버깅: 수신되는 모든 CAN ID 기록 (10번 루프마다 출력될 것)
            if self.log_counter == 0:
                debug_ids.add(hex(frame.can_id))

            # cia402.py 라이브러리로 TPDO 패킷 파싱 시도
            feedback = self.protocol.parse_feedback(frame)
            if feedback is not None:
                node_id = feedback.node_id
                received_any = True
                
                # 우리가 제어하는 Node ID (1, 2, 3)인 경우에만 갱신
                if node_id in self.node_to_idx:
                    idx = self.node_to_idx[node_id]
                    
                    # TPDO 데이터에 오프셋 적용
                    if feedback.position_raw is not None:
                        self.current_positions[idx] = (feedback.position_raw / self.COUNTS_PER_RADIAN) - self.joint_offsets[idx]
                    if feedback.velocity_raw is not None:
                        self.current_velocities[idx] = feedback.velocity_raw / self.VELOCITY_COUNTS_PER_RADIAN
        
            # SDO 응답 파싱 (직접 요청한 위치 데이터 확인)
            sdo_res = self.protocol.parse_sdo_response(frame)
            if sdo_res is not None and sdo_res.value is not None:
                node_id = sdo_res.node_id
                if node_id in self.node_to_idx:
                    idx = self.node_to_idx[node_id]
                    # Position Actual Value (0x6064 또는 0x6864)인 경우 처리
                    if sdo_res.index in [0x6064, 0x6864]:
                        # 32비트 부호 있는 정수로 변환 (Unsigned -> Signed)
                        val = sdo_res.value
                        if val > 0x7FFFFFFF: val -= 0x100000000
                        # SDO 응답 데이터에 오프셋 적용
                        self.current_positions[idx] = (val / self.COUNTS_PER_RADIAN) - self.joint_offsets[idx]
                        received_any = True

        if self.log_counter == 0:
            if not received_any:
                self.get_logger().warning(f"No valid TPDO parsed. Seen IDs: {list(debug_ids)}")
            else:
                self.get_logger().debug(f"Received valid TPDOs, current IDs: {list(debug_ids)}")

    def control_loop(self):
        """실제 엔코더 피드백 기반 1:1 매칭 제어 루프"""
        # 1. 전압 인가 직전에 현재 모터의 최신 실제 위치/속도 업데이트
        self.update_encoder_feedback()
        
        for i, (node_id, axis) in enumerate(self.real_axes):
            # 오차 계산 (진짜 센서 각도 기준!)
            error = self.target_positions[i] - self.current_positions[i]
            
            # 1. PD 제어 값 계산
            pd_voltage = (self.Kp * error) - (self.Kd * self.current_velocities[i])
            
            # 2. 정지 마찰력 보상 및 데드밴드 적용 (로직 개선)
            if abs(error) > 0.01:  # 정밀도를 위해 데드밴드를 0.01로 조정
                # 에러가 존재하면 항상 마찰 보상 전압을 방향에 맞춰 추가
                pd_voltage += (self.stiction_offset if error > 0 else -self.stiction_offset)
                clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, pd_voltage))
            else:
                clamped_voltage = 0.0
            
            self.applied_voltages[i] = clamped_voltage
            # 4. SDO 전압 전송
            volt_frame = self.protocol.make_q_axis_voltage_mv_sdo(
                node_id=node_id, 
                voltage_mv=int(clamped_voltage), 
                axis=axis
            )
            self.bus.send(volt_frame)

        # 4.5 터미널에 실제 각도 출력 (약 5Hz 주기로 출력)
        self.log_counter += 1
        if self.log_counter % 10 == 0:
            pos_str = ", ".join([f"{name}: {pos:.3f}" for name, pos in zip(self.joint_names, self.current_positions)])
            volt_str = ", ".join([f"{name}: {volt:.0f}mV" for name, volt in zip(self.joint_names, self.applied_voltages)])
            self.get_logger().info(f"[Real] Angles(rad): {pos_str}")
            self.get_logger().info(f"[Real] Voltages: {volt_str}")
            self.log_counter = 0

        # 5. 수신한 실제 각도를 RViz2에 발행
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down... Stopping Joint 1, 3, 5.")
        for node_id, axis in self.real_axes:
            try:
                self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
                self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.DISABLE_OPERATION))
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = RealWelconDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()