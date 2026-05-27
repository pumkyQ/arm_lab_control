import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import threading

class VirtualWelconDriver(Node):
    def __init__(self):
        super().__init__('virtual_welcon_driver')
        
        # 1. ROS2 Publisher & Subscriber 설정
        # 실제 움직임을 RViz2에 보낼 Topic
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        # GUI로부터 목표 각도를 받을 Topic
        self.target_sub = self.create_subscription(
            JointState, 
            'target_joints', 
            self.target_joint_callback, 
            10
        )
        
        # 2. 로봇 상태 변수 정의 (5자유도)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.positions = [0.0] * 5
        self.velocities = [0.0] * 5
        self.voltages = [0.0] * 5         # 현재 모터 인가 전압 (mV)
        self.target_positions = [0.0] * 5  # 목표 각도 (Radian)
        
        # 3. PD 제어 게인 (Gain) 설정 (시뮬레이션 물리 특성에 맞춤)
        self.Kp = 25000.0   # 비례 게인 (오차가 클수록 전압을 세게 줌)
        self.Kd = 1500.0    # 미분 게인 (속도가 빠르면 댐핑을 주어 오버슈트 방지)
        self.voltage_limit = 10000.0  # 웰콘 드라이버 최대 전압 제한 (±10V)

        # 4. 가상 CAN 버스 (vcan0) 연결
        try:
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
            self.get_logger().info("Successfully connected to virtual CAN (vcan0)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect vcan0: {e}")
            return

        # 백그라운드 CAN 패킷 수신 쓰레드 시작
        self.can_thread = threading.Thread(target=self.receive_can_messages, daemon=True)
        self.can_thread.start()

        # 50Hz (20ms) 주기로 제어 루프 및 물리 시뮬레이션 실행
        self.timer = self.create_timer(0.02, self.control_and_physics_loop)

    def target_joint_callback(self, msg: JointState):
        """GUI 조작 시 목표 각도(target_joints)를 업데이트하는 콜백 함수"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.target_positions[i] = msg.position[idx]

    def receive_can_messages(self):
        """[기존 기능 유지] 외부 SDO 전압 명령(set_axis_voltages.py) 수신 처리"""
        while rclpy.ok():
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue

                node_id = msg.arbitration_id - 0x600
                if 1 <= node_id <= 3:
                    index = int.from_bytes(msg.data[1:3], "little")
                    
                    if index == 0x2103:  # Node Axis 1 (Joint 1, 3, 5)
                        voltage_mv = int.from_bytes(msg.data[4:8], "little", signed=True)
                        joint_idx = (node_id - 1) * 2
                        if joint_idx < 5:
                            self.voltages[joint_idx] = float(voltage_mv)
                            # 외부 전압 직접 명령이 들어오면 해당 축의 목표각도를 현재각도로 동기화 (PD제어 충돌 방지)
                            self.target_positions[joint_idx] = self.positions[joint_idx]
                            
                    elif index == 0x2903:  # Node Axis 2 (Joint 2, 4)
                        voltage_mv = int.from_bytes(msg.data[4:8], "little", signed=True)
                        joint_idx = (node_id - 1) * 2 + 1
                        if joint_idx < 5:
                            self.voltages[joint_idx] = float(voltage_mv)
                            self.target_positions[joint_idx] = self.positions[joint_idx]

            except Exception as e:
                pass

    def control_and_physics_loop(self):
        """PD 제어 연산 및 모터 물리 상태 업데이트 루프 (50Hz)"""
        dt = 0.02 # 20ms
        
        for i in range(5):
            # 1. PD 제어기 (위치 제어 루프)
            # 외부 SDO 전압 입력이 없을 때(0일 때)만 PD 제어기가 작동하도록 설정
            error = self.target_positions[i] - self.positions[i]
            
            # PD 제어 공식 계산 (Kp * error - Kd * velocity)
            pd_voltage = (self.Kp * error) - (self.Kd * self.velocities[i])
            
            # 전압 제한 (±10,000 mV)
            self.voltages[i] = max(-self.voltage_limit, min(self.voltage_limit, pd_voltage))

            # 2. 가상 물리 법칙 연산 (전압 -> 속도 -> 각도)
            # 가속도 = 전압에 비례
            acceleration = self.voltages[i] * 0.0001
            self.velocities[i] += acceleration * dt
            self.velocities[i] *= 0.85  # 자연스러운 마찰 댐핑 효과
            
            # 각도 업데이트
            self.positions[i] += self.velocities[i] * dt
            
            # 하드웨어 기하학적 한계 적용 (-1.57 ~ 1.57 Radian)
            if self.positions[i] > 1.57:
                self.positions[i] = 1.57
                self.velocities[i] = 0.0
            elif self.positions[i] < -1.57:
                self.positions[i] = -1.57
                self.velocities[i] = 0.0

        # 3. RViz2 시각화를 위해 최종 조인트 각도 발행
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = self.velocities
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    driver = VirtualWelconDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
