import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import threading

class VirtualWelconDriver(Node):
    def __init__(self):
        super().__init__('virtual_welcon_driver')
        
        # ROS2 Joint State Publisher 설정
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 5개 관절 초기 상태 (위치, 속도)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.positions = [0.0] * 5
        self.velocities = [0.0] * 5
        self.voltages = [0.0] * 5  # 각 모터에 인가된 가상 전압 (mV 단위)
        
        # 가상 CAN 버스 (vcan0) 연결
        try:
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
            self.get_logger().info("Successfully connected to virtual CAN (vcan0)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect vcan0: {e}")
            return

        # 백그라운드에서 CAN 패킷 수신을 위한 쓰레드 시작
        self.can_thread = threading.Thread(target=self.receive_can_messages, daemon=True)
        self.can_thread.start()

        # 50Hz (20ms) 주기로 로봇 조인트 상태 업데이트 및 발행
        self.timer = self.create_timer(0.02, self.update_and_publish_joints)

    def receive_can_messages(self):
        """가상 CAN 메시지를 해석하여 모터 전압 명령어 업데이트"""
        while rclpy.ok():
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue

                # SDO Write (0x600 + Node ID) 패킷 필터링
                # Node 1 (Joint 1,2), Node 2 (Joint 3,4), Node 3 (Joint 5)
                node_id = msg.arbitration_id - 0x600
                if 1 <= node_id <= 3:
                    command = msg.data[0]
                    index = int.from_bytes(msg.data[1:3], "little")
                    subindex = msg.data[3]
                    
                    # Q-Axis Voltage Write 오브젝트 감지 (코드의 0x2103, 0x2903 등)
                    if index == 0x2103:  # Node_ID의 Axis 1 (Joint 1, 3, 5)
                        voltage_mv = int.from_bytes(msg.data[4:8], "little", signed=True)
                        joint_idx = (node_id - 1) * 2
                        if joint_idx < 5:
                            self.voltages[joint_idx] = voltage_mv
                    elif index == 0x2903:  # Node_ID의 Axis 2 (Joint 2, 4)
                        voltage_mv = int.from_bytes(msg.data[4:8], "little", signed=True)
                        joint_idx = (node_id - 1) * 2 + 1
                        if joint_idx < 5:
                            self.voltages[joint_idx] = voltage_mv

            except Exception as e:
                self.get_logger().error(f"Error receiving CAN: {e}")

    def update_and_publish_joints(self):
        """전압에 기초한 매우 간단한 모터 물리 연산 수행 (전압 -> 가속도 -> 속도 -> 각도)"""
        dt = 0.02 # 20ms
        
        for i in range(5):
            # 전압(mV)에 비례한 가상 토크(가속도) 계산 후 속도 업데이트
            acceleration = self.voltages[i] * 0.0001  # 적절한 감속 스케일링
            self.velocities[i] += acceleration * dt
            # 속도에 비례한 마찰 저항 (자연 정지 유도)
            self.velocities[i] *= 0.9
            
            # 각도 업데이트
            self.positions[i] += self.velocities[i] * dt
            
            # 소프트 한계치 적용 (-1.57 rad ~ +1.57 rad)
            if self.positions[i] > 1.57:
                self.positions[i] = 1.57
                self.velocities[i] = 0.0
            elif self.positions[i] < -1.57:
                self.positions[i] = -1.57
                self.velocities[i] = 0.0

        # ROS2 JointState 토픽 발행
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
