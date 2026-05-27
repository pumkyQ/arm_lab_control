#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import struct

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')
        
        # ROS2 구독자: joint_states 토픽으로부터 현재 위치 및 속도 수신
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # 가상 CAN 버스 (vcan0) 연결
        try:
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
            self.get_logger().info("PD Controller connected to vcan0")
        except Exception as e:
            self.get_logger().error(f"Failed to connect vcan0: {e}")
            return

        # PD 제어 게인 및 목표 위치 설정 (5개 관절)
        # 이 값들은 virtual_welcon_driver의 물리 시뮬레이션 특성에 따라 튜닝이 필요합니다.
        self.declare_parameter('kp', [8000.0, 8000.0, 8000.0, 8000.0, 8000.0])
        self.declare_parameter('kd', [400.0, 400.0, 400.0, 400.0, 400.0])
        self.declare_parameter('targets', [0.5, -0.5, 0.8, -0.2, 0.0]) # 예시 목표 각도 (rad)

    def joint_state_callback(self, msg):
        """JointState 메시지를 받을 때마다 PD 제어 법칙 계산 및 CAN 전송"""
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        targets = self.get_parameter('targets').value

        for i, name in enumerate(msg.name):
            if i >= 5:
                break
            
            curr_pos = msg.position[i]
            curr_vel = msg.velocity[i]
            target_pos = targets[i]
            
            # PD 제어 법칙: V = Kp * (target - curr) + Kd * (target_vel - curr_vel)
            # 목표 속도는 0으로 가정 (정지 상태 지향)
            error = target_pos - curr_pos
            error_dot = 0.0 - curr_vel
            
            control_output = (kp[i] * error) + (kd[i] * error_dot)
            
            # 전압 제한 (mV 단위, virtual_welcon_driver가 해석하는 범위)
            voltage_mv = int(max(-10000, min(10000, control_output)))
            
            # CAN 메시지 전송
            self.send_can_voltage(i, voltage_mv)

    def send_can_voltage(self, joint_idx, voltage_mv):
        """Joint Index를 CAN Node ID 및 Object Index로 변환하여 SDO 전송"""
        # Node 1: Joint 0, 1 | Node 2: Joint 2, 3 | Node 3: Joint 4
        node_id = (joint_idx // 2) + 1
        is_axis2 = (joint_idx % 2 != 0)
        
        can_id = 0x600 + node_id
        # virtual_welcon_driver.py의 receive_can_messages 로직과 일치시킴
        index = 0x2903 if is_axis2 else 0x2103
        
        # CANopen SDO Write 4-byte payload 구성
        data = bytearray([0x23]) # Command Specifier: Write 4-byte
        data.extend(struct.pack('<H', index)) # 2-byte Index (Little Endian)
        data.append(0x00) # Subindex
        data.extend(struct.pack('<i', voltage_mv)) # 4-byte signed int Data (Little Endian)
        
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            self.get_logger().error(f"CAN Send Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()