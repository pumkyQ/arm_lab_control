import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import os
import time

# 조교님 코드가 있는 경로를 파이썬 시스템 패스에 추가
sys.path.append(os.path.expanduser('~/Documents/양태헌교수님학연생/arm_lab_control/kitech_v1'))
from motor_control.cia402 import Cia402Protocol, Cia402Mode, Cia402Controlword
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
        
        # 3. 실제 제어 게인 및 마찰 오프셋 설정
        self.Kp = 12000.0   
        self.Kd = 400.0     
        self.voltage_limit = 9500.0     # 9.5V 안전 한계선
        self.stiction_offset = 4500.0   # 기어 마찰력을 이기기 위한 4.5V 밀어주기 전압

        # 4. Welcon 드라이버 프로토콜 초기화
        self.protocol = Cia402Protocol()
        
        # 5. 실제 SocketCAN (can0) 연결
        try:
            self.bus = SocketCanBus(channel='can0', receive_timeout=0.001)
            self.bus.open()
            self.get_logger().info("Successfully connected to REAL CAN (can0)")
        except Exception as e:
            self.get_logger().error(f"Failed to connect can0: {e}. Is PEAK-USB connected?")
            return

        # 6. 하드웨어 초기화 (NMT Start + Enable) - 실제 구동할 3개 포트만 활성화
        self.init_hardware()

        # 50Hz (20ms) 주기로 실시간 제어 루프 실행
        self.timer = self.create_timer(0.02, self.control_loop)

    def init_hardware(self):
        """실제 모터 드라이버들을 켜고 구동 가능 상태(Enable)로 만듭니다."""
        self.get_logger().info("Initializing Welcon Hardware (Joint 1, 3, 5)...")
        # NMT Start
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)

        # 실제 매핑: joint1=(Node1,Axis1), joint3=(Node2,Axis1), joint5=(Node3,Axis1)
        self.real_axes = [(1,1), (2,1), (3,1)]
        
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

    def control_loop(self):
        """1:1 매칭 제어 루프"""
        self.real_axes = [(1,1), (2,1), (3,1)]
        
        for i, (node_id, axis) in enumerate(self.real_axes):
            # 오차 계산
            error = self.target_positions[i] - self.current_positions[i]
            
            # 1. PD 제어 값 계산
            pd_voltage = (self.Kp * error) - (self.Kd * self.current_velocities[i])
            
            # 2. 정지 마찰력 보상 적용
            if abs(error) > 0.03:
                if error > 0:
                    pd_voltage += self.stiction_offset
                else:
                    pd_voltage -= self.stiction_offset
            
            # 3. 최대 안전 전압 제한
            clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, pd_voltage))
            
            if abs(error) < 0.015:
                clamped_voltage = 0.0
            
            # 4. SDO 전압 전송
            volt_frame = self.protocol.make_q_axis_voltage_mv_sdo(
                node_id=node_id, 
                voltage_mv=int(clamped_voltage), 
                axis=axis
            )
            self.bus.send(volt_frame)

            # 5. 시뮬레이션 각도 동기화용 연산
            self.current_velocities[i] += clamped_voltage * 0.00004 * 0.02
            self.current_velocities[i] *= 0.8  
            self.current_positions[i] += self.current_velocities[i] * 0.02

        # 6. 현재 각도를 RViz2에 발행
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down... Stopping Joint 1, 3, 5.")
        self.real_axes = [(1,1), (2,1), (3,1)]
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
