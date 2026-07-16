#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys
import os
import time

# kitech_v1 패키지 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.dirname(current_dir)
kitech_path = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path:
    sys.path.append(kitech_path)

from motor_control.cia402 import Cia402Protocol, Cia402Controlword, Cia402Object
from motor_control.can_bus import SocketCanBus

class RealWelconDriver(Node):
    def __init__(self):
        super().__init__('real_welcon_driver')
        
        # 1. ROS 2 통신 설정
        # 1.1 Command Subscriber: 컨트롤러의 전압 제어 명령 수집
        self.volt_sub = self.create_subscription(
            Float64MultiArray, 
            '/joint_voltage_cmd', 
            self.voltage_callback, 
            10
        )
        # 1.2 Feedback Publisher: 실시간 엔코더 및 상태 정보 발행
        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_states_raw', 
            10
        )
        
        # 2. 제어 대상 조인트 및 축 매핑 정보 정의 (j1~j4)
        self.joint_names = ['j1', 'j2', 'j3', 'j4']
        self.real_axes = [(3, 1), (2, 1), (1, 1), (3, 2)]
        
        # 실시간 하드웨어 피드백 상태 보관 변수
        self.current_positions = [0.0] * 4
        self.current_velocities = [0.0] * 4
        self.current_status_words = [0] * 4
        self.applied_voltages = [0.0] * 4
        
        # 각 조인트별 SDO 주소 매핑 테이블 구성
        self.sdo_map = {}
        for idx, (node_id, axis) in enumerate(self.real_axes):
            self.sdo_map[(node_id, axis)] = {
                'pos': Cia402Object(0x6064 if axis == 1 else 0x6864),
                'vel': Cia402Object(0x606c if axis == 1 else 0x686c),
                'status': Cia402Object(0x6041 if axis == 1 else 0x6841),
                'idx': idx
            }

        # 3. Welcon 드라이버 프로토콜 및 CAN 버스 초기화
        self.protocol = Cia402Protocol()
        
        try:
            # 50Hz 제어 루프를 고려하여 비차단식 수신(receive_timeout=0.0) 설정
            self.bus = SocketCanBus(channel='can0', receive_timeout=0.0)
            self.bus.open()
            self.get_logger().info("✅ Successfully connected to REAL CAN (can0)")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect can0: {e}. Is PEAK-USB connected?")
            self.bus = None
            return

        # 4. 하드웨어 초기화 (NMT Start + Enable 시퀀스 진행)
        self.init_hardware()

        # 5. 50Hz (20ms) 주기로 피드백 수집 및 ROS 토픽 발행 루프 실행
        self.log_counter = 0
        self.timer = self.create_timer(0.02, self.feedback_loop)

    def init_hardware(self):
        """실제 모터 드라이버들을 전압 모드로 가동하고 Operation Enable 시퀀스를 순차 수행"""
        if self.bus is None:
            return
            
        self.get_logger().info("▶ Initializing Welcon Hardware (Joint 1, 2, 3, 4)...")
        # NMT Start (전체 노드 일괄 시작)
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)
        
        for node_id, axis in self.real_axes:
            # SDO 모드 쓰기: -11 (Voltage Control Mode)
            mode_frame = self.protocol.make_axis_mode_sdo(node_id, axis, -11)
            self.bus.send(mode_frame)
            time.sleep(0.02)
            
            # State Machine 기동 상태 단계를 순차 입력
            for label, ctrl in (
                ("fault reset", Cia402Controlword.FAULT_RESET),
                ("shutdown", Cia402Controlword.SHUTDOWN),
                ("switch on", Cia402Controlword.SWITCH_ON),
                ("enable operation", Cia402Controlword.ENABLE_OPERATION)
            ):
                frame = self.protocol.make_axis_controlword_sdo(node_id, axis, ctrl)
                self.bus.send(frame)
                time.sleep(0.02)
                
        self.get_logger().info("🔥 Joint 1, 2, 3, 4 Active axes enabled in Voltage Mode!")

    def voltage_callback(self, msg: Float64MultiArray):
        """컨트롤러 노드로부터 받은 전압 제어 신호를 모터 드라이버 SDO 명령으로 즉시 송신"""
        if self.bus is None:
            return
            
        if len(msg.data) >= 4:
            for idx, (node_id, axis) in enumerate(self.real_axes):
                voltage = int(msg.data[idx])
                # 전압 절대 한계 안전 클리핑 (최대 9500mV)
                voltage = max(-9500, min(9500, voltage))
                self.applied_voltages[idx] = float(voltage)
                
                volt_frame = self.protocol.make_q_axis_voltage_mv_sdo(node_id, voltage, axis)
                self.bus.send(volt_frame)

    def feedback_loop(self):
        """50Hz 주기로 CAN 버스로부터 현재 위치/속도/상태를 폴링하여 ROS 2 토픽으로 전파"""
        if self.bus is None:
            return

        # 1. 모든 노드의 SDO 읽기 요청 송신
        for (node_id, axis), maps in self.sdo_map.items():
            self.bus.send(self.protocol.make_sdo_read(node_id, maps['pos']))
            self.bus.send(self.protocol.make_sdo_read(node_id, maps['vel']))
            self.bus.send(self.protocol.make_sdo_read(node_id, maps['status']))

        # 2. CAN 버스 버퍼 폴링 (최대 8ms 동안 수신 응답 분석)
        timeout_end = time.monotonic() + 0.008
        while time.monotonic() < timeout_end:
            try:
                frame = self.bus.recv(timeout=0.001)
                if frame is None:
                    continue
                node_id = frame.can_id - 0x580
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res is not None and sdo_res.value is not None:
                    for (n_id, axis), maps in self.sdo_map.items():
                        if n_id == node_id:
                            pos_idx = 0x6064 if axis == 1 else 0x6864
                            vel_idx = 0x606c if axis == 1 else 0x686c
                            status_idx = 0x6041 if axis == 1 else 0x6841
                            idx = maps['idx']
                            
                            val = sdo_res.value
                            if val > 0x7FFFFFFF:
                                val -= 0x100000000
                                
                            if sdo_res.index == pos_idx:
                                self.current_positions[idx] = float(val)
                            elif sdo_res.index == vel_idx:
                                self.current_velocities[idx] = float(val)
                            elif sdo_res.index == status_idx:
                                self.current_status_words[idx] = val
                                # 드라이버 레벨 자동 알람 해제 (FAULT 상태인 경우 자동 RESET 명령 송출)
                                if val & 0x08:
                                    self.get_logger().warn(
                                        f"⚠️ Joint {idx+1} (Node {node_id}, Axis {axis}) FAULT detected! "
                                        f"Status: {hex(val)}. Sending FAULT_RESET..."
                                    )
                                    reset_frame = self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.FAULT_RESET)
                                    self.bus.send(reset_frame)
            except Exception:
                pass

        # 3. ROS 2 JointState 퍼블리시
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = self.current_velocities
        msg.effort = [float(stat) for stat in self.current_status_words]
        self.joint_pub.publish(msg)

        # 4. 실시간 상태 터미널 출력 (5Hz 주기)
        self.log_counter += 1
        if self.log_counter % 10 == 0:
            pos_str = ", ".join([f"{name}: {pos:8.1f}" for name, pos in zip(self.joint_names, self.current_positions)])
            volt_str = ", ".join([f"{name}: {volt:5.0f}mV" for name, volt in zip(self.joint_names, self.applied_voltages)])
            self.get_logger().info(f"[Driver] Raw Pos: [{pos_str}]")
            self.get_logger().info(f"[Driver] Volts:   [{volt_str}]")
            self.log_counter = 0

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down... Stopping all joints safely.")
        if self.bus is not None:
            for node_id, axis in self.real_axes:
                try:
                    # 안전을 위해 모든 전압 0으로 해제 및 드라이버 비활성화
                    self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
                    self.bus.send(self.protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.DISABLE_OPERATION))
                except Exception:
                    pass
            self.bus.close()
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