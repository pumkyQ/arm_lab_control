#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import select
import termios
import tty
import os
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class KitechMultiJointController(Node):
    def __init__(self):
        super().__init__('kitech_multi_joint_controller_node')
        
        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 물리 상수 및 조인트별 측정 데이터 매핑]
        # ----------------------------------------------------------------------
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO                                         
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi) 
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN            
        self.voltage_limit = 9500.0      # 최대 인가 전압 한계 (mV)
        
        # 조인트별 하드웨어 정보 매핑 데이터 테이블 (j1:3-1, j2:3-2, j3:1-1, j4:1-2)
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -1260.0, 'FLEX': -1030.0, 'EXT': -1260.0},
            'j2': {'NODE_ID': 3, 'AXIS': 2, 'ALIGN': -1990.0, 'FLEX': -1040.0, 'EXT': -2055.0},
            'j3': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': -1706.0, 'FLEX':  -854.0, 'EXT': -1769.0},
            'j4': {'NODE_ID': 1, 'AXIS': 2, 'ALIGN':   574.0, 'FLEX':  1626.0, 'EXT':  -422.0}
        }
        
        # 제어 게인 및 불감대 세팅 (0.5도 요청 반영)
        self.Kp = 350.0
        self.Kd = 15.0         
        self.Ki = 0.5          
        self.Ki_limit = 500.0  
        self.DEADZONE_DEG = 3.0
        self.DEADZONE_THRESH_COUNT = self.DEADZONE_DEG * self.PULSES_PER_DEGREE

        self.LOOP_RATE = 50.0            
        self.dt = 1.0 / self.LOOP_RATE
        
        # ----------------------------------------------------------------------
        # [🔄 실시간 상태 변수 초기화]
        # ----------------------------------------------------------------------
        self.active_mode = 'j1'  
        self.input_buffer = ""
        self.last_loop_time = time.monotonic()
        self.cycle_time_ms = 0.0
        self.init_retry_counter = 0
        self.received_first_feedback = False

        # 각 조인트 상태 트래킹용 딕셔너리
        self.joint_states = {}
        for j_key in ['j1', 'j2', 'j3', 'j4']:
            self.joint_states[j_key] = {
                'target_count': self.JOINT_CONFIG[j_key]['ALIGN'], 
                'current_count': self.JOINT_CONFIG[j_key]['ALIGN'],
                'velocity_raw': 0.0,
                'status_word': 0,
                'error_integral': 0.0
            }
        
        # 터미널 상태 백업
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ----------------------------------------------------------------------
        # [📡 ROS 2 Publisher & Subscriber 정의]
        # ----------------------------------------------------------------------
        # 상태 로깅용 토픽
        self.pub_status = self.create_publisher(Float64MultiArray, '/multi_joint_status', 10)
        # 드라이버 전송용 전압 제어 토픽
        self.pub_voltage = self.create_publisher(Float64MultiArray, '/joint_voltage_cmd', 10)
        
        # 라디안 제어 명령 구독 토픽
        self.sub_j1 = self.create_subscription(Float64, '/j1_target_rad', lambda msg: self.ros_callback(msg, 'j1'), 10)
        self.sub_j2 = self.create_subscription(Float64, '/j2_target_rad', lambda msg: self.ros_callback(msg, 'j2'), 10)
        self.sub_j3 = self.create_subscription(Float64, '/j3_target_rad', lambda msg: self.ros_callback(msg, 'j3'), 10)
        self.sub_j4 = self.create_subscription(Float64, '/j4_target_rad', lambda msg: self.ros_callback(msg, 'j4'), 10)

        # 드라이버 피드백 구독 토픽
        self.sub_feedback = self.create_subscription(
            JointState, 
            '/joint_states_raw', 
            self.feedback_callback, 
            10
        )

        # 피드백 수신 즉시 제어 연산을 실행하므로 타이머는 생성하지 않습니다.

    def feedback_callback(self, msg: JointState):
        """드라이버 노드로부터 실시간 엔코더 및 상태 정보 업데이트"""
        is_first = not self.received_first_feedback
        for i, name in enumerate(msg.name):
            if name in self.joint_states:
                self.joint_states[name]['current_count'] = msg.position[i]
                self.joint_states[name]['velocity_raw'] = msg.velocity[i]
                self.joint_states[name]['status_word'] = int(msg.effort[i])
                
                # 처음 피드백을 수신했을 때, 목표 위치를 현재 실제 위치로 동기화하여 갑작스러운 기동(Jerk) 방지
                if is_first:
                    self.joint_states[name]['target_count'] = msg.position[i]
                    
        self.received_first_feedback = True
        
        # 피드백을 수신할 때마다 제어 루프를 즉시 동기 실행하여 통신 지연 최소화
        self.control_loop()

    def ros_callback(self, msg, joint_key):
        """ROS 라디안 토픽 각도 제어 변환"""
        target_degree = msg.data * (180.0 / np.pi)
        cfg = self.JOINT_CONFIG[joint_key]
        requested_count = cfg['ALIGN'] + (target_degree * self.PULSES_PER_DEGREE)
        self.update_target_with_limits(joint_key, requested_count)

    def update_target_with_limits(self, joint_key, requested_count):
        """소프트웨어 가드 한계 예외 처리"""
        cfg = self.JOINT_CONFIG[joint_key]
        min_lim = min(cfg['FLEX'], cfg['EXT'])
        max_lim = max(cfg['FLEX'], cfg['EXT'])
        
        if requested_count < min_lim: clamped = min_lim
        elif requested_count > max_lim: clamped = max_lim
        else: clamped = requested_count
        
        self.joint_states[joint_key]['target_count'] = float(clamped)

    def control_loop(self):
        # 1. 하드웨어 드라이버로부터 첫 피드백이 올 때까지 제어 정지 대기
        if not self.received_first_feedback:
            if self.init_retry_counter % 50 == 0:
                self.get_logger().info("⏳ Waiting for hardware driver feedback (/joint_states_raw)...")
            self.init_retry_counter += 1
            return

        self.cycle_time_ms = (time.monotonic() - self.last_loop_time) * 1000.0
        self.last_loop_time = time.monotonic()

        # ⌨️ 터미널 키보드 입력 인터페이스 핸들러
        while kbhit():
            try:
                char = os.read(sys.stdin.fileno(), 1).decode()
                if char in ['\r', '\n']:
                    user_input = self.input_buffer.strip().lower()
                    self.input_buffer = ""
                    
                    if user_input in ['j1', 'j2', 'j3', 'j4']:
                        self.active_mode = user_input
                        self.get_logger().info(f"🔄 제어 모드가 변경되었습니다 ➡️ [{self.active_mode.upper()} 모드]")
                    elif user_input:
                        try:
                            target_degree = float(user_input)
                            cfg = self.JOINT_CONFIG[self.active_mode]
                            requested_count = cfg['ALIGN'] + (target_degree * self.PULSES_PER_DEGREE)
                            self.update_target_with_limits(self.active_mode, requested_count)
                        except ValueError: pass
                    break
                elif char in ['\x08', '\x7f']:
                    if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                else: self.input_buffer += char
            except: pass

        # 2. 4개 관절 독립 병렬 PI-D + Feedforward 연산 진행 및 명령 발행
        ros_log_data = []
        voltage_cmd_data = []
        
        for j_key in ['j1', 'j2', 'j3', 'j4']:
            cfg = self.JOINT_CONFIG[j_key]
            state = self.joint_states[j_key]
            
            # 드라이버 레벨에서 FAULT 복구 시퀀스를 돌리므로, FAULT 상태 해제까지 대기 (전압 0mV 인가)
            if (state['status_word'] & 0x08):
                voltage_cmd_data.append(0.0)
                ros_log_data.extend([0.0, 0.0])
                continue

            error_count = state['target_count'] - state['current_count']
            
            # 1) 미분 성분 (Derivative on Feedback)
            v_d = -self.Kd * state['velocity_raw']
            
            # 2) 1.0도 정밀 불감대 제어 및 전압 즉시 차단 (진동 방지)
            if abs(error_count) <= self.DEADZONE_THRESH_COUNT:
                state['error_integral'] = 0.0
                v_p = 0.0
                v_i = 0.0
                v_d = 0.0
                v_emf = 0.0
                total_voltage = 0.0
            else:
                v_p = self.Kp * error_count
                state['error_integral'] += error_count * self.dt
                # Anti-Windup 클리핑 기법 적용 (Ki=0일 때 분모가 0이 되는 것을 방지)
                if self.Ki != 0.0:
                    state['error_integral'] = max(-self.Ki_limit/self.Ki, min(self.Ki_limit/self.Ki, state['error_integral']))
                else:
                    state['error_integral'] = 0.0  # Ki=0일 때는 적분 자체를 누적하지 않음
                
                v_i = self.Ki * state['error_integral']
                # 3) Back-EMF Feedforward 전방 보상
                v_emf = self.K_emf_count * state['velocity_raw']
                # 4) 전압 합성 및 물리 가드 한계 안전 조치
                total_voltage = v_p + v_i + v_d + v_emf
            
            min_lim = min(cfg['FLEX'], cfg['EXT'])
            max_lim = max(cfg['FLEX'], cfg['EXT'])
            if (state['current_count'] >= max_lim and total_voltage > 0) or \
               (state['current_count'] <= min_lim and total_voltage < 0):
                total_voltage = 0.0
                state['error_integral'] = 0.0

            clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
            voltage_cmd_data.append(float(clamped_voltage))

            real_deg = (state['current_count'] - cfg['ALIGN']) / self.PULSES_PER_DEGREE
            ros_log_data.extend([real_deg, clamped_voltage])

        # 3. 전압 제어 명령 토픽 발행
        volt_msg = Float64MultiArray()
        volt_msg.data = voltage_cmd_data
        self.pub_voltage.publish(volt_msg)

        # 4. ROS 상태 로깅 토픽 발행
        status_msg = Float64MultiArray()
        status_msg.data = ros_log_data
        self.pub_status.publish(status_msg)

        # 5. 인터페이스 리프레시 출력
        sys.stdout.write(
            f"\r 🟢 [현재모드: {self.active_mode.upper()}] | "
            f"J1_deg: {(self.joint_states['j1']['current_count']-self.JOINT_CONFIG['j1']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"J2_deg: {(self.joint_states['j2']['current_count']-self.JOINT_CONFIG['j2']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"J3_deg: {(self.joint_states['j3']['current_count']-self.JOINT_CONFIG['j3']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"J4_deg: {(self.joint_states['j4']['current_count']-self.JOINT_CONFIG['j4']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"입력창 ➡️ {self.input_buffer}"
        )
        sys.stdout.flush()

    def shutdown_hook(self):
        self.get_logger().info("🛑 Shutting down controller... Sending stop command to driver.")
        try:
            # 안전을 위해 모든 조인트 전압 0mV로 정지 명령 송신
            volt_msg = Float64MultiArray()
            volt_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub_voltage.publish(volt_msg)
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KitechMultiJointController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.shutdown_hook(); rclpy.shutdown()

if __name__ == '__main__':
    main()