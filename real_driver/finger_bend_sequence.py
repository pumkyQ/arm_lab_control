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

class FingerBendSequenceController(Node):
    def __init__(self):
        super().__init__('finger_bend_sequence_controller_node')
        
        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 물리 상수 및 조인트별 측정 데이터 매핑]
        # ----------------------------------------------------------------------
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO                                         
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi) 
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN            
        self.voltage_limit = 9500.0      # 최대 인가 전압 한계 (mV)
        
        # 조인트별 하드웨어 정보 매핑 데이터 테이블 (Node 1, 2, 3, 4)
        # j2의 EXT는 사용자가 업데이트한 -1278.0 값을 반영합니다.
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -214.0, 'FLEX': 806.0, 'EXT': -1242.0},
            'j2': {'NODE_ID': 2, 'AXIS': 1, 'ALIGN': -1008.0, 'FLEX': -655.0, 'EXT': -1335.0}, 
            'j3': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': 596.0, 'FLEX': 1651.0,  'EXT': -397.0},
            'j4': {'NODE_ID': 3, 'AXIS': 2, 'ALIGN': -388.0, 'FLEX': 664.0, 'EXT': -1384.0} 
        }
        
        # 제어 게인 및 불감대 세팅 (조인트별 개별 설정 적용)
        self.GAIN_CONFIG = {
            'j1': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 800.0},
            'j2': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0},
            'j3': {'Kp': 450.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 800.0, 'DEADZONE_DEG': 1.5, 'FRICT_COMP': 1200.0}, # 뻑뻑한 조인트3 전용 마찰 보상 세팅
            'j4': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0}
        }

        self.LOOP_RATE = 50.0            
        self.dt = 1.0 / self.LOOP_RATE
        self.LPF_ALPHA = 0.25            # test_3node.py 필터 계수 적용 (25% 주입)
        
        # ----------------------------------------------------------------------
        # [🔄 실시간 상태 및 시퀀스 변수 초기화]
        # ----------------------------------------------------------------------
        self.current_state = "STANDBY"
        self.state_start_time = time.monotonic()
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
                'filtered_velocity_old': 0.0, # LPF 필터 상태 백업용
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

        # 드라이버 피드백 구독 토픽
        self.sub_feedback = self.create_subscription(
            JointState, 
            '/joint_states_raw', 
            self.feedback_callback, 
            10
        )

        print("\n" + "=" * 60)
        print(" 🎯 KITECH 손가락 관절 순차 구동기 (Sequence Controller)")
        print(" ▶ j2, j3, j4 정렬 후 각각 20도, 40도, 35도 순차 기동")
        print(" ▶ 's' 키: 시퀀스 시작 | 'r' 키: 리셋 | 'q' 키: 안전 종료")
        print("=" * 60 + "\n")

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

    def update_target_degree(self, joint_key, degree):
        """관절 각도(도) 기반 목표 카운트 업데이트 및 소프트웨어 제한(Clamping) 적용"""
        cfg = self.JOINT_CONFIG[joint_key]
        requested_count = cfg['ALIGN'] + (degree * self.PULSES_PER_DEGREE)
        
        min_lim = min(cfg['FLEX'], cfg['EXT'])
        max_lim = max(cfg['FLEX'], cfg['EXT'])
        
        clamped = max(min_lim, min(max_lim, requested_count))
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
                    
                    if user_input == 's':
                        if self.current_state in ["STANDBY", "HOLD"]:
                            self.current_state = "ALIGN"
                            self.state_start_time = time.monotonic()
                            self.get_logger().info("🔥 [시퀀스 시작] j2, j3, j4 정렬을 시작합니다 (0°로 정렬)")
                    elif user_input == 'r':
                        self.current_state = "STANDBY"
                        for name in ['j1', 'j2', 'j3', 'j4']:
                            self.joint_states[name]['target_count'] = self.joint_states[name]['current_count']
                            self.joint_states[name]['error_integral'] = 0.0
                        self.get_logger().info("🔄 [리셋] 대기 상태로 복귀 및 목표치를 현재 위치로 동기화했습니다.")
                    elif user_input == 'q':
                        self.get_logger().info("🛑 [종료] 안전 종료 시퀀스를 실행합니다.")
                        raise KeyboardInterrupt
                    break
                elif char in ['\x08', '\x7f']:
                    if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                else: self.input_buffer += char
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except: pass

        # ----------------------------------------------------------------------
        # [🔄 시퀀스 제어 상태기기 (State Machine)]
        # ----------------------------------------------------------------------
        elapsed_time = time.monotonic() - self.state_start_time

        if self.current_state == "ALIGN":
            # j1(Joint 1)은 0도 고정, j2, j3, j4 모두 0도 정렬 (ALIGN 위치)
            self.update_target_degree('j1', 0.0)
            self.update_target_degree('j2', 0.0)
            self.update_target_degree('j3', 0.0)
            self.update_target_degree('j4', 0.0)
            if elapsed_time > 3.0:
                self.current_state = "MOVE_J2"
                self.state_start_time = time.monotonic()
                self.get_logger().info("➡️ [1단계] 조인트 2 구동 시작 (0° ➡️ 20°)")

        elif self.current_state == "MOVE_J2":
            self.update_target_degree('j1', 0.0)
            self.update_target_degree('j2', 20.0)
            self.update_target_degree('j3', 0.0)
            self.update_target_degree('j4', 0.0)
            if elapsed_time > 0.3:
                self.current_state = "MOVE_J3"
                self.state_start_time = time.monotonic()
                self.get_logger().info("➡️ [2단계] 조인트 3 구동 시작 (0° ➡️ 40°)")

        elif self.current_state == "MOVE_J3":
            self.update_target_degree('j1', 0.0)
            self.update_target_degree('j2', 20.0)
            self.update_target_degree('j3', 40.0)
            self.update_target_degree('j4', 0.0)
            if elapsed_time > 0.3:
                self.current_state = "MOVE_J4"
                self.state_start_time = time.monotonic()
                self.get_logger().info("➡️ [3단계] 조인트 4 구동 시작 (0° ➡️ 35°)")

        elif self.current_state == "MOVE_J4":
            self.update_target_degree('j1', 0.0)
            self.update_target_degree('j2', 20.0)
            self.update_target_degree('j3', 40.0)
            self.update_target_degree('j4', 35.0)
            if elapsed_time > 0.3:
                self.current_state = "HOLD"
                self.get_logger().info("✅ [시퀀스 완료] j2=20°, j3=40°, j4=35° 파지 포즈 수렴 완료.")

        elif self.current_state == "HOLD":
            self.update_target_degree('j1', 0.0)
            self.update_target_degree('j2', 20.0)
            self.update_target_degree('j3', 40.0)
            self.update_target_degree('j4', 35.0)

        # ----------------------------------------------------------------------
        # 🎯 4개 관절 독립 병렬 PI-D + Feedforward 연산 진행 및 명령 발행
        # ----------------------------------------------------------------------
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
            
            # 조인트별 개별 게인 및 마찰보상 매핑 적용
            gc = self.GAIN_CONFIG[j_key]
            kp = gc['Kp']
            kd = gc['Kd']
            ki = gc['Ki']
            ki_limit = gc['Ki_limit']
            deadzone_thresh = gc['DEADZONE_DEG'] * self.PULSES_PER_DEGREE
            frict_comp = gc['FRICT_COMP']
            
            # LPF (저역통과필터) 연산 수행 (test_3node.py 방식)
            filtered_velocity = (self.LPF_ALPHA * state['velocity_raw']) + ((1.0 - self.LPF_ALPHA) * state['filtered_velocity_old'])
            state['filtered_velocity_old'] = filtered_velocity
            
            # 1) 미분 성분 (Derivative on Feedback) 및 EMF 보상 (필터링된 속도 사용)
            v_d = -kd * filtered_velocity
            v_emf = self.K_emf_count * filtered_velocity
            
            # 2) 정밀 불감대 제어 및 백래시 방지용 Tail 제어
            if abs(error_count) <= deadzone_thresh:
                state['error_integral'] = 0.0
                active_direction = np.sign(filtered_velocity if filtered_velocity != 0.0 else error_count)
                if active_direction != 0:
                    v_stiction_tail = (frict_comp * 0.8) * active_direction
                else:
                    v_stiction_tail = 0.0
                total_voltage = v_d + v_emf + v_stiction_tail
            else:
                v_p = kp * error_count
                state['error_integral'] += error_count * self.dt
                # Anti-Windup 클리핑 기법 적용
                if ki != 0.0:
                    state['error_integral'] = max(-ki_limit/ki, min(ki_limit/ki, state['error_integral']))
                else:
                    state['error_integral'] = 0.0
                
                v_i = ki * state['error_integral']
                # 3) 정마찰 보상 (Friction Compensation Offset)
                v_frict = np.sign(error_count) * frict_comp
                # 4) 전압 합성 및 물리 가드 한계 안전 조치
                total_voltage = v_p + v_i + v_d + v_emf + v_frict
            
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

        # 5. 인터페이스 리프레시 출력 (실시간 절대엔코더 값 / 정렬 기준값)
        sys.stdout.write(
            f"\r ⚙️ [상태: {self.current_state:8s}] | "
            f"J1: {int(self.joint_states['j1']['current_count']):5d}/{int(self.JOINT_CONFIG['j1']['ALIGN']):5d} | "
            f"J2: {int(self.joint_states['j2']['current_count']):5d}/{int(self.JOINT_CONFIG['j2']['ALIGN']):5d} | "
            f"J3: {int(self.joint_states['j3']['current_count']):5d}/{int(self.JOINT_CONFIG['j3']['ALIGN']):5d} | "
            f"J4: {int(self.joint_states['j4']['current_count']):5d}/{int(self.JOINT_CONFIG['j4']['ALIGN']):5d} | "
            f"입력: {self.input_buffer}"
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
    node = FingerBendSequenceController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.shutdown_hook(); rclpy.shutdown()

if __name__ == '__main__':
    main()
