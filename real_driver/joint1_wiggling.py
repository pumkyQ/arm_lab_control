#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import os
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

class KitechJointAutoTest(Node):
    def __init__(self):
        super().__init__('kitech_joint_auto_test_node')
        
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
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -218.0, 'FLEX': 806.0, 'EXT': -1242.0},
            'j2': {'NODE_ID': 2, 'AXIS': 1, 'ALIGN': -1538.0,  'FLEX': -514.0, 'EXT': 2562.0}, 
            'j3': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': 687.0, 'FLEX': 1711.0,  'EXT': -337.0},
            'j4': {'NODE_ID': 3, 'AXIS': 2, 'ALIGN': -360.0, 'FLEX': 664.0, 'EXT': -1384.0} 
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
        # [🔄 실시간 상태 및 시퀀스 변수 초기화]
        # ----------------------------------------------------------------------
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
            
        # 자동 테스트 시퀀스 상태 변수
        self.test_state = "WAIT_DRIVER"  # WAIT_DRIVER ➡️ GO_TO_ALIGN ➡️ GO_TO_PLUS_20 ➡️ GO_TO_MINUS_20
        self.state_start_time = time.monotonic()
        self.settle_timeout = 3.5  # 목표 위치 도달 대기 타임아웃 (초)

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

        self.get_logger().info("🤖 Joint 1 Auto Test Node Initialized.")

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
                    
        if is_first:
            self.received_first_feedback = True
            self.test_state = "GO_TO_ALIGN"
            self.state_start_time = time.monotonic()
            self.get_logger().info("🔥 Driver connected. Sequence started: Moving to ALIGN pose...")
        
        # 피드백을 수신할 때마다 제어 루프를 즉시 동기 실행하여 통신 지연 최소화
        self.control_loop()

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
        if not self.received_first_feedback:
            return

        current_time = time.monotonic()
        elapsed_time = current_time - self.state_start_time

        # 1. 자동 테스트 시퀀스 상태 기계 (State Machine) 관리
        j1_state = self.joint_states['j1']
        j1_cfg = self.JOINT_CONFIG['j1']
        j1_error_count = j1_state['target_count'] - j1_state['current_count']
        j1_error_deg = abs(j1_error_count) / self.PULSES_PER_DEGREE

        if self.test_state == "GO_TO_ALIGN":
            # Joint 1 목표 각도를 ALIGN으로 설정
            self.update_target_with_limits('j1', j1_cfg['ALIGN'])
            
            # 오차가 1.0도 이내로 진입했거나 대기 시간이 3.5초를 경과한 경우 다음 상태로 전환
            if j1_error_deg <= 1.0 or elapsed_time >= self.settle_timeout:
                self.test_state = "GO_TO_PLUS_20"
                self.state_start_time = current_time
                self.get_logger().info("➡️ Arrived at ALIGN. Moving J1 to +20 degrees...")
                
        elif self.test_state == "GO_TO_PLUS_20":
            # Joint 1 목표 각도를 ALIGN + 20도로 설정
            target_pos = j1_cfg['ALIGN'] + (20.0 * self.PULSES_PER_DEGREE)
            self.update_target_with_limits('j1', target_pos)
            
            # 오차가 1.0도 이내로 진입했거나 대기 시간이 3.5초를 경과한 경우 다음 상태로 전환
            if j1_error_deg <= 1.0 or elapsed_time >= self.settle_timeout:
                self.test_state = "GO_TO_MINUS_20"
                self.state_start_time = current_time
                self.get_logger().info("➡️ Arrived at +20 deg. Moving J1 to -20 degrees...")
                
        elif self.test_state == "GO_TO_MINUS_20":
            # Joint 1 목표 각도를 ALIGN - 20도로 설정
            target_pos = j1_cfg['ALIGN'] - (20.0 * self.PULSES_PER_DEGREE)
            self.update_target_with_limits('j1', target_pos)
            
            # 오차가 1.0도 이내로 진입했거나 대기 시간이 3.5초를 경과한 경우 다음 상태로 전환
            if j1_error_deg <= 1.0 or elapsed_time >= self.settle_timeout:
                self.test_state = "GO_TO_PLUS_20"
                self.state_start_time = current_time
                self.get_logger().info("➡️ Arrived at -20 deg. Moving J1 to +20 degrees...")

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
        j1_deg = (j1_state['current_count'] - j1_cfg['ALIGN']) / self.PULSES_PER_DEGREE
        j1_tgt_deg = (j1_state['target_count'] - j1_cfg['ALIGN']) / self.PULSES_PER_DEGREE
        sys.stdout.write(
            f"\r 🤖 [AUTO TEST] 상태: {self.test_state:<15} | "
            f"J1 목표: {j1_tgt_deg:+.1f}° | J1 현재: {j1_deg:+.1f}° | "
            f"오차: {j1_error_deg:5.2f}° | "
            f"경과: {elapsed_time:4.1f}s"
        )
        sys.stdout.flush()

    def shutdown_hook(self):
        self.get_logger().info("🛑 Shutting down auto test node... Sending stop command to driver.")
        try:
            volt_msg = Float64MultiArray()
            volt_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub_voltage.publish(volt_msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = KitechJointAutoTest()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.shutdown_hook(); rclpy.shutdown()

if __name__ == '__main__':
    main()
