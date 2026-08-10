#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🕹️ KITECH 손가락 관절 키보드 제어 전용 컨트롤러 (Ubuntu / ROS 2 전용)
========================================================================
- real_welcon_driver.py 노드가 CAN 통신을 전담하고, 본 노드는 키보드 제어만 전담합니다.
- ROS 2 피드백 토픽 (/joint_states_raw)으로 위치/속도를 수신하여 PI-D 연산을 수행하고,
- 제어 전압 명령을 ROS 2 토픽 (/joint_voltage_cmd)으로 전송합니다.

[⌨️ 키보드 조작 안내]
  - [스페이스바] 또는 's' / 's'+Enter : 조인트 2, 3, 4번 빠른 굽히기 (BEND_SEQ)
  - [ESC] 또는 'r' / 'r'+Enter       : 모든 관절 0° 원점 복귀 정렬 (ALIGN)
  - [↑ / ↓ 화살표]                 : Joint 1 관절 실시간 상하 선형 조종
  - [← / → 화살표]                 : Joint 2, 3, 4번 굽힘(←) / 펴짐(→) 실시간 선형 조종
  - 'q' / 'Q'                        : 전압 0mV 즉시 차단 및 안전 종료
"""

import sys
import os
import time
import select
import termios
import tty
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# kitech_v1 패키지 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))   # .../haptic
real_driver_dir = os.path.dirname(current_dir)             # .../real_driver
workspace_dir = os.path.dirname(real_driver_dir)           # .../arm_lab_control
kitech_path = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path:
    sys.path.append(kitech_path)
if workspace_dir not in sys.path:
    sys.path.append(workspace_dir)


class KeyInputReader:
    """우분투 터미널에서 화살표 키, ESC, Space, s, r, q 등을 논블로킹으로 감지하는 클래스"""

    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def get_key(self, timeout=0.0):
        """키 입력 파싱 (화살표, ESC, 스페이스바, 알파벳 구별)"""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None

        ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')

        # ESC 키 또는 이스케이프 시퀀스 파싱
        if ch == '\x1b':
            rlist_seq, _, _ = select.select([sys.stdin], [], [], 0.02)
            if not rlist_seq:
                return 'ESC'
            
            seq1 = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
            if seq1 == '[':
                rlist_seq2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if rlist_seq2:
                    seq2 = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
                    if seq2 == 'A':
                        return 'UP'
                    elif seq2 == 'B':
                        return 'DOWN'
                    elif seq2 == 'C':
                        return 'RIGHT'
                    elif seq2 == 'D':
                        return 'LEFT'
            return 'ESC'

        elif ch == ' ':
            return 'SPACE'
        elif ch in ['\r', '\n']:
            return 'ENTER'
        else:
            return ch.lower()

    def close(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


class WelconKeyboardDriver(Node):
    def __init__(self):
        super().__init__('welcon_keyboard_controller')

        # ----------------------------------------------------------------------
        # [⚙️ ROS 2 퍼블리셔 & 서브스크라이버]
        # QoS depth=1 설정으로 버퍼링으로 인한 시차/지연을 완전 제거
        # ----------------------------------------------------------------------
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.volt_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_voltage_cmd', 
            qos_profile
        )
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states_raw', 
            self.joint_state_callback, 
            qos_profile
        )

        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 물리 상수]
        # ----------------------------------------------------------------------
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi)
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN
        self.voltage_limit = 10000.0  # real_welcon_driver.py와 10V(10000mV) 한계 동기화

        # ----------------------------------------------------------------------
        # [⚙️ 4개 조인트 구성 (최신 calibration 값 반영)]
        # 배선: j1=노드3-1, j2=노드3-2, j3=노드1-1, j4=노드1-2
        # ----------------------------------------------------------------------
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -1260.0, 'FLEX': -1030.0, 'EXT': -1260.0},
            'j2': {'NODE_ID': 3, 'AXIS': 2, 'ALIGN': -1990.0, 'FLEX': -1040.0, 'EXT': -2055.0},
            'j3': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': -1706.0, 'FLEX':  -854.0, 'EXT': -1769.0},
            'j4': {'NODE_ID': 1, 'AXIS': 2, 'ALIGN':   574.0, 'FLEX':  1626.0, 'EXT':  -422.0}
        }

        # PID 게인 및 마찰 보상 파라미터 (joy+switch.ino 아두이노 게인값 100% 반영)
        self.GAIN_CONFIG = {
            'j1': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 300.0,  'DEADZONE_DEG': 2.0, 'FRICT_COMP': 1000.0},
            'j2': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 1000.0, 'DEADZONE_DEG': 6.0, 'FRICT_COMP': 1000.0},
            'j3': {'Kp': 450.0, 'Kd': 15.0, 'Ki': 3.0, 'Ki_limit': 1500.0, 'DEADZONE_DEG': 6.0, 'FRICT_COMP': 1200.0},
            'j4': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 1000.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0}
        }

        self.LOOP_RATE = 50.0  # 50Hz (20ms)
        self.dt = 1.0 / self.LOOP_RATE
        self.LPF_ALPHA = 0.25

        # ----------------------------------------------------------------------
        # [🔄 상태 및 비례 조종 비율 변수]
        # ----------------------------------------------------------------------
        self.current_state = "ALIGN_ONLY"  # 시작 시 자동 정렬 / STANDBY, ALIGN_ONLY, BEND_SEQ, HOLD
        self.state_start_time = time.monotonic()
        self.last_feedback_time = time.monotonic()
        
        self.joy_j1_ratio = 0.0      # 0.0 (0°) ~ 1.0 (20.21°)
        self.joy_flex_ratio = 0.0    # 0.0 (완전 펴짐) ~ 1.0 (최대 굽힘)
        self.filtered_j1_ratio = 0.0
        self.filtered_flex_ratio = 0.0

        # 문자열 엔터 입력 버퍼 (s + Enter, r + Enter 대응)
        self.input_buffer = ""

        # 관절 상태 트래킹
        self.joint_states = {}
        for j_key in ['j1', 'j2', 'j3', 'j4']:
            align_cnt = self.JOINT_CONFIG[j_key]['ALIGN']
            self.joint_states[j_key] = {
                'target_count': align_cnt,
                'current_count': align_cnt,
                'velocity_raw': 0.0,
                'filtered_velocity_old': 0.0,
                'status_word': 0,
                'error_integral': 0.0,
                'prev_error': 0.0
            }

    def joint_state_callback(self, msg: JointState):
        """real_welcon_driver 노드에서 발행하는 실시간 하드웨어 피드백 토픽 수신"""
        self.last_feedback_time = time.monotonic()
        for idx, name in enumerate(msg.name):
            if name in self.joint_states:
                if len(msg.position) > idx:
                    self.joint_states[name]['current_count'] = float(msg.position[idx])
                if len(msg.velocity) > idx:
                    self.joint_states[name]['velocity_raw'] = float(msg.velocity[idx])
                if len(msg.effort) > idx:
                    self.joint_states[name]['status_word'] = int(msg.effort[idx])

    def update_target_degree(self, joint_key, degree):
        """목표 각도를 카운트로 변환하여 Constrain Clamping 적용"""
        cfg = self.JOINT_CONFIG[joint_key]
        req_count = cfg['ALIGN'] + (degree * self.PULSES_PER_DEGREE)
        
        min_lim = min(cfg['FLEX'], cfg['EXT'])
        max_lim = max(cfg['FLEX'], cfg['EXT'])
        
        clamped = max(min_lim, min(max_lim, req_count))
        self.joint_states[joint_key]['target_count'] = float(clamped)

    def send_voltages(self, voltage_dict):
        """ROS 2 토픽(/joint_voltage_cmd)으로 전압 제어 명령 인가 (mV)"""
        msg = Float64MultiArray()
        msg.data = [
            float(voltage_dict.get('j1', 0.0)),
            float(voltage_dict.get('j2', 0.0)),
            float(voltage_dict.get('j3', 0.0)),
            float(voltage_dict.get('j4', 0.0))
        ]
        self.volt_pub.publish(msg)

    def run(self):
        key_reader = KeyInputReader()

        print("=" * 70)
        print(" 🎯 KITECH 손가락 관절 키보드 통합 컨트롤러 (Ubuntu ROS 2 Control Node)")
        print("=" * 70)
        print(" 🔘 [스페이스바] 또는 's' / 's'+Enter : 손가락 빠른 굽히기 (BEND_SEQ)")
        print(" 🔘 [ESC]       또는 'r' / 'r'+Enter : 0° 원점 순차 정렬 (ALIGN)")
        print(" 🔘 'e'         또는 'e'+Enter       : J1 관절 -1260 ↔️ -1020 count 3회 왕복 운동 (WIGGLE_J1)")
        print(" 🕹️ [↑ 화살표] / [↓ 화살표]         : J1 관절 상하 비례 이동 / 복귀")
        print(" 🕹️ [← 화살표] / [→ 화살표]         : J2, J3, J4 관절 비례 굽힘 / 펴짐")
        print(" 🛑 'q' / 'Q'                       : 0V 인가 및 안전 종료")
        print("=" * 70 + "\n")
        print("🔄 [자동 정렬] 시작 시 모든 관절을 정렬 위치(0°)로 이동합니다...\n")

        try:
            while rclpy.ok():
                loop_start_time = time.monotonic()

                # 1. ROS 2 통신 이벤트 수신 (누적 토픽 일괄 수신)
                rclpy.spin_once(self, timeout_sec=0.0)

                # 2. 키보드 입력 처리
                key = key_reader.get_key(timeout=0.0)

                if key is not None:
                    # A) [스페이스 바] 또는 's' / 's'+Enter -> 굽히기 (BEND_SEQ)
                    if key == 'SPACE' or key == 's':
                        if self.current_state in ["STANDBY", "HOLD", "ALIGN_ONLY", "ALIGN_J2", "ALIGN_J3", "ALIGN_J4"]:
                            self.current_state = "BEND_SEQ"
                            self.state_start_time = time.monotonic()
                            self.joy_flex_ratio = 1.0
                            print("\n🔥 [순차 굽히기 시퀀스 시작] (Spacebar / 's') -> J2(20°) ➡️ J3(40°) ➡️ J4(70°) 단계별 굽힘 실행")

                    # B) [ESC] 또는 'r' / 'r'+Enter -> 순차 정렬 (ALIGN_J2 -> ALIGN_J3 -> ALIGN_J4)
                    elif key == 'ESC' or key == 'r':
                        self.current_state = "ALIGN_J2"
                        self.state_start_time = time.monotonic()
                        self.joy_j1_ratio = 0.0
                        self.joy_flex_ratio = 0.0
                        print("\n🔄 [순차 정렬 시작] (ESC / 'r') -> J2 ➡️ J3 ➡️ J4 순서로 0° 원점 복귀")

                    # C) 'e' 또는 'e'+Enter -> J1 관절 +5° 3회 왕복 운동 (WIGGLE_J1)
                    elif key == 'e':
                        self.current_state = "WIGGLE_J1"
                        self.state_start_time = time.monotonic()
                        print("\n↔️ ['e' 키 실행] Joint 1번 (-1260 ↔️ -1020 count) 3회 왕복 운동 시작")

                    # C) 화살표 키 -> 조이스틱 대체 실시간 비례 제어
                    elif key == 'UP':
                        if self.current_state in ["STANDBY", "HOLD"]:
                            self.current_state = "STANDBY"
                            self.joy_j1_ratio = min(1.0, self.joy_j1_ratio + 0.1)

                    elif key == 'DOWN':
                        if self.current_state in ["STANDBY", "HOLD"]:
                            self.current_state = "STANDBY"
                            self.joy_j1_ratio = max(0.0, self.joy_j1_ratio - 0.1)

                    elif key == 'LEFT':
                        if self.current_state in ["STANDBY", "HOLD"]:
                            self.current_state = "STANDBY"
                            self.joy_flex_ratio = min(1.0, self.joy_flex_ratio + 0.1)

                    elif key == 'RIGHT':
                        if self.current_state in ["STANDBY", "HOLD"]:
                            self.current_state = "STANDBY"
                            self.joy_flex_ratio = max(0.0, self.joy_flex_ratio - 0.1)

                    # D) 엔터 키 및 문자 버퍼 처리 (s+Enter, r+Enter 지원)
                    elif key == 'ENTER':
                        cmd = self.input_buffer.strip().lower()
                        self.input_buffer = ""
                        if cmd == 's':
                            self.current_state = "BEND_SEQ"
                            self.state_start_time = time.monotonic()
                            self.joy_flex_ratio = 1.0
                            print("\n🔥 [s + Enter 순차 굽히기 시작] J2(20°) ➡️ J3(40°) ➡️ J4(70°) 단계별 굽힘 실행")
                        elif cmd == 'r':
                            self.current_state = "ALIGN_J2"
                            self.state_start_time = time.monotonic()
                            self.joy_j1_ratio = 0.0
                            self.joy_flex_ratio = 0.0
                            print("\n🔄 [r + Enter 순차 정렬 시작] J2 ➡️ J3 ➡️ J4 순서로 0° 원점 복귀")
                        elif cmd == 'e':
                            self.current_state = "WIGGLE_J1"
                            self.state_start_time = time.monotonic()
                            print("\n↔️ ['e + Enter' 실행] Joint 1번 (-1260 ↔️ -1020 count) 3회 왕복 운동 시작")
                        elif cmd == 'q':
                            print("\n🛑 [종료] 사용자 종료 요청.")
                            raise KeyboardInterrupt

                    elif key == 'q':
                        print("\n🛑 [종료] 'q' 키 누름 -> 안전 종료.")
                        raise KeyboardInterrupt

                    else:
                        if len(key) == 1 and key.isalnum():
                            self.input_buffer += key

                # 3. 비례 조종 비율 LPF 처리 (부드러운 움직임 유지)
                self.filtered_j1_ratio = (0.15 * self.joy_j1_ratio) + (0.85 * self.filtered_j1_ratio)
                self.filtered_flex_ratio = (0.15 * self.joy_flex_ratio) + (0.85 * self.filtered_flex_ratio)

                # 4. 상태기기 (State Machine) 처리
                elapsed_time = time.monotonic() - self.state_start_time

                if self.current_state == "ALIGN_ONLY" or self.current_state == "ALIGN_J2":
                    # 1단계: J1 0° 고정, J2 0° 정렬 (0.3초 대기)
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "ALIGN_J3"
                        self.state_start_time = time.monotonic()
                        print("\n➡️ [정렬 2단계] 조인트 3번 0° 정렬 시작")

                elif self.current_state == "ALIGN_J3":
                    # 2단계: J1 0°, J2 0°, J3 0° 정렬 (0.3초 대기)
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 0.0)
                    self.update_target_degree('j3', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "ALIGN_J4"
                        self.state_start_time = time.monotonic()
                        print("➡️ [정렬 3단계] 조인트 4번 0° 정렬 시작")

                elif self.current_state == "ALIGN_J4":
                    # 3단계: J1~J4 모든 관절 0° 정렬 완료 후 HOLD 고정 (0.3초 대기)
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 0.0)
                    self.update_target_degree('j3', 0.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "HOLD"
                        print("✅ [순차 정렬 완료] 조인트 2 ➡️ 3 ➡️ 4 순차 0° 원점 복귀 완료.")

                elif self.current_state == "WIGGLE_J1":
                    # 1회 왕복 주기: 2.0초 (총 3회 = 6.0초)
                    wiggle_period = 2.0
                    num_cycles = 3.0
                    total_wiggle_time = wiggle_period * num_cycles
                    
                    if elapsed_time <= total_wiggle_time:
                        # -1260 count (0°) ~ -1020 count (+21.09°) 사인파 부드러운 왕복 운동
                        max_wiggle_deg = (-1020.0 - (-1260.0)) / self.PULSES_PER_DEGREE  # 240 / 11.378 = 21.0933°
                        target_j1_deg = (max_wiggle_deg / 2.0) * (1.0 - np.cos(2.0 * np.pi * (1.0 / wiggle_period) * elapsed_time))
                        self.update_target_degree('j1', target_j1_deg)
                        self.update_target_degree('j2', 0.0)
                        self.update_target_degree('j3', 0.0)
                        self.update_target_degree('j4', 0.0)
                    else:
                        # 3회 왕복 완료 후 ALIGN_J2 순차 정렬 상태 전환
                        self.current_state = "ALIGN_J2"
                        self.state_start_time = time.monotonic()
                        print("\n✅ [J1 왕복 완료] Joint 1번 (-1260 ↔️ -1020 count) 3회 왕복 완료. 0° 원점 복귀 정렬 시작.")

                elif self.current_state == "BEND_SEQ":
                    # 0단계: 0.5초간 0° 정렬 후 순차 굽히기 진입
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 0.0)
                    self.update_target_degree('j3', 0.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 0.5:
                        self.current_state = "MOVE_J2"
                        self.state_start_time = time.monotonic()
                        print("\n➡️ [1단계] 조인트 2 구동 시작 (0° ➡️ 20°)")

                elif self.current_state == "MOVE_J2":
                    # 1단계: J2 관절 20° 굽힘 구동 (0.3초 대기)
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 0.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "MOVE_J3"
                        self.state_start_time = time.monotonic()
                        print("➡️ [2단계] 조인트 3 구동 시작 (0° ➡️ 40°)")

                elif self.current_state == "MOVE_J3":
                    # 2단계: J3 관절 40° 굽힘 구동 (0.3초 대기)
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 40.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "MOVE_J4"
                        self.state_start_time = time.monotonic()
                        print("➡️ [3단계] 조인트 4 구동 시작 (0° ➡️ 70°)")

                elif self.current_state == "MOVE_J4":
                    # 3단계: J4 관절 70° 굽힘 구동 완료 후 HOLD 고정
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 40.0)
                    self.update_target_degree('j4', 70.0)
                    if elapsed_time > 0.3:
                        self.current_state = "HOLD"
                        print("✅ [순차 시퀀스 완료] j2=20°, j3=40°, j4=70° 빠른 파지 포즈 수렴 완료.")

                elif self.current_state == "STANDBY":
                    target_j1_deg = 20.21 * self.filtered_j1_ratio
                    target_j2_deg = 20.0 * self.filtered_flex_ratio
                    target_j3_deg = 40.0 * self.filtered_flex_ratio
                    target_j4_deg = 70.0 * self.filtered_flex_ratio

                    self.update_target_degree('j1', target_j1_deg)
                    self.update_target_degree('j2', target_j2_deg)
                    self.update_target_degree('j3', target_j3_deg)
                    self.update_target_degree('j4', target_j4_deg)

                elif self.current_state == "HOLD":
                    pass

                # 5. PI-D + Feedforward 전압 제어 연산 (독립 운용 최적화)
                voltage_cmds = {}
                time_since_feedback = time.monotonic() - self.last_feedback_time

                for j_key in ['j1', 'j2', 'j3', 'j4']:
                    cfg = self.JOINT_CONFIG[j_key]
                    state = self.joint_states[j_key]
                    gc = self.GAIN_CONFIG[j_key]

                    # 피드백 타임아웃(0.5초 이상 수신 없음) 시 안전 0V 차단
                    if time_since_feedback > 0.5:
                        voltage_cmds[j_key] = 0.0
                        state['error_integral'] = 0.0
                        continue

                    error_count = state['target_count'] - state['current_count']

                    # Velocity LPF 저역통과 필터
                    filtered_vel = (self.LPF_ALPHA * state['velocity_raw']) + ((1.0 - self.LPF_ALPHA) * state['filtered_velocity_old'])
                    state['filtered_velocity_old'] = filtered_vel

                    v_d = -gc['Kd'] * filtered_vel
                    v_emf = self.K_emf_count * filtered_vel
                    deadzone_thresh = gc['DEADZONE_DEG'] * self.PULSES_PER_DEGREE

                    if abs(error_count) <= deadzone_thresh:
                        state['error_integral'] = 0.0
                        total_v = 0.0  # 데드존 범위 내 지터 방지 0V
                    else:
                        # ⚡ [왕복 오실레이션 방지 1] 오차 부호 반전(Overshoot) 시 적분항 즉시 초기화
                        if (state['prev_error'] * error_count) < 0.0:
                            state['error_integral'] = 0.0
                        state['prev_error'] = error_count

                        v_p = gc['Kp'] * error_count
                        state['error_integral'] += error_count * self.dt
                        if gc['Ki'] != 0.0:
                            limit_val = gc['Ki_limit'] / gc['Ki']
                            state['error_integral'] = max(-limit_val, min(limit_val, state['error_integral']))
                        else:
                            state['error_integral'] = 0.0

                        v_i = gc['Ki'] * state['error_integral']
                        
                        # ⚡ [왕복 오실레이션 방지 2] np.tanh 기반 스무스 마찰 보상 (불감대 경계 전압 충격 연속화)
                        v_frict = np.tanh(error_count / (deadzone_thresh * 2.0)) * gc['FRICT_COMP']
                        total_v = v_p + v_i + v_d + v_emf + v_frict

                    # 물리적인 이동 한계 가드
                    min_lim = min(cfg['FLEX'], cfg['EXT'])
                    max_lim = max(cfg['FLEX'], cfg['EXT'])
                    if (state['current_count'] >= max_lim and total_v > 0) or \
                       (state['current_count'] <= min_lim and total_v < 0):
                        total_v = 0.0
                        state['error_integral'] = 0.0

                    clamped_v = max(-self.voltage_limit, min(self.voltage_limit, total_v))
                    voltage_cmds[j_key] = clamped_v

                # 6. ROS 2 토픽으로 전압 명령 인가
                self.send_voltages(voltage_cmds)

                # 7. 실시간 터미널 모니터링 한 줄 출력
                sys.stdout.write(
                    f"\r ⚙️ [{self.current_state:10s}] | "
                    f"J1:{int(self.joint_states['j1']['current_count']):5d}/{int(self.joint_states['j1']['target_count']):5d} | "
                    f"J2:{int(self.joint_states['j2']['current_count']):5d}/{int(self.joint_states['j2']['target_count']):5d} | "
                    f"J3:{int(self.joint_states['j3']['current_count']):5d}/{int(self.joint_states['j3']['target_count']):5d} | "
                    f"J4:{int(self.joint_states['j4']['current_count']):5d}/{int(self.joint_states['j4']['target_count']):5d} | "
                    f"Volts:[{int(voltage_cmds['j1'])},{int(voltage_cmds['j2'])},{int(voltage_cmds['j3'])},{int(voltage_cmds['j4'])}]"
                )
                sys.stdout.flush()

                # Loop timing (50Hz)
                elapsed_loop = time.monotonic() - loop_start_time
                sleep_time = self.dt - elapsed_loop
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n🛑 [안전 정지] 모든 조인트 전압 0mV 차단 및 프로그램 종료...")
            self.send_voltages({'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'j4': 0.0})
        finally:
            key_reader.close()


def main(args=None):
    rclpy.init(args=args)
    driver = WelconKeyboardDriver()
    try:
        driver.run()
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
