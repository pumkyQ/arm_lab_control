#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🕹️ KITECH 손가락 관절 키보드 간소화 컨트롤러 (ROS 2 기반)
========================================================================
[조작 방법]
  - 's' 키 또는 [스페이스바] : 손가락 굽히기 (J2=40°, J3=40°, J4=40°)
  - 'r' 키 또는 [ESC]       : 0° 원점 복귀 / 펴기 (모든 조인트 0°)
  - 'q' 키                 : 전압 0mV 즉시 차단 및 종료

[ROS 2 토픽]
  - Sub: /joint_states_raw  (real_welcon_driver.py 피드백 수신)
  - Pub: /joint_voltage_cmd (real_welcon_driver.py 전압 명령 발행)
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
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class KeyInputReader:
    """터미널 키 입력 파서"""
    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def get_key(self, timeout=0.0):
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None

        ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
        if ch == '\x1b':
            return 'ESC'
        elif ch == ' ':
            return 'SPACE'
        elif ch in ['\r', '\n']:
            return 'ENTER'
        else:
            return ch.lower()

    def close(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


class WelconSimpleKeyboardController(Node):
    def __init__(self):
        super().__init__('welcon_simple_keyboard_controller')

        # 1. ROS 2 퍼블리셔 및 서브스크라이버 설정 (QoS depth=1)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.volt_pub = self.create_publisher(
            Float64MultiArray, '/joint_voltage_cmd', qos_profile)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states_raw', self.joint_state_callback, qos_profile)

        # 2. 하드웨어 물리 상수 및 조인트 구성
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi)
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN
        self.voltage_limit = 10000.0  # 최대 전압 10V (10000mV)

        # 조인트 매핑 (Node 3, Axis 1/2  |  Node 1, Axis 1/2)
        self.JOINT_CONFIG = {
            'j1': {'ALIGN': -1260.0, 'FLEX': -1030.0, 'EXT': -1260.0},
            'j2': {'ALIGN': -1990.0, 'FLEX': -1040.0, 'EXT': -2055.0},
            'j3': {'ALIGN': -1706.0, 'FLEX':  -854.0, 'EXT': -1769.0},
            'j4': {'ALIGN':   574.0, 'FLEX':  1626.0, 'EXT':  -422.0}
        }

        # PID 게인 및 마찰 보상 파라미터 (MOTOR_DIR: 모터 전압 방향 부호 계수)
        self.GAIN_CONFIG = {
            'j1': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 300.0,  'DEADZONE_DEG': 2.0, 'FRICT_COMP': 1000.0, 'MOTOR_DIR': 1.0},
            'j2': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 1000.0, 'DEADZONE_DEG': 6.0, 'FRICT_COMP': 1000.0, 'MOTOR_DIR': 1.0},
            'j3': {'Kp': 450.0, 'Kd': 15.0, 'Ki': 3.0, 'Ki_limit': 1500.0, 'DEADZONE_DEG': 6.0, 'FRICT_COMP': 1200.0, 'MOTOR_DIR': 1.0},
            'j4': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 1000.0, 'DEADZONE_DEG': 6.0, 'FRICT_COMP': 1000.0, 'MOTOR_DIR': 1.0}
        }

        self.LOOP_RATE = 50.0  # 50Hz (20ms)
        self.dt = 1.0 / self.LOOP_RATE
        self.LPF_ALPHA = 0.25

        # 3. 관절 제어 상태 관리
        self.current_mode = "ALIGN"  # "ALIGN" (원점/펴기)  or  "BEND" (굽히기)
        self.last_feedback_time = time.monotonic()

        self.joint_states = {}
        for j_key in ['j1', 'j2', 'j3', 'j4']:
            align_cnt = self.JOINT_CONFIG[j_key]['ALIGN']
            self.joint_states[j_key] = {
                'target_count': align_cnt,
                'current_count': align_cnt,
                'velocity_raw': 0.0,
                'filtered_velocity_old': 0.0,
                'error_integral': 0.0,
                'prev_error': 0.0
            }

    def joint_state_callback(self, msg: JointState):
        """real_welcon_driver로부터 엔코더 피드백 수신"""
        self.last_feedback_time = time.monotonic()
        for idx, name in enumerate(msg.name):
            if name in self.joint_states:
                if len(msg.position) > idx:
                    self.joint_states[name]['current_count'] = float(msg.position[idx])
                if len(msg.velocity) > idx:
                    self.joint_states[name]['velocity_raw'] = float(msg.velocity[idx])

    def set_target_degrees(self, j1_deg, j2_deg, j3_deg, j4_deg):
        """각 조인트 목표 각도를 카운트로 변환 후 범위 클램핑"""
        targets = {'j1': j1_deg, 'j2': j2_deg, 'j3': j3_deg, 'j4': j4_deg}
        for j_key, deg in targets.items():
            cfg = self.JOINT_CONFIG[j_key]
            req_cnt = cfg['ALIGN'] + (deg * self.PULSES_PER_DEGREE)
            min_lim = min(cfg['FLEX'], cfg['EXT'])
            max_lim = max(cfg['FLEX'], cfg['EXT'])
            clamped = max(min_lim, min(max_lim, req_cnt))
            self.joint_states[j_key]['target_count'] = float(clamped)

    def send_voltages(self, voltage_dict):
        """ROS 2 전압 명령 토픽 발행"""
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
        print(" 🎯 KITECH 손가락 관절 키보드 간소화 컨트롤러")
        print("=" * 70)
        print(" 🔘 [f] 키                 : 조인트 1번(J1) 20° 독립 구동")
        print(" 🔘 [s] 키 또는 [스페이스바] : 조인트 2, 3, 4번 20° 굽히기 (J2=20°, J3=20°, J4=20°)")
        print(" 🔘 [r] 키 또는 [ESC]       : 0° 원점 복귀 / 펴기 (모든 조인트 0°)")
        print(" 🛑 [q] 키                 : 전압 0mV 인가 및 프로그램 종료")
        print("=" * 70 + "\n")
        print("🔄 [초기화] 모든 조인트를 0° 원점 복귀 정렬 상태로 가동합니다...\n")

        # 시작 시 0° 정렬 세팅
        self.set_target_degrees(0.0, 0.0, 0.0, 0.0)

        try:
            while rclpy.ok():
                loop_start_time = time.monotonic()

                # 1. ROS 2 통신 피드백 수신
                rclpy.spin_once(self, timeout_sec=0.0)

                # 2. 키 입력 처리
                key = key_reader.get_key(timeout=0.0)
                if key is not None:
                    if key == 'f':
                        self.current_mode = "MOVE_J1"
                        self.set_target_degrees(20.0, 0.0, 0.0, 0.0)
                        print("\n☝️ [J1 구동 명령] J1=20°, J2=0°, J3=0°, J4=0° 구동 시작")

                    elif key in ['s', 'SPACE']:
                        self.current_mode = "BEND"
                        self.set_target_degrees(0.0, 20.0, 20.0, 20.0)
                        print("\n🔥 [굽히기 명령] J2=20°, J3=20°, J4=20° 굽힘 시작")

                    elif key in ['r', 'ESC']:
                        self.current_mode = "ALIGN"
                        self.set_target_degrees(0.0, 0.0, 0.0, 0.0)
                        print("\n🔄 [원점 펴기 명령] 모든 조인트 0° 정렬 위치 복귀")

                    elif key == 'q':
                        print("\n🛑 [종료] 사용자 종료 요청.")
                        raise KeyboardInterrupt

                # 3. PID 전압 제어 연산
                voltage_cmds = {}
                time_since_feedback = time.monotonic() - self.last_feedback_time

                for j_key in ['j1', 'j2', 'j3', 'j4']:
                    cfg = self.JOINT_CONFIG[j_key]
                    state = self.joint_states[j_key]
                    gc = self.GAIN_CONFIG[j_key]

                    # 피드백 타임아웃(0.5초 이상 피드백 끊김 시 안전 0V 차단)
                    if time_since_feedback > 0.5:
                        voltage_cmds[j_key] = 0.0
                        state['error_integral'] = 0.0
                        continue

                    error_count = state['target_count'] - state['current_count']

                    # Velocity LPF
                    filtered_vel = (self.LPF_ALPHA * state['velocity_raw']) + \
                                   ((1.0 - self.LPF_ALPHA) * state['filtered_velocity_old'])
                    state['filtered_velocity_old'] = filtered_vel

                    v_d = -gc['Kd'] * filtered_vel
                    v_emf = self.K_emf_count * filtered_vel
                    deadzone_thresh = gc['DEADZONE_DEG'] * self.PULSES_PER_DEGREE

                    if abs(error_count) <= deadzone_thresh:
                        state['error_integral'] = 0.0
                        total_v = 0.0
                    else:
                        if (state['prev_error'] * error_count) < 0.0:
                            state['error_integral'] = 0.0
                        state['prev_error'] = error_count

                        v_p = gc['Kp'] * error_count
                        state['error_integral'] += error_count * self.dt

                        if gc['Ki'] > 0.0:
                            max_integral = gc['Ki_limit'] / gc['Ki']
                            state['error_integral'] = max(-max_integral, min(max_integral, state['error_integral']))
                            v_i = gc['Ki'] * state['error_integral']
                        else:
                            v_i = 0.0

                        v_frict = np.tanh(error_count / (deadzone_thresh * 2.0)) * gc['FRICT_COMP']
                        total_v = (v_p + v_i + v_d + v_emf + v_frict) * gc.get('MOTOR_DIR', 1.0)

                    # 소프트웨어 이동 제한 한계 가드
                    min_lim = min(cfg['FLEX'], cfg['EXT'])
                    max_lim = max(cfg['FLEX'], cfg['EXT'])
                    if (state['current_count'] >= max_lim and total_v > 0) or \
                       (state['current_count'] <= min_lim and total_v < 0):
                        total_v = 0.0
                        state['error_integral'] = 0.0

                    clamped_v = max(-self.voltage_limit, min(self.voltage_limit, total_v))
                    voltage_cmds[j_key] = clamped_v

                # 4. 전압 명령 발행
                self.send_voltages(voltage_cmds)

                # 5. 터미널 실시간 모니터링 출력
                sys.stdout.write(
                    f"\r ⚙️ [{self.current_mode:5s}] | "
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
            print("\n\n🛑 [안전 정지] 모든 조인트 전압 0mV 차단 및 종료...")
            self.send_voltages({'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'j4': 0.0})
        finally:
            key_reader.close()


def main(args=None):
    rclpy.init(args=args)
    controller = WelconSimpleKeyboardController()
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
