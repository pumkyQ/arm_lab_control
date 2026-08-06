#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🦾 KITECH 손가락 순차 구동기 (ROS 2 없이 SocketCAN 직결 단일 독립 스크립트)
========================================================================
- PEAK-USB (can0, 1Mbps) 하드웨어에 직접 연결하여 NMT 및 Operation Enable 시퀀스를 수행합니다.
- ROS 2 설치 없이 단일 파이썬 스크립트로 j2, j3, j4 관절을 0° 정렬 후 20° -> 40° -> 35° 순차 구동합니다.
- 's' 키: 시퀀스 시작 | 'r' 키: 리셋 | 'q' 키: 전압 0V 즉시 차단 및 안전 종료
"""

import sys
import os
import time
import select
import termios
import tty
import numpy as np

# kitech_v1 패키지 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.dirname(current_dir)
kitech_path = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path:
    sys.path.append(kitech_path)
if workspace_dir not in sys.path:
    sys.path.append(workspace_dir)

try:
    from motor_control.cia402 import Cia402Protocol, Cia402Controlword, Cia402Object
    from motor_control.can_bus import SocketCanBus
except ModuleNotFoundError as e:
    print("\n❌ kitech_v1 모듈을 찾지 못했습니다. 디렉토리 구조를 확인해 주세요!")
    raise e

def kbhit():
    """키보드 입력을 비동기로 감지하는 함수"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class StandaloneFingerBendController:
    def __init__(self, can_channel='can0'):
        self.can_channel = can_channel
        
        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 물리 상수 및 조인트별 측정 데이터 매핑]
        # ----------------------------------------------------------------------
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi)
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN
        self.voltage_limit = 9500.0      # 최대 인가 전압 한계 (mV)

        # 4개 조인트 (j1, j2, j3, j4) 노드 및 축 매핑
        # j1: Node 3 Axis 1 | j2: Node 2 Axis 1 | j3: Node 1 Axis 1 | j4: Node 3 Axis 2
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -214.0,  'FLEX': 806.0,   'EXT': -1242.0},
            'j2': {'NODE_ID': 2, 'AXIS': 1, 'ALIGN': -1008.0, 'FLEX': -655.0,  'EXT': -1335.0}, 
            'j3': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': 596.0,   'FLEX': 1651.0,  'EXT': -397.0},
            'j4': {'NODE_ID': 3, 'AXIS': 2, 'ALIGN': -388.0,  'FLEX': 664.0,   'EXT': -1384.0} 
        }

        # 조인트별 제어 게인 및 마찰 보상 세팅
        self.GAIN_CONFIG = {
            'j1': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 800.0},
            'j2': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0},
            'j3': {'Kp': 450.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 800.0, 'DEADZONE_DEG': 1.5, 'FRICT_COMP': 1200.0},
            'j4': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0}
        }

        self.LOOP_RATE = 50.0            # 50Hz (20ms 주기)
        self.dt = 1.0 / self.LOOP_RATE
        self.LPF_ALPHA = 0.25            # LPF 저역통과 필터 계수

        # ----------------------------------------------------------------------
        # [🔄 상태 기기 변수 초기화]
        # ----------------------------------------------------------------------
        self.current_state = "STANDBY"
        self.state_start_time = time.monotonic()
        self.input_buffer = ""

        # 각 조인트 상태 트래킹
        self.joint_states = {}
        for j_key in ['j1', 'j2', 'j3', 'j4']:
            self.joint_states[j_key] = {
                'target_count': self.JOINT_CONFIG[j_key]['ALIGN'], 
                'current_count': self.JOINT_CONFIG[j_key]['ALIGN'],
                'velocity_raw': 0.0,
                'filtered_velocity_old': 0.0,
                'status_word': 0,
                'error_integral': 0.0
            }

        # SDO 객체 매핑
        self.protocol = Cia402Protocol()
        self.sdo_map = {}
        for j_key, cfg in self.JOINT_CONFIG.items():
            node_id = cfg['NODE_ID']
            axis = cfg['AXIS']
            self.sdo_map[j_key] = {
                'pos': Cia402Object(0x6064 if axis == 1 else 0x6864),
                'vel': Cia402Object(0x606c if axis == 1 else 0x686c),
                'status': Cia402Object(0x6041 if axis == 1 else 0x6841),
                'voltage': Cia402Object(0x60ed if axis == 1 else 0x68ed) # 전압 인가 SDO
            }

        # 터미널 설정 백업 (비동기 키보드 입력용)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ----------------------------------------------------------------------
        # [📡 SocketCAN 연결 및 하드웨어 초기화]
        # ----------------------------------------------------------------------
        try:
            self.can_context = SocketCanBus(channel=self.can_channel, receive_timeout=0.0)
            self.bus = self.can_context.__enter__()
            print(f"\n✅ CAN 버스 연결 성공: {self.can_channel}")
        except Exception as e:
            print(f"\n❌ CAN 버스 연결 실패 ({self.can_channel}): {e}")
            self.bus = None

    def init_hardware(self):
        """Welcon 하드웨어 전압 모드 설정 및 NMT Start + Enable 시퀀스 진행"""
        if self.bus is None: return

        print("▶ 모터 드라이버 (j1, j2, j3, j4) 초기화 및 Operation Enable 진행 중...")
        
        # NMT Start (모든 노드 개시)
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)

        for j_key, cfg in self.JOINT_CONFIG.items():
            node_id = cfg['NODE_ID']
            axis = cfg['AXIS']
            
            # 전압 제어 모드 (-11) 설정
            mode_frame = self.protocol.make_axis_mode_sdo(node_id, axis, -11)
            self.bus.send(mode_frame)
            time.sleep(0.02)

            # State Machine Enable 시퀀스
            for ctrl in (Cia402Controlword.FAULT_RESET,
                         Cia402Controlword.SHUTDOWN,
                         Cia402Controlword.SWITCH_ON,
                         Cia402Controlword.ENABLE_OPERATION):
                cw_frame = self.protocol.make_controlword_sdo(node_id, axis, ctrl)
                self.bus.send(cw_frame)
                time.sleep(0.02)

        print("✅ 하드웨어 가동 준비 완료!\n")

    def update_target_degree(self, joint_key, degree):
        """관절 각도(도) 기반 목표 카운트 업데이트 및 Clamping"""
        cfg = self.JOINT_CONFIG[joint_key]
        requested_count = cfg['ALIGN'] + (degree * self.PULSES_PER_DEGREE)
        
        min_lim = min(cfg['FLEX'], cfg['EXT'])
        max_lim = max(cfg['FLEX'], cfg['EXT'])
        
        clamped = max(min_lim, min(max_lim, requested_count))
        self.joint_states[joint_key]['target_count'] = float(clamped)

    def read_joint_feedbacks(self):
        """SDO 읽기 요청 및 하드웨어 엔코더 위치 피드백 수신"""
        if self.bus is None: return

        # 1. SDO 위치/속도 요청 송신
        for j_key, cfg in self.JOINT_CONFIG.items():
            node_id = cfg['NODE_ID']
            pos_obj = self.sdo_map[j_key]['pos']
            vel_obj = self.sdo_map[j_key]['vel']
            
            self.bus.send(self.protocol.make_sdo_read(node_id, pos_obj.index, pos_obj.subindex))
            self.bus.send(self.protocol.make_sdo_read(node_id, vel_obj.index, vel_obj.subindex))

        # 2. CAN 버스로부터 수신 팩킷 파싱
        start_t = time.monotonic()
        while (time.monotonic() - start_t) < 0.005: # 5ms 동안 수신 버퍼 긁기
            msg = self.bus.recv(timeout=0.0)
            if msg is None: break

            for j_key, cfg in self.JOINT_CONFIG.items():
                node_id = cfg['NODE_ID']
                pos_obj = self.sdo_map[j_key]['pos']
                vel_obj = self.sdo_map[j_key]['vel']

                parsed_pos = self.protocol.parse_sdo_read_response(msg, node_id, pos_obj.index, pos_obj.subindex)
                if parsed_pos is not None:
                    self.joint_states[j_key]['current_count'] = float(parsed_pos)

                parsed_vel = self.protocol.parse_sdo_read_response(msg, node_id, vel_obj.index, vel_obj.subindex)
                if parsed_vel is not None:
                    self.joint_states[j_key]['velocity_raw'] = float(parsed_vel)

    def send_voltages(self, voltage_dict):
        """모든 조인트에 계산된 전압(mV) SDO 송신"""
        if self.bus is None: return

        for j_key, volt in voltage_dict.items():
            cfg = self.JOINT_CONFIG[j_key]
            volt_obj = self.sdo_map[j_key]['voltage']
            
            # 16비트 정수형 전압 변환 (-9500mV ~ +9500mV)
            volt_int = int(round(volt))
            frame = self.protocol.make_sdo_write(cfg['NODE_ID'], volt_obj.index, volt_obj.subindex, volt_int, data_length=2)
            self.bus.send(frame)

    def run(self):
        self.init_hardware()

        print("=" * 65)
        print(" 🎯 KITECH 손가락 관절 순차 구동기 (Standalone Controller)")
        print(" ▶ j2, j3, j4 정렬 후 각각 20도, 40도, 35도 순차 기동")
        print(" ▶ 's' 키 + Enter: 시퀀스 시작 | 'r' 키 + Enter: 리셋 | 'q' 키: 안전 종료")
        print("=" * 65 + "\n")

        try:
            while True:
                loop_start_time = time.monotonic()

                # 1. 하드웨어 피드백 수신
                self.read_joint_feedbacks()

                # 2. 키보드 비동기 입력 처리
                while kbhit():
                    char = os.read(sys.stdin.fileno(), 1).decode()
                    if char in ['\r', '\n']:
                        cmd = self.input_buffer.strip().lower()
                        self.input_buffer = ""
                        
                        if cmd == 's':
                            if self.current_state in ["STANDBY", "HOLD"]:
                                self.current_state = "ALIGN"
                                self.state_start_time = time.monotonic()
                                print("\n🔥 [시퀀스 시작] j2, j3, j4 정렬을 시작합니다 (0°로 정렬)")
                        elif cmd == 'r':
                            self.current_state = "STANDBY"
                            for name in ['j1', 'j2', 'j3', 'j4']:
                                self.joint_states[name]['target_count'] = self.joint_states[name]['current_count']
                                self.joint_states[name]['error_integral'] = 0.0
                            print("\n🔄 [리셋] 대기 상태로 복귀 및 목표치를 현재 위치로 동기화했습니다.")
                        elif cmd == 'q':
                            print("\n🛑 [종료] 안전 종료 시퀀스를 실행합니다.")
                            raise KeyboardInterrupt
                    elif char in ['\x08', '\x7f']:
                        if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                    else:
                        self.input_buffer += char

                # 3. 시퀀스 제어 상태기기 (State Machine)
                elapsed_time = time.monotonic() - self.state_start_time

                if self.current_state == "ALIGN":
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 0.0)
                    self.update_target_degree('j3', 0.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 3.0:
                        self.current_state = "MOVE_J2"
                        self.state_start_time = time.monotonic()
                        print("\n➡️ [1단계] 조인트 2 구동 시작 (0° ➡️ 20°)")

                elif self.current_state == "MOVE_J2":
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 0.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "MOVE_J3"
                        self.state_start_time = time.monotonic()
                        print("\n➡️ [2단계] 조인트 3 구동 시작 (0° ➡️ 40°)")

                elif self.current_state == "MOVE_J3":
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 40.0)
                    self.update_target_degree('j4', 0.0)
                    if elapsed_time > 0.3:
                        self.current_state = "MOVE_J4"
                        self.state_start_time = time.monotonic()
                        print("\n➡️ [3단계] 조인트 4 구동 시작 (0° ➡️ 35°)")

                elif self.current_state == "MOVE_J4":
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 40.0)
                    self.update_target_degree('j4', 35.0)
                    if elapsed_time > 0.3:
                        self.current_state = "HOLD"
                        print("\n✅ [시퀀스 완료] j2=20°, j3=40°, j4=35° 파지 포즈 수렴 완료.")

                elif self.current_state == "HOLD":
                    self.update_target_degree('j1', 0.0)
                    self.update_target_degree('j2', 20.0)
                    self.update_target_degree('j3', 40.0)
                    self.update_target_degree('j4', 35.0)

                # 4. PI-D + Feedforward 전압 제어 연산
                voltage_cmds = {}
                for j_key in ['j1', 'j2', 'j3', 'j4']:
                    cfg = self.JOINT_CONFIG[j_key]
                    state = self.joint_states[j_key]
                    gc = self.GAIN_CONFIG[j_key]

                    error_count = state['target_count'] - state['current_count']

                    # LPF 저역통과필터
                    filtered_vel = (self.LPF_ALPHA * state['velocity_raw']) + ((1.0 - self.LPF_ALPHA) * state['filtered_velocity_old'])
                    state['filtered_velocity_old'] = filtered_vel

                    v_d = -gc['Kd'] * filtered_vel
                    v_emf = self.K_emf_count * filtered_vel
                    deadzone_thresh = gc['DEADZONE_DEG'] * self.PULSES_PER_DEGREE

                    if abs(error_count) <= deadzone_thresh:
                        state['error_integral'] = 0.0
                        active_dir = np.sign(filtered_vel if filtered_vel != 0.0 else error_count)
                        v_stiction_tail = (gc['FRICT_COMP'] * 0.8) * active_dir if active_dir != 0 else 0.0
                        total_v = v_d + v_emf + v_stiction_tail
                    else:
                        v_p = gc['Kp'] * error_count
                        state['error_integral'] += error_count * self.dt
                        if gc['Ki'] != 0.0:
                            state['error_integral'] = max(-gc['Ki_limit']/gc['Ki'], min(gc['Ki_limit']/gc['Ki'], state['error_integral']))
                        else:
                            state['error_integral'] = 0.0
                        
                        v_i = gc['Ki'] * state['error_integral']
                        v_frict = np.sign(error_count) * gc['FRICT_COMP']
                        total_v = v_p + v_i + v_d + v_emf + v_frict

                    # 물리 한계 가드
                    min_lim = min(cfg['FLEX'], cfg['EXT'])
                    max_lim = max(cfg['FLEX'], cfg['EXT'])
                    if (state['current_count'] >= max_lim and total_v > 0) or \
                       (state['current_count'] <= min_lim and total_v < 0):
                        total_v = 0.0
                        state['error_integral'] = 0.0

                    clamped_v = max(-self.voltage_limit, min(self.voltage_limit, total_v))
                    voltage_cmds[j_key] = clamped_v

                # 5. 하드웨어 전압 전송
                self.send_voltages(voltage_cmds)

                # 6. 실시간 터미널 모니터링 출력
                sys.stdout.write(
                    f"\r ⚙️ [상태: {self.current_state:8s}] | "
                    f"J1: {int(self.joint_states['j1']['current_count']):5d}/{int(self.JOINT_CONFIG['j1']['ALIGN']):5d} | "
                    f"J2: {int(self.joint_states['j2']['current_count']):5d}/{int(self.JOINT_CONFIG['j2']['ALIGN']):5d} | "
                    f"J3: {int(self.joint_states['j3']['current_count']):5d}/{int(self.JOINT_CONFIG['j3']['ALIGN']):5d} | "
                    f"J4: {int(self.joint_states['j4']['current_count']):5d}/{int(self.JOINT_CONFIG['j4']['ALIGN']):5d} | "
                    f"입력: {self.input_buffer}"
                )
                sys.stdout.flush()

                # Loop Rate 타이밍 동기화 (50Hz)
                elapsed_loop = time.monotonic() - loop_start_time
                sleep_time = self.dt - elapsed_loop
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n🛑 [안전 정지] 모든 조인트 전압 0mV 차단 및 종료...")
            self.send_voltages({'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'j4': 0.0})
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

if __name__ == '__main__':
    controller = StandaloneFingerBendController(can_channel='can0')
    controller.run()
