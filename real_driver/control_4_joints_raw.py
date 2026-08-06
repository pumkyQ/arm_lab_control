#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import select
import termios
import tty
import os

# =========================================================================
# [⚙️ 패키지 라이브러리 경로 절대 추적 알고리즘 반영]
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.dirname(current_dir)

kitech_path = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path:
    sys.path.append(kitech_path)
if workspace_dir not in sys.path:
    sys.path.append(workspace_dir)

try:
    from motor_control.can_bus import SocketCanBus
    from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword
except ModuleNotFoundError as e:
    print("\n❌ 패키지를 찾지 못했습니다. 디렉토리 구조를 확인해 주세요!")
    raise e

CAN_CHANNEL = "can0"

def kbhit():
    """키보드 입력이 있는지 확인하는 비동기 함수"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class RawEncoderController:
    def __init__(self):
        # ----------------------------------------------------------------------
        # [⚙️ 4개 모터 (조인트) Node & Axis 매핑]
        # ----------------------------------------------------------------------
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 1, 'AXIS': 1},
            'j2': {'NODE_ID': 1, 'AXIS': 2},
            'j3': {'NODE_ID': 2, 'AXIS': 1},
            'j4': {'NODE_ID': 2, 'AXIS': 2},
        }
        
        # PD 제어 게인 (단순화를 위해 P, D 게인만 사용, 필요에 따라 튜닝 가능)
        self.Kp = 350.0
        self.Kd = 15.0
        self.voltage_limit = 8000.0  # 안전을 위한 8V 전압 제한
        
        self.dt = 0.02 # 50Hz 주기
        
        self.active_mode = 'j1'
        self.input_buffer = ""
        self.is_hardware_ready = False
        
        # 각 조인트 상태 트래킹용 딕셔너리
        self.joint_states = {}
        for j_key in self.JOINT_CONFIG.keys():
            self.joint_states[j_key] = {
                'target_count': 0.0, 
                'current_count': 0.0,
                'velocity_raw': 0.0,
                'status_word': 0,
                'is_first_read': True # 초기 위치를 목표 위치로 삼기 위한 플래그
            }
            
        # 터미널 상태 백업 (비동기 입력을 위해 설정 변경)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.protocol = Cia402Protocol()
        # Axis 1용 SDO 객체 (0x60xx)
        self.pos_obj_a1 = Cia402Object(0x6064)
        self.vel_obj_a1 = Cia402Object(0x606c)
        self.stat_obj_a1 = Cia402Object(0x6041)

        # Axis 2용 SDO 객체 (0x68xx)
        self.pos_obj_a2 = Cia402Object(0x6864)
        self.vel_obj_a2 = Cia402Object(0x686c)
        self.stat_obj_a2 = Cia402Object(0x6841)

        try:
            self.can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
            self.bus = self.can_bus_context.__enter__()
            print(f"✅ CAN 연결 성공: {CAN_CHANNEL}")
        except Exception as e:
            print(f"❌ CAN 연결 실패: {e}")
            self.bus = None

    def init_hardware(self):
        """모든 조인트의 하드웨어 활성화 및 초기 엔코더 위치 세팅"""
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.05)

        for j_key, cfg in self.JOINT_CONFIG.items():
            node = cfg['NODE_ID']
            axis = cfg['AXIS']
            self.bus.send(self.protocol.make_axis_mode_sdo(node, axis, -11)) 
            time.sleep(0.02)
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                self.bus.send(self.protocol.make_axis_controlword_sdo(node, axis, ctrl))
                time.sleep(0.02)
                
        # 각 모터가 구동되자마자 튀지 않도록, 최초 현재 엔코더 값을 읽어서 Target으로 고정
        for j_key, cfg in self.JOINT_CONFIG.items():
            obj = self.pos_obj_a1 if cfg['AXIS'] == 1 else self.pos_obj_a2
            self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], obj))
            time.sleep(0.02)
            
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                frame = self.bus.recv(timeout=0.005)
                if frame and frame.can_id == 0x580 + cfg['NODE_ID']:
                    sdo_res = self.protocol.parse_sdo_response(frame)
                    if sdo_res and sdo_res.value is not None:
                        if sdo_res.index in [0x6064, 0x6864]:
                            val = sdo_res.value
                            if val > 0x7FFFFFFF: val -= 0x100000000
                            self.joint_states[j_key]['current_count'] = float(val)
                            self.joint_states[j_key]['target_count'] = float(val)
                            self.joint_states[j_key]['is_first_read'] = False
                            break

    def run(self):
        if self.bus is None: return
        self.init_hardware()
        self.is_hardware_ready = True
        
        print("\n🔥 하드웨어 준비 완료. 제어를 시작합니다.")
        print("💡 사용법 1: 'j2' 입력 후 엔터 -> 제어할 대상을 조인트 2로 변경")
        print("💡 사용법 2: '1500' 입력 후 엔터 -> 현재 선택된 조인트의 목표 엔코더를 1500으로 변경")
        print("💡 사용법 3: 'j3 -500' 입력 후 엔터 -> 조인트 3의 목표 엔코더를 -500으로 즉시 변경")
        print(" [종료: Ctrl + C]\n")

        next_time = time.monotonic()
        
        try:
            while True:
                # --------------------------------------------------------------
                # ⌨️ 터미널 키보드 입력 핸들러
                # --------------------------------------------------------------
                while kbhit():
                    char = os.read(sys.stdin.fileno(), 1).decode()
                    if char in ['\r', '\n']:
                        user_input = self.input_buffer.strip().lower()
                        self.input_buffer = ""
                        
                        parts = user_input.split()
                        if len(parts) == 1:
                            if parts[0] in self.JOINT_CONFIG:
                                self.active_mode = parts[0] # j1, j2 등 모드 변경
                            else:
                                try: # 현재 활성화된 모터의 타겟 변경
                                    self.joint_states[self.active_mode]['target_count'] = float(parts[0])
                                except ValueError: pass
                                
                        elif len(parts) == 2:
                            if parts[0] in self.JOINT_CONFIG:
                                self.active_mode = parts[0]
                                try: # j1 1500 형식으로 타겟 즉시 변경
                                    self.joint_states[self.active_mode]['target_count'] = float(parts[1])
                                except ValueError: pass
                                
                    elif char in ['\x08', '\x7f']: # 백스페이스 처리
                        if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                    else:
                        self.input_buffer += char

                # --------------------------------------------------------------
                # 📡 SDO 데이터 수집 (현재 엔코더, 속도, 상태)
                # --------------------------------------------------------------
                for j_key, cfg in self.JOINT_CONFIG.items():
                    pos = self.pos_obj_a1 if cfg['AXIS'] == 1 else self.pos_obj_a2
                    vel = self.vel_obj_a1 if cfg['AXIS'] == 1 else self.vel_obj_a2
                    stat = self.stat_obj_a1 if cfg['AXIS'] == 1 else self.stat_obj_a2
                    
                    self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], pos))
                    self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], vel))
                    self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], stat))
                    
                timeout_end = time.monotonic() + 0.006
                while time.monotonic() < timeout_end:
                    frame = self.bus.recv(timeout=0.001)
                    if frame is None: continue
                    node_id = frame.can_id - 0x580
                    sdo_res = self.protocol.parse_sdo_response(frame)
                    
                    if sdo_res and sdo_res.value is not None:
                        val = sdo_res.value
                        if val > 0x7FFFFFFF: val -= 0x100000000
                        
                        # 응답이 온 Index를 보고 어떤 조인트인지 매핑 업데이트
                        for j_key, cfg in self.JOINT_CONFIG.items():
                            if cfg['NODE_ID'] == node_id:
                                if cfg['AXIS'] == 1:
                                    if sdo_res.index == 0x6064: self.joint_states[j_key]['current_count'] = float(val)
                                    elif sdo_res.index == 0x606c: self.joint_states[j_key]['velocity_raw'] = float(val)
                                    elif sdo_res.index == 0x6041: self.joint_states[j_key]['status_word'] = val
                                elif cfg['AXIS'] == 2:
                                    if sdo_res.index == 0x6864: self.joint_states[j_key]['current_count'] = float(val)
                                    elif sdo_res.index == 0x686c: self.joint_states[j_key]['velocity_raw'] = float(val)
                                    elif sdo_res.index == 0x6841: self.joint_states[j_key]['status_word'] = val

                # --------------------------------------------------------------
                # 🎯 PD 제어 연산 및 모터에 전압 전송
                # --------------------------------------------------------------
                for j_key, cfg in self.JOINT_CONFIG.items():
                    state = self.joint_states[j_key]
                    
                    # Fault(에러) 상태면 리셋 패킷 송신
                    if (state['status_word'] & 0x08): 
                        self.bus.send(self.protocol.make_axis_controlword_sdo(cfg['NODE_ID'], cfg['AXIS'], Cia402Controlword.FAULT_RESET))
                        continue

                    # 만약 통신 지연으로 엔코더 값을 한 번도 못 읽었다면 제어를 스킵하여 급발진 방지
                    if state['is_first_read']: continue

                    error = state['target_count'] - state['current_count']
                    v_p = self.Kp * error
                    v_d = -self.Kd * state['velocity_raw']
                    
                    total_voltage = v_p + v_d
                    clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
                    
                    # 전압 인가 송신
                    self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(cfg['NODE_ID'], int(clamped_voltage), cfg['AXIS']))

                # --------------------------------------------------------------
                # 🖥️ 화면 모니터링 출력
                # --------------------------------------------------------------
                sys.stdout.write(
                    f"\r 🟢 [현재모드: {self.active_mode.upper():2s}] | "
                    f"J1: {self.joint_states['j1']['current_count']:6.0f} | "
                    f"J2: {self.joint_states['j2']['current_count']:6.0f} | "
                    f"J3: {self.joint_states['j3']['current_count']:6.0f} | "
                    f"J4: {self.joint_states['j4']['current_count']:6.0f} | "
                    f"입력: {self.input_buffer:8s}"
                )
                sys.stdout.flush()

                # 50Hz (0.02초) 주기 준수
                next_time += self.dt
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_time = time.monotonic()

        except KeyboardInterrupt:
            print("\n\n프로그램을 종료합니다...")
        finally:
            self.shutdown_hook()

    def shutdown_hook(self):
        """종료 시 모든 모터에 0V 인가 후 안전 종료"""
        for j_key, cfg in self.JOINT_CONFIG.items():
            try: self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(cfg['NODE_ID'], 0, cfg['AXIS']))
            except: pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        if self.bus: self.can_bus_context.__exit__(None, None, None)

if __name__ == '__main__':
    ctrl = RawEncoderController()
    ctrl.run()
