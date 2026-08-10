#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import os
import numpy as np
import select
import termios
import tty

# 프로젝트 구조(kitech_v1 패키지) 라이브러리 참조 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

def kbhit():
    """Non-blocking keyboard input check"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

def main():
    # -------------------------------------------------------------------------
    # [⚙️ 설정 구역] 모니터링 및 제어 대상 노드 및 축 설정 (조인트 1, 2, 3, 4)
    # -------------------------------------------------------------------------
    REAL_AXES = [(3, 1), (3, 2), (1, 1), (1, 2)]
    JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4"]
    CAN_CHANNEL = "can0"
    
    # [⚙️ 제어 게인 및 불감대 세팅] (finger_bend_sequence.py 파라미터 적용)
    GAIN_CONFIG = {
        'Joint1': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 800.0},
        'Joint2': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0},
        'Joint3': {'Kp': 450.0, 'Kd': 15.0, 'Ki': 1.5, 'Ki_limit': 800.0, 'DEADZONE_DEG': 1.5, 'FRICT_COMP': 1200.0}, # 조인트3 전용 보상
        'Joint4': {'Kp': 350.0, 'Kd': 15.0, 'Ki': 0.5, 'Ki_limit': 500.0, 'DEADZONE_DEG': 3.0, 'FRICT_COMP': 1000.0}
    }
    # -------------------------------------------------------------------------

    protocol = Cia402Protocol()
    node_axis_to_idx = {key: i for i, key in enumerate(REAL_AXES)}
    
    print("=" * 75)
    print(" 🎯 실시간 절대 엔코더 값 제어 & 모니터링 툴 (4조인트)")
    print(" - 'j1' ~ 'j4' + Enter 입력 후 숫자를 타이핑해 개별 조인트를 구동합니다.")
    print(" - 'q' + Enter 또는 Ctrl + C 입력 시 전압이 즉시 차단되고 안전 종료됩니다.")
    print("=" * 75)

    # 터미널 백업 및 cbreak 모드 전환 (비차단 실시간 키입력 수집용)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
            # 1. 하드웨어 드라이버 활성화 시퀀스 (sdos mode -11 & enable operation)
            print("\n▶ 하드웨어 드라이버 활성화 중...")
            bus.send(protocol.make_nmt_start(0))
            time.sleep(0.05)
            for node_id, axis in REAL_AXES:
                bus.send(protocol.make_axis_mode_sdo(node_id, axis, -11))
                time.sleep(0.02)
                for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                             Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                    bus.send(protocol.make_axis_controlword_sdo(node_id, axis, ctrl))
                    time.sleep(0.02)
            print("✅ 하드웨어 준비 완료. 모니터링 및 실시간 제어를 시작합니다.\n")

            encoder_values = [0.0] * 4
            target_values = [0.0] * 4
            velocities = [0.0] * 4
            filtered_velocities_old = [0.0] * 4
            status_words = [0] * 4
            error_integrals = [0.0] * 4
            
            LPF_ALPHA = 0.25
            first_sync = [True] * 4
            input_buffer = ""
            last_loop_time = time.monotonic()

            while True:
                # ----------------------------------------------------------------------
                # ⌨️ 터미널 비차단식 키보드 입력 인터페이스 핸들러
                # ----------------------------------------------------------------------
                while kbhit():
                    char = os.read(sys.stdin.fileno(), 1).decode()
                    if char in ['\r', '\n']:
                        cmd = input_buffer.strip().lower()
                        input_buffer = ""
                        
                        if cmd in ['j1', 'j2', 'j3', 'j4']:
                            # 터미널 입출력 복구
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                            idx = int(cmd[1]) - 1
                            print(f"\n💬 [{cmd.upper()} 타겟 입력] 이동할 절대 엔코더 값을 입력하세요 (현재: {int(encoder_values[idx])}): ", end="", flush=True)
                            try:
                                val_str = sys.stdin.readline().strip()
                                target_val = float(val_str)
                                target_values[idx] = target_val
                                print(f"🎯 {cmd.upper()} 목표 엔코더 값이 {target_val}로 업데이트되었습니다.")
                            except ValueError:
                                print("⚠️ 숫자가 올바르지 않습니다. 설정을 취소합니다.")
                            # 다시 비차단 입력 모드로 변경
                            tty.setcbreak(sys.stdin.fileno())
                        elif cmd == 'q':
                            raise KeyboardInterrupt
                    elif char in ['\x08', '\x7f']:
                        if len(input_buffer) > 0:
                            input_buffer = input_buffer[:-1]
                    else:
                        input_buffer += char

                # ----------------------------------------------------------------------
                # 📡 CAN 데이터 비동기 SDO 요청 및 수신
                # ----------------------------------------------------------------------
                # 각 축에 현재 위치(0x6064/0x6864), 속도(0x606c/0x686c), 상태(0x6041/0x6841) 조회 SDO 송신
                for node_id, axis in REAL_AXES:
                    pos_idx = 0x6064 if axis == 1 else 0x6864
                    vel_idx = 0x606c if axis == 1 else 0x686c
                    stat_idx = 0x6041 if axis == 1 else 0x6841
                    
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(pos_idx)))
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(vel_idx)))
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(stat_idx)))

                # 15ms 대기하며 수신 버퍼 비우기
                timeout_end = time.monotonic() + 0.015
                while time.monotonic() < timeout_end:
                    frame = bus.recv(timeout=0.001)
                    if frame is None:
                        continue

                    # SDO 응답인지 체크 (0x580 ~ 0x5FF)
                    if (frame.can_id & 0x780) == 0x580:
                        node_id = frame.can_id - 0x580
                        sdo_res = protocol.parse_sdo_response(frame)
                        if sdo_res and sdo_res.value is not None:
                            val = sdo_res.value
                            if val > 0x7FFFFFFF:
                                val -= 0x100000000
                            
                            # 인덱스 검사하여 축 식별
                            idx_hex = sdo_res.index & 0xFF00
                            axis = 1 if idx_hex == 0x6000 else 2 if idx_hex == 0x6800 else None
                            
                            if axis is not None and (node_id, axis) in node_axis_to_idx:
                                idx = node_axis_to_idx[(node_id, axis)]
                                sub_idx = sdo_res.index & 0x00FF
                                if sub_idx == 0x64:    # position
                                    encoder_values[idx] = float(val)
                                elif sub_idx == 0x6c:  # velocity
                                    velocities[idx] = float(val)
                                elif sub_idx == 0x41:  # statusword
                                    status_words[idx] = val

                # ----------------------------------------------------------------------
                # 🎯 4축 병렬 PI-D 제어 전압 명령 연산 및 전송
                # ----------------------------------------------------------------------
                dt = time.monotonic() - last_loop_time
                last_loop_time = time.monotonic()
                if dt <= 0:
                    dt = 0.02

                for i, name in enumerate(JOINT_NAMES):
                    node_id, axis = REAL_AXES[i]
                    gc = GAIN_CONFIG[name]
                    
                    # 최초 1회 실측 위치로 목표값 정렬(Jerk 방지)
                    if first_sync[i] and encoder_values[i] != 0.0:
                        target_values[i] = encoder_values[i]
                        first_sync[i] = False
                        
                    # 드라이버 FAULT 복구 시퀀스
                    if (status_words[i] & 0x08):
                        bus.send(protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.FAULT_RESET))
                        continue

                    error_count = target_values[i] - encoder_values[i]
                    
                    kp = gc['Kp']
                    kd = gc['Kd']
                    ki = gc['Ki']
                    ki_limit = gc['Ki_limit']
                    deadzone_thresh = gc['DEADZONE_DEG'] * 11.378
                    frict_comp = gc['FRICT_COMP']
                    
                    # LPF (저역통과필터) 연산 수행 (test_3node.py 방식)
                    filtered_velocity = (LPF_ALPHA * velocities[i]) + ((1.0 - LPF_ALPHA) * filtered_velocities_old[i])
                    filtered_velocities_old[i] = filtered_velocity
                    
                    # 1) 피드백 미분 제어 (v_d) 및 EMF 보상 계산 (필터링된 속도 사용)
                    v_d = -kd * filtered_velocity
                    
                    PULSES_PER_DEGREE = 11.378
                    COUNTS_PER_RADIAN = PULSES_PER_DEGREE * (180.0 / np.pi)
                    K_emf_rad = 8.632 * 406.4
                    K_emf_count = K_emf_rad / COUNTS_PER_RADIAN
                    v_emf = K_emf_count * filtered_velocity
                    
                    # 2) 정밀 불감대 제어 및 백래시 방지용 Tail 제어
                    if abs(error_count) <= deadzone_thresh:
                        error_integrals[i] = 0.0
                        active_direction = np.sign(filtered_velocity if filtered_velocity != 0.0 else error_count)
                        if active_direction != 0:
                            v_stiction_tail = (frict_comp * 0.8) * active_direction
                        else:
                            v_stiction_tail = 0.0
                        total_voltage = v_d + v_emf + v_stiction_tail
                    else:
                        v_p = kp * error_count
                        error_integrals[i] += error_count * dt
                        if ki != 0.0:
                            error_integrals[i] = max(-ki_limit/ki, min(ki_limit/ki, error_integrals[i]))
                        else:
                            error_integrals[i] = 0.0
                        v_i = ki * error_integrals[i]
                        
                        # 3) 정마찰력(Stiction) 보상
                        v_frict = np.sign(error_count) * frict_comp
                        
                        total_voltage = v_p + v_i + v_d + v_emf + v_frict

                    # 최대 인가 전압 클리핑
                    voltage_limit = 9500.0
                    clamped_voltage = max(-voltage_limit, min(voltage_limit, total_voltage))
                    
                    # 하드웨어로 Q축 전압 SDO 명령 송출
                    bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, int(clamped_voltage), axis))

                # ----------------------------------------------------------------------
                # 🖥️ 실시간 출력 및 터미널 동기화
                # ----------------------------------------------------------------------
                output_str = "\r"
                for i, name in enumerate(JOINT_NAMES):
                    output_str += f"[{name}: 현재 {int(encoder_values[i]):6d} / 목표 {int(target_values[i]):6d}]   "
                output_str += f"| 입력: {input_buffer}"
                sys.stdout.write(output_str)
                sys.stdout.flush()

                # 약 30Hz 주기로 모니터링 반복
                time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n\n🛑 [안전 종료] 전압 차단 및 모든 모터 안전 정지 명령 전송 중...")
        # 모든 모터에 전압 0V 송신하여 안전 정지
        try:
            with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as shutdown_bus:
                for node_id, axis in REAL_AXES:
                    shutdown_bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
            print("✅ 전압 인가 중단 완료. 안심하고 전원을 끄셔도 됩니다.")
        except Exception as e:
            print(f"⚠️ 정지 명령 송신 중 오류 발생: {e}")
    finally:
        # 터미널 원상 복구
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()