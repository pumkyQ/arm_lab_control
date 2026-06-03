#!/usr/bin/env python3
import time
import sys
import os
import select
import termios
import tty
import numpy as np

# kitech_v1 패키지 라이브러리 참조 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

def kbhit():
    """키보드 입력 여부 확인 (Non-blocking)"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

def main():
    # -------------------------------------------------------------------------
    # [⚙️ 설정 구역] 진단 대상 설정
    # -------------------------------------------------------------------------
    TARGET_NODES = [1, 2, 3]
    current_test_node = 2  # 현재 테스트 중인 노드 (기본값 Node 2)
    CAN_CHANNEL = "can0"
    MONITOR_INTERVAL = 0.05 # 약 20Hz
    voltage_step = 500      # +/- 키 입력 시 변화할 전압량 (mV)
    CURRENT_SAFETY_LIMIT_MA = 1000  # 1A 이상 흐르면 즉시 차단 (소형 모터 보호용)
    STALL_CHECK_THRESHOLD_MV = 2000 # 2V 이상인데 안 움직이면 Stall로 간주
    # -------------------------------------------------------------------------

    protocol = Cia402Protocol()
    
    print("=" * 70)
    print(f" 🔍 Welcon 드라이버 실시간 교차 진단 도구")
    print(" - Statusword: 드라이버의 현재 상태 (Fault 여부 등)")
    print(" - Error Reg: 발생한 에러의 종류")
    print(" - Current: 실제 흐르는 전류 (단선/부하 판단용)")
    print(" - Pos: 모든 노드의 현재 엔코더 값")
    print("-" * 70)
    print(" [조작 방법]")
    print("  - [1], [2], [3]: 테스트할 노드 변경")
    print("  - [+]: 전압 증가 | [-]: 전압 감소 | [Space]: 즉시 정지(0mV)")
    print("  - [r]: 현재 노드 에러 리셋 (Fault Reset)")
    print("  - [Ctrl + C]: 종료 및 전압 차단")
    print("-" * 70)
    time.sleep(1.0)

    # 터미널 설정 저장 및 Non-blocking/No-echo 설정
    old_settings = termios.tcgetattr(sys.stdin)

    with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
        bus.send(protocol.make_nmt_start(0))
        time.sleep(0.1)

        # [🔍] 모든 노드 활성화 시도
        print(f" ▶ 모든 노드({TARGET_NODES}) 활성화 시도 중...")
        for node_id in TARGET_NODES:
            bus.send(protocol.make_axis_mode_sdo(node_id, 1, -11)) # Voltage Mode
            time.sleep(0.02)
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                bus.send(protocol.make_axis_controlword_sdo(node_id, 1, ctrl))
                time.sleep(0.02)
        
        print(" ▶ 모니터링 시작... (손으로 조인트를 움직여 N2 값이 변하는지 보세요)")

        # 데이터 저장소
        encoder_values = {1: 0, 2: 0, 3: 0}
        last_encoder_values = {1: 0, 2: 0, 3: 0}
        current_status = 0
        current_error_reg = 0
        actual_currents = {1: 0, 2: 0, 3: 0}
        test_voltage = 0
        
        try:
            # 엔터 없이 키 입력을 받기 위한 설정
            tty.setcbreak(sys.stdin.fileno())

            while True:
                # 1. 키보드 입력 처리
                if kbhit():
                    key = sys.stdin.read(1)
                    if key == '+':
                        test_voltage += voltage_step
                    elif key == '-':
                        test_voltage -= voltage_step
                    elif key == ' ':
                        test_voltage = 0
                    elif key in ['1', '2', '3']:
                        current_test_node = int(key)
                        test_voltage = 0 # 노드 변경 시 안전을 위해 전압 초기화
                    elif key == 'r':
                        # 현재 테스트 중인 노드에 Fault Reset 명령 송신
                        print(f"\n [Reset] Node {current_test_node} 에러 리셋 시도...")
                        bus.send(protocol.make_axis_controlword_sdo(current_test_node, 1, Cia402Controlword.FAULT_RESET))
                        time.sleep(0.1)
                        # 줄바꿈 없이 위쪽 라인에 정보를 갱신하기 위해 로그 출력 방식 변경 필요
                    
                    # 안전을 위해 최대 전압 9V 제한
                    test_voltage = max(-9000, min(9000, test_voltage))

                # 1. 각 노드에 위치 및 전류 요청, 현재 테스트 노드 상세 상태 요청
                for node_id in TARGET_NODES:
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(0x6064)))
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(0x6078)))
                
                bus.send(protocol.make_sdo_read(current_test_node, Cia402Object(0x6041))) # Statusword
                bus.send(protocol.make_sdo_read(current_test_node, Cia402Object(0x603F))) # Error Register

                # 선택된 노드에만 전압 인가
                bus.send(protocol.make_q_axis_voltage_mv_sdo(current_test_node, int(test_voltage), 1))

                # 2. CAN 버스에서 응답 수집
                timeout_end = time.monotonic() + 0.03
                while time.monotonic() < timeout_end:
                    last_encoder_values = encoder_values.copy()
                    frame = bus.recv(timeout=0.001)
                    if frame is None: continue

                    if (frame.can_id & 0x780) == 0x580:
                        node_id = frame.can_id & 0x7F
                        if node_id in TARGET_NODES:
                            sdo_res = protocol.parse_sdo_response(frame)
                            if sdo_res and sdo_res.value is not None:
                                val = sdo_res.value
                                if val > 0x7FFFFFFF: val -= 0x100000000
                                
                                if sdo_res.index == 0x6064:
                                    encoder_values[node_id] = val
                                elif sdo_res.index == 0x6078:
                                    # 16비트 부호 있는 정수(Integer16) 처리
                                    if val > 0x7FFF: val -= 0x10000
                                    actual_currents[node_id] = val
                                    
                                    # [안전 장치] 과전류 감지 시 전압 즉시 차단
                                    if abs(val) > CURRENT_SAFETY_LIMIT_MA:
                                        test_voltage = 0
                                        bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, 1))
                                        print(f"\n 🔥 [위험] Node {node_id} 과전류 감지({val}mA)! 전압을 강제 차단했습니다.")
                                        print(" 기구부 고착이나 Stall 상태를 확인하십시오.")
                                
                                if node_id == current_test_node:
                                    if sdo_res.index == 0x6041:
                                        current_status = val
                                    elif sdo_res.index == 0x603F:
                                        current_error_reg = val

                # [Stall 감지 로직] 전압은 들어가는데 엔코더 변화가 없는 경우
                if abs(test_voltage) > STALL_CHECK_THRESHOLD_MV and \
                   abs(actual_currents[current_test_node]) > 50:
                    if last_encoder_values[current_test_node] == encoder_values[current_test_node]:
                         sys.stdout.write(f"\n ⚠️ [STALL WARNING] Node {current_test_node} 가 물리적으로 막혀있을 가능성이 큼! \n")
                         sys.stdout.flush()

                # 3. 화면 출력
                # Statusword 해석
                fault_status = "FAULT" if (current_status & 0x08) else "OK"
                enabled = "ON" if (current_status & 0x04) else "OFF"
                
                # 전체 조인트 위치 출력
                pos_info = " | ".join([f"N{node}: {encoder_values[node]:7d}" for node in TARGET_NODES])
                curr_info = " | ".join([f"C{node}: {actual_currents[node]:4d}mA" for node in TARGET_NODES])
                
                # 현재 테스트 중인 노드의 추정 저항값 계산 (V/I = R)
                # 전류가 너무 작으면 계산에서 제외 (0 나누기 방지)
                est_res = abs(test_voltage / actual_currents[current_test_node]) if abs(actual_currents[current_test_node]) > 5 else 0
                res_str = f"{est_res:5.1f}Ω" if est_res > 0 else "High-Z"

                # 상세 진단 정보 (현재 테스트 중인 노드 번호 포함)
                diag_info = f"TEST:Node {current_test_node} | Out:{test_voltage:5d}mV | R:{res_str} | Stat:{hex(current_status)} ({enabled}/{fault_status}) | Err:{hex(current_error_reg)}"
                
                sys.stdout.write(f"\r{pos_info} | {curr_info} | {diag_info}             ")
                sys.stdout.flush()

                time.sleep(MONITOR_INTERVAL)

        except KeyboardInterrupt:
            print("\n\n⚠️ 안전을 위해 모든 전압을 차단하고 종료합니다.")
            for node_id in TARGET_NODES:
                bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, 1))
        finally:
            # 터미널 설정을 원래대로 복구
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()