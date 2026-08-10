#!/usr/bin/env python3
import time
import sys
import os
import select
import termios
import tty

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
    REAL_AXES = [(3, 1), (3, 2), (1, 1), (1, 2)] # 조인트 1, 2, 3, 4 순서
    JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4"]
    current_test_joint_idx = 0  # 현재 테스트 중인 조인트 인덱스 (기본값 Joint 1)
    CAN_CHANNEL = "can0"
    MONITOR_INTERVAL = 0.05 # 약 20Hz
    voltage_step = 500      # +/- 키 입력 시 변화할 전압량 (mV)
    CURRENT_SAFETY_LIMIT_MA = 1000  # 1A 이상 흐르면 즉시 차단 (모터 보호용)
    STALL_CHECK_THRESHOLD_MV = 2000 # 2V 이상인데 안 움직이면 Stall로 간주
    # -------------------------------------------------------------------------

    protocol = Cia402Protocol()
    node_axis_to_idx = {key: i for i, key in enumerate(REAL_AXES)}
    
    print("=" * 75)
    print(f" 🔍 Welcon 드라이버 실시간 교차 진단 도구 (4조인트)")
    print(" - Statusword: 드라이버의 현재 상태 (Fault 여부 등)")
    print(" - Error Reg: 발생한 에러의 종류")
    print(" - Current: 실제 흐르는 전류 (단선/부하 판단용)")
    print(" - Pos: 모든 조인트의 현재 엔코더 값")
    print("-" * 75)
    print(" [조작 방법]")
    print("  - [1], [2], [3], [4]: 테스트할 조인트 변경")
    print("  - [+]: 전압 증가 | [-]: 전압 감소 | [Space]: 즉시 정지(0mV)")
    print("  - [r]: 현재 조인트 에러 리셋 (Fault Reset)")
    print("  - [Ctrl + C]: 종료 및 전압 차단")
    print("=" * 75)
    time.sleep(1.0)

    # 터미널 설정 저장
    old_settings = termios.tcgetattr(sys.stdin)

    with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
        bus.send(protocol.make_nmt_start(0))
        time.sleep(0.1)

        # [🔍] 모든 노드/축 활성화 시도
        print(f" ▶ 모든 조인트({JOINT_NAMES}) 활성화 시도 중...")
        for node_id, axis in REAL_AXES:
            bus.send(protocol.make_axis_mode_sdo(node_id, axis, -11)) # Voltage Mode
            time.sleep(0.02)
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                bus.send(protocol.make_axis_controlword_sdo(node_id, axis, ctrl))
                time.sleep(0.02)
        
        print(" ▶ 모니터링 및 진단 시작...")

        # 데이터 저장소
        encoder_values = [0] * 4
        last_encoder_values = [0] * 4
        current_status = 0
        current_error_reg = 0
        actual_currents = [0] * 4
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
                    elif key in ['1', '2', '3', '4']:
                        current_test_joint_idx = int(key) - 1
                        test_voltage = 0 # 조인트 변경 시 안전을 위해 전압 초기화
                    elif key == 'r':
                        # 현재 테스트 중인 노드/축에 Fault Reset 명령 송신
                        node_id, axis = REAL_AXES[current_test_joint_idx]
                        print(f"\n [Reset] {JOINT_NAMES[current_test_joint_idx]} (Node {node_id}-Ax{axis}) 에러 리셋 시도...")
                        bus.send(protocol.make_axis_controlword_sdo(node_id, axis, Cia402Controlword.FAULT_RESET))
                        time.sleep(0.1)
                    
                    # 안전을 위해 최대 전압 9V 제한
                    test_voltage = max(-9000, min(9000, test_voltage))

                # 1. 각 노드에 위치 및 전류 요청
                for node_id, axis in REAL_AXES:
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(0x6064 if axis == 1 else 0x6864)))
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(0x6078 if axis == 1 else 0x6878)))
                
                # 현재 테스트 중인 축 상세 상태 요청
                t_node, t_axis = REAL_AXES[current_test_joint_idx]
                bus.send(protocol.make_sdo_read(t_node, Cia402Object(0x6041 if t_axis == 1 else 0x6841))) # Statusword
                bus.send(protocol.make_sdo_read(t_node, Cia402Object(0x603F if t_axis == 1 else 0x683F))) # Error Register

                # 선택된 노드/축에만 전압 인가
                bus.send(protocol.make_q_axis_voltage_mv_sdo(t_node, int(test_voltage), t_axis))

                # 2. CAN 버스에서 응답 수집 (약 30ms 동안)
                timeout_end = time.monotonic() + 0.03
                while time.monotonic() < timeout_end:
                    last_encoder_values = encoder_values.copy()
                    frame = bus.recv(timeout=0.001)
                    if frame is None: continue

                    if (frame.can_id & 0x780) == 0x580:
                        node_id = frame.can_id & 0x7F
                        sdo_res = protocol.parse_sdo_response(frame)
                        if sdo_res and sdo_res.value is not None:
                            axis = 1 if sdo_res.index in [0x6064, 0x6078, 0x6041, 0x603F] else 2 if sdo_res.index in [0x6864, 0x6878, 0x6841, 0x683F] else None
                            if axis is not None and (node_id, axis) in node_axis_to_idx:
                                idx = node_axis_to_idx[(node_id, axis)]
                                val = sdo_res.value
                                
                                if sdo_res.index in [0x6064, 0x6864]:
                                    if val > 0x7FFFFFFF: val -= 0x100000000
                                    encoder_values[idx] = val
                                elif sdo_res.index in [0x6078, 0x6878]:
                                    if val > 0x7FFF: val -= 0x10000
                                    actual_currents[idx] = val
                                    
                                    # [안전 장치] 과전류 감지 시 전압 즉시 차단
                                    if idx == current_test_joint_idx and abs(val) > CURRENT_SAFETY_LIMIT_MA:
                                        test_voltage = 0
                                        bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
                                        print(f"\n 🔥 [위험] {JOINT_NAMES[idx]} 과전류 감지({val}mA)! 전압을 강제 차단했습니다.")
                                        print(" 기구부 고착이나 Stall 상태를 확인하십시오.")
                                
                                if idx == current_test_joint_idx:
                                    if sdo_res.index in [0x6041, 0x6841]:
                                        current_status = val
                                    elif sdo_res.index in [0x603F, 0x683F]:
                                        current_error_reg = val

                # [Stall 감지 로직] 전압은 들어가는데 엔코더 변화가 없는 경우
                if abs(test_voltage) > STALL_CHECK_THRESHOLD_MV and \
                   abs(actual_currents[current_test_joint_idx]) > 50:
                    if last_encoder_values[current_test_joint_idx] == encoder_values[current_test_joint_idx]:
                         sys.stdout.write(f"\n ⚠️ [STALL WARNING] {JOINT_NAMES[current_test_joint_idx]} 가 물리적으로 막혀있을 가능성이 큼! \n")
                         sys.stdout.flush()

                # 3. 화면 출력
                # Statusword 해석
                fault_status = "FAULT" if (current_status & 0x08) else "OK"
                enabled = "ON" if (current_status & 0x04) else "OFF"
                
                # 전체 조인트 위치 및 전류 출력
                pos_info = " | ".join([f"{JOINT_NAMES[i]}:{encoder_values[i]:7d}" for i in range(4)])
                curr_info = " | ".join([f"C{i+1}:{actual_currents[i]:4d}mA" for i in range(4)])
                
                # 현재 테스트 중인 축의 추정 저항값 계산 (V/I = R)
                est_res = abs(test_voltage / actual_currents[current_test_joint_idx]) if abs(actual_currents[current_test_joint_idx]) > 5 else 0
                res_str = f"{est_res:5.1f}Ω" if est_res > 0 else "High-Z"

                # 상세 진단 정보
                diag_info = f"TEST:{JOINT_NAMES[current_test_joint_idx]} | Out:{test_voltage:5d}mV | R:{res_str} | Stat:{hex(current_status)} ({enabled}/{fault_status}) | Err:{hex(current_error_reg)}"
                
                sys.stdout.write(f"\r{pos_info} | {curr_info} | {diag_info}             ")
                sys.stdout.flush()

                time.sleep(MONITOR_INTERVAL)

        except KeyboardInterrupt:
            print("\n\n⚠️ 안전을 위해 모든 전압을 차단하고 종료합니다.")
            for node_id, axis in REAL_AXES:
                bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis))
        finally:
            # 터미널 설정을 원래대로 복구
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()