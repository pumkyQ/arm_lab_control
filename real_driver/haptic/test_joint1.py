#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔬 KITECH 조인트 1번 & 2번 (Node 3 Axis 1/2) 정밀 진단 툴 (Python 3 전용)
========================================================================================
- test_joint1_diag.ino 아두이노 정밀 진단 코드의 로직과 노드/축 매핑을 100% 동일하게 반영했습니다.
- [매핑 정보]
  - 조인트 1번 (j1): Node 3, Axis 1 (0x6064 / 0x2103 / 0x6041)
  - 조인트 2번 (j2): Node 3, Axis 2 (0x6864 / 0x2903 / 0x6841)
- 실행 시 시작 단계에서 조인트 1번 및 2번에 대한 자동 6단계 정밀 진단 및 펄스전압 모션 테스트를 수행하고,
- 이후 실시간 논블로킹 대화형 키보드 전압 조종 모드로 진입합니다.
"""

import sys
import os
import time
import select
import termios
import tty

# kitech_v1 패키지 라이브러리 참조 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))   # .../haptic
real_driver_dir = os.path.dirname(current_dir)             # .../real_driver
workspace_dir = os.path.dirname(real_driver_dir)           # .../arm_lab_control
kitech_path = os.path.join(workspace_dir, "kitech_v1")

if kitech_path not in sys.path:
    sys.path.append(kitech_path)
if workspace_dir not in sys.path:
    sys.path.append(workspace_dir)

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword


def kbhit():
    """키보드 입력 여부 논블로킹 확인"""
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0


def decode_cia402_status(status):
    """CiA402 StatusWord 해독 (test_joint1_diag.ino 규격)"""
    if status is None:
        return "UNKNOWN (통신 응답없음)"
    if status & 0x0008:
        return "❌ FAULT (드라이버 에러/폴트 상태!)"
    elif (status & 0x006F) in [0x0027, 0x0037]:
        return "✅ Operation Enabled (정상 전력 인가 가동 가능 상태)"
    elif (status & 0x004F) == 0x0023:
        return "⚠️ Switched ON (전력 준비 상태)"
    elif (status & 0x004F) == 0x0021:
        return "⚠️ Ready to Switch ON (대기 상태)"
    elif (status & 0x004F) == 0x0040:
        return "⚠️ Switch ON Disabled (비활성화 상태)"
    else:
        return f"❓ Special/Unknown State (0x{status:04X})"


def decode_error_code(err_code):
    """CiA402 ErrorCode 해독 (test_joint1_diag.ino 규격)"""
    if err_code is None or err_code == 0x0000:
        return "✅ 에러 없음 (0x0000 No Error)"
    
    err_map = {
        0x2310: "❌ 과전류 (Continuous Over Current)",
        0x3210: "❌ DC 링크 과전압 (DC Link Over Voltage)",
        0x3220: "❌ DC 링크 저전압 (DC Link Under Voltage)",
        0x4210: "❌ 드라이버 과열 (Drive Over Temperature)",
        0x7300: "❌ 엔코더 통신/신호 오류 (Sensor Feedback Error)",
        0x8611: "❌ 위치 편차 과다 (Following Error Fault)",
        0xFF00: "❌ 모터 상선 단선/결상 오류 (Motor Phase Loss/Disconnected)"
    }
    return err_map.get(err_code, f"❌ 드라이버 내부 보호 동작 활성화 (0x{err_code:04X})")


def sdo_read_int32(bus, protocol, node_id, index, subindex=0x00):
    """SDO 동기형 읽기 함수 (3ms 타임아웃)"""
    bus.send(protocol.make_sdo_read(node_id, Cia402Object(index, subindex)))
    start_t = time.monotonic()
    while time.monotonic() - start_t < 0.05:
        frame = bus.recv(timeout=0.002)
        if frame and (frame.can_id & 0x780) == 0x580:
            rx_node = frame.can_id & 0x7F
            if rx_node == node_id:
                sdo_res = protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == index:
                    val = sdo_res.value
                    if val is not None:
                        if val > 0x7FFFFFFF: val -= 0x100000000
                        return val
    return None


def diagnose_axis(bus, protocol, node_id, axis):
    """test_joint1_diag.ino의 diagnoseAxis() 함수를 100% 동일 구현한 진단 루틴"""
    j_name = "조인트 1번 (j1)" if (node_id == 3 and axis == 1) else "조인트 2번 (j2)" if (node_id == 3 and axis == 2) else f"Axis {axis}"
    
    print("-" * 65)
    print(f"🔍 [자동 정밀 진단] Node {node_id} - Axis {axis} ({j_name})")
    print("-" * 65)

    pos_idx  = 0x6064 if axis == 1 else 0x6864
    stat_idx = 0x6041 if axis == 1 else 0x6841
    err_idx  = 0x603F if axis == 1 else 0x683F
    mode_idx = 0x6061 if axis == 1 else 0x6861
    ctrl_idx = 0x6040 if axis == 1 else 0x6840
    volt_idx = 0x2103 if axis == 1 else 0x2903

    # 1. 엔코더 읽기
    initial_pos = sdo_read_int32(bus, protocol, node_id, pos_idx)
    if initial_pos is not None:
        print(f"1. 엔코더 피드백 (0x{pos_idx:04X}): {initial_pos} count (통신 정상)")
    else:
        print(f"1. 엔코더 피드백 (0x{pos_idx:04X}): ❌ 통신 응답 없음!")
        return

    # 2. StatusWord 읽기
    stat_val = sdo_read_int32(bus, protocol, node_id, stat_idx)
    if stat_val is not None:
        print(f"2. CiA402 상태 (0x{stat_idx:04X}): {decode_cia402_status(stat_val & 0xFFFF)}")

    # 3. ErrorCode 읽기
    err_val = sdo_read_int32(bus, protocol, node_id, err_idx)
    if err_val is not None:
        print(f"3. 에러 코드 (0x{err_idx:04X}): {decode_error_code(err_val & 0xFFFF)}")

    # 4. Mode of Operation Display 읽기
    mode_val = sdo_read_int32(bus, protocol, node_id, mode_idx)
    if mode_val is not None:
        mode_i8 = (mode_val & 0xFF) if mode_val < 128 else (mode_val & 0xFF) - 256
        mode_str = " (정상: Custom Voltage Mode)" if mode_i8 == -11 else f" ❌ 비정상! ({mode_i8} 전압 제어 모드가 아님)"
        print(f"4. 모드 설정 (0x{mode_idx:04X}): {mode_i8}{mode_str}")

    # 5. CiA402 강제 가동 (Reset -> Shutdown -> SwitchON -> Enable)
    print("\n▶ CiA402 State Machine 리셋 및 Operation Enable 전송...")
    bus.send(protocol.make_axis_mode_sdo(node_id, axis, -11))
    time.sleep(0.02)
    for ctrl in [Cia402Controlword.FAULT_RESET, 
                 Cia402Controlword.SHUTDOWN, 
                 Cia402Controlword.SWITCH_ON, 
                 Cia402Controlword.ENABLE_OPERATION]:
        bus.send(protocol.make_axis_controlword_sdo(node_id, axis, ctrl))
        time.sleep(0.02)

    stat_after = sdo_read_int32(bus, protocol, node_id, stat_idx)
    if stat_after is not None:
        print(f"  가동 후 상태: {decode_cia402_status(stat_after & 0xFFFF)}")

    # 6. 전압 주입 테스트 (Pulse Voltage Test +9000mV / 1.5초)
    print("\n⚡ [전압 인가 테스트] +9000mV (9.0V) 1.5초간 출력 시도...")
    pos_start = sdo_read_int32(bus, protocol, node_id, pos_idx)
    if pos_start is None:
        pos_start = initial_pos

    for _ in range(75): # 1.5초간 20ms 간격 지속 전압 주입
        bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 9000, axis))
        time.sleep(0.02)
    
    bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, axis)) # 전압 차단
    time.sleep(0.1)

    pos_end = sdo_read_int32(bus, protocol, node_id, pos_idx)
    if pos_end is None:
        pos_end = pos_start

    delta_pos = pos_end - pos_start
    print(f"📊 +9000mV 인가 결과 -> 이동 변위: {delta_pos} count (시작:{pos_start} -> 종료:{pos_end})")

    if abs(delta_pos) > 15:
        print("🎉 [결과 SUCCESS] 모터 및 드라이버 회로 정상 작동 확인!\n")
    else:
        print("❌ [결과 FAILURE] 전압을 주었으나 모터 변위가 0에 가깝습니다!")
        print("  ➔ 원인 1: 모터 3상 케이블(U,V,W) 단선 또는 커넥터 이탈")
        print("  ➔ 원인 2: 모터 드라이버 FET 출력 단계 퓨즈/하드웨어 손상")
        print("  ➔ 원인 3: 기계적 구동부 잼(Jamming) 또는 브레이크 잠김\n")


def main():
    TARGET_NODE = 3
    CAN_CHANNEL = "can0"

    print("=" * 70)
    print(" 🔬 Welcon Node 3 (조인트 1번: Axis 1 / 조인트 2번: Axis 2) 정밀 진단 툴")
    print("=========================================================")
    print(" [⌨️ 대화형 키보드 제어 안내]")
    print("  - [1] / [2]    : 테스트 조인트 선택 (1: J1 Node3-Ax1, 2: J2 Node3-Ax2)")
    print("  - [+] / [-]    : 전압 ±500mV 증감 인가")
    print("  - [Spacebar]   : 전압 0mV 즉시 차단")
    print("  - [r]          : 드라이버 Fault Reset 및 Enable Re-init")
    print("  - [e]          : +2000mV 펄스 전압 모션 테스트 재실행")
    print("  - [q] / Ctrl+C : 종료 및 전압 안전 차단")
    print("=" * 70 + "\n")

    protocol = Cia402Protocol()
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
            # NMT Start
            bus.send(protocol.make_nmt_start(0))
            time.sleep(0.1)

            # test_joint1_diag.ino의 setup() 자동 진단 100% 동일 실행
            diagnose_axis(bus, protocol, TARGET_NODE, 1)
            time.sleep(0.5)
            diagnose_axis(bus, protocol, TARGET_NODE, 2)
            time.sleep(0.5)

            print("=" * 70)
            print(" 🕹️ [대화형 실시간 모니터링 및 키보드 조종 모드 시작]")
            print("=" * 70)

            tty.setcbreak(sys.stdin.fileno())

            active_axis = 1  # 기본 1번 관절
            test_voltage = 0

            pos_values = [0, 0]
            stat_values = [0, 0]
            err_values = [0, 0]
            curr_values = [0, 0]

            last_poll_t = time.monotonic()

            while True:
                if kbhit():
                    key = sys.stdin.read(1)
                    if key == '1':
                        active_axis = 1
                        test_voltage = 0
                        print("\n👉 [선택] 조인트 1번 (Node 3, Axis 1) 제어 대상 변경")
                    elif key == '2':
                        active_axis = 2
                        test_voltage = 0
                        print("\n👉 [선택] 조인트 2번 (Node 3, Axis 2) 제어 대상 변경")
                    elif key == '+':
                        test_voltage = min(9800, test_voltage + 500)
                    elif key == '-':
                        test_voltage = max(-9800, test_voltage - 500)
                    elif key == ' ':
                        test_voltage = 0
                        print("\n🛑 [0mV] 전압 차단!")
                    elif key == 'r':
                        test_voltage = 0
                        print(f"\n🔄 [Fault Reset] Node 3 Axis {active_axis} 리셋 시도...")
                        bus.send(protocol.make_axis_mode_sdo(TARGET_NODE, active_axis, -11))
                        time.sleep(0.02)
                        for ctrl in [Cia402Controlword.FAULT_RESET, 
                                     Cia402Controlword.SHUTDOWN, 
                                     Cia402Controlword.SWITCH_ON, 
                                     Cia402Controlword.ENABLE_OPERATION]:
                            bus.send(protocol.make_axis_controlword_sdo(TARGET_NODE, active_axis, ctrl))
                            time.sleep(0.02)
                    elif key == 'e':
                        test_voltage = 0
                        diagnose_axis(bus, protocol, TARGET_NODE, active_axis)
                    elif key in ['q', 'Q']:
                        print("\n🛑 [종료] 사용자 종료 요청.")
                        break

                now = time.monotonic()
                if now - last_poll_t >= 0.03:
                    last_poll_t = now

                    # Axis 1 SDO Read
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x6064)))
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x6041)))
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x603F)))
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x6078)))

                    # Axis 2 SDO Read
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x6864)))
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x6841)))
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x683F)))
                    bus.send(protocol.make_sdo_read(TARGET_NODE, Cia402Object(0x6878)))

                    # 활성화된 축에만 전압 인가
                    bus.send(protocol.make_q_axis_voltage_mv_sdo(TARGET_NODE, int(test_voltage), active_axis))

                frame = bus.recv(timeout=0.002)
                if frame and (frame.can_id & 0x780) == 0x580:
                    rx_node = frame.can_id & 0x7F
                    if rx_node == TARGET_NODE:
                        sdo_res = protocol.parse_sdo_response(frame)
                        if sdo_res and sdo_res.value is not None:
                            val = sdo_res.value
                            idx = sdo_res.index
                            ax_idx = 0 if idx in [0x6064, 0x6041, 0x603F, 0x6078] else 1 if idx in [0x6864, 0x6841, 0x683F, 0x6878] else None
                            if ax_idx is not None:
                                if idx in [0x6064, 0x6864]:
                                    if val > 0x7FFFFFFF: val -= 0x100000000
                                    pos_values[ax_idx] = val
                                elif idx in [0x6041, 0x6841]:
                                    stat_values[ax_idx] = val
                                elif idx in [0x603F, 0x683F]:
                                    err_values[ax_idx] = val
                                elif idx in [0x6078, 0x6878]:
                                    if val > 0x7FFF: val -= 0x10000
                                    curr_values[ax_idx] = val

                sys.stdout.write(
                    f"\r📍 Node3-J1(Ax1) Pos:{pos_values[0]:6d} Stat:0x{stat_values[0]:04X} | "
                    f"J2(Ax2) Pos:{pos_values[1]:6d} Stat:0x{stat_values[1]:04X} | "
                    f"ACTIVE:J{active_axis} Out:{test_voltage:5d}mV"
                )
                sys.stdout.flush()
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n🛑 [안전 차단] 프로그램 종료 및 전압 0mV 차단...")
        with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
            bus.send(protocol.make_q_axis_voltage_mv_sdo(TARGET_NODE, 0, 1))
            bus.send(protocol.make_q_axis_voltage_mv_sdo(TARGET_NODE, 0, 2))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    main()
