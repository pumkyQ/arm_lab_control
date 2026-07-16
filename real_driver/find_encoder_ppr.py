#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import os
import select
import termios
import tty

# kitech_v1 패키지 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control import SocketCanBus, Cia402Protocol, Cia402Controlword
from motor_control.cia402 import Cia402Object

def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

def get_input(prompt):
    """표준 input()을 사용하되 KeyboardInterrupt 예외를 처리"""
    try:
        return input(prompt)
    except KeyboardInterrupt:
        print("\n[취소되었습니다.]")
        return None

def main():
    CAN_CHANNEL = "can0"
    
    # integrated_joint_control.py의 조인트 매핑 정보와 매칭
    JOINT_CONFIG = {
        '1': {'KEY': 'j1', 'NODE_ID': 3, 'AXIS': 1, 'NAME': 'J1 (Node 3, Axis 1)'},
        '2': {'KEY': 'j2', 'NODE_ID': 2, 'AXIS': 1, 'NAME': 'J2 (Node 2, Axis 1)'}, 
        '3': {'KEY': 'j3', 'NODE_ID': 1, 'AXIS': 1, 'NAME': 'J3 (Node 1, Axis 1)'},
        '4': {'KEY': 'j4', 'NODE_ID': 3, 'AXIS': 2, 'NAME': 'J4 (Node 3, Axis 2)'} 
    }

    protocol = Cia402Protocol()

    try:
        can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
        bus = can_bus_context.__enter__()
        print(f"✅ CAN 연결 성공: {CAN_CHANNEL}")
    except Exception as e:
        print(f"❌ CAN 연결 실패: {e}")
        return 1

    # CANopen 장치 활성화 (NMT Start)
    nmt_frame = protocol.make_nmt_start(0)
    bus.send(nmt_frame)
    time.sleep(0.1)

    while True:
        print("\n" + "=" * 60)
        print(" 🛠️ 조인트 전압 구동 및 ALIGN 엔코더 측정 유틸리티")
        print(" - 모터를 전압으로 제어하여 천천히 정렬 위치로 이동시킵니다.")
        print(" - 수동 역구동(Backdrive)을 하지 않으므로 감속기 보호에 안전합니다.")
        print("=" * 60)
        for key, cfg in JOINT_CONFIG.items():
            print(f" [{key}] {cfg['NAME']}")
        print(" [Q] 프로그램 종료")
        print("=" * 60)
        
        choice = get_input(" ▶ 구동할 조인트를 선택하세요: ")
        if choice is None:
            continue
        choice = choice.strip().lower()
        
        if choice == 'q':
            break
            
        if choice not in JOINT_CONFIG:
            print("❌ 올바른 선택이 아닙니다. 다시 선택해주세요.")
            continue
            
        cfg = JOINT_CONFIG[choice]
        node_id = cfg['NODE_ID']
        axis = cfg['AXIS']
        joint_name = cfg['NAME']
        
        # 전압 입력 받기
        print(f"\n[선택됨] {joint_name}")
        print(" ▶ 구동할 전압(mV)을 입력하세요. (양수: 순방향, 음수: 역방향)")
        print("   - 권장 전압: 3000 ~ 4500 mV (천천히 정밀하게 움직이는 속도)")
        print("   - 최대 안전 범위: -8000 ~ +8000 mV (0 입력 시 메뉴로 복귀)")
        
        voltage_str = get_input(" ▶ 전압 입력 (mV): ")
        if voltage_str is None:
            continue
        voltage_str = voltage_str.strip()
        
        try:
            voltage = int(voltage_str)
        except ValueError:
            print("❌ 올바른 정수 값이 아닙니다.")
            continue
            
        if voltage == 0:
            print("▶ 동작을 취소하고 메뉴로 돌아갑니다.")
            continue
            
        if abs(voltage) > 9500:
            print("❌ 전압 값이 너무 큽니다. 하드웨어 보호를 위해 9500mV 이하로 설정해주세요.")
            continue

        # 모터 드라이버 활성화 시퀀스
        print(f"\n▶ Node {node_id} (Axis {axis}) 활성화 시도 중...")
        bus.send(protocol.make_axis_mode_sdo(node_id, axis, -11)) # Voltage Mode
        time.sleep(0.02)
        for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                     Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
            bus.send(protocol.make_axis_controlword_sdo(node_id, axis, ctrl))
            time.sleep(0.02)

        # 시작 전 기존 수신 버퍼 청소
        while bus.recv(timeout=0.005):
            pass

        print(f"\n📢 구동 준비 완료! 전압 {voltage} mV 인가 대기 중.")
        get_input(" ▶ [Enter] 키를 누르면 구동이 시작됩니다...")

        print("\n🚀 모터가 구동 중입니다!")
        print("🚨 [중요] 정지하고 싶은 위치(ALIGN)에 도달하면 아무 키나 누르세요!!")
        print("-" * 60)

        pos_idx = 0x6064 if axis == 1 else 0x6864
        pos_obj = Cia402Object(pos_idx)
        current_count = 0.0

        # 키보드 비차단 입력 모드 설정
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # 혹은 기존 키 버퍼에 남아있던 값 비우기
        while kbhit():
            os.read(sys.stdin.fileno(), 1)

        try:
            while True:
                # 1. 전압 명령 송신
                tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=node_id, voltage_mv=voltage, axis=axis)
                bus.send(tx_frame)
                
                # 2. 실시간 엔코더 카운트 요청
                bus.send(protocol.make_sdo_read(node_id, pos_obj))
                
                # 3. 응답 대기 및 갱신
                deadline = time.monotonic() + 0.02
                while time.monotonic() < deadline:
                    try:
                        rx = bus.recv(timeout=0.002)
                        if rx and rx.can_id == 0x580 + node_id:
                            sdo_res = protocol.parse_sdo_response(rx)
                            if sdo_res and sdo_res.index == pos_idx and sdo_res.value is not None:
                                val = sdo_res.value
                                if val > 0x7FFFFFFF:
                                    val -= 0x100000000
                                current_count = val
                                break
                    except Exception:
                        pass
                
                # 4. 화면 출력
                sys.stdout.write(f"\r 🔄 [이동 중] 현재 엔코더 카운트: {current_count:8.0f} | (정지하려면 아무 키나 입력)")
                sys.stdout.flush()

                # 5. 키 입력 감지 시 루프 탈출
                if kbhit():
                    os.read(sys.stdin.fileno(), 1) # 입력 키 소모
                    break

                time.sleep(0.01)

        except Exception as e:
            print(f"\n❌ 루프 에러 발생: {e}")
        finally:
            # 전압 즉시 0mV 차단 (안전을 위해 여러 번 반복 송신)
            for _ in range(5):
                zero_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=node_id, voltage_mv=0, axis=axis)
                bus.send(zero_frame)
                time.sleep(0.01)
                
            # 터미널 복구
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        print("\n\n🛑 정지 명령 송신 완료 (전압 0mV 리셋)")
        print(" ▶ 모터 관성 정지 대기 중 (0.5초)...")
        time.sleep(0.5)

        # 버퍼 비우기
        while bus.recv(timeout=0.005):
            pass

        # 최종 카운트 최종 확인용 읽기
        bus.send(protocol.make_sdo_read(node_id, pos_obj))
        final_count = None
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rx = bus.recv(timeout=0.01)
            if rx and rx.can_id == 0x580 + node_id:
                sdo_res = protocol.parse_sdo_response(rx)
                if sdo_res and sdo_res.index == pos_idx and sdo_res.value is not None:
                    val = sdo_res.value
                    if val > 0x7FFFFFFF:
                        val -= 0x100000000
                    final_count = val
                    break

        if final_count is None:
            final_count = current_count # 실패 시 루프 내 마지막 값 대체

        print("=" * 60)
        print(f" 🎉 [측정 결과] {joint_name}")
        print(f" ➡️  최종 정지 엔코더 카운트: {final_count:.1f}")
        print("=" * 60)
        get_input("\n 엔터를 누르면 메인 메뉴로 돌아갑니다...")

    # 안전하게 리소스 종료
    can_bus_context.__exit__(None, None, None)
    print("\n👋 프로그램을 안전하게 종료합니다.")

if __name__ == '__main__':
    main()