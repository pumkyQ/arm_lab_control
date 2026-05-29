#!/usr/bin/env python3
import time
import sys
import os
import math

# 형님 프로젝트 구조(kitech_v1 패키지) 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control import SocketCanBus, Cia402Protocol, Cia402Controlword
from motor_control.cia402 import Cia402Object

def main():
    # -------------------------------------------------------------------------
    # [⚙️ 설정 구역] 5번 조인트(Node 3) 최종 디버깅 세팅
    # -------------------------------------------------------------------------
    # 조인트 1 -> Node 1, 조인트 3 -> Node 2, 조인트 5 -> Node 3
    TARGET_NODE_ID = 3  
    TARGET_AXIS = 1     
    CAN_CHANNEL = "can0"
    
    TEST_VOLTAGE_MV = 4500  # 4.5V 인가 (눈으로 식별하기 딱 좋은 속도)
    DRIVE_DURATION = 1.5    # 1.5초 동안 구동
    # -------------------------------------------------------------------------

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(f" 🛡️ 하드웨어 보호형 엔코더 카운트(PPR) 자동 구동 실험 (통신 보강 버전)")
    print(f" [대상 하드웨어] Node ID: {TARGET_NODE_ID}, Axis: {TARGET_AXIS}")
    print(f" [안전 인가 전압] {TEST_VOLTAGE_MV} mV (구동 시간: {DRIVE_DURATION}초)")
    print("=" * 60)

    with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
        
        print("\n▶ CANopen 네트워크 장치 가동 (NMT Start)")
        nmt_frame = protocol.make_nmt_start(0)
        bus.send(nmt_frame)
        time.sleep(0.1)

        # [추가] 모터 드라이버 활성화 시퀀스 (Enable Operation)
        print(f"▶ Node {TARGET_NODE_ID} 활성화 시도 중...")
        # 0. 운전 모드 설정 (Voltage Mode: -11) - working driver와 동일하게 추가
        bus.send(protocol.make_axis_mode_sdo(TARGET_NODE_ID, TARGET_AXIS, -11))
        time.sleep(0.05)

        for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                     Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
            frame = protocol.make_axis_controlword_sdo(TARGET_NODE_ID, TARGET_AXIS, ctrl)
            bus.send(frame)
            time.sleep(0.05)

        # 1. 시작 전 엔코더 카운트 읽기
        print("[Step 1] 구동 전 초기 엔코더 카운트 측정 중...")
        pos_idx = 0x6064 if TARGET_AXIS == 1 else 0x6864

        # 🔴 [보강] 읽기 요청 전 버퍼를 비워 이전 설정 응답 패킷 제거
        while bus.recv(timeout=0.01): pass

        pos_frame = protocol.make_sdo_read(TARGET_NODE_ID, Cia402Object(pos_idx))
        bus.send(pos_frame)
        
        start_count = None
        seen_ids = set()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rx = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if rx:
                seen_ids.add(hex(rx.can_id))
            
            if rx and rx.can_id == 0x580 + TARGET_NODE_ID:
                response = protocol.parse_sdo_response(rx)
                # 🔴 [보강] 응답의 인덱스가 우리가 요청한 위치(pos_idx)인지 반드시 확인
                if response and response.index == pos_idx and response.abort_code is None:
                    start_count = response.value
                    break
                    
        if start_count is None:
            print(f"❌ 에러: Node {TARGET_NODE_ID} 로부터 응답이 없습니다.")
            if seen_ids:
                print(f"💡 현재 버스에서 관측되는 ID 목록: {sorted(list(seen_ids))}")
                print(f"   (위 목록에 0x581, 0x582 등이 있다면 TARGET_NODE_ID를 해당 번호로 바꿔보세요.)")
            else:
                print("💡 버스에 아무런 데이터가 없습니다. 전원이나 CAN 배선, 전송 속도(Baudrate)를 확인하세요.")
            return 1
            
        if start_count > 0x7FFFFFFF:
            start_count -= 0x100000000

        print(f" ▶ 초기 엔코더 카운트(Start Count): {start_count}")
        print("-" * 60)
        input(" 준비가 완료되었다면 엔터를 누르세요. 모터가 자동으로 미세 구동됩니다... ")
        print("-" * 60)
        
        # 2. 자동 미세 구동 시작
        print(f" ▶ 모터 구동 중... ({TEST_VOLTAGE_MV}mV 인가)")
        end_time = time.monotonic() + DRIVE_DURATION
        
        while time.monotonic() < end_time:
            tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=TEST_VOLTAGE_MV, axis=TARGET_AXIS)
            bus.send(tx_frame)
            time.sleep(0.01)
            
        # 3. 구동 즉시 중지 (전압 0mV 리셋)
        print(" ▶ 구동 완료. 안전을 위해 전압을 즉시 차단(0mV)합니다.")
        zero_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=0, axis=TARGET_AXIS)
        bus.send(zero_frame)
        
        # 🔴 [보강 1] 모터가 완전히 멈출 때까지 넉넉히 대기 (0.5초)
        print(" ▶ 모터 관성 정지 대기 중 (0.5초)...")
        time.sleep(0.5)
        
        # 🔴 [보강 2] 구동 중에 버퍼에 쌓인 찌꺼기 패킷 싹 다 긁어내서 버리기 (Clear Buffer)
        print(" ▶ CAN 통신 수신 버퍼 청소 중 (Flush)...")
        flush_count = 0
        while True:
            dummy_rx = bus.recv(timeout=0.005)
            if dummy_rx is None:
                break
            flush_count += 1
        print(f"   (버퍼에서 쓸모없는 찌꺼기 {flush_count}개 패킷 제거 완료)")
        
        # 4. 구동 후 최종 엔코더 카운트 재요청
        print("[Step 2] 구동 후 최종 엔코더 카운트 깨끗한 상태로 측정 중...")
        bus.send(pos_frame)
        
        end_count = None
        # 타임아웃을 1.0초로 2배 늘려서 확실하게 응답 대기
        deadline = time.monotonic() + 1.0 
        while time.monotonic() < deadline:
            rx = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if rx and rx.can_id == 0x580 + TARGET_NODE_ID:
                response = protocol.parse_sdo_response(rx)
                if response and response.index == pos_idx and response.abort_code is None:
                    end_count = response.value
                    break
                    
        if end_count is None:
            print("❌ 에러: 구동 후 최종 카운트를 읽어오지 못했습니다.")
            print("💡 팁: 혹시 정지 순간 드라이버가 뻗었을 수 있으니 파워를 껐다 켜고 다시 해보세요!")
            return 1
            
        if end_count > 0x7FFFFFFF:
            end_count -= 0x100000000
            
        print(f" ▶ 최종 엔코더 카운트(End Count): {end_count}")
        
        # 5. 정밀 기계공학 역산 연산부
        delta_count = abs(end_count - start_count)
        print("-" * 60)
        print(f" 📊 [실험 데이터 결과]")
        print(f" ▶ 회전하는 동안 변화한 총 Raw 카운트량: {delta_count} counts")
        print("-" * 60)
        
        try:
            actual_deg = float(input(" ▶ 실제 관절이 움직인 각도(도, Degree) 입력: "))
            if actual_deg <= 0:
                print("❌ 각도는 0보다 커야 계산이 가능합니다.")
                return 1
                
            actual_rad = actual_deg * (math.pi / 180.0)
            pulses_per_deg = delta_count / actual_deg
            computed_scale = delta_count / actual_rad
            
            print("=" * 60)
            print(" 🎉 [최종 팩트체크 결론]")
            print(f" ▶ 1도(Degree)당 펄스 수: {pulses_per_deg:.2f} counts/deg")
            print(f" 형님, 메인 코드의 'COUNTS_PER_RADIAN' 자리에 아래 값을 넣으시면 끝납니다!")
            print(f" ➡️  COUNTS_PER_RADIAN = {computed_scale:.1f}")
            print("=" * 60)
            
        except ValueError:
            print("❌ 올바른 숫자를 입력하지 않아 연산을 종료합니다.")

if __name__ == '__main__':
    sys.exit(main())