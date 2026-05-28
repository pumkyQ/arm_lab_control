#!/usr/bin/env python3
import time
import sys
import os

# 형님 프로젝트 구조(kitech_v1 패키지) 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control import SocketCanBus, Cia402Protocol
from motor_control.cia402 import Cia402Object

def main():
    # -------------------------------------------------------------------------
    # [설정 구역] Fault 에러 방지 및 안정적인 카운트 측정을 위한 튜닝 파라미터
    # -------------------------------------------------------------------------
    TARGET_NODE_ID = 1  # 테스트할 모터의 노드 ID (Joint 1)
    TARGET_AXIS = 1     # 테스트할 모터의 축 번호
    CAN_CHANNEL = "can0"
    
    # 7000mV에서 드라이버가 뻗는 것을 방지하기 위해 6000mV(6V)로 안전 타협
    TEST_VOLTAGE_MV = 6000  
    # 가동 한계선에 부딪히기 전에 짧고 굵게 끊어 치기 위해 0.8초로 압축
    DRIVE_DURATION = 0.8    
    # -------------------------------------------------------------------------

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(f" 🛡️ 하드웨어 보호형 엔코더 카운트(PPR) 자동 구동 실험 (최종 디버깅 버전)")
    print(f" [대상 하드웨어] Node ID: {TARGET_NODE_ID}, Axis: {TARGET_AXIS}")
    print(f" [안전 인가 전압] {TEST_VOLTAGE_MV} mV (구동 시간: {DRIVE_DURATION}초)")
    print("=" * 60)

    with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
        
        print("\n▶ CANopen 네트워크 장치 가동 (NMT Start)")
        nmt_frame = protocol.make_nmt_start(0)
        bus.send(nmt_frame)
        time.sleep(0.1)

        # 1. 시작 전 엔코더 카운트 읽기
        print("[Step 1] 구동 전 초기 엔코더 카운트 측정 중...")
        pos_idx = 0x6064 if TARGET_AXIS == 1 else 0x6864
        pos_frame = protocol.make_sdo_read(TARGET_NODE_ID, Cia402Object(pos_idx))
        bus.send(pos_frame)
        
        start_count = None
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rx = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if rx and rx.can_id == 0x580 + TARGET_NODE_ID:
                response = protocol.parse_sdo_response(rx)
                if response and response.abort_code is None:
                    start_count = response.value
                    break
                    
        if start_count is None:
            print("❌ 에러: 모터 드라이버와 통신 실패 (전원, ID, CAN선을 확인하세요)")
            return 1
            
        if start_count > 0x7FFFFFFF:
            start_count -= 0x100000000

        print(f" ▶ 초기 엔코더 카운트(Start Count): {start_count}")
        print("-" * 60)
        print(" ⚠️ [중요 안내] 실행 전 손가락 마디를 가동 범위 한가운데(중앙)에 놓아주세요!")
        input(" 준비가 완료되었다면 엔터를 누르세요. 모터가 자동으로 미세 구동됩니다... ")
        print("-" * 60)
        
        # 2. 자동 미세 구동 시작
        print(f" ▶ 모터 구동 중... ({TEST_VOLTAGE_MV}mV 인가)")
        end_time = time.monotonic() + DRIVE_DURATION
        
        while time.monotonic() < end_time:
            tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=TEST_VOLTAGE_MV, axis=TARGET_AXIS)
            bus.send(tx_frame)
            time.sleep(0.01)
            
        # 3. 구동 즉시 중지 (안전을 위해 전압 0mV 리셋)
        print(" ▶ 구동 완료. 안전을 위해 전압을 즉시 차단(0mV)합니다.")
        zero_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=0, axis=TARGET_AXIS)
        bus.send(zero_frame)
        
        # [수정 사항 1] 드라이버가 급정거 후 전기적