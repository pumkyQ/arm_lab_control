#!/usr/bin/env python3
import time
import sys
import os
import numpy as np

# 프로젝트 구조(kitech_v1 패키지) 라이브러리 참조 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

def main():
    # =========================================================================
    # [⚙️ 정밀 설정] 조인트 1번 및 5번 전용 정렬 로직 (조인트 3 제외)
    # =========================================================================
    TARGET_NODES = [1, 3]    # 조인트 1(Node 1), 조인트 5(Node 3)
    TARGET_AXIS = 1          # 1축 고정
    
    # 실측된 정렬 목표 카운트
    ALIGN_TARGET_COUNTS = [-984, -337] 
    
    # 하드웨어 상수 (740 pulses/rad)
    COUNTS_PER_RADIAN = 740.0 
    
    # 오실레이션 방지 및 정밀 정렬을 위한 튜닝 게인
    # index 0: Node 1 (J1), index 1: Node 3 (J5)
    Kp_list = [32000.0, 22000.0]         
    Ki_list = [400.0, 300.0]             
    Kd_list = [2500.0, 1800.0]           # 댐핑 대폭 강화
    stiction_offset_list = [5500.0, 2500.0] # 튐 방지를 위해 보상 전압 소폭 하향
    
    voltage_limit = 8000.0    # 하드웨어 보호 (7.5V)
    i_limit = 1200.0          # Anti-windup 제한
    ERROR_THRESH_COUNT = 2    # 오차 허용 범위 (2펄스)
    dt = 0.02                 # 50Hz 제어
    # =========================================================================

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(f" 🎯 조인트 1번 & 5번 동시 영점 정렬 스크립트")
    print(f" ▶ 대상: Node 1(J1) ➡️ {ALIGN_TARGET_COUNTS[0]}, Node 3(J5) ➡️ {ALIGN_TARGET_COUNTS[1]}")
    print(" ⚠️ 고장난 조인트 3(Node 2)은 제어 대상에서 제외되었습니다.")
    print(f" ▶ 목표 엔코더 카운트: {ALIGN_TARGET_COUNTS}")
    print("=" * 60)

    with SocketCanBus('can0', receive_timeout=0.0) as bus:
        # 1. 네트워크 및 노드 초기화
        bus.send(protocol.make_nmt_start(0))
        time.sleep(0.1)
        
        # 조인트 3(Node 2)은 혹시 모르니 전압 0V 명령을 명시적으로 한 번 보냄
        bus.send(protocol.make_q_axis_voltage_mv_sdo(2, 0, 1))
        
        for node_id in TARGET_NODES:
            # 전압 모드 설정
            bus.send(protocol.make_axis_mode_sdo(node_id, TARGET_AXIS, -11))
            time.sleep(0.05)
            # 드라이버 활성화
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                bus.send(protocol.make_axis_controlword_sdo(node_id, TARGET_AXIS, ctrl))
                time.sleep(0.03)

        print(" ✅ 하드웨어 활성화 완료. 정렬을 시작합니다.")

        target_rads = [count / COUNTS_PER_RADIAN for count in ALIGN_TARGET_COUNTS]
        error_integrals = [0.0, 0.0]
        period = 1.0 / 50.0  # 50Hz
        next_time = time.monotonic()
        
        # SDO 충돌 방지를 위한 인덱스 정의
        pos_idx = 0x6064
        vel_idx = 0x606c

        try:
            while True:
                # 2. 데이터 요청 (SDO 버퍼 관리를 위해 명시적 요청)
                for node_id in TARGET_NODES:
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(pos_idx)))
                    bus.send(protocol.make_sdo_read(node_id, Cia402Object(vel_idx)))
                
                current_counts = [None] * len(TARGET_NODES)
                current_vel_raws = [0] * len(TARGET_NODES)
                
                # 3. 데이터 수집 (SDO 응답 검증 강화)
                timeout_end = time.monotonic() + 0.012
                while time.monotonic() < timeout_end:
                    frame = bus.recv(timeout=0.001)
                    if frame and (frame.can_id & 0x780 == 0x580):
                        node_id = frame.can_id & 0x7F
                        if node_id not in TARGET_NODES: continue
                        
                        idx = TARGET_NODES.index(node_id)
                        sdo_res = protocol.parse_sdo_response(frame)
                        if sdo_res and sdo_res.value is not None and sdo_res.abort_code is None:
                            val = sdo_res.value
                            if val > 0x7FFFFFFF: val -= 0x100000000
                            
                            if sdo_res.index == pos_idx:
                                current_counts[idx] = val
                            elif sdo_res.index == vel_idx:
                                current_vel_raws[idx] = val

                if any(c is None for c in current_counts): continue

                # 4. 제어 연산 및 명령 전송
                all_reached = True
                status_msg = ""
                
                for i, node_id in enumerate(TARGET_NODES):
                    current_rad = current_counts[i] / COUNTS_PER_RADIAN
                    current_vel_rad = current_vel_raws[i] / COUNTS_PER_RADIAN
                    error_rad = target_rads[i] - current_rad
                    error_count = ALIGN_TARGET_COUNTS[i] - current_counts[i]

                    if abs(error_count) > ERROR_THRESH_COUNT:
                        all_reached = False
                        v_pd = (Kp_list[i] * error_rad) - (Kd_list[i] * current_vel_rad)

                        # Anti-windup PID
                        error_integrals[i] += error_rad * dt
                        error_integrals[i] = max(-i_limit/Ki_list[i], min(i_limit/Ki_list[i], error_integrals[i]))
                        v_i = Ki_list[i] * error_integrals[i]

                        v_stiction = stiction_offset_list[i] * (1.0 if error_rad > 0 else -1.0)
                        total_voltage = v_pd + v_i + v_stiction
                        clamped_voltage = max(-voltage_limit, min(voltage_limit, total_voltage))
                    else:
                        clamped_voltage = 0.0
                    
                    bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, int(clamped_voltage), TARGET_AXIS))
                    status_msg += f"[N{node_id}: {current_counts[i]:4d}] "

                if all_reached:
                    print(f"\n\n 🎉 모든 조인트 정렬 완료! {status_msg}")
                    break

                print(f"\r ⚙️ 정렬 중... {status_msg}", end='', flush=True)

                # 주기 유지
                next_time += period
                time.sleep(max(0, next_time - time.monotonic()))

        except KeyboardInterrupt:
            print("\n ⚠️ 정지 명령 수신.")
        
        # 종료 시 전압 차단
        for node_id in TARGET_NODES:
            bus.send(protocol.make_q_axis_voltage_mv_sdo(node_id, 0, TARGET_AXIS))
        print("\n 🔒 모든 모터 전압이 차단되었습니다.")

if __name__ == '__main__':
    main()