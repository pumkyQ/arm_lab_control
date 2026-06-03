#!/usr/bin/env python3
import time
import sys
import select
import os
import numpy as np

# 프로젝트 구조(kitech_v1 패키지) 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object

def kbhit():
    """터미널 키보드 입력 여부를 비동기 체크하는 헬퍼 함수"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

def main():
    # =========================================================================
    # [⚙️ 1번 조인트 측정 데이터 및 소프트 한계점 세팅]
    # =========================================================================
    TARGET_NODE_ID = 1  
    TARGET_AXIS = 1     
    
    ALIGN_RAW_COUNT = -980      # 1자 정렬 기준 엔코더 값
    MAX_FLEX_LIMIT = -651       # 최대 굽힘 한계 (Upper bound 카운트 기준)
    MAX_EXT_LIMIT = -2014       # 최대 펼침 한계 (Lower bound 카운트 기준)
    PULSES_PER_DEGREE = 11.489  # 1도당 펄스 수
    
    # 10도 이동 시 변위 카운트 계산
    DEGREE_STEP = 10.0
    COUNT_STEP = DEGREE_STEP * PULSES_PER_DEGREE 
    
    # 기존 황금 게인 세팅 유지 (Count 단위 기반)
    Kp = 550
    Kd = 0.42
    stiction_offset = 1600.0    # 초기 마찰 저항 돌파용 전압 (1.485V)
    
    # [역기전력 단위 변환 적용 섹션]
    GEAR_RATIO = 406.4
    K_emf_rad = 8.632 * GEAR_RATIO                         # 팀원 분 스펙: 약 3508.03 mV/(rad/s)
    COUNTS_PER_RADIAN = PULSES_PER_DEGREE * (180.0 / np.pi) # 형님 팩트 기반 변환 상수: 약 658.289 Counts/rad
    
    # [최종 카운트 기준 역기전력 상수] 단위: mV / (Counts/s)
    K_emf_count = K_emf_rad / COUNTS_PER_RADIAN            # 약 5.329 mV/(Counts/s)
    
    voltage_limit = 9500.0      # 드라이버 보호용 최대 전압 제한 (9.5V)
    ERROR_THRESH_COUNT = 2.0    # 목표 도달 허용 오차 판정선 (1 카운트 이내)
    
    # 🌟 [저역통과필터(LPF) 파라미터 세팅]
    # 필터 계수 (0.0에 가까울수록 필터링 강함/지연 증가, 1.0이면 필터 해제)
    # 50Hz 제어 루프에서 고주파 노이즈 및 방향 전환 시 순간 튀는 속도를 잡기 위한 튜닝값
    LPF_ALPHA = 0.12
    # =========================================================================

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(" 🕹️ 1번 조인트(Node 1) LPF 속도 필터 및 불감대 연속성 개선 제어 스크립트")
    print(f" [안전 가동 범위] 최대 펼침({MAX_EXT_LIMIT}) ◀──▶ 최대 굽힘({MAX_FLEX_LIMIT})")
    print(f" [제어 스텝 단위] {DEGREE_STEP}° 구동시 {COUNT_STEP:.1f} 카운트 변위 적용")
    print(f" [K_emf 변환 체크] {K_emf_rad:.2f} mV/(rad/s) ➡️ {K_emf_count:.4f} mV/(Counts/s)")
    print(f" [LPF Alpha 설정] {LPF_ALPHA} (현재 주입 비율 25%)")
    print("=" * 60)

    with SocketCanBus('can0', receive_timeout=0.0) as bus:
        
        # CANopen NMT Start
        nmt_frame = protocol.make_nmt_start(0)
        bus.send(nmt_frame)
        time.sleep(0.1)

        print(f"\n⏳ [Booting] Node {TARGET_NODE_ID}의 현재 절대 엔코더 값을 확인 중...")
        pos_obj = Cia402Object(0x6064)
        bus.send(protocol.make_sdo_read(TARGET_NODE_ID, pos_obj))
        
        current_raw_count = None
        deadline = time.monotonic() + 1.0  
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=0.01)
            if frame and frame.can_id == 0x580 + TARGET_NODE_ID:
                sdo_res = protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                    current_raw_count = sdo_res.value
                    break
                    
        if current_raw_count is None:
            print(f"❌ 에러: Node {TARGET_NODE_ID}로부터 엔코더 피드백을 수신하지 못했습니다.")
            return
            
        if current_raw_count > 0x7FFFFFFF:
            current_raw_count -= 0x100000000
            
        print(f" ✅ 현재 절대 엔코더 위치 파싱 성공: [현재 카운트: {current_raw_count}]")
        print(f" 🚀 [자동 정렬 가동] 일자 정렬 위치인 '{ALIGN_RAW_COUNT}' 값으로 자동 시동합니다.")
        print("-" * 60)

        # 시작 타겟을 일자 정렬(-980) 위치로 강제 고정
        target_raw_count = float(ALIGN_RAW_COUNT)
        current_velocity_raw = 0.0
        filtered_velocity_old = 0.0  # LPF 직전 주기 값 저장용
        applied_voltage = 0.0

        period = 1.0 / 50.0
        next_time = time.monotonic()

        while True:
            try:
                # 1. 키보드 입력 비동기 처리
                if kbhit():
                    user_input = sys.stdin.readline().strip()
                    if user_input in ['+', '-']:
                        step = COUNT_STEP if user_input == '+' else -COUNT_STEP
                        new_target = target_raw_count + step
                        
                        # 🛡️ 소프트 한계점 통제
                        if new_target > MAX_FLEX_LIMIT:
                            target_raw_count = float(MAX_FLEX_LIMIT)
                            print(f" ⚠️ [경고] 최대 굽힘 한계치({MAX_FLEX_LIMIT}) 도달!")
                        elif new_target < MAX_EXT_LIMIT:
                            target_raw_count = float(MAX_EXT_LIMIT)
                            print(f" ⚠️ [경고] 최대 펼침 한계치({MAX_EXT_LIMIT}) 도달!")
                        else:
                            target_raw_count = new_target
                            approx_deg = (target_raw_count - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                            print(f" ▶ [명령 수신] 목표 카운트 변경 ➡️ {int(target_raw_count)} (일자정렬 대비 약 {approx_deg:+.1f}°)")

                # 2. SDO 피드백 읽기 패킷 요청
                pos_obj = Cia402Object(0x6064)
                vel_obj = Cia402Object(0x606c)
                bus.send(protocol.make_sdo_read(TARGET_NODE_ID, pos_obj))
                bus.send(protocol.make_sdo_read(TARGET_NODE_ID, vel_obj))
                
                # 3. 데이터 수집 (8ms 윈도우)
                timeout_end = time.monotonic() + 0.008
                while time.monotonic() < timeout_end:
                    frame = bus.recv(timeout=0.001)
                    if frame is None: continue
                    
                    sdo_res = protocol.parse_sdo_response(frame)
                    if sdo_res and sdo_res.value is not None and sdo_res.node_id == TARGET_NODE_ID:
                        val = sdo_res.value
                        if val > 0x7FFFFFFF: val -= 0x100000000
                        
                        if sdo_res.index == 0x6064:
                            current_raw_count = val
                        elif sdo_res.index == 0x606c:
                            current_velocity_raw = val

                # =============================================================
                # 4. 필터 및 불감대 제어 개선 연산부
                # =============================================================
                
                # 🌟 [1단계: 속도 피드백 저역통과필터(LPF) 연산]
                # 급격한 방향 전환 시 발생하는 고주파 속도 스파이크 및 노이즈 차단
                # =============================================================
                # 4. 필터 및 불감대 제어 개선 연산부 (버전 1)
                # =============================================================
                
                # 1단계: 속도 피드백 LPF
                filtered_velocity = (LPF_ALPHA * current_velocity_raw) + ((1.0 - LPF_ALPHA) * filtered_velocity_old)
                filtered_velocity_old = filtered_velocity
                
                # 엔코더 카운트 오차 계산
                error_count = target_raw_count - current_raw_count
                
                # 필터링된 속도 기반 PD 및 역기전력 연산
                v_pd = (Kp * error_count) - (Kd * filtered_velocity)
                v_emf = K_emf_count * filtered_velocity
                
                # [2단계: 방향 전환 및 백래시 탈출을 위한 전압 연속성 확보]
                if abs(error_count) > ERROR_THRESH_COUNT:
                    # 오차가 클 때는 기존대로 정마찰 보상 인가
                    v_stiction = stiction_offset * np.sign(error_count)
                    total_voltage = v_pd + v_stiction + v_emf
                else:
                    # 🌟 핵심 수정: 오차가 불감대(1 카운트) 안으로 들어왔더라도 
                    # 기어가 반대편에 완전히 안착할 수 있도록 이동 방향으로 정마찰 전압의 80%를 유지하여 밀어줌
                    # filtered_velocity의 부호나 error_count의 마지막 부호를 추종
                    active_direction = np.sign(filtered_velocity if filtered_velocity != 0 else error_count)
                    
                    if active_direction != 0:
                        v_stiction_tail = (stiction_offset * 0.8) * active_direction
                    else:
                        v_stiction_tail = 0.0
                        
                    # 불감대 내에서도 브레이크(Kd), 역기전력(v_emf), 그리고 안착 전압(v_stiction_tail)을 모두 유지
                    total_voltage = (-Kd * filtered_velocity) + v_emf + v_stiction_tail
                    
                # 물리적 하드웨어 한계점 통제 (동일)
                if (current_raw_count >= MAX_FLEX_LIMIT and total_voltage > 0) or \
                   (current_raw_count <= MAX_EXT_LIMIT and total_voltage < 0):
                    total_voltage = 0.0
                    
                clamped_voltage = max(-voltage_limit, min(voltage_limit, total_voltage))
                applied_voltage = clamped_voltage
                
                tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=int(clamped_voltage), axis=TARGET_AXIS)
                bus.send(tx_frame)

                # 5. 실시간 카운트 및 상태 모니터링 출력
                real_deg = (current_raw_count - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                print(f" [Cmd_Count: {int(target_raw_count):5d}] | "
                      f"[Real_Count: {int(current_raw_count):5d}] | "
                      f"[Offset_Deg: {real_deg:5.1f}°] | "
                      f"[Volt: {applied_voltage:4.0f}mV]", end='\r')

                next_time += period
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_time = time.monotonic()

            except KeyboardInterrupt:
                print(f"\n\n⚠️ 안전 장치 발동: 1번 조인트(Node {TARGET_NODE_ID}) 전압을 즉시 차단(0mV)하고 프로그램을 안전 종료합니다.")
                zero_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=0, axis=TARGET_AXIS)
                bus.send(zero_frame)
                break

if __name__ == '__main__':
    main()