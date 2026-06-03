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
    # [⚙️ 하드웨어 세팅 및 제어 파라미터]
    # =========================================================================
    TARGET_NODE_ID = 1  
    TARGET_AXIS = 1     
    
    ALIGN_RAW_COUNT = -980      # 1자 정렬 기준 엔코더 값
    MAX_FLEX_LIMIT = -651       # 최대 굽힘 한계 (카운트 상한선)
    MAX_EXT_LIMIT = -2014       # 최대 펼침 한계 (카운트 하한선)
    PULSES_PER_DEGREE = 11.489  # 1도당 펄스 수
    
    # 10도 이동 모션용 step 계산
    DEGREE_STEP = 10.0
    COUNT_STEP = DEGREE_STEP * PULSES_PER_DEGREE 
    
    # 제어 게인 (단위 호환성 검증용 기본 세팅)
    Kp = 750
    Kd = 0.005
    stiction_offset = 1485.0    # 마찰 저항 극복 전압 (1.485V)
    
    # 역기전력 상수
    GEAR_RATIO = 406.4
    K_emf_rad = 8.632 * GEAR_RATIO
    COUNTS_PER_RADIAN = PULSES_PER_DEGREE * (180.0 / np.pi)
    K_emf_count = K_emf_rad / COUNTS_PER_RADIAN            # 약 5.329 mV/(Counts/s)
    
    voltage_limit = 9500.0      # 최대 제한 전압 (9.5V)
    ERROR_THRESH_COUNT = 1.0    # 목표 도달 허용 오차 (1 카운트)
    LPF_ALPHA = 0.25            # 속도 필터 계수
    # =========================================================================

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(" 📊 1번 조인트 10도 모션 구동 및 실시간 순간 속도 계측 스크립트")
    print(f" [안전 가동 범위] 최대 펼침({MAX_EXT_LIMIT}) ◀──▶ 최대 굽힘({MAX_FLEX_LIMIT})")
    print(f" [모션 제어 단위] {DEGREE_STEP}° 명령 시 {COUNT_STEP:.1f} 카운트 단위 변위 이동")
    print(" [조작 방법] ")
    print("   - [+] 입력 후 Enter : +10도 이동 (굽힘 방향)")
    print("   - [-] 입력 후 Enter : -10도 이동 (펼침 방향)")
    print("   - Ctrl + C : 안전 종료 (모터 전압 즉시 차단)")
    print("=" * 60)

    with SocketCanBus('can0', receive_timeout=0.0) as bus:
        # CANopen NMT Start
        bus.send(protocol.make_nmt_start(0))
        time.sleep(0.1)

        print(f"\n⏳ [Booting] Node {TARGET_NODE_ID}의 현재 엔코더 값을 수신 중...")
        pos_obj = Cia402Object(0x6064)
        bus.send(protocol.make_sdo_read(TARGET_NODE_ID, pos_obj))
        
        last_raw_count = None
        while last_raw_count is None:
            frame = bus.recv(timeout=0.01)
            if frame and frame.can_id == 0x580 + TARGET_NODE_ID:
                sdo_res = protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                    last_raw_count = sdo_res.value
                    if last_raw_count > 0x7FFFFFFF: last_raw_count -= 0x100000000

        print(f" ✅ 초기 위치 동기화 완료: [현재 카운트: {last_raw_count}]")
        print(f" 🚀 일자 정렬 위치인 '{ALIGN_RAW_COUNT}' 값을 초기 타겟으로 구동을 시작합니다.")
        print("-" * 60)

        target_raw_count = float(ALIGN_RAW_COUNT)
        last_time = time.monotonic()
        filtered_velocity_old = 0.0
        
        period = 0.02  # 50Hz 제어 루프 주기
        next_time = time.monotonic()

        while True:
            try:
                # 1. 키보드 입력을 통한 10도 모션 타겟 업데이트 (비동기)
                if kbhit():
                    user_input = sys.stdin.readline().strip()
                    if user_input in ['+', '-']:
                        step = COUNT_STEP if user_input == '+' else -COUNT_STEP
                        new_target = target_raw_count + step
                        
                        # 하드웨어 보호용 소프트 한계점 통제
                        if new_target > MAX_FLEX_LIMIT:
                            target_raw_count = float(MAX_FLEX_LIMIT)
                            print(f" ⚠️ [경고] 최대 굽힘 한계점({MAX_FLEX_LIMIT}) 도달!")
                        elif new_target < MAX_EXT_LIMIT:
                            target_raw_count = float(MAX_EXT_LIMIT)
                            print(f" ⚠️ [경고] 최대 펼침 한계점({MAX_EXT_LIMIT}) 도달!")
                        else:
                            target_raw_count = new_target
                            approx_deg = (target_raw_count - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                            print(f" ▶ [모션 명령] 목표 위치 변경 ➡️ {int(target_raw_count)} (약 {approx_deg:+.1f}°)")

                # 2. 현재 엔코더 값 피드백 요청
                bus.send(protocol.make_sdo_read(TARGET_NODE_ID, pos_obj))
                
                current_raw_count = None
                timeout_end = time.monotonic() + 0.008
                while time.monotonic() < timeout_end:
                    frame = bus.recv(timeout=0.001)
                    if frame and frame.can_id == 0x580 + TARGET_NODE_ID:
                        sdo_res = protocol.parse_sdo_response(frame)
                        if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                            current_raw_count = sdo_res.value
                            if current_raw_count > 0x7FFFFFFF: current_raw_count -= 0x100000000
                            break

                if current_raw_count is not None:
                    current_time = time.monotonic()
                    
                    # 3. 시간 및 카운트 변위 기반 실시간 순간 속도 계산
                    dt = current_time - last_time
                    delta_count = current_raw_count - last_raw_count
                    
                    if dt > 0:
                        instant_velocity_counts_per_sec = delta_count / dt
                    else:
                        instant_velocity_counts_per_sec = 0.0
                    
                    # 속도 피드백 저역통과필터(LPF) 연산
                    filtered_velocity = (LPF_ALPHA * instant_velocity_counts_per_sec) + ((1.0 - LPF_ALPHA) * filtered_velocity_old)
                    filtered_velocity_old = filtered_velocity
                    
                    # 4. 제어 전압 연산 (PD + 마찰보상 + 역기전력 보상)
                    error_count = target_raw_count - current_raw_count
                    v_pd = (Kp * error_count) - (Kd * filtered_velocity)
                    v_emf = K_emf_count * filtered_velocity
                    
                    if abs(error_count) > ERROR_THRESH_COUNT:
                        v_stiction = stiction_offset * np.sign(error_count)
                        total_voltage = v_pd + v_stiction + v_emf
                    else:
                        # 불감대 내 제어 연속성 유지 (멈칫 현상 방지)
                        active_direction = np.sign(filtered_velocity if filtered_velocity != 0 else error_count)
                        v_stiction_tail = (stiction_offset * 0.8) * active_direction if active_direction != 0 else 0.0
                        total_voltage = (-Kd * filtered_velocity) + v_emf + v_stiction_tail
                        
                    # 안전 장치: 소프트 리미트 영역 전압 차단
                    if (current_raw_count >= MAX_FLEX_LIMIT and total_voltage > 0) or \
                       (current_raw_count <= MAX_EXT_LIMIT and total_voltage < 0):
                        total_voltage = 0.0
                        
                    clamped_voltage = max(-voltage_limit, min(voltage_limit, total_voltage))
                    
                    # 드라이버에 전압 송신
                    tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=int(clamped_voltage), axis=TARGET_AXIS)
                    bus.send(tx_frame)
                    
                    # 5. 데이터 업데이트 및 화면 출력
                    last_raw_count = current_raw_count
                    last_time = current_time
                    
                    print(f" [Cmd: {int(target_raw_count):5d}] | "
                          f"[Pos: {current_raw_count:5d}] | "
                          f"[Err: {int(error_count):4d}] | "
                          f"🔥 [Vel: {instant_velocity_counts_per_sec:8.1f} Counts/s] | "
                          f"[Volt: {clamped_voltage:4.0f}mV]", end='\r')

                # 주기 제어 (50Hz)
                next_time += period
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_time = time.monotonic()

            except KeyboardInterrupt:
                print(f"\n\n⚠️ 안전 차단: Node {TARGET_NODE_ID} 전압을 즉시 차단(0mV)하고 계측을 종료합니다.")
                zero_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=0, axis=TARGET_AXIS)
                bus.send(zero_frame)
                break

if __name__ == '__main__':
    main()