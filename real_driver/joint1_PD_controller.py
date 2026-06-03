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
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

def kbhit():
    """터미널 키보드 입력 여부를 비동기 체크하는 헬퍼 함수"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

def main():
    # =========================================================================
    # [⚙️ 하드웨어 세팅 및 튜닝 완료된 파라미터 정보]
    # =========================================================================
    TARGET_NODE_ID = 1  
    TARGET_AXIS = 1     
    
    ALIGN_RAW_COUNT = -980      # 1자 정렬 기준 엔코더 원점 값
    MAX_FLEX_LIMIT = -651       # 최대 굽힘 한계 (카운트 상한선)
    MAX_EXT_LIMIT = -2014       # 최대 펼침 한계 (카운트 하한선)
    PULSES_PER_DEGREE = 11.489  # 1도당 펄스 수
    
    # 최종 튜닝 완료된 황금 게인 세팅 (Count 단위 기반)
    Kp = 550
    Kd = 0.42
    stiction_offset = 1600.0    # 마찰 저항 극복 전압 (1.6V)
    
    # 역기전력 상수 변환
    GEAR_RATIO = 406.4
    K_emf_rad = 8.632 * GEAR_RATIO                         
    COUNTS_PER_RADIAN = PULSES_PER_DEGREE * (180.0 / np.pi) 
    K_emf_count = K_emf_rad / COUNTS_PER_RADIAN            # 약 5.329 mV/(Counts/s)
    
    voltage_limit = 9500.0      # 최대 제한 전압 (9.5V)
    ERROR_THRESH_COUNT = 2.0    # 목표 도달 허용 오차 판정선 (2 카운트)
    LPF_ALPHA = 0.12            # 필터 튜닝값 (12% 주입 비율)
    # =========================================================================

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(" 🕹️ 1번 조인트 각도 지정 입력형 정밀 모션 제어 스크립트")
    print(f" [안전 가동 범위] 최대 펼침({MAX_EXT_LIMIT}) ◀──▶ 최대 굽힘({MAX_FLEX_LIMIT})")
    print(f" [하드웨어 게인] Kp: {Kp} | Kd: {Kd} | Stiction: {stiction_offset}mV")
    print(" [사용 방법] ")
    print("   1. 실행 시 자동으로 일자 정렬(-980 Count, 0.0°) 위치로 가동 및 정렬됩니다.")
    print("   2. 정렬 완료 후 터미널에 원하는 목표 각도를 입력하고 Enter를 누르십시오.")
    print("      (예 입력: 10 또는 -15.5 또는 0)")
    print("   3. Ctrl + C : 안전 종료 (모터 전압 즉시 차단)")
    print("=" * 60)

    with SocketCanBus('can0', receive_timeout=0.0) as bus:
        # CANopen NMT Start
        nmt_frame = protocol.make_nmt_start(0)
        bus.send(nmt_frame)
        time.sleep(0.1)

        # [추가] 하드웨어 활성화: 전압 모드 설정 및 Operation Enabled 상태 전환
        print(f" ▶ Node {TARGET_NODE_ID} 활성화 시도 중...")
        # 1. 운전 모드 설정 (Voltage Mode: -11)
        bus.send(protocol.make_axis_mode_sdo(TARGET_NODE_ID, TARGET_AXIS, -11))
        time.sleep(0.05)

        # 2. CiA402 상태 기기 전환 (Fault Reset -> Shutdown -> Switch On -> Enable)
        for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                     Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
            bus.send(protocol.make_axis_controlword_sdo(TARGET_NODE_ID, TARGET_AXIS, ctrl))
            time.sleep(0.03)

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
        print(f" 🚀 [자동 정렬 시작] 일자 정렬 원점 위치('{ALIGN_RAW_COUNT}')로 자동 이동합니다.")
        print(" 💡 정렬 완료 후 원하는 목표 각도(deg)를 입력할 수 있는 대기 상태로 전환됩니다.")
        print("-" * 60)

        # 시스템 시작 시 타겟을 일자 정렬 위치로 강제 고정하여 초기화 가동
        target_raw_count = float(ALIGN_RAW_COUNT)
        current_velocity_raw = 0.0
        filtered_velocity_old = 0.0  
        applied_voltage = 0.0

        period = 1.0 / 50.0
        next_time = time.monotonic()
        
        # 키보드 입력 버퍼 및 안내 메시지 플래그
        input_buffer = ""
        prompt_shown = False

        while True:
            try:
                # 1. 터미널 각도 입력 처리 (비동기 논블로킹 방식)
                if kbhit():
                    char = sys.stdin.read(1)
                    if char == '\n':  # Enter가 입력되었을 때 버퍼 해석
                        user_input = input_buffer.strip()
                        input_buffer = ""  # 버퍼 초기화
                        prompt_shown = False
                        
                        if user_input:
                            try:
                                # 입력된 문자를 실수형 각도로 파싱
                                target_degree = float(user_input)
                                
                                # 일자 정렬 기준(0도) 대비 오프셋 변위 카운트 연산
                                requested_count = ALIGN_RAW_COUNT + (target_degree * PULSES_PER_DEGREE)
                                
                                # 🛡️ 소프트웨어 제어 한계점 상하한 통제
                                if requested_count > MAX_FLEX_LIMIT:
                                    target_raw_count = float(MAX_FLEX_LIMIT)
                                    allowed_deg = (MAX_FLEX_LIMIT - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                                    print(f"\n ⚠️ [한계 제한] 입력 각도가 소프트 제한을 초과하여 최대 굽힘 한계로 고정합니다. (제한: {allowed_deg:.2f}°)")
                                elif requested_count < MAX_EXT_LIMIT:
                                    target_raw_count = float(MAX_EXT_LIMIT)
                                    allowed_deg = (MAX_EXT_LIMIT - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                                    print(f"\n ⚠️ [한계 제한] 입력 각도가 소프트 제한을 초과하여 최대 펼침 한계로 고정합니다. (제한: {allowed_deg:.2f}°)")
                                else:
                                    target_raw_count = requested_count
                                    print(f"\n ▶ [목표 갱신] 제어 타겟 각도 지령 수신 ➡️ {target_degree:+.2f}° (목표 카운트: {int(target_raw_count)})")
                                    
                            except ValueError:
                                print(f"\n ❌ 잘못된 입력입니다. 숫자(예: 10, -5.5) 형식으로 입력해 주십시오.")
                    else:
                        # Enter가 아니라면 버퍼에 문자 누적 및 에코 출력
                        input_buffer += char
                        sys.stdout.write(char)
                        sys.stdout.flush()

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
                # 4. 검증 완료된 필터 및 불감대 제어 연산부
                # =============================================================
                
                # 1단계: 속도 피드백 LPF 연산 (형님의 0.12 황금 계수 적용)
                filtered_velocity = (LPF_ALPHA * current_velocity_raw) + ((1.0 - LPF_ALPHA) * filtered_velocity_old)
                filtered_velocity_old = filtered_velocity
                
                # 엔코더 카운트 오차 계산
                error_count = target_raw_count - current_raw_count
                
                # 튜닝된 게인 기반 PD 및 역기전력 유도 전압 연산
                v_pd = (Kp * error_count) - (Kd * filtered_velocity)
                v_emf = K_emf_count * filtered_velocity
                
                # 2단계: 방향 전환 및 백래시 탈출을 위한 전압 연속성 확보 로직
                if abs(error_count) > ERROR_THRESH_COUNT:
                    v_stiction = stiction_offset * np.sign(error_count)
                    total_voltage = v_pd + v_stiction + v_emf
                else:
                    # 오차가 불감대(2 카운트) 이내로 들어왔을 때 안착 제어 가동
                    active_direction = np.sign(filtered_velocity if filtered_velocity != 0 else error_count)
                    
                    if active_direction != 0:
                        v_stiction_tail = (stiction_offset * 0.7) * active_direction
                    else:
                        v_stiction_tail = 0.0
                        
                    total_voltage = (-Kd * filtered_velocity) + v_emf + v_stiction_tail
                    
                # 물리적 하드웨어 한계점 통제
                if (current_raw_count >= MAX_FLEX_LIMIT and total_voltage > 0) or \
                   (current_raw_count <= MAX_EXT_LIMIT and total_voltage < 0):
                    total_voltage = 0.0
                    
                clamped_voltage = max(-voltage_limit, min(voltage_limit, total_voltage))
                applied_voltage = clamped_voltage
                
                tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=int(clamped_voltage), axis=TARGET_AXIS)
                bus.send(tx_frame)

                # 5. 실시간 상태 및 목표 각도 대비 모니터링 출력
                real_deg = (current_raw_count - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                cmd_deg = (target_raw_count - ALIGN_RAW_COUNT) / PULSES_PER_DEGREE
                
                # 터미널 한 줄에 현재 제어 상태를 동적으로 출력
                sys.stdout.write(f"\r [목표: {cmd_deg:+6.1f}°] | [현재: {real_deg:+6.1f}°] | [오차: {int(error_count):3d} Count] | [출력: {applied_voltage:4.0f}mV] | 입력창 ➡️ {input_buffer}")
                sys.stdout.flush()

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