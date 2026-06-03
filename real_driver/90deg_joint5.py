#!/usr/bin/env python3
import time
import sys
import select
import os
import numpy as np

# 조교님 패키지(kitech_v1) 라이브러리 참조 경로 자동 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object

def clamp(val, limit):
    if val > limit: return limit
    if val < -limit: return -limit
    return val

def kbhit():
    """터미널 키보드 입력 여부를 비동기 체크하는 헬퍼 함수"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

def main():
    # =========================================================================
    # [⚙️ 5번 조인트(Node 3) 단독 전용 제어 파라미터 및 게인 튜닝 부]
    # =========================================================================
    TARGET_NODE_ID = 3  # 5번 조인트 고정
    TARGET_AXIS = 1     # 1축 고정
    
    # 형님이 찾으신 실제 꺾인 각도 반영 스케일 상수 (70~72도 보정치)
    COUNTS_PER_RADIAN = 750.0          
    VELOCITY_COUNTS_PER_RADIAN = 750.0 
    
    # 🏎️ 목표치 근처에서 굼뜨는 현상을 진압한 짱짱한 게인 세팅
    Kp = 7500.0          # 근접 시에도 끝까지 밀어주는 힘 상향
    Kd = 150.0           # 끈적거림 방지를 위해 브레이크 하향
    stiction_offset = 1500.0  # 초기 마찰 락을 스무스하게 뚫어버리는 전압(1.5V)
    
    # 소프트웨어 안전 마진 제한 (영점 기준 좌우 90도 범위 제한)
    min_limit = -1.57    # 약 -90도
    max_limit = 1.57     # 약 +90도
    
    voltage_limit = 9500.0  # 드라이버 보호용 최대 전압 제한 (9.5V)
    ERROR_THRESH = 0.002    # 목표 도달 허용 오차 판정선
    
    # 5도 단위 스텝 (5 * pi / 180)
    RAD_STEP = 90.0 * (3.1415926535 / 180.0)
    # =========================================================================

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(" 🕹️ 5번 조인트(Node 3) 시작 지점 『0도 영점 자동 보정』 제어 스크립트")
    print(f" [하드웨어 스펙 매핑] COUNTS_PER_RADIAN = {COUNTS_PER_RADIAN}")
    print(" [조작 방법] ")
    print("   - [+] 입력 후 Enter : 5번 관절 목표 각도 +5도")
    print("   - [-] 입력 후 Enter : 5번 관절 목표 각도 -5도")
    print("   - Ctrl + C : 안전 종료 (모터 전압 즉시 차단 및 프로그램 해제)")
    print("=" * 60)

    with SocketCanBus('can0', receive_timeout=0.0) as bus:
        
        # CANopen 네트워크 장치 기선 가동
        nmt_frame = protocol.make_nmt_start(0)
        bus.send(nmt_frame)
        time.sleep(0.1)

        # ---------------------------------------------------------------------
        # 🎯 [핵심 추가] 프로그램 시작 직후 최초 엔코더 원시 카운트 획득 루틴
        # ---------------------------------------------------------------------
        print("\n⏳ [Homing] 현재 위치를 0도로 잡기 위해 초기 엔코더 값을 파싱 중...")
        pos_obj = Cia402Object(0x6064)
        bus.send(protocol.make_sdo_read(TARGET_NODE_ID, pos_obj))
        
        initial_raw_count = None
        deadline = time.monotonic() + 1.0  # 최대 1초간 대기하며 첫 패킷 수집
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=0.01)
            if frame and frame.can_id == 0x580 + TARGET_NODE_ID:
                sdo_res = protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                    initial_raw_count = sdo_res.value
                    break
                    
        if initial_raw_count is None:
            print("❌ 에러: 초기 엔코더 값을 읽어오지 못했습니다. 전원 및 CAN 연결을 확인하세요.")
            return
            
        if initial_raw_count > 0x7FFFFFFF:
            initial_raw_count -= 0x100000000
            
        print(f" ✅ 영점 보정 완료! [초기 엔코더 카운트(0도 기준점): {initial_raw_count}]")
        print("-" * 60)
        # ---------------------------------------------------------------------

        # 제어 변수 초기화
        target_position = 0.0
        current_position = 0.0
        current_velocity = 0.0
        applied_voltage = 0.0

        print(f" ▶ 현재 제어 루프 가동 중... (실시간 측정 기준 0.0° 스타트)")

        # 50Hz 주기 설정
        period = 1.0 / 50.0
        next_time = time.monotonic()

        while True:
            try:
                # 1. 키보드 입력 비동기 처리
                if kbhit():
                    user_input = sys.stdin.readline().strip()
                    if user_input in ['+', '-']:
                        step = RAD_STEP if user_input == '+' else -RAD_STEP
                        new_target = target_position + step
                        target_position = max(min_limit, min(max_limit, new_target))
                        print(f" ▶ [명령 수신] 5번 조인트 목표 각도 변경 ➡️ {target_position*(180/3.141592):.1f}°")

                # 2. SDO 피드백 읽기 패킷 요청
                pos_obj = Cia402Object(0x6064)
                vel_obj = Cia402Object(0x606c)
                bus.send(protocol.make_sdo_read(TARGET_NODE_ID, pos_obj))
                bus.send(protocol.make_sdo_read(TARGET_NODE_ID, vel_obj))
                
                # 3. 8ms 동안 데이터 수집
                timeout_end = time.monotonic() + 0.008
                while time.monotonic() < timeout_end:
                    frame = bus.recv(timeout=0.001)
                    if frame is None: continue
                    
                    sdo_res = protocol.parse_sdo_response(frame)
                    if sdo_res and sdo_res.value is not None and sdo_res.node_id == TARGET_NODE_ID:
                        val = sdo_res.value
                        if val > 0x7FFFFFFF: val -= 0x100000000
                        
                        if sdo_res.index == 0x6064:
                            # 🔴 [정석 오프셋 연산] 현재 읽은 카운트(val)에서 시작 카운트(initial_raw_count)를 빼줍니다!
                            relative_count = val - initial_raw_count
                            current_position = relative_count / COUNTS_PER_RADIAN
                        elif sdo_res.index == 0x606c:
                            current_velocity = val / VELOCITY_COUNTS_PER_RADIAN

                # 4. 순수 1차원 PD 제어 및 마찰 보상 연산
                error = target_position - current_position
                
                # 오차 기반 PD 수식
                v_pd = (Kp * error) - (Kd * current_velocity)
                
                if abs(error) > ERROR_THRESH:
                    v_stiction = stiction_offset * np.sign(error)
                    total_voltage = v_pd + v_stiction
                else:
                    total_voltage = 0.0
                    
                # 하드웨어 물리 한계 보호 장치
                if (current_position >= max_limit and total_voltage > 0) or \
                   (current_position <= min_limit and total_voltage < 0):
                    total_voltage = 0.0
                    
                clamped_voltage = max(-voltage_limit, min(voltage_limit, total_voltage))
                applied_voltage = clamped_voltage
                
                # 모터 드라이버 전압 명령 전송
                tx_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=int(clamped_voltage), axis=TARGET_AXIS)
                bus.send(tx_frame)

                # 5. 실시간 모니터링 출력
                print(f" [Cmd_J5: {target_position*(180/3.141592):5.1f}°] | "
                      f"[Real_J5: {current_position*(180/3.141592):5.1f}°] | "
                      f"[Out_Volt: {applied_voltage:4.0f}mV]", end='\r')

                # 정밀 루프 주기 동기화 (50Hz)
                next_time += period
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_time = time.monotonic()

            except KeyboardInterrupt:
                print("\n\n⚠️ 안전 장치 발동: 5번 조인트 전압을 즉시 차단(0mV)하고 종료합니다.")
                zero_frame = protocol.make_q_axis_voltage_mv_sdo(node_id=TARGET_NODE_ID, voltage_mv=0, axis=TARGET_AXIS)
                bus.send(zero_frame)
                break

if __name__ == '__main__':
    main()