#!/usr/bin/env python3
import time
import sys
import os

# 프로젝트 구조(kitech_v1 패키지) 라이브러리 참조 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object

def main():
    # -------------------------------------------------------------------------
    # [⚙️ 설정 구역] 모니터링 대상 노드 설정
    # -------------------------------------------------------------------------
    TARGET_NODES = [1, 2, 3]  # 조인트 1, 3, 5에 대응하는 Node ID
    CAN_CHANNEL = "can0"
    # -------------------------------------------------------------------------

    protocol = Cia402Protocol()
    
    print("=" * 60)
    print(" 🔍 실시간 절대 엔코더 값(Raw Count) 모니터링 스크립트")
    print(" 이 코드는 제어 명령을 보내지 않으며, 현재 위치값만 읽습니다.")
    print(" 조인트를 손으로 움직여 매핑할 절대 엔코더 값을 확인하세요.")
    print(" [종료: Ctrl + C]")
    print("=" * 60)

    with SocketCanBus(CAN_CHANNEL, receive_timeout=0.0) as bus:
        # 하드웨어 응답을 활성화하기 위해 NMT Start 명령 송신
        bus.send(protocol.make_nmt_start(0))
        time.sleep(0.1)

        encoder_values = {node: 0 for node in TARGET_NODES}
        
        try:
            while True:
                # 1. 각 노드에 현재 위치(Position Actual Value) SDO 요청
                for node_id in TARGET_NODES:
                    # 조교님 라이브러리 특성상 Axis 1은 0x6064 인덱스 사용
                    pos_obj = Cia402Object(0x6064)
                    bus.send(protocol.make_sdo_read(node_id, pos_obj))

                # 2. CAN 버스에서 응답 수집 (약 15ms 동안)
                timeout_end = time.monotonic() + 0.015
                while time.monotonic() < timeout_end:
                    frame = bus.recv(timeout=0.001)
                    if frame is None:
                        continue

                    # SDO 응답(0x580 + NodeID)인지 확인
                    if (frame.can_id & 0x780) == 0x580:
                        node_id = frame.can_id & 0x7F
                        if node_id in TARGET_NODES:
                            sdo_res = protocol.parse_sdo_response(frame)
                            
                            # 인덱스가 0x6064(위치)이고 값이 정상적으로 왔는지 확인
                            if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                                val = sdo_res.value
                                # 32비트 Unsigned를 Signed로 변환
                                if val > 0x7FFFFFFF:
                                    val -= 0x100000000
                                encoder_values[node_id] = val

                # 3. 화면 출력 (한 줄에 표시)
                output_str = "\r"
                for node_id in TARGET_NODES:
                    output_str += f"[Node {node_id} (J{node_id*2-1 if node_id > 1 else 1}): {encoder_values[node_id]:8d}]   "
                
                print(output_str, end="", flush=True)

                # 너무 빠른 루프 방지 (약 20Hz)
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n모니터링을 종료합니다.")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"\n❌ 실행 중 에러 발생: {e}")