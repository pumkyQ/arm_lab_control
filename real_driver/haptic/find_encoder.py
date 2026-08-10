#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📍 Welcon 4개 관절 (j1~j4) 실시간 엔코더 카운트 모니터링 툴
======================================================================
- find_encoder.ino 의 기능을 Ubuntu SocketCAN(PEAK-USB, can0, 1Mbps)으로 완전 이식
- 50Hz 주기로 SDO Read를 통해 엔코더 카운트를 읽어 터미널에 출력합니다.

[실행 방법]
  sudo ip link set can0 up type can bitrate 1000000
  python3 find_encoder.py
"""

import sys
import os
import time

# kitech_v1 경로 설정
# 이 파일 위치: arm_lab_control/real_driver/haptic/
# kitech_v1 위치: arm_lab_control/kitech_v1/
current_dir     = os.path.dirname(os.path.abspath(__file__))   # .../haptic
real_driver_dir = os.path.dirname(current_dir)                 # .../real_driver
workspace_dir   = os.path.dirname(real_driver_dir)             # .../arm_lab_control
kitech_path     = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path:
    sys.path.append(kitech_path)

try:
    from motor_control.cia402 import Cia402Protocol, Cia402Object
    from motor_control.can_bus import SocketCanBus
except ModuleNotFoundError as e:
    print("\n❌ kitech_v1 모듈을 찾지 못했습니다. 디렉토리 구조를 확인해 주세요!")
    raise e


# =========================================================================
# ⚙️ 4개 관절 노드 및 축 정보 정의 (j1 ~ j4)
# =========================================================================
JOINTS = [
    {"name": "j1 (Node 1, Axis 1)", "node_id": 1, "axis": 1},
    {"name": "j2 (Node 1, Axis 2)", "node_id": 1, "axis": 2},
    {"name": "j3 (Node 3, Axis 1)", "node_id": 3, "axis": 1},
    {"name": "j4 (Node 3, Axis 2)", "node_id": 3, "axis": 2},
]

CAN_CHANNEL  = "can0"
LOOP_HZ      = 50       # 엔코더 읽기 주기 (50Hz = 20ms)
LOG_INTERVAL = 0.1      # 터미널 출력 주기 (100ms)
NMT_INTERVAL = 1.0      # NMT Operational 재전송 주기 (1초)
SDO_TIMEOUT  = 0.004    # SDO 응답 대기 최대 시간 (4ms)


def parse_sdo_int32(data: bytes, index: int, subindex: int):
    """
    SDO Upload Response 파싱.
    매칭되면 signed int32 값을 반환, 아니면 None.
    """
    if len(data) < 8:
        return None
    cmd   = data[0]
    r_idx = int.from_bytes(data[1:3], "little")
    r_sub = data[3]
    if r_idx != index or r_sub != subindex:
        return None
    if cmd == 0x43:   # 4바이트 expedited upload
        return int.from_bytes(data[4:8], "little", signed=True)
    if cmd == 0x4B:   # 2바이트 expedited upload
        return int.from_bytes(data[4:6], "little", signed=True)
    if cmd == 0x4F:   # 1바이트 expedited upload
        return int.from_bytes(data[4:5], "little", signed=True)
    return None


def read_all_encoders(bus, protocol: Cia402Protocol):
    """
    4개 관절에 SDO Read 요청 → 응답 수집 → 카운트 반환
    반환: {joint_name: count or None}
    """
    # 위치 SDO 오브젝트 (axis1=0x6064, axis2=0x6864)
    pos_objects = {}
    for j in JOINTS:
        idx = 0x6064 if j["axis"] == 1 else 0x6864
        pos_objects[j["name"]] = (j["node_id"], idx, 0x00)

    # 일괄 요청 전송 (SDO 충돌 방지를 위해 300µs 간격)
    for jname, (nid, idx, sub) in pos_objects.items():
        obj = Cia402Object(idx, sub)
        bus.send(protocol.make_sdo_read(nid, obj))
        time.sleep(0.0003)

    # 응답 수집 (최대 SDO_TIMEOUT 동안)
    results = {j["name"]: None for j in JOINTS}
    deadline = time.monotonic() + SDO_TIMEOUT

    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break
        # 응답 CAN ID: 0x580 + node_id
        if not (0x580 <= msg.can_id <= 0x5FF):
            continue
        resp_node = msg.can_id - 0x580
        for jname, (nid, idx, sub) in pos_objects.items():
            if resp_node == nid:
                val = parse_sdo_int32(msg.data, idx, sub)
                if val is not None:
                    results[jname] = val

    return results


def main():
    print("=" * 65)
    print(" 🎯 Welcon 4개 관절 (j1~j4) 실시간 엔코더 카운트 모니터링")
    print("=" * 65)
    print(f"  CAN 채널 : {CAN_CHANNEL} (1Mbps)")
    print(f"  읽기 주기 : {LOOP_HZ}Hz  |  출력 주기 : {int(LOG_INTERVAL * 1000)}ms")
    print("  종료     : Ctrl+C")
    print("=" * 65 + "\n")

    protocol = Cia402Protocol()

    try:
        with SocketCanBus(channel=CAN_CHANNEL, receive_timeout=0.0) as bus:
            print(f"✅ SocketCAN 연결 성공: {CAN_CHANNEL}\n")

            # NMT Start (전체 노드 개시)
            bus.send(protocol.make_nmt_start(0))
            time.sleep(0.1)
            print("▶ 실시간 엔코더 출력 모니터링을 시작합니다...\n")

            last_nmt_time  = time.monotonic()
            last_loop_time = time.monotonic()
            last_log_time  = time.monotonic()
            dt = 1.0 / LOOP_HZ

            encoder_counts = {j["name"]: None for j in JOINTS}

            while True:
                now = time.monotonic()

                # 1초 주기 NMT Operational 유지
                if now - last_nmt_time >= NMT_INTERVAL:
                    last_nmt_time = now
                    bus.send(protocol.make_nmt_start(0))

                # 50Hz 주기로 엔코더 읽기
                if now - last_loop_time >= dt:
                    last_loop_time = now
                    encoder_counts = read_all_encoders(bus, protocol)

                # 100ms 간격으로 터미널 출력
                if now - last_log_time >= LOG_INTERVAL:
                    last_log_time = now

                    parts = []
                    for i, j in enumerate(JOINTS):
                        val = encoder_counts.get(j["name"])
                        label = f"J{i + 1}"
                        if val is not None:
                            parts.append(f"{label}: {val:6d} cnt")
                        else:
                            parts.append(f"{label}:  OFFLINE")

                    sys.stdout.write("\r📍 [ENCODER]  " + "  |  ".join(parts) + "    ")
                    sys.stdout.flush()

                # 루프 타이밍 유지
                elapsed = time.monotonic() - now
                sleep_t = dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n\n🛑 모니터링 종료.")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        raise


if __name__ == "__main__":
    main()
