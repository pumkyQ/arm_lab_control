#!/usr/bin/env python3
import can
import time
import struct

def main():
    # vcan0 인터페이스 연결
    try:
        bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
        print("🚀 가상 Welcon 노드(Node 1) 시작됨 (vcan0)")
    except:
        print("❌ vcan0를 찾을 수 없습니다. 설정을 먼저 진행하세요.")
        return

    # 가상 상태 변수
    pos = -980.0
    vel = 0.0
    voltage = 0.0
    last_time = time.monotonic()

    while True:
        msg = bus.recv(timeout=0.01)
        curr_time = time.monotonic()
        dt = curr_time - last_time
        last_time = curr_time

        # 간단한 물리 시뮬레이션 (전압에 따른 위치 변화)
        if abs(voltage) > 500: # 500mV 이상일 때만 움직임 (마찰 흉내)
            vel = voltage * 0.05
            pos += vel * dt
        else:
            vel *= 0.5 # 감쇠

        if msg is None: continue

        # SDO 요청 처리 (0x600 + NodeID 1)
        if msg.arbitration_id == 0x601:
            command = msg.data[0]
            index = struct.unpack('<H', msg.data[1:3])[0]
            subindex = msg.data[3]

            # SDO Read 요청 (0x40)
            if command == 0x40:
                response_data = bytearray([0x43, msg.data[1], msg.data[2], msg.data[3], 0, 0, 0, 0])
                
                if index == 0x6064: # Position
                    val = int(pos)
                    response_data[4:8] = struct.pack('<i', val)
                elif index == 0x606c: # Velocity
                    val = int(vel)
                    response_data[4:8] = struct.pack('<i', val)
                elif index == 0x6041: # Statusword (0x07: Switched On / 0x04: Op Enabled)
                    response_data[0] = 0x4B # 2-byte data
                    response_data[4:6] = struct.pack('<H', 0x0007 | 0x0004)
                elif index == 0x6078: # Current
                    response_data[0] = 0x4B
                    response_data[4:6] = struct.pack('<h', int(voltage/10))
                
                bus.send(can.Message(arbitration_id=0x581, data=response_data, is_extended_id=False))

            # SDO Write 요청 (0x23: 4-byte)
            elif command == 0x23:
                if index == 0x2103: # 전압 명령 (Welcon 특수 인덱스)
                    voltage = float(struct.unpack('<i', msg.data[4:8])[0])
                
                # 쓰기 확인 응답
                res = bytearray([0x60, msg.data[1], msg.data[2], msg.data[3], 0, 0, 0, 0])
                bus.send(can.Message(arbitration_id=0x581, data=res, is_extended_id=False))

if __name__ == "__main__":
    main()