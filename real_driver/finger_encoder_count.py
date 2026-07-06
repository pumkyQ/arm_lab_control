#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import select
import termios
import tty
import os
from std_msgs.msg import Float64MultiArray

# =========================================================================
# [⚙️ 패키지 라이브러리 참조 경로 자동 추가]
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

CAN_CHANNEL = "can0" 

def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class KitechHardwareCalibrator(Node):
    def __init__(self):
        super().__init__('kitech_hardware_calibrator_node')
        
        # 🎯 대상 노드 및 축 설정
        self.TARGET_NODE_ID = 1  
        self.TARGET_AXIS = 1     
        
        # ⚡ [핵심] 하드웨어 파손 방지용 미세 구동 안전 전압 세팅 (mV 단위)
        # 기구부 마찰을 겨우 뚫고 천천히 움직일 만한 최소 전압으로 튜닝 필요 (예: 500mV ~ 1200mV)
        self.JOG_VOLTAGE_STEP = 9000.0 
        
        self.LOOP_RATE = 20.0  # 모니터링 및 제어 주기 20Hz
        self.dt = 1.0 / self.LOOP_RATE

        self.current_raw_count = 0
        self.status_word = 0
        self.target_voltage = 0.0  # 실시간 인가될 조그 전압
        self.input_buffer = ""
        
        # 터미널 설정 (키 입력 즉시 감지)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Protocol 및 Object 초기화
        self.protocol = Cia402Protocol()
        self.pos_obj = Cia402Object(0x6064)
        self.status_obj = Cia402Object(0x6041)

        # SocketCAN 연결
        try:
            self.can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
            self.bus = self.can_bus_context.__enter__()
            self.get_logger().info(f"✅ CAN 연결 성공: {CAN_CHANNEL} (Node {self.TARGET_NODE_ID} 조그 캘리브레이션)")
            self.init_can_hardware_active()
        except Exception as e:
            self.get_logger().error(f"❌ CAN 연결 실패: {e}")
            self.bus = None

        self.timer = self.create_timer(self.dt, self.control_loop)

    def init_can_hardware_active(self):
        """스스로 움직일 수 있도록 드라이버를 정상 구동 상태(Enable)로 진입"""
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)
        
        # Voltage Mode(-11) 진입
        self.bus.send(self.protocol.make_axis_mode_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, -11))
        time.sleep(0.05)

        # CiA402 상태기기 완벽 활성화 (Enable Operation 단계까지 진입)
        for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                     Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, ctrl))
            time.sleep(0.03)
            
        # 대기 상태 전압 0mV 초기화
        self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(self.TARGET_NODE_ID, 0, self.TARGET_AXIS))

        print("\n" + "="*60)
        print("🎮 [자체 미세 구동형 조그 캘리브레이션 모드 활성화]")
        print(f" ➡️  현재 조그 인가 전압 단위: {self.JOG_VOLTAGE_STEP} mV")
        print("-"*60)
        print(" [움직임 조종 키] (입력 후 엔터)")
        print("   j  : 굽힘 방향 이동 (양의 전압 인가)")
        print("   k  : 펼침 방향 이동 (음의 전압 인가)")
        print("   s  : 즉시 정지 (0mV 상태로 복귀)")
        print("-"*60)
        print(" [좌표 저장 단축키] (원하는 위치 도달 후 입력+엔터)")
        print("   a  : 일자 정렬(Align) 카운트 등록 및 출력")
        print("   f  : 최대 굽힘(Flex) 한계 카운트 등록 및 출력")
        print("   e  : 최대 펼침(Ext) 한계 카운트 등록 및 출력")
        print("="*60 + "\n")

    def control_loop(self):
        if self.bus is None: return

        # ----------------------------------------------------------------------
        # 🎮 키보드 입력 처리 (조그 제어 및 확정 명령)
        # ----------------------------------------------------------------------
        while kbhit():
            try:
                char = os.read(sys.stdin.fileno(), 1).decode()
                if char in ['\r', '\n']:
                    cmd = self.input_buffer.strip().lower()
                    self.input_buffer = ""
                    
                    # 조그 구동 제어 명령어
                    if cmd == 'j':
                        self.target_voltage = self.JOG_VOLTAGE_STEP
                        print(f"\n▶ [JOG] 굽힘 구동 시작 (+{self.target_voltage} mV)")
                    elif cmd == 'k':
                        self.target_voltage = -self.JOG_VOLTAGE_STEP
                        print(f"\n▶ [JOG] 펼침 구동 시작 ({self.target_voltage} mV)")
                    elif cmd == 's':
                        self.target_voltage = 0.0
                        print("\n🛑 [JOG] 즉시 구동 정지 (0 mV)")
                        
                    # 한계 좌표 확정 명령어
                    elif cmd == 'a':
                        print(f"\n🟩 [CALIB] 일자 정렬(Align) 값 등록 완료 ➡️  self.ALIGN_RAW_COUNT = {self.current_raw_count}")
                    elif cmd == 'f':
                        print(f"\n🟨 [CALIB] 최대 굽힘(Flex) 한계 등록 완료 ➡️  self.MAX_FLEX_LIMIT = {self.current_raw_count}")
                    elif cmd == 'e':
                        print(f"\n🟥 [CALIB] 최대 펼침(Ext) 한계 등록 완료 ➡️  self.MAX_EXT_LIMIT = {self.current_raw_count}")
                    break
                elif char in ['\x08', '\x7f']:
                    if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                else:
                    self.input_buffer += char
            except: pass

        # SDO 데이터 수집 (위치 및 상태 데이터 요청)
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.status_obj))
        
        timeout_end = time.monotonic() + 0.008
        while time.monotonic() < timeout_end:
            try:
                frame = self.bus.recv(timeout=0.001)
                if frame is None: continue
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.value is not None and sdo_res.node_id == self.TARGET_NODE_ID:
                    val = sdo_res.value
                    if val > 0x7FFFFFFF: val -= 0x100000000
                    if sdo_res.index == 0x6064: 
                        self.current_raw_count = val
                    elif sdo_res.index == 0x6041: 
                        self.status_word = val
            except: pass

        # 설정된 조그 전압을 실시간으로 SDO 송신 (지속 명령)
        try:
            self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(self.TARGET_NODE_ID, int(self.target_voltage), self.TARGET_AXIS))
        except: pass

        # 터미널 실시간 출력 상태창
        sys.stdout.write(f"\r [Out: {int(self.target_voltage):4d} mV] | 현재 엔코더: {self.current_raw_count:6d} | 입력창 ➡️ {self.input_buffer}      ")
        sys.stdout.flush()

    def shutdown_hook(self):
        self.get_logger().warn("\n⚠️ 안전 장치: 모터 전압 완전 차단(0mV)")
        try:
            self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(self.TARGET_NODE_ID, 0, self.TARGET_AXIS))
            time.sleep(0.05)
        except: pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.can_bus_context.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    node = KitechHardwareCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        rclpy.shutdown()

if __name__ == '__main__':
    main()