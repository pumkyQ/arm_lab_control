#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import select
import termios
import tty
import csv
from datetime import datetime
import os
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray

# =========================================================================
# [⚙️ 해결 방법 1: 패키지 라이브러리 경로 절대 추적 알고리즘 반영]
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__)) # real_driver 폴더
workspace_dir = os.path.dirname(current_dir)             # arm_lab_control 폴더

# 1. kitech_v1 폴더 내부를 직접 참조할 수 있도록 경로 등록
kitech_path = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path:
    sys.path.append(kitech_path)

# 2. 혹시 모를 상위 패키지 참조 형태 분기를 위해 workspace 자체도 함께 등록
if workspace_dir not in sys.path:
    sys.path.append(workspace_dir)

# =========================================================================
# [📡 하위 모듈 Import] - 경로 등록 직후 수행해야 정상 로드됩니다.
# =========================================================================
try:
    from motor_control.can_bus import SocketCanBus
    from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword
except ModuleNotFoundError as e:
    print("\n❌ 여전히 패키지를 찾지 못했습니다. 디렉토리 구조를 확인해 주세요!")
    print(f"현재 탐색된 Workspace: {workspace_dir}")
    print(f"현재 탐색된 kitech_v1 경로: {kitech_path}\n")
    raise e

CAN_CHANNEL = "can0" 

def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class KitechMultiJointController(Node):
    def __init__(self):
        super().__init__('kitech_multi_joint_controller_node')
        
        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 물리 상수 및 조인트별 측정 데이터 매핑]
        # ----------------------------------------------------------------------
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO                                         
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi) 
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN            
        self.voltage_limit = 9500.0      # 최대 인가 전압 한계 (mV)
        
        # 조인트별 하드웨어 정보 매핑 데이터 테이블 반영 (Node 1, 2, 3)
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': -1250.0, 'FLEX': -540.0, 'EXT': -2000.0},
            'j2': {'NODE_ID': 2, 'AXIS': 1, 'ALIGN': 510.0,  'FLEX': 1619.0, 'EXT': -290.0}, 
            'j3': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -337.0, 'FLEX': 824.0,  'EXT': -1313.0} 
        }
        
        # 제어 게인 및 불감대 세팅 (0.5도 요청 반영)
        self.Kp = 350.0
        self.Kd = 15.0         
        self.Ki = 0.5          
        self.Ki_limit = 500.0  
        self.DEADZONE_DEG = 0.5
        self.DEADZONE_THRESH_COUNT = self.DEADZONE_DEG * self.PULSES_PER_DEGREE

        self.LOOP_RATE = 50.0            
        self.dt = 1.0 / self.LOOP_RATE
        
        # ----------------------------------------------------------------------
        # [🔄 실시간 상태 변수 초기화]
        # ----------------------------------------------------------------------
        self.active_mode = 'j1'  
        self.input_buffer = ""
        self.last_loop_time = time.monotonic()
        self.cycle_time_ms = 0.0
        self.is_hardware_ready = False
        self.init_retry_counter = 0

        # 각 조인트 상태 트래킹용 딕셔너리
        self.joint_states = {}
        for j_key in ['j1', 'j2', 'j3']:
            self.joint_states[j_key] = {
                'target_count': self.JOINT_CONFIG[j_key]['ALIGN'], 
                'current_count': self.JOINT_CONFIG[j_key]['ALIGN'],
                'velocity_raw': 0.0,
                'status_word': 0,
                'error_integral': 0.0
            }
        
        # 터미널 상태 백업
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ROS 2 Publisher & Subscriber
        self.pub_status = self.create_publisher(Float64MultiArray, '/multi_joint_status', 10)
        self.sub_j1 = self.create_subscription(Float64, '/j1_target_rad', lambda msg: self.ros_callback(msg, 'j1'), 10)
        self.sub_j2 = self.create_subscription(Float64, '/j2_target_rad', lambda msg: self.ros_callback(msg, 'j2'), 10)
        self.sub_j3 = self.create_subscription(Float64, '/j3_target_rad', lambda msg: self.ros_callback(msg, 'j3'), 10)

        # CiA402 통신 객체 할당
        self.protocol = Cia402Protocol()
        self.pos_obj = Cia402Object(0x6064)
        self.vel_obj = Cia402Object(0x606c)
        self.status_obj = Cia402Object(0x6041)

        try:
            self.can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
            self.bus = self.can_bus_context.__enter__()
            self.get_logger().info(f"✅ CAN 연결 성공: {CAN_CHANNEL}")
        except Exception as e:
            self.get_logger().error(f"❌ CAN 연결 실패: {e}")
            self.bus = None

        self.timer = self.create_timer(self.dt, self.control_loop)

    def init_all_joints_hardware(self):
        """3개 노드의 하드웨어를 순차적으로 CiA402 통신 활성화"""
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.05)

        for j_key, cfg in self.JOINT_CONFIG.items():
            node = cfg['NODE_ID']
            axis = cfg['AXIS']
            self.bus.send(self.protocol.make_axis_mode_sdo(node, axis, -11)) 
            time.sleep(0.02)
            for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                         Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
                self.bus.send(self.protocol.make_axis_controlword_sdo(node, axis, ctrl))
                time.sleep(0.02)

        # 각 노드의 초기 엔코더 위치 실시간 동기화
        for j_key, cfg in self.JOINT_CONFIG.items():
            self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], self.pos_obj))
            time.sleep(0.02)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                frame = self.bus.recv(timeout=0.005)
                if frame and frame.can_id == 0x580 + cfg['NODE_ID']:
                    sdo_res = self.protocol.parse_sdo_response(frame)
                    if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                        val = sdo_res.value
                        if val > 0x7FFFFFFF: val -= 0x100000000
                        self.joint_states[j_key]['current_count'] = float(val)
                        self.joint_states[j_key]['target_count'] = float(val) 
                        break

    def ros_callback(self, msg, joint_key):
        """ROS 라디안 토픽 각도 제어 변환"""
        target_degree = msg.data * (180.0 / np.pi)
        cfg = self.JOINT_CONFIG[joint_key]
        requested_count = cfg['ALIGN'] + (target_degree * self.PULSES_PER_DEGREE)
        self.update_target_with_limits(joint_key, requested_count)

    def update_target_with_limits(self, joint_key, requested_count):
        """소프트웨어 가드 한계 예외 처리"""
        cfg = self.JOINT_CONFIG[joint_key]
        min_lim = min(cfg['FLEX'], cfg['EXT'])
        max_lim = max(cfg['FLEX'], cfg['EXT'])
        
        if requested_count < min_lim: clamped = min_lim
        elif requested_count > max_lim: clamped = max_lim
        else: clamped = requested_count
        
        self.joint_states[joint_key]['target_count'] = float(clamped)

    def control_loop(self):
        if self.bus is None: return

        if not self.is_hardware_ready:
            if self.init_retry_counter % 50 == 0:
                try:
                    self.init_all_joints_hardware()
                    self.is_hardware_ready = True
                    self.get_logger().info("🔥 전체 멀티 관절 노드 준비 완료! (j1, j2, j3 모드 대기)")
                except Exception as e: pass
            self.init_retry_counter += 1
            return

        self.cycle_time_ms = (time.monotonic() - self.last_loop_time) * 1000.0
        self.last_loop_time = time.monotonic()

        # ⌨️ 터미널 키보드 입력 인터페이스 핸들러
        while kbhit():
            try:
                char = os.read(sys.stdin.fileno(), 1).decode()
                if char in ['\r', '\n']:
                    user_input = self.input_buffer.strip().lower()
                    self.input_buffer = ""
                    
                    if user_input in ['j1', 'j2', 'j3']:
                        self.active_mode = user_input
                        self.get_logger().info(f"🔄 제어 모드가 변경되었습니다 ➡️ [{self.active_mode.upper()} 모드]")
                    elif user_input:
                        try:
                            target_degree = float(user_input)
                            cfg = self.JOINT_CONFIG[self.active_mode]
                            requested_count = cfg['ALIGN'] + (target_degree * self.PULSES_PER_DEGREE)
                            self.update_target_with_limits(self.active_mode, requested_count)
                        except ValueError: pass
                    break
                elif char in ['\x08', '\x7f']:
                    if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                else: self.input_buffer += char
            except: pass

        # SDO 데이터 일괄 수집
        for j_key, cfg in self.JOINT_CONFIG.items():
            self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], self.pos_obj))
            self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], self.vel_obj))
            self.bus.send(self.protocol.make_sdo_read(cfg['NODE_ID'], self.status_obj))
            
        timeout_end = time.monotonic() + 0.006
        while time.monotonic() < timeout_end:
            try:
                frame = self.bus.recv(timeout=0.001)
                if frame is None: continue
                node_id = frame.can_id - 0x580
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.value is not None:
                    for j_key, cfg in self.JOINT_CONFIG.items():
                        if cfg['NODE_ID'] == node_id:
                            val = sdo_res.value
                            if val > 0x7FFFFFFF: val -= 0x100000000
                            if sdo_res.index == 0x6064: self.joint_states[j_key]['current_count'] = float(val)
                            elif sdo_res.index == 0x606c: self.joint_states[j_key]['velocity_raw'] = float(val)
                            elif sdo_res.index == 0x6041: self.joint_states[j_key]['status_word'] = val
            except: pass

        # 3개 관절 독립 병렬 PI-D + Feedforward 연산 및 명령 송신
        ros_log_data = []
        for j_key, cfg in self.JOINT_CONFIG.items():
            state = self.joint_states[j_key]
            
            if (state['status_word'] & 0x08):
                self.bus.send(self.protocol.make_axis_controlword_sdo(cfg['NODE_ID'], cfg['AXIS'], Cia402Controlword.FAULT_RESET))
                continue

            error_count = state['target_count'] - state['current_count']
            
            # 1) 미분 성분 (Derivative on Feedback)
            v_d = -self.Kd * state['velocity_raw']
            
            # 2) 0.5도 정밀 불감대 제어 및 적분 누수
            if abs(error_count) <= self.DEADZONE_THRESH_COUNT:
                state['error_integral'] *= 0.95
                v_p = 0.0
            else:
                v_p = self.Kp * error_count
                state['error_integral'] += error_count * self.dt
                state['error_integral'] = max(-self.Ki_limit/self.Ki, min(self.Ki_limit/self.Ki, state['error_integral']))
                
            v_i = self.Ki * state['error_integral']
            
            # 3) Back-EMF Feedforward 전방 보상
            v_emf = self.K_emf_count * state['velocity_raw']
            
            # 4) 전압 합성 및 물리 가드 한계 안전 조치
            total_voltage = v_p + v_i + v_d + v_emf
            
            min_lim = min(cfg['FLEX'], cfg['EXT'])
            max_lim = max(cfg['FLEX'], cfg['EXT'])
            if (state['current_count'] >= max_lim and total_voltage > 0) or \
               (state['current_count'] <= min_lim and total_voltage < 0):
                total_voltage = 0.0
                state['error_integral'] = 0.0

            clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
            self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(cfg['NODE_ID'], int(clamped_voltage), cfg['AXIS']))

            real_deg = (state['current_count'] - cfg['ALIGN']) / self.PULSES_PER_DEGREE
            ros_log_data.extend([real_deg, clamped_voltage])

        # ROS 상태 토픽 발행
        status_msg = Float64MultiArray(); status_msg.data = ros_log_data; self.pub_status.publish(status_msg)

        # 인터페이스 리프레시 출력
        sys.stdout.write(
            f"\r 🟢 [현재모드: {self.active_mode.upper()}] | "
            f"J1_deg: {(self.joint_states['j1']['current_count']-self.JOINT_CONFIG['j1']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"J2_deg: {(self.joint_states['j2']['current_count']-self.JOINT_CONFIG['j2']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"J3_deg: {(self.joint_states['j3']['current_count']-self.JOINT_CONFIG['j3']['ALIGN'])/self.PULSES_PER_DEGREE:+.1f}° | "
            f"입력창 ➡️ {self.input_buffer}"
        )
        sys.stdout.flush()

    def shutdown_hook(self):
        for j_key, cfg in self.JOINT_CONFIG.items():
            try: self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(cfg['NODE_ID'], 0, cfg['AXIS']))
            except: pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        if self.bus: self.can_bus_context.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    node = KitechMultiJointController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.shutdown_hook(); rclpy.shutdown()

if __name__ == '__main__':
    main()