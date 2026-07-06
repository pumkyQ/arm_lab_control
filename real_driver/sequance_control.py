#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import sys
import os
import numpy as np
from std_msgs.msg import Float64MultiArray

# =========================================================================
# [⚙️ 패키지 라이브러리 경로 자동 추가]
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__)) 
workspace_dir = os.path.dirname(current_dir)             

kitech_path = os.path.join(workspace_dir, "kitech_v1")
if kitech_path not in sys.path: sys.path.append(kitech_path)
if workspace_dir not in sys.path: sys.path.append(workspace_dir)

try:
    from motor_control.can_bus import SocketCanBus
    from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword
except ModuleNotFoundError as e:
    print(f"\n❌ 패키지 탐색 실패: {workspace_dir} 구조를 확인해 주세요.\n")
    raise e

CAN_CHANNEL = "can0" 

class KitechPartialSequenceController(Node):
    def __init__(self):
        super().__init__('kitech_partial_sequence_controller_node')
        
        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 세팅 및 파라미터 매핑]
        # ----------------------------------------------------------------------
        self.PULSES_PER_DEGREE = 11.378  
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO                                         
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi) 
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN            
        self.voltage_limit = 9500.0      
        
        # 실측 데이터 반영 (Node 1, 2, 3)
        self.JOINT_CONFIG = {
            'j1': {'NODE_ID': 1, 'AXIS': 1, 'ALIGN': -980.0, 'FLEX': -651.0, 'EXT': -2014.0},
            'j2': {'NODE_ID': 2, 'AXIS': 1, 'ALIGN': 510.0,  'FLEX': 1619.0, 'EXT': -290.0}, # 뻑뻑함 -> 0도 고정용
            'j3': {'NODE_ID': 3, 'AXIS': 1, 'ALIGN': -337.0, 'FLEX': 824.0,  'EXT': -1313.0} 
        }
        
        # 제어 게인 및 불감대 세팅 (0.5도)
        self.Kp = 350.0
        self.Kd = 15.0         
        self.Ki = 0.5          
        self.Ki_limit = 500.0  
        self.DEADZONE_DEG = 0.5
        self.DEADZONE_THRESH_COUNT = self.DEADZONE_DEG * self.PULSES_PER_DEGREE

        self.LOOP_RATE = 50.0            
        self.dt = 1.0 / self.LOOP_RATE
        
        # ----------------------------------------------------------------------
        # [🔄 시퀀스 제어 상태기기(State Machine) 변수]
        # ----------------------------------------------------------------------
        self.current_state = "INIT_ALIGN"  
        self.state_start_time = time.monotonic()
        self.is_hardware_ready = False
        self.init_retry_counter = 0

        # 각 관절 제어 상태 데이터 공간
        self.joint_states = {}
        for j_key in ['j1', 'j2', 'j3']:
            self.joint_states[j_key] = {
                'target_count': self.JOINT_CONFIG[j_key]['ALIGN'], 
                'current_count': self.JOINT_CONFIG[j_key]['ALIGN'],
                'velocity_raw': 0.0,
                'status_word': 0,
                'error_integral': 0.0
            }

        self.pub_status = self.create_publisher(Float64MultiArray, '/multi_joint_status', 10)

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
                        break

    def update_target_degree(self, joint_key, degree):
        cfg = self.JOINT_CONFIG[joint_key]
        requested_count = cfg['ALIGN'] + (degree * self.PULSES_PER_DEGREE)
        
        min_lim = min(cfg['FLEX'], cfg['EXT'])
        max_lim = max(cfg['FLEX'], cfg['EXT'])
        clamped = max(min_lim, min(max_lim, requested_count))
        
        self.joint_states[joint_key]['target_count'] = float(clamped)

    def control_loop(self):
        if self.bus is None: return

        if not self.is_hardware_ready:
            if self.init_retry_counter % 50 == 0:
                try:
                    self.init_all_joints_hardware()
                    self.is_hardware_ready = True
                    self.state_start_time = time.monotonic()
                    self.get_logger().info("🔥 하드웨어 연결 완료! j1, j3 순차 구동 시퀀스를 시작합니다.")
                except: pass
            self.init_retry_counter += 1
            return

        # 📡 실시간 SDO 피드백 수집
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

        # ----------------------------------------------------------------------
        # ⚙️ [수정] 1, 3번 조인트 맞춤형 타임라인 상태 기기 (State Machine)
        # ----------------------------------------------------------------------
        elapsed_time = time.monotonic() - self.state_start_time

        if self.current_state == "INIT_ALIGN":
            # 모든 관절을 최초 1자(0도)로 정렬 및 고정
            self.update_target_degree('j1', 0.0)
            self.update_target_degree('j2', 0.0)
            self.update_target_degree('j3', 0.0)
            if elapsed_time > 3.0:  
                self.current_state = "MOVE_J1"
                self.state_start_time = time.monotonic()
                self.get_logger().info("➡️ [1단계] 조인트 1 구동 시작 (0° ➡️ 20°)")

        elif self.current_state == "MOVE_J1":
            # j1만 20도로 구동, 뻑뻑한 j2와 대기 중인 j3은 0도 유지
            self.update_target_degree('j1', -30.0)
            self.update_target_degree('j2', 0.0)
            self.update_target_degree('j3', 0.0)
            if elapsed_time > 2.5:  
                self.current_state = "MOVE_J3"
                self.state_start_time = time.monotonic()
                self.get_logger().info("➡️ [2단계] 조인트 3(Node3) 구동 시작 (0° ➡️ 40°)")

        elif self.current_state == "MOVE_J3":
            # j1은 20도 유지, 뻑뻑한 j2는 0도 고정, j3만 40도로 최종 구동
            self.update_target_degree('j1', -30.0)
            self.update_target_degree('j2', 0.0)
            self.update_target_degree('j3', -60.0)
            if elapsed_time > 3.0:
                self.current_state = "POSE_COMPLETE"
                self.get_logger().info("✅ [시퀀스 완료] j1=20°, j2=0°(고정), j3=40° 파지 포즈 수렴 완료.")

        elif self.current_state == "POSE_COMPLETE":
            # 수렴 상태에서 최종 타겟 각도 지속 고정 (토크 홀딩)
            self.update_target_degree('j1', -20.0)
            self.update_target_degree('j2', 0.0)
            self.update_target_degree('j3', -60.0)

        # ----------------------------------------------------------------------
        # 🎯 병렬 독립 PI-D 및 전압 명령 전송 연산
        # ----------------------------------------------------------------------
        ros_log_data = []
        for j_key, cfg in self.JOINT_CONFIG.items():
            state = self.joint_states[j_key]
            
            if (state['status_word'] & 0x08):
                self.bus.send(self.protocol.make_axis_controlword_sdo(cfg['NODE_ID'], cfg['AXIS'], Cia402Controlword.FAULT_RESET))
                continue

            error_count = state['target_count'] - state['current_count']
            v_d = -self.Kd * state['velocity_raw']
            
            if abs(error_count) <= self.DEADZONE_THRESH_COUNT:
                state['error_integral'] *= 0.95
                v_p = 0.0
            else:
                v_p = self.Kp * error_count
                state['error_integral'] += error_count * self.dt
                state['error_integral'] = max(-self.Ki_limit/self.Ki, min(self.Ki_limit/self.Ki, state['error_integral']))
                
            v_i = self.Ki * state['error_integral']
            v_emf = self.K_emf_count * state['velocity_raw']
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

        # 🖥️ 화면 모니터링 출력
        j1_now = (self.joint_states['j1']['current_count'] - self.JOINT_CONFIG['j1']['ALIGN']) / self.PULSES_PER_DEGREE
        j2_now = (self.joint_states['j2']['current_count'] - self.JOINT_CONFIG['j2']['ALIGN']) / self.PULSES_PER_DEGREE
        j3_now = (self.joint_states['j3']['current_count'] - self.JOINT_CONFIG['j3']['ALIGN']) / self.PULSES_PER_DEGREE
        
        sys.stdout.write(
            f"\r ⚙️ [상태: {self.current_state:14s}] | "
            f"J1: {j1_now:+.1f}°/20° | J2(고정): {j2_now:+.1f}°/0° | J3: {j3_now:+.1f}°/40°    "
        )
        sys.stdout.flush()

    def shutdown_hook(self):
        for j_key, cfg in self.JOINT_CONFIG.items():
            try: self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(cfg['NODE_ID'], 0, cfg['AXIS']))
            except: pass
        if self.bus: self.can_bus_context.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    node = KitechPartialSequenceController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.shutdown_hook(); rclpy.shutdown()

if __name__ == '__main__':
    main()