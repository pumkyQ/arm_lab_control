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
# [⚙️ 패키지 라이브러리 경로 자동 추가]
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

AUTO_STAIR_TEST = False  
CAN_CHANNEL = "can0" 

def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class KitechJointController(Node):
    def __init__(self):
        super().__init__('kitech_joint_controller_node')
        
        # ----------------------------------------------------------------------
        # [⚙️ 하드웨어 물리 상수 및 한계치]
        # ----------------------------------------------------------------------
        self.TARGET_NODE_ID = 2
        self.TARGET_AXIS = 1     
        
        self.ALIGN_RAW_COUNT = 510     
        self.MAX_FLEX_LIMIT = 1619     
        self.MAX_EXT_LIMIT = -500     
        self.PULSES_PER_DEGREE = 11.378  # 4096 / 360
        
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO                                         
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi) 
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN            
        
        self.voltage_limit = 9500.0 # 최대 인가 전압 한계 (mV)
        
        # ----------------------------------------------------------------------
        # [🎯 정석 제어 (PID + FF) 파라미터 세팅]
        # ----------------------------------------------------------------------
        self.LOOP_RATE = 50.0            
        self.dt = 1.0 / self.LOOP_RATE

        # 정밀 불감대 및 오차 판정 기준 (0.5도)
        self.DEADZONE_DEG = 0.5
        self.DEADZONE_THRESH_COUNT = self.DEADZONE_DEG * self.PULSES_PER_DEGREE

        # 오차 상태 변수 초기화 (I제어 및 미분 유도용)
        self.error_integral = 0.0
        self.prev_error_count = 0.0
        
        # 제어 상태 변수
        self.current_raw_count = self.ALIGN_RAW_COUNT
        self.current_velocity_raw = 0.0
        self.actual_current_ma = 0.0
        self.status_word = 0
        self.last_loop_time = time.monotonic()
        self.cycle_time_ms = 0.0
        self.input_buffer = ""
        
        # 터미널 상태 백업
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ROS 2 Publisher
        self.pub_status = self.create_publisher(Float64MultiArray, '/joint_control_status', 10)

        # 실험 모드 변수 초기화
        self.test_timer_ticks = 0
        self.STEP_INTERVAL_SEC = 2.5     
        self.TICKS_PER_STEP = int(self.STEP_INTERVAL_SEC * self.LOOP_RATE)
        self.is_hardware_ready = False   
        self.init_retry_counter = 0      
        
        self.start_deg = (self.MAX_FLEX_LIMIT - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        
        if AUTO_STAIR_TEST:
            self.current_mode = 1            
            self.current_step_deg = self.start_deg
            self.target_raw_count = float(self.MAX_FLEX_LIMIT) 
            self.test_state = "SWEEP"        
            self.apply_mode_parameters()     
        else:
            self.current_mode = 1
            self.target_raw_count = float(self.ALIGN_RAW_COUNT)
            self.test_state = "READY"
            self.apply_mode_parameters()

        # 데이터 로깅 설정
        self.log_dir = os.path.join(current_dir, "log_data")
        if not os.path.exists(self.log_dir): os.makedirs(self.log_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.log_dir, f"control_log_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        header = ["Timestamp", "Target_Deg", "Real_Deg", "Error_Deg", "Voltage_mV", 
                  "Raw_Vel", "Filtered_Vel", "V_EMF", "Control_Mode"]
        self.csv_writer.writerow(header)

        # CiA402 통신 초기화
        self.protocol = Cia402Protocol()
        self.pos_obj = Cia402Object(0x6064)
        self.vel_obj = Cia402Object(0x606c)
        self.status_obj = Cia402Object(0x6041)
        self.curr_obj = Cia402Object(0x6078)  
        self.err_obj = Cia402Object(0x603f)   

        try:
            self.can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
            self.bus = self.can_bus_context.__enter__()
            self.get_logger().info(f"✅ CAN 인터페이스 연결 성공: {CAN_CHANNEL}")
        except Exception as e:
            self.get_logger().error(f"❌ CAN 연결 실패: {e}")
            self.bus = None

        if not AUTO_STAIR_TEST:
            self.sub_target = self.create_subscription(Float64, '/joint_target_rad', self.callback_target_rad, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def apply_mode_parameters(self):
        """
        🚀 [정석 튜닝 카테고리] 
        정석 PID 알고리즘 구조에서는 안착 능력 향상을 위해 소량의 Ki(적분 게인)를 도입합니다.
        Ki_limit은 Anti-Windup(적분 누적 방지) 한계선입니다.
        """
        if self.current_mode == 1: # 기본 응답 모드
            self.Kp = 350.0
            self.Kd = 15.0         
            self.Ki = 0.5          
            self.Ki_limit = 500.0  # mV 한계
            self.get_logger().info("⚙️ 모드 1: 표준 PID 제어 세팅 완료")
            
        elif self.current_mode == 2: # 고강성 모드 (추종 성능 극대화)
            self.Kp = 400.0
            self.Kd = 20.0
            self.Ki = 1.5
            self.Ki_limit = 800.0
            self.get_logger().info("⚙️ 모드 2: 고강성 PID 제어 세팅 완료")
            
        elif self.current_mode == 3: # 컴플라이언트 모드 (유연 제어)
            self.Kp = 200.0
            self.Kd = 10.0
            self.Ki = 0.1
            self.Ki_limit = 200.0
            self.get_logger().info("⚙️ 모드 3: 유연 저게인 PID 제어 세팅 완료")
        
        # 모드가 변경되면 기존 적분 오차 잔여물 리셋
        self.error_integral = 0.0

    def init_can_hardware(self):
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)
        self.bus.send(self.protocol.make_axis_mode_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, -11))
        time.sleep(0.05)

        for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                     Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, ctrl))
            time.sleep(0.03)

        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        boot_success = False
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=0.01)
            if frame and frame.can_id == 0x580 + self.TARGET_NODE_ID:
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                    val = sdo_res.value
                    if val > 0x7FFFFFFF: val -= 0x100000000
                    self.current_raw_count = val
                    self.prev_error_count = 0.0
                    boot_success = True
                    break
                    
        if not boot_success: raise RuntimeError("CAN Booting Failed")

    def callback_target_rad(self, msg):
        if AUTO_STAIR_TEST: return
        target_degree = msg.data * (180.0 / np.pi)
        requested_count = self.ALIGN_RAW_COUNT + (target_degree * self.PULSES_PER_DEGREE)
        self.update_target_with_limit(requested_count)

    def update_target_with_limit(self, requested_count):
        if requested_count > self.MAX_FLEX_LIMIT: self.target_raw_count = float(self.MAX_FLEX_LIMIT)
        elif requested_count < self.MAX_EXT_LIMIT: self.target_raw_count = float(self.MAX_EXT_LIMIT)
        else: self.target_raw_count = requested_count

    def control_loop(self):
        if self.bus is None: return

        if not self.is_hardware_ready:
            if self.init_retry_counter % 100 == 0:
                try:
                    self.init_can_hardware()
                    self.is_hardware_ready = True
                    self.get_logger().info("✅ 하드웨어 준비 완료!")
                except Exception as e:
                    dummy_msg = Float64MultiArray()
                    dummy_msg.data = [0.0] * 8
                    self.pub_status.publish(dummy_msg)
            self.init_retry_counter += 1
            return

        current_time = time.monotonic()
        self.cycle_time_ms = (current_time - self.last_loop_time) * 1000.0
        self.last_loop_time = current_time

        # [자동 스캔/수동 키보드 시퀀스 핸들러 상단 유지 (기존과 동일)]
        if AUTO_STAIR_TEST:
            self.test_timer_ticks += 1
            if self.test_state == "SWEEP" and self.test_timer_ticks >= self.TICKS_PER_STEP:
                self.test_timer_ticks = 0
                final_error_count = self.target_raw_count - self.current_raw_count
                final_error_deg = final_error_count / self.PULSES_PER_DEGREE
                real_deg = (self.current_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
                self.get_logger().info(f"📊 [MODE {self.current_mode}] 목표: {int(self.target_raw_count)} -> 오차: {final_error_deg:+.4f}°")
                self.current_step_deg -= 5.0
                requested_count = self.ALIGN_RAW_COUNT + (self.current_step_deg * self.PULSES_PER_DEGREE)
                if requested_count <= self.MAX_EXT_LIMIT:
                    self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                    self.test_state = "RETURN_TO_START"
                else:
                    self.update_target_with_limit(requested_count)
            elif self.test_state == "RETURN_TO_START" and self.test_timer_ticks >= int(3.0 * self.LOOP_RATE):
                self.test_timer_ticks = 0
                self.current_mode += 1
                if self.current_mode > 3:
                    self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                    globals()['AUTO_STAIR_TEST'] = False 
                    self.test_state = "READY"
                else:
                    self.apply_mode_parameters()
                    self.current_step_deg = self.start_deg
                    self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                    self.test_state = "SWEEP"
        else:
            while kbhit():
                try:
                    char = os.read(sys.stdin.fileno(), 1).decode()
                    if char in ['\r', '\n']:
                        user_input = self.input_buffer.strip().lower()
                        self.input_buffer = ""
                        if user_input.startswith('m'):
                            mode_num = int(user_input[1:])
                            if 1 <= mode_num <= 3: self.current_mode = mode_num; self.apply_mode_parameters()
                        elif user_input:
                            target_degree = float(user_input)
                            self.update_target_with_limit(self.ALIGN_RAW_COUNT + (target_degree * self.PULSES_PER_DEGREE))
                        break
                    elif char in ['\x08', '\x7f']:
                        if len(self.input_buffer) > 0: self.input_buffer = self.input_buffer[:-1]
                    else: self.input_buffer += char
                except: pass

        # SDO 통신 수집 (기존 동일)
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.vel_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.status_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.curr_obj))
        
        timeout_end = time.monotonic() + 0.004
        while time.monotonic() < timeout_end:
            try:
                frame = self.bus.recv(timeout=0.001)
                if frame is None: continue
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.value is not None and sdo_res.node_id == self.TARGET_NODE_ID:
                    val = sdo_res.value
                    if val > 0x7FFFFFFF: val -= 0x100000000
                    if sdo_res.index == 0x6064: self.current_raw_count = val
                    elif sdo_res.index == 0x606c: self.current_velocity_raw = val
                    elif sdo_res.index == 0x6041: self.status_word = val
            except: pass

        if (self.status_word & 0x08): 
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.FAULT_RESET))
            return
        elif not (self.status_word & 0x04):
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.SWITCH_ON))
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.ENABLE_OPERATION))
            return

        # =========================================================================
        # 🎯 [이 부분이 정답형 PID + Feedforward 산업용 표준 알고리즘 핵심]
        # =========================================================================
        error_count = self.target_raw_count - self.current_raw_count
        
        # 1) 미분 성분 (D-term): Target의 급격한 변동에 튀지 않도록 오차의 차분(미분) 대신 
        #    실제 물리적 측정 속도를 활용하는 'Derivative on Feedback' 방식을 채택합니다.
        v_d = -self.Kd * self.current_velocity_raw
        
        # 2) 불감대(Dead Zone) 및 적분 성분(I-term) 처리
        if abs(error_count) <= self.DEADZONE_THRESH_COUNT:
            # 1도 이내 안착 시 오차 축적을 정지하고 수렴력을 유지하기 위해 I-term을 보존 또는 감쇄시킵니다.
            # 출력을 무작정 0으로 끊으면 채터링이 생기므로, D-term(댐퍼)만 남겨 감쇄시킵니다.
            self.error_integral *= 0.95  # 서서히 잔여 적분 오차 소거 (Anti-chattering)
            v_p = 0.0
        else:
            # 불감대 밖에서만 정상적으로 P 제어 및 오차 적분 수행
            v_p = self.Kp * error_count
            self.error_integral += error_count * self.dt
            # Anti-Windup 클리핑 기법 적용
            self.error_integral = max(-self.Ki_limit/self.Ki, min(self.Ki_limit/self.Ki, self.error_integral))

        v_i = self.Ki * self.error_integral

        # 3) 속도 피드포워드 (Back-EMF 전방 보상)
        v_emf = self.K_emf_count * self.current_velocity_raw

        # 4) 토탈 제어 출력 합성
        total_voltage = v_p + v_i + v_d + v_emf

        # 5) 물리적 한계점 하드웨어 보호 클리핑
        if (self.current_raw_count >= self.MAX_FLEX_LIMIT and total_voltage > 0) or \
           (self.current_raw_count <= self.MAX_EXT_LIMIT and total_voltage < 0):
            total_voltage = 0.0
            self.error_integral = 0.0  # 벽에 부딪혔을 땐 적분 초기화
            
        clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
        self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(self.TARGET_NODE_ID, int(clamped_voltage), self.TARGET_AXIS))

        # 데이터 업데이트 및 저장 처리 (기존과 동일)
        self.prev_error_count = error_count
        real_deg = (self.current_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        cmd_deg = (self.target_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        error_deg = error_count / self.PULSES_PER_DEGREE
        
        log_row = [cmd_deg, real_deg, error_deg, clamped_voltage, float(self.current_velocity_raw), float(self.current_velocity_raw), v_emf, float(self.current_mode)]
        status_msg = Float64MultiArray(); status_msg.data = log_row; self.pub_status.publish(status_msg)
        self.csv_writer.writerow([datetime.now().strftime("%H:%M:%S.%f")[:-3]] + log_row)

        sys.stdout.write(f"\r [M{self.current_mode}] [Cycle: {self.cycle_time_ms:4.1f}ms] | Target:{int(self.target_raw_count)} | Act:{self.current_raw_count} | V:{clamped_voltage:4.0f}mV | Stat:{hex(self.status_word)}       ")
        sys.stdout.flush()

    def shutdown_hook(self):
        try:
            self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(self.TARGET_NODE_ID, 0, self.TARGET_AXIS))
            time.sleep(0.05)
        except: pass
        finally:
            if hasattr(self, 'csv_file'): self.csv_file.close()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.can_bus_context.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    node = KitechJointController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: node.shutdown_hook(); rclpy.shutdown()

if __name__ == '__main__':
    main()