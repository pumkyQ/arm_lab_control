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
# [⚙️ 프로젝트 구조 kitech_v1 패키지 라이브러리 참조 경로 자동 추가]
# =========================================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, "kitech_v1"))

from motor_control.can_bus import SocketCanBus
from motor_control.cia402 import Cia402Protocol, Cia402Object, Cia402Controlword

# =========================================================================
# [실험 및 테스트 모드 설정]
# AUTO_STAIR_TEST -> True : 3개 모드 순차 반복 스캔 모드 (모드 종료 후 -651 복귀)
#                     False: 외부 ROS 2 토픽(/joint_target_rad) 수신 모드
# =========================================================================
AUTO_STAIR_TEST = False  

def kbhit():
    """터미널 키보드 입력 여부를 비동기 체크하는 헬퍼 함수"""
    dr, dw, de = select.select([sys.stdin], [], [], 0.0)
    return len(dr) > 0

class KitechJointController(Node):
    def __init__(self):
        super().__init__('kitech_joint_controller_node')
        
        # =========================================================================
        # [⚙️ 하드웨어 기본 한계치 세팅]
        # =========================================================================
        self.TARGET_NODE_ID = 1  
        self.TARGET_AXIS = 1     
        
        self.ALIGN_RAW_COUNT = -980      # 1자 정렬 기준 엔코더 원점 값 (0.0°)
        self.MAX_FLEX_LIMIT = -651       # 최대 굽힘 한계 (카운트 상한선, 약 +28.63°)
        self.MAX_EXT_LIMIT = -2014       # 최대 펼침 한계 (카운트 하한선, 약 -90.0°)
        self.PULSES_PER_DEGREE = 11.489  # 1도당 펄스 수
        
        self.GEAR_RATIO = 406.4
        self.K_emf_rad = 8.632 * self.GEAR_RATIO                                         
        self.COUNTS_PER_RADIAN = self.PULSES_PER_DEGREE * (180.0 / np.pi) 
        self.K_emf_count = self.K_emf_rad / self.COUNTS_PER_RADIAN            
        
        self.voltage_limit = 9500.0      
        self.ERROR_THRESH_COUNT = 2.0    
        self.LPF_ALPHA = 0.25            # 필터 지연을 더 줄여 Kd가 역방향 변화에 즉각 반응하도록 수정
        
        self.LOOP_RATE = 50.0            # 50 Hz 제어
        self.dt = 1.0 / self.LOOP_RATE

        # 제어 상태 변수 초기화
        self.current_raw_count = self.ALIGN_RAW_COUNT
        self.current_velocity_raw = 0.0
        self.filtered_velocity_old = 0.0  
        self.actual_current_ma = 0.0
        self.error_register = 0
        self.status_word = 0
        self.input_buffer = ""
        
        # 터미널 설정 저장 및 cbreak 모드 설정 (엔터 없이 키 입력 즉시 감지)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ----------------------------------------------------------------------
        # [🆕 3가지 모드 제어 및 다중 시퀀스 변수 초기화]
        # ----------------------------------------------------------------------
        self.test_timer_ticks = 0
        self.STEP_INTERVAL_SEC = 2.5     # 각 각도별 데이터 안착 대기 시간
        self.TICKS_PER_STEP = int(self.STEP_INTERVAL_SEC * self.LOOP_RATE)
        
        self.start_deg = (self.MAX_FLEX_LIMIT - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        
        if AUTO_STAIR_TEST:
            self.current_mode = 1            # 1모드부터 시작 (1 -> 2 -> 3)
            self.current_step_deg = self.start_deg
            self.target_raw_count = float(self.MAX_FLEX_LIMIT) 
            self.test_state = "SWEEP"        # 초기 상태: 5도씩 계단식 스캔 주행
            self.apply_mode_parameters()     # 1모드 파라미터 적용
        else:
            self.current_mode = 1
            self.target_raw_count = float(self.ALIGN_RAW_COUNT)
            self.test_state = "READY"
            self.apply_mode_parameters()
            self.get_logger().info("⌨️ [수동 모드] 각도(deg) 또는 모드(m1, m2, m3)를 입력하세요.")
            self.get_logger().info("예: '10' 입력 시 10도로 이동, 'm2' 입력 시 고강성 모드 변경")

        # ----------------------------------------------------------------------
        # [💾 자동 데이터 로깅 설정]
        # ----------------------------------------------------------------------
        self.log_dir = os.path.join(current_dir, "log_data")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # 파일명에 날짜와 시간을 포함
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.log_dir, f"control_log_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV 헤더 작성
        header = ["Timestamp", "Target_Deg", "Real_Deg", "Error_Deg", "Voltage_mV", 
                  "Raw_Vel", "Filtered_Vel", "V_EMF", "Control_Mode"]
        self.csv_writer.writerow(header)
        self.get_logger().info(f"💾 데이터 기록 시작: {self.csv_filename}")

        # Protocol 및 Object 초기화
        self.protocol = Cia402Protocol()
        self.pos_obj = Cia402Object(0x6064)
        self.vel_obj = Cia402Object(0x606c)
        self.status_obj = Cia402Object(0x6041)
        self.curr_obj = Cia402Object(0x6078)  
        self.err_obj = Cia402Object(0x603f)   

        # SocketCAN 컨텍스트 매니저 수동 진입
        self.can_bus_context = SocketCanBus('can0', receive_timeout=0.0)
        self.bus = self.can_bus_context.__enter__()

        # CANopen 부팅 확인
        self.init_can_hardware()

        # ROS 2 Pub/Sub
        self.pub_status = self.create_publisher(Float64MultiArray, '/joint_control_status', 10)
        
        if not AUTO_STAIR_TEST:
            self.sub_target = self.create_subscription(
                Float64, '/joint_target_rad', self.callback_target_rad, 10)
            self.get_logger().info("📥 ROS 2 토픽 제어 모드 활성화")
        else:
            self.get_logger().info(f"🚀 [다중 모드 자동 스캔] MODE 1 스위핑 시작 (지령: {self.MAX_FLEX_LIMIT} Count)")

        # 50Hz 타이머 가동
        self.timer = self.create_timer(self.dt, self.control_loop)

    def apply_mode_parameters(self):
        if self.current_mode == 1:
            self.Kp = 130            # 미세 오차 해결을 위해 130으로 절충
            self.Kd = 2.0            # 역방향 진동 억제를 위해 브레이크(Kd) 대폭 강화
            self.stiction_offset = 1500.0 # 마찰을 확실히 뚫기 위해 1.5V 수준으로 복구
            self.get_logger().info("⚙️ [PARAMETER] 모드 1 활성화: 기본 게인 제어 세팅 완료")
        elif self.current_mode == 2:
            self.Kp = 130      
            self.Kd = 3.0             # 고강성 모드에서도 Kd를 충분히 확보
            self.stiction_offset = 1500.0  
            self.get_logger().info("⚙️ [PARAMETER] 모드 2 활성화: 고강성(High-Gain) 제어 세팅 완료")
        elif self.current_mode == 3:
            self.Kp = 140     
            self.Kd = 3.0
            self.stiction_offset = 1350.0
            self.get_logger().info("⚙️ [PARAMETER] 모드 3 활성화: 저유연(Soft-Gain) 제어 세팅 완료")

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

        self.get_logger().info(f"⏳ [Booting] Node {self.TARGET_NODE_ID} 엔코더 확인 중...")
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        
        boot_success = False
        deadline = time.monotonic() + 2.0  
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=0.01)
            if frame and frame.can_id == 0x580 + self.TARGET_NODE_ID:
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                    val = sdo_res.value
                    if val > 0x7FFFFFFF: val -= 0x100000000 
                    self.current_raw_count = val
                    boot_success = True
                    break
                    
        if not boot_success:
            self.get_logger().error("❌ SDO 피드백 수신 실패. 전원이나 CAN 연결을 확인하세요.")
            raise RuntimeError("CAN Booting Failed")
            
        self.get_logger().info(f"✅ 절대 엔코더 위치 파싱 성공: [현재: {self.current_raw_count}]")

    def callback_target_rad(self, msg):
        if AUTO_STAIR_TEST: return
        target_degree = msg.data * (180.0 / np.pi)
        requested_count = self.ALIGN_RAW_COUNT + (target_degree * self.PULSES_PER_DEGREE)
        self.update_target_with_limit(requested_count)

    def update_target_with_limit(self, requested_count):
        if requested_count > self.MAX_FLEX_LIMIT:
            self.target_raw_count = float(self.MAX_FLEX_LIMIT)
        elif requested_count < self.MAX_EXT_LIMIT:
            self.target_raw_count = float(self.MAX_EXT_LIMIT)
        else:
            self.target_raw_count = requested_count

    def control_loop(self):
        # 1. 다중 모드 자동 스캔 시퀀스 상태 머신
        if AUTO_STAIR_TEST:
            self.test_timer_ticks += 1
            
            if self.test_state == "SWEEP":
                if self.test_timer_ticks >= self.TICKS_PER_STEP:
                    self.test_timer_ticks = 0
                    
                    final_error_count = self.target_raw_count - self.current_raw_count
                    final_error_deg = final_error_count / self.PULSES_PER_DEGREE
                    real_deg = (self.current_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
                    
                    self.get_logger().info(f"📊 [MODE {self.current_mode}] 목표: {int(self.target_raw_count)} count ({real_deg:+.1f}° 구간) -> 오차: {final_error_deg:+.4f}°")
                    
                    self.current_step_deg -= 5.0
                    requested_count = self.ALIGN_RAW_COUNT + (self.current_step_deg * self.PULSES_PER_DEGREE)
                    
                    if requested_count <= self.MAX_EXT_LIMIT:
                        self.get_logger().warn(f"📢 [SWEEP DONE] MODE {self.current_mode}의 전 구간 스캔이 끝났습니다. 다음 모드 준비를 시작합니다.")
                        self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                        self.test_state = "RETURN_TO_START"
                    else:
                        self.update_target_with_limit(requested_count)

            elif self.test_state == "RETURN_TO_START":
                RETURN_TICKS = int(3.0 * self.LOOP_RATE)
                if self.test_timer_ticks >= RETURN_TICKS:
                    self.test_timer_ticks = 0
                    self.current_mode += 1
                    
                    if self.current_mode > 3:
                        self.get_logger().info("🎉 [ALL EXPERIMENTS COMPLETE] 모든 복귀 및 오차 측정 실험이 완료되었습니다!")
                        globals()['AUTO_STAIR_TEST'] = False 
                        self.test_state = "READY"
                    else:
                        self.get_logger().info(f"↩️ 시작점(-651) 정렬 완료. [MODE {self.current_mode}]로 진입합니다.")
                        self.apply_mode_parameters()
                        self.current_step_deg = self.start_deg
                        self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                        self.test_state = "SWEEP"
        else:
            if kbhit():
                char = sys.stdin.read(1)
                if char == '\n':
                    user_input = self.input_buffer.strip().lower()
                    self.input_buffer = ""
                    
                    if user_input.startswith('m'): 
                        try:
                            mode_num = int(user_input[1:])
                            if 1 <= mode_num <= 3:
                                self.current_mode = mode_num
                                self.apply_mode_parameters()
                        except: pass
                    else: 
                        try:
                            target_degree = float(user_input)
                            requested_count = self.ALIGN_RAW_COUNT + (target_degree * self.PULSES_PER_DEGREE)
                            self.update_target_with_limit(requested_count)
                            self.get_logger().info(f"▶ 목표 각도 변경: {target_degree}°")
                        except ValueError: pass
                else:
                    self.input_buffer += char

        # 2. SDO 피드백 수집 (12ms 윈도우)
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.vel_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.status_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.curr_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.err_obj))
        
        timeout_end = time.monotonic() + 0.012 
        while time.monotonic() < timeout_end:
            frame = self.bus.recv(timeout=0.001)
            if frame is None: continue
            
            sdo_res = self.protocol.parse_sdo_response(frame)
            if sdo_res and sdo_res.value is not None and sdo_res.node_id == self.TARGET_NODE_ID:
                val = sdo_res.value
                if val > 0x7FFFFFFF: val -= 0x100000000
                
                if sdo_res.index == 0x6064: self.current_raw_count = val
                elif sdo_res.index == 0x606c: self.current_velocity_raw = val
                elif sdo_res.index == 0x6041: self.status_word = val
                elif sdo_res.index == 0x6078:
                    if val > 0x7FFF: val -= 0x10000
                    self.actual_current_ma = val
                elif sdo_res.index == 0x603f: self.error_register = val

        # 3. 드라이버 상태 체크 및 자동 복구
        if (self.status_word & 0x08): 
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.FAULT_RESET))
            return
        elif not (self.status_word & 0x04): 
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.SWITCH_ON))
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.ENABLE_OPERATION))
            return

        # 속도 LPF 연산
        filtered_velocity = (self.LPF_ALPHA * self.current_velocity_raw) + ((1.0 - self.LPF_ALPHA) * self.filtered_velocity_old)
        self.filtered_velocity_old = filtered_velocity
        
        error_count = self.target_raw_count - self.current_raw_count
        v_pd = (self.Kp * error_count) - (self.Kd * filtered_velocity)
        v_emf = self.K_emf_count * filtered_velocity
        
        # =========================================================================
        # [🛠️ 반영 완료: 고정밀 안착 제어 및 마찰 보상 알고리즘]
        # =========================================================================
        if abs(error_count) > self.ERROR_THRESH_COUNT:
            # 1단계: 오차가 클 때 - 정마찰력을 강력히 뚫어내며 타겟 추종
            v_stiction = self.stiction_offset * np.sign(error_count) 
            total_voltage = v_pd + v_stiction + v_emf
        else:
            # 2단계: 오차가 불감대(2 count) 이내일 때 - 댐핑 및 감소된 Tail 전압으로 소프트 안착
            active_direction = np.sign(filtered_velocity if filtered_velocity != 0 else error_count)
            
            if active_direction != 0:
                v_stiction_tail = (self.stiction_offset * 0.4) * active_direction # 50% -> 40%로 줄여 역방향 안착 시 헌팅 방지
            else:
                v_stiction_tail = 0.0
                
            total_voltage = (-self.Kd * filtered_velocity) + v_emf + v_stiction_tail

        # 소프트 리밋 확인
        if (self.current_raw_count >= self.MAX_FLEX_LIMIT and total_voltage > 0) or \
           (self.current_raw_count <= self.MAX_EXT_LIMIT and total_voltage < 0):
            total_voltage = 0.0
            
        clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
        
        tx_frame = self.protocol.make_q_axis_voltage_mv_sdo(
            node_id=self.TARGET_NODE_ID, voltage_mv=int(clamped_voltage), axis=self.TARGET_AXIS)
        self.bus.send(tx_frame)

        # 4. 상태 출력 및 데이터 퍼블리시 / 저장
        real_deg = (self.current_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        cmd_deg = (self.target_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        error_deg = error_count / self.PULSES_PER_DEGREE
        
        log_row = [
            cmd_deg, real_deg, error_deg, clamped_voltage,
            float(self.current_velocity_raw), filtered_velocity, v_emf, float(self.current_mode)
        ]
        
        status_msg = Float64MultiArray()
        status_msg.data = log_row
        self.pub_status.publish(status_msg)

        current_time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.csv_writer.writerow([current_time_str] + log_row)

        sys.stdout.write(f"\r [M{self.current_mode}] Target:{int(self.target_raw_count)} | Act:{self.current_raw_count} | V:{clamped_voltage:4.0f}mV | Cur:{self.actual_current_ma:4.0f}mA | Stat:{hex(self.status_word)} | Err:{hex(self.error_register)}")
        sys.stdout.flush()

    def shutdown_hook(self):
        self.get_logger().warn("\n⚠️ 안전 장치: 모터 전압 차단(0mV)")
        try:
            zero_frame = self.protocol.make_q_axis_voltage_mv_sdo(
                node_id=self.TARGET_NODE_ID, voltage_mv=0, axis=self.TARGET_AXIS)
            self.bus.send(zero_frame)
            time.sleep(0.05)
        except Exception as e:
            print(f"SDO 전송 실패: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            if hasattr(self, 'csv_file'):
                self.csv_file.close()
                self.get_logger().info(f"💾 데이터 로그가 저장되었습니다: {self.csv_filename}")
            self.can_bus_context.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    node = KitechJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()  # 터미널 복구 및 모터 정지 우선
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()