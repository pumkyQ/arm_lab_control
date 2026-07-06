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
#                    False: 외부 ROS 2 토픽(/joint_target_rad) 수신 모드
# =========================================================================
AUTO_STAIR_TEST = False  

# =========================================================================
# [🌐 통신 인터페이스 설정]
# 가상 환경 테스트 시 "vcan0", 실제 하드웨어 연결 시 "can0"
# =========================================================================
CAN_CHANNEL = "can0" 

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
        self.ERROR_THRESH_COUNT = 10.0    
        self.ERROR_THRESH_COUNT = 2.0    # 정밀 제어를 위해 데드밴드를 2카운트로 복원
        self.LPF_ALPHA = 0.25            # 속도 노이즈 제거 및 안정성 확보를 위해 LPF 재적용
        
        self.LOOP_RATE = 50.0            # 50 Hz 제어
        self.dt = 1.0 / self.LOOP_RATE

        # 제어 상태 변수 초기화
        self.current_raw_count = self.ALIGN_RAW_COUNT
        self.current_velocity_raw = 0.0
        self.filtered_velocity_old = 0.0 # LPF를 위한 이전 속도값 변수 추가
        self.actual_current_ma = 0.0
        self.error_register = 0
        self.status_word = 0
        self.last_loop_time = time.monotonic() # 주기 측정을 위한 이전 시간 저장
        self.cycle_time_ms = 0.0               # 측정된 주기를 ms 단위로 저장
        self.input_buffer = ""
        
        # 터미널 설정 저장 및 cbreak 모드 설정 (엔터 없이 키 입력 즉시 감지)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # ROS 2 Pub/Sub (하드웨어 초기화 전에 생성하여 PlotJuggler에서 즉시 보이도록 함)
        self.pub_status = self.create_publisher(Float64MultiArray, '/joint_control_status', 10)
        self.get_logger().info("📢 ROS 2 Publisher 활성화: /joint_control_status")

        # ----------------------------------------------------------------------
        # [🆕 3가지 모드 제어 및 다중 시퀀스 변수 초기화]
        # ----------------------------------------------------------------------
        self.test_timer_ticks = 0
        self.STEP_INTERVAL_SEC = 2.5     # 각 각도별 데이터 안착 대기 시간
        self.TICKS_PER_STEP = int(self.STEP_INTERVAL_SEC * self.LOOP_RATE)
        self.is_hardware_ready = False   # 하드웨어 준비 완료 플래그
        self.init_retry_counter = 0      # 초기화 재시도 간격 제어
        
        self.start_deg = (self.MAX_FLEX_LIMIT - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        
        if AUTO_STAIR_TEST:
            self.current_mode = 1            # 1모드부터 시작 (1 -> 2 -> 3)
            self.current_step_deg = self.start_deg
            self.target_raw_count = float(self.MAX_FLEX_LIMIT) 
            self.test_state = "SWEEP"        # 초기 상태: 5도씩 계단식 스캔 주행
            self.apply_mode_parameters()     # 1모드 파라미터 적용
        else:
            self.current_mode = 0
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
        
        # 파일명에 날짜와 시간을 포함 (예: control_log_20231027_153022.csv)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.log_dir, f"control_log_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV 헤더 작성 (status_msg.data와 매칭)
        header = ["Timestamp", "Target_Deg", "Real_Deg", "Error_Deg", "Voltage_mV", 
                  "Raw_Vel", "Filtered_Vel", "V_EMF", "Control_Mode"]
                  "Raw_Vel", "Filtered_Vel", "V_EMF", "Control_Mode", "Current_mA"]
        self.csv_writer.writerow(header)
        self.get_logger().info(f"💾 데이터 기록 시작: {self.csv_filename}")


        # Protocol 및 Object 초기화
        self.protocol = Cia402Protocol()
        self.pos_obj = Cia402Object(0x6064)
        self.vel_obj = Cia402Object(0x606c)
        self.status_obj = Cia402Object(0x6041)
        self.curr_obj = Cia402Object(0x6078)  # 실제 전류 값 객체 추가
        self.err_obj = Cia402Object(0x603f)   # 에러 레지스터 추가

        # SocketCAN 컨텍스트 매니저 수동 진입
        self.can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
        self.bus = self.can_bus_context.__enter__()
        try:
            self.can_bus_context = SocketCanBus(CAN_CHANNEL, receive_timeout=0.0)
            self.bus = self.can_bus_context.__enter__()
            self.get_logger().info(f"✅ CAN 인터페이스 연결 성공: {CAN_CHANNEL}")
        except Exception as e:
            self.get_logger().error(f"❌ CAN 연결 실패 ({CAN_CHANNEL}): {e}")
            self.get_logger().error("⚠️ 'sudo ip link set can0 up...' 명령어를 확인하세요.")
            self.bus = None

        # ⚠️ [수정] 여기서 init_can_hardware()를 직접 호출하지 않습니다 (블로킹 방지)
        # self.init_can_hardware() 

        if not AUTO_STAIR_TEST:
            self.sub_target = self.create_subscription(
                Float64, '/joint_target_rad', self.callback_target_rad, 10)
            self.get_logger().info("📥 ROS 2 토픽 제어 모드 활성화")
        else:
            self.get_logger().info(f"🚀 [다중 모드 자동 스캔] MODE 1 스위핑 시작 (지령: {self.MAX_FLEX_LIMIT} Count)")

        # 50Hz 타이머 가동
        self.timer = self.create_timer(self.dt, self.control_loop)

    def apply_mode_parameters(self):
        """
        🆕 [실험 설계 핵심] 3가지 모드별 제어 성능 파라미터를 가변하는 함수
        필요에 따라 각 모드별 강성(Kp)이나 정마찰 보상 토크를 다르게 튜닝하여 비교 평가할 수 있습니다.
        """
        if self.current_mode == 1:
            self.Kp = 330
            self.Kd = 0.21
            self.Kp = 150            # 안정적인 저강성 게인
            self.Kd = 4.0            # LPF와 함께 동작하는 적절한 댐핑 게인
            self.stiction_offset = 1600.0
            self.get_logger().info("⚙️ [PARAMETER] 모드 1 활성화: 기본 게인 제어 세팅 완료")
            
        elif self.current_mode == 2:
            self.Kp = 650          # 강성 증가
            self.Kd = 0.48
            self.Kp = 550          # 강성 증가
            self.Kd = 2.0
            self.stiction_offset = 1750.0  
            self.get_logger().info("⚙️ [PARAMETER] 모드 2 활성화: 고강성(High-Gain) 제어 세팅 완료")
            
        elif self.current_mode == 3:
            self.Kp = 450          # 유연한 제어
            self.Kd = 0.35
            self.Kp = 350          # 유연한 제어
            self.Kd = 1.2
            self.stiction_offset = 1450.0
            self.get_logger().info("⚙️ [PARAMETER] 모드 3 활성화: 저유연(Soft-Gain) 제어 세팅 완료")

    def init_can_hardware(self):
        nmt_frame = self.protocol.make_nmt_start(0)
        self.bus.send(nmt_frame)
        time.sleep(0.1)
        
        # 1. [핵심] 운전 모드 설정 (Voltage Mode: -11)
        self.bus.send(self.protocol.make_axis_mode_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, -11))
        time.sleep(0.05)

        # 2. [핵심] CiA402 상태 기기 전환 (Fault Reset -> Shutdown -> Switch On -> Enable)
        for ctrl in [Cia402Controlword.FAULT_RESET, Cia402Controlword.SHUTDOWN, 
                     Cia402Controlword.SWITCH_ON, Cia402Controlword.ENABLE_OPERATION]:
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, ctrl))
            time.sleep(0.03)

        self.get_logger().info(f"⏳ [Booting] Node {self.TARGET_NODE_ID} 엔코더 확인 중...")
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        
        boot_success = False
        deadline = time.monotonic() + 0.5  # 실제 기기는 응답 속도가 느릴 수 있으므로 0.5초 권장
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=0.01)
            if frame and frame.can_id == 0x580 + self.TARGET_NODE_ID:
                sdo_res = self.protocol.parse_sdo_response(frame)
                if sdo_res and sdo_res.index == 0x6064 and sdo_res.value is not None:
                    val = sdo_res.value
                    if val > 0x7FFFFFFF: val -= 0x100000000 # Signed 변환
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
        # 0. 하드웨어 미준비 시 초기화 시도
        if self.bus is None:
            # 버스가 없으면 제어 루프를 실행하지 않고 대기 (노드는 살아있음)
            return

        if not self.is_hardware_ready:
            # 2초(약 100틱)마다 한 번씩만 초기화 시도 (ROS 2 통신 틈 확보 및 까딱거림 방지)
            if self.init_retry_counter % 100 == 0:
                try:
                    self.get_logger().info("⏳ 하드웨어 연결 시도 중...")
                    self.init_can_hardware()
                    self.is_hardware_ready = True
                    self.get_logger().info("✅ 하드웨어 준비 완료!")
                except Exception as e:
                    self.get_logger().error(f"❌ 하드웨어 대기 중... {e}")
                    # 토픽이 PlotJuggler에 뜨도록 초기화 실패 중에도 빈 메시지 발행
                    dummy_msg = Float64MultiArray()
                    dummy_msg.data = [0.0] * 8
                    self.pub_status.publish(dummy_msg)
            self.init_retry_counter += 1
            return

        # --- 실제 제어 주기(Cycle Time) 측정 ---
        current_time = time.monotonic()
        self.cycle_time_ms = (current_time - self.last_loop_time) * 1000.0
        self.last_loop_time = current_time

        # ----------------------------------------------------------------------
        # 1. [🆕 수정] 다중 모드 및 모드 간 복귀 시퀀스 상태 머신
        # ----------------------------------------------------------------------
        if AUTO_STAIR_TEST:
            self.test_timer_ticks += 1
            
            # [A상태: 현재 모드에서 5도씩 감소하며 최대 펼침 한계까지 전진 스캔]
            if self.test_state == "SWEEP":
                if self.test_timer_ticks >= self.TICKS_PER_STEP:
                    self.test_timer_ticks = 0
                    
                    # 현재 스텝의 정상상태 오차 데이터 정산 및 기록 출력
                    final_error_count = self.target_raw_count - self.current_raw_count
                    final_error_deg = final_error_count / self.PULSES_PER_DEGREE
                    real_deg = (self.current_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
                    
                    self.get_logger().info(f"📊 [MODE {self.current_mode}] 목표: {int(self.target_raw_count)} count ({real_deg:+.1f}° 구간) -> 오차: {final_error_deg:+.4f}°")
                    
                    # 5도 하향 차감 후 다음 목표 카운트 연산
                    self.current_step_deg -= 5.0
                    requested_count = self.ALIGN_RAW_COUNT + (self.current_step_deg * self.PULSES_PER_DEGREE)
                    
                    # 최대 펼침 한계(-2014)에 완전히 도달했는지 판별
                    if requested_count <= self.MAX_EXT_LIMIT:
                        self.get_logger().warn(f"📢 [SWEEP DONE] MODE {self.current_mode}의 전 구간 스캔이 끝났습니다. 다음 모드 준비를 시작합니다.")
                        
                        # 지령을 복귀 목표인 최대 굽힘 한계(-651)로 강제 변경
                        self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                        self.test_state = "RETURN_TO_START"
                    else:
                        self.update_target_with_limit(requested_count)

            # [B상태: 다음 모드 시작 전, 안전하게 최대 굽힘 한계(-651)로 원거리 고속 복귀]
            elif self.test_state == "RETURN_TO_START":
                # 복귀 기동 후 완벽히 안착할 수 있도록 충분히 긴 시간 대기 (예: 3초 대기)
                RETURN_TICKS = int(3.0 * self.LOOP_RATE)
                if self.test_timer_ticks >= RETURN_TICKS:
                    self.test_timer_ticks = 0
                    
                    # 모드 카운트 증가
                    self.current_mode += 1
                    
                    # 3가지 모드가 모두 최종 완료된 경우 시퀀스 안전 완전 종료
                    if self.current_mode > 3:
                        self.get_logger().info("🎉 [ALL EXPERIMENTS COMPLETE] 모드 1, 2, 3의 모든 복귀 및 오차 측정 실험이 안전하게 완료되었습니다!")
                        self.get_logger().info("안전을 위해 모드를 대기 모드로 전환하며, 위치를 시작점에 고정합니다.")
                        self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                        # 무한 반복을 원한다면 self.current_mode = 1 로 리셋 가능
                        # 여기서는 안전을 위해 자동 모드를 비활성화 처리합니다.
                        globals()['AUTO_STAIR_TEST'] = False 
                        self.test_state = "READY"
                    else:
                        # 다음 모드가 남아있다면 파라미터를 교체 적용하고 다시 스위핑 준비
                        self.get_logger().info(f"↩️ 시작점(-651) 정렬 완료. [MODE {self.current_mode}]로 진입합니다.")
                        self.apply_mode_parameters()
                        
                        self.current_step_deg = self.start_deg
                        self.target_raw_count = float(self.MAX_FLEX_LIMIT)
                        self.test_state = "SWEEP"
        else:
            # [수동 입력 모드] 터미널 입력 처리
            while kbhit():
                try:
                    # os.read를 사용하여 Python 내부 버퍼링 문제 방지
                    char = os.read(sys.stdin.fileno(), 1).decode()
                    if char == '\r' or char == '\n':
                        user_input = self.input_buffer.strip().lower()
                        self.input_buffer = ""
                        
                        if user_input.startswith('m'): # 모드 변경
                            try:
                                mode_num = int(user_input[1:])
                                if 1 <= mode_num <= 3:
                                    self.current_mode = mode_num
                                    self.apply_mode_parameters()
                            except: pass
                        elif user_input: # 각도 입력
                            try:
                                target_degree = float(user_input)
                                requested_count = self.ALIGN_RAW_COUNT + (target_degree * self.PULSES_PER_DEGREE)
                                self.update_target_with_limit(requested_count)
                                self.get_logger().info(f"\n▶ 목표 각도 변경: {target_degree}°")
                            except ValueError: pass
                        break
                    elif char in ['\x08', '\x7f']: # 백스페이스 처리
                        if len(self.input_buffer) > 0:
                            self.input_buffer = self.input_buffer[:-1]
                    else:
                        self.input_buffer += char
                except: pass

        # ----------------------------------------------------------------------
        # 2. [통합] CAN 데이터 수집 및 제어 로직 실행
        # ----------------------------------------------------------------------
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.pos_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.vel_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.status_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.curr_obj))
        self.bus.send(self.protocol.make_sdo_read(self.TARGET_NODE_ID, self.err_obj))
        
        timeout_end = time.monotonic() + 0.004
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
                    elif sdo_res.index == 0x606c:
                        self.current_velocity_raw = val
                    elif sdo_res.index == 0x6041:
                        self.status_word = val
                    elif sdo_res.index == 0x6078:
                        if val > 0x7FFF: val -= 0x10000
                        self.actual_current_ma = val
                    elif sdo_res.index == 0x603f:
                        self.error_register = val
            except Exception: pass

        # [상태 체크 및 자동 복구]
        if (self.status_word & 0x08): 
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.FAULT_RESET))
            return
        elif not (self.status_word & 0x04):
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.SWITCH_ON))
            self.bus.send(self.protocol.make_axis_controlword_sdo(self.TARGET_NODE_ID, self.TARGET_AXIS, Cia402Controlword.ENABLE_OPERATION))
            return

        # 제어 연산
        # 1. 속도 LPF 연산
        filtered_velocity = (self.LPF_ALPHA * self.current_velocity_raw) + ((1.0 - self.LPF_ALPHA) * self.filtered_velocity_old)
        self.filtered_velocity_old = filtered_velocity

        # 2. 제어 연산 (필터링된 속도 사용)
        error_count = self.target_raw_count - self.current_raw_count
        v_pd = (self.Kp * error_count) - (self.Kd * self.current_velocity_raw)
        v_emf = self.K_emf_count * self.current_velocity_raw
        v_pd = (self.Kp * error_count) - (self.Kd * filtered_velocity)
        v_emf = self.K_emf_count * filtered_velocity
        
        # 3. 정밀 안착을 위한 불감대(Deadband) 로직 적용
        if abs(error_count) > self.ERROR_THRESH_COUNT:
            # 오차가 클 때: PD + 마찰 보상
            v_stiction = self.stiction_offset * np.sign(error_count)
            total_voltage = v_pd + v_stiction + v_emf
        else:
            total_voltage = 0.0
            # 오차가 작을 때: 부드러운 안착을 위한 Tail 전압 제어
            active_direction = np.sign(filtered_velocity if filtered_velocity != 0 else error_count)
            if active_direction != 0:
                v_stiction_tail = (self.stiction_offset * 0.4) * active_direction
            else:
                v_stiction_tail = 0.0
            total_voltage = (-self.Kd * filtered_velocity) + v_emf + v_stiction_tail

        if (self.current_raw_count >= self.MAX_FLEX_LIMIT and total_voltage > 0) or \
           (self.current_raw_count <= self.MAX_EXT_LIMIT and total_voltage < 0):
            total_voltage = 0.0
            
        clamped_voltage = max(-self.voltage_limit, min(self.voltage_limit, total_voltage))
        self.bus.send(self.protocol.make_q_axis_voltage_mv_sdo(self.TARGET_NODE_ID, int(clamped_voltage), self.TARGET_AXIS))

        # 데이터 발행 및 로깅
        # 4. 데이터 발행 및 로깅
        real_deg = (self.current_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        cmd_deg = (self.target_raw_count - self.ALIGN_RAW_COUNT) / self.PULSES_PER_DEGREE
        error_deg = error_count / self.PULSES_PER_DEGREE
        
        log_row = [cmd_deg, real_deg, error_deg, clamped_voltage, float(self.current_velocity_raw), float(self.current_velocity_raw), v_emf, float(self.current_mode)]
        log_row = [cmd_deg, real_deg, error_deg, clamped_voltage, float(self.current_velocity_raw), filtered_velocity, v_emf, float(self.current_mode), float(self.actual_current_ma)]
        
        status_msg = Float64MultiArray()
        status_msg.data = log_row
        self.pub_status.publish(status_msg)

        current_time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.csv_writer.writerow([current_time_str] + log_row)

        sys.stdout.write(f"\r [M{self.current_mode}] [Cycle: {self.cycle_time_ms:4.1f}ms] | Target:{int(self.target_raw_count)} | Act:{self.current_raw_count} | V:{clamped_voltage:4.0f}mV | Stat:{hex(self.status_word)} | 입력창 ➡️ {self.input_buffer}      ")
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
            # CSV 파일 닫기
            if hasattr(self, 'csv_file'):
                self.csv_file.close()
                self.get_logger().info(f"💾 로그 저장 완료: {self.csv_filename}")
            # 터미널 설정을 원래대로 복구 (매우 중요)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.can_bus_context.__exit__(None, None, None)


def main(args=None):
    rclpy.init(args=args)
    node = KitechJointController()
    
    # 멀티스레드 익스큐터를 사용하여 제어 루프와 ROS 통신(Discovery)을 분리
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        rclpy.shutdown()

if __name__ == '__main__':
    main()