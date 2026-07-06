import time
import numpy as np

class ControlCycleMonitor:
    def __init__(self, target_hz=50):
        self.target_period = 1.0 / target_hz
        self.start_time = time.perf_counter()
        self.last_time = time.perf_counter()
        self.cycle_history = []
        self.count = 0

    def check(self):
        current_time = time.perf_counter()
        # 이전 루프 시작부터 현재 루프 시작까지 걸린 실제 시간 (Interval)
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 첫 번째 호출은 제외 (이전 기록이 없으므로)
        if self.count > 0:
            self.cycle_history.append(dt)
        
        self.count += 1

        # 50번(약 1초)마다 통계 출력
        if self.count % 50 == 0 and len(self.cycle_history) > 0:
            avg_dt = np.mean(self.cycle_history)
            max_dt = np.max(self.cycle_history)
            min_dt = np.min(self.cycle_history)
            jitter = max_dt - min_dt # 최대-최소 차이 (지터)
            std_dev = np.std(self.cycle_history) # 표준편차

            print(f"\n--- [PC 제어 주기 모니터링 현황] ---")
            print(f"목표 주기: {self.target_period*1000:.2f} ms ({1/self.target_period} Hz)")
            print(f"실제 평균: {avg_dt*1000:.2f} ms ({1/avg_dt:.2f} Hz)")
            print(f"최대 주기: {max_dt*1000:.2f} ms (느려짐)")
            print(f"최소 주기: {min_dt*1000:.2f} ms (빨라짐)")
            print(f"지터(Jitter): {jitter*1000:.4f} ms")
            print(f"표준편차: {std_dev*1000:.4f} ms")
            
            # 히스토리 초기화 (메모리 관리 및 실시간성 유지)
            self.cycle_history = []

# --- [형님의 실제 제어 루프 적용 예시] ---
monitor = ControlCycleMonitor(target_hz=50) # 50Hz 목표

try:
    while True:
        loop_start = time.perf_counter()
        
        # 1. 제어 주기 측정 시작
        monitor.check()
        
        # 2. 형님의 기존 제어 로직 (PD제어, CAN통신 등)
        # -------------------------------------------
        # applied_volt = joints[0].update(...)
        # can_bus.send(...)
        # -------------------------------------------
        
        # 3. 50Hz를 맞추기 위한 강제 휴식 (PC 환경에서 최대한 주기를 맞춤)
        loop_end = time.perf_counter()
        process_time = loop_end - loop_start
        sleep_time = (1.0/50.0) - process_time
        
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\n측정 종료.")