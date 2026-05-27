# Arm Lab Control (ROS 2 & CANopen)

Ubuntu 22.04와 ROS 2 Humble 환경에서 Welcon 모터 드라이버를 제어하는 프로젝트입니다.

## 주요 기능
- **Virtual Driver**: `vcan0`를 이용한 3축 핑거 시뮬레이션 및 PD 제어.
- **Real Driver**: 실제 Welcon 드라이버와 SocketCAN을 이용한 1:1 위치 제어 및 마찰 보상.
- **Control Scripts**: 손가락 정렬(`align_finger.py`) 및 SDO 전압 제어 스크립트 포함.

## 설치 및 실행

### 가상 캔 설정
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 실행 방법
1. 드라이버 실행: `python3 real_driver/real_welcon_driver.py`
2. GUI 제어: `ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -r joint_states:=target_joints`

## 하드웨어 구성
- Controller: Welcon WE2x Series
- Communication: CANopen (via SocketCAN)
