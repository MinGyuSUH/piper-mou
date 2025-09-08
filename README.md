# Piper Arm Control Guide

이 문서는 CAN 통신 설정부터 실제 로봇 제어 및 MoveIt 실행까지의 전체 명령어 흐름을 정리한 가이드입니다.

---

## 1. CAN 통신 설정 및 제어 노드 실행

piper setting : https://github.com/agilexrobotics/piper_ros/tree/humble

```bash
# CAN 인터페이스 설정 (bitrate: 1 Mbps)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# CAN 통신 확인
candump can0
```

### 제어 노드 실행

```bash
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p gripper_exist:=true -p gripper_val_mutiple:=2
```

### 모터 활성화

```bash
ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true"
```

혹은 다음 명령으로도 가능:

```bash
ros2 topic pub /enable_flag std_msgs/msg/Bool "data: true"
```

---

## 2. MoveIt 실행

### 시뮬레이션 모드

```bash
ros2 launch piper_with_gripper_moveit wm_demo.launch.py
```

### 실제 하드웨어 제어 모드

```bash
ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true
```

---

## 3. 파일 실행

### target_pose_server 실행

realsense-viewer

```bash
cd ros2_ws/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src함

```bash
cd ros2_ws/src/piper_ros/src/piper_moveit/piper_with_gripper_moveit/src
python Logic_inte_ba.py ## basket to wm
python Logic_inte_wm.py ## wm to basket
```

> ⓘ ROS2 Humble + Piper Arm 기준 문서입니다.
