# Piper Arm Control Guide

이 문서는 CAN 통신 설정부터 실제 로봇 제어 및 MoveIt 실행까지의 전체 명령어 흐름을 정리한 가이드입니다.

---

## 1. CAN 통신 설정 및 제어 노드 실행

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

## 3. 조인트 명령어 실행

```bash
ros2 run piper_with_gripper_moveit joint_goal_cmd_node
```

명령어 실행 후 터미널에 조인트 8개 값(라디안) 입력

### 예시 위치값

| 이름           | 조인트 값 (라디안) |
|----------------|--------------------|
| wm initial     | 1.564081372 0.453823104 -1.531984412 -0.048843 1.279726728 0.316922592 0 0 |
| basket End     | -1.552795104 2.661326416 -1.513371664 -0.264206824 0.611970408 0.3951066 0 0 |
| basket initial | -1.637974156 1.572471936 -1.272121144 -0.056658112 1.2838784 0.290896144 0 0 |
| wm End         | 1.609313664 2.148507704 -2.277418864 -0.015978704 0.340925536 0.262567088 0 0 |

---

> ⓘ ROS2 Humble + Piper Arm 기준 문서입니다.
