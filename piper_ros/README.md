### 1. can 통신

sudo ip link set can0 type can bitrate 1000000

sudo ip link set up can0

candump can0 # 통신 확인

ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p gripper_exist:=true -p gripper_val_mutiple:=2 # 실제 HW 제어 노드

ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ true\ # 활성화

### 2. moveit 실행

ros2 launch piper_with_gripper_moveit wm_demo.launch.py # sim

ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true # real

### 3. 명령
ros2 run piper_with_gripper_moveit joint_goal_node # cmd창에 조인트 8개 각도(라디안) 입력

wm initial : 1.564081372 0.453823104 -1.531984412 -0.048843 1.279726728 0.316922592 0 0

basket End : -1.552795104 2.661326416 -1.513371664 -0.264206824 0.611970408 0.3951066 0 0

basket initial :-1.637974156 1.572471936 -1.272121144 -0.056658112 1.2838784 0.290896144 0 0

wm End : 1.609313664 2.148507704 -2.277418864 -0.015978704 0.340925536 0.262567088 0 0
