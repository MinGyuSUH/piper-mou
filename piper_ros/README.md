### 1. can 통신

sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
candump can0 # 통신 확인

ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p gripper_exist:=true -p gripper_val_mutiple:=2
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ true\

### 2. moveit 실행

ros2 launch piper_with_gripper_moveit wm_demo.launch.py # sim
ros2 launch piper_with_gripper_moveit wm_demo.launch.py real:=true # real

### 3. 명령
ros2 run piper_with_gripper_moveit joint_goal_node # cmd창에 조인트 8개 각도(라디안) 입력
