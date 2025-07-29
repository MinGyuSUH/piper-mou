cmake_minimum_required(VERSION 3.22)
project(piper_with_gripper_moveit)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)

add_executable(planning_scene_node src/planning_scene.cpp)
add_executable(joint_goal_node src/joint_goal.cpp)
add_executable(joint_goal_cmd_node src/joint_goal_cmd.cpp)

ament_target_dependencies(planning_scene_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(joint_goal_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  control_msgs
)

ament_target_dependencies(joint_goal_cmd_node
  rclcpp
  moveit_ros_planning_interface
  control_msgs
)

target_link_libraries(joint_goal_node pthread)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(TARGETS
  planning_scene_node
  joint_goal_node
  joint_goal_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)