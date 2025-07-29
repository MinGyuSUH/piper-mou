#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_goal_cmd_node");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting MoveGroupInterface for 'arm'");
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  std::vector<std::string> arm_joint_names = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
  };
  std::vector<std::string> gripper_joint_names = {"joint7", "joint8"};

  std::vector<double> joint_values(8);
  std::cout << "Enter joint values for joint1 to joint8 (space-separated): ";
  for (int i = 0; i < 8; ++i)
    std::cin >> joint_values[i];

  // Set arm goal
  move_group.setJointValueTarget({
    {"joint1", joint_values[0]},
    {"joint2", joint_values[1]},
    {"joint3", joint_values[2]},
    {"joint4", joint_values[3]},
    {"joint5", joint_values[4]},
    {"joint6", joint_values[5]},
  });

  // Plan and execute
  auto success = static_cast<bool>(move_group.move());
  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to move arm");
    rclcpp::shutdown();
    return 1;
  }

  // Create gripper action client
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto gripper_client = rclcpp_action::create_client<FollowJointTrajectory>(node, "/gripper_controller/follow_joint_trajectory");
  if (!gripper_client->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    rclcpp::shutdown();
    return 1;
  }

  // Send gripper goal
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = gripper_joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {joint_values[6], joint_values[7]};
  point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.points.push_back(point);

  goal_msg.trajectory.header.stamp = node->now();

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  gripper_client->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(node->get_logger(), "Sent gripper goal");

  rclcpp::shutdown();
  return 0;
}