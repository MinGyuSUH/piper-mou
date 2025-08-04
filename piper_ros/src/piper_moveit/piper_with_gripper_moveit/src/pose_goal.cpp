#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread> // std::thread 사용을 위해 추가

#include "piper_with_gripper_moveit/action/pose_goal.hpp"

using PoseGoal = piper_with_gripper_moveit::action::PoseGoal;
using GoalHandlePoseGoal = rclcpp_action::ServerGoalHandle<PoseGoal>;
using namespace std::placeholders;

class PoseGoalActionServer : public rclcpp::Node
{
public:
  explicit PoseGoalActionServer(const rclcpp::NodeOptions& options)
  : Node("pose_goal_node", options), logger_(this->get_logger())
  {
    RCLCPP_INFO(logger_, "PoseGoalActionServer 초기화 중...");
    
    // MoveGroupInterface를 생성할 때 this(Node::SharedPtr)와 planning group 이름을 전달합니다.
    // 생성자에서 바로 초기화하기 위해 지연 초기화를 사용합니다.
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this), "arm");

    action_server_ = rclcpp_action::create_server<PoseGoal>(
      this,
      "pose_goal",
      std::bind(&PoseGoalActionServer::handle_goal, this, _1, _2),
      std::bind(&PoseGoalActionServer::handle_cancel, this, _1),
      std::bind(&PoseGoalActionServer::handle_accepted, this, _1)
    );

    RCLCPP_INFO(logger_, "PoseGoalActionServer 준비 완료.");
  }

private:
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  rclcpp_action::Server<PoseGoal>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PoseGoal::Goal> goal)
  {
    RCLCPP_INFO(logger_, "목표 pose 수신: x=%.2f, y=%.2f, z=%.2f",
                goal->target_pose.pose.position.x,
                goal->target_pose.pose.position.y,
                goal->target_pose.pose.position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePoseGoal>&)
  {
    RCLCPP_INFO(logger_, "취소 요청 수신.");
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePoseGoal>& goal_handle)
  {
    std::thread{std::bind(&PoseGoalActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePoseGoal>& goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PoseGoal::Result>();

    RCLCPP_INFO(logger_, "목표 position으로 플래닝 시작...");

    // MoveGroupInterface를 사용하여 목표 위치 설정
    // setPositionTarget은 현재 엔드 이펙터의 방향을 유지하면서 위치만 변경합니다.
    move_group_->setPositionTarget(
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z);

    // 플랜 및 실행
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(logger_, "경로 생성 성공, 실행 중...");
      auto exec_result = move_group_->execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        result->success = true;
        result->message = "동작 성공.";
        goal_handle->succeed(result);
        RCLCPP_INFO(logger_, "동작 성공.");
        return;
      }
      else
      {
        result->success = false;
        result->message = "실행 실패. 에러 코드: " + std::to_string(exec_result.val);
        RCLCPP_ERROR(logger_, "%s", result->message.c_str());
      }
    }
    else
    {
      result->success = false;
      result->message = "경로 생성 실패.";
      RCLCPP_ERROR(logger_, "경로 생성 실패.");
    }
    
    goal_handle->abort(result);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // MoveIt 파라미터를 노드에 로드하기 위해 NodeOptions를 사용합니다.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PoseGoalActionServer>(node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}