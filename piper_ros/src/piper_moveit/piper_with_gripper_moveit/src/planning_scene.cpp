#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

double wm_y = 0.065 ;


void addBox(moveit::planning_interface::PlanningSceneInterface& psi,
            const std::string& id,
            const std::vector<double>& size,
            const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.clear();
  primitive.dimensions.push_back(size[0]);
  primitive.dimensions.push_back(size[1]);
  primitive.dimensions.push_back(size[2]);

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  psi.applyCollisionObject(object);
}

geometry_msgs::msg::Pose makePose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Pose makeRotatedPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

void addCircularDrum(moveit::planning_interface::PlanningSceneInterface& psi) //큰 원
{
  const int num_segments = 72;
  const double radius = 0.29;
  const double center_y = 0.28 + 0.3 + 0.05 +wm_y ;
  const double center_z = 0.875;
  const double box_thickness = 0.005*2;
  const double box_depth = 0.0127*2;
  const double box_height = 0.28*2;

  for (int i = 0; i < num_segments; ++i)
  {
    double angle = 2 * M_PI * i / num_segments;
    double x = radius * cos(angle);
    double z = radius * sin(angle);

    std::string id = "ow_drum_" + std::to_string(i);
    addBox(psi, id, {box_thickness, box_height, box_depth},
           makeRotatedPose(x, center_y, center_z + z, 0, -angle, 0));
  }
}

void addWasherDrum(moveit::planning_interface::PlanningSceneInterface& psi) //작은 원
{
  const int num_segments = 36;
  const double radius = 0.185;
  const double center_y = 0.3+wm_y;  // wm_y 포함된 y
  const double center_z = 0.875;
  const double box_thickness = 0.0161*2;  // x축 방향 길이
  const double box_height = 0.0500*2;     // y축 방향 길이 (드럼 높이)
  const double box_depth = 0.0161*2;      // z축 방향 길이

  for (int i = 0; i < num_segments; ++i)
  {
    double angle = 2 * M_PI * i / num_segments;
    double x = radius * cos(angle);
    double z = radius * sin(angle);
    std::string id = "wm_drum_seg_" + std::to_string(i);
    addBox(psi, id,
           {box_thickness, box_height, box_depth},
           makeRotatedPose(x, center_y, center_z + z, 0, -angle, 0));
  }
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_scene_objects_node");
  moveit::planning_interface::PlanningSceneInterface psi;

  rclcpp::sleep_for(2s);

  addCircularDrum(psi);
  addWasherDrum(psi);

  // 책상 상판
  addBox(psi, "desk_top", {0.56, 0.28, 0.03}, makePose(0.0, 0.0, 0.33));
  // 책상 왼쪽 다리
  addBox(psi, "desk_leg_left", {0.03, 0.28, 0.33}, makePose(-0.26, 0.0, 0.165));
  // 책상 오른쪽 다리
  addBox(psi, "desk_leg_right", {0.03, 0.28, 0.33}, makePose(0.26, 0.0, 0.165));

  double basket_y = - (0.25+0.14);

  // addBox(psi, "s", {0.255, 0.255, 0.255}, makePose(0, 0.255/2, 0));

  // 바구니 - 바닥
  addBox(psi, "basket_bottom", {0.4, 0.28, 0.025}, makePose(0.0, 0.0+basket_y, 0.0125));
  // 바구니 - 앞/뒤 벽
  addBox(psi, "basket_wall_front", {0.4, 0.004, 0.17}, makePose(0.0, 0.142+basket_y, 0.085));
  addBox(psi, "basket_wall_back", {0.4, 0.004, 0.17}, makePose(0.0, -0.142+basket_y, 0.085));
  // 바구니 - 좌/우 벽
  addBox(psi, "basket_wall_left", {0.004, 0.28, 0.17}, makePose(0.202, 0.0+basket_y, 0.085));
  addBox(psi, "basket_wall_right", {0.004, 0.28, 0.17}, makePose(-0.202, 0.0+basket_y, 0.085));

  // // 세탁기 - 앞판 위쪽
  addBox(psi, "front_top", {0.6860, 0.01, 0.4000}, makeRotatedPose(0, 0.255+wm_y, 1.26000, 0.00000, 0.00000, 0.00000));
  addBox(psi, "front_bottom", {0.6860, 0.010, 0.330}, makeRotatedPose(0, 0.255+wm_y, 0.525, 0.00000, 0.00000, 0.00000));
  addBox(psi, "front_left", {0.1580, 0.010, 0.370}, makeRotatedPose(0.08600-0.35, 0.255+wm_y, 0.875, 0.00000, 0.00000, 0.00000));
  addBox(psi, "front_right", {0.1580, 0.010, 0.370}, makeRotatedPose(0.61400-0.35, 0.255+wm_y, 0.875, 0.00000, 0.00000, 0.00000));
  addBox(psi, "corner_filler_tr", {0.150, 0.010, 0.150}, makeRotatedPose(0.53500-0.35, 0.255+wm_y, 1.06000, 0.00000, -0.78540, 0.00000));
  addBox(psi, "corner_filler_tl", {0.150, 0.010, 0.150}, makeRotatedPose(0.16500-0.35, 0.255+wm_y, 1.06000, 0.00000, -0.78540, 0.00000));
  addBox(psi, "corner_filler_br", {0.150, 0.010, 0.150}, makeRotatedPose(0.53500-0.35, 0.255+wm_y, 0.69000, 0.00000, -0.78540, 0.00000));
  addBox(psi, "corner_filler_bl", {0.150, 0.010, 0.150}, makeRotatedPose(0.16500-0.35, 0.255+wm_y, 0.69000, 0.00000, -0.78540, 0.00000));

  addBox(psi, "stand_base", {0.686, 0.875, 0.36}, makePose(0, 0.6875+wm_y, 0.18));  // 하단 받침대
  // 상단 몸통
  // addBox(psi, "main_body_back", {0.686, 0.02, 1.11}, makePose(0, 1.125, 0.915));
  addBox(psi, "main_body_left", {0.02, 0.875, 1.11}, makePose(-0.333, 0.6875+wm_y, 0.915));   
  addBox(psi, "main_body_right", {0.02, 0.875, 1.11}, makePose(0.333, 0.6875+wm_y, 0.915));  
  addBox(psi, "main_body_top", {0.686, 0.875, 0.02}, makePose(0, 0.6875+wm_y, 1.465));
  addBox(psi, "main_body_bottom", {0.686, 0.875, 0.02}, makePose(0, 0.6875+wm_y, 0.365));

  RCLCPP_INFO(node->get_logger(), "Planning Scene이 추가되었습니다.");

  // 0.2 + 

  rclcpp::shutdown();
  return 0;
}
