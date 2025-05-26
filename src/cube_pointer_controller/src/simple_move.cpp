#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node  = rclcpp::Node::make_shared("simple_move_cpp");
  static const std::string GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, GROUP);
  move_group.setPoseReferenceFrame("base_link");

  geometry_msgs::msg::Pose target;
  target.position.x = 0.40;
  target.position.y = 0.00;
  target.position.z = 0.20;
  target.orientation.w = 1.0;
  move_group.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
    move_group.execute(plan);

  rclcpp::shutdown();
  return 0;
}
