#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_move_linear");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  move_group.setGoalTolerance(0.02);
  move_group.setPlanningTime(5.0);

  geometry_msgs::msg::Pose target_pose_1;
  target_pose_1.orientation.w = 1.0;
  target_pose_1.position.x = 0.0;
  target_pose_1.position.y = 0.0;
  target_pose_1.position.z = 0.0;

  move_group.setPoseTarget(target_pose_1);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
    // move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
