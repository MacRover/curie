#include <vector>
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
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose;
  double fraction;

  move_group.setGoalTolerance(0.02);
  move_group.setPlanningTime(10.0);

  target_pose.orientation.x = 0.5;
  target_pose.orientation.y = -0.5;
  target_pose.orientation.z = -0.5;
  target_pose.orientation.w = 0.5;
  target_pose.position.x = 0.376;
  target_pose.position.y = -0.056;
  target_pose.position.z = 0.3;

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
    goto cleanup;
  }

  move_group.clearPoseTargets();
  move_group.setStartStateToCurrentState();
  target_pose.position.x += 0.2;
  waypoints.push_back(target_pose);

  fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (fraction > 0.0)
  {
    RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to compute Cartesian path");
    goto cleanup;
  }

cleanup:
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
