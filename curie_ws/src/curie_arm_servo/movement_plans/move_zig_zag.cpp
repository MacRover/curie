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

  move_group.setGoalTolerance(0.001);
  move_group.setPlanningTime(10.0);

  const double delta = 0.3;
  double x_delta_arr[] = {0, delta, -delta, delta, -delta};
  double z_delta_arr[] = {0, 0,     delta,  0,     -delta};
  bool success;

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 0.5;
  target_pose.orientation.y = -0.5;
  target_pose.orientation.z = -0.5;
  target_pose.orientation.w = 0.5;
  target_pose.position.x = 0.2;
  target_pose.position.y = -0.0555596;
  target_pose.position.z = 0.2;

  for (size_t i = 0; i < 5; i++)
  {
    target_pose.position.x += x_delta_arr[i];
    target_pose.position.z += z_delta_arr[i];
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
      move_group.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Planning failed");
      break;
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
