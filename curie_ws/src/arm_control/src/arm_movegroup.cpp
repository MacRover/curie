#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_movegroup");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  move_group.setPlanningTime(5.0);

  std::vector<double> target_joint_positions = {
    0.0,  // shoulder
    -1.0, // elbow
    1.2,  // wrist pitch
    0.0   // wrist roll
  };

  move_group.setJointValueTarget(target_joint_positions);

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
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
