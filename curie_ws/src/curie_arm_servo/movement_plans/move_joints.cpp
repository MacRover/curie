#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_move_joints");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  move_group.setPlanningTime(5.0);

  std::vector<double> target_joint_positions_1 = {
    -0.4,  // base
    0.78,  // shoulder
    0.5,  // elbow
    0.3,   // wrist pitch
    0.78    // wrist roll
  };

  std::vector<double> target_joint_positions_2 = {
    0.0,  // base
    0.2,  // shoulder
    1.2,  // elbow
    0.0,   // wrist pitch
    0.0   // wrist roll
  };

  for (int i = 0; i < 10; i++)
  {
    move_group.setJointValueTarget((i % 2 == 0) ? target_joint_positions_1 : target_joint_positions_2);

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
  }


  rclcpp::shutdown();
  spinner.join();
  return 0;
}
