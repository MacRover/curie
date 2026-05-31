#include <memory> 
#include <rclcpp/rclcpp.hpp> 
#include <moveit/move_group_interface/move_group_interface.h> 

static const std::string PLANNING_GROUP= "arm"; 
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_testing"); 

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<rclcpp::Node>(
        "moveit_testing", 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(node); 
    std::thread spinner([&executor]() { executor.spin(); }); 
    
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    geometry_msgs::msg::Pose target_pose; 

    RCLCPP_INFO(rclcpp::get_logger("moveit_testing"), 
        "Current pose: x=%.3f y=%.3f z=%.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z);
    RCLCPP_INFO(LOGGER, "Current orientation: x=%.3f y=%.3f z=%.3f w=%.3f", 
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y, 
        current_pose.pose.orientation.z, 
        current_pose.pose.orientation.w);
    target_pose.orientation = current_pose.pose.orientation; 
    target_pose.position.x = 0.737;
    target_pose.position.y = -0.337; 
    target_pose.position.z = 0.539;
    move_group.setPoseTarget(target_pose); 
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS); 
    RCLCPP_INFO(rclcpp::get_logger("moveit_testing"), "Plan %s", success ? "SUCCEEDED" : "FAILED"); 
    
    if (success) { 
        move_group.move(); 
    }
    
    rclcpp::shutdown(); 
    spinner.join(); 
    return 0; 
}

