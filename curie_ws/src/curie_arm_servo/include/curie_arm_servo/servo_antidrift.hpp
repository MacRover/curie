#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class ServoAntiDrifter
{
public:
    ServoAntiDrifter(const rclcpp::NodeOptions & options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return node_->get_node_base_interface();
    }
private:
    void _twist_callback(const geometry_msgs::msg::TwistStamped& msg);
    void _robot_state_callback(void);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    moveit::core::RobotStatePtr current_state_;
    geometry_msgs::msg::TwistStamped corrected_twist_msg_;

    double kP_;
    double t_dz;
    std::string ee_frame;
    std::string planning_frame;
    std::string jmg_name;
    Eigen::Vector3d tvec_eef_vel;
};