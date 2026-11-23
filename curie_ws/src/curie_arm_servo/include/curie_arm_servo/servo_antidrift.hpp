#pragma once
#include <cstdint>
#include <cmath>
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
    void _timer_callback(void);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    moveit::core::RobotStatePtr current_state_;
    geometry_msgs::msg::TwistStamped latest_msg_, prev_msg_;

    bool enabled, state_changed;
    double t_dz;
    double pub_period_;
    double timer_count_;
    double pos_refresh_count_;
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    std::string ee_frame;
    std::string planning_frame;
    std::string jmg_name;
    Eigen::RowVector3d tvec_err_int, tvec_err_prev;
    Eigen::Vector3d projected_eef_pos_;
    Eigen::Quaterniond projected_eef_rot_;
    Eigen::Matrix3d pid_gain_pmat_;
    Eigen::Vector3d pid_gain_rmat_;
    double eef_term_speed_;
};