#include "curie_arm_servo/servo_antidrift.hpp"

ServoAntiDrifter::ServoAntiDrifter(const rclcpp::NodeOptions & options) : 
    Node("antidrift_node", options)
{
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10,
        std::bind(&ServoAntiDrifter::_twist_callback, this, std::placeholders::_1));
    
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_corrected_cmds", 10);
}

void ServoAntiDrifter::_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    twist_pub_->publish(*msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ServoAntiDrifter)