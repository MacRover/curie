#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class ServoAntiDrifter : public rclcpp::Node
{
public:
    ServoAntiDrifter(const rclcpp::NodeOptions & options);
private:
    void _twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
};