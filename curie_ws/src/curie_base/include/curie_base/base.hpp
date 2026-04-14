#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace base
{
    class Basestation : public rclcpp::Node
    {
    public:
        Basestation(const rclcpp::NodeOptions & options);
    private:
        void _joy_drive_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_drive_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_pub_;

        geometry_msgs::msg::TwistStamped cmd_vel_msg_;

        double max_linear_speed;
        double max_angular_speed;
        double turbo_multiplier;
    };
} // namespace basestation