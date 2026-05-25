#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace base
{
    class Basestation : public rclcpp::Node
    {
    public:
        Basestation(const rclcpp::NodeOptions & options);
    private:
        void _joy_drive_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        double _map(double value, double istart, double iend, double ostart, double oend);
        void _handle_robot_state(bool enable, bool disable);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_drive_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_pub_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
        rclcpp::TimerBase::SharedPtr state_timer_;

        geometry_msgs::msg::TwistStamped cmd_vel_msg_;
        std_srvs::srv::SetBool::Request::SharedPtr enable_req_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future;
        uint32_t request_id;
        double max_linear_speed;
        double max_angular_speed;
        double turbo_multiplier;
        bool request_handled, button_pressed;
    };
} // namespace basestation