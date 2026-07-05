#pragma once
#include <cstdint>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

typedef enum
{
    JOINT_VELOCITY,
    SERVO
} ArmMode;

namespace base
{
    class Basestation : public rclcpp::Node
    {
    public:
        Basestation(const rclcpp::NodeOptions & options);
    private:
        void _joy_drive_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void _joy_arm_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        double _map(double value, double istart, double iend, double ostart, double oend);
        void _handle_servo_state(bool enable, bool disable);
        void _handle_robot_state(bool enable, bool disable);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_drive_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr arm_drive_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drive_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_servo_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_joint_pub_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_enable_client_, servo_disable_client_;
        rclcpp::TimerBase::SharedPtr state_timer_, servo_timer_;

        geometry_msgs::msg::TwistStamped cmd_vel_msg_;
        geometry_msgs::msg::TwistStamped servo_cmd_msg_;
        std_msgs::msg::Float64MultiArray arm_cmd_msg_;
        std_srvs::srv::SetBool::Request::SharedPtr enable_req_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture servo_future;
        uint32_t request_id, servo_request_id;
        double max_linear_speed;
        double max_angular_speed;
        double turbo_multiplier;
        std::vector<double> arm_max_speeds;
        bool servo_request_handled, request_handled;
        bool servo_button_pressed, button_pressed;
        bool servoing;
        
        ArmMode arm_mode;
    };
} // namespace basestation