#include "curie_base/base.hpp"

typedef enum : uint8_t
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    BACK = 6,
    GUIDE = 8,
    START = 7,
    LEFT_STICK = 9,
    RIGHT_STICK = 10,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    DPAD_UP = 11,
    DPAD_DOWN = 12,
    DPAD_LEFT = 13,
    DPAD_RIGHT = 14
} Button;

typedef enum : uint8_t
{
    LEFT_X = 0,
    LEFT_Y = 1,
    LEFT_TRIGGER = 2,
    RIGHT_X = 3,
    RIGHT_Y = 4,
    RIGHT_TRIGGER = 5
} Axis;

base::Basestation::Basestation(const rclcpp::NodeOptions & options) : Node("basestation", options)
{
    this->declare_parameter("max_linear_speed", 1.0);
    this->declare_parameter("max_angular_speed", 1.0);
    this->declare_parameter("turbo_multiplier", 1.0);
    max_linear_speed = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed = this->get_parameter("max_angular_speed").as_double();
    turbo_multiplier = this->get_parameter("turbo_multiplier").as_double();

    joy_drive_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Basestation::_joy_drive_callback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
}

void base::Basestation::_joy_drive_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    cmd_vel_msg_.header.stamp = this->now();
    double turbo_factor = 1.0f + msg->axes[RIGHT_TRIGGER] * turbo_multiplier;
    
    cmd_vel_msg_.twist.linear.x = msg->axes[LEFT_Y] * max_linear_speed * turbo_factor;
    cmd_vel_msg_.twist.angular.z = msg->axes[RIGHT_X] * max_angular_speed * turbo_factor;
    drive_pub_->publish(cmd_vel_msg_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(base::Basestation)