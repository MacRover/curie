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

base::Basestation::Basestation(const rclcpp::NodeOptions & options) : Node("basestation", options), request_handled(true), button_pressed(false)
{
    this->declare_parameter("max_linear_speed", 1.0);
    this->declare_parameter("max_angular_speed", 1.0);
    this->declare_parameter("turbo_multiplier", 1.0);
    max_linear_speed = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed = this->get_parameter("max_angular_speed").as_double();
    turbo_multiplier = this->get_parameter("turbo_multiplier").as_double();

    enable_req_ = std::make_shared<std_srvs::srv::SetBool::Request>();
    enable_client_ = this->create_client<std_srvs::srv::SetBool>("heartbeat/enable");
    while (rclcpp::ok() && !enable_client_->wait_for_service(std::chrono::seconds(5))) 
    {
        RCLCPP_INFO(this->get_logger(), "Heartbeat service not available, waiting...");
    }

    joy_drive_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Basestation::_joy_drive_callback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Basestation OK");
}

double base::Basestation::_map(double value, double istart, double iend, double ostart, double oend)
{
    return ostart + (oend - ostart) * (value - istart) / (iend - istart);
}

void base::Basestation::_joy_drive_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    cmd_vel_msg_.header.stamp = this->now();
    double turbo_factor = this->_map(msg->axes[RIGHT_TRIGGER], 1.0, -1.0, 1.0, turbo_multiplier);
    
    cmd_vel_msg_.twist.linear.x = msg->axes[LEFT_Y] * max_linear_speed * turbo_factor;
    cmd_vel_msg_.twist.angular.z = msg->axes[RIGHT_X] * max_angular_speed * turbo_factor;
    drive_pub_->publish(cmd_vel_msg_);

    _handle_robot_state(msg->buttons[A], msg->buttons[B]);
}

void base::Basestation::_handle_robot_state(bool enable, bool disable)
{
    if (!request_handled)
    {
        return;
    }
    if (!button_pressed && (enable || disable))
    {
        button_pressed = true;
        request_handled = false;
        enable_req_->data = enable || !disable;
        auto result = enable_client_->async_send_request(enable_req_, 
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
            request_handled = true;
            if (result.get()->success) {
                RCLCPP_INFO(this->get_logger(), "%s", result.get()->message.c_str());
            }
        });

        future = result.future;
        request_id = result.request_id;
        state_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                    RCLCPP_WARN(this->get_logger(), "Request timed out, is the rover connected?");
                    // Clear pending request and allow new requests to be sent
                    enable_client_->remove_pending_request(request_id);
                    request_handled = true;
                }
                state_timer_->cancel();
            }
        );
    }
    else if (!(enable || disable))
    {
        button_pressed = false;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(base::Basestation)