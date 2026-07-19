#include "curie_base/base.hpp"

#define DEG_TO_RAD (M_PI / 180.0f)

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

typedef enum : uint8_t
{
    PARAM_ARM_BASE = 0,
    PARAM_ARM_SHOULDER,
    PARAM_ARM_ELBOW,
    PARAM_ARM_PITCH,
    PARAM_ARM_ROLL,
} ROSParamIndex;

base::Basestation::Basestation(const rclcpp::NodeOptions & options) : 
    Node("basestation", options), 
    servo_request_handled(true),
    request_handled(true), 
    servo_button_pressed(false),
    button_pressed(false),
    arm_mode(JOINT_VELOCITY)
{
    this->declare_parameter("drive.max_linear_speed", 1.0);
    this->declare_parameter("drive.max_angular_speed", 1.0);
    this->declare_parameter("drive.turbo_multiplier", 1.0);

    this->declare_parameter("arm.max_base_angular_speed", 1.0);
    this->declare_parameter("arm.max_shoulder_angular_speed", 1.0);
    this->declare_parameter("arm.max_elbow_angular_speed", 1.0);
    this->declare_parameter("arm.max_pitch_angular_speed", 1.0);
    this->declare_parameter("arm.max_roll_angular_speed", 1.0);

    this->declare_parameter("arm.servoing", false);
    servoing = this->get_parameter("arm.servoing").as_bool();

    max_linear_speed = this->get_parameter("drive.max_linear_speed").as_double();
    max_angular_speed = this->get_parameter("drive.max_angular_speed").as_double();
    turbo_multiplier = this->get_parameter("drive.turbo_multiplier").as_double();
    arm_max_speeds.push_back(this->get_parameter("arm.max_base_angular_speed").as_double());
    arm_max_speeds.push_back(this->get_parameter("arm.max_shoulder_angular_speed").as_double());
    arm_max_speeds.push_back(this->get_parameter("arm.max_elbow_angular_speed").as_double());
    arm_max_speeds.push_back(this->get_parameter("arm.max_pitch_angular_speed").as_double());
    arm_max_speeds.push_back(this->get_parameter("arm.max_roll_angular_speed").as_double());

    enable_req_ = std::make_shared<std_srvs::srv::SetBool::Request>();
    enable_client_ = this->create_client<std_srvs::srv::SetBool>("heartbeat/enable");
    while (rclcpp::ok() && !enable_client_->wait_for_service(std::chrono::seconds(5))) 
    {
        RCLCPP_INFO(this->get_logger(), "Heartbeat service not available, waiting...");
    }

    switch_req_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    while (rclcpp::ok() && !controller_client_->wait_for_service(std::chrono::seconds(5))) 
    {
        RCLCPP_INFO(this->get_logger(), "Controller manager not available, waiting...");
    }

    servo_enable_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/unpause_servo");
    servo_disable_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/pause_servo");
    while (rclcpp::ok() && servoing && 
            !(servo_enable_client_->wait_for_service(std::chrono::seconds(5)) && 
              servo_disable_client_->wait_for_service(std::chrono::seconds(5))))
    {
        RCLCPP_INFO(this->get_logger(), "Servo not available, waiting...");
    }

    joy_drive_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Basestation::_joy_drive_callback, this, std::placeholders::_1));
    drive_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    arm_drive_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy1", 10, std::bind(&Basestation::_joy_arm_callback, this, std::placeholders::_1));
    arm_joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/vel_commands", 10);
    arm_servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

    RCLCPP_INFO(this->get_logger(), "Basestation OK");
}

double base::Basestation::_map(double value, double istart, double iend, double ostart, double oend)
{
    return ostart + (oend - ostart) * (value - istart) / (iend - istart);
}

void base::Basestation::_joy_arm_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool button_ik = msg->buttons[X];
    bool button_fk = msg->buttons[Y];
    if (servoing && button_ik)
    {
        arm_mode = SERVO;
    }
    else if (button_fk)
    {
        arm_mode = JOINT_VELOCITY;
    }

    switch (arm_mode)
    {
        case SERVO:
        {
            servo_cmd_msg_.header.stamp = this->now();
            servo_cmd_msg_.header.frame_id = "base_link";
            servo_cmd_msg_.twist.linear.x = msg->axes[RIGHT_Y];
            servo_cmd_msg_.twist.linear.z = (msg->axes[RIGHT_TRIGGER] - msg->axes[LEFT_TRIGGER]) / 2.0;
            servo_cmd_msg_.twist.angular.x = msg->axes[LEFT_X] * -1.0;
            servo_cmd_msg_.twist.angular.y = msg->axes[LEFT_Y] * -1.0;

            arm_servo_pub_->publish(servo_cmd_msg_);
            break;
        }
        case JOINT_VELOCITY:
        {
            double pitch_axis = (msg->axes[RIGHT_TRIGGER] - msg->axes[LEFT_TRIGGER]) / 2.0;
            arm_cmd_msg_.data = {
                this->_map(
                    msg->axes[RIGHT_X], 
                    -1.0, 1.0, 
                    -arm_max_speeds[PARAM_ARM_BASE], arm_max_speeds[PARAM_ARM_BASE]
                ),
                this->_map(
                    msg->axes[RIGHT_Y] * -1.0, 
                    -1.0, 1.0, 
                    -arm_max_speeds[PARAM_ARM_SHOULDER], arm_max_speeds[PARAM_ARM_SHOULDER]
                ),
                this->_map(
                    msg->axes[LEFT_Y] * -1.0, 
                    -1.0, 1.0, 
                    -arm_max_speeds[PARAM_ARM_ELBOW], arm_max_speeds[PARAM_ARM_ELBOW]
                ),
                this->_map(
                    pitch_axis, 
                    -1.0, 1.0, 
                    -arm_max_speeds[PARAM_ARM_PITCH], arm_max_speeds[PARAM_ARM_PITCH]
                ),
                this->_map(
                    msg->axes[LEFT_X], 
                    -1.0, 1.0, 
                    -arm_max_speeds[PARAM_ARM_ROLL], arm_max_speeds[PARAM_ARM_ROLL]
                ),
            };
            arm_joint_pub_->publish(arm_cmd_msg_);
            break;
        }
    }

    if (servoing)
    {
        _handle_servo_state(button_ik, button_fk);   
    }
    _handle_robot_state(msg->buttons[A], msg->buttons[B]);
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

void base::Basestation::_handle_servo_state(bool enable, bool disable)
{
    if (!servo_request_handled)
    {
        return;
    }
    if (!servo_button_pressed && (enable || disable))
    {
        servo_button_pressed = true;
        servo_request_handled = false;
        auto client = enable ? servo_enable_client_ : servo_disable_client_;
        RCLCPP_INFO(this->get_logger(), "Sending request to %s servo", enable ? "enable" : "disable");
        auto result = client->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this, enable](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
                (void)result;
                servo_request_handled = true;
                switch_req_->start_asap = true;
                switch_req_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
                if (enable) {
                    RCLCPP_INFO(this->get_logger(), "Servo enabled");
                    switch_req_->activate_controllers = {"arm_position_controller"};
                    switch_req_->deactivate_controllers = {"arm_controller"};
                } else {
                    RCLCPP_INFO(this->get_logger(), "Servo disabled");
                    switch_req_->activate_controllers = {"arm_controller"};
                    switch_req_->deactivate_controllers = {"arm_position_controller"};
                }
                // Whenever switching from JTC to position controller specifically, we need to prime the servo node
                // so it does not send stale commands which causes the arm to jump back to the position right before
                // it was paused. To do so, send a single small twist command to the servo node, essentially "pinging" it to
                // update its internal state.
                geometry_msgs::msg::TwistStamped update_msg;
                update_msg.header.stamp = this->now();
                update_msg.header.frame_id = "base_link";
                update_msg.twist.linear.x = -0.0001;
                arm_servo_pub_->publish(update_msg);
                controller_client_->async_send_request(switch_req_);
            });
        
        servo_future = result.future;
        servo_request_id = result.request_id;
        servo_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (servo_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                    RCLCPP_WARN(this->get_logger(), 
                        "Request timed out, servo node may still be initializing or rover is not connected"
                    );
                    // Clear pending request and allow new requests to be sent
                    servo_enable_client_->remove_pending_request(servo_request_id);
                    servo_request_handled = true;
                }
                servo_timer_->cancel();
            }
        );
    }
    else if (!(enable || disable))
    {
        servo_button_pressed = false;
    }
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