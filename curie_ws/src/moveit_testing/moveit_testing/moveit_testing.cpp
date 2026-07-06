#include "moveit_testing.hpp"

PoseSubscriberMover::PoseSubscriberMover() : Node(
    "moveit_testing",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    string_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/target_string", 10,
        std::bind(&PoseSubscriberMover::string_callback, this, _1));

    key_pose_client_ = this->create_client<keyboard_interfaces::srv::KeyPose>("key_pose");
}

void PoseSubscriberMover::init_moveit()
{
    auto moveit_node = std::make_shared<rclcpp::Node>("moveit_internal");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node, PLANNING_GROUP);
    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(moveit_node);
    moveit_spin_thread_ = std::thread([this]() { moveit_executor_->spin(); });

    home_pose_ = move_group_->getCurrentPose();

    RCLCPP_INFO(this->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
        home_pose_.pose.position.x,
        home_pose_.pose.position.y,
        home_pose_.pose.position.z);
}

geometry_msgs::msg::PoseStamped PoseSubscriberMover::get_key_pose(const std::string & key)
{
    if (!key_pose_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "KeyPose service not available");
        return geometry_msgs::msg::PoseStamped();
    }

    auto request = std::make_shared<keyboard_interfaces::srv::KeyPose::Request>();
    request->key = key;

    auto future = key_pose_client_->async_send_request(request);
    future.wait();

    auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN(this->get_logger(), "Invalid key requested: '%s'", key.c_str());
        return geometry_msgs::msg::PoseStamped();
    }
    return response->pose;
}

void PoseSubscriberMover::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!move_group_) return;
    if (is_typing_.exchange(true)) {
        RCLCPP_WARN(this->get_logger(), "Already typing, ignoring new string");
        return;
    }

    std::string text = msg->data;

    typing_thread_ = std::thread([this, text]() {
        for (char c : text) {
            std::string key(1, c);
            RCLCPP_INFO(this->get_logger(), "Requesting key: %s", key.c_str());

            auto pose = get_key_pose(key);
            if (pose.header.frame_id.empty()) {
                RCLCPP_WARN(this->get_logger(), "Skipping key '%s'", key.c_str());
                continue;
            }

            if (plan_and_execute(pose)) {
                go_home();
            }
        }
        is_typing_.store(false);
    });
    typing_thread_.detach();
}

bool PoseSubscriberMover::plan_and_execute(const geometry_msgs::msg::PoseStamped& pose_msg)
{
    geometry_msgs::msg::PoseStamped target_pose = home_pose_;
    target_pose.pose.position.x += pose_msg.pose.position.x;
    target_pose.pose.position.y += pose_msg.pose.position.y;

    move_group_->setPositionTarget(
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z);
    move_group_->setGoalTolerance(0.01);
    move_group_->setPlanningTime(10.0);

    RCLCPP_INFO(this->get_logger(), "Target pose: x=%.3f y=%.3f z=%.3f",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(this->get_logger(), "Plan %s", success ? "SUCCEEDED" : "FAILED");

    if (success) {
        move_group_->execute(my_plan);
        return true;
    }
    return false;
}

void PoseSubscriberMover::go_home()
{
    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0};
    move_group_->setJointValueTarget(home_joints);
    move_group_->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group_->execute(my_plan);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscriberMover>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    node->init_moveit();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}