#include "moveit_testing.hpp"

PoseSubscriberMover::PoseSubscriberMover() : Node(
    "moveit_testing",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    key_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/key_target_pose", 1,
        std::bind(&PoseSubscriberMover::key_pose_callback, this, _1));

    string_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/target_string", 10,
        std::bind(&PoseSubscriberMover::string_callback, this, _1));

    target_key_pub_ = this->create_publisher<std_msgs::msg::String>("/target_key", 10);
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
void PoseSubscriberMover::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!move_group_) return;
    if (is_typing_) {
        RCLCPP_WARN(this->get_logger(), "Already typing, ignoring new string");
        return;
    }

    std::string text = msg->data;
    is_typing_ = true;

    typing_thread_ = std::thread([this, text]() {

        for (char c : text) {
            std::string key(1, c);

            // Reset flag before publishing
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                pose_ready_ = false;
            }

            // Publish key
            auto key_msg = std_msgs::msg::String();
            key_msg.data = key;
            target_key_pub_->publish(key_msg);

            // Wait for ArUco to respond
            std::unique_lock<std::mutex> lock(pose_mutex_);
            pose_cv_.wait(lock, [this] { return pose_ready_; });

            plan_and_execute(latest_key_pose_);
            go_home();
        }

        is_typing_ = false;
    });
    typing_thread_.detach();
}

void PoseSubscriberMover::key_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_); 
    latest_key_pose_ = *msg; 
    pose_ready_ = true; 
    pose_cv_.notify_one(); 
}

bool PoseSubscriberMover::plan_and_execute(const geometry_msgs::msg::PoseStamped& pose_msg)
{
    geometry_msgs::msg::PoseStamped target_pose = home_pose_;
    target_pose.pose.position.x += pose_msg.pose.position.x;
    target_pose.pose.position.y += pose_msg.pose.position.y;
    //target_pose.pose.position.z += pose_msg.pose.position.z;

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