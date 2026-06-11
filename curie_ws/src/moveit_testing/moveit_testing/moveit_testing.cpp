#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <mutex> 
#include <condition_variable>

using std::placeholders::_1;
static const std::string PLANNING_GROUP = "arm";

class PoseSubscriberMover : public rclcpp::Node
{
public:
    PoseSubscriberMover() : Node(
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

    void init_moveit()
    {
        auto moveit_node = std::make_shared<rclcpp::Node>("moveit_internal");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            moveit_node, PLANNING_GROUP);
        moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        moveit_executor_->add_node(moveit_node);
        moveit_spin_thread_ = std::thread([this]() { moveit_executor_->spin(); });

        stored_pose_ = move_group_->getCurrentPose();
        home_pose_ = stored_pose_;  // save home pose once at init

        RCLCPP_INFO(this->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
            stored_pose_.pose.position.x,
            stored_pose_.pose.position.y,
            stored_pose_.pose.position.z);
    }

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg)
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

    void key_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_); 
        latest_key_pose_ = *msg; 
        pose_ready_ = true; 
        pose_cv_.notify_one(); 
    }

    bool plan_and_execute(const geometry_msgs::msg::PoseStamped& pose_msg)
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

    void go_home()
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

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr key_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_key_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
    std::thread moveit_spin_thread_;
    std::thread typing_thread_;
    geometry_msgs::msg::PoseStamped stored_pose_;
    geometry_msgs::msg::PoseStamped home_pose_;
    geometry_msgs::msg::PoseStamped latest_key_pose_;
    std::atomic<bool> is_typing_{false};
    std::mutex pose_mutex_;
    std::condition_variable pose_cv_;
    bool pose_ready_{false};
};

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