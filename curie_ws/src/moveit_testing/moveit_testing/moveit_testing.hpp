#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;
static const std::string PLANNING_GROUP = "arm";

class PoseSubscriberMover : public rclcpp::Node
{
public:
    PoseSubscriberMover();
    void init_moveit();

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg);
    void key_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool plan_and_execute(const geometry_msgs::msg::PoseStamped& pose_msg);
    void go_home();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr key_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_key_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
    std::thread moveit_spin_thread_;
    std::thread typing_thread_;
    geometry_msgs::msg::PoseStamped home_pose_;
    geometry_msgs::msg::PoseStamped latest_key_pose_;
    std::atomic<bool> is_typing_{false};
    std::mutex pose_mutex_;
    std::condition_variable pose_cv_;
    bool pose_ready_{false};
};