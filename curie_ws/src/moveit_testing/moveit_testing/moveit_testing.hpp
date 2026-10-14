#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "keyboard_interfaces/srv/key_pose.hpp"
using std::placeholders::_1;
static const std::string PLANNING_GROUP = "arm";

class PoseSubscriberMover : public rclcpp::Node
{
public:
  PoseSubscriberMover();
  void init_moveit();

private:
  void string_callback(const std_msgs::msg::String::SharedPtr msg);
  geometry_msgs::msg::PoseStamped get_key_pose(const std::string & key);
  bool plan_and_execute(const geometry_msgs::msg::PoseStamped & pose_msg);
  void go_home();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  rclcpp::Client<keyboard_interfaces::srv::KeyPose>::SharedPtr key_pose_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
  std::thread moveit_spin_thread_;
  std::thread typing_thread_;
  std::atomic<bool> is_typing_{false};

  geometry_msgs::msg::PoseStamped home_pose_;
};