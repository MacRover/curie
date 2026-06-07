#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;
static const std::string PLANNING_GROUP = "arm";

class PoseSubscriberMover : public rclcpp::Node
{
public:
    PoseSubscriberMover() : Node(
        "moveit_testing",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/key_target_pose", 1,
            std::bind(&PoseSubscriberMover::topic_callback, this, _1));
    }

    void init_moveit()
    {
        auto moveit_node = std::make_shared<rclcpp::Node>("moveit_internal");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            moveit_node, PLANNING_GROUP);

        moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        moveit_executor_->add_node(moveit_node);
        moveit_spin_thread_ = std::thread([this]() { moveit_executor_->spin(); });

        // Store initial pose once
        stored_pose_ = move_group_->getCurrentPose();

        RCLCPP_INFO(this->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
            stored_pose_.pose.position.x,
            stored_pose_.pose.position.y,
            stored_pose_.pose.position.z);
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!move_group_) return;

        geometry_msgs::msg::PoseStamped target_pose = stored_pose_;
        target_pose.pose.position.x += msg->pose.position.x;
        target_pose.pose.position.y += msg->pose.position.y;
        target_pose.pose.position.z += msg->pose.position.z;

        move_group_->setPositionTarget(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        ); 
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
            stored_pose_ = target_pose;  // update only on success
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
    std::thread moveit_spin_thread_;
    geometry_msgs::msg::PoseStamped stored_pose_;
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