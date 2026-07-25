#include "curie_arm_servo/servo_antidrift.hpp"

ServoAntiDrifter::ServoAntiDrifter(const rclcpp::NodeOptions & options) : 
    node_{std::make_shared<rclcpp::Node>("servo_antidrift", options)}
{
    node_->declare_parameter("move_group_name", "arm");
    node_->declare_parameter("joint_topic", "/joint_states");
    node_->declare_parameter("ee_frame_name", "endeffector_link");
    node_->declare_parameter("planning_frame", "base_link");
    node_->declare_parameter("translation_dz", 0.0);
    node_->declare_parameter("kP", 0.0);

    kP_ = node_->get_parameter("kP").as_double();
    jmg_name = node_->get_parameter("move_group_name").as_string();
    planning_frame = node_->get_parameter("planning_frame").as_string();
    ee_frame = node_->get_parameter("ee_frame_name").as_string();
    t_dz = node_->get_parameter("translation_dz").as_double();
    std::string joint_topic = node_->get_parameter("joint_topic").as_string();
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description", "planning_scene_monitor");
    planning_scene_monitor_->startStateMonitor(joint_topic);
    planning_scene_monitor_->startSceneMonitor("/planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            std::string(node_->get_fully_qualified_name()) +
                                                            "/publish_planning_scene");
    planning_scene_monitor_->requestPlanningSceneState();

    twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10,
        std::bind(&ServoAntiDrifter::_twist_callback, this, std::placeholders::_1));
    
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_corrected_cmds", 10);
}

void ServoAntiDrifter::_twist_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    if (planning_scene_monitor_ == nullptr || !planning_scene_monitor_->getStateMonitor()->haveCompleteState()) 
    {
        return;
    }
    else if (msg.twist.linear.x == 0.0 && msg.twist.linear.y == 0.0 && msg.twist.linear.z == 0.0)
    {
        twist_pub_->publish(msg);
        return;
    }

    current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    Eigen::Vector3d tvec_cmd_vel{msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z};
    Eigen::Vector3d rvec_cmd_vel{msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z};

    const Eigen::Matrix3d cmd_to_planning_frame_transform = (
        current_state_->getGlobalLinkTransform(planning_frame).inverse() * 
        current_state_->getGlobalLinkTransform(msg.header.frame_id)
    ).linear();
    tvec_cmd_vel = cmd_to_planning_frame_transform * tvec_cmd_vel;
    rvec_cmd_vel = cmd_to_planning_frame_transform * rvec_cmd_vel;

    if (std::abs(tvec_cmd_vel.x()) < t_dz) tvec_cmd_vel.x() = 0.0;
    if (std::abs(tvec_cmd_vel.y()) < t_dz) tvec_cmd_vel.y() = 0.0;
    if (std::abs(tvec_cmd_vel.z()) < t_dz) tvec_cmd_vel.z() = 0.0;

    Eigen::VectorXd q_dot;
    current_state_->copyJointGroupVelocities(jmg_name, q_dot);
    const Eigen::MatrixXd jacobian = current_state_->getJacobian(current_state_->getJointModelGroup(jmg_name));
    Eigen::VectorXd tvec_eef_norm = (jacobian * q_dot).normalized();

    Eigen::Vector3d tvec_error = tvec_cmd_vel.normalized() - tvec_eef_norm.topRows<3>();
    corrected_twist_msg_.header.stamp = msg.header.stamp;
    corrected_twist_msg_.header.frame_id = planning_frame;
    corrected_twist_msg_.twist.linear.x = std::min(1.0, std::max(-1.0, tvec_cmd_vel.x() + kP_ * tvec_error.x()));
    corrected_twist_msg_.twist.linear.y = std::min(1.0, std::max(-1.0, tvec_cmd_vel.y() + kP_ * tvec_error.y()));
    corrected_twist_msg_.twist.linear.z = std::min(1.0, std::max(-1.0, tvec_cmd_vel.z() + kP_ * tvec_error.z()));
    corrected_twist_msg_.twist.angular.x = rvec_cmd_vel.x();
    corrected_twist_msg_.twist.angular.y = rvec_cmd_vel.y();
    corrected_twist_msg_.twist.angular.z = rvec_cmd_vel.z();
    RCLCPP_INFO(node_->get_logger(), "CMD Velocity: [%f, %f, %f]", tvec_cmd_vel.normalized()[0], tvec_cmd_vel.normalized()[1], tvec_cmd_vel.normalized()[2]);
    RCLCPP_INFO(node_->get_logger(), "EEF Velocity: [%f, %f, %f]", tvec_eef_norm[0], tvec_eef_norm[1], tvec_eef_norm[2]);

    twist_pub_->publish(corrected_twist_msg_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ServoAntiDrifter)