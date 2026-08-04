#include "curie_arm_servo/servo_antidrift.hpp"

ServoAntiDrifter::ServoAntiDrifter(const rclcpp::NodeOptions & options) : 
    node_{std::make_shared<rclcpp::Node>("servo_antidrift", options)},
    state_changed(true),
    timer_count_(0.0),
    pos_refresh_count_(0.0),
    tvec_err_int{Eigen::Vector3d::Zero()},
    tvec_err_prev{Eigen::Vector3d::Zero()},
    projected_eef_pos_{Eigen::Vector3d::Zero()}
{
    node_->declare_parameter("move_group_name", "arm");
    node_->declare_parameter("joint_topic", "/joint_states");
    node_->declare_parameter("ee_frame_name", "endeffector_link");
    node_->declare_parameter("planning_frame", "base_link");
    node_->declare_parameter("translation_dz", 0.0);
    node_->declare_parameter("publish_rate", 20.0);
    node_->declare_parameter("antidrift_enable", false);

    node_->declare_parameter("drift_axes.x.pid_min", -1.0);
    node_->declare_parameter("drift_axes.x.pid_max", 1.0);
    node_->declare_parameter("drift_axes.y.pid_min", -1.0);
    node_->declare_parameter("drift_axes.y.pid_max", 1.0);
    node_->declare_parameter("drift_axes.z.pid_min", -1.0);
    node_->declare_parameter("drift_axes.z.pid_max", 1.0);

    node_->declare_parameter("drift_axes.x.linear_kP", 0.0);
    node_->declare_parameter("drift_axes.x.linear_kI", 0.0);
    node_->declare_parameter("drift_axes.x.linear_kD", 0.0);
    node_->declare_parameter("drift_axes.y.linear_kP", 0.0);
    node_->declare_parameter("drift_axes.y.linear_kI", 0.0);
    node_->declare_parameter("drift_axes.y.linear_kD", 0.0);
    node_->declare_parameter("drift_axes.z.linear_kP", 0.0);
    node_->declare_parameter("drift_axes.z.linear_kI", 0.0);
    node_->declare_parameter("drift_axes.z.linear_kD", 0.0);
    node_->declare_parameter("drift_axes.x.euler_kP", 0.0);
    node_->declare_parameter("drift_axes.y.euler_kP", 0.0);
    node_->declare_parameter("drift_axes.z.euler_kP", 0.0);

    node_->declare_parameter("eef_max_speed", 0.2);
    eef_term_speed_ = node_->get_parameter("eef_max_speed").as_double();

    double kP, kI, kD;
    kP = node_->get_parameter("drift_axes.x.linear_kP").as_double();
    kI = node_->get_parameter("drift_axes.x.linear_kI").as_double();
    kD = node_->get_parameter("drift_axes.x.linear_kD").as_double();
    pid_gain_pmat_.row(0) << kP, kI, kD;
    kP = node_->get_parameter("drift_axes.y.linear_kP").as_double();
    kI = node_->get_parameter("drift_axes.y.linear_kI").as_double();
    kD = node_->get_parameter("drift_axes.y.linear_kD").as_double();
    pid_gain_pmat_.row(1) << kP, kI, kD;
    kP = node_->get_parameter("drift_axes.z.linear_kP").as_double();
    kI = node_->get_parameter("drift_axes.z.linear_kI").as_double();
    kD = node_->get_parameter("drift_axes.z.linear_kD").as_double();
    pid_gain_pmat_.row(2) << kP, kI, kD;

    kP = node_->get_parameter("drift_axes.x.euler_kP").as_double();
    pid_gain_rmat_.x() = kP;
    kP = node_->get_parameter("drift_axes.y.euler_kP").as_double();
    pid_gain_rmat_.y() = kP;
    kP = node_->get_parameter("drift_axes.z.euler_kP").as_double();
    pid_gain_rmat_.z() = kP;

    min_x = node_->get_parameter("drift_axes.x.pid_min").as_double();
    max_x = node_->get_parameter("drift_axes.x.pid_max").as_double();
    min_y = node_->get_parameter("drift_axes.y.pid_min").as_double();
    max_y = node_->get_parameter("drift_axes.y.pid_max").as_double();
    min_z = node_->get_parameter("drift_axes.z.pid_min").as_double();
    max_z = node_->get_parameter("drift_axes.z.pid_max").as_double();

    enabled = node_->get_parameter("antidrift_enable").as_bool();
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
        [this](const geometry_msgs::msg::TwistStamped& msg) {
            timer_count_ = 0.0;
            latest_msg_ = msg;
        }
    );
    
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_corrected_cmds", 10);
    
    pub_period_ = 1.0 / node_->get_parameter("publish_rate").as_double();
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds((uint32_t)(pub_period_ * 1000.0)),
        std::bind(&ServoAntiDrifter::_timer_callback, this));
}

void ServoAntiDrifter::_timer_callback(void)
{
    if (timer_count_ >= 0.5 || !planning_scene_monitor_->getStateMonitor()->haveCompleteState())
    {
        return;
    }

    timer_count_ += pub_period_;
    if (!enabled || (latest_msg_.twist.linear.x == 0.0 && latest_msg_.twist.linear.y == 0.0 && latest_msg_.twist.linear.z == 0.0))
    {
        twist_pub_->publish(latest_msg_);
        state_changed = true;
        return;
    }
    else if ((std::abs(latest_msg_.twist.linear.x) < t_dz && std::abs(latest_msg_.twist.linear.x) >= t_dz) ||
            (std::abs(latest_msg_.twist.linear.y) < t_dz && std::abs(latest_msg_.twist.linear.y) >= t_dz) ||
            (std::abs(latest_msg_.twist.linear.z) < t_dz && std::abs(latest_msg_.twist.linear.z) >= t_dz))
    {
        state_changed = true;
    }

    prev_msg_ = latest_msg_;
    current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    const Eigen::Matrix3d planning_to_cmd_frame_transform = (
        current_state_->getGlobalLinkTransform(latest_msg_.header.frame_id).inverse() * 
        current_state_->getGlobalLinkTransform(planning_frame)
    ).linear();

    Eigen::Vector3d eef_pos_cmd_frame = planning_to_cmd_frame_transform * current_state_->getGlobalLinkTransform(ee_frame).translation();
    Eigen::Quaterniond eef_rot_cmd_frame(current_state_->getGlobalLinkTransform(ee_frame).linear());
    if (state_changed)
    {
        state_changed = false;
        projected_eef_pos_ = eef_pos_cmd_frame;
        projected_eef_rot_ = eef_rot_cmd_frame;
        tvec_err_int = Eigen::RowVector3d::Zero();
    }
    Eigen::Vector3d tvec_cmd_vel{latest_msg_.twist.linear.x, latest_msg_.twist.linear.y, latest_msg_.twist.linear.z};
    Eigen::Vector3d rvec_cmd_vel{latest_msg_.twist.angular.x, latest_msg_.twist.angular.y, latest_msg_.twist.angular.z};

    Eigen::VectorXd q_dot;
    current_state_->copyJointGroupVelocities(jmg_name, q_dot);
    const Eigen::MatrixXd jacobian = current_state_->getJacobian(current_state_->getJointModelGroup(jmg_name));
    Eigen::Matrix3d tvec_eef_vel = (jacobian * q_dot).topRows<3>().asDiagonal();

    // Assuming commands are unitless:
    projected_eef_pos_ += tvec_eef_vel * tvec_cmd_vel * pub_period_;
    pos_refresh_count_ += pub_period_;

    Eigen::Vector3d current_eef_pos = eef_pos_cmd_frame;
    if (pos_refresh_count_ >= 0.1)
    {
        pos_refresh_count_ = 0.0;
        if (tvec_cmd_vel.x() != 0.0) projected_eef_pos_.x() = current_eef_pos.x();
        if (tvec_cmd_vel.y() != 0.0) projected_eef_pos_.y() = current_eef_pos.y();
        if (tvec_cmd_vel.z() != 0.0) projected_eef_pos_.z() = current_eef_pos.z();
    }

    // Next up, construct the error matrix and apply the PID gains
    Eigen::RowVector3d tvec_error = (projected_eef_pos_ - current_eef_pos).transpose();
    tvec_err_int += tvec_error * pub_period_;
    Eigen::RowVector3d tvec_err_deriv = (tvec_error - tvec_err_prev) / pub_period_;
    tvec_err_prev = tvec_error;
    Eigen::Matrix3d matrix_err;
    matrix_err << tvec_error, tvec_err_int, tvec_err_deriv;

    Eigen::Quaternion q_error = projected_eef_rot_ * eef_rot_cmd_frame.conjugate();
    if (q_error.w() < 0.0)
    {
        q_error.coeffs() *= -1.0;
    }
    Eigen::Vector3d rvec_error = 2 * atan(q_error.vec().norm() / q_error.w()) * q_error.vec().normalized();
    double temp_x, temp_y, temp_z;
    temp_x = rvec_error.x();
    temp_y = rvec_error.y();
    temp_z = rvec_error.z();
    rvec_error.x() = -temp_y;
    rvec_error.y() = -temp_x;
    rvec_error.z() = temp_z;
    
    // RCLCPP_INFO(node_->get_logger(), "tvec_error: [%f, %f, %f], rvec_error: [%f, %f, %f]", 
    //             tvec_error.x(), tvec_error.y(), tvec_error.z(), rvec_error.x(), rvec_error.y(), rvec_error.z());

    // The values we care for are on the diagonal
    Eigen::Vector3d pid_output_ = (pid_gain_pmat_ * matrix_err).diagonal();
    Eigen::Vector3d pid_r_output_ = pid_gain_rmat_.cwiseProduct(rvec_error);
    pid_output_.x() = std::clamp(pid_output_.x(), min_x, max_x);
    pid_output_.y() = std::clamp(pid_output_.y(), min_y, max_y);
    pid_output_.z() = std::clamp(pid_output_.z(), min_z, max_z);
    pid_r_output_.x() = std::clamp(pid_r_output_.x(), min_x, max_x);
    pid_r_output_.y() = std::clamp(pid_r_output_.y(), min_y, max_y);
    pid_r_output_.z() = std::clamp(pid_r_output_.z(), min_z, max_z);
    Eigen::Vector3d corrected_tvec = tvec_cmd_vel + pid_output_;
    Eigen::Vector3d corrected_rvec = rvec_cmd_vel + pid_r_output_;

    geometry_msgs::msg::TwistStamped corrected_twist_msg_;
    corrected_twist_msg_.header = latest_msg_.header;
    corrected_twist_msg_.twist.linear.x = std::clamp(corrected_tvec.x(), -1.0, 1.0);
    corrected_twist_msg_.twist.linear.y = std::clamp(corrected_tvec.y(), -1.0, 1.0);
    corrected_twist_msg_.twist.linear.z = std::clamp(corrected_tvec.z(), -1.0, 1.0);
    corrected_twist_msg_.twist.angular.x = std::clamp(corrected_rvec.x(), -1.0, 1.0);
    corrected_twist_msg_.twist.angular.y = std::clamp(corrected_rvec.y(), -1.0, 1.0);
    corrected_twist_msg_.twist.angular.z = std::clamp(corrected_rvec.z(), -1.0, 1.0);
    twist_pub_->publish(corrected_twist_msg_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ServoAntiDrifter)