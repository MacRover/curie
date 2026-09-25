#include "curie_arm_controller/arm_controller.hpp"
#include <math.h>

#define RAD_TO_DEG (360.0f / (2.0f * M_PI))
#define RADPS_TO_DEGPM (60.0f * RAD_TO_DEG)

namespace curie_arm_controller
{
hardware_interface::CallbackReturn CurieArmController::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    bool use_vcan_interface;
    std::string use_vcan_hw_param = info_.hardware_parameters["use_vcan"];
    std::transform(use_vcan_hw_param.begin(), use_vcan_hw_param.end(), use_vcan_hw_param.begin(), ::tolower);
    std::istringstream(use_vcan_hw_param) >> std::boolalpha >> use_vcan_interface;

    std::string filter_state_interfaces_param = info_.hardware_parameters["filter_state_interfaces"];

    std::transform(filter_state_interfaces_param.begin(), filter_state_interfaces_param.end(), filter_state_interfaces_param.begin(), ::tolower);
    std::istringstream(filter_state_interfaces_param) >> std::boolalpha >> filter_state_interfaces_;

    RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "Virtual CAN is %s. Using CAN interface '%s'.",
        use_vcan_interface ? "enabled" : "disabled",
        use_vcan_interface ? "vcan0" : "can0"
    );

    RCLCPP_INFO(
        rclcpp::get_logger("ArmSystem"),
        "Encoder state filtering is %s.",
        filter_state_interfaces_ ? "enabled" : "disabled"
    );

int8_t arm_init_result = arm_hardware_.initialize(&use_vcan_interface);

if (arm_init_result == static_cast<int8_t>(hardware::SparkArmInitResult::CAN_OPEN_ERROR)){
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Failed to open CAN interface '%s'.", use_vcan_interface ? "vcan0" : "can0");

    return hardware_interface::CallbackReturn::ERROR;
}

if (arm_init_result == static_cast<int8_t>(hardware::SparkArmInitResult::DEVICE_COMMUNICATION_ERROR)){
    RCLCPP_ERROR(
        rclcpp::get_logger("ArmSystem"),
        "CAN interface opened, but communication with one or more "
        "arm devices failed during initialization."
    );

    return hardware_interface::CallbackReturn::ERROR;
}

int8_t arm_vel_init_result = arm_vel_hardware_.initialize(&use_vcan_interface);

if (arm_vel_init_result < 0){
    
    RCLCPP_ERROR(
        rclcpp::get_logger("ArmSystem"),
        "Failed to initialize arm velocity hardware on CAN interface '%s'.",
        use_vcan_interface ? "vcan0" : "can0"
    );

    return hardware_interface::CallbackReturn::ERROR;
}
    arm_hw_thread_ = std::thread(&hardware::SparkArmInterface::run, &arm_hardware_);
    joint_velocities_.resize(info_.joints.size(), 0.0);
    joint_positions_.resize(info_.joints.size(), 0.0);
    raw_joint_positions_.resize(info_.joints.size(), 0.0);
    joint_lpfs_.resize(info_.joints.size());
    hw_pos_commands_.resize(info_.joints.size(), 0.0);
    hw_vel_commands_.resize(info_.joints.size(), 0.0);
    hw_pos_commands_prev_.resize(info_.joints.size(), 0.0);
    
    lpf_damping_frequency_ = std::stod(info_.hardware_parameters["lpf_damping_frequency"]);

    lpf_damping_intensity_ = std::stod(info_.hardware_parameters["lpf_damping_intensity"]);

    memset(&status_, 0, sizeof(status_));
    arm_vel_state_ = false;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CurieArmController::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.joints[i].name, "position", &joint_positions_[i]);

        state_interfaces.emplace_back(
            info_.joints[i].name, "velocity", &joint_velocities_[i]);

        state_interfaces.emplace_back(
            info_.joints[i].name, "raw_position", &raw_joint_positions_[i]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CurieArmController::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name, "position", &hw_pos_commands_[i]);
    command_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_vel_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CurieArmController::on_activate(
            const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurieArmController::on_deactivate(
            const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CurieArmController::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;

    if (arm_hardware_.read(static_cast<void*>(&status_)) < 0)
    {
        return hardware_interface::return_type::ERROR;
    }

    if (filter_state_interfaces_ && !lpf_configured_)  // filter state interfaces enable and LPF not yet configured
    {
        const double sampling_frequency = 1.0 / period.seconds();

        for (size_t i = 0; i < joint_positions_.size(); i++)
        {
            joint_lpfs_[i].set_params(
                sampling_frequency,
                lpf_damping_frequency_,
                lpf_damping_intensity_);

            joint_lpfs_[i].configure();
        }

        RCLCPP_INFO(
            rclcpp::get_logger("ArmSystem"),
            "Encoder LPF configured: sampling=%.1f Hz, damping=%.1f Hz, intensity=%.1f",
            sampling_frequency,
            lpf_damping_frequency_,
            lpf_damping_intensity_);

        lpf_configured_ = true;
    }

    // Convert raw encoder positions from degrees to radians
    double raw_base = status_.arm.base_status.dutyCycleEncPosition / RAD_TO_DEG;

    double raw_shoulder = status_.arm.shoulder_status.dutyCycleEncPosition / RAD_TO_DEG;

    double raw_elbow = status_.arm.elbow_status.dutyCycleEncPosition / RAD_TO_DEG;

    double raw_wrist_pitch = status_.arm.wrist_pitch_status.dutyCycleEncPosition / RAD_TO_DEG;

    double raw_wrist_roll = status_.arm.wrist_roll_status.dutyCycleEncPosition / RAD_TO_DEG;

    double raw_gripper = status_.arm.gripper_status.dutyCycleEncPosition / RAD_TO_DEG;

    // Shift raw encoder domain/angle from [0, 2*pi] to [-pi, pi]
    if (raw_base > M_PI)
        raw_base -= 2.0 * M_PI;

    if (raw_shoulder > M_PI)
        raw_shoulder -= 2.0 * M_PI;

    if (raw_elbow > M_PI)
        raw_elbow -= 2.0 * M_PI;

    if (raw_wrist_pitch > M_PI)
        raw_wrist_pitch -= 2.0 * M_PI;

    if (raw_wrist_roll > M_PI)
        raw_wrist_roll -= 2.0 * M_PI;

    if (raw_gripper > M_PI)
        raw_gripper -= 2.0 * M_PI;

    // Store unfiltered encoder positions for testing
    raw_joint_positions_[0] = raw_base;
    raw_joint_positions_[1] = raw_shoulder;
    raw_joint_positions_[2] = raw_elbow;
    raw_joint_positions_[3] = raw_wrist_pitch;
    raw_joint_positions_[4] = raw_wrist_roll;
    raw_joint_positions_[5] = raw_gripper;
    
    // Raw encoder -> raw_joint_positions_ -> joint_positions_ (filtered or unfiltered)
    if (filter_state_interfaces_)
    {
        if (!lpf_initialized_)
        {
            joint_positions_ = raw_joint_positions_; // Initialize joint positions with raw values on first read

            for (size_t i = 0; i < joint_lpfs_.size(); i++)
            {
                double dummy_output; // Every filter needs an output variable to update
                joint_lpfs_[i].update(raw_joint_positions_[i], dummy_output);
            }

            lpf_initialized_ = true;

        }else{

            for (size_t i = 0; i < joint_lpfs_.size(); i++)
            {
                joint_lpfs_[i].update(raw_joint_positions_[i], joint_positions_[i]);
            }
        }
    }else{

        joint_positions_ = raw_joint_positions_;
    }

    joint_velocities_[0] = status_.arm.base_status.dutyCycleEncVelocity / RAD_TO_DEG;
    joint_velocities_[1] = status_.arm.shoulder_status.dutyCycleEncVelocity / RAD_TO_DEG;
    joint_velocities_[2] = status_.arm.elbow_status.dutyCycleEncVelocity / RAD_TO_DEG;
    joint_velocities_[3] = status_.arm.wrist_pitch_status.dutyCycleEncVelocity / RAD_TO_DEG;
    joint_velocities_[4] = status_.arm.wrist_roll_status.dutyCycleEncVelocity / RAD_TO_DEG;
    joint_velocities_[5] = status_.arm.gripper_status.dutyCycleEncVelocity / RAD_TO_DEG;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurieArmController::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    commands_.arm.base_position = hw_pos_commands_[0] * RAD_TO_DEG;
    commands_.arm.shoulder_position = hw_pos_commands_[1] * RAD_TO_DEG;
    commands_.arm.elbow_position = hw_pos_commands_[2] * RAD_TO_DEG;
    commands_.arm.wrist_pitch_position = hw_pos_commands_[3] * RAD_TO_DEG;
    commands_.arm.wrist_roll_position = hw_pos_commands_[4] * RAD_TO_DEG;
    commands_.arm.gripper_position = hw_pos_commands_[5] * RAD_TO_DEG;

    commands_.arm.base_velocity = hw_vel_commands_[0] * RADPS_TO_DEGPM;
    commands_.arm.shoulder_velocity = hw_vel_commands_[1] * RADPS_TO_DEGPM;
    commands_.arm.elbow_velocity = hw_vel_commands_[2] * RADPS_TO_DEGPM;
    commands_.arm.wrist_pitch_velocity = hw_vel_commands_[3] * RADPS_TO_DEGPM;
    commands_.arm.wrist_roll_velocity = hw_vel_commands_[4] * RADPS_TO_DEGPM;

    // Override position commands when in teleop/velocity mode
    for (size_t i = 0; i < hw_vel_commands_.size(); i++)
    {
        if (arm_vel_state_ && std::abs(hw_pos_commands_[i] - hw_pos_commands_prev_[i]) > 1e-5)
        {
            arm_vel_state_ = false;
        }
        if (std::abs(hw_vel_commands_[i]) > 0.01)
        {
            arm_vel_state_ = true;
            hw_pos_commands_prev_ = hw_pos_commands_;
            break;
        }
    }

    // RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "ARM COMMANDS: Base: %.2f, Shoulder: %.2f, Elbow: %.2f, Wrist Pitch: %.2f, Wrist Roll: %.2f",
    //     commands_.arm.base_position, commands_.arm.shoulder_position, commands_.arm.elbow_position,
    //     commands_.arm.wrist_pitch_position, commands_.arm.wrist_roll_position);

    if (arm_vel_state_)
    {
        if (arm_vel_hardware_.write(static_cast<void*>(&commands_)) < 0)
        {
            return hardware_interface::return_type::ERROR;
        }
    }
    else if (arm_hardware_.write(static_cast<void*>(&commands_)) < 0)
    {
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

CurieArmController::~CurieArmController()
{
    arm_hardware_.shutdown();
    arm_vel_hardware_.shutdown();
    arm_hw_thread_.join();
}

}  // namespace curie_arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  curie_arm_controller::CurieArmController, hardware_interface::SystemInterface)