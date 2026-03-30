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
    if (arm_hardware_.initialize(&use_vcan_interface) < 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    arm_hw_thread_ = std::thread(&hardware::SparkArmInterface::run, &arm_hardware_);
    joint_velocities_.resize(info_.joints.size(), 0.0);
    joint_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

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
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CurieArmController::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name, "position", &hw_commands_[i]);
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

hardware_interface::return_type CurieArmController::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    if (arm_hardware_.read(static_cast<void*>(&arm_status_)) < 0)
    {
        return hardware_interface::return_type::ERROR;
    }

    joint_positions_[0] = arm_status_.base_status.dutyCycleEncVPosition / RAD_TO_DEG;
    joint_positions_[1] = arm_status_.shoulder_status.dutyCycleEncVPosition / RAD_TO_DEG;
    joint_positions_[2] = arm_status_.elbow_status.dutyCycleEncVPosition / RAD_TO_DEG;
    joint_positions_[3] = arm_status_.wrist_pitch_status.dutyCycleEncVPosition / RAD_TO_DEG;
    // joint_positions_[4] = arm_status_.wrist_roll_status.dutyCycleEncVPosition / RAD_TO_DEG;

    joint_velocities_[0] = arm_status_.base_status.dutyCycleEncVelocity / RADPS_TO_DEGPM;
    joint_velocities_[1] = arm_status_.shoulder_status.dutyCycleEncVelocity / RADPS_TO_DEGPM;
    joint_velocities_[2] = arm_status_.elbow_status.dutyCycleEncVelocity / RADPS_TO_DEGPM;
    joint_velocities_[3] = arm_status_.wrist_pitch_status.dutyCycleEncVelocity / RADPS_TO_DEGPM;
    // joint_velocities_[4] = arm_status_.wrist_roll_status.dutyCycleEncVelocity / RADPS_TO_DEGPM;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurieArmController::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    arm_commands_.base_position = hw_commands_[0] * RAD_TO_DEG;
    arm_commands_.shoulder_position = hw_commands_[1] * RAD_TO_DEG;
    arm_commands_.elbow_position = hw_commands_[2] * RAD_TO_DEG;
    arm_commands_.wrist_pitch_position = hw_commands_[3] * RAD_TO_DEG;
    // arm_commands_.wrist_roll_position = hw_commands_[4] * RAD_TO_DEG;
    arm_commands_.wrist_roll_position = 0.0f;

    if (arm_hardware_.write(static_cast<void*>(&arm_commands_)) < 0)
    {
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

CurieArmController::~CurieArmController()
{
    arm_hardware_.shutdown();
    arm_hw_thread_.join();
}

}  // namespace curie_arm_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  curie_arm_controller::CurieArmController, hardware_interface::SystemInterface)