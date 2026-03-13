#include "curie_arm_controller/arm_controller.hpp"
#include <math.h>

#define RPM_TO_RADPS (2.0f * M_PI / 60.0f)

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

    if (arm_hardware_.initialize() < 0)
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
        info_.joints[i].name, "velocity", &hw_commands_[i]);
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

    joint_positions_[0] = arm_status_.base_status.dutyCycleEncVPosition * RPM_TO_RADPS;
    joint_positions_[1] = arm_status_.shoulder_status.dutyCycleEncVPosition * RPM_TO_RADPS;
    joint_positions_[2] = arm_status_.elbow_status.dutyCycleEncVPosition * RPM_TO_RADPS;
    joint_positions_[3] = arm_status_.wrist_pitch_status.dutyCycleEncVPosition * RPM_TO_RADPS;
    // joint_positions_[4] = arm_status_.wrist_roll_status.dutyCycleEncVPosition * RPM_TO_RADPS;

    joint_velocities_[0] = arm_status_.base_status.dutyCycleEncVelocity * RPM_TO_RADPS;
    joint_velocities_[1] = arm_status_.shoulder_status.dutyCycleEncVelocity * RPM_TO_RADPS;
    joint_velocities_[2] = arm_status_.elbow_status.dutyCycleEncVelocity * RPM_TO_RADPS;
    joint_velocities_[3] = arm_status_.wrist_pitch_status.dutyCycleEncVelocity * RPM_TO_RADPS;
    // joint_velocities_[4] = arm_status_.wrist_roll_status.dutyCycleEncVelocity * RPM_TO_RADPS;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurieArmController::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    arm_commands_.base_velocity = hw_commands_[0] / RPM_TO_RADPS;
    arm_commands_.shoulder_velocity = hw_commands_[1] / RPM_TO_RADPS;
    arm_commands_.elbow_velocity = hw_commands_[2] / RPM_TO_RADPS;
    arm_commands_.wrist_pitch_velocity = hw_commands_[3] / RPM_TO_RADPS;
    // arm_commands_.wrist_roll_velocity = hw_commands_[4] / RPM_TO_RADPS;

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