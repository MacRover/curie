#include "curie_drive_controller/drive_controller.hpp"
#include <math.h>

#define RPM_TO_RADPS (2.0f * M_PI / 60.0f)

namespace curie_drive_controller
{
hardware_interface::CallbackReturn CurieDiffDriveController::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (drive_hardware_.initialize() < 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    drive_hw_thread_ = std::thread(&hardware::SparkDriveInterface::run, &drive_hardware_);
    wheel_velocities_.resize(info_.joints.size(), 0.0);
    wheel_positions_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CurieDiffDriveController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        info_.joints[i].name, "position", &wheel_positions_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &wheel_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CurieDiffDriveController::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CurieDiffDriveController::on_activate(
            const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurieDiffDriveController::on_deactivate(
            const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CurieDiffDriveController::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    int8_t ret = drive_hardware_.read(static_cast<void*>(&drive_status_));

    if (ret < 0)
    {
        return hardware_interface::return_type::ERROR;
    }
    
    // Convert from RPM to rad/s
    wheel_velocities_[0] = drive_status_.fl_status.primaryEncoderVelocity * RPM_TO_RADPS;
    wheel_velocities_[1] = drive_status_.fr_status.primaryEncoderVelocity * RPM_TO_RADPS;
    wheel_velocities_[2] = drive_status_.ml_status.primaryEncoderVelocity * RPM_TO_RADPS;
    wheel_velocities_[3] = drive_status_.mr_status.primaryEncoderVelocity * RPM_TO_RADPS;
    wheel_velocities_[4] = drive_status_.bl_status.primaryEncoderVelocity * RPM_TO_RADPS;
    wheel_velocities_[5] = drive_status_.br_status.primaryEncoderVelocity * RPM_TO_RADPS;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurieDiffDriveController::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    drive_commands_.fl_velocity = hw_commands_[0] / RPM_TO_RADPS;
    drive_commands_.fr_velocity = hw_commands_[1] / RPM_TO_RADPS;
    drive_commands_.ml_velocity = hw_commands_[2] / RPM_TO_RADPS;
    drive_commands_.mr_velocity = hw_commands_[3] / RPM_TO_RADPS;
    drive_commands_.bl_velocity = hw_commands_[4] / RPM_TO_RADPS;
    drive_commands_.br_velocity = hw_commands_[5] / RPM_TO_RADPS;

    drive_hardware_.write(static_cast<void*>(&drive_commands_));

    return hardware_interface::return_type::OK;
}

CurieDiffDriveController::~CurieDiffDriveController()
{
    drive_hardware_.shutdown();
    drive_hw_thread_.join();
}

}  // namespace curie_drive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  curie_drive_controller::CurieDiffDriveController, hardware_interface::SystemInterface)