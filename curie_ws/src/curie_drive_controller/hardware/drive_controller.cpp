#include "curie_drive_controller/drive_controller.hpp"

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

    // if (drive_hardware_.initialize() < 0)
    // {
    //     return hardware_interface::CallbackReturn::ERROR;
    // }
    // t_controller_ = std::thread(&hardware::SparkDriveInterface::run, &drive_hardware_);
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
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurieDiffDriveController::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    return hardware_interface::return_type::OK;
}

CurieDiffDriveController::~CurieDiffDriveController()
{
    // drive_hardware_.shutdown();
    // if (t_controller_.joinable())
    // {
    //     t_controller_.join();
    // }
}

}  // namespace curie_drive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  curie_drive_controller::CurieDiffDriveController, hardware_interface::SystemInterface)