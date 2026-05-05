#include "curie_drive_controller/drive_controller.hpp"
#include <math.h>

#define RAD_TO_DEG (360.0f / (2.0f * M_PI))
#define RADPS_TO_DEGPM (60.0f * RAD_TO_DEG)
#define VELOCITY_DEADZONE 0.1f

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

    bool use_vcan_interface;
    std::string use_vcan_hw_param = info_.hardware_parameters["use_vcan"];
    std::transform(use_vcan_hw_param.begin(), use_vcan_hw_param.end(), use_vcan_hw_param.begin(), ::tolower);
    std::istringstream(use_vcan_hw_param) >> std::boolalpha >> use_vcan_interface;
    if (drive_hardware_.initialize(&use_vcan_interface) < 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    drive_hw_thread_ = std::thread(&hardware::SparkDriveInterface::run, &drive_hardware_);
    wheel_velocities_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    wheel_state_ = true;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CurieDiffDriveController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
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
    
    // Convert from deg/s to rad/s
    wheel_velocities_[0] = drive_status_.drive.fl_status.primaryEncoderVelocity / RADPS_TO_DEGPM;
    wheel_velocities_[1] = drive_status_.drive.fr_status.primaryEncoderVelocity / RADPS_TO_DEGPM;
    wheel_velocities_[2] = drive_status_.drive.ml_status.primaryEncoderVelocity / RADPS_TO_DEGPM;
    wheel_velocities_[3] = drive_status_.drive.mr_status.primaryEncoderVelocity / RADPS_TO_DEGPM;
    wheel_velocities_[4] = drive_status_.drive.bl_status.primaryEncoderVelocity / RADPS_TO_DEGPM;
    wheel_velocities_[5] = drive_status_.drive.br_status.primaryEncoderVelocity / RADPS_TO_DEGPM;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurieDiffDriveController::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // Only send commands when necessary (i.e. when the vehicle is moving)
    bool vehicle_moving_ = false;
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        if (std::abs(hw_commands_[i]) > VELOCITY_DEADZONE)
        {
            vehicle_moving_ = true;
            wheel_state_ = true;
            break;
        }
    }

    if (wheel_state_)
    {
        drive_commands_.drive.fl_velocity = (vehicle_moving_) ? hw_commands_[0] * RADPS_TO_DEGPM : 0.0f;
        drive_commands_.drive.fr_velocity = (vehicle_moving_) ? hw_commands_[1] * RADPS_TO_DEGPM : 0.0f;
        drive_commands_.drive.ml_velocity = (vehicle_moving_) ? hw_commands_[2] * RADPS_TO_DEGPM : 0.0f;
        drive_commands_.drive.mr_velocity = (vehicle_moving_) ? hw_commands_[3] * RADPS_TO_DEGPM : 0.0f;
        drive_commands_.drive.bl_velocity = (vehicle_moving_) ? hw_commands_[4] * RADPS_TO_DEGPM : 0.0f;
        drive_commands_.drive.br_velocity = (vehicle_moving_) ? hw_commands_[5] * RADPS_TO_DEGPM : 0.0f;

        drive_hardware_.write(static_cast<void*>(&drive_commands_));

        if (!vehicle_moving_)
        {
            wheel_state_ = false;
        }
    }

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