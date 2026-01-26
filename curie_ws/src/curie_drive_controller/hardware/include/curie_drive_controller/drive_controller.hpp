#ifndef CURIE_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_
#define CURIE_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_

#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
// From curie_hw_control
#include "spark_drive_hardware.hpp"

namespace curie_drive_controller
{
    class CurieDiffDriveController : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(CurieDiffDriveController)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        ~CurieDiffDriveController() override;

    private:
        std::thread t_controller_;
        hardware::SparkDriveInterface drive_hardware_;
        std::vector<double> wheel_velocities_;
        std::vector<double> wheel_positions_;
        std::vector<double> hw_commands_;

        SparkDriveCmd drive_commands_;
        SparkDriveStatus drive_status_;
    };
}
#endif  // CURIE_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_