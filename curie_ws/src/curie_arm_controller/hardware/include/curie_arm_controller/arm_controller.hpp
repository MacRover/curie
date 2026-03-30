#ifndef CURIE_ARM_CONTROLLER__ARM_CONTROLLER_HPP_
#define CURIE_ARM_CONTROLLER__ARM_CONTROLLER_HPP_

#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "curie_hw_control/spark_arm_hardware.hpp"

namespace curie_arm_controller
{
    class CurieArmController : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(CurieArmController)

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
        
        ~CurieArmController() override;

    private:
        std::thread arm_hw_thread_;
        hardware::SparkArmInterface arm_hardware_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_positions_;
        std::vector<double> hw_commands_;

        SparkCommand commands_;
        SparkStatus status_;
    };
}





#endif