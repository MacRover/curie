#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "spark_mmrt/can/SocketCanTransport.hpp"
#include "spark_mmrt/device/roboRIO.hpp"

class SparkMaxHeartbeat : public rclcpp::Node
{
public:
    SparkMaxHeartbeat(std::string name);
private:
    void _enable_callback(const std::shared_ptr<rmw_request_id_t> request_header, 
                   const std_srvs::srv::SetBool::Request::SharedPtr req, 
                   std_srvs::srv::SetBool::Response::SharedPtr resp);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;

    spark_mmrt::can::SocketCanTransport can_transport_;
    RoboRIO roboRIO_;
    bool enabled;
};