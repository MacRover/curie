#pragma once
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "spark_mmrt/can/SocketCanTransport.hpp"
#include "spark_mmrt/device/SparkMax.hpp"

class TestSparkMax : public rclcpp::Node
{
public:
    TestSparkMax(std::string name);
private:
    void _callback(const std_msgs::msg::Float32::SharedPtr msg);
    void _timer_callback(void);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    spark_mmrt::can::SocketCanTransport can_transport_;
    SparkMax spark_max_;

};