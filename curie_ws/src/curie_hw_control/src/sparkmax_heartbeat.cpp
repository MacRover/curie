#include "curie_hw_control/sparkmax_heartbeat.hpp"

SparkMaxHeartbeat::SparkMaxHeartbeat(std::string name) : Node(name), roboRIO_(can_transport_), enabled(true)
{
    this->declare_parameter("use_vcan", true);
    can_transport_.open(this->get_parameter("use_vcan").as_bool() ? "vcan0" : "can0");
    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "heartbeat/enable",
        std::bind(&SparkMaxHeartbeat::_enable_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { if (enabled) roboRIO_.heartbeat(); }
    );
}

void SparkMaxHeartbeat::_enable_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr resp)
{
    (void)request_header;
    enabled = req->data;
    resp->success = true;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SparkMaxHeartbeat>("heartbeat"));
  rclcpp::shutdown();

  return 0;
}