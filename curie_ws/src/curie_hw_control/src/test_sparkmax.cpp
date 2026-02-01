#include "curie_hw_control/test_sparkmax.hpp"

TestSparkMax::TestSparkMax(std::string name) : Node(name), spark_max_(can_transport_, 1)
{
    can_transport_.open("can0", SPARK_DRIVETRAIN);
    sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "sparkmax/duty_cycle_cmd",
        10,
        std::bind(&TestSparkMax::_callback, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TestSparkMax::_timer_callback, this)
    );
}

void TestSparkMax::_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float duty_cycle = msg->data;
    spark_max_.setDutyCycle(duty_cycle);
}

void TestSparkMax::_timer_callback(void)
{
    spark_max_.heartbeat();
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSparkMax>("test_sparkmax_node"));
  rclcpp::shutdown();

  return 0;
}
