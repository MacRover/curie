#include <chrono>
#include <memory>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class GripperStallGuard : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  GripperStallGuard()
  : Node("gripper_stall_guard")
  {
    target_position_ = this->declare_parameter<double>(
      "target_position", 0.5);

    max_effort_ = this->declare_parameter<double>(
      "max_effort", 1.0);

    action_client_ =
      rclcpp_action::create_client<GripperCommand>(
        this,
        "/hand_controller/gripper_cmd");

    start_timer_ = this->create_wall_timer(
      500ms,
      [this]()
      {
        send_initial_goal();
      });
  }

private:
  void send_initial_goal()
  {
    start_timer_->cancel();

    if (!action_client_->wait_for_action_server(5s))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Gripper action server is unavailable");

      rclcpp::shutdown();
      return;
    }

    GripperCommand::Goal goal;
    goal.command.position = target_position_;
    goal.command.max_effort = max_effort_;

    auto send_goal_options =
      rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle)
      {
        if (!goal_handle)
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "Initial gripper goal was rejected");

          rclcpp::shutdown();
          return;
        }

        RCLCPP_INFO(
          this->get_logger(),
          "Initial gripper goal accepted");
      };

    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & result)
      {
        handle_initial_result(result);
      };

    RCLCPP_INFO(
      this->get_logger(),
      "Sending gripper goal: position=%.4f, max_effort=%.4f",
      target_position_,
      max_effort_);

    action_client_->async_send_goal(
      goal,
      send_goal_options);
  }

  void handle_initial_result(
    const GoalHandle::WrappedResult & result)
  {
    if (!result.result)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Initial gripper goal returned no result");

      rclcpp::shutdown();
      return;
    }

    double stall_position = result.result->position;

    RCLCPP_INFO(
      this->get_logger(),
      "Initial result: position=%.4f, stalled=%s, reached_goal=%s",
      stall_position,
      result.result->stalled ? "true" : "false",
      result.result->reached_goal ? "true" : "false");

    if (!result.result->stalled)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Gripper did not stall");

      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Stall detected at position %.4f",
      stall_position);

    send_hold_goal(stall_position);
  }

  void send_hold_goal(double hold_position)
  {
    GripperCommand::Goal goal;
    goal.command.position = hold_position;
    goal.command.max_effort = max_effort_;

    auto send_goal_options =
      rclcpp_action::Client<GripperCommand>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle)
      {
        if (!goal_handle)
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "Hold goal was rejected");

          rclcpp::shutdown();
          return;
        }

        hold_goal_handle_ = goal_handle;

        RCLCPP_INFO(
          this->get_logger(),
          "Hold goal accepted, sending cancel request");

        action_client_->async_cancel_goal(
          goal_handle,
          [this](auto cancel_response)
          {
            if (cancel_response->goals_canceling.empty())
            {
              RCLCPP_ERROR(
                this->get_logger(),
                "Hold goal cancel request was rejected");

              rclcpp::shutdown();
              return;
            }

            RCLCPP_INFO(
              this->get_logger(),
              "Hold goal cancel request accepted");
          });
      };

    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Gripper is now holding the stall position");
        }
        else
        {
          RCLCPP_WARN(
            this->get_logger(),
            "Hold goal finished without a canceled result");
        }

        rclcpp::shutdown();
      };

    RCLCPP_INFO(
      this->get_logger(),
      "Sending hold goal at position %.4f",
      hold_position);

    action_client_->async_send_goal(
      goal,
      send_goal_options);
  }

  rclcpp_action::Client<GripperCommand>::SharedPtr
    action_client_;

  GoalHandle::SharedPtr hold_goal_handle_;

  rclcpp::TimerBase::SharedPtr start_timer_;

  double target_position_;
  double max_effort_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperStallGuard>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}