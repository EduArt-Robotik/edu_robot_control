#include "remote_control.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <edu_robot/msg/detail/mode__struct.hpp>
#include <edu_robot/srv/detail/set_mode__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <memory>
#include <functional>
#include <stdexcept>
#include <string>

namespace eduart {

constexpr std::array<ButtonInterpreter, 2u> unassigned_buttons = {
  ButtonInterpreter(Command::Disable, "disable", 8u),
  ButtonInterpreter(Command::Enable, "enable", 9u)
};

constexpr std::array<AxisInterpreter, 4u> unassigned_joy_axis = {
  AxisInterpreter(Command::Forward, "forward", 1u),
  AxisInterpreter(Command::Left, "left", 0u),
  AxisInterpreter(Command::Turn, "turn", 2u),
  AxisInterpreter(Command::Throttle, "throttle", 3u)
};

RemoteControl::RemoteControl() : rclcpp::Node("remote_control")
{
  // Do ROS related initializations.
  _sub_joy = create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::QoS(2).best_effort().durability_volatile(),
    std::bind(&RemoteControl::callbackJoy, this, std::placeholders::_1)
  );
  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::QoS(2).best_effort().durability_volatile()
  );
  _client_set_mode = create_client<edu_robot::srv::SetMode>("set_mode");

  // Do mapping of the joy buttons.
  const std::string parameter_prefix = "joy_mapping";

  for (const auto& button : unassigned_buttons) {
    declare_parameter<int>(parameter_prefix + "/button/" + button.parameterName(), button.defaultIndex());
  }
  for (const auto& button : unassigned_buttons) {
    const std::size_t index = static_cast<std::size_t>(
      get_parameter(parameter_prefix + "/button/" + button.parameterName()).as_int()
    );
    const auto search = _button_mapping.find(index);

    if (search != _button_mapping.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Button index " << index << " is already in use. Skipping button \""
                                        << button.parameterName() << "\"");
      continue;                                        
    }

    RCLCPP_INFO_STREAM(get_logger(), "Assign button \"" << button.parameterName() << "\" to index " << index << ".");
    _button_mapping[index] = button;
  }

  // Do mapping of the joy axes.
  for (const auto& axis : unassigned_joy_axis) {
    declare_parameter<int>(parameter_prefix + "/axis/" + axis.parameterName(), axis.defaultIndex());
  }
  for (const auto& axis : unassigned_joy_axis) {
    const std::size_t index = static_cast<std::size_t>(
      get_parameter(parameter_prefix + "/axis/" + axis.parameterName()).as_int()
    );
    const auto search = _axis_mapping.find(index);

    if (search != _axis_mapping.end()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Button index " << index << " is already in use. Skipping button \""
                                        << axis.parameterName() << "\"");
      continue;                                        
    }

    RCLCPP_INFO_STREAM(get_logger(), "Assign axis \"" << axis.parameterName() << "\" to index " << index << ".");
    _axis_mapping[index] = axis;
  }

  // Get other parameters.
  declare_parameter<float>("max_linear_velocity", _parameter.max_linear_velocity);
  _parameter.max_linear_velocity = get_parameter("max_linear_velocity").as_double();
  declare_parameter<float>("max_angular_velocity", _parameter.max_angular_velocity);
  _parameter.max_angular_velocity = get_parameter("max_angular_velocity").as_double();
}

RemoteControl::~RemoteControl()
{

}

void RemoteControl::callbackJoy(std::shared_ptr<const sensor_msgs::msg::Joy> msg)
{
  // Clear Tx Message
  geometry_msgs::msg::Twist twist_cmd;

  twist_cmd.angular.x = 0.0;
  twist_cmd.angular.y = 0.0;
  twist_cmd.angular.z = 0.0;

  twist_cmd.linear.x = 0.0;
  twist_cmd.linear.y = 0.0;
  twist_cmd.linear.z = 0.0;

  // Axes
  for (const auto& axis : _axis_mapping) {
    if (axis.first >= msg->axes.size()) {
      RCLCPP_FATAL_STREAM(get_logger(), "Configuration error. Axis \"" << axis.second.parameterName() << "\" got index = "
                                        << axis.first << " which is out of range of received joy message.");
      throw std::runtime_error("Out of range index in AxisInterpreter.");                                        
    }

    switch (axis.second.command()) {
      case Command::Forward: 
        // expect axis values between 0 and 1, however it is ensured that the max speed is not exceeded.
        twist_cmd.linear.x = std::min(_parameter.max_linear_velocity,
                                      msg->axes[axis.first] * _parameter.max_linear_velocity);
        break;

      case Command::Left:
        // expect axis values between 0 and 1, however it is ensured that the max speed is not exceeded.
        twist_cmd.linear.y = std::min(_parameter.max_linear_velocity,
                                      msg->axes[axis.first] * _parameter.max_linear_velocity);      
        break;

      case Command::Turn:
        // expect axis values between 0 and 1, however it is ensured that the max speed is not exceeded.
        twist_cmd.angular.z = std::min(_parameter.max_angular_velocity,
                                       msg->axes[axis.first] * _parameter.max_angular_velocity);
        break;

      case Command::Throttle:
        // \todo clarify functionality
        break;

      default:
        throw std::runtime_error("Unexpected axis command.");
    }
  }

  // Buttons
  for (auto& button : _button_mapping) {
    if (button.first >= msg->buttons.size()) {
      RCLCPP_FATAL_STREAM(get_logger(), "Configuration error. Button \"" << button.second.parameterName() << "\" got index = "
                                        << button.first << " which is out of range of received joy message.");
      throw std::runtime_error("Out of range index in ButtonInterpreter.");        
    }
    if (false == button.second.hasJustBeenPressed(msg->buttons[button.first])) {
      continue;
    }

    switch (button.second.command()) {
      using ResponseFuture = rclcpp::Client<edu_robot::srv::SetMode>::SharedFutureWithRequest;

      case Command::Disable: {
        auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
        request->mode.value = edu_robot::msg::Mode::INACTIVE;

        RCLCPP_INFO(get_logger(), "Send set mode request mode = INACTIVE.");
        _client_set_mode->async_send_request(request, [logger = get_logger()](ResponseFuture future) {
          auto request = future.get().first;
          auto response = future.get().second;

          if (response->state.mode.value != request->mode.value) {
            RCLCPP_ERROR_STREAM(logger, "Can't disable robot! Robot is in mode = " << response->state.mode.value);
            return;
          }

          RCLCPP_INFO(logger, "Set mode INACTIVE successfully.");
        });
        
        break;
      }

      case Command::Enable: {
        auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
        request->mode.value = edu_robot::msg::Mode::REMOTE_CONTROLLED;

        RCLCPP_INFO(get_logger(), "Send set mode request mode = REMOTE_CONTROLLED.");
        _client_set_mode->async_send_request(request, [logger = get_logger()](ResponseFuture future) {
          auto request = future.get().first;
          auto response = future.get().second;

          if (response->state.mode.value != request->mode.value) {
            RCLCPP_ERROR_STREAM(logger, "Can't disable robot! Robot is in mode = " << response->state.mode.value);
            return;
          }

          RCLCPP_INFO(logger, "Set mode REMOTE_CONTROLLED successfully.");
        });

        break;
      }

      default:
        throw std::runtime_error("Unexpected button command.");      
    }
  }

  _pub_twist->publish(twist_cmd);
}

} // end namespace eduart

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::RemoteControl>());
  rclcpp::shutdown();
}
