/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstddef>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/client.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <edu_robot/srv/set_mode.hpp>

#include <memory>
#include <map>

namespace eduart {

enum class Command {
  None,
  Disable,
  Enable,
  Forward,
  Left,
  Turn,
  Throttle,
};

class JoyInterpreter
{
public:
  constexpr JoyInterpreter(const Command command, char const* const parameter_name, const std::size_t default_index)
    : _command(command)
    , _parameter_name(parameter_name)
    , _default_index(default_index)
  { }

  inline Command command() const { return _command; }
  inline char const* parameterName() const { return _parameter_name; }
  inline std::size_t defaultIndex() const { return _default_index; }

private:
  Command _command;
  char const* _parameter_name;
  std::size_t _default_index;
};

class ButtonInterpreter : public JoyInterpreter
{
public:
  constexpr ButtonInterpreter(const Command command = Command::None, char const* const parameter_name = "",
                              const std::size_t default_index = 0u)
    : JoyInterpreter(command, parameter_name, default_index)
    , _last_state(false)
  { }

  inline bool hasJustBeenPressed(const bool current_button_state) {
    const bool local_last_state = _last_state;
    _last_state = current_button_state;

    return local_last_state == false && current_button_state == true;
  }

private:
  bool _last_state;
};

class AxisInterpreter : public JoyInterpreter
{
public:
  constexpr AxisInterpreter(const Command command = Command::None, char const* const parameter_name = "",
                            const std::size_t default_index = 0u)
    : JoyInterpreter(command, parameter_name, default_index)
  { }
};

class RemoteControl : public rclcpp::Node
{
public:
  struct Parameter {
    float max_linear_velocity  = 1.0f;
    float max_angular_velocity = M_PI;
  };

  RemoteControl();
  ~RemoteControl() override;

private:
  void callbackJoy(std::shared_ptr<const sensor_msgs::msg::Joy> msg);

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> _sub_joy;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;
  std::shared_ptr<rclcpp::Client<edu_robot::srv::SetMode>> _client_set_mode;

  // Expect maximum 20 buttons and 20 axis...
  std::map<std::size_t, ButtonInterpreter> _button_mapping;
  std::map<std::size_t, AxisInterpreter> _axis_mapping;

  Parameter _parameter;
};

} // end namespace eduart