/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/client.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <edu_robot/msg/set_lighting_color.hpp>
#include <edu_robot/srv/set_mode.hpp>

#include <memory>
#include <map>
#include <vector>

namespace eduart {

enum class Command {
  None,
  Disable,
  Enable,
  Forward,
  Left,
  Turn,
  Throttle,
  SwitchToSkidDriveKinematic,
  SwitchToMecanumDriveKinematic,
  OverrideCollisionAvoidance,
  EnableFleetDrive,
  IndicateTurnLeft,
  IndicateTurnRight,
  IndicateWarning,
  IndicateOperation,
  IndicateParking,
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
  template<typename... LinkedButtonIndices>
  ButtonInterpreter(const Command command = Command::None, char const* const parameter_name = "",
                    const std::size_t default_index = 0u, const LinkedButtonIndices... linked_indices)
    : JoyInterpreter(command, parameter_name, default_index)
    , _last_state(false)
    , _current_state(false)
  {
    if constexpr (sizeof...(LinkedButtonIndices) > 0) {
      _linked_buttons = std::make_shared<std::vector<std::size_t>>(linked_indices...);
    }
  }

  inline void updateState(const bool current_button_state) {
    _last_state = _current_state;
    _current_state = current_button_state;
  }
  inline bool hasJustBeenPressed() {
    return _last_state == false && _current_state == true;
  }
  inline bool hasJustBeenReleased() {
    return _last_state == true && _current_state == false;
  }

private:
  bool _last_state;
  bool _current_state;
  std::shared_ptr<std::vector<std::size_t>> _linked_buttons = nullptr; //> this buttons has also been pressed
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

  struct function_binding {
    struct {
      std::function<void()> positive = nullptr;
      std::function<void()> negative = nullptr;
    } edge;
  };

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> _sub_joy;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;
  std::shared_ptr<rclcpp::Publisher<edu_robot::msg::SetLightingColor>> _pub_lighting;
  std::shared_ptr<rclcpp::Client<edu_robot::srv::SetMode>> _client_set_mode;
  std::map<Command, function_binding> _command_binding;

  // Expect maximum 20 buttons and 20 axis...
  std::map<std::size_t, ButtonInterpreter> _button_mapping;
  std::map<std::size_t, AxisInterpreter> _axis_mapping;

  Parameter _parameter;
};

} // end namespace eduart