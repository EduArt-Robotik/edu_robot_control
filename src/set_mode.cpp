#include "set_mode.h"

namespace eduart {
namespace control_function {

using ResponseFuture = rclcpp::Client<edu_robot::srv::SetMode>::SharedFutureWithRequest;

std::string get_mode_string(const edu_robot::msg::Mode mode)
{
  std::string mode_string;

  if (mode.value & edu_robot::msg::Mode::INACTIVE) {
    mode_string += "INACTIVE|";
  }
  if (mode.value & edu_robot::msg::Mode::REMOTE_CONTROLLED) {
    mode_string += "REMOTE CONTROLLED|";
  }
  if (mode.value & edu_robot::msg::Mode::FLEET) {
    mode_string += "FLEET|";
  }
  if (mode.value & edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE_ENABLED) {
    mode_string += "COLLISION_AVOIDANCE_OVERRIDE_ENABLED|";
  }
  if (mode.value & edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE_DISABLED) {
    mode_string += "COLLISION_AVOIDANCE_OVERRIDE_DISABLED|";
  }  
  if (mode.value & edu_robot::msg::Mode::SKID_DRIVE) {
    mode_string += "SKID_DRIVE|";
  }
  if (mode.value & edu_robot::msg::Mode::MECANUM_DRIVE) {
    mode_string += "MECANUM_DRIVE|";
  }

  if (mode_string.empty() == false) {
    mode_string.pop_back();
  }

  return mode_string;
}

void disable(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::INACTIVE;

  RCLCPP_INFO(node.get_logger(), "Send set mode request mode = INACTIVE.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't disable robot! Robot is in mode = " << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Set mode INACTIVE successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());      
    }
  );
}

void enable(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::REMOTE_CONTROLLED;

  RCLCPP_INFO(node.get_logger(), "Send set mode request mode = REMOTE_CONTROLLED.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't enable robot! Robot is in mode = " << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Set mode REMOTE_CONTROLLED successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());      
    }
  );
}

void switch_to_skid_kinematic(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::SKID_DRIVE;

  RCLCPP_INFO(node.get_logger(), "Switch to skid drive kinematic.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't switch to skid drive kinematic! Robot is in mode = "
                                    << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Switched to skid drive kinematic successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());
    }
  );  
}

void switch_to_mecanum_kinematic(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::MECANUM_DRIVE;

  RCLCPP_INFO(node.get_logger(), "Switch to mecanum drive kinematic.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't switch to mecanum drive kinematic! Robot is in mode = "
                                    << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Switched to mecanum drive kinematic successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());
    }
  );  
}

void enable_fleet_drive(
  rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::FLEET;

  RCLCPP_INFO(node.get_logger(), "Switch to fleet drive mode.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't switch to fleet drive mode! Robot is in mode = "
                                    << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Switched to fleet drive mode successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());
    }
  );
}

void disable_fleet_drive(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  disable(node, service_client);
}

void enable_collision_avoidance_override(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE_ENABLED;

  RCLCPP_INFO(node.get_logger(), "Override collision avoidance.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't override collision avoidance! Robot is in mode = "
                                    << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Overriding collision avoidance successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());
    }
  );  
}

void disable_collision_avoidance_override(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.value = edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE_DISABLED;

  RCLCPP_INFO(node.get_logger(), "Disable overriding of collision avoidance.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      auto request = future.get().first;
      auto response = future.get().second;

      if ((response->state.mode.value & request->mode.value) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't disable overriding of collision avoidance! Robot is in mode = "
                                    << get_mode_string(response->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Disabled overriding of collision avoidance successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response->state.mode).c_str());
    }
  );  
}

} // end namespace control_function
} // end namespace eduart
