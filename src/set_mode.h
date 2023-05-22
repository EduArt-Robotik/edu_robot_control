/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/srv/set_mode.hpp>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>

namespace eduart {
namespace control_function {

std::string get_mode_string(const edu_robot::msg::Mode mode);
void disable(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
void enable(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
void switch_to_skid_kinematic(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
void switch_to_mecanum_kinematic(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
/**
 * \brief Switch to fleet drive mode. 
 * \param node ROS node from where this function is called.
 * \param service_client Service client that will be used for service call.
 */
void enable_fleet_drive(
  rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
/**
 * \brief Disables the fleet drive. The robots will no longer listen to the fleet control and they will be disabled.
 */
void disable_fleet_drive(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
void enable_collision_avoidance_override(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);
void disable_collision_avoidance_override(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client);

} // end namespace control_function
} // end namespace eduart
