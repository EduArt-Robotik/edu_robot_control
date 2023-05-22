/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace control {
namespace activation_function {

template <typename Type = float>
inline Type linear(const Type joy_in, const Type max_velocity) {
  return joy_in * max_velocity;
}

template <typename Type = float>
inline Type quadric(const Type joy_in, const Type max_velocity) {
  return joy_in * joy_in * max_velocity * static_cast<Type>(joy_in < 0.0 ? -1.0 : 1.0);
}

} // end namespace activation function
} // end namespace control
} // end namespace eduart
