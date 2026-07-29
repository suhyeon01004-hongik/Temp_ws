#include "vehicle_control/joy_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vehicle_control {
namespace {

float clamp(float value, float minimum, float maximum) {
  return std::max(minimum, std::min(value, maximum));
}

float pedalValue(float axis, bool inverted) {
  if (!std::isfinite(axis)) {
    return 0.0F;
  }
  const float bounded = clamp(axis, -1.0F, 1.0F);
  return inverted ? (1.0F - bounded) * 0.5F
                  : (bounded + 1.0F) * 0.5F;
}

float steeringValue(float axis, bool inverted, float deadzone, float scale,
                    float expo) {
  if (!std::isfinite(axis)) {
    return 0.0F;
  }
  float bounded = clamp(axis, -1.0F, 1.0F);
  if (inverted) {
    bounded = -bounded;
  }
  const float magnitude = std::fabs(bounded);
  if (magnitude <= deadzone) {
    return 0.0F;
  }
  const float normalized = (magnitude - deadzone) / (1.0F - deadzone);
  const float curved =
      (1.0F - expo) * normalized +
      expo * normalized * normalized * normalized;
  return std::copysign(curved * scale, bounded);
}

bool reject(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

}  // namespace

JoyMapper::JoyMapper(JoyMappingConfig config) : config_(config) {
  if (config_.steering_axis < 0 || config_.brake_axis < 0 ||
      config_.accel_axis < 0) {
    throw std::invalid_argument("joystick axis indexes must be non-negative");
  }
  if (!std::isfinite(config_.steering_deadzone) ||
      config_.steering_deadzone < 0.0F ||
      config_.steering_deadzone >= 1.0F) {
    throw std::invalid_argument("steering deadzone must be in [0, 1)");
  }
  if (!std::isfinite(config_.steering_scale) ||
      config_.steering_scale <= 0.0F ||
      config_.steering_scale > 1.0F) {
    throw std::invalid_argument("steering scale must be in (0, 1]");
  }
  if (!std::isfinite(config_.steering_expo) ||
      config_.steering_expo < 0.0F ||
      config_.steering_expo > 1.0F) {
    throw std::invalid_argument("steering expo must be in [0, 1]");
  }
}

bool JoyMapper::map(const std::vector<float>& axes, ControlCommand* output,
                    std::string* error) const {
  if (output == nullptr) {
    return reject(error, "output command is null");
  }
  const int largest_axis =
      std::max(config_.steering_axis,
               std::max(config_.brake_axis, config_.accel_axis));
  if (axes.size() <= static_cast<std::size_t>(largest_axis)) {
    return reject(error, "joy message does not contain configured axes");
  }

  output->steering =
      steeringValue(axes[static_cast<std::size_t>(config_.steering_axis)],
                    config_.steering_inverted, config_.steering_deadzone,
                    config_.steering_scale, config_.steering_expo);
  output->brake =
      pedalValue(axes[static_cast<std::size_t>(config_.brake_axis)],
                 config_.brake_inverted);
  output->accel =
      pedalValue(axes[static_cast<std::size_t>(config_.accel_axis)],
                 config_.accel_inverted);
  output->gear = 4U;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace vehicle_control
