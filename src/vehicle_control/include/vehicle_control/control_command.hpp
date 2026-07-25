#pragma once

#include <cstdint>

namespace vehicle_control {

struct ControlCommand {
  ControlCommand(float accel_value = 0.0F, float brake_value = 0.0F,
                 float steering_value = 0.0F,
                 std::uint8_t gear_value = 4U)
      : accel(accel_value),
        brake(brake_value),
        steering(steering_value),
        gear(gear_value) {}

  float accel;
  float brake;
  float steering;
  std::uint8_t gear;
};

}  // namespace vehicle_control
