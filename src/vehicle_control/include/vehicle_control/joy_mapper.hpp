#pragma once

#include <string>
#include <vector>

#include "vehicle_control/control_command.hpp"

namespace vehicle_control {

struct JoyMappingConfig {
  int steering_axis{0};
  int brake_axis{2};
  int accel_axis{5};
  bool steering_inverted{false};
  bool brake_inverted{false};
  bool accel_inverted{false};
  float steering_deadzone{0.05F};
};

class JoyMapper {
 public:
  explicit JoyMapper(JoyMappingConfig config);

  bool map(const std::vector<float>& axes, ControlCommand* output,
           std::string* error) const;

 private:
  JoyMappingConfig config_;
};

}  // namespace vehicle_control
