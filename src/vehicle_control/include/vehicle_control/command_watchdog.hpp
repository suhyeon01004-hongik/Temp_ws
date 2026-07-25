#pragma once

#include "vehicle_control/control_command.hpp"

namespace vehicle_control {

class CommandWatchdog {
 public:
  CommandWatchdog(double timeout_seconds, float safe_brake);

  ControlCommand select(const ControlCommand& latest, bool has_command,
                        double age_seconds) const;

 private:
  double timeout_seconds_;
  float safe_brake_;
};

}  // namespace vehicle_control
