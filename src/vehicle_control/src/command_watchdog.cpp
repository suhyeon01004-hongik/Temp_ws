#include "vehicle_control/command_watchdog.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vehicle_control {

CommandWatchdog::CommandWatchdog(double timeout_seconds, float safe_brake)
    : timeout_seconds_(timeout_seconds), safe_brake_(safe_brake) {
  if (!std::isfinite(timeout_seconds_) || timeout_seconds_ <= 0.0) {
    throw std::invalid_argument("command timeout must be positive");
  }
  if (!std::isfinite(safe_brake_)) {
    safe_brake_ = 0.0F;
  }
  safe_brake_ = std::max(0.0F, std::min(safe_brake_, 1.0F));
}

ControlCommand CommandWatchdog::select(const ControlCommand& latest,
                                       bool has_command,
                                       double age_seconds) const {
  const bool stale =
      !has_command || !std::isfinite(age_seconds) || age_seconds < 0.0 ||
      age_seconds > timeout_seconds_;
  if (stale) {
    const std::uint8_t safe_gear = has_command ? latest.gear : 4U;
    return ControlCommand(0.0F, safe_brake_, 0.0F, safe_gear);
  }
  return latest;
}

}  // namespace vehicle_control
