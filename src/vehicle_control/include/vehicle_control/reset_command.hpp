#pragma once

#include <string>
#include <vector>

namespace vehicle_control {

struct ProcessResult {
  bool started = false;
  int exit_code = -1;
  std::string error;
};

struct MoraiResetOptions {
  std::string window_name{"Simulator"};
  std::string reset_key{"i"};
  std::string control_toggle_key{"q"};
  double focus_delay_seconds{0.2};
  double key_hold_seconds{0.01};
  double reset_key_hold_seconds{0.12};
  double mode_settle_seconds{0.25};
  double builtin_settle_seconds{0.0001};
  double reset_settle_seconds{0.1};
};

std::vector<std::string> buildMoraiResetCommand(
    const MoraiResetOptions& options);

ProcessResult executeCommand(const std::vector<std::string>& command);

}  // namespace vehicle_control
