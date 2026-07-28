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
  double key_hold_seconds{0.12};
  double mode_settle_seconds{0.25};
  double reset_settle_seconds{1.5};
};

std::vector<std::string> buildMoraiResetCommand(
    const MoraiResetOptions& options);

ProcessResult executeCommand(const std::vector<std::string>& command);

}  // namespace vehicle_control
