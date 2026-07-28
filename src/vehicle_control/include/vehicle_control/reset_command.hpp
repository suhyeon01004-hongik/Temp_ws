#pragma once

#include <string>
#include <vector>

namespace vehicle_control {

struct ProcessResult {
  bool started = false;
  int exit_code = -1;
  std::string error;
};

std::vector<std::string> buildMoraiResetCommand(
    const std::string& window_name, const std::string& reset_key);

ProcessResult executeCommand(const std::vector<std::string>& command);

}  // namespace vehicle_control
