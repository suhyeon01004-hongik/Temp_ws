#include "vehicle_control/reset_command.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace vehicle_control {
namespace {

std::string formatSeconds(const double seconds, const int precision = 3) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << seconds;
  return stream.str();
}

void validateTiming(const double seconds, const char* name) {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and nonnegative");
  }
}

void appendHeldKey(std::vector<std::string>* command,
                   const std::string& key, const std::string& hold_seconds) {
  command->insert(command->end(),
                  {"keydown", "--clearmodifiers", key, "sleep", hold_seconds,
                   "keyup", "--clearmodifiers", key});
}

}  // namespace

std::vector<std::string> buildMoraiResetCommand(
    const MoraiResetOptions& options) {
  if (options.window_name.empty()) {
    throw std::invalid_argument("MORAI window name must not be empty");
  }
  if (options.reset_key.empty()) {
    throw std::invalid_argument("MORAI reset key must not be empty");
  }
  if (options.control_toggle_key.empty()) {
    throw std::invalid_argument(
        "MORAI control toggle key must not be empty");
  }
  validateTiming(options.focus_delay_seconds, "focus_delay_seconds");
  validateTiming(options.key_hold_seconds, "key_hold_seconds");
  validateTiming(
      options.reset_key_hold_seconds, "reset_key_hold_seconds");
  validateTiming(options.mode_settle_seconds, "mode_settle_seconds");
  validateTiming(
      options.builtin_settle_seconds, "builtin_settle_seconds");
  validateTiming(options.reset_settle_seconds, "reset_settle_seconds");

  const std::string key_hold =
      formatSeconds(options.key_hold_seconds);
  const std::string reset_key_hold =
      formatSeconds(options.reset_key_hold_seconds);
  const std::string mode_settle =
      formatSeconds(options.mode_settle_seconds, 4);
  const std::string builtin_settle =
      formatSeconds(options.builtin_settle_seconds, 4);
  std::vector<std::string> command{
      "xdotool",        "search", "--onlyvisible", "--name",
      options.window_name, "windowactivate", "--sync", "sleep",
      formatSeconds(options.focus_delay_seconds)};

  // MORAI ignores I while AV-ExternalCtrl is active. Switch to
  // Manual-Keyboard, reset, then cycle back through Built-In to ExternalCtrl.
  appendHeldKey(&command, options.control_toggle_key, key_hold);
  command.insert(command.end(), {"sleep", mode_settle});
  appendHeldKey(&command, options.reset_key, reset_key_hold);
  command.insert(command.end(),
                 {"sleep", formatSeconds(options.reset_settle_seconds)});
  appendHeldKey(&command, options.control_toggle_key, key_hold);
  command.insert(command.end(), {"sleep", builtin_settle});
  appendHeldKey(&command, options.control_toggle_key, key_hold);
  return command;
}

ProcessResult executeCommand(const std::vector<std::string>& command) {
  ProcessResult result;
  if (command.empty() || command.front().empty()) {
    result.error = "command must contain an executable";
    return result;
  }

  int error_pipe[2];
  if (pipe(error_pipe) != 0) {
    result.error = std::string("pipe failed: ") + std::strerror(errno);
    return result;
  }

  if (fcntl(error_pipe[1], F_SETFD, FD_CLOEXEC) == -1) {
    const int saved_errno = errno;
    close(error_pipe[0]);
    close(error_pipe[1]);
    result.error =
        std::string("fcntl failed: ") + std::strerror(saved_errno);
    return result;
  }

  const pid_t child = fork();
  if (child == -1) {
    const int saved_errno = errno;
    close(error_pipe[0]);
    close(error_pipe[1]);
    result.error = std::string("fork failed: ") + std::strerror(saved_errno);
    return result;
  }

  if (child == 0) {
    close(error_pipe[0]);

    std::vector<char*> arguments;
    arguments.reserve(command.size() + 1);
    for (const std::string& argument : command) {
      arguments.push_back(const_cast<char*>(argument.c_str()));
    }
    arguments.push_back(nullptr);

    execvp(arguments.front(), arguments.data());

    const int exec_errno = errno;
    const ssize_t ignored =
        write(error_pipe[1], &exec_errno, sizeof(exec_errno));
    static_cast<void>(ignored);
    _exit(127);
  }

  close(error_pipe[1]);

  int exec_errno = 0;
  ssize_t received;
  do {
    received = read(error_pipe[0], &exec_errno, sizeof(exec_errno));
  } while (received == -1 && errno == EINTR);
  close(error_pipe[0]);

  int status = 0;
  pid_t waited;
  do {
    waited = waitpid(child, &status, 0);
  } while (waited == -1 && errno == EINTR);

  if (waited == -1) {
    result.error = std::string("waitpid failed: ") + std::strerror(errno);
    return result;
  }

  if (received == static_cast<ssize_t>(sizeof(exec_errno))) {
    result.exit_code = 127;
    result.error =
        std::string("could not start ") + command.front() + ": " +
        std::strerror(exec_errno);
    return result;
  }
  if (received == -1) {
    result.error = std::string("failed to read child status: ") +
                   std::strerror(errno);
    return result;
  }

  result.started = true;
  if (WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.exit_code = 128 + WTERMSIG(status);
    result.error = "command terminated by signal";
  } else {
    result.error = "command did not exit normally";
  }
  return result;
}

}  // namespace vehicle_control
