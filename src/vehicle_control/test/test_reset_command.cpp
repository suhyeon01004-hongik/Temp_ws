#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "vehicle_control/reset_command.hpp"

namespace vehicle_control {
namespace {

TEST(ResetCommandTest, BuildsXdotoolCommandWithoutUsingAShell) {
  MoraiResetOptions options;
  const std::vector<std::string> command = buildMoraiResetCommand(options);

  const std::vector<std::string> expected{
      "xdotool",        "search",         "--onlyvisible", "--name",
      "Simulator",      "windowactivate", "--sync",        "sleep",
      "0.200",          "keydown",        "--clearmodifiers", "q",
      "sleep",          "0.010",          "keyup",         "--clearmodifiers",
      "q",              "sleep",          "0.2500",        "keydown",
      "--clearmodifiers", "i",            "sleep",         "0.120",
      "keyup",          "--clearmodifiers", "i",           "sleep",
      "0.100",          "keydown",        "--clearmodifiers", "q",
      "sleep",          "0.010",          "keyup",         "--clearmodifiers",
      "q",              "sleep",          "0.0001",        "keydown",
      "--clearmodifiers", "q",            "sleep",         "0.010",
      "keyup",          "--clearmodifiers", "q"};
  EXPECT_EQ(command, expected);
}

TEST(ResetCommandTest, RejectsEmptyWindowName) {
  MoraiResetOptions options;
  options.window_name = "";
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}

TEST(ResetCommandTest, RejectsEmptyResetKey) {
  MoraiResetOptions options;
  options.reset_key = "";
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}

TEST(ResetCommandTest, RejectsEmptyControlToggleKey) {
  MoraiResetOptions options;
  options.control_toggle_key = "";
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}

TEST(ResetCommandTest, RejectsNegativeTiming) {
  MoraiResetOptions options;
  options.reset_settle_seconds = -0.1;
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}

TEST(ResetCommandTest, RejectsNegativeBuiltinSettle) {
  MoraiResetOptions options;
  options.builtin_settle_seconds = -0.1;
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}

TEST(ResetCommandTest, RejectsNegativeResetKeyHold) {
  MoraiResetOptions options;
  options.reset_key_hold_seconds = -0.1;
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}

TEST(ResetCommandTest, ReportsSuccessfulProcessExit) {
  const ProcessResult result = executeCommand({"/bin/true"});

  EXPECT_TRUE(result.started);
  EXPECT_EQ(result.exit_code, 0);
  EXPECT_TRUE(result.error.empty());
}

TEST(ResetCommandTest, ReportsNonzeroProcessExit) {
  const ProcessResult result = executeCommand({"/bin/false"});

  EXPECT_TRUE(result.started);
  EXPECT_NE(result.exit_code, 0);
}

TEST(ResetCommandTest, ReportsMissingExecutable) {
  const ProcessResult result =
      executeCommand({"/definitely/not/a/real/executable"});

  EXPECT_FALSE(result.started);
  EXPECT_NE(result.exit_code, 0);
  EXPECT_FALSE(result.error.empty());
}

TEST(ResetCommandTest, RejectsEmptyCommand) {
  const ProcessResult result = executeCommand({});

  EXPECT_FALSE(result.started);
  EXPECT_NE(result.exit_code, 0);
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
