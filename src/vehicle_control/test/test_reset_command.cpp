#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "vehicle_control/reset_command.hpp"

namespace vehicle_control {
namespace {

TEST(ResetCommandTest, BuildsXdotoolCommandWithoutUsingAShell) {
  const std::vector<std::string> command =
      buildMoraiResetCommand("Simulator", "i");

  const std::vector<std::string> expected{
      "xdotool",       "search", "--onlyvisible", "--name", "Simulator",
      "windowactivate", "--sync", "key",           "--clearmodifiers", "i"};
  EXPECT_EQ(command, expected);
}

TEST(ResetCommandTest, RejectsEmptyWindowName) {
  EXPECT_THROW(buildMoraiResetCommand("", "i"), std::invalid_argument);
}

TEST(ResetCommandTest, RejectsEmptyResetKey) {
  EXPECT_THROW(buildMoraiResetCommand("Simulator", ""), std::invalid_argument);
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
