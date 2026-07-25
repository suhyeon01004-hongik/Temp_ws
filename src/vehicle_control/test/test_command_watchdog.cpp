#include <gtest/gtest.h>

#include <stdexcept>

#include "vehicle_control/command_watchdog.hpp"

namespace vehicle_control {
namespace {

TEST(CommandWatchdog, FreshCommandPassesThrough) {
  const CommandWatchdog watchdog(0.25, 0.5F);
  const ControlCommand input(0.7F, 0.0F, -0.2F, 4U);

  const ControlCommand actual = watchdog.select(input, true, 0.1);

  EXPECT_FLOAT_EQ(0.7F, actual.accel);
  EXPECT_FLOAT_EQ(0.0F, actual.brake);
  EXPECT_FLOAT_EQ(-0.2F, actual.steering);
  EXPECT_EQ(4U, actual.gear);
}

TEST(CommandWatchdog, StaleCommandUsesConfiguredSafeBrake) {
  const CommandWatchdog watchdog(0.25, 0.5F);

  const ControlCommand actual =
      watchdog.select(ControlCommand(1.0F, 0.0F, 0.8F, 4U), true, 0.3);

  EXPECT_FLOAT_EQ(0.0F, actual.accel);
  EXPECT_FLOAT_EQ(0.5F, actual.brake);
  EXPECT_FLOAT_EQ(0.0F, actual.steering);
  EXPECT_EQ(4U, actual.gear);
}

TEST(CommandWatchdog, MissingCommandUsesSafeCommand) {
  const CommandWatchdog watchdog(0.25, 0.4F);

  const ControlCommand actual =
      watchdog.select(ControlCommand(1.0F, 0.0F, 0.8F, 2U), false, 0.0);

  EXPECT_FLOAT_EQ(0.0F, actual.accel);
  EXPECT_FLOAT_EQ(0.4F, actual.brake);
  EXPECT_FLOAT_EQ(0.0F, actual.steering);
  EXPECT_EQ(4U, actual.gear);
}

TEST(CommandWatchdog, NegativeAgeUsesSafeCommand) {
  const CommandWatchdog watchdog(0.25, 0.4F);

  const ControlCommand actual =
      watchdog.select(ControlCommand(1.0F, 0.0F, 0.8F, 4U), true, -0.1);

  EXPECT_FLOAT_EQ(0.0F, actual.accel);
  EXPECT_FLOAT_EQ(0.4F, actual.brake);
}

TEST(CommandWatchdog, SafeBrakeIsClamped) {
  const CommandWatchdog watchdog(0.25, 2.0F);

  const ControlCommand actual =
      watchdog.select(ControlCommand(), false, 0.0);

  EXPECT_FLOAT_EQ(1.0F, actual.brake);
}

TEST(CommandWatchdog, NonPositiveTimeoutIsRejected) {
  EXPECT_THROW(
      { const CommandWatchdog watchdog(0.0, 0.5F); },
      std::invalid_argument);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
