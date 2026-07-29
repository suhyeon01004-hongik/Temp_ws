#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "morai_path_tracking/pid_controller.hpp"

namespace morai_path_tracking {
namespace {

TEST(LongitudinalPid, ProducesAccelBelowTarget) {
  LongitudinalPid pid(PidConfig{});
  const auto command = pid.update(3.0, 0.0, 0.1);
  EXPECT_GT(command.accel, 0.0);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, ProducesBrakeAboveTarget) {
  LongitudinalPid pid(PidConfig{});
  const auto command = pid.update(3.0, 4.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_GT(command.brake, 0.0);
}

TEST(LongitudinalPid, ClampsAccelAndBrakeOutputs) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.maximum_accel = 0.4;
  config.maximum_brake = 0.6;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.4, pid.update(2.0, 0.0, 0.1).accel);
  EXPECT_DOUBLE_EQ(0.6, pid.update(0.0, 2.0, 0.1).brake);
}

TEST(LongitudinalPid, KeepsAccelAndBrakeMutuallyExclusive) {
  LongitudinalPid pid(PidConfig{});
  const auto accelerating = pid.update(3.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, accelerating.brake);
  const auto braking = pid.update(0.0, 3.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, braking.accel);
}

TEST(LongitudinalPid, AppliesDeadband) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 0.0;
  config.kd = 0.0;
  config.error_deadband_mps = 0.05;
  LongitudinalPid pid(config);
  const auto command = pid.update(1.04, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, UsesDerivativeOfMeasurement) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 0.0;
  config.kd = 1.0;
  config.maximum_brake = 20.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.0, pid.update(3.0, 0.0, 0.1).brake);
  EXPECT_DOUBLE_EQ(10.0, pid.update(3.0, 1.0, 0.1).brake);
}

TEST(LongitudinalPid, ClampsIntegralState) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_limit = 0.2;
  config.maximum_accel = 2.0;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.2, pid.update(1.0, 0.0, 1.0).accel);
  EXPECT_DOUBLE_EQ(0.2, pid.update(1.0, 0.0, 1.0).accel);
}

TEST(LongitudinalPid, PreventsIntegralWindupIntoSaturation) {
  PidConfig config;
  config.kp = 1.0;
  config.ki = 1.0;
  config.kd = 0.0;
  config.integral_limit = 10.0;
  config.maximum_accel = 0.5;
  LongitudinalPid pid(config);
  EXPECT_DOUBLE_EQ(0.5, pid.update(1.0, 0.0, 1.0).accel);
  EXPECT_DOUBLE_EQ(0.0, pid.update(0.0, 0.0, 1.0).accel);
}

TEST(LongitudinalPid, ResetClearsIntegralAndDerivativeHistory) {
  PidConfig config;
  config.kp = 0.0;
  config.ki = 1.0;
  config.kd = 1.0;
  config.maximum_accel = 10.0;
  LongitudinalPid pid(config);
  ASSERT_GT(pid.update(1.0, 0.0, 1.0).accel, 0.0);
  pid.reset();
  const auto command = pid.update(0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, RejectsInvalidUpdateInputs) {
  LongitudinalPid pid(PidConfig{});
  EXPECT_THROW(pid.update(1.0, 0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(pid.update(1.0, 0.0, -0.1), std::invalid_argument);
  EXPECT_THROW(pid.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1),
               std::invalid_argument);
  EXPECT_THROW(pid.update(1.0, std::numeric_limits<double>::infinity(), 0.1),
               std::invalid_argument);
}

TEST(LongitudinalPid, RejectsInvalidConfiguration) {
  PidConfig config;
  config.kp = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.ki = std::numeric_limits<double>::infinity();
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.integral_limit = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);

  config = PidConfig{};
  config.maximum_brake = -0.1;
  EXPECT_THROW({ LongitudinalPid pid(config); }, std::invalid_argument);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
