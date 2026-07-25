#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "vehicle_control/joy_mapper.hpp"

namespace vehicle_control {
namespace {

TEST(JoyMapper, ReleasedTriggersProduceCoastingCommand) {
  const JoyMapper mapper(JoyMappingConfig{});
  const std::vector<float> axes{0.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F};
  ControlCommand output;

  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(0.0F, output.accel);
  EXPECT_FLOAT_EQ(0.0F, output.brake);
  EXPECT_FLOAT_EQ(0.0F, output.steering);
  EXPECT_EQ(4U, output.gear);
}

TEST(JoyMapper, FullyPressedTriggersProduceFullPedalCommands) {
  const JoyMapper mapper(JoyMappingConfig{});
  const std::vector<float> axes{0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F};
  ControlCommand output;

  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(1.0F, output.accel);
  EXPECT_FLOAT_EQ(1.0F, output.brake);
}

TEST(JoyMapper, SteeringDeadzoneIsRemovedAndRangeIsRescaled) {
  const JoyMapper mapper(JoyMappingConfig{});
  const std::vector<float> axes{0.525F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F};
  ControlCommand output;

  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_NEAR(0.5F, output.steering, 1.0e-6F);
}

TEST(JoyMapper, SteeringInsideDeadzoneIsCentered) {
  const JoyMapper mapper(JoyMappingConfig{});
  const std::vector<float> axes{0.04F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F};
  ControlCommand output;

  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(0.0F, output.steering);
}

TEST(JoyMapper, ConfiguredAxesCanBeInverted) {
  JoyMappingConfig config;
  config.steering_inverted = true;
  config.brake_inverted = true;
  config.accel_inverted = true;
  const JoyMapper mapper(config);
  const std::vector<float> axes{1.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F};
  ControlCommand output;

  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(-1.0F, output.steering);
  EXPECT_FLOAT_EQ(1.0F, output.accel);
  EXPECT_FLOAT_EQ(1.0F, output.brake);
}

TEST(JoyMapper, NonFiniteAxesProduceNeutralOutputs) {
  const JoyMapper mapper(JoyMappingConfig{});
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> axes{nan, 0.0F, nan, 0.0F, 0.0F, nan};
  ControlCommand output;

  ASSERT_TRUE(mapper.map(axes, &output, nullptr));
  EXPECT_FLOAT_EQ(0.0F, output.steering);
  EXPECT_FLOAT_EQ(0.0F, output.accel);
  EXPECT_FLOAT_EQ(0.0F, output.brake);
}

TEST(JoyMapper, MissingConfiguredAxisRejectsInput) {
  const JoyMapper mapper(JoyMappingConfig{});
  ControlCommand output;
  std::string error;

  EXPECT_FALSE(mapper.map(std::vector<float>{0.0F}, &output, &error));
  EXPECT_FALSE(error.empty());
}

TEST(JoyMapper, NullOutputRejectsInput) {
  const JoyMapper mapper(JoyMappingConfig{});
  std::string error;

  EXPECT_FALSE(mapper.map(
      std::vector<float>{0.0F, 0.0F, -1.0F, 0.0F, 0.0F, -1.0F},
      nullptr, &error));
  EXPECT_FALSE(error.empty());
}

TEST(JoyMapper, NegativeAxisIndexIsRejected) {
  JoyMappingConfig config;
  config.accel_axis = -1;

  EXPECT_THROW({ const JoyMapper mapper(config); }, std::invalid_argument);
}

TEST(JoyMapper, InvalidSteeringDeadzoneIsRejected) {
  JoyMappingConfig config;
  config.steering_deadzone = 1.0F;

  EXPECT_THROW({ const JoyMapper mapper(config); }, std::invalid_argument);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
