#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

#include "vehicle_control/morai_ctrl_packet.hpp"

namespace vehicle_control {
namespace {

TEST(MoraiCtrlPacket, EncodesMolitCompetitionLayoutExactly) {
  const MoraiCtrlPacket actual =
      encodeMoraiCtrlPacket(ControlCommand(0.5F, 0.25F, -1.0F, 4U));
  const std::array<std::uint8_t, 55U> expected{{
      0x23U, 0x4dU, 0x6fU, 0x72U, 0x61U, 0x69U, 0x43U, 0x74U,
      0x72U, 0x6cU, 0x43U, 0x6dU, 0x64U, 0x24U,
      0x17U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x02U, 0x04U, 0x01U,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x3fU,
      0x00U, 0x00U, 0x80U, 0x3eU,
      0x00U, 0x00U, 0x80U, 0xbfU,
      0x0dU, 0x0aU,
  }};

  EXPECT_EQ(55U, actual.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual.begin()));
}

TEST(MoraiCtrlPacket, ClampsPedalAndSteeringRanges) {
  const MoraiCtrlPacket actual =
      encodeMoraiCtrlPacket(ControlCommand(2.0F, -1.0F, 3.0F, 4U));

  EXPECT_EQ(0x00U, actual[41]);
  EXPECT_EQ(0x00U, actual[42]);
  EXPECT_EQ(0x80U, actual[43]);
  EXPECT_EQ(0x3fU, actual[44]);
  EXPECT_EQ(0x00U, actual[45]);
  EXPECT_EQ(0x00U, actual[46]);
  EXPECT_EQ(0x00U, actual[47]);
  EXPECT_EQ(0x00U, actual[48]);
  EXPECT_EQ(0x00U, actual[49]);
  EXPECT_EQ(0x00U, actual[50]);
  EXPECT_EQ(0x80U, actual[51]);
  EXPECT_EQ(0x3fU, actual[52]);
}

TEST(MoraiCtrlPacket, NonFiniteControlsBecomeNeutral) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const MoraiCtrlPacket actual =
      encodeMoraiCtrlPacket(ControlCommand(nan, nan, nan, 4U));

  for (std::size_t index = 41U; index <= 52U; ++index) {
    EXPECT_EQ(0x00U, actual[index]);
  }
}

TEST(MoraiCtrlPacket, EncodesSelectedReverseGear) {
  const MoraiCtrlPacket actual =
      encodeMoraiCtrlPacket(ControlCommand(0.0F, 0.0F, 0.0F, 2U));

  EXPECT_EQ(0x02U, actual[31]);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
