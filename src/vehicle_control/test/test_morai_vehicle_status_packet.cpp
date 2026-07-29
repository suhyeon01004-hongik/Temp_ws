#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "vehicle_control/morai_vehicle_status_packet.hpp"

namespace vehicle_control {
namespace {

void writeInt32(std::int32_t value, std::size_t offset,
                std::vector<std::uint8_t>* packet) {
  const std::uint32_t bits = static_cast<std::uint32_t>(value);
  (*packet)[offset] = static_cast<std::uint8_t>(bits & 0xffU);
  (*packet)[offset + 1U] =
      static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  (*packet)[offset + 2U] =
      static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  (*packet)[offset + 3U] =
      static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

void writeFloat(float value, std::size_t offset,
                std::vector<std::uint8_t>* packet) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeInt32(static_cast<std::int32_t>(bits), offset, packet);
}

std::vector<std::uint8_t> makePacket(std::int32_t data_length,
                                     std::size_t control_mode_offset,
                                     std::size_t gear_offset,
                                     std::size_t speed_offset,
                                     float signed_speed_kph) {
  constexpr std::size_t kEnvelopeSize = 11U + 4U + 12U + 2U;
  std::vector<std::uint8_t> packet(
      kEnvelopeSize + static_cast<std::size_t>(data_length), 0U);
  constexpr char kHeader[] = "#MoraiInfo$";
  std::copy(kHeader, kHeader + 11U, packet.begin());
  writeInt32(data_length, 11U, &packet);
  packet[control_mode_offset] = 2U;
  packet[gear_offset] = 4U;
  writeFloat(signed_speed_kph, speed_offset, &packet);
  packet[packet.size() - 2U] = '\r';
  packet[packet.size() - 1U] = '\n';
  return packet;
}

TEST(MoraiVehicleStatusPacket, DecodesTimestampedPacket) {
  const std::vector<std::uint8_t> packet =
      makePacket(216, 35U, 36U, 37U, -12.5F);
  MoraiVehicleStatus status;
  std::string error;

  ASSERT_TRUE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, &error))
      << error;
  EXPECT_EQ(status.control_mode, 2);
  EXPECT_EQ(status.gear, 4);
  EXPECT_FLOAT_EQ(status.signed_speed_kph, -12.5F);
}

TEST(MoraiVehicleStatusPacket, DecodesTimestampedPacketWithAppendedFields) {
  const std::vector<std::uint8_t> packet =
      makePacket(224, 35U, 36U, 37U, 7.25F);
  MoraiVehicleStatus status;
  std::string error;

  ASSERT_TRUE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, &error))
      << error;
  EXPECT_EQ(status.control_mode, 2);
  EXPECT_EQ(status.gear, 4);
  EXPECT_FLOAT_EQ(status.signed_speed_kph, 7.25F);
}

TEST(MoraiVehicleStatusPacket, DecodesLegacyPacket) {
  const std::vector<std::uint8_t> packet =
      makePacket(132, 27U, 28U, 29U, 3.25F);
  MoraiVehicleStatus status;

  ASSERT_TRUE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, nullptr));
  EXPECT_EQ(status.control_mode, 2);
  EXPECT_EQ(status.gear, 4);
  EXPECT_FLOAT_EQ(status.signed_speed_kph, 3.25F);
}

TEST(MoraiVehicleStatusPacket, RejectsWrongHeader) {
  std::vector<std::uint8_t> packet =
      makePacket(216, 35U, 36U, 37U, 0.0F);
  packet[1U] = 'X';
  MoraiVehicleStatus status;

  EXPECT_FALSE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, nullptr));
}

TEST(MoraiVehicleStatusPacket, RejectsTruncatedPacket) {
  std::vector<std::uint8_t> packet =
      makePacket(216, 35U, 36U, 37U, 0.0F);
  packet.resize(40U);
  MoraiVehicleStatus status;

  EXPECT_FALSE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, nullptr));
}

TEST(MoraiVehicleStatusPacket, RejectsUnsupportedLayout) {
  const std::vector<std::uint8_t> packet =
      makePacket(200, 35U, 36U, 37U, 0.0F);
  MoraiVehicleStatus status;

  EXPECT_FALSE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, nullptr));
}

TEST(MoraiVehicleStatusPacket, RejectsNullOutput) {
  const std::vector<std::uint8_t> packet =
      makePacket(216, 35U, 36U, 37U, 0.0F);

  EXPECT_FALSE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), nullptr, nullptr));
}

TEST(MoraiVehicleStatusPacket, RejectsNonFiniteSpeed) {
  const std::vector<std::uint8_t> packet = makePacket(
      216, 35U, 36U, 37U, std::numeric_limits<float>::quiet_NaN());
  MoraiVehicleStatus status;

  EXPECT_FALSE(
      decodeMoraiVehicleStatus(packet.data(), packet.size(), &status, nullptr));
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
