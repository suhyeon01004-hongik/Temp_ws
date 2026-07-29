#include "vehicle_control/morai_vehicle_status_packet.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace vehicle_control {
namespace {

void setError(const char* message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

std::uint32_t readUint32(const std::uint8_t* bytes) {
  return static_cast<std::uint32_t>(bytes[0U]) |
         (static_cast<std::uint32_t>(bytes[1U]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2U]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3U]) << 24U);
}

float readFloat(const std::uint8_t* bytes) {
  const std::uint32_t bits = readUint32(bytes);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

bool decodeMoraiVehicleStatus(const std::uint8_t* packet,
                              std::size_t packet_size,
                              MoraiVehicleStatus* status,
                              std::string* error) {
  if (packet == nullptr || status == nullptr) {
    setError("packet and status must not be null", error);
    return false;
  }

  constexpr char kHeader[] = "#MoraiInfo$";
  constexpr std::size_t kHeaderSize = 11U;
  constexpr std::size_t kDataLengthOffset = 11U;
  constexpr std::size_t kPayloadOffset = 27U;
  constexpr std::size_t kTailSize = 2U;
  if (packet_size < kPayloadOffset + kTailSize ||
      !std::equal(kHeader, kHeader + kHeaderSize, packet)) {
    setError("invalid MORAI vehicle status header", error);
    return false;
  }

  const std::uint32_t data_length =
      readUint32(packet + kDataLengthOffset);
  std::size_t control_mode_offset = 0U;
  std::size_t gear_offset = 0U;
  std::size_t speed_offset = 0U;
  if (data_length == 132U) {
    control_mode_offset = 27U;
    gear_offset = 28U;
    speed_offset = 29U;
  } else if (data_length >= 216U) {
    control_mode_offset = 35U;
    gear_offset = 36U;
    speed_offset = 37U;
  } else {
    setError("unsupported MORAI vehicle status layout", error);
    return false;
  }

  const std::size_t expected_size =
      kPayloadOffset + static_cast<std::size_t>(data_length) + kTailSize;
  if (packet_size < expected_size || speed_offset + sizeof(float) > packet_size) {
    setError("truncated MORAI vehicle status packet", error);
    return false;
  }
  if (packet[expected_size - 2U] != '\r' ||
      packet[expected_size - 1U] != '\n') {
    setError("invalid MORAI vehicle status tail", error);
    return false;
  }

  const float signed_speed_kph = readFloat(packet + speed_offset);
  if (!std::isfinite(signed_speed_kph)) {
    setError("MORAI vehicle speed is not finite", error);
    return false;
  }

  status->control_mode =
      static_cast<std::int8_t>(packet[control_mode_offset]);
  status->gear = static_cast<std::int8_t>(packet[gear_offset]);
  status->signed_speed_kph = signed_speed_kph;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace vehicle_control
