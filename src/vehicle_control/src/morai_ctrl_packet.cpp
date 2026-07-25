#include "vehicle_control/morai_ctrl_packet.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace vehicle_control {
namespace {

float boundedControl(float value, float minimum, float maximum) {
  if (!std::isfinite(value)) {
    return 0.0F;
  }
  return std::max(minimum, std::min(value, maximum));
}

void writeUint32(std::uint32_t value, std::size_t offset,
                 MoraiCtrlPacket* packet) {
  (*packet)[offset] = static_cast<std::uint8_t>(value & 0xffU);
  (*packet)[offset + 1U] =
      static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  (*packet)[offset + 2U] =
      static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  (*packet)[offset + 3U] =
      static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

void writeFloat(float value, std::size_t offset, MoraiCtrlPacket* packet) {
  static_assert(sizeof(float) == sizeof(std::uint32_t),
                "MORAI protocol requires 32-bit floats");
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeUint32(bits, offset, packet);
}

}  // namespace

MoraiCtrlPacket encodeMoraiCtrlPacket(const ControlCommand& command) {
  MoraiCtrlPacket packet{};
  constexpr char kHeader[] = "#MoraiCtrlCmd$";
  static_assert(sizeof(kHeader) - 1U == 14U,
                "MORAI control header must be 14 bytes");
  std::copy(kHeader, kHeader + 14U, packet.begin());

  writeUint32(23U, 14U, &packet);
  packet[30U] = 2U;
  packet[31U] =
      command.gear >= 1U && command.gear <= 5U ? command.gear : 4U;
  packet[32U] = 1U;
  writeFloat(0.0F, 33U, &packet);
  writeFloat(0.0F, 37U, &packet);
  writeFloat(boundedControl(command.accel, 0.0F, 1.0F), 41U, &packet);
  writeFloat(boundedControl(command.brake, 0.0F, 1.0F), 45U, &packet);
  writeFloat(boundedControl(command.steering, -1.0F, 1.0F), 49U,
             &packet);
  packet[53U] = '\r';
  packet[54U] = '\n';
  return packet;
}

}  // namespace vehicle_control
