#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace vehicle_control {

struct MoraiVehicleStatus {
  std::int8_t control_mode{0};
  std::int8_t gear{0};
  float signed_speed_kph{0.0F};
};

bool decodeMoraiVehicleStatus(const std::uint8_t* packet,
                              std::size_t packet_size,
                              MoraiVehicleStatus* status,
                              std::string* error);

}  // namespace vehicle_control
