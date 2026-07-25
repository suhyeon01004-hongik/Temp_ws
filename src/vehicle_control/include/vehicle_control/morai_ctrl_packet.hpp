#pragma once

#include <array>
#include <cstdint>

#include "vehicle_control/control_command.hpp"

namespace vehicle_control {

using MoraiCtrlPacket = std::array<std::uint8_t, 55U>;

MoraiCtrlPacket encodeMoraiCtrlPacket(const ControlCommand& command);

}  // namespace vehicle_control
