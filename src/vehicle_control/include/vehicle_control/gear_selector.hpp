#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace vehicle_control {

enum class GearSelectionStatus {
  kNoRequest,
  kChanged,
  kSpeedUnavailable,
  kTooFast,
  kAmbiguousButtons,
  kInvalidButtonMessage,
};

struct GearSelectorConfig {
  int drive_button{0};
  int neutral_button{1};
  int reverse_button{2};
  int park_button{3};
  std::uint8_t initial_gear{4U};
  double maximum_change_speed_mps{0.5};
};

struct GearSelectionResult {
  std::uint8_t gear;
  GearSelectionStatus status;
};

class GearSelector {
 public:
  explicit GearSelector(const GearSelectorConfig& config);

  GearSelectionResult update(const std::vector<std::int32_t>& buttons,
                             bool speed_valid, double speed_mps);
  std::uint8_t gear() const;

 private:
  std::array<int, 4U> buttons_;
  std::array<bool, 4U> previous_pressed_{{false, false, false, false}};
  std::uint8_t gear_;
  double maximum_change_speed_mps_;
};

}  // namespace vehicle_control
