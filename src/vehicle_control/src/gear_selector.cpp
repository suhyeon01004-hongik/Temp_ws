#include "vehicle_control/gear_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vehicle_control {
namespace {

void validateConfig(const GearSelectorConfig& config) {
  const std::array<int, 4U> buttons{{
      config.drive_button,
      config.neutral_button,
      config.reverse_button,
      config.park_button,
  }};
  if (std::any_of(buttons.begin(), buttons.end(),
                  [](int button) { return button < 0; })) {
    throw std::invalid_argument("gear button indexes must be non-negative");
  }

  std::array<int, 4U> sorted_buttons = buttons;
  std::sort(sorted_buttons.begin(), sorted_buttons.end());
  if (std::adjacent_find(sorted_buttons.begin(), sorted_buttons.end()) !=
      sorted_buttons.end()) {
    throw std::invalid_argument("gear button indexes must be unique");
  }
  if (config.initial_gear < 1U || config.initial_gear > 4U) {
    throw std::invalid_argument("initial gear must be Park, Reverse, Neutral, or Drive");
  }
  if (!std::isfinite(config.maximum_change_speed_mps) ||
      config.maximum_change_speed_mps < 0.0) {
    throw std::invalid_argument(
        "maximum gear change speed must be finite and non-negative");
  }
}

}  // namespace

GearSelector::GearSelector(const GearSelectorConfig& config)
    : buttons_{{
          config.drive_button,
          config.neutral_button,
          config.reverse_button,
          config.park_button,
      }},
      gear_(config.initial_gear),
      maximum_change_speed_mps_(config.maximum_change_speed_mps) {
  validateConfig(config);
}

GearSelectionResult GearSelector::update(
    const std::vector<std::int32_t>& buttons, bool speed_valid,
    double speed_mps) {
  const int largest_button =
      *std::max_element(buttons_.begin(), buttons_.end());
  if (buttons.size() <= static_cast<std::size_t>(largest_button)) {
    return GearSelectionResult{
        gear_, GearSelectionStatus::kInvalidButtonMessage};
  }

  std::array<bool, 4U> pressed{{false, false, false, false}};
  std::array<bool, 4U> rising{{false, false, false, false}};
  std::size_t pressed_count = 0U;
  std::size_t rising_count = 0U;
  std::size_t requested_index = 0U;
  for (std::size_t index = 0U; index < buttons_.size(); ++index) {
    pressed[index] =
        buttons[static_cast<std::size_t>(buttons_[index])] != 0;
    rising[index] = pressed[index] && !previous_pressed_[index];
    if (pressed[index]) {
      ++pressed_count;
    }
    if (rising[index]) {
      ++rising_count;
      requested_index = index;
    }
  }
  previous_pressed_ = pressed;

  if (rising_count == 0U) {
    return GearSelectionResult{gear_, GearSelectionStatus::kNoRequest};
  }
  if (pressed_count != 1U || rising_count != 1U) {
    return GearSelectionResult{
        gear_, GearSelectionStatus::kAmbiguousButtons};
  }
  if (!speed_valid || !std::isfinite(speed_mps)) {
    return GearSelectionResult{
        gear_, GearSelectionStatus::kSpeedUnavailable};
  }
  if (std::fabs(speed_mps) > maximum_change_speed_mps_) {
    return GearSelectionResult{gear_, GearSelectionStatus::kTooFast};
  }

  constexpr std::array<std::uint8_t, 4U> kGears{{4U, 3U, 2U, 1U}};
  gear_ = kGears[requested_index];
  return GearSelectionResult{gear_, GearSelectionStatus::kChanged};
}

std::uint8_t GearSelector::gear() const { return gear_; }

}  // namespace vehicle_control
