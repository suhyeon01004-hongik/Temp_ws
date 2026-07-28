#include "vehicle_control/button_edge.hpp"

#include <cstddef>
#include <stdexcept>

namespace vehicle_control {

ButtonEdge::ButtonEdge(int button_index) : button_index_(button_index) {
  if (button_index_ < 0) {
    throw std::invalid_argument("button index must be non-negative");
  }
}

ButtonEdgeResult ButtonEdge::update(
    const std::vector<std::int32_t>& buttons) {
  if (buttons.size() <= static_cast<std::size_t>(button_index_)) {
    previous_pressed_ = false;
    return ButtonEdgeResult::kInvalidButtonMessage;
  }

  const bool pressed =
      buttons[static_cast<std::size_t>(button_index_)] != 0;
  const bool rising_edge = pressed && !previous_pressed_;
  previous_pressed_ = pressed;
  return rising_edge ? ButtonEdgeResult::kRisingEdge
                     : ButtonEdgeResult::kNoEdge;
}

}  // namespace vehicle_control
