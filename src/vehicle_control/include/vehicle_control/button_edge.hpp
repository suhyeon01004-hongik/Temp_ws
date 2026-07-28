#pragma once

#include <cstdint>
#include <vector>

namespace vehicle_control {

enum class ButtonEdgeResult {
  kNoEdge,
  kRisingEdge,
  kInvalidButtonMessage,
};

class ButtonEdge {
 public:
  explicit ButtonEdge(int button_index);

  ButtonEdgeResult update(const std::vector<std::int32_t>& buttons);

 private:
  int button_index_{0};
  bool previous_pressed_{false};
};

}  // namespace vehicle_control
