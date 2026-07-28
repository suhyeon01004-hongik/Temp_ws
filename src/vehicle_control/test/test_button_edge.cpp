#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "vehicle_control/button_edge.hpp"

namespace vehicle_control {
namespace {

TEST(ButtonEdge, ReportsOneRisingEdgeWhileButtonIsHeld) {
  ButtonEdge edge(8);
  std::vector<std::int32_t> buttons(11U, 0);
  buttons[8U] = 1;

  EXPECT_EQ(ButtonEdgeResult::kRisingEdge, edge.update(buttons));
  EXPECT_EQ(ButtonEdgeResult::kNoEdge, edge.update(buttons));
  EXPECT_EQ(ButtonEdgeResult::kNoEdge, edge.update(buttons));
}

TEST(ButtonEdge, ReportsAnotherEdgeAfterReleaseAndRepress) {
  ButtonEdge edge(8);
  std::vector<std::int32_t> buttons(11U, 0);
  buttons[8U] = 1;
  ASSERT_EQ(ButtonEdgeResult::kRisingEdge, edge.update(buttons));

  buttons[8U] = 0;
  EXPECT_EQ(ButtonEdgeResult::kNoEdge, edge.update(buttons));
  buttons[8U] = 1;
  EXPECT_EQ(ButtonEdgeResult::kRisingEdge, edge.update(buttons));
}

TEST(ButtonEdge, ReportsShortButtonArray) {
  ButtonEdge edge(8);
  const std::vector<std::int32_t> buttons(8U, 0);

  EXPECT_EQ(ButtonEdgeResult::kInvalidButtonMessage, edge.update(buttons));
}

TEST(ButtonEdge, RejectsNegativeButtonIndex) {
  EXPECT_THROW({ const ButtonEdge edge(-1); }, std::invalid_argument);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
