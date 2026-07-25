#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "vehicle_control/gear_selector.hpp"

namespace vehicle_control {
namespace {

std::vector<std::int32_t> releasedButtons() {
  return std::vector<std::int32_t>(11U, 0);
}

TEST(GearSelector, FaceButtonsSelectDriveNeutralReverseAndPark) {
  struct Case {
    int button;
    std::uint8_t expected_gear;
  };
  const Case cases[] = {
      {0, 4U},
      {1, 3U},
      {2, 2U},
      {3, 1U},
  };

  for (const Case& test_case : cases) {
    GearSelector selector(GearSelectorConfig{});
    std::vector<std::int32_t> buttons = releasedButtons();
    buttons[static_cast<std::size_t>(test_case.button)] = 1;

    const GearSelectionResult result =
        selector.update(buttons, true, 0.5);

    EXPECT_EQ(GearSelectionStatus::kChanged, result.status);
    EXPECT_EQ(test_case.expected_gear, result.gear);
    EXPECT_EQ(test_case.expected_gear, selector.gear());
  }
}

TEST(GearSelector, RejectsRequestAboveConfiguredSpeed) {
  GearSelector selector(GearSelectorConfig{});
  std::vector<std::int32_t> buttons = releasedButtons();
  buttons[2] = 1;

  const GearSelectionResult result = selector.update(buttons, true, 0.51);

  EXPECT_EQ(GearSelectionStatus::kTooFast, result.status);
  EXPECT_EQ(4U, result.gear);
}

TEST(GearSelector, HeldRejectedButtonDoesNotChangeAfterVehicleSlows) {
  GearSelector selector(GearSelectorConfig{});
  std::vector<std::int32_t> buttons = releasedButtons();
  buttons[2] = 1;

  EXPECT_EQ(GearSelectionStatus::kTooFast,
            selector.update(buttons, true, 1.0).status);
  const GearSelectionResult held_result =
      selector.update(buttons, true, 0.0);

  EXPECT_EQ(GearSelectionStatus::kNoRequest, held_result.status);
  EXPECT_EQ(4U, held_result.gear);
}

TEST(GearSelector, AcceptsNewPressAfterRejectedButtonIsReleased) {
  GearSelector selector(GearSelectorConfig{});
  std::vector<std::int32_t> buttons = releasedButtons();
  buttons[2] = 1;
  selector.update(buttons, true, 1.0);
  selector.update(releasedButtons(), true, 0.0);

  const GearSelectionResult result = selector.update(buttons, true, 0.0);

  EXPECT_EQ(GearSelectionStatus::kChanged, result.status);
  EXPECT_EQ(2U, result.gear);
}

TEST(GearSelector, RejectsMissingAndNonFiniteSpeed) {
  {
    GearSelector selector(GearSelectorConfig{});
    std::vector<std::int32_t> buttons = releasedButtons();
    buttons[2] = 1;
    EXPECT_EQ(GearSelectionStatus::kSpeedUnavailable,
              selector.update(buttons, false, 0.0).status);
  }
  {
    GearSelector selector(GearSelectorConfig{});
    std::vector<std::int32_t> buttons = releasedButtons();
    buttons[2] = 1;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(GearSelectionStatus::kSpeedUnavailable,
              selector.update(buttons, true, nan).status);
  }
}

TEST(GearSelector, RejectsSimultaneousGearButtons) {
  GearSelector selector(GearSelectorConfig{});
  std::vector<std::int32_t> buttons = releasedButtons();
  buttons[0] = 1;
  buttons[2] = 1;

  const GearSelectionResult result = selector.update(buttons, true, 0.0);

  EXPECT_EQ(GearSelectionStatus::kAmbiguousButtons, result.status);
  EXPECT_EQ(4U, result.gear);
}

TEST(GearSelector, RejectsButtonMessagesThatAreTooShort) {
  GearSelector selector(GearSelectorConfig{});

  const GearSelectionResult result =
      selector.update(std::vector<std::int32_t>(3U, 0), true, 0.0);

  EXPECT_EQ(GearSelectionStatus::kInvalidButtonMessage, result.status);
  EXPECT_EQ(4U, result.gear);
}

TEST(GearSelector, UsesConfiguredButtonsThresholdAndInitialGear) {
  GearSelectorConfig config;
  config.drive_button = 4;
  config.neutral_button = 5;
  config.reverse_button = 6;
  config.park_button = 7;
  config.initial_gear = 1U;
  config.maximum_change_speed_mps = 2.0;
  GearSelector selector(config);
  std::vector<std::int32_t> buttons = releasedButtons();
  buttons[4] = 1;

  const GearSelectionResult result = selector.update(buttons, true, 1.5);

  EXPECT_EQ(GearSelectionStatus::kChanged, result.status);
  EXPECT_EQ(4U, result.gear);
}

TEST(GearSelector, RejectsInvalidConfiguration) {
  GearSelectorConfig negative_button;
  negative_button.drive_button = -1;
  EXPECT_THROW((void)GearSelector{negative_button}, std::invalid_argument);

  GearSelectorConfig duplicate_button;
  duplicate_button.park_button = duplicate_button.drive_button;
  EXPECT_THROW((void)GearSelector{duplicate_button}, std::invalid_argument);

  GearSelectorConfig invalid_initial_gear;
  invalid_initial_gear.initial_gear = 5U;
  EXPECT_THROW((void)GearSelector{invalid_initial_gear},
               std::invalid_argument);

  GearSelectorConfig negative_speed;
  negative_speed.maximum_change_speed_mps = -0.1;
  EXPECT_THROW((void)GearSelector{negative_speed}, std::invalid_argument);
}

}  // namespace
}  // namespace vehicle_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
