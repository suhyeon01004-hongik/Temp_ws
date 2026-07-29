#include "morai_path_tracking/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace morai_path_tracking {
namespace {

void requireFiniteNonNegative(const char* name, double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and non-negative");
  }
}

void requireFinitePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

}  // namespace

LongitudinalPid::LongitudinalPid(const PidConfig& config) : config_(config) {
  requireFiniteNonNegative("kp", config_.kp);
  requireFiniteNonNegative("ki", config_.ki);
  requireFiniteNonNegative("kd", config_.kd);
  requireFiniteNonNegative("integral_limit", config_.integral_limit);
  requireFinitePositive("integral_unwind_rate_per_sec",
                        config_.integral_unwind_rate_per_sec);
  requireFiniteNonNegative("error_deadband_mps", config_.error_deadband_mps);
  requireFiniteNonNegative("maximum_accel", config_.maximum_accel);
  requireFiniteNonNegative("maximum_brake", config_.maximum_brake);
}

LongitudinalCommand LongitudinalPid::update(double target_speed_mps,
                                            double measured_speed_mps,
                                            double dt_sec) {
  if (!std::isfinite(target_speed_mps) || !std::isfinite(measured_speed_mps) ||
      !std::isfinite(dt_sec) || dt_sec <= 0.0) {
    throw std::invalid_argument(
        "target speed, measured speed, and dt must be finite and dt positive");
  }

  const double error = target_speed_mps - measured_speed_mps;
  const bool inside_deadband =
      std::abs(error) <= config_.error_deadband_mps;
  const double derivative = !inside_deadband && has_previous_measurement_
                                ? (measured_speed_mps - previous_measurement_mps_) /
                                      dt_sec
                                : 0.0;
  const double candidate_integral = inside_deadband
                                        ? std::copysign(
                                              std::max(0.0, std::abs(integral_) -
                                                                config_.integral_unwind_rate_per_sec *
                                                                    dt_sec),
                                              integral_)
                                        : clamp(integral_ + error * dt_sec,
                                                -config_.integral_limit,
                                                config_.integral_limit);
  const double candidate_output =
      config_.kp * (inside_deadband ? 0.0 : error) +
      config_.ki * candidate_integral -
      config_.kd * derivative;

  const bool saturated_high = candidate_output > config_.maximum_accel;
  const bool saturated_low = candidate_output < -config_.maximum_brake;
  if (inside_deadband ||
      !((saturated_high && error > 0.0) ||
        (saturated_low && error < 0.0))) {
    integral_ = candidate_integral;
  }

  previous_measurement_mps_ = measured_speed_mps;
  has_previous_measurement_ = true;

  if (candidate_output >= 0.0) {
    return {clamp(candidate_output, 0.0, config_.maximum_accel), 0.0};
  }
  return {0.0, clamp(-candidate_output, 0.0, config_.maximum_brake)};
}

void LongitudinalPid::reset() {
  integral_ = 0.0;
  has_previous_measurement_ = false;
  previous_measurement_mps_ = 0.0;
}

}  // namespace morai_path_tracking
