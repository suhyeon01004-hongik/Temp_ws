#pragma once

namespace morai_path_tracking {

struct PidConfig {
  double kp{0.35};
  double ki{0.08};
  double kd{0.02};
  double integral_limit{2.0};
  double error_deadband_mps{0.05};
  double maximum_accel{0.40};
  double maximum_brake{0.60};
};

struct LongitudinalCommand {
  double accel{0.0};
  double brake{0.0};
};

class LongitudinalPid {
 public:
  explicit LongitudinalPid(const PidConfig& config);
  LongitudinalCommand update(double target_speed_mps,
                             double measured_speed_mps, double dt_sec);
  void reset();

 private:
  PidConfig config_;
  double integral_{0.0};
  bool has_previous_measurement_{false};
  double previous_measurement_mps_{0.0};
};

}  // namespace morai_path_tracking
