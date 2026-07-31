#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PurePursuitConfigContractTest(unittest.TestCase):
    def test_runtime_yaml_contains_every_required_parameter(self):
        with (
            PACKAGE_ROOT / "config" / "molit_2026_path_tracking.yaml"
        ).open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)["path_tracking_controller_node"]

        expected = {
            "local_path_topic",
            "odometry_topic",
            "vehicle_status_topic",
            "command_topic",
            "controller_status_topic",
            "lookahead_point_topic",
            "stanley_projection_point_topic",
            "expected_frame_id",
            "expected_velocity_frame_id",
            "control_rate_hz",
            "path_timeout_sec",
            "odometry_timeout_sec",
            "vehicle_status_timeout_sec",
            "maximum_input_skew_sec",
            "input_sync_queue_size",
            "minimum_control_dt_sec",
            "maximum_control_dt_sec",
            "safe_brake_command",
            "wheelbase_m",
            "lookahead_base_m",
            "lookahead_speed_gain_sec",
            "lookahead_curvature_gain_m",
            "lookahead_min_m",
            "lookahead_max_m",
            "minimum_target_distance_m",
            "maximum_steering_angle_deg",
            "lateral_controller",
            "stanley_gain",
            "stanley_softening_speed_mps",
            "stanley_minimum_control_speed_mps",
            "stanley_heading_window_m",
            "stanley_heading_error_gain",
            "stanley_curvature_feedforward_gain",
            "stanley_curvature_preview_distance_m",
            "stanley_yaw_rate_damping_gain_sec",
            "stanley_yaw_rate_damping_nonlinear_gain_sec2",
            "stanley_maximum_steering_rate_deg_per_sec",
            "hybrid_mass_kg",
            "hybrid_yaw_inertia_kgm2",
            "hybrid_front_cornering_stiffness_n_per_rad",
            "hybrid_rear_cornering_stiffness_n_per_rad",
            "hybrid_front_axle_to_cg_m",
            "hybrid_rear_axle_to_cg_m",
            "hybrid_process_noise_sideslip",
            "hybrid_process_noise_yaw_rate",
            "hybrid_measurement_noise_sideslip",
            "hybrid_measurement_noise_yaw_rate",
            "hybrid_initial_covariance_sideslip",
            "hybrid_initial_covariance_yaw_rate",
            "hybrid_initial_pure_pursuit_probability",
            "hybrid_initial_stanley_probability",
            "hybrid_stanley_probability_min",
            "hybrid_stanley_probability_max",
            "hybrid_transition_pure_pursuit_to_pure_pursuit",
            "hybrid_transition_pure_pursuit_to_stanley",
            "hybrid_transition_stanley_to_pure_pursuit",
            "hybrid_transition_stanley_to_stanley",
            "hybrid_transition_speed_gain",
            "hybrid_transition_reference_speed_kph",
            "hybrid_minimum_model_speed_mps",
            "hybrid_pure_pursuit_cross_track_correction_gain",
            "hybrid_candidate_conflict_curvature_threshold_m_inv",
            "hybrid_candidate_conflict_cross_track_threshold_m",
            "hybrid_cross_track_recovery_full_scale_m",
            "hybrid_cross_track_recovery_heading_error_suppression_start_deg",
            "hybrid_cross_track_recovery_heading_error_suppression_full_deg",
            "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio",
            "hybrid_maximum_steering_rate_deg_per_sec",
            "hybrid_steering_return_rate_multiplier",
            "target_speed_kph",
            "minimum_curve_speed_kph",
            "maximum_lateral_acceleration_mps2",
            "curvature_speed_reduction_gain_m",
            "curvature_preview_distance_m",
            "lookahead_curvature_preview_distance_m",
            "curvature_sample_spacing_m",
            "curve_approach_deceleration_mps2",
            "curvature_epsilon_m_inv",
            "target_speed_acceleration_limit_mps2",
            "curve_target_speed_acceleration_limit_mps2",
            "target_speed_deceleration_limit_mps2",
            "target_speed_filter_time_constant_sec",
            "speed_filter_time_constant_sec",
            "speed_kp",
            "speed_ki",
            "speed_kd",
            "speed_integral_limit",
            "speed_integral_unwind_rate_per_sec",
            "speed_error_deadband_mps",
            "speed_accel_feedforward_gain_per_mps",
            "speed_coast_overspeed_kph",
            "speed_brake_overspeed_kph",
            "hard_brake_activation_speed_kph",
            "minimum_hard_brake_command",
            "maximum_accel_command",
            "maximum_brake_command",
            "longitudinal_command_rate_limit_per_sec",
        }
        self.assertEqual(set(config), expected)
        self.assertEqual(config["vehicle_status_topic"], "/vehicle/competition_status")
        self.assertEqual(config["target_speed_kph"], 58.0)
        self.assertEqual(config["minimum_curve_speed_kph"], 12.0)
        self.assertEqual(config["maximum_lateral_acceleration_mps2"], 1.8)
        self.assertEqual(config["curvature_speed_reduction_gain_m"], 5.0)
        self.assertEqual(config["curvature_preview_distance_m"], 45.0)
        self.assertEqual(config["lookahead_curvature_preview_distance_m"], 8.0)
        self.assertEqual(config["curvature_sample_spacing_m"], 2.0)
        self.assertEqual(config["curve_approach_deceleration_mps2"], 1.0)
        self.assertEqual(config["target_speed_acceleration_limit_mps2"], 2.0)
        self.assertEqual(
            config["curve_target_speed_acceleration_limit_mps2"], 0.2
        )
        self.assertEqual(config["target_speed_deceleration_limit_mps2"], 5.0)
        self.assertEqual(config["lookahead_base_m"], 4.0)
        self.assertEqual(config["lookahead_speed_gain_sec"], 0.75)
        self.assertEqual(config["lookahead_curvature_gain_m"], 8.0)
        self.assertEqual(config["lookahead_min_m"], 4.0)
        self.assertEqual(config["lookahead_max_m"], 16.0)
        self.assertEqual(config["lateral_controller"], "hybrid")
        self.assertEqual(config["stanley_gain"], 2.0)
        self.assertEqual(config["stanley_softening_speed_mps"], 2.0)
        self.assertEqual(config["stanley_minimum_control_speed_mps"], 1.0)
        self.assertEqual(config["stanley_heading_window_m"], 4.0)
        self.assertEqual(config["stanley_heading_error_gain"], 0.6)
        self.assertEqual(config["stanley_curvature_feedforward_gain"], 1.0)
        self.assertEqual(config["stanley_curvature_preview_distance_m"], 8.0)
        self.assertEqual(config["stanley_yaw_rate_damping_gain_sec"], 0.1)
        self.assertEqual(
            config["stanley_yaw_rate_damping_nonlinear_gain_sec2"], 0.4
        )
        self.assertEqual(
            config["stanley_maximum_steering_rate_deg_per_sec"], 60.0
        )
        self.assertEqual(config["hybrid_mass_kg"], 2000.0)
        self.assertEqual(config["hybrid_yaw_inertia_kgm2"], 4000.0)
        self.assertEqual(
            config["hybrid_front_cornering_stiffness_n_per_rad"], 60000.0
        )
        self.assertEqual(
            config["hybrid_rear_cornering_stiffness_n_per_rad"], 60000.0
        )
        self.assertEqual(config["hybrid_front_axle_to_cg_m"], 1.5)
        self.assertEqual(config["hybrid_rear_axle_to_cg_m"], 1.5)
        self.assertEqual(config["hybrid_process_noise_sideslip"], 0.1)
        self.assertEqual(config["hybrid_process_noise_yaw_rate"], 0.01)
        self.assertEqual(config["hybrid_measurement_noise_sideslip"], 0.001)
        self.assertEqual(config["hybrid_measurement_noise_yaw_rate"], 0.001)
        self.assertEqual(
            config["hybrid_initial_pure_pursuit_probability"], 0.8
        )
        self.assertEqual(config["hybrid_initial_stanley_probability"], 0.2)
        self.assertEqual(config["hybrid_stanley_probability_min"], 0.15)
        self.assertEqual(config["hybrid_stanley_probability_max"], 0.90)
        self.assertEqual(config["hybrid_transition_speed_gain"], 0.1)
        self.assertEqual(config["hybrid_transition_reference_speed_kph"], 60.0)
        self.assertEqual(
            config["hybrid_pure_pursuit_cross_track_correction_gain"], 0.60
        )
        self.assertEqual(
            config["hybrid_candidate_conflict_curvature_threshold_m_inv"],
            0.015,
        )
        self.assertEqual(
            config["hybrid_candidate_conflict_cross_track_threshold_m"], 0.45
        )
        self.assertEqual(
            config["hybrid_cross_track_recovery_full_scale_m"], 0.55
        )
        self.assertEqual(
            config[
                "hybrid_cross_track_recovery_heading_error_suppression_start_deg"
            ],
            15.0,
        )
        self.assertEqual(
            config[
                "hybrid_cross_track_recovery_heading_error_suppression_full_deg"
            ],
            17.5,
        )
        self.assertEqual(
            config[
                "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio"
            ],
            0.30,
        )
        self.assertEqual(
            config["hybrid_maximum_steering_rate_deg_per_sec"], 60.0
        )
        self.assertEqual(
            config["hybrid_steering_return_rate_multiplier"], 2.0
        )
        self.assertEqual(config["speed_kp"], 0.18)
        self.assertEqual(config["speed_ki"], 0.02)
        self.assertEqual(config["speed_kd"], 0.0)
        self.assertEqual(config["speed_integral_limit"], 1.0)
        self.assertEqual(config["speed_error_deadband_mps"], 0.10)
        self.assertEqual(config["speed_accel_feedforward_gain_per_mps"], 0.008)
        self.assertEqual(config["speed_coast_overspeed_kph"], 0.2)
        self.assertEqual(config["speed_brake_overspeed_kph"], 1.8)
        self.assertEqual(config["hard_brake_activation_speed_kph"], 59.0)
        self.assertEqual(config["minimum_hard_brake_command"], 0.25)
        self.assertEqual(config["target_speed_filter_time_constant_sec"], 0.35)
        self.assertEqual(config["longitudinal_command_rate_limit_per_sec"], 2.0)
        self.assertNotIn("target_speed_mps", config)
        self.assertEqual(config["speed_filter_time_constant_sec"], 0.0)
        self.assertEqual(config["maximum_steering_angle_deg"], 40.0)


if __name__ == "__main__":
    unittest.main()
