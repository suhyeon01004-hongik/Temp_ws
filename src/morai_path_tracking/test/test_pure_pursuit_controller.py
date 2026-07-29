#!/usr/bin/env python3

import subprocess
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from morai_udp_bridge.msg import ActuatorCommand
from nav_msgs.msg import Odometry, Path


class PurePursuitControllerTest(unittest.TestCase):
    SAFE_BRAKE = 0.5
    INPUT_TIMEOUT_SEC = 0.25
    INVALID_REACTION_TIMEOUT_SEC = 0.12
    TIMEOUT_REACTION_GRACE_SEC = 0.20
    ODOMETRY_REFRESH_PERIOD_SEC = 0.04
    WAIT_TIMEOUT_SEC = 3.0

    def setUp(self):
        self._commands = []
        self._command_lock = threading.Lock()
        self._command_subscriber = rospy.Subscriber(
            "/control/actuator_command", ActuatorCommand, self._on_command, queue_size=50
        )
        self._odometry_publisher = rospy.Publisher(
            "/localization/odometry", Odometry, queue_size=1
        )
        self._path_publisher = rospy.Publisher("/local_path", Path, queue_size=1)

    def _on_command(self, message):
        with self._command_lock:
            self._commands.append((time.monotonic(), message))

    def _commands_after(self, mark):
        with self._command_lock:
            return [entry for entry in self._commands if entry[0] >= mark]

    @staticmethod
    def _is_safe(command):
        return (
            command.accel == 0.0
            and command.brake == PurePursuitControllerTest.SAFE_BRAKE
            and command.steering_angle_rad == 0.0
        )

    @staticmethod
    def _safe_at_or_after_cutoff(receipt_time, command, cutoff):
        del receipt_time
        return (
            PurePursuitControllerTest._is_safe(command)
            and command.header.stamp >= cutoff
        )

    def _wait_for(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail("timed out waiting for " + description)

    def _wait_for_until(self, predicate, deadline, description):
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail("timed out waiting for " + description)

    def _safe_commands_before_output_cutoff(self, start, cutoff):
        return [
            command
            for receipt_time, command in self._commands_after(start)
            if self._is_safe(command) and command.header.stamp < cutoff
        ]

    def _wait_for_safe_output_window(
        self, start, not_before, output_deadline, monotonic_deadline, description
    ):
        self._wait_for_until(
            lambda: any(
                self._is_safe(command)
                and command.header.stamp >= not_before
                and command.header.stamp < output_deadline
                for _, command in self._commands_after(start)
            ),
            monotonic_deadline,
            description,
        )

    def _assert_no_safe_output_before(self, start, cutoff, description):
        early_safe_commands = self._safe_commands_before_output_cutoff(start, cutoff)
        self.assertFalse(
            early_safe_commands,
            "{} (cutoff={} now={})".format(
                description, cutoff.to_sec(), rospy.Time.now().to_sec()
            ),
        )

    def _wait_for_safe_after_cutoff_without_early(
        self, start, cutoff, monotonic_deadline, description
    ):
        while time.monotonic() < monotonic_deadline and not rospy.is_shutdown():
            self._assert_no_safe_output_before(
                start, cutoff, "controller published a safe command before " + description
            )
            if any(
                self._safe_at_or_after_cutoff(receipt_time, command, cutoff)
                for receipt_time, command in self._commands_after(start)
            ):
                return
            rospy.sleep(0.01)
        self._assert_no_safe_output_before(
            start, cutoff, "controller published a safe command before " + description
        )
        self.fail("timed out waiting for " + description)

    def _publish_inputs(
        self,
        points=None,
        odometry_frame="map",
        path_frame="map",
        odometry_stamp=None,
        path_stamp=None,
        position=(0.0, 0.0, 0.0),
        speed=0.0,
        orientation=(0.0, 0.0, 0.0, 1.0),
        publish_odometry=True,
        publish_path=True,
    ):
        if points is None:
            points = [(float(x), 0.0) for x in range(10)]
        odometry = Odometry()
        odometry.header.frame_id = odometry_frame
        if odometry_stamp is not None:
            odometry.header.stamp = odometry_stamp
        odometry.pose.pose.position.x = position[0]
        odometry.pose.pose.position.y = position[1]
        odometry.pose.pose.position.z = position[2]
        odometry.pose.pose.orientation.x = orientation[0]
        odometry.pose.pose.orientation.y = orientation[1]
        odometry.pose.pose.orientation.z = orientation[2]
        odometry.pose.pose.orientation.w = orientation[3]
        odometry.twist.twist.linear.x = speed

        path = Path()
        path.header.frame_id = path_frame
        if path_stamp is not None:
            path.header.stamp = path_stamp
        for x, y in points:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        publish_mark = time.monotonic()
        publication_stamp = rospy.Time.now()
        if odometry_stamp is None:
            odometry.header.stamp = publication_stamp
        if path_stamp is None:
            path.header.stamp = publication_stamp
        if publish_odometry:
            self._odometry_publisher.publish(odometry)
        if publish_path:
            self._path_publisher.publish(path)
        return {
            "monotonic_mark": publish_mark,
            "publication_stamp": publication_stamp,
            "odometry_stamp": odometry.header.stamp,
            "path_stamp": path.header.stamp,
        }

    def _wait_for_subscriptions(self):
        self._wait_for(
            lambda: self._odometry_publisher.get_num_connections() > 0
            and self._path_publisher.get_num_connections() > 0,
            self.WAIT_TIMEOUT_SEC,
            "controller subscriptions to test odometry and path publishers",
        )

    def _establish_valid_baseline(self):
        inputs = self._publish_inputs()
        mark = inputs["monotonic_mark"]
        cutoff = inputs["path_stamp"] + rospy.Duration(self.INPUT_TIMEOUT_SEC)
        self._wait_for(
            lambda: any(
                command.accel > 0.0
                and command.brake == 0.0
                and abs(command.steering_angle_rad) < 1.0e-4
                and command.header.stamp >= inputs["publication_stamp"]
                for _, command in self._commands_after(mark)
            ),
            self.WAIT_TIMEOUT_SEC,
            "fresh valid straight-path command",
        )
        valid_receipts = [
            receipt_time
            for receipt_time, command in self._commands_after(mark)
            if command.accel > 0.0
            and command.brake == 0.0
            and abs(command.steering_angle_rad) < 1.0e-4
            and command.header.stamp >= inputs["publication_stamp"]
        ]
        return {
            "monotonic_mark": mark,
            "valid_receipt": min(valid_receipts),
            "publication_stamp": inputs["publication_stamp"],
            "retained_path_stamp": inputs["path_stamp"],
            "cutoff": cutoff,
        }

    def _assert_invalid_inputs_brake(self, description, **kwargs):
        baseline = self._establish_valid_baseline()
        invalid_inputs = self._publish_inputs(**kwargs)
        remaining_sec = (baseline["cutoff"] - invalid_inputs["publication_stamp"]).to_sec()
        if remaining_sec < self.INVALID_REACTION_TIMEOUT_SEC:
            self.fail(
                "fresh baseline left only {:.3f}s before its natural expiry; "
                "need at least {:.3f}s for invalid-input reaction".format(
                    remaining_sec, self.INVALID_REACTION_TIMEOUT_SEC
                )
            )
        self._wait_for_safe_output_window(
            invalid_inputs["monotonic_mark"],
            invalid_inputs["publication_stamp"],
            baseline["cutoff"],
            time.monotonic() + self.INVALID_REACTION_TIMEOUT_SEC,
            description,
        )

    @staticmethod
    def _controller_parameters(safe_brake_command):
        return {
            "local_path_topic": "/local_path",
            "odometry_topic": "/localization/odometry",
            "command_topic": "/invalid_config_command",
            "expected_frame_id": "map",
            "control_rate_hz": 30.0,
            "path_timeout_sec": 0.25,
            "odometry_timeout_sec": 0.25,
            "maximum_input_skew_sec": 0.10,
            "safe_brake_command": safe_brake_command,
            "wheelbase_m": 3.0,
            "lookahead_base_m": 3.0,
            "lookahead_speed_gain_sec": 0.5,
            "lookahead_min_m": 3.0,
            "lookahead_max_m": 6.0,
            "minimum_target_distance_m": 0.5,
            "maximum_steering_angle_deg": 40.0,
            "target_speed_mps": 3.0,
            "speed_kp": 0.35,
            "speed_ki": 0.08,
            "speed_kd": 0.02,
            "speed_integral_limit": 2.0,
            "speed_error_deadband_mps": 0.05,
            "maximum_accel_command": 0.40,
            "maximum_brake_command": 0.60,
        }

    def test_rejects_nan_safe_brake_configuration(self):
        node_name = "invalid_config_controller"
        rospy.set_param("/" + node_name, self._controller_parameters(float("nan")))
        process = subprocess.Popen(
            [
                "rosrun",
                "morai_path_tracking",
                "pure_pursuit_controller_node",
                "__name:=" + node_name,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            self._wait_for(
                lambda: process.poll() is not None,
                self.WAIT_TIMEOUT_SEC,
                "controller rejection of NaN safe_brake_command",
            )
            self.assertNotEqual(0, process.returncode)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=1.0)
            rospy.delete_param("/" + node_name)

    def test_output_stamp_classification_rejects_late_delivery_of_early_safe(self):
        cutoff = rospy.Time(10, 250000000)
        command = ActuatorCommand()
        command.brake = self.SAFE_BRAKE
        command.header.stamp = rospy.Time(10, 249000000)
        self.assertFalse(
            self._safe_at_or_after_cutoff(11.0, command, cutoff),
            "an early controller output must remain early when delivery is delayed",
        )

    def test_safe_state_for_missing_and_invalid_inputs(self):
        initial_mark = time.monotonic()
        self._wait_for(
            lambda: any(
                self._is_safe(command) for _, command in self._commands_after(initial_mark)
            ),
            self.WAIT_TIMEOUT_SEC,
            "initial safe braking command before controller inputs",
        )
        self._wait_for_subscriptions()

        baseline = self._establish_valid_baseline()
        path_timeout_cutoff = baseline["cutoff"]
        pre_cutoff_deadline = (
            baseline["monotonic_mark"]
            + self.INPUT_TIMEOUT_SEC
            + self.TIMEOUT_REACTION_GRACE_SEC
        )
        while (
            rospy.Time.now() < path_timeout_cutoff
            and time.monotonic() < pre_cutoff_deadline
            and not rospy.is_shutdown()
        ):
            self._publish_inputs(
                publish_path=False,
                odometry_stamp=baseline["retained_path_stamp"],
            )
            self._assert_no_safe_output_before(
                baseline["valid_receipt"],
                path_timeout_cutoff,
                "controller braked before the retained path receipt timeout",
            )
            refresh_deadline = min(
                pre_cutoff_deadline,
                time.monotonic() + self.ODOMETRY_REFRESH_PERIOD_SEC,
            )
            while time.monotonic() < refresh_deadline and not rospy.is_shutdown():
                self._assert_no_safe_output_before(
                    baseline["valid_receipt"],
                    path_timeout_cutoff,
                    "controller braked before the retained path receipt timeout",
                )
                rospy.sleep(0.01)
        if rospy.Time.now() < path_timeout_cutoff:
            self.fail("ROS time did not reach the retained path timeout before deadline")
        self._wait_for_safe_after_cutoff_without_early(
            baseline["valid_receipt"],
            path_timeout_cutoff,
            time.monotonic() + self.TIMEOUT_REACTION_GRACE_SEC,
            "safe brake after the retained path receipt timeout",
        )

        self._assert_invalid_inputs_brake(
            "safe brake for wrong input frame", odometry_frame="odom", path_frame="odom"
        )
        self._assert_invalid_inputs_brake(
            "safe brake for zero input stamps",
            odometry_stamp=rospy.Time(),
            path_stamp=rospy.Time(),
        )

        stale_stamp = rospy.Time.now() - rospy.Duration(1.0)
        self._assert_invalid_inputs_brake(
            "safe brake for stale input stamps",
            odometry_stamp=stale_stamp,
            path_stamp=stale_stamp,
        )
        future_stamp = rospy.Time.now() + rospy.Duration(1.0)
        self._assert_invalid_inputs_brake(
            "safe brake for future input stamps",
            odometry_stamp=future_stamp,
            path_stamp=future_stamp,
        )

        skew_now = rospy.Time.now()
        self._assert_invalid_inputs_brake(
            "safe brake for excessive input stamp skew",
            odometry_stamp=skew_now - rospy.Duration(0.15),
            path_stamp=skew_now,
        )
        self._assert_invalid_inputs_brake(
            "safe brake for non-finite odometry pose", position=(float("nan"), 0.0, 0.0)
        )
        self._assert_invalid_inputs_brake(
            "safe brake for non-finite odometry speed", speed=float("inf")
        )
        self._assert_invalid_inputs_brake(
            "safe brake for non-finite path point", points=[(0.0, 0.0), (float("nan"), 1.0)]
        )
        self._assert_invalid_inputs_brake(
            "safe brake for invalid odometry quaternion", orientation=(0.0, 0.0, 0.0, 0.0)
        )

    def test_timeout_does_not_brake_immediately_after_valid_left_command(self):
        self._wait_for_subscriptions()
        self._establish_valid_baseline()
        inputs = self._publish_inputs(
            points=[(0.0, 0.0), (3.0, 1.5), (6.0, 3.0), (9.0, 4.5)]
        )
        mark = inputs["monotonic_mark"]
        timeout_cutoff = inputs["path_stamp"] + rospy.Duration(self.INPUT_TIMEOUT_SEC)
        self._wait_for_until(
            lambda: any(
                command.header.stamp < timeout_cutoff
                and command.header.stamp >= inputs["publication_stamp"]
                and command.steering_angle_rad > 0.0
                and command.accel > 0.0
                and command.brake == 0.0
                for _, command in self._commands_after(mark)
            ),
            mark + self.INVALID_REACTION_TIMEOUT_SEC,
            "positive steering command for fresh left-curving path",
        )
        self._wait_for_safe_after_cutoff_without_early(
            mark,
            timeout_cutoff,
            mark + self.INPUT_TIMEOUT_SEC + self.TIMEOUT_REACTION_GRACE_SEC,
            "safe braking command after the input timeout",
        )


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_test")
    rostest.rosrun("morai_path_tracking", "pure_pursuit_controller", PurePursuitControllerTest)
