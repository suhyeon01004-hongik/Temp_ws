#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from vehicle_control.msg import VehicleCommand


class CyvoxMappingTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._commands = []
        self._command_subscriber = rospy.Subscriber(
            "/test/cyvox/command",
            VehicleCommand,
            self._on_command,
            queue_size=10,
        )
        self._joy_publisher = rospy.Publisher(
            "/test/cyvox/joy",
            Joy,
            queue_size=10,
        )
        self._odometry_publisher = rospy.Publisher(
            "/test/cyvox/odometry",
            Odometry,
            queue_size=10,
        )

        deadline = time.monotonic() + 3.0
        while (
            self._joy_publisher.get_num_connections() == 0
            or self._command_subscriber.get_num_connections() == 0
        ):
            if time.monotonic() >= deadline:
                self.fail("joystick teleop topic connections were not ready")
            rospy.sleep(0.01)

    def tearDown(self):
        self._command_subscriber.unregister()
        self._joy_publisher.unregister()
        self._odometry_publisher.unregister()

    def _on_command(self, command):
        with self._condition:
            self._commands.append(command)
            self._condition.notify_all()

    def _publish_speed(self, speed_mps):
        odometry = Odometry()
        odometry.header.stamp = rospy.Time.now()
        odometry.twist.twist.linear.x = speed_mps
        for _ in range(5):
            self._odometry_publisher.publish(odometry)
            rospy.sleep(0.02)

    def _map_input(self, brake_axis, accel_axis, buttons=None):
        sequence = int(time.monotonic_ns() % (2**32))
        frame_id = "cyvox-test-{}".format(sequence)
        joy = Joy()
        joy.header.seq = sequence
        joy.header.frame_id = frame_id
        joy.axes = [0.0, 0.0, brake_axis, 0.0, 0.0, accel_axis]
        joy.buttons = list(buttons) if buttons is not None else [0] * 11

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self._joy_publisher.publish(joy)
            with self._condition:
                matching = [
                    command
                    for command in self._commands
                    if command.header.frame_id == frame_id
                ]
                if matching:
                    return matching[-1]
                self._condition.wait(timeout=0.05)
        observed = [
            (command.header.frame_id, command.brake, command.accel)
            for command in self._commands[-5:]
        ]
        self.fail(
            "joystick teleop node did not publish a matching command; "
            "expected frame_id={}, observed={}".format(frame_id, observed)
        )

    def _map_axes(self, brake_axis, accel_axis):
        return self._map_input(brake_axis, accel_axis)

    def _select_gear(self, button_index, speed_mps):
        self._publish_speed(speed_mps)
        self._map_input(1.0, 1.0)
        buttons = [0] * 11
        buttons[button_index] = 1
        return self._map_input(1.0, 1.0, buttons)

    def test_initial_trigger_state_is_loaded_from_device(self):
        self.assertTrue(
            rospy.get_param(
                "/cyvox_mapping_test/joy_node/default_trig_val"
            )
        )

    def test_released_triggers_produce_coasting_command(self):
        command = self._map_axes(brake_axis=1.0, accel_axis=1.0)

        self.assertAlmostEqual(command.brake, 0.0)
        self.assertAlmostEqual(command.accel, 0.0)

    def test_pressed_triggers_produce_full_pedal_commands(self):
        command = self._map_axes(brake_axis=-1.0, accel_axis=-1.0)

        self.assertAlmostEqual(command.brake, 1.0)
        self.assertAlmostEqual(command.accel, 1.0)

    def test_reverse_button_changes_gear_at_low_speed(self):
        command = self._select_gear(button_index=2, speed_mps=0.0)

        self.assertEqual(command.gear, VehicleCommand.GEAR_REVERSE)

    def test_park_button_is_rejected_above_speed_limit(self):
        self._select_gear(button_index=0, speed_mps=0.0)
        command = self._select_gear(button_index=3, speed_mps=1.0)

        self.assertEqual(command.gear, VehicleCommand.GEAR_DRIVE)

    def test_stale_odometry_rejects_gear_change(self):
        self._select_gear(button_index=0, speed_mps=0.0)
        self._map_input(1.0, 1.0)
        rospy.sleep(0.2)
        buttons = [0] * 11
        buttons[2] = 1

        command = self._map_input(1.0, 1.0, buttons)

        self.assertEqual(command.gear, VehicleCommand.GEAR_DRIVE)


if __name__ == "__main__":
    rospy.init_node("test_cyvox_mapping")
    rostest.rosrun(
        "vehicle_control",
        "cyvox_mapping",
        CyvoxMappingTest,
    )
