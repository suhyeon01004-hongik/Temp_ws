#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import Joy
from vehicle_control.msg import VehicleCommand


class StandaloneGearFallbackTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._commands = []
        self._command_subscriber = rospy.Subscriber(
            "/test/standalone_gear/command",
            VehicleCommand,
            self._on_command,
            queue_size=10,
        )
        self._joy_publisher = rospy.Publisher(
            "/test/standalone_gear/joy",
            Joy,
            queue_size=10,
        )

        deadline = time.monotonic() + 3.0
        while (
            self._joy_publisher.get_num_connections() == 0
            or self._command_subscriber.get_num_connections() == 0
        ):
            if time.monotonic() >= deadline:
                self.fail("standalone gear test topics were not ready")
            rospy.sleep(0.01)

    def tearDown(self):
        self._command_subscriber.unregister()
        self._joy_publisher.unregister()

    def _on_command(self, command):
        with self._condition:
            self._commands.append(command)
            self._condition.notify_all()

    def _publish_input(self, brake_axis, accel_axis, reverse_pressed):
        sequence = int(time.monotonic_ns() % (2**32))
        frame_id = "standalone-gear-{}".format(sequence)
        message = Joy()
        message.header.seq = sequence
        message.header.frame_id = frame_id
        message.axes = [
            0.0,
            0.0,
            brake_axis,
            0.0,
            0.0,
            accel_axis,
        ]
        message.buttons = [0] * 11
        message.buttons[2] = int(reverse_pressed)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self._joy_publisher.publish(message)
            with self._condition:
                matching = [
                    command
                    for command in self._commands
                    if command.header.frame_id == frame_id
                ]
                if matching:
                    return matching[-1]
                self._condition.wait(timeout=0.05)
        self.fail("joystick teleop did not publish the test command")

    def test_missing_status_requires_brake_to_change_gear(self):
        released = 1.0
        pressed = -1.0

        without_brake = self._publish_input(
            brake_axis=released,
            accel_axis=released,
            reverse_pressed=True,
        )
        self.assertEqual(without_brake.gear, VehicleCommand.GEAR_DRIVE)

        self._publish_input(
            brake_axis=released,
            accel_axis=released,
            reverse_pressed=False,
        )
        with_brake = self._publish_input(
            brake_axis=pressed,
            accel_axis=released,
            reverse_pressed=True,
        )
        self.assertEqual(with_brake.gear, VehicleCommand.GEAR_REVERSE)


if __name__ == "__main__":
    rospy.init_node("test_standalone_gear_fallback")
    rostest.rosrun(
        "vehicle_control",
        "standalone_gear_fallback",
        StandaloneGearFallbackTest,
    )
