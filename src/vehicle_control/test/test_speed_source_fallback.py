#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from vehicle_control.msg import VehicleCommand


class SpeedSourceFallbackTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._commands = []
        self._command_subscriber = rospy.Subscriber(
            "/test/speed_fallback/command",
            VehicleCommand,
            self._on_command,
            queue_size=10,
        )
        self._joy_publisher = rospy.Publisher(
            "/test/speed_fallback/joy",
            Joy,
            queue_size=10,
        )
        self._odometry_publisher = rospy.Publisher(
            "/test/speed_fallback/odometry",
            Odometry,
            queue_size=10,
        )

        deadline = time.monotonic() + 3.0
        while (
            self._joy_publisher.get_num_connections() == 0
            or self._command_subscriber.get_num_connections() == 0
        ):
            if time.monotonic() >= deadline:
                self.fail("speed fallback test topics were not ready")
            rospy.sleep(0.01)

    def tearDown(self):
        self._command_subscriber.unregister()
        self._joy_publisher.unregister()
        self._odometry_publisher.unregister()

    def _on_command(self, command):
        with self._condition:
            self._commands.append(command)
            self._condition.notify_all()

    def _publish_stopped_odometry(self):
        message = Odometry()
        message.twist.twist.linear.x = 0.0
        message.twist.twist.linear.y = 0.0
        for _ in range(5):
            message.header.stamp = rospy.Time.now()
            self._odometry_publisher.publish(message)
            rospy.sleep(0.02)

    def _publish_joy(self, reverse_pressed):
        sequence = int(time.monotonic_ns() % (2**32))
        frame_id = "speed-fallback-{}".format(sequence)
        message = Joy()
        message.header.seq = sequence
        message.header.frame_id = frame_id
        message.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0]
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

    def test_stopped_odometry_allows_gear_change_without_brake(self):
        self._publish_joy(reverse_pressed=False)
        self._publish_stopped_odometry()

        command = self._publish_joy(reverse_pressed=True)

        self.assertAlmostEqual(command.brake, 0.0)
        self.assertEqual(command.gear, VehicleCommand.GEAR_REVERSE)


if __name__ == "__main__":
    rospy.init_node("test_speed_source_fallback")
    rostest.rosrun(
        "vehicle_control",
        "speed_source_fallback",
        SpeedSourceFallbackTest,
    )
