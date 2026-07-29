#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from morai_udp_bridge.msg import ActuatorCommand
from nav_msgs.msg import Odometry, Path


class PurePursuitControllerTest(unittest.TestCase):
    def setUp(self):
        self._commands = []
        self._command_lock = threading.Lock()
        self._command_subscriber = rospy.Subscriber(
            "/control/actuator_command", ActuatorCommand, self._on_command, queue_size=10
        )
        self._odometry_publisher = rospy.Publisher(
            "/localization/odometry", Odometry, queue_size=1
        )
        self._path_publisher = rospy.Publisher("/local_path", Path, queue_size=1)

    def _on_command(self, message):
        with self._command_lock:
            self._commands.append(message)

    def _commands_since(self, index):
        with self._command_lock:
            return list(self._commands[index:])

    def _wait_for(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail("timed out waiting for " + description)

    def _publish_odometry(self):
        message = Odometry()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.0
        self._odometry_publisher.publish(message)

    def _publish_path(self, points):
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            message.poses.append(pose)
        self._path_publisher.publish(message)

    def test_safe_then_tracks_then_brakes_after_inputs_expire(self):
        self._wait_for(
            lambda: any(
                command.accel == 0.0
                and command.brake == 0.5
                and command.steering_angle_rad == 0.0
                for command in self._commands_since(0)
            ),
            3.0,
            "initial safe braking command before controller inputs",
        )

        command_index = len(self._commands_since(0))
        self._wait_for(
            lambda: self._odometry_publisher.get_num_connections() > 0
            and self._path_publisher.get_num_connections() > 0,
            3.0,
            "controller subscriptions to test odometry and path publishers",
        )
        self._publish_odometry()
        self._publish_path([(float(x), 0.0) for x in range(10)])
        self._wait_for(
            lambda: any(
                command.accel > 0.0
                and command.brake == 0.0
                and abs(command.steering_angle_rad) < 1.0e-4
                for command in self._commands_since(command_index)
            ),
            3.0,
            "accelerating straight-path command",
        )

        command_index = len(self._commands_since(0))
        self._publish_odometry()
        self._publish_path([(0.0, 0.0), (3.0, 1.5), (6.0, 3.0), (9.0, 4.5)])
        self._wait_for(
            lambda: any(
                command.steering_angle_rad > 0.0
                for command in self._commands_since(command_index)
            ),
            3.0,
            "positive steering command for left-curving path",
        )

        command_index = len(self._commands_since(0))
        self._wait_for(
            lambda: any(
                command.accel == 0.0
                and command.brake == 0.5
                and command.steering_angle_rad == 0.0
                for command in self._commands_since(command_index)
            ),
            1.5,
            "safe braking command after input timeout",
        )


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_test")
    rostest.rosrun("morai_path_tracking", "pure_pursuit_controller", PurePursuitControllerTest)
