#!/usr/bin/env python3

import socket
import time
import unittest

import rospy
import rostest
from std_msgs.msg import Empty


class UdpSenderResetPauseTest(unittest.TestCase):
    def setUp(self):
        self._receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._receiver.bind(("127.0.0.1", 19093))
        self._reset_publisher = rospy.Publisher(
            "/test/udp_sender/reset", Empty, queue_size=1
        )

    def tearDown(self):
        self._receiver.close()

    def _wait_for_subscriber(self, timeout):
        deadline = time.monotonic() + timeout
        while (
            self._reset_publisher.get_num_connections() == 0
            and time.monotonic() < deadline
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.01)
        self.assertGreater(self._reset_publisher.get_num_connections(), 0)

    def _drain(self, duration):
        deadline = time.monotonic() + duration
        self._receiver.settimeout(0.01)
        while time.monotonic() < deadline:
            try:
                self._receiver.recvfrom(4096)
            except socket.timeout:
                pass

    def test_reset_request_pauses_then_resumes_udp(self):
        self._receiver.settimeout(2.0)
        self._receiver.recvfrom(4096)
        self._wait_for_subscriber(2.0)

        self._reset_publisher.publish(Empty())
        self._drain(0.08)

        self._receiver.settimeout(0.20)
        with self.assertRaises(socket.timeout):
            self._receiver.recvfrom(4096)

        rospy.sleep(0.45)
        self._receiver.settimeout(1.0)
        payload, _ = self._receiver.recvfrom(4096)
        self.assertGreater(len(payload), 0)


if __name__ == "__main__":
    rospy.init_node("test_udp_sender_reset_pause")
    rostest.rosrun(
        "vehicle_control",
        "test_udp_sender_reset_pause",
        UdpSenderResetPauseTest,
    )
