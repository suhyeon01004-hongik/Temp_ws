#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class PathVisualizerMarkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_path_visualizer_marker_contract")
        cls.pose_publisher = rospy.Publisher(
            "/test/localization_pose", PoseStamped, queue_size=1, latch=True
        )
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while (
            cls.pose_publisher.get_num_connections() == 0
            and rospy.Time.now() < timeout
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.05)

    def test_vehicle_attached_markers_use_base_link(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = 12.0
        pose.pose.position.y = -3.0
        pose.pose.orientation.z = 0.7071067811865476
        pose.pose.orientation.w = 0.7071067811865476
        self.pose_publisher.publish(pose)

        markers = rospy.wait_for_message(
            "/test/path_markers", MarkerArray, timeout=5.0
        ).markers
        namespaces = {marker.ns for marker in markers}
        self.assertIn("vehicle_origin", namespaces)
        self.assertIn("vehicle_heading", namespaces)
        self.assertNotIn("current_position", namespaces)

        origin_markers = [
            marker for marker in markers if marker.ns == "vehicle_origin"
        ]
        sphere = next(
            marker for marker in origin_markers if marker.type == Marker.SPHERE
        )
        label = next(
            marker
            for marker in origin_markers
            if marker.type == Marker.TEXT_VIEW_FACING
        )
        heading = next(
            marker for marker in markers if marker.ns == "vehicle_heading"
        )

        for marker in (sphere, label, heading):
            self.assertEqual(marker.header.frame_id, "base_link")
            self.assertTrue(marker.frame_locked)
            self.assertEqual(marker.header.stamp, rospy.Time())

        self.assertEqual(label.text, "REAR AXLE")
        self.assertAlmostEqual(sphere.pose.position.x, 0.0)
        self.assertAlmostEqual(sphere.pose.position.y, 0.0)
        self.assertAlmostEqual(sphere.pose.position.z, 0.0)
        self.assertAlmostEqual(heading.pose.orientation.x, 0.0)
        self.assertAlmostEqual(heading.pose.orientation.y, 0.0)
        self.assertAlmostEqual(heading.pose.orientation.z, 0.0)
        self.assertAlmostEqual(heading.pose.orientation.w, 1.0)


if __name__ == "__main__":
    rostest.rosrun(
        "morai_visualization",
        "path_visualizer_marker_contract",
        PathVisualizerMarkerTest,
    )
