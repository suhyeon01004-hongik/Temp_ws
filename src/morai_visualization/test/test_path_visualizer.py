#!/usr/bin/env python3

import unittest

import rospy
import rostest
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class PathVisualizerMarkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_path_visualizer_marker_contract")
        cls.pose_publisher = rospy.Publisher(
            "/test/localization_pose", PoseStamped, queue_size=1, latch=True
        )
        cls.lookahead_publisher = rospy.Publisher(
            "/test/lookahead_point", PointStamped, queue_size=1, latch=True
        )
        cls.stanley_projection_publisher = rospy.Publisher(
            "/test/stanley_projection_point",
            PointStamped,
            queue_size=1,
            latch=True,
        )
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while (
            (
                cls.pose_publisher.get_num_connections() == 0
                or cls.lookahead_publisher.get_num_connections() == 0
                or cls.stanley_projection_publisher.get_num_connections() == 0
            )
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

    def test_lookahead_is_a_point_without_connection_line(self):
        lookahead = PointStamped()
        lookahead.header.stamp = rospy.Time.now()
        lookahead.header.frame_id = "base_link"
        lookahead.point.x = 3.0
        lookahead.point.y = 0.5
        self.lookahead_publisher.publish(lookahead)

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        markers = []
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            markers = rospy.wait_for_message(
                "/test/path_markers", MarkerArray, timeout=1.0
            ).markers
            if any(marker.ns == "lookahead_point" for marker in markers):
                break
        lookahead_markers = [
            marker for marker in markers if marker.ns == "lookahead_point"
        ]
        self.assertEqual(len(lookahead_markers), 1)
        marker = lookahead_markers[0]
        self.assertEqual(marker.type, Marker.SPHERE)
        self.assertEqual(marker.header.frame_id, "base_link")
        self.assertTrue(marker.frame_locked)
        self.assertAlmostEqual(marker.pose.position.x, 3.0)
        self.assertAlmostEqual(marker.pose.position.y, 0.5)
        self.assertAlmostEqual(marker.scale.x, 0.8)
        self.assertGreater(marker.lifetime.to_sec(), 0.0)
        self.assertFalse(
            any(
                candidate.ns == "lookahead_point"
                and candidate.type in (Marker.LINE_LIST, Marker.LINE_STRIP)
                for candidate in markers
            )
        )

    def test_stanley_projection_is_a_distinct_point_without_line(self):
        projection = PointStamped()
        projection.header.stamp = rospy.Time.now()
        projection.header.frame_id = "base_link"
        projection.point.x = 3.0
        projection.point.y = -0.2
        self.stanley_projection_publisher.publish(projection)

        deadline = rospy.Time.now() + rospy.Duration(5.0)
        markers = []
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            markers = rospy.wait_for_message(
                "/test/path_markers", MarkerArray, timeout=1.0
            ).markers
            if any(
                marker.ns == "stanley_projection_point" for marker in markers
            ):
                break
        projection_markers = [
            marker
            for marker in markers
            if marker.ns == "stanley_projection_point"
        ]
        self.assertEqual(len(projection_markers), 1)
        marker = projection_markers[0]
        self.assertEqual(marker.type, Marker.SPHERE)
        self.assertEqual(marker.header.frame_id, "base_link")
        self.assertTrue(marker.frame_locked)
        self.assertAlmostEqual(marker.pose.position.x, 3.0)
        self.assertAlmostEqual(marker.pose.position.y, -0.2)
        self.assertNotEqual(marker.color, None)
        self.assertFalse(
            any(
                candidate.ns == "stanley_projection_point"
                and candidate.type in (Marker.LINE_LIST, Marker.LINE_STRIP)
                for candidate in markers
            )
        )


if __name__ == "__main__":
    rostest.rosrun(
        "morai_visualization",
        "path_visualizer_marker_contract",
        PathVisualizerMarkerTest,
    )
