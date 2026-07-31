#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_rviz(name):
    with (PACKAGE_ROOT / "rviz" / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def display_by_class(profile, class_name):
    displays = profile["Visualization Manager"]["Displays"]
    return next(display for display in displays if display["Class"] == class_name)


def displays_without_class(profile, class_name):
    return [
        display
        for display in profile["Visualization Manager"]["Displays"]
        if display["Class"] != class_name
    ]


class VisualizationAssetsTest(unittest.TestCase):
    def test_combined_profile_has_path_vehicle_tf_and_lidar(self):
        profile_path = PACKAGE_ROOT / "rviz" / "path_lidar.rviz"
        self.assertTrue(profile_path.exists(), "missing path_lidar.rviz")
        profile = load_rviz("path_lidar.rviz")
        manager = profile["Visualization Manager"]

        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        self.assertEqual(manager["Views"]["Current"]["Target Frame"], "base_link")
        self.assertEqual(
            display_by_class(profile, "rviz/MarkerArray")["Marker Topic"],
            "/visualization/path",
        )
        self.assertEqual(
            display_by_class(profile, "rviz/PointCloud2")["Topic"],
            "/lidar3D",
        )
        self.assertTrue(display_by_class(profile, "rviz/RobotModel")["Enabled"])
        self.assertFalse(display_by_class(profile, "rviz/TF")["Enabled"])

    def test_combined_profile_is_path_top_view_plus_lidar_only(self):
        path_profile = load_rviz("path.rviz")
        combined_profile = load_rviz("path_lidar.rviz")
        path_manager = path_profile["Visualization Manager"]
        combined_manager = combined_profile["Visualization Manager"]

        self.assertEqual(
            combined_manager["Views"],
            path_manager["Views"],
        )
        self.assertEqual(
            combined_manager["Global Options"],
            path_manager["Global Options"],
        )
        self.assertEqual(
            displays_without_class(combined_profile, "rviz/PointCloud2"),
            path_manager["Displays"],
        )

    def test_lidar_profiles_keep_latest_cloud_until_next_message(self):
        for profile_name in ("lidar.rviz", "path_lidar.rviz"):
            with self.subTest(profile=profile_name):
                point_cloud = display_by_class(
                    load_rviz(profile_name), "rviz/PointCloud2"
                )
                self.assertEqual(point_cloud["Decay Time"], 0.0)
                self.assertEqual(point_cloud["Color Transformer"], "FlatColor")
                self.assertEqual(point_cloud["Color"], "120; 220; 150")
                self.assertEqual(point_cloud["Alpha"], 0.45)

    def test_path_profiles_hide_tf_connectors_by_default(self):
        for profile_name in ("path.rviz", "path_lidar.rviz"):
            with self.subTest(profile=profile_name):
                tf_display = display_by_class(load_rviz(profile_name), "rviz/TF")
                self.assertFalse(tf_display["Enabled"])

        lidar_tf = display_by_class(load_rviz("lidar.rviz"), "rviz/TF")
        self.assertTrue(lidar_tf["Enabled"])

    def test_combined_launch_reuses_path_visualizer(self):
        launch_path = PACKAGE_ROOT / "launch" / "path_lidar.launch"
        self.assertTrue(launch_path.exists(), "missing path_lidar.launch")
        root = ET.parse(str(launch_path)).getroot()
        include = root.find(
            "./include[@file='$(find morai_visualization)/launch/path.launch']"
        )
        self.assertIsNotNone(include)
        start_rviz = include.find("./arg[@name='start_rviz']")
        self.assertIsNotNone(start_rviz)
        self.assertEqual(start_rviz.attrib["value"], "false")

        rviz_node = root.find("./node[@pkg='rviz']")
        self.assertIsNotNone(rviz_node)
        self.assertIn("$(arg rviz_config)", rviz_node.attrib["args"])

    def test_path_profile_names_the_rear_axle_origin(self):
        profile = load_rviz("path.rviz")
        marker_display = display_by_class(profile, "rviz/MarkerArray")
        namespaces = marker_display["Namespaces"]
        self.assertTrue(namespaces["vehicle_origin"])
        self.assertTrue(namespaces["lookahead_point"])
        self.assertTrue(namespaces["stanley_projection_point"])
        self.assertNotIn("current_position", namespaces)

    def test_lidar_only_profile_remains_localization_independent(self):
        profile = load_rviz("lidar.rviz")
        manager = profile["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "base_link")
        self.assertEqual(manager["Views"]["Current"]["Target Frame"], "base_link")
        self.assertEqual(
            display_by_class(profile, "rviz/PointCloud2")["Topic"],
            "/lidar3D",
        )


if __name__ == "__main__":
    unittest.main()
