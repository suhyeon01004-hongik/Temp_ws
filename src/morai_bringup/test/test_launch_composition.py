#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


class BringupLaunchCompositionTest(unittest.TestCase):
    def load_launch(self, name):
        path = LAUNCH_DIR / name
        self.assertTrue(path.exists(), "missing launch file {}".format(name))
        return ET.parse(str(path)).getroot()

    @staticmethod
    def include_files(root):
        return {
            include.attrib["file"]
            for include in root.findall(".//include")
        }

    @staticmethod
    def argument_defaults(root):
        return {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("./arg")
        }

    @staticmethod
    def include_by_file(root, file_name):
        return root.find("./group/include[@file='{}']".format(file_name))

    @staticmethod
    def include_arguments(include):
        return {
            argument.attrib["name"]: argument.attrib["value"]
            for argument in include.findall("./arg")
        }

    def test_sensor_bringup_has_no_localization_or_path_contract(self):
        root = self.load_launch("molit_2026_sensors.launch")
        argument_names = {
            argument.attrib["name"] for argument in root.findall("./arg")
        }
        forbidden_arguments = {
            "use_gps_localization",
            "use_path_manager",
            "localization_config",
            "route_path_config",
            "global_path_file",
        }
        self.assertTrue(forbidden_arguments.isdisjoint(argument_names))

        includes = self.include_files(root)
        self.assertNotIn(
            "$(find morai_localization)/launch/localization.launch",
            includes,
        )
        self.assertNotIn(
            "$(find morai_path_manager)/launch/route_path_publisher.launch",
            includes,
        )

    def test_sensor_lidar_uses_fixed_scan_cut_without_map_dependency(self):
        root = self.load_launch("molit_2026_sensors.launch")
        defaults = self.argument_defaults(root)
        self.assertEqual(defaults["lidar_cut_angle"], "0.0")
        self.assertEqual(defaults["lidar_fixed_frame"], "")
        self.assertEqual(defaults["lidar_target_frame"], "")

        driver = self.include_by_file(
            root,
            "$(find velodyne_driver)/launch/nodelet_manager.launch",
        )
        self.assertIsNotNone(driver)
        driver_arguments = self.include_arguments(driver)
        self.assertEqual(
            driver_arguments["cut_angle"],
            "$(arg lidar_cut_angle)",
        )
        self.assertEqual(driver_arguments["model"], "VLP16")

        transform = self.include_by_file(
            root,
            "$(find velodyne_pointcloud)/launch/transform_nodelet.launch",
        )
        self.assertIsNotNone(transform)
        transform_arguments = self.include_arguments(transform)
        self.assertEqual(
            transform_arguments["fixed_frame"],
            "$(arg lidar_fixed_frame)",
        )
        self.assertEqual(
            transform_arguments["target_frame"],
            "$(arg lidar_target_frame)",
        )

    def test_localization_bringup_wraps_localization_package(self):
        root = self.load_launch("molit_2026_localization.launch")
        includes = self.include_files(root)
        self.assertEqual(
            includes,
            {"$(find morai_localization)/launch/localization.launch"},
        )

    def test_path_manager_bringup_wraps_path_manager_package(self):
        root = self.load_launch("molit_2026_path_manager.launch")
        includes = self.include_files(root)
        self.assertEqual(
            includes,
            {
                "$(find morai_path_manager)/launch/"
                "route_path_publisher.launch"
            },
        )

    def test_full_stack_composes_all_three_bringup_launches(self):
        root = self.load_launch("molit_2026_stack.launch")
        includes = self.include_files(root)
        expected = {
            "$(find morai_bringup)/launch/molit_2026_sensors.launch",
            "$(find morai_bringup)/launch/molit_2026_localization.launch",
            "$(find morai_bringup)/launch/molit_2026_path_manager.launch",
        }
        self.assertEqual(includes, expected)

        argument_names = {
            argument.attrib["name"] for argument in root.findall("./arg")
        }
        self.assertNotIn("use_gps_localization", argument_names)
        self.assertNotIn("use_path_manager", argument_names)

    def test_full_stack_enables_lidar_ego_motion_compensation(self):
        root = self.load_launch("molit_2026_stack.launch")
        defaults = self.argument_defaults(root)
        self.assertEqual(defaults["lidar_fixed_frame"], "map")
        self.assertEqual(defaults["lidar_target_frame"], "lidar_link")

        sensors = root.find(
            "./include[@file='$(find morai_bringup)/launch/"
            "molit_2026_sensors.launch']"
        )
        self.assertIsNotNone(sensors)
        sensor_arguments = self.include_arguments(sensors)
        self.assertEqual(
            sensor_arguments["lidar_fixed_frame"],
            "$(arg lidar_fixed_frame)",
        )
        self.assertEqual(
            sensor_arguments["lidar_target_frame"],
            "$(arg lidar_target_frame)",
        )

    def test_legacy_integration_test_launch_is_removed(self):
        legacy_path = LAUNCH_DIR / "gps_localization_path_test.launch"
        self.assertFalse(legacy_path.exists())

    def test_visualization_bringup_entrypoints_wrap_visualization_package(
        self,
    ):
        for name in ("path", "lidar", "path_lidar"):
            root = self.load_launch("visualization/{}.launch".format(name))
            self.assertEqual(
                self.include_files(root),
                {
                    "$(find morai_visualization)/launch/{}.launch".format(
                        name
                    )
                },
            )


if __name__ == "__main__":
    unittest.main()
