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

    def test_full_stack_does_not_require_map_tf_during_lidar_startup(self):
        root = self.load_launch("molit_2026_stack.launch")
        defaults = self.argument_defaults(root)
        self.assertEqual(defaults["lidar_fixed_frame"], "")
        self.assertEqual(defaults["lidar_target_frame"], "")

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

    def test_autonomous_bringup_composes_tracking_without_manual_control(self):
        root = self.load_launch("molit_2026_autonomous.launch")
        expected = {
            "$(find morai_bringup)/launch/molit_2026_stack.launch",
            "$(find morai_path_tracking)/launch/pure_pursuit.launch",
            "$(find morai_udp_bridge)/launch/control_sender.launch",
        }
        self.assertEqual(self.include_files(root), expected)
        self.assertNotIn(
            "vehicle_control", ET.tostring(root, encoding="unicode")
        )

        defaults = self.argument_defaults(root)
        self.assertEqual(
            defaults["localization_config"],
            "$(find morai_localization)/config/molit_2026_kcity.yaml",
        )
        self.assertEqual(
            defaults["route_path_config"],
            "$(find morai_path_manager)/config/"
            "molit_2026_kcity_route_path.yaml",
        )
        self.assertEqual(
            defaults["global_path_file"],
            "$(find morai_path_manager)/map/R-KR_PG_K-City_2025/"
            "2026_molit_comp_global_path.txt",
        )
        self.assertEqual(
            defaults["controller_config"],
            "$(find morai_path_tracking)/config/"
            "molit_2026_pure_pursuit.yaml",
        )
        self.assertEqual(
            defaults["control_sender_config"],
            "$(find morai_udp_bridge)/config/molit_2026_control.yaml",
        )

        stack = root.find(
            "./include[@file='$(find morai_bringup)/launch/"
            "molit_2026_stack.launch']"
        )
        self.assertIsNotNone(stack)
        self.assertEqual(
            self.include_arguments(stack),
            {
                "publish_description": "$(arg publish_description)",
                "use_lidar": "$(arg use_lidar)",
                "bridge_config": "$(arg bridge_config)",
                "vehicle_config": "$(arg vehicle_config)",
                "sensor_mount_config": "$(arg sensor_mount_config)",
                "localization_config": "$(arg localization_config)",
                "route_path_config": "$(arg route_path_config)",
                "global_path_file": "$(arg global_path_file)",
                "lidar_device_ip": "$(arg lidar_device_ip)",
                "lidar_port": "$(arg lidar_port)",
                "lidar_hz": "$(arg lidar_hz)",
                "lidar_frame": "$(arg lidar_frame)",
                "lidar_cut_angle": "$(arg lidar_cut_angle)",
                "lidar_fixed_frame": "$(arg lidar_fixed_frame)",
                "lidar_target_frame": "$(arg lidar_target_frame)",
            },
        )

        controller = root.find(
            "./include[@file='$(find morai_path_tracking)/launch/"
            "pure_pursuit.launch']"
        )
        self.assertIsNotNone(controller)
        self.assertEqual(
            self.include_arguments(controller),
            {"config": "$(arg controller_config)"},
        )

        sender = root.find(
            "./include[@file='$(find morai_udp_bridge)/launch/"
            "control_sender.launch']"
        )
        self.assertIsNotNone(sender)
        self.assertEqual(
            self.include_arguments(sender),
            {"config": "$(arg control_sender_config)"},
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
