#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import re


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
    def direct_include_files(root):
        return [
            include.attrib["file"]
            for include in root.findall("./include")
        ]

    @staticmethod
    def argument_defaults(root):
        return {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("./arg")
        }

    @staticmethod
    def root_argument_pairs(root):
        return [
            (argument.attrib["name"], argument.attrib.get("default"))
            for argument in root.findall("./arg")
        ]

    @staticmethod
    def include_by_file(root, file_name):
        return root.find("./group/include[@file='{}']".format(file_name))

    @staticmethod
    def include_arguments(include):
        return {
            argument.attrib["name"]: argument.attrib["value"]
            for argument in include.findall("./arg")
        }

    @staticmethod
    def include_argument_pairs(include):
        return [
            (argument.attrib["name"], argument.attrib["value"])
            for argument in include.findall("./arg")
        ]

    def assert_exact_direct_include_files(self, root, expected):
        self.assertEqual(self.direct_include_files(root), expected)

    @staticmethod
    def workspace_launch_path(include_file):
        match = re.fullmatch(r"\$\(find ([^)]+)\)/launch/(.+)", include_file)
        if match is None:
            return None

        path = PACKAGE_ROOT.parent / match.group(1) / "launch" / match.group(2)
        return path if path.exists() else None

    def resolved_launch_roots(self, root):
        roots = []
        visited = set()

        def visit(current_root):
            for include in current_root.findall(".//include"):
                path = self.workspace_launch_path(include.attrib["file"])
                if path is None or path in visited:
                    continue
                visited.add(path)
                child_root = ET.parse(str(path)).getroot()
                roots.append(child_root)
                visit(child_root)

        roots.append(root)
        visit(root)
        return roots

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
        expected_includes = [
            "$(find morai_bringup)/launch/molit_2026_stack.launch",
            "$(find morai_udp_bridge)/launch/"
            "competition_vehicle_status_receiver.launch",
            "$(find morai_path_tracking)/launch/path_tracking.launch",
            "$(find morai_udp_bridge)/launch/control_sender.launch",
        ]
        self.assertEqual(
            self.root_argument_pairs(root),
            [
                ("publish_description", "true"),
                ("use_lidar", "true"),
                (
                    "bridge_config",
                    "$(find morai_udp_bridge)/config/molit_2026.yaml",
                ),
                (
                    "vehicle_config",
                    "$(find ioniq5_description)/config/vehicle_specs.yaml",
                ),
                (
                    "sensor_mount_config",
                    "$(find ioniq5_description)/config/"
                    "molit_2026_sensor_mounts.yaml",
                ),
                (
                    "localization_config",
                    "$(find morai_localization)/config/molit_2026_kcity.yaml",
                ),
                (
                    "route_path_config",
                    "$(find morai_path_manager)/config/"
                    "molit_2026_kcity_route_path.yaml",
                ),
                (
                    "global_path_file",
                    "$(find morai_path_manager)/map/R-KR_PG_K-City_2025/"
                    "2026_molit_comp_global_path.txt",
                ),
                ("lidar_device_ip", ""),
                ("lidar_port", "2368"),
                ("lidar_hz", "15.0"),
                ("lidar_frame", "lidar_link"),
                ("lidar_cut_angle", "0.0"),
                ("lidar_fixed_frame", ""),
                ("lidar_target_frame", ""),
                (
                    "vehicle_status_config",
                    "$(find morai_udp_bridge)/config/"
                    "molit_2026_vehicle_status.yaml",
                ),
                (
                    "controller_config",
                    "$(find morai_path_tracking)/config/"
                    "molit_2026_path_tracking.yaml",
                ),
                (
                    "control_sender_config",
                    "$(find morai_udp_bridge)/config/molit_2026_control.yaml",
                ),
            ],
        )

        self.assert_exact_direct_include_files(root, expected_includes)
        direct_includes = root.findall("./include")
        self.assertEqual(
            self.include_argument_pairs(direct_includes[0]),
            [
                ("publish_description", "$(arg publish_description)"),
                ("use_lidar", "$(arg use_lidar)"),
                ("bridge_config", "$(arg bridge_config)"),
                ("vehicle_config", "$(arg vehicle_config)"),
                ("sensor_mount_config", "$(arg sensor_mount_config)"),
                ("localization_config", "$(arg localization_config)"),
                ("route_path_config", "$(arg route_path_config)"),
                ("global_path_file", "$(arg global_path_file)"),
                ("lidar_device_ip", "$(arg lidar_device_ip)"),
                ("lidar_port", "$(arg lidar_port)"),
                ("lidar_hz", "$(arg lidar_hz)"),
                ("lidar_frame", "$(arg lidar_frame)"),
                ("lidar_cut_angle", "$(arg lidar_cut_angle)"),
                ("lidar_fixed_frame", "$(arg lidar_fixed_frame)"),
                ("lidar_target_frame", "$(arg lidar_target_frame)"),
            ],
        )
        self.assertEqual(
            self.include_argument_pairs(direct_includes[1]),
            [("config", "$(arg vehicle_status_config)")],
        )
        self.assertEqual(
            self.include_argument_pairs(direct_includes[2]),
            [("config", "$(arg controller_config)")],
        )
        self.assertEqual(
            self.include_argument_pairs(direct_includes[3]),
            [("config", "$(arg control_sender_config)")],
        )

        for resolved_root in self.resolved_launch_roots(root):
            for include in resolved_root.findall(".//include"):
                self.assertNotRegex(
                    include.attrib["file"].lower(),
                    r"rviz|vehicle_control|manual",
                )
            for node in resolved_root.findall(".//node"):
                self.assertNotRegex(
                    " ".join(node.attrib.values()).lower(),
                    r"rviz|vehicle_control|manual",
                )

    def test_exact_launch_helpers_do_not_hide_duplicate_elements(self):
        duplicate_include_root = ET.fromstring(
            "<launch><include file='stack.launch'/><include "
            "file='stack.launch'/></launch>"
        )
        self.assertEqual(self.include_files(duplicate_include_root), {"stack.launch"})
        with self.assertRaises(AssertionError):
            self.assert_exact_direct_include_files(
                duplicate_include_root,
                ["stack.launch"],
            )

        duplicate_argument_include = ET.fromstring(
            "<include><arg name='config' value='config.yaml'/><arg "
            "name='config' value='config.yaml'/></include>"
        )
        self.assertEqual(
            self.include_arguments(duplicate_argument_include),
            {"config": "config.yaml"},
        )
        self.assertEqual(
            self.include_argument_pairs(duplicate_argument_include),
            [
                ("config", "config.yaml"),
                ("config", "config.yaml"),
            ],
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
