from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

from yahboomcar_control_bringup.urdf_patch import inject_ros2_control_block


class RobotDescriptionPublisher(Node):
    def __init__(self, xml: str, topic: str) -> None:
        super().__init__("robot_description_publisher")

        # Transient-local makes this behave like a "latched" topic, so late-joining
        # subscribers (e.g. ros_gz_sim create) still get the URDF.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._pub = self.create_publisher(String, topic, qos)
        self._msg = String(data=xml)

        # Publish a few times to be resilient to startup races.
        self._count = 0
        self._timer = self.create_timer(0.2, self._tick)

    def _tick(self) -> None:
        self._pub.publish(self._msg)
        self._count += 1
        if self._count >= 5:
            self.get_logger().info("Published robot_description (x5); exiting.")
            rclpy.shutdown()


def _rewrite_package_uris(urdf_xml: str, description_pkg: str) -> str:
    share_dir = get_package_share_directory(description_pkg)
    file_prefix = f"file://{share_dir}/"
    return urdf_xml.replace(f"package://{description_pkg}/", file_prefix)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="robot_description")
    parser.add_argument("--urdf", required=True, help="Absolute path to URDF file to publish.")
    parser.add_argument(
        "--historical-cli",
        action="store_true",
        help=(
            "Preserve the April 29 CLI behavior by parsing launch-appended ROS args as plain "
            "argv. This intentionally reproduces the historical exit-code-2 failure mode."
        ),
    )
    parser.add_argument(
        "--inject-ros2-control",
        action="store_true",
        help="Inject a minimal <ros2_control> block before publishing.",
    )
    parser.add_argument(
        "--rewrite-package-uris",
        action="store_true",
        help=(
            "Rewrite package://<pkg>/... mesh URIs to absolute file://... paths. "
            "This is needed for Gazebo, which does not resolve ROS package URIs."
        ),
    )
    parser.add_argument(
        "--description-pkg",
        default="yahboomcar_description",
        help="ROS package that contains meshes referenced by the URDF (default: yahboomcar_description).",
    )
    parser.add_argument(
        "--controllers-yaml",
        default="",
        help=(
            "Absolute path to controllers.yaml. If set, also injects a Gazebo plugin tag so "
            "gz_ros2_control starts controller_manager at spawn time."
        ),
    )
    parser.add_argument("--enable-lidar", action="store_true")
    parser.add_argument("--lidar-parent-link", default="radar_Link")
    parser.add_argument("--lidar-frame", default="laser_frame")
    parser.add_argument("--lidar-topic", default="/scan")

    if argv is None:
        argv_in = sys.argv
    else:
        argv_in = [sys.argv[0], *argv]

    if "--historical-cli" in argv_in:
        # Intentionally keep the original behavior so the diagnostic launch can
        # reproduce the April 29 failure mode under `ros2 launch`.
        args = parser.parse_args(argv_in[1:])
    else:
        non_ros_argv = remove_ros_args(argv_in)
        args, _unknown = parser.parse_known_args(non_ros_argv[1:])

    try:
        with open(args.urdf, "r", encoding="utf-8") as f:
            xml = f.read()
    except OSError as e:
        print(f"Failed to read URDF: {e}", file=sys.stderr)
        return 2

    if args.inject_ros2_control:
        xml = inject_ros2_control_block(
            xml,
            controllers_yaml=(args.controllers_yaml or None),
            robot_param_node="robot_state_publisher",
            robot_param="robot_description",
            controller_manager_name="controller_manager",
            enable_lidar=args.enable_lidar,
            lidar_parent_link=args.lidar_parent_link,
            lidar_frame=args.lidar_frame,
            lidar_topic=args.lidar_topic,
        )

    if args.rewrite_package_uris:
        xml = _rewrite_package_uris(xml, args.description_pkg)

    rclpy.init(args=argv_in)
    node = RobotDescriptionPublisher(xml=xml, topic=args.topic)
    rclpy.spin(node)
    return 0
