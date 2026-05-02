from __future__ import annotations

import argparse
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ros_gz_interfaces.srv import SpawnEntity

from yahboomcar_control_bringup.urdf_patch import inject_ros2_control_block


def _read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _rewrite_package_uris(urdf_xml: str, description_pkg: str) -> str:
    share_dir = get_package_share_directory(description_pkg)
    # `file://` + `/abs/path` => `file:///abs/path` (valid file URI).
    return urdf_xml.replace(f"package://{description_pkg}/", f"file://{share_dir}/")


class EntitySpawner(Node):
    def __init__(
        self,
        *,
        service_name: str,
        xml: str,
        entity_name: str,
        pose: Pose,
        allow_renaming: bool,
        request_timeout_s: float,
        retries: int,
        retry_sleep_s: float,
    ) -> None:
        super().__init__("spawn_entity")
        self._client = self.create_client(SpawnEntity, service_name)
        self._xml = xml
        self._entity_name = entity_name
        self._pose = pose
        self._allow_renaming = allow_renaming
        self._request_timeout_s = request_timeout_s
        self._retries = retries
        self._retry_sleep_s = retry_sleep_s

    def run(self) -> bool:
        self.get_logger().info(f"Waiting for service {self._client.srv_name} ...")
        if not self._client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Spawn service not available after 30s.")
            return False

        req = SpawnEntity.Request()
        req.entity_factory.name = self._entity_name
        req.entity_factory.allow_renaming = self._allow_renaming
        req.entity_factory.sdf = self._xml  # ros_gz_sim accepts URDF here and converts internally.
        req.entity_factory.pose = self._pose

        for attempt in range(1, self._retries + 1):
            self.get_logger().info(
                f"Spawning '{self._entity_name}' (attempt {attempt}/{self._retries}) ..."
            )
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._request_timeout_s)

            if future.done():
                try:
                    resp = future.result()
                except Exception as e:  # pylint: disable=broad-except
                    self.get_logger().warn(f"Spawn call failed: {e}")
                else:
                    if resp.success:
                        self.get_logger().info("Spawn succeeded.")
                        return True
                    self.get_logger().warn("Spawn service returned success=false.")
            else:
                self.get_logger().warn("Spawn call timed out waiting for response.")

            if attempt < self._retries:
                time.sleep(self._retry_sleep_s)

        self.get_logger().error("All spawn attempts failed.")
        return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="default")
    parser.add_argument("--name", default="yahboomcar")
    parser.add_argument("--urdf", required=True, help="Absolute path to URDF file.")
    parser.add_argument("--description-pkg", default="yahboomcar_description")
    parser.add_argument("--z", type=float, default=0.30)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--allow-renaming", action="store_true")
    parser.add_argument("--request-timeout", type=float, default=15.0)
    parser.add_argument("--retries", type=int, default=6)
    parser.add_argument("--retry-sleep", type=float, default=1.0)
    parser.add_argument("--inject-ros2-control", action="store_true")
    parser.add_argument("--controllers-yaml", default="")
    parser.add_argument("--rewrite-package-uris", action="store_true")
    parser.add_argument("--enable-lidar", action="store_true")
    parser.add_argument("--lidar-parent-link", default="radar_Link")
    parser.add_argument("--lidar-frame", default="laser_frame")
    parser.add_argument("--lidar-topic", default="/scan")
    # `ros2 launch` appends ROS-specific args (e.g. `--ros-args --params-file ...`).
    # Strip those before argparse parsing, but keep the original argv for `rclpy.init`
    # so ROS parameters (like `use_sim_time`) still load.
    if argv is None:
        argv_in = sys.argv
    else:
        argv_in = [sys.argv[0], *argv]

    non_ros_argv = remove_ros_args(argv_in)
    args, _unknown = parser.parse_known_args(non_ros_argv[1:])

    xml = _read_file(args.urdf)
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

    pose = Pose()
    pose.position.x = float(args.x)
    pose.position.y = float(args.y)
    pose.position.z = float(args.z)
    pose.orientation.w = 1.0

    rclpy.init(args=argv_in)
    node = EntitySpawner(
        service_name=f"/world/{args.world}/create",
        xml=xml,
        entity_name=args.name,
        pose=pose,
        allow_renaming=bool(args.allow_renaming),
        request_timeout_s=float(args.request_timeout),
        retries=int(args.retries),
        retry_sleep_s=float(args.retry_sleep),
    )
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
