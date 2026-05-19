from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node

from yahboomcar_control_bringup.urdf_patch import inject_ros2_control_block


def _read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_config = LaunchConfiguration("world")

    bringup_share = get_package_share_directory("yahboomcar_control_bringup")
    desc_share = get_package_share_directory("yahboomcar_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    gz_ros2_control_prefix = get_package_prefix("gz_ros2_control")
    gz_ros2_control_libdir = os.path.join(gz_ros2_control_prefix, "lib")

    # Gazebo resolves `model://<name>/...` by searching resource paths for `<name>/...`.
    # Adding `<prefix>/share` makes `model://yahboomcar_description/...` resolve to
    # `<prefix>/share/yahboomcar_description/...` (where your meshes live).
    desc_prefix_share = os.path.dirname(desc_share)

    default_world_path = os.path.join(bringup_share, "worlds", "empty.sdf")
    urdf_path = os.path.join(desc_share, "urdf", "MicroROS.urdf")
    controllers_path = os.path.join(bringup_share, "config", "controllers.yaml")
    ekf_config_path = os.path.join(bringup_share, "config", "ekf.yaml")

    # Runtime-only injection: keeps the CAD-export URDF untouched on disk.
    # We also rewrite mesh URIs to absolute file:// paths because Gazebo Fortress doesn't
    # understand ROS `package://...` URIs.
    robot_description = inject_ros2_control_block(
        _read_file(urdf_path),
        controllers_yaml=controllers_path,
        enable_lidar=True,
        lidar_parent_link="radar_Link",
        lidar_frame="laser_frame",
        lidar_topic="/scan",
        enable_imu=True,
        enable_base_footprint=True,
        enable_camera=True,
    )
    robot_description = robot_description.replace(
        "package://yahboomcar_description/",
        f"file://{desc_share}/",
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            # -r: run, -v 4: verbose enough to debug, but not insane.
            "gz_args": [
                TextSubstitution(text="-r "),
                PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
                TextSubstitution(text="-v 2 "),
                world_config,
            ],
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
        output="screen",
    )

    # Spawn the URDF into Gazebo via the official create node.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "yahboomcar",
            "-topic",
            "robot_description",
            "-z",
            "0.30",
        ],
        output="screen",
    )


    # Keep the TF tree standard for mobile bases.
    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    camera_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["camera_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Start ros2_control spawners only after the model creation process has exited.
    # The LiDAR-enabled model takes longer to spawn, so a fixed timer can race the
    # controller_manager startup and leave the spawners waiting forever.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": use_sim_time}],
    )

    controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_spawner, diff_drive_spawner, camera_spawner],
                )
            ],
        )
    )

    # Bridge /cmd_vel (standard) to /diff_drive_controller/cmd_vel_unstamped (Humble default)
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_relay",
        arguments=["/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world_path),
            DeclareLaunchArgument("headless", default_value="false"),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    TextSubstitution(text=desc_prefix_share),
                    TextSubstitution(text=":"),
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    TextSubstitution(text=desc_prefix_share),
                    TextSubstitution(text=":"),
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            # Ensure Gazebo can locate the ros2_control system plugin installed under the ROS prefix.
            SetEnvironmentVariable(
                name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                value=[
                    TextSubstitution(text=gz_ros2_control_libdir),
                    TextSubstitution(text=":"),
                    EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=[
                    TextSubstitution(text=gz_ros2_control_libdir),
                    TextSubstitution(text=":"),
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            gz,
            clock_bridge,
            robot_state_publisher,
            spawn,
            ekf_node,
            controller_spawners,
            cmd_vel_relay,
        ]
    )
