import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yahboomcar_control_bringup')
    
    # Define path to the static dock configuration
    dock_config = os.path.join(pkg_dir, 'config', 'dock.yaml')
    
    # ── Nodes ─────────────────────────────────────────────────────────────────
    
    # 1. The Eye: Detects the ArUco marker and publishes pixel coordinates
    detector_node = Node(
        package='yahboomcar_control_bringup',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[dock_config]
    )

    # 2. The Brain: Action Server for Staged Docking
    coordinator_node = Node(
        package='yahboomcar_control_bringup',
        executable='dock_coordinator',
        name='dock_coordinator',
        output='screen',
        parameters=[dock_config]
    )

    return LaunchDescription([
        detector_node,
        coordinator_node
    ])
