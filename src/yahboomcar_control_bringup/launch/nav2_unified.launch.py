import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Path to our custom params
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    quiet = LaunchConfiguration('quiet')
    node_output = PythonExpression(["'log' if '", quiet, "' == 'true' else 'screen'"])

    # Names of nodes to manage in order. 
    # Pruned to match typical Yahboom / Nav2 Simple configs
    lifecycle_nodes = [
        'map_server', 
        'amcl', 
        'planner_server', 
        'controller_server', 
        'behavior_server', 
        'bt_navigator', 
        'waypoint_follower'
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('quiet', default_value='false'),

        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output=node_output,
            parameters=[params_file, {'yaml_filename': map_yaml_file}]
        ),

        # 2. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output=node_output,
            parameters=[params_file]
        ),

        # 3. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output=node_output,
            parameters=[params_file]
        ),

        # 4. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output=node_output,
            parameters=[params_file]
        ),

        # 5. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output=node_output,
            parameters=[params_file]
        ),

        # 6. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output=node_output,
            parameters=[params_file]
        ),

        # 7. Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output=node_output,
            parameters=[params_file]
        ),

        # THE MASTER MANAGER (Renamed to 'lifecycle_manager_navigation' for RViz)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output=node_output,
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': lifecycle_nodes}]
        ),
    ])
