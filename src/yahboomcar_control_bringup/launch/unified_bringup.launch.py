import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_share = get_package_share_directory("yahboomcar_control_bringup")
    
    # Declare arguments
    sim_arg = DeclareLaunchArgument(
        "sim", default_value="true", description="Launch Gazebo simulation"
    )
    nav_arg = DeclareLaunchArgument(
        "nav", default_value="true", description="Launch Nav2 navigation stack"
    )
    dock_arg = DeclareLaunchArgument(
        "dock", default_value="true", description="Launch Docking mission nodes"
    )
    rosbridge_arg = DeclareLaunchArgument(
        "rosbridge", default_value="true", description="Launch rosbridge WebSocket server"
    )
    video_server_arg = DeclareLaunchArgument(
        "video_server", default_value="true", description="Launch web_video_server image streamer"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false", description="Run Gazebo headless"
    )
    
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/salah/Storage/ros2-test/mapsave1.yaml",
        description="Path to map yaml file"
    )
    nav2_params_arg = DeclareLaunchArgument(
        "nav2_params",
        default_value=os.path.join(bringup_share, "config", "nav2_params.yaml"),
        description="Path to Nav2 params yaml file"
    )

    # 1. Simulation Bringup
    sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim_control.launch.py")),
        condition=IfCondition(LaunchConfiguration("sim")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "headless": LaunchConfiguration("headless"),
        }.items()
    )

    # 2. Rosbridge WebSocket server
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            )
        ),
        condition=IfCondition(LaunchConfiguration("rosbridge"))
    )

    # 3. Web Video Server
    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("video_server")),
        parameters=[{"port": 8080}]
    )

    # 4. Navigation Stack (Nav2)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav2_unified.launch.py")),
        condition=IfCondition(LaunchConfiguration("nav")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("nav2_params"),
        }.items()
    )

    # 5. Docking Mission Nodes
    dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "dock_mission.launch.py")),
        condition=IfCondition(LaunchConfiguration("dock")),
    )

    return LaunchDescription([
        sim_arg,
        nav_arg,
        dock_arg,
        rosbridge_arg,
        video_server_arg,
        use_sim_time_arg,
        headless_arg,
        map_arg,
        nav2_params_arg,
        sim_control,
        rosbridge,
        web_video_server,
        nav2,
        dock
    ])
