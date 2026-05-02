from __future__ import annotations

import xml.etree.ElementTree as ET


WHEEL_JOINTS = ("zq_Joint", "zh_Joint", "yq_Joint", "yh_Joint")


def inject_ros2_control_block(
    urdf_xml: str,
    *,
    controllers_yaml: str | None = None,
    robot_param_node: str = "robot_state_publisher",
    robot_param: str = "robot_description",
    controller_manager_name: str = "controller_manager",
    enable_lidar: bool = False,
    lidar_parent_link: str = "radar_Link",
    lidar_frame: str = "laser_frame",
    lidar_topic: str = "/scan",
) -> str:
    """
    Inject a minimal <ros2_control> block into a CAD-export URDF.

    Why:
      - The SolidWorks-exported URDF is not xacro-friendly to wrap cleanly.
      - We want to keep the description package pristine so future CAD exports
        don't create merge pain.

    This function adds:
      - one system hardware plugin for Gazebo Fortress: gz_ros2_control/GazeboSimSystem
      - velocity command + position/velocity state interfaces for the 4 wheel joints

    If controllers_yaml is provided, it also injects a Gazebo plugin block so
    gz_ros2_control is loaded at spawn time (avoids runtime "attach plugin"
    service calls which are flaky on some setups).
    """
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError(f"Expected <robot> root, got <{root.tag}>")

    # If already present, keep the input as-is.
    if root.find("ros2_control") is None:
        ros2_control = ET.Element("ros2_control", attrib={"name": "YahboomCarSystem", "type": "system"})

        hardware = ET.SubElement(ros2_control, "hardware")
        plugin = ET.SubElement(hardware, "plugin")
        plugin.text = "gz_ros2_control/GazeboSimSystem"

        for jname in WHEEL_JOINTS:
            j = ET.SubElement(ros2_control, "joint", attrib={"name": jname})
            ET.SubElement(j, "command_interface", attrib={"name": "velocity"})
            ET.SubElement(j, "state_interface", attrib={"name": "position"})
            ET.SubElement(j, "state_interface", attrib={"name": "velocity"})

        # Keep it near the end for readability when debugging robot_description.
        root.append(ros2_control)

    if controllers_yaml:
        # Gazebo (Ignition/Fortress) consumes URDF via an internal URDF->SDF conversion.
        # The converter honors <gazebo> plugin tags and will place them as SDF model plugins.
        existing = [
            g
            for g in root.findall("gazebo")
            for p in g.findall("plugin")
            if p.get("filename") == "libgz_ros2_control-system.so"
        ]
        if not existing:
            gazebo = ET.Element("gazebo")
            plugin = ET.SubElement(
                gazebo,
                "plugin",
                attrib={
                    "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
                    "filename": "libgz_ros2_control-system.so",
                },
            )
            # These tags are read from the SDF plugin element.
            ET.SubElement(plugin, "namespace").text = "/"
            ET.SubElement(plugin, "parameters").text = controllers_yaml
            ET.SubElement(plugin, "robot_param_node").text = robot_param_node
            ET.SubElement(plugin, "robot_param").text = robot_param
            ET.SubElement(plugin, "controller_manager_name").text = controller_manager_name
            root.append(gazebo)

    if enable_lidar:
        _inject_lidar(root, parent_link=lidar_parent_link, lidar_frame=lidar_frame, topic=lidar_topic)

    # ElementTree doesn't preserve formatting; that's OK because this is runtime-only.
    return ET.tostring(root, encoding="unicode")


def _inject_lidar(root: ET.Element, *, parent_link: str, lidar_frame: str, topic: str) -> None:
    # Create a dedicated frame for the sensor so the TF is explicit and stable.
    if root.find(f"./link[@name='{lidar_frame}']") is None:
        root.append(ET.Element("link", attrib={"name": lidar_frame}))

    joint_name = f"{lidar_frame}_joint"
    if root.find(f"./joint[@name='{joint_name}']") is None:
        joint = ET.Element("joint", attrib={"name": joint_name, "type": "fixed"})
        ET.SubElement(joint, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(joint, "parent", attrib={"link": parent_link})
        ET.SubElement(joint, "child", attrib={"link": lidar_frame})
        root.append(joint)

    # Avoid duplicating the sensor if this patch runs multiple times.
    for gz in root.findall("gazebo"):
        if gz.get("reference") != lidar_frame:
            continue
        for sensor in gz.findall("sensor"):
            if sensor.get("name") == "front_lidar":
                return

    gazebo = ET.Element("gazebo", attrib={"reference": lidar_frame})
    sensor = ET.SubElement(gazebo, "sensor", attrib={"name": "front_lidar", "type": "lidar"})
    ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"
    ET.SubElement(sensor, "visualize").text = "false"
    ET.SubElement(sensor, "update_rate").text = "10"

    lidar = ET.SubElement(sensor, "lidar")
    scan = ET.SubElement(lidar, "scan")
    horiz = ET.SubElement(scan, "horizontal")
    ET.SubElement(horiz, "samples").text = "360"
    ET.SubElement(horiz, "min_angle").text = "-3.14159"
    ET.SubElement(horiz, "max_angle").text = "3.14159"
    rng = ET.SubElement(lidar, "range")
    ET.SubElement(rng, "min").text = "0.3"
    ET.SubElement(rng, "max").text = "12"

    ET.SubElement(sensor, "topic").text = topic
    ET.SubElement(sensor, "gz_frame_id").text = lidar_frame

    root.append(gazebo)
