from __future__ import annotations

import xml.etree.ElementTree as ET


WHEEL_JOINTS = ("zq_Joint", "zh_Joint", "yq_Joint", "yh_Joint")
HEAD_JOINTS = ("jq1_Joint", "jq2_Joint")


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
    enable_imu: bool = False,
    imu_link: str = "imu_Link",
    imu_topic: str = "/imu",
    enable_camera: bool = False,
    camera_link: str = "jq2_Link",
    camera_topic: str = "/camera/image_raw",
    enable_base_footprint: bool = True,
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

        for jname in HEAD_JOINTS:
            j = ET.SubElement(ros2_control, "joint", attrib={"name": jname})
            ET.SubElement(j, "command_interface", attrib={"name": "position"})
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

    if enable_imu:
        _inject_imu(root, reference_link=imu_link, topic=imu_topic)

    if enable_camera:
        _inject_camera(root, reference_link=camera_link, topic=camera_topic)

    if enable_base_footprint:
        _inject_base_footprint(root)
        _boost_base_mass(root, target_mass=1.0)
        _simplify_collisions(root)
        _fix_head_joint_limits(root)

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
    sensor = ET.SubElement(gazebo, "sensor", attrib={"name": "front_lidar", "type": "gpu_lidar"})
    ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"
    ET.SubElement(sensor, "visualize").text = "true"
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = "20"

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


def _inject_imu(root: ET.Element, *, reference_link: str, topic: str) -> None:
    """
    Attach an Ignition IMU sensor to the specified link.
    """
    # Avoid duplicate sensors if patched multiple times
    for gz in root.findall("gazebo"):
        if gz.get("reference") != reference_link:
            continue
        for sensor in gz.findall("sensor"):
            if sensor.get("type") == "imu":
                return

    gazebo = ET.Element("gazebo", attrib={"reference": reference_link})
    sensor = ET.SubElement(gazebo, "sensor", attrib={"name": "imu_sensor", "type": "imu"})
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = "50"
    ET.SubElement(sensor, "visualize").text = "true"
    ET.SubElement(sensor, "topic").text = topic
    ET.SubElement(sensor, "gz_frame_id").text = reference_link

    root.append(gazebo)


def _inject_base_footprint(root: ET.Element) -> None:
    """
    Ensure base_footprint exists and is the parent of base_link.
    This fixes the 'root link inertia' warning and stabilizes physics.
    """
    if root.find("./link[@name='base_footprint']") is not None:
        return

    # Create footprint (no mass, purely for TF/Root purposes)
    footprint = ET.Element("link", attrib={"name": "base_footprint"})
    root.insert(0, footprint)

    # Move base_link to be a child of base_footprint
    joint = ET.Element("joint", attrib={"name": "base_footprint_joint", "type": "fixed"})
    ET.SubElement(joint, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", attrib={"link": "base_footprint"})
    ET.SubElement(joint, "child", attrib={"link": "base_link"})
    root.insert(1, joint)


def _boost_base_mass(root: ET.Element, target_mass: float = 1.0) -> None:
    """
    Override the mass of base_link to provide more physical 'heft'.
    """
    base_link = root.find("./link[@name='base_link']")
    if base_link is None:
        return

    inertial = base_link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(base_link, "inertial")

    mass_elem = inertial.find("mass")
    if mass_elem is None:
        mass_elem = ET.SubElement(inertial, "mass")

    current_mass = float(mass_elem.get("value", "0.1"))
    scale = target_mass / current_mass

    mass_elem.set("value", str(target_mass))

    inertia = inertial.find("inertia")
    if inertia is not None:
        for attr in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            val = float(inertia.get(attr, "0.0"))
            inertia.set(attr, str(val * scale))


def _simplify_collisions(root: ET.Element) -> None:
    """
    Replace complex mesh collisions with a simple box for high-performance physics.
    """
    base_link = root.find("./link[@name='base_link']")
    if base_link is None:
        return

    # Remove all existing collisions (typically heavy meshes)
    for col in base_link.findall("collision"):
        base_link.remove(col)

    # Add a simple box collision roughly matching the car's body
    collision = ET.SubElement(base_link, "collision", attrib={"name": "base_collision"})
    # Center it slightly above the footprint so it covers the chassis
    ET.SubElement(collision, "origin", attrib={"xyz": "0 0 0.05", "rpy": "0 0 0"})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", attrib={"size": "0.25 0.18 0.1"})


def _fix_head_joint_limits(root: ET.Element) -> None:
    """
    Override zero-effort/velocity limits on revolute joints to allow motion.
    """
    for joint in root.findall(".//joint"):
        name = joint.get("name")
        if name in HEAD_JOINTS:
            limit = joint.find("limit")
            if limit is not None:
                # Give it enough 'muscles' to hold up a camera
                limit.set("effort", "10.0")
                limit.set("velocity", "1.0")


def _inject_camera(root: ET.Element, *, reference_link: str, topic: str) -> None:
    """
    Attach an Ignition RGBD camera sensor to the specified link.
    """
    # Avoid duplicate sensors if patched multiple times
    for gz in root.findall("gazebo"):
        if gz.get("reference") != reference_link:
            continue
        for sensor in gz.findall("sensor"):
            if sensor.get("type") == "rgbd":
                return

    gazebo = ET.Element("gazebo", attrib={"reference": reference_link})
    sensor = ET.SubElement(gazebo, "sensor", attrib={"name": "camera_sensor", "type": "camera"})
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = "30"
    ET.SubElement(sensor, "visualize").text = "true"
    ET.SubElement(sensor, "topic").text = topic
    ET.SubElement(sensor, "gz_frame_id").text = reference_link
    ET.SubElement(sensor, "pose").text = "0.05 0 0 0 0 0"

    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = "1.047"  # ~60 degrees
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = "640"
    ET.SubElement(image, "height").text = "480"
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.05"
    ET.SubElement(clip, "far").text = "8.0"

    root.append(gazebo)
