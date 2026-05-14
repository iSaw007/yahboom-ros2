from glob import glob

from setuptools import find_packages, setup

package_name = "yahboomcar_control_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="salah",
    maintainer_email="you@example.com",
    description=(
        "Bringup wrapper that adds ros2_control to the Yahboom CAD-exported URDF "
        "without modifying it on disk."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_description_publisher = yahboomcar_control_bringup.robot_description_publisher:main",
            "spawn_entity = yahboomcar_control_bringup.spawn_entity:main",
            "head_teleop = yahboomcar_control_bringup.head_teleop:main",
        ]
    },
)
