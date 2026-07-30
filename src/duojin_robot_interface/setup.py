from glob import glob
import os

from setuptools import find_packages, setup


package_name = "duojin_robot_interface"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duojin Team",
    maintainer_email="duojin@example.com",
    description="Safe ROS 2 adapter and Python API for R1 Lite arms.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm = duojin_robot_interface.arm_cli:main",
            "arm_motion_server = duojin_robot_interface.arm_motion_server:main",
            "arm_pose_display = duojin_robot_interface.arm_pose_display:main",
        ],
    },
)
