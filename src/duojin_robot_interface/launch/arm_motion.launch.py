"""Start the Duojin arm API without starting any vendor controller."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("duojin_robot_interface")
    config_path = os.path.join(package_share, "config", "arm_motion.yaml")
    execute = LaunchConfiguration("execute")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Explicitly allow validated arm goals to reach the SDK.",
            ),
            LogInfo(
                msg=[
                    "Duojin arm API starting with execute=",
                    execute,
                    ". false means PREVIEW ONLY; true permits real motion.",
                ]
            ),
            Node(
                package="duojin_robot_interface",
                executable="arm_motion_server",
                output="screen",
                parameters=[
                    config_path,
                    {"execute": ParameterValue(execute, value_type=bool)},
                ],
            ),
        ]
    )
