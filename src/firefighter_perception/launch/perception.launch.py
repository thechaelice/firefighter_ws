"""Launch the thermal perception node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The MLX90641 reader is a standalone C tool that lives in the repo, not in an
# install space, so its path has to be supplied. Override with reader_path:=...
DEFAULT_READER = os.path.expanduser("~/firefighter_ws/scripts/thermal/mlx90641_frames")


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("firefighter_perception"), "config", "perception.yaml"
    )

    reader_path = LaunchConfiguration("reader_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "reader_path",
                default_value=DEFAULT_READER,
                description="Absolute path to the mlx90641_frames reader executable.",
            ),
            Node(
                package="firefighter_perception",
                executable="thermal_node",
                name="firefighter_perception",
                output="screen",
                parameters=[config, {"reader_path": reader_path}],
            ),
        ]
    )
