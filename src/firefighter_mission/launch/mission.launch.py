"""Launch the mission behaviour node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("firefighter_mission"), "config", "mission.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="firefighter_mission",
                executable="mission_node",
                name="firefighter_mission",
                output="screen",
                parameters=[config],
            )
        ]
    )
