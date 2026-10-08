"""Launch the mission behaviour node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("firefighter_mission"), "config", "mission.yaml"
    )

    use_nav2 = ParameterValue(LaunchConfiguration("use_nav2"), value_type=bool)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_nav2",
                default_value="true",
                description="false = publish the goal only and wait for a manual "
                            "nav_goal_reached event.",
            ),
            Node(
                package="firefighter_mission",
                executable="mission_node",
                name="firefighter_mission",
                output="screen",
                parameters=[config, {"use_nav2": use_nav2}],
            ),
        ]
    )
