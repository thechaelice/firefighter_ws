"""Launch the ESP32 <-> ROS 2 bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("firefighter_bridge"), "config", "bridge.yaml"
    )

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyAMA0",
                description="Serial device shared with the mobility ESP32.",
            ),
            DeclareLaunchArgument(
                "baud", default_value="115200", description="UART baud rate."
            ),
            Node(
                package="firefighter_bridge",
                executable="bridge_node",
                name="firefighter_bridge",
                output="screen",
                parameters=[config, {"port": port, "baud": baud}],
            ),
        ]
    )
