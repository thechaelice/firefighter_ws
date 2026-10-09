"""Publish the firefighter robot description and its TF tree.

``robot_state_publisher`` broadcasts the fixed sensor mounts (``base_link ->
laser`` / ``thermal_camera``). On the real robot the wheel and caster joints
have no encoders feeding ``/joint_states``, so ``joint_state_publisher`` holds
them at zero; in simulation Gazebo publishes them instead
(``publish_joint_states:=false``).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_MODEL = os.path.join(
    get_package_share_directory("firefighter_description"),
    "urdf", "firefighter.urdf.xacro",
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("model", default_value=DEFAULT_MODEL,
                                  description="XACRO file to publish."),
            DeclareLaunchArgument("publish_joint_states", default_value="true",
                                  description="false when something else (Gazebo) "
                                              "publishes /joint_states."),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description,
                             "use_sim_time": use_sim_time}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("publish_joint_states")),
            ),
        ]
    )
