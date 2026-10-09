"""Run the firefighter stack against Gazebo (Harmonic) instead of the real robot.

Gazebo stands in for the hardware and publishes the same topics it does::

    gz DiffDrive            /cmd_vel -> wheels, -> /wheel_odom   (the ESP32 bridge)
    gz gpu_lidar            -> /scan                             (the RPLIDAR)
    gz thermal camera       -> /thermal/raw                      (the MLX90641)
    gz JointStatePublisher  -> /joint_states

Everything above the hardware - rf2o, the EKF, SLAM Toolbox, Nav2, the mission
FSM and RViz - is the unmodified ``bringup.launch.py`` with ``hardware:=false``,
so its arguments (``slam``, ``nav2_enabled``, ``rviz``, ...) work here too.

Thermal perception is the real node as well: it is pointed at
``sim_thermal_frames`` instead of the MLX90641 reader, which prints the Gazebo
camera's frames in the reader's format, so ``/thermal/image`` and
``/flame_event`` come out of the same detection code as on the robot.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("firefighter_description")
    bringup_share = get_package_share_directory("firefighter_bringup")
    default_world = os.path.join(description_share, "worlds", "firehouse.sdf")

    # Gazebo resolves package://firefighter_description/... against this path.
    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.dirname(description_share),
            os.pathsep,
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", LaunchConfiguration("world")],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "firefighter",
            "-topic", "/robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", "0.05",
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/thermal/raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "hardware": "false",
            "use_sim_time": "true",
            "model": os.path.join(description_share, "urdf", "firefighter.gazebo.xacro"),
            "thermal_reader_path": os.path.join(
                get_package_prefix("firefighter_perception"),
                "lib", "firefighter_perception", "sim_thermal_frames",
            ),
            "startup_delay": LaunchConfiguration("startup_delay"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world,
                                  description="Gazebo world (SDF) to load."),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("startup_delay", default_value="8.0",
                                  description="Seconds before SLAM/Nav2 start; "
                                              "Gazebo needs longer than the robot."),
            resource_path,
            gz_sim,
            spawn_robot,
            bridge,
            stack,
        ]
    )
