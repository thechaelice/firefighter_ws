"""Single entry point for the whole firefighter robot stack.

Brings the robot up in dependency order::

    model      firefighter_description              -> /robot_description, base_link -> laser
    sensing    sllidar                              -> /scan
    odometry   rf2o  +  robot_localization EKF      -> /odom_rf2o, odom -> base_link
    robot      firefighter_bridge (ESP32 UART)      -> /wheel_odom, /beacon_event
               firefighter_perception (MLX90641)    -> /thermal/image, /flame_event
    autonomy   slam_toolbox                         -> map -> odom
               nav2_bringup navigation              -> NavigateToPose action
               firefighter_mission                  -> behaviour FSM
    operator   rviz2

``hardware:=false`` skips the nodes that talk to real devices (sllidar and the
ESP32 bridge); ``sim.launch.py`` uses it and lets Gazebo supply ``/scan``,
``/wheel_odom`` and ``/joint_states`` instead. Thermal perception always runs -
in simulation ``thermal_reader_path`` points it at a simulated frame source.

The top-level ``config/`` and ``maps/`` directories live in the workspace, not in
any package's share directory, so they are located through the ``FIREFIGHTER_WS``
environment variable (default ``~/firefighter_ws``). Set it if the workspace moves.
"""
import os
from pathlib import Path
from typing import Dict

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

WORKSPACE = Path(os.environ.get("FIREFIGHTER_WS", "~/firefighter_ws")).expanduser()
DEFAULT_CONFIG_DIR = str(WORKSPACE / "config")
DEFAULT_MAP = str(WORKSPACE / "maps" / "map.yaml")
DEFAULT_THERMAL_READER = str(WORKSPACE / "scripts" / "thermal" / "mlx90641_frames")
DEFAULT_MODEL = os.path.join(
    get_package_share_directory("firefighter_description"),
    "urdf", "firefighter.urdf.xacro",
)


def _include(package: str, launch_file: str, arguments: Dict[str, object],
             condition=None):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_dir = LaunchConfiguration("config_dir")
    serial_port = LaunchConfiguration("serial_port")
    bridge_port = LaunchConfiguration("bridge_port")
    thermal_reader = LaunchConfiguration("thermal_reader_path")
    map_file = LaunchConfiguration("map")
    startup_delay = LaunchConfiguration("startup_delay")
    hardware = IfCondition(LaunchConfiguration("hardware"))

    def cfg(name: str):
        return PathJoinSubstitution([config_dir, name])

    # ---- model -----------------------------------------------------------
    # robot_state_publisher owns the sensor mounts (base_link -> laser,
    # base_link -> thermal_camera); they come from the URDF, not launch args.
    description = _include(
        "firefighter_description", "description.launch.py",
        {
            "use_sim_time": use_sim_time,
            "model": LaunchConfiguration("model"),
            "publish_joint_states": LaunchConfiguration("hardware"),
        },
    )

    # ---- sensing ---------------------------------------------------------
    lidar = _include(
        "sllidar_ros2", "sllidar_a1_launch.py",
        {"serial_port": serial_port, "use_sim_time": use_sim_time},
        condition=hardware,
    )

    # ---- odometry --------------------------------------------------------
    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[cfg("rf2o.yaml"), {"use_sim_time": use_sim_time}],
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[cfg("ekf.yaml"), {"use_sim_time": use_sim_time}],
    )

    # ---- robot -----------------------------------------------------------
    bridge = _include(
        "firefighter_bridge", "bridge.launch.py", {"port": bridge_port},
        condition=hardware,
    )
    perception = _include(
        "firefighter_perception", "perception.launch.py",
        {"reader_path": thermal_reader},
    )

    # ---- autonomy --------------------------------------------------------
    # `slam` selects the source of map -> odom: SLAM Toolbox (mapping) or AMCL
    # against a saved map (localization). They are mutually exclusive, and both
    # are delayed: the costmaps need TF and /scan to exist before they start.
    slam_include = _include(
        "slam_toolbox", "online_async_launch.py",
        {"slam_params_file": cfg("slam_toolbox.yaml"), "use_sim_time": use_sim_time},
    )
    slam = TimerAction(
        period=startup_delay,
        actions=[slam_include],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    localization_include = _include(
        "nav2_bringup", "localization_launch.py",
        {
            "map": map_file,
            "params_file": cfg("nav2_params.yaml"),   # else AMCL uses base_footprint
            "use_sim_time": use_sim_time,
        },
    )
    localization = TimerAction(
        period=startup_delay,
        actions=[localization_include],
        condition=UnlessCondition(LaunchConfiguration("slam")),
    )

    nav2_include = _include(
        "nav2_bringup", "navigation_launch.py",
        {
            "params_file": cfg("nav2_params.yaml"),
            "use_sim_time": use_sim_time,
            "autostart": "true",
        },
    )
    nav2 = TimerAction(
        period=startup_delay,
        actions=[nav2_include],
        condition=IfCondition(LaunchConfiguration("nav2_enabled")),
    )

    # Without Nav2 there is no action server to send the goal to.
    mission = _include(
        "firefighter_mission", "mission.launch.py",
        {"use_nav2": LaunchConfiguration("nav2_enabled")},
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("firefighter_bringup"),
                         "rviz", "firefighter.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0",
                                  description="LiDAR serial port."),
            DeclareLaunchArgument("bridge_port", default_value="/dev/ttyAMA0",
                                  description="UART shared with the mobility ESP32."),
            DeclareLaunchArgument("config_dir", default_value=DEFAULT_CONFIG_DIR),
            DeclareLaunchArgument("map", default_value=DEFAULT_MAP,
                                  description="Map yaml, used when slam:=false."),
            DeclareLaunchArgument("slam", default_value="true",
                                  description="true = SLAM Toolbox mapping; "
                                              "false = AMCL on a saved map."),
            DeclareLaunchArgument("nav2_enabled", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("startup_delay", default_value="3.0",
                                  description="Seconds to wait before starting "
                                              "SLAM/Nav2, so TF and /scan exist."),
            DeclareLaunchArgument("thermal_reader_path", default_value=DEFAULT_THERMAL_READER),
            DeclareLaunchArgument("hardware", default_value="true",
                                  description="false = no LiDAR / ESP32 bridge "
                                              "(simulation)."),
            DeclareLaunchArgument("model", default_value=DEFAULT_MODEL,
                                  description="Robot XACRO published on "
                                              "/robot_description."),
            description,
            lidar,
            rf2o,
            ekf,
            bridge,
            perception,
            slam,
            localization,
            nav2,
            mission,
            rviz,
        ]
    )
