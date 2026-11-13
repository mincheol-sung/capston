from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg_name, launch_file, launch_arguments=None):
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            "launch",
            launch_file,
        ])
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=launch_arguments.items() if launch_arguments else None,
    )


def generate_launch_description():
    slam_params_file = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "config",
        "slam_toolbox.yaml",
    ])

    robot_bringup = _include(
        "amr_bringup",
        "bringup.launch.py",
        {
            "mock_mode": LaunchConfiguration("mock_mode"),
            "use_joystick": LaunchConfiguration("use_joystick"),
            "use_front_lidar": LaunchConfiguration("use_front_lidar"),
            "use_rear_lidar": LaunchConfiguration("use_rear_lidar"),
            "use_depth_camera": LaunchConfiguration("use_depth_camera"),
            "front_lidar_serial_port": LaunchConfiguration("front_lidar_serial_port"),
            "rear_lidar_serial_port": LaunchConfiguration("rear_lidar_serial_port"),
        },
    )

    slam_toolbox = _include(
        "slam_toolbox",
        "online_async_launch.py",
        {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "autostart": "true",
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mock_mode",
            default_value="false",
            description="Use false for the real robot, true for ROS-only dry runs.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="true",
            description="Start joy_node and joy_teleop_node for gamepad control.",
        ),
        DeclareLaunchArgument("use_front_lidar", default_value="true"),
        DeclareLaunchArgument("use_rear_lidar", default_value="true"),
        DeclareLaunchArgument(
            "use_depth_camera",
            default_value="false",
            description="Depth camera is not required for 2D slam_toolbox mapping.",
        ),
        DeclareLaunchArgument("front_lidar_serial_port", default_value="/dev/lidar_front"),
        DeclareLaunchArgument("rear_lidar_serial_port", default_value="/dev/lidar_rear"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params_file),
        robot_bringup,
        slam_toolbox,
    ])
