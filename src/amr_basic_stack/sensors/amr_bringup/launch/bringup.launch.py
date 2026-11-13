from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_launch = PathJoinSubstitution([
        FindPackageShare("amr_description"),
        "launch",
        "robot_state.launch.py",
    ])
    base_launch = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "launch",
        "base.launch.py",
    ])
    sensors_launch = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "launch",
        "sensors.launch.py",
    ])
    joy_teleop_config = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "config",
        "joy_teleop.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "mock_mode",
            default_value="false",
            description="Use false for full robot bringup, true for ROS-only motor tests.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="true",
            description="Start joy_node and joy_teleop_node for gamepad control.",
        ),
        DeclareLaunchArgument("use_front_lidar", default_value="true"),
        DeclareLaunchArgument("use_rear_lidar", default_value="true"),
        DeclareLaunchArgument("use_depth_camera", default_value="true"),
        DeclareLaunchArgument("front_lidar_serial_port", default_value="/dev/lidar_front"),
        DeclareLaunchArgument("rear_lidar_serial_port", default_value="/dev/lidar_rear"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={"mock_mode": LaunchConfiguration("mock_mode")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={
                "use_front_lidar": LaunchConfiguration("use_front_lidar"),
                "use_rear_lidar": LaunchConfiguration("use_rear_lidar"),
                "front_lidar_serial_port": LaunchConfiguration("front_lidar_serial_port"),
                "rear_lidar_serial_port": LaunchConfiguration("rear_lidar_serial_port"),
                "use_depth_camera": LaunchConfiguration("use_depth_camera"),
            }.items(),
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[joy_teleop_config],
            condition=IfCondition(LaunchConfiguration("use_joystick")),
        ),
        Node(
            package="amr_base",
            executable="joy_teleop_node",
            name="joy_teleop",
            output="screen",
            parameters=[joy_teleop_config],
            condition=IfCondition(LaunchConfiguration("use_joystick")),
        ),
    ])
