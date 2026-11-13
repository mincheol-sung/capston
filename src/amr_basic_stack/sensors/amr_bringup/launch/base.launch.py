from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "config",
        "base.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "mock_mode",
            default_value="true",
            description="Use true for ROS-only tests, false for real RoboClaw serial output.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="odom->base_link TF 발행. robot_localization EKF와 함께 쓸 땐 false(EKF가 소유).",
        ),
        Node(
            package="amr_base",
            executable="roboclaw_mecanum_node",
            name="roboclaw_mecanum_node",
            output="screen",
            parameters=[
                config_file,
                {"mock_mode": LaunchConfiguration("mock_mode")},
                {"publish_tf": LaunchConfiguration("publish_tf")},
            ],
        ),
    ])
