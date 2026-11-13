from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "config",
        "base.yaml",
    ])

    driver_node = Node(
        package="amr_base",
        executable="roboclaw_mecanum_node",
        name="roboclaw_mecanum_node",
        output="screen",
        parameters=[
            config_file,
            {"mock_mode": LaunchConfiguration("mock_mode")},
            {"publish_tf": LaunchConfiguration("publish_tf")},
        ],
    )

    test_node = Node(
        package="amr_base",
        executable="velocity_step_test",
        name="velocity_step_test",
        output="screen",
        parameters=[{
            "output_directory": LaunchConfiguration("output_directory"),
            "driver_node_name": "/roboclaw_mecanum_node",
        }],
    )

    delayed_test = TimerAction(
        period=LaunchConfiguration("startup_delay_s"),
        actions=[test_node],
    )
    shutdown_after_test = RegisterEventHandler(
        OnProcessExit(
            target_action=test_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="velocity step test completed")
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mock_mode",
            default_value="true",
            description="Use false only for a secured real-robot test.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish odom to base TF during the test.",
        ),
        DeclareLaunchArgument(
            "startup_delay_s",
            default_value="2.0",
            description="Delay before starting the step sequence.",
        ),
        DeclareLaunchArgument(
            "output_directory",
            default_value=EnvironmentVariable("PWD", default_value="."),
            description="Directory for timestamped velocity step CSV files.",
        ),
        driver_node,
        shutdown_after_test,
        delayed_test,
    ])
