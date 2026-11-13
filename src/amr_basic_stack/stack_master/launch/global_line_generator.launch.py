#!/usr/bin/env python3
"""Generate global waypoints with the Theta* global planner."""

import os
import shlex
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import FindExecutable


def get_source_maps_dir():
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        sibling_maps = parent / "planner" / "global_planner" / "maps"
        if sibling_maps.is_dir():
            return str(sibling_maps)

        workspace_maps = (
            parent
            / "src"
            / "amr_basic_stack"
            / "planner"
            / "global_planner"
            / "maps"
        )
        if workspace_maps.is_dir():
            return str(workspace_maps)

    return "/home/smoc/2026amr_ws/src/amr_basic_stack/planner/global_planner/maps"


def get_source_cyclonedds_config():
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        sibling_config = parent / "stack_master" / "config" / "cyclonedds.xml"
        if sibling_config.is_file():
            return str(sibling_config)

        workspace_config = (
            parent
            / "src"
            / "amr_basic_stack"
            / "stack_master"
            / "config"
            / "cyclonedds.xml"
        )
        if workspace_config.is_file():
            return str(workspace_config)

    return "/home/smoc/2026amr_ws/src/amr_basic_stack/stack_master/config/cyclonedds.xml"


def launch_setup(context, *args, **kwargs):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    pose_mode = value("pose_mode").lower()
    planner_cmd = [
        FindExecutable(name="ros2").perform(context),
        "run",
        "global_planner",
        "global_planner_node",
        value("map"),
        "--planner",
        value("planner"),
        "--map-dir",
        value("map_dir"),
        "--output-dir",
        value("output_dir"),
        "--robot-size",
        value("robot_size"),
        "--waypoint-interval",
        value("waypoint_interval"),
    ]

    publisher_cmd = [
        FindExecutable(name="ros2").perform(context),
        "run",
        "global_planner",
        "global_path_publisher_node",
        value("map"),
        "--waypoints-file",
        os.path.join(value("output_dir"), "waypoints.csv"),
        "--path-topic",
        value("path_topic"),
        "--publish-period",
        value("publish_period"),
        "--wait-timeout",
        value("publisher_wait_timeout"),
    ]
    if value("keep_alive").lower() != "true":
        publisher_cmd.append("--once")

    if pose_mode == "yaml":
        planner_cmd.append("--no-preview")
        planner_cmd.append("--use-yaml-pose")
    elif pose_mode == "world":
        planner_cmd.append("--no-preview")
        planner_cmd.extend([
            "--start",
            value("start_x"),
            value("start_y"),
            "--goal",
            value("goal_x"),
            value("goal_y"),
        ])
    elif pose_mode == "grid":
        planner_cmd.append("--no-preview")
        planner_cmd.extend([
            "--start-grid",
            value("start_grid_x"),
            value("start_grid_y"),
            "--goal-grid",
            value("goal_grid_x"),
            value("goal_grid_y"),
        ])
    elif pose_mode == "interactive":
        if value("show_map").lower() == "true":
            planner_cmd.append("--show-map")
    else:
        raise ValueError("pose_mode must be one of: yaml, world, grid, interactive")

    if pose_mode == "interactive":
        shell_cmd = " ".join(shlex.quote(part) for part in planner_cmd)
        planner_process = ExecuteProcess(
            cmd=["bash", "-lc", f"{shell_cmd} < /dev/tty"],
            output="screen",
            emulate_tty=True,
        )
    else:
        planner_process = ExecuteProcess(
            cmd=planner_cmd,
            output="screen",
            emulate_tty=True,
        )

    publisher_process = ExecuteProcess(
        cmd=publisher_cmd,
        output="screen",
        emulate_tty=True,
    )

    return [
        planner_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=planner_process,
                on_exit=[publisher_process],
            )
        ),
    ]


def generate_launch_description():
    map_name = LaunchConfiguration("map")
    default_map_dir = get_source_maps_dir()
    default_output_dir = PathJoinSubstitution([
        default_map_dir,
        map_name,
    ])
    set_cyclonedds_uri = SetEnvironmentVariable(
        "CYCLONEDDS_URI",
        f"file://{get_source_cyclonedds_config()}",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="my_factory_map",
            description="Map folder/name under global_planner/maps.",
        ),
        DeclareLaunchArgument(
            "planner",
            default_value="theta_star",
            description="Global planner implementation. Example: theta_star.",
        ),
        DeclareLaunchArgument(
            "path_topic",
            default_value="/global_path",
            description="nav_msgs/Path topic for generated global waypoints.",
        ),
        DeclareLaunchArgument(
            "keep_alive",
            default_value="true",
            description="Keep global_path_publisher_node alive after publishing the path.",
        ),
        DeclareLaunchArgument(
            "publish_period",
            default_value="1.0",
            description="Seconds between /global_path republishes while alive.",
        ),
        DeclareLaunchArgument(
            "publisher_wait_timeout",
            default_value="5.0",
            description=(
                "Seconds for publisher node to wait for waypoints.csv."
            ),
        ),
        DeclareLaunchArgument(
            "map_dir",
            default_value=default_map_dir,
            description="Directory containing <map_name>/<map_name>.yaml.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=default_output_dir,
            description=(
                "Directory for generated waypoints.csv and path_result.png."
            ),
        ),
        DeclareLaunchArgument(
            "pose_mode",
            default_value="interactive",
            description=(
                "Use yaml start/goal, world coordinates, grid coordinates, "
                "or interactive terminal input."
            ),
        ),
        DeclareLaunchArgument(
            "show_map",
            default_value="true",
            description="Show the map window in interactive mode when a GUI is available.",
        ),
        DeclareLaunchArgument("start_x", default_value="-4.45"),
        DeclareLaunchArgument("start_y", default_value="4.45"),
        DeclareLaunchArgument("goal_x", default_value="4.55"),
        DeclareLaunchArgument("goal_y", default_value="-4.55"),
        DeclareLaunchArgument("start_grid_x", default_value="5"),
        DeclareLaunchArgument("start_grid_y", default_value="5"),
        DeclareLaunchArgument("goal_grid_x", default_value="95"),
        DeclareLaunchArgument("goal_grid_y", default_value="95"),
        DeclareLaunchArgument(
            "robot_size",
            default_value="0.30",
            description="Robot safety size in meters for obstacle inflation.",
        ),
        DeclareLaunchArgument(
            "waypoint_interval",
            default_value="0.1",
            description="Dense waypoint interval in meters.",
        ),
        set_cyclonedds_uri,
        OpaqueFunction(function=launch_setup),
    ])
