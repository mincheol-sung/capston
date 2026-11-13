from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    pkg_share = FindPackageShare("amr_description")

    xacro_path = PathJoinSubstitution([
        pkg_share, "urdf", "robots", "amr_basic.urdf.xacro",
    ])
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_path,
        " ",
        "robot_name:=",
        robot_name,
        " ",
        "prefix:=",
        prefix,
        " ",
        "use_gazebo:=true",
    ])

    bridge_config = PathJoinSubstitution([
        pkg_share, "config", "amr_basic", "gz_bridge.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        pkg_share, "rviz", "amr_description.rviz",
    ])

    # Gazebo resolves package:// mesh URIs through GZ_SIM_RESOURCE_PATH,
    # which must point at the parent of the package share directory.
    set_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        PathJoinSubstitution([pkg_share, ".."]),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py",
            ])
        ),
        launch_arguments={"gz_args": ["-r -v 1 ", world]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_name,
            "-z", "0.05",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            "use_sim_time": True,
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="amr_basic"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("world", default_value="empty.sdf"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        # Controllers can only be spawned once the robot (and with it the
        # gz_ros2_control plugin / controller_manager) exists in the sim.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[mecanum_drive_controller_spawner],
            )
        ),
        rviz,
    ])
