from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_path = PathJoinSubstitution([
        FindPackageShare("amr_description"),
        "urdf",
        "robots",
        "amr_basic.urdf.xacro",
    ])
    # robot_description = Command([
    #     FindExecutable(name="xacro"),
    #     " ",
    #     xacro_path,
    #     " ",
    #     "robot_name:=",
    #     robot_name,
    #     " ",
    #     "prefix:=",
    #     prefix,
    #     " ",
    #     "use_gazebo:=",
    #     use_gazebo,
    # ])
    # 수정 시작
    robot_description = ParameterValue(
        Command([
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
            "use_gazebo:=",
            use_gazebo,
        ]),
        value_type=str,
    )
    # 수정 끝

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="amr_basic"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
