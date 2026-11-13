from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
    
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("amr_description"),
        "rviz",
        "amr_description.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="amr_basic"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(use_robot_state_publisher),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(use_joint_state_publisher),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
