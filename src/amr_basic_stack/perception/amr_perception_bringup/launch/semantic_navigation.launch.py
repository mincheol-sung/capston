"""Add object mapping/navigation to an already running AMR localization stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate_identity(context):
    if not LaunchConfiguration('map_id').perform(context).strip():
        raise RuntimeError('semantic_navigation requires non-empty map_id')
    if (IfCondition(LaunchConfiguration('enable_cartographer_correction')).evaluate(context)
            and not LaunchConfiguration('graph_id').perform(context).strip()):
        raise RuntimeError(
            'enable_cartographer_correction=true requires non-empty graph_id')


def generate_launch_description():
    """Use one inference node, either launched here or configured externally."""
    config = LaunchConfiguration('config')
    shared = {
        'map_id': ParameterValue(LaunchConfiguration('map_id'), value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map_id', description='Required fixed map identity and revision'
            ),
            DeclareLaunchArgument(
                'config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('semantic_slam'), 'config', 'semantic.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'database_path', default_value='~/.ros/amr/semantic_objects.json'
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'graph_id', default_value='',
                description='Required for correction: saved Cartographer graph/session identity'),
            DeclareLaunchArgument('enable_cartographer_correction', default_value='true'),
            DeclareLaunchArgument(
                'start_detector',
                default_value='false',
                description='true only when no RGB-D inference node is running',
            ),
            DeclareLaunchArgument('yolo_model_path', default_value=''),
            DeclareLaunchArgument('enable_goal_dispatch', default_value='false'),
            OpaqueFunction(function=_validate_identity),
            Node(
                package='sensor_fusion',
                executable='rgbd_detection_node',
                output='screen',
                name='rgbd_detection_node',
                parameters=[
                    config,
                    {
                        'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                        'use_sim_time': shared['use_sim_time'],
                    },
                ],
                condition=IfCondition(LaunchConfiguration('start_detector')),
            ),
            Node(
                package='semantic_slam',
                executable='semantic_slam_node',
                output='screen',
                name='semantic_slam_node',
                parameters=[
                    config,
                    shared,
                    {
                        'database_path': ParameterValue(
                            LaunchConfiguration('database_path'), value_type=str),
                        'cartographer_graph_id': ParameterValue(
                            LaunchConfiguration('graph_id'), value_type=str),
                        'enable_cartographer_correction': ParameterValue(
                            LaunchConfiguration('enable_cartographer_correction'), value_type=bool),
                    },
                ],
            ),
            Node(
                package='semantic_slam',
                executable='semantic_navigation_node',
                output='screen',
                name='semantic_navigation_node',
                parameters=[
                    config,
                    shared,
                    {
                        'enable_goal_dispatch': ParameterValue(
                            LaunchConfiguration('enable_goal_dispatch'), value_type=bool
                        ),
                        'require_pose_graph': ParameterValue(
                            LaunchConfiguration('enable_cartographer_correction'), value_type=bool),
                    },
                ],
            ),
        ]
    )
