"""Compare radius/cluster, optionally actual tracked memberships, on identical scans."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('dynamic_filter'), 'config')
    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_front_topic', default_value='/scan_front_carto'),
        DeclareLaunchArgument('scan_rear_topic', default_value='/scan_rear_carto'),
        DeclareLaunchArgument('tracks_topic', default_value='/tracked_object_modes'),
        DeclareLaunchArgument('include_tracked_cluster', default_value='false'),
        DeclareLaunchArgument('tracked_observations_topic', default_value='/tracked_observations'),
        DeclareLaunchArgument('front_sensor_id', default_value='/scan_front'),
        DeclareLaunchArgument('rear_sensor_id', default_value='/scan_rear'),
        DeclareLaunchArgument('radius_config', default_value=os.path.join(
            config_dir, 'dynamic_filter.yaml')),
        DeclareLaunchArgument('cluster_config', default_value=os.path.join(
            config_dir, 'dynamic_filter_cluster.yaml')),
        DeclareLaunchArgument('tracked_cluster_config', default_value=os.path.join(
            config_dir, 'dynamic_filter_tracked_cluster.yaml')),
    ]
    for method in ('radius', 'cluster', 'tracked_cluster'):
        for side in ('front', 'rear'):
            options = ({'condition': IfCondition(LaunchConfiguration('include_tracked_cluster'))}
                       if method == 'tracked_cluster' else {})
            actions.append(Node(
                package='dynamic_filter', executable='dynamic_filter_node',
                name=f'dynamic_filter_{method}_{side}', output='screen',
                parameters=[LaunchConfiguration(f'{method}_config'), {
                    'mask_method': method,
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                   value_type=bool),
                    'tracks_topic': LaunchConfiguration('tracks_topic'),
                    'tracked_observations_topic': LaunchConfiguration(
                        'tracked_observations_topic'),
                    'observation_sensor_id': LaunchConfiguration(f'{side}_sensor_id'),
                    'scan_input_topic': 'scan_in', 'scan_output_topic': 'scan_out',
                }],
                remappings=[('scan_in', LaunchConfiguration(f'scan_{side}_topic')),
                            ('scan_out', f'/dynamic_filter_compare/{method}/scan_{side}')],
                **options,
            ))
    return LaunchDescription(actions)
