"""Expand actual launch actions and inspect resolved Cartographer routing."""

import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.utilities import visit_all_entities_and_collect_futures
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

STACK = Path(__file__).resolve().parents[3]
LAUNCH = STACK / 'perception/amr_perception_bringup/launch'


@pytest.fixture
def expand(tmp_path, monkeypatch):
    prefix = tmp_path / 'index'
    index = prefix / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    for package in STACK.rglob('package.xml'):
        if {'build', 'install', 'log'} & set(package.relative_to(STACK).parts):
            continue
        name = ET.parse(package).getroot().findtext('name')
        (index / name).write_text('')
        (prefix / 'share' / name).symlink_to(package.parent, target_is_directory=True)
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix) + ':' + os.getenv('AMENT_PREFIX_PATH', ''))
    captured, includes = {}, []

    def text(context, value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def capture(action, context):
        name = text(context, action._Node__node_name or action.node_executable)
        values = {}
        for item in evaluate_parameters(context, action._Node__parameters):
            if isinstance(item, dict):
                values.update(item)
            else:
                document = yaml.safe_load(Path(item).read_text())
                values.update(document.get('/**', {}).get('ros__parameters', {}))
                values.update(document.get(name, {}).get('ros__parameters', {}))
        captured[name] = {
            'parameters': values,
            'remappings': {text(context, key): text(context, value)
                          for key, value in action._Node__remappings or []},
        }

    original_include = IncludeLaunchDescription.execute

    def include(action, context):
        location = text(context, action.launch_description_source._LaunchDescriptionSource__location)
        args = {key: text(context, value) for key, value in action.launch_arguments}
        includes.append((location, args))
        if location.endswith('cartographer.launch.py'):
            return original_include(action, context)
        return None

    def forbid(*_, **__):
        raise AssertionError('No processes may start in routing tests')

    monkeypatch.setattr(Node, 'execute', capture)
    monkeypatch.setattr(ExecuteProcess, 'execute', forbid)
    monkeypatch.setattr(IncludeLaunchDescription, 'execute', include)

    def run(path, **options):
        context = LaunchContext()
        context.launch_configurations.update(options)
        spec = importlib.util.spec_from_file_location('routing_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        visit_all_entities_and_collect_futures(module.generate_launch_description(), context)
        return captured, includes

    return run


@pytest.mark.parametrize('enabled', ['false', 'true'])
@pytest.mark.parametrize('custom', [False, True])
def test_cartographer_actual_scan_routes(expand, enabled, custom):
    source_front = '/custom/front' if custom else '/scan_front_carto'
    source_rear = '/custom/rear' if custom else '/scan_rear_carto'
    nodes, _ = expand(LAUNCH / 'cartographer.launch.py', use_dynamic_filter=enabled,
                      scan_front_topic=source_front, scan_rear_topic=source_rear,
                      use_sim_time='true', mode='mapping')
    remaps = nodes['cartographer_node']['remappings']
    if enabled == 'true':
        assert remaps['scan_1'] == '/scan_front_static'
        assert remaps['scan_2'] == '/scan_rear_static'
        for side, source in [('front', source_front), ('rear', source_rear)]:
            node = nodes['dynamic_filter_' + side]
            assert node['remappings'] == {'scan_in': source, 'scan_out': f'/scan_{side}_static'}
            assert node['parameters']['use_sim_time'] is True
            assert node['parameters']['filter_policy'] == 'confirmed_person'
    else:
        assert remaps['scan_1'] == source_front and remaps['scan_2'] == source_rear
        assert not any(name.startswith('dynamic_filter') for name in nodes)


def test_perception_does_not_duplicate_filters(expand):
    nodes, _ = expand(LAUNCH / 'perception_stack.launch.py')
    assert 'object_tracker_node' in nodes
    assert not any(name.startswith('dynamic_filter') for name in nodes)


def test_autonomy_forwards_options_to_real_cartographer(expand, tmp_path):
    folder = tmp_path / 'maps' / 'demo'
    folder.mkdir(parents=True)
    (folder / 'demo.pbstream').write_bytes(b'launch test only')
    nodes, includes = expand(STACK / 'stack_master/launch/autonomy.launch.py',
                             map='demo', maps_dir=str(folder.parent),
                             use_dynamic_filter='true', use_sim_time='true',
                             scan_front_topic='/bag/front', scan_rear_topic='/bag/rear')
    assert nodes['dynamic_filter_front']['remappings']['scan_in'] == '/bag/front'
    assert nodes['dynamic_filter_rear']['remappings']['scan_in'] == '/bag/rear'
    assert nodes['cartographer_node']['remappings']['scan_1'] == '/scan_front_static'
    robot = [args for path, args in includes if path.endswith('amr_system.launch.py')]
    assert len(robot) == 1 and robot[0]['use_perception'] == 'true'
    assert robot[0]['use_object_tracker'] == 'true'


def test_reject_feedback_loop(expand):
    with pytest.raises(RuntimeError, match='source topics'):
        expand(LAUNCH / 'cartographer.launch.py', use_dynamic_filter='true',
               scan_front_topic='/scan_front_static')


def test_custom_config_cannot_bypass_launch_source_arguments(expand, tmp_path):
    config = tmp_path / 'custom.yaml'
    config.write_text('/**:\n  ros__parameters:\n    filter_policy: dynamic_only\n'
                      '    scan_input_topic: /wrong_input\n    scan_output_topic: /wrong_output\n')
    nodes, _ = expand(LAUNCH / 'cartographer.launch.py', use_dynamic_filter='true',
                      dynamic_filter_config=str(config), scan_front_topic='/custom/front')
    front = nodes['dynamic_filter_front']
    assert front['parameters']['filter_policy'] == 'dynamic_only'
    assert front['parameters']['scan_input_topic'] == 'scan_in'
    assert front['parameters']['scan_output_topic'] == 'scan_out'
    assert front['remappings']['scan_in'] == '/custom/front'


def test_cluster_config_routes_both_sensors(expand):
    config = STACK / 'perception/dynamic_filter/config/dynamic_filter_cluster.yaml'
    nodes, _ = expand(LAUNCH / 'cartographer.launch.py', use_dynamic_filter='true',
                      dynamic_filter_config=str(config), mode='mapping')
    for side in ('front', 'rear'):
        assert nodes[f'dynamic_filter_{side}']['parameters']['mask_method'] == 'cluster'
    assert nodes['cartographer_node']['remappings']['scan_1'] == '/scan_front_static'


def test_comparison_launch_has_four_separate_outputs_and_no_cartographer(expand):
    nodes, _ = expand(STACK / 'perception/dynamic_filter/launch/compare_filters.launch.py',
                      use_sim_time='true', scan_front_topic='/bag/front',
                      scan_rear_topic='/bag/rear', tracks_topic='/bag/tracks')
    assert len(nodes) == 4
    for method in ('radius', 'cluster'):
        for side in ('front', 'rear'):
            node = nodes[f'dynamic_filter_{method}_{side}']
            assert node['parameters']['mask_method'] == method
            assert node['parameters']['tracks_topic'] == '/bag/tracks'
            assert node['parameters']['use_sim_time'] is True
            assert node['remappings']['scan_in'] == f'/bag/{side}'
            assert node['remappings']['scan_out'] == f'/dynamic_filter_compare/{method}/scan_{side}'


def test_comparison_can_include_actual_tracker_memberships(expand):
    nodes, _ = expand(STACK / 'perception/dynamic_filter/launch/compare_filters.launch.py',
                      include_tracked_cluster='true', tracked_observations_topic='/bag/links',
                      front_sensor_id='/native/front', rear_sensor_id='/native/rear')
    assert len(nodes) == 6
    for side in ('front', 'rear'):
        node = nodes[f'dynamic_filter_tracked_cluster_{side}']
        assert node['parameters']['mask_method'] == 'tracked_cluster'
        assert node['parameters']['observation_sensor_id'] == f'/native/{side}'
        assert node['parameters']['tracked_observations_topic'] == '/bag/links'
        assert node['remappings']['scan_out'] == f'/dynamic_filter_compare/tracked_cluster/scan_{side}'


def test_cartographer_tracked_cluster_config_has_native_sensor_identity(expand):
    config = STACK / 'perception/dynamic_filter/config/dynamic_filter_tracked_cluster.yaml'
    nodes, _ = expand(LAUNCH / 'cartographer.launch.py', use_dynamic_filter='true',
                      dynamic_filter_config=str(config), mode='mapping')
    for side in ('front', 'rear'):
        node = nodes[f'dynamic_filter_{side}']
        assert node['parameters']['mask_method'] == 'tracked_cluster'
        assert node['parameters']['observation_sensor_id'] == f'/scan_{side}'
        assert node['remappings']['scan_in'] == f'/scan_{side}_carto'
        assert node['remappings']['scan_out'] == f'/scan_{side}_static'
    assert nodes['cartographer_node']['remappings']['scan_1'] == '/scan_front_static'
    assert nodes['cartographer_node']['remappings']['scan_2'] == '/scan_rear_static'
