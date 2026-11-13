"""Check semantic launch configuration without starting processes."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities import visit_all_entities_and_collect_futures
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


@pytest.fixture
def expand(tmp_path, monkeypatch):
    prefix = tmp_path / 'index'
    index = prefix / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    for package in (ROOT / 'src').rglob('package.xml'):
        if {'build', 'install', 'log'} & set(package.relative_to(ROOT / 'src').parts):
            continue
        name = ET.parse(package).getroot().findtext('name')
        (index / name).write_text('')
        (prefix / 'share' / name).symlink_to(package.parent, target_is_directory=True)
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    captured = []

    def capture(action, context):
        executable = perform_substitutions(
            context, normalize_to_list_of_substitutions(action.node_executable))
        parameters = evaluate_parameters(context, action._Node__parameters)
        values = {}
        for item in parameters:
            if isinstance(item, dict):
                values.update(item)
            else:
                values.update(yaml.safe_load(Path(item).read_text())[executable]['ros__parameters'])
        captured.append((executable, values))

    def forbid(*_, **__):
        raise AssertionError('No process may start in launch tests')

    monkeypatch.setattr(Node, 'execute', capture)
    monkeypatch.setattr(ExecuteProcess, 'execute', forbid)

    def run(filename, **options):
        captured.clear()
        context = LaunchContext()
        context.launch_configurations.update(options)
        spec = importlib.util.spec_from_file_location('semantic_launch_test', LAUNCH / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        visit_all_entities_and_collect_futures(module.generate_launch_description(), context)
        return dict(captured)

    return run, captured, index


@pytest.mark.parametrize('graph_id', ['', '   '])
def test_navigation_rejects_missing_graph_before_nodes(expand, graph_id):
    run, captured, _ = expand
    with pytest.raises(RuntimeError, match='graph_id'):
        run('semantic_navigation.launch.py', map_id='lab', graph_id=graph_id)
    assert captured == []


@pytest.mark.parametrize('enabled', ['false', 'true'])
def test_navigation_correction_and_typed_identity(expand, enabled):
    run, _, _ = expand
    nodes = run('semantic_navigation.launch.py', map_id='123',
                enable_cartographer_correction=enabled,
                graph_id='456' if enabled == 'true' else '')
    mapper = nodes['semantic_slam_node']
    assert mapper['map_id'] == '123'
    assert mapper['enable_cartographer_correction'] is (enabled == 'true')
    assert nodes['semantic_navigation_node']['require_pose_graph'] is (enabled == 'true')
    assert 'rgbd_detection_node' not in nodes


def test_perception_reads_yaml_and_keeps_correction_off(expand):
    run, _, _ = expand
    nodes = run('perception_stack.launch.py', use_semantic_slam='true', semantic_map_id='123')
    mapper = nodes['semantic_slam_node']
    assert mapper['map_id'] == '123'
    assert mapper['save_period_sec'] == 10.0
    assert mapper['localization_settle_time'] == 2.0
    assert mapper['enable_cartographer_correction'] is False
    assert nodes['rgbd_detection_node']['publish_semantic'] is False
    assert nodes['rgbd_detection_node']['inference_hz'] == 0.0


def test_perception_custom_yaml_and_overrides(expand, tmp_path):
    run, _, _ = expand
    config = tmp_path / 'custom.yaml'
    config.write_text('semantic_slam_node:\n  ros__parameters:\n    save_period_sec: 23.0\n')
    db = str(tmp_path / 'objects.json')
    nodes = run('perception_stack.launch.py', use_semantic_slam='true',
                semantic_map_id='123', semantic_graph_id='456',
                semantic_database_path=db, semantic_config=str(config),
                enable_cartographer_correction='true')
    mapper = nodes['semantic_slam_node']
    assert mapper['save_period_sec'] == 23.0
    assert mapper['cartographer_graph_id'] == '456'
    assert mapper['database_path'] == db
    assert mapper['enable_cartographer_correction'] is True


@pytest.mark.parametrize('options, message', [
    ({'semantic_map_id': '  '}, 'semantic_map_id'),
    ({'semantic_map_id': 'lab', 'enable_cartographer_correction': 'true'}, 'semantic_graph_id'),
])
def test_perception_validation_precedes_all_nodes(expand, options, message):
    run, captured, _ = expand
    with pytest.raises(RuntimeError, match=message):
        run('perception_stack.launch.py', use_semantic_slam='true', **options)
    assert captured == []


def test_disabled_perception_does_not_resolve_semantic_package(expand):
    run, _, index = expand
    (index / 'semantic_slam').unlink()
    nodes = run('perception_stack.launch.py')
    assert 'semantic_slam_node' not in nodes
