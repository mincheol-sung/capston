"""Expand real nested launch files while intercepting every process action.

No ROS process, sensor driver or motor is executed. Verify effective wiring,
scope, typed parameters, identity persistence and mutually exclusive ownership.
"""

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

ROOT = Path(__file__).resolve().parents[4]
LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'autonomy.launch.py'


def load_launch():
    spec = importlib.util.spec_from_file_location('autonomy_test', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def expand(tmp_path, monkeypatch):
    prefix = tmp_path / 'index'
    index = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    for package in (ROOT / 'src').rglob('package.xml'):
        if {'build', 'install', 'log'} & set(package.relative_to(ROOT / 'src').parts):
            continue
        name = ET.parse(package).getroot().findtext('name')
        (index / name).write_text('')
        (prefix / 'share' / name).symlink_to(package.parent, target_is_directory=True)
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    folder = tmp_path / 'maps' / 'lab'
    folder.mkdir(parents=True)
    (folder / 'lab.yaml').write_text('image: lab.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    (folder / 'lab.pgm').write_bytes(b'fixture image v1')
    (folder / 'lab.pbstream').write_bytes(b'fixture graph v1')
    captured = []

    def capture(action, context):
        executable = perform_substitutions(
            context, normalize_to_list_of_substitutions(action.node_executable)
        )
        parameters = []
        if executable in (
            'rgbd_detection_node',
            'semantic_slam_node',
            'semantic_navigation_node',
            'depth_preprocess_node',
            'realsense2_camera_node',
            'traversability_node',
        ):
            parameters = evaluate_parameters(context, action._Node__parameters)
        captured.append(
            {
                'executable': executable,
                'parameters': parameters,
                'rviz_config': context.launch_configurations.get('rviz_config'),
                'scope': dict(context.launch_configurations),
            }
        )
        return None

    def forbid_process(*_, **__):
        raise AssertionError('Launch test must never execute a process')

    monkeypatch.setattr(Node, 'execute', capture)
    monkeypatch.setattr(ExecuteProcess, 'execute', forbid_process)

    def run(**options):
        captured.clear()
        context = LaunchContext()
        context.launch_configurations.update({'map': 'lab', 'maps_dir': str(folder.parent)})
        context.launch_configurations.update(options)
        description = load_launch().generate_launch_description()
        visit_all_entities_and_collect_futures(description, context)
        return list(captured), context

    return run, folder, index


def entries(nodes, executable):
    return [n for n in nodes if n['executable'] == executable]


def params(node):
    values = {}
    for item in node['parameters']:
        if isinstance(item, dict):
            values.update(item)
    return values


def test_disabled_preserves_baseline_without_resolving_semantic_files(expand):
    run, folder, index = expand
    (index / 'semantic_slam').unlink()
    (folder / 'lab.pgm').unlink()  # Disabled feature never fingerprints the map.
    nodes, context = run()
    assert not entries(nodes, 'semantic_slam_node')
    assert not entries(nodes, 'semantic_navigation_node')
    assert not entries(nodes, 'depth_preprocess_node')
    detector = params(entries(nodes, 'rgbd_detection_node')[0])
    assert detector['publish_semantic'] is False and detector['inference_hz'] == 0.0
    assert context.launch_configurations['rviz_config'].endswith('stack_master/rviz/autonomy.rviz')
    assert '_semantic_map_id' not in context.launch_configurations
    assert params(entries(nodes, 'traversability_node')[0])['use_depth'] is False


@pytest.mark.parametrize('voxel', ['false', 'true'])
def test_enabled_reuses_inference_and_starts_each_semantic_node_once(expand, voxel):
    run, _, _ = expand
    nodes, context = run(use_semantic_navigation='true', use_voxel_3d=voxel, use_sim_time='true')
    assert len(entries(nodes, 'rgbd_detection_node')) == 1
    assert len(entries(nodes, 'semantic_slam_node')) == 1
    assert len(entries(nodes, 'semantic_navigation_node')) == 1
    assert len(entries(nodes, 'depth_preprocess_node')) == 1
    assert params(entries(nodes, 'traversability_node')[0])['use_depth'] is True
    detector = params(entries(nodes, 'rgbd_detection_node')[0])
    mapper = params(entries(nodes, 'semantic_slam_node')[0])
    navigator = params(entries(nodes, 'semantic_navigation_node')[0])
    camera = params(entries(nodes, 'realsense2_camera_node')[0])
    assert detector['publish_semantic'] is True and detector['inference_hz'] == 5.0
    assert mapper['enable_cartographer_correction'] is True
    assert navigator['require_pose_graph'] is True and navigator['enable_goal_dispatch'] is False
    assert (
        mapper['map_id']
        == navigator['map_id']
        == context.launch_configurations['_semantic_map_id']
    )
    assert mapper['cartographer_graph_id'].startswith('carto-')
    assert mapper['database_path'].endswith('.json')
    assert camera['pointcloud.enable'] is True and camera['decimation_filter.enable'] is False
    assert mapper['use_sim_time'] is True and navigator['use_sim_time'] is True
    assert context.launch_configurations['rviz_config'].endswith(
        'semantic_slam/rviz/semantic.rviz'
    )
    assert context.launch_configurations['use_rviz'] == 'true'  # Child scope did not disable RViz.


def test_manual_overrides_and_dispatch_reach_child_without_starting_second_detector(expand):
    run, _, _ = expand
    nodes, _ = run(
        use_semantic_navigation='true',
        semantic_map_id='saved-lab-v2',
        semantic_graph_id='saved-carto-session',
        semantic_database_path='/tmp/saved.json',
        semantic_goal_dispatch='true',
        inference_hz='3.0',
        rviz_config='/tmp/custom.rviz',
        yolo_model_path='/tmp/custom_openvino',
    )
    mapper = params(entries(nodes, 'semantic_slam_node')[0])
    navigator = params(entries(nodes, 'semantic_navigation_node')[0])
    detector = params(entries(nodes, 'rgbd_detection_node')[0])
    assert mapper['map_id'] == 'saved-lab-v2' and mapper['database_path'] == '/tmp/saved.json'
    assert mapper['cartographer_graph_id'] == 'saved-carto-session'
    assert navigator['enable_goal_dispatch'] is True
    assert (
        detector['inference_hz'] == 3.0 and detector['yolo_model_path'] == '/tmp/custom_openvino'
    )
    assert entries(nodes, 'rviz2')[0]['rviz_config'] == '/tmp/custom.rviz'


def test_automatic_session_is_new_while_map_identity_is_stable(expand):
    run, _, _ = expand
    _, first = run(use_semantic_navigation='true')
    _, second = run(use_semantic_navigation='true')
    assert (
        first.launch_configurations['_semantic_map_id']
        == second.launch_configurations['_semantic_map_id']
    )
    assert (
        first.launch_configurations['_semantic_graph_id']
        != second.launch_configurations['_semantic_graph_id']
    )
    assert (
        first.launch_configurations['_semantic_database_path']
        != second.launch_configurations['_semantic_database_path']
    )


@pytest.mark.parametrize('filename', ['lab.yaml', 'lab.pgm', 'lab.pbstream'])
def test_map_content_change_selects_different_object_memory(expand, filename):
    run, folder, _ = expand
    _, first = run(use_semantic_navigation='true', semantic_graph_id='same-session')
    path = folder / filename
    path.write_bytes(path.read_bytes() + b'\n')
    _, second = run(use_semantic_navigation='true', semantic_graph_id='same-session')
    assert (
        first.launch_configurations['_semantic_map_id']
        != second.launch_configurations['_semantic_map_id']
    )
    assert (
        first.launch_configurations['_semantic_database_path']
        != second.launch_configurations['_semantic_database_path']
    )


def test_fixed_map_mode_reuses_db_without_a_graph_session(expand):
    run, _, _ = expand
    first, a = run(use_semantic_navigation='true', semantic_cartographer_correction='false')
    _, b = run(use_semantic_navigation='true', semantic_cartographer_correction='false')
    assert (
        a.launch_configurations['_semantic_database_path']
        == b.launch_configurations['_semantic_database_path']
    )
    mapper = params(entries(first, 'semantic_slam_node')[0])
    navigator = params(entries(first, 'semantic_navigation_node')[0])
    assert (
        mapper['cartographer_graph_id'] == '' and mapper['enable_cartographer_correction'] is False
    )
    assert navigator['require_pose_graph'] is False


def test_missing_perception_fails_before_any_process_is_started(expand):
    run, _, _ = expand
    with pytest.raises(RuntimeError, match='requires use_perception'):
        run(use_semantic_navigation='true', use_perception='false')
