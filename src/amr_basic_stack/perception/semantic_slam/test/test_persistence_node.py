"""Exercise the mapper's save timer, alias response and final synchronous save."""

from types import SimpleNamespace

from amr_interfaces.msg import SemanticObjectArray

import pytest

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rclpy.time import Time

from semantic_slam.object_map import ObjectMap
from semantic_slam.object_map import Observation
from semantic_slam.semantic_slam_node import SemanticSlamNode


def test_reused_point_messages_publish_fresh_independent_serialized_snapshots():
    memory = ObjectMap('lab')
    memory.update([Observation(56, 'chair', 1., [[0., 0., 1.], [.1, 0., 1.], [0., .1, 1.]],
                               [1., 0., 0., 0., 1., 0., 0., 0., 1.])])
    snapshots = []
    node = SimpleNamespace(
        memory=memory, carto=None, p={'map_frame': 'map'},
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1.)),
        pub=SimpleNamespace(publish=lambda msg: snapshots.append(serialize_message(msg))),
        markers=SimpleNamespace(publish=serialize_message))
    SemanticSlamNode._publish(node)
    record = next(iter(memory.objects.values()))
    cached = node._point_messages[record.object_id]
    before = record.points[0][0]
    record.points[0][0] += 2.
    SemanticSlamNode._publish(node)
    assert node._point_messages[record.object_id] is cached
    first, second = [deserialize_message(data, SemanticObjectArray) for data in snapshots]
    assert first.objects[0].points[0].x == pytest.approx(before)
    assert second.objects[0].points[0].x == pytest.approx(before + 2.)
    record.points.append([3., 0., 1.])
    SemanticSlamNode._publish(node)
    assert node._point_messages[record.object_id] is not cached
    assert len(deserialize_message(snapshots[-1], SemanticObjectArray).objects[0].points) == 4
    memory.objects.clear()
    SemanticSlamNode._publish(node)
    assert not node._point_messages
    assert not deserialize_message(snapshots[-1], SemanticObjectArray).objects


@pytest.fixture
def mapper_context(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '83')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('CYCLONEDDS_URI', '')
    path = tmp_path / 'node.json'
    rclpy.init(args=['--ros-args', '-p', 'map_id:=test-map',
                     '-p', f'database_path:={path}'])
    nodes = []

    def create():
        node = SemanticSlamNode.__new__(SemanticSlamNode)
        nodes.append(node)
        node.__init__()
        return node

    yield create, nodes, path
    for node in nodes:
        if hasattr(node, 'memory'):
            node.destroy_node()
        elif hasattr(node, 'handle'):
            if hasattr(node, 'lock'):
                node.lock.close()
            Node.destroy_node(node)
    rclpy.try_shutdown()


def test_one_save_timer_defaults_to_ten_seconds(mapper_context):
    create, _, _ = mapper_context
    node = create()
    assert node.get_parameter('save_period_sec').value == 10.0
    periods = [timer.timer_period_ns for timer in node.timers]
    assert sorted(periods) == [100000000, 1000000000, 10000000000]


def test_live_writer_lock_blocks_a_second_mapper(mapper_context):
    create, _, path = mapper_context
    first = create()
    temporary = path.with_suffix('.json.tmp')
    temporary.write_bytes(b'orphan owned by the lock holder')
    with pytest.raises(RuntimeError, match='Another semantic mapper'):
        create()
    assert first.lock and not first.lock.closed
    assert temporary.read_bytes() == b'orphan owned by the lock holder'
    first.memory.dirty = True
    first.memory.save()
    assert not temporary.exists()


@pytest.mark.parametrize('period', [0.0, -1.0, float('nan'), float('inf'), 3.5])
def test_configured_save_period_is_finite_and_positive(mapper_context, monkeypatch, period):
    create, _, path = mapper_context
    original = Node.get_parameter

    def get_parameter(node, name):
        if name == 'save_period_sec':
            return SimpleNamespace(value=period)
        return original(node, name)

    monkeypatch.setattr(Node, 'get_parameter', get_parameter)
    if period == 3.5:
        node = create()
        assert [t.timer_period_ns for t in node.timers].count(3500000000) == 1
        assert len(list(node.timers)) == 3
    else:
        with pytest.raises(ValueError, match='save_period_sec'):
            create()
        assert not path.exists() and not path.with_suffix('.lock').exists()


@pytest.mark.parametrize('error_type', [OSError, ValueError])
def test_alias_and_shutdown_flush_and_save_errors_keep_dirty(
    mapper_context, monkeypatch, error_type,
):
    create, nodes, path = mapper_context
    node = create()
    points = [[0, 0, 1], [0.1, 0, 1], [0, 0.1, 1]]
    node.memory.update([Observation(56, 'chair', 1.0, points,
                                    [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01])])
    object_id = next(iter(node.memory.objects))
    response = node._alias(SimpleNamespace(object_id=object_id, alias='desk chair'),
                           SimpleNamespace())
    assert response.success and not node.memory.dirty
    assert ObjectMap('test-map', str(path)).objects[object_id].alias == 'desk chair'
    old = path.read_bytes()
    errors = []

    def fail():
        raise error_type('injected persistence failure')

    with monkeypatch.context() as patch:
        patch.setattr(node.memory, 'save', fail)
        patch.setattr(node, 'get_logger', lambda: SimpleNamespace(error=errors.append))
        node.memory.set_alias(object_id, 'pending alias')
        node._save()
        assert errors and node.memory.dirty and path.read_bytes() == old
        response = node._alias(SimpleNamespace(object_id=object_id, alias='final alias'),
                               SimpleNamespace())
        assert not response.success and node.memory.dirty and len(errors) == 2
        assert response.message == 'alias set, not persisted: injected persistence failure'
    node.destroy_node()
    nodes.remove(node)
    assert not node.memory.dirty
    assert ObjectMap('test-map', str(path)).objects[object_id].alias == 'final alias'
    # The shutdown also releases the writer lock.
    replacement = create()
    assert replacement.memory.objects[object_id].alias == 'final alias'


def test_alias_validation_failure_is_distinct_from_persistence_failure(mapper_context):
    create, _, _ = mapper_context
    node = create()
    response = node._alias(SimpleNamespace(object_id='missing', alias='chair'), SimpleNamespace())
    assert not response.success and response.message == 'alias not set: Unknown object ID'
    assert not node.memory.dirty
