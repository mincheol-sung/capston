"""Partial graph replies and late responses must never overwrite good geometry."""

from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np

import semantic_slam.cartographer_adapter as adapter_module
from semantic_slam.cartographer_adapter import CartographerAdapter
from semantic_slam.object_map import ObjectMap
from semantic_slam.object_map import Observation
from semantic_slam.pose_graph import NodePose
from semantic_slam.pose_graph import PoseGraph


def setup_adapter(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(adapter_module, 'time', NS(monotonic=lambda: clock[0]))
    states, query = Mock(), Mock()
    states.service_is_ready.return_value = query.service_is_ready.return_value = True
    state_future, query_first, query_second = Future(), Future(), Future()
    states.call_async.return_value = state_future
    query.call_async.side_effect = [query_first, query_second]
    node = Mock()
    node.create_client.side_effect = [states, query]
    node.get_clock.return_value.now.return_value.nanoseconds = 100000000000
    graph = PoseGraph('session')
    graph.active = 1
    graph.replace(1, [NodePose(t, [0, 0, 0], [0, 0, 0, 1]) for t in (1, 1.5)])
    anchor, points, cov = graph.bind(1.2, [[2, 0, 0], [2, 0.1, 0], [2, 0, 0.1]], np.eye(3))
    memory = ObjectMap('lab', graph_id='session')
    memory.update([Observation(56, 'chair', 1.2, points.tolist(), cov.ravel().tolist(),
                               anchor=anchor)])
    parameters = {
        'cartographer_graph_id': 'session', 'graph_max_interpolation_gap': 1.0,
        'graph_max_nodes': 100, 'cartographer_states_service': '/get_trajectory_states',
        'cartographer_query_service': '/trajectory_query', 'graph_poll_period': 1.0,
        'graph_service_timeout': 3.0, 'graph_max_trajectories': 8, 'graph_freshness': 3.0,
        'cartographer_trajectory_id': -1, 'map_frame': 'map', 'graph_node_timeout': 5.0,
    }
    adapter = CartographerAdapter(node, memory, parameters)
    adapter.graph, adapter.valid, adapter.last_success = graph, True, 0.0
    return adapter, clock, states, state_future, query_first, query_second


def states_reply(ids, values):
    return NS(status=NS(code=0), trajectory_states=NS(trajectory_id=ids, trajectory_state=values))


def test_timeout_cancels_late_request_without_committing_graph(monkeypatch):
    adapter, clock, client, future, _, _ = setup_adapter(monkeypatch)
    original = adapter.graph
    adapter.tick()
    clock[0] = 4.0
    adapter.tick()
    assert not adapter.ready and adapter.graph is original
    assert future.cancelled()
    client.remove_pending_request.assert_called_once_with(future)
    assert adapter.memory.revision == 0


def test_partial_multi_trajectory_reply_is_not_committed(monkeypatch):
    adapter, _, _, states, first, second = setup_adapter(monkeypatch)
    original = adapter.graph
    position = list(next(iter(adapter.memory.objects.values())).position)
    adapter.tick()
    states.set_result(states_reply([1, 2], [1, 0]))
    adapter.tick()
    from geometry_msgs.msg import PoseStamped
    poses = []
    for stamp in (1, 1.5):
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp.sec = 'map', int(stamp)
        pose.header.stamp.nanosec = int((stamp % 1) * 1000000000)
        pose.pose.position.x, pose.pose.orientation.w = 10.0, 1.0
        poses.append(pose)
    first.set_result(NS(status=NS(code=0), trajectory=poses))
    adapter.tick()
    second.set_result(NS(status=NS(code=13)))
    adapter.tick()
    assert not adapter.ready and adapter.graph is original
    assert adapter.memory.revision == 0
    assert next(iter(adapter.memory.objects.values())).position == position


def test_ambiguous_active_trajectories_require_explicit_selection(monkeypatch):
    adapter, _, _, future, _, _ = setup_adapter(monkeypatch)
    adapter.tick()
    future.set_result(states_reply([1, 2], [0, 0]))
    adapter.tick()
    assert not adapter.ready
    assert adapter.error == 'Exactly one active trajectory is required'


def trajectory_reply(x):
    from geometry_msgs.msg import PoseStamped
    poses = []
    for stamp in (1, 1.5):
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp.sec = 'map', int(stamp)
        pose.header.stamp.nanosec = int((stamp % 1) * 1000000000)
        pose.pose.position.x, pose.pose.orientation.w = x, 1.0
        poses.append(pose)
    return NS(status=NS(code=0), trajectory=poses)


def test_finished_trajectory_final_optimization_reaches_objects(monkeypatch):
    """ACTIVE -> FINISHED triggers a last optimization; anchors must still follow it."""
    adapter, _, _, states, first, _ = setup_adapter(monkeypatch)
    before = list(next(iter(adapter.memory.objects.values())).position)
    adapter.tick()
    states.set_result(states_reply([1], [1]))
    adapter.tick()
    first.set_result(trajectory_reply(10.0))
    adapter.tick()
    record = next(iter(adapter.memory.objects.values()))
    assert record.position[0] == before[0] + 10.0
    assert adapter.memory.revision == 1 and adapter.memory.dirty
    # Correction is applied, but no new observation or mission may use this graph.
    assert not adapter.ready and adapter.graph.active is None
    assert adapter.error.startswith('No active trajectory')


def test_no_active_and_no_anchors_fails_closed(monkeypatch):
    adapter, _, _, states, _, _ = setup_adapter(monkeypatch)
    adapter.memory.objects.clear()
    original = adapter.graph
    adapter.tick()
    states.set_result(states_reply([1], [1]))
    adapter.tick()
    assert not adapter.ready and adapter.graph is original
    assert adapter.error == 'No active trajectory and no anchored objects to reproject'


def test_deleted_trajectory_anchors_do_not_block_mapping(monkeypatch):
    """Anchors on a DELETED trajectory are pruned instead of exhausting capacity."""
    adapter, _, _, states, first, _ = setup_adapter(monkeypatch)
    adapter.p['graph_max_trajectories'] = 1
    adapter.node.get_clock.return_value.now.return_value.nanoseconds = 1500000000
    record = next(iter(adapter.memory.objects.values()))
    record.anchors.append(dict(record.anchors[0], trajectory_id=5))
    adapter.tick()
    states.set_result(states_reply([1, 5], [0, 3]))
    adapter.tick()
    first.set_result(trajectory_reply(0.0))
    adapter.tick()
    assert adapter.ready and adapter.error == ''
    assert [a['trajectory_id'] for a in record.anchors] == [1]
    assert record.support_valid


def test_capacity_keeps_active_and_most_recent_trajectories(monkeypatch):
    adapter, _, _, states, _, _ = setup_adapter(monkeypatch)
    adapter.p['graph_max_trajectories'] = 2
    record = next(iter(adapter.memory.objects.values()))
    anchor = record.anchors[0]
    record.anchors = [dict(anchor, trajectory_id=1, stamp=1.0),
                      dict(anchor, trajectory_id=2, stamp=1.4)]
    adapter.tick()
    states.set_result(states_reply([1, 2, 3], [2, 2, 0]))
    adapter.tick()
    assert {adapter.current, *adapter.queue} == {2, 3}
    assert adapter.staged == {1: []}
