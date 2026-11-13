"""Loop-closure correction must preserve identity and raw evidence, not drift."""

import json
import math
from unittest.mock import Mock

import numpy as np

import pytest

from semantic_slam.object_map import ObjectMap
from semantic_slam.object_map import Observation
from semantic_slam.pose_graph import FrozenPoints
from semantic_slam.pose_graph import NodePose
from semantic_slam.pose_graph import PoseGraph


def node(stamp, x=0.0, angle=0.0):
    return NodePose(stamp, [x, 0.0, 0.0], [0.0, 0.0, math.sin(angle/2), math.cos(angle/2)])


def graph():
    result = PoseGraph('saved-session-1')
    result.active = 1
    result.replace(1, [node(t) for t in (1.0, 1.5, 2.0, 2.5, 3.0)])
    return result


def test_raw_evidence_array_reuse_is_immutable_and_json_compatible():
    source = [[.1, .2, .3], [.4, .5, .6], [.7, .8, .9]]
    encoded = json.dumps(source)
    points = FrozenPoints(source)
    source[0][0] = 99.
    assert json.dumps(points) == encoded
    assert points == json.loads(encoded)
    assert not points != json.loads(encoded)
    assert FrozenPoints(points) is points
    view = np.asarray(points)
    assert np.asarray(points) is view
    with pytest.raises(TypeError):
        points[0][0] = 99.
    with pytest.raises(ValueError):
        view[0][0] = 99.
    with pytest.raises(ValueError):
        view.setflags(write=True)
    assert json.dumps(points) == encoded


def observe(g, stamp, x=2.0):
    points = [[x+dx, dy, z] for dx in (-0.1, 0.1) for dy in (-0.1, 0.1) for z in (0.4, 0.6)]
    anchor, points, cov = g.bind(stamp, points, np.diag([0.01, 0.04, 0.09]))
    return Observation(56, 'chair', stamp, points.tolist(), cov.ravel().tolist(), anchor=anchor)


def test_shared_anchor_pose_cache_preserves_independent_geometry_and_evidence():
    g = graph()
    memory = ObjectMap('lab', graph_id=g.graph_id)
    memory.update([observe(g, 1.1, x=2.), observe(g, 1.1, x=5.)])
    raw = json.dumps([r.anchors for r in memory.objects.values()])
    g.replace(1, [node(t, 1.) for t in (1., 1.5, 2., 2.5, 3.)])
    g.pose = Mock(wraps=g.pose)
    assert memory.reproject(g)
    assert g.pose.call_count == 1
    assert sorted(r.position[0] for r in memory.objects.values()) == [3., 6.]
    # Optimization changes the resolved pose, never the retained sensor points.
    before_points = [a['points'] for r in json.loads(raw) for a in r]
    assert [a['points'] for r in memory.objects.values() for a in r.anchors] == before_points
    assert not memory.reproject(g)
    g.replace(1, [node(t, 2.) for t in (1., 1.5, 2., 2.5, 3.)])
    assert memory.reproject(g)
    assert sorted(r.position[0] for r in memory.objects.values()) == [4., 7.]


@pytest.mark.parametrize('bad', [
    NodePose(4., [0., 0.], [0., 0., 0., 1.]),
    NodePose(4., [0., math.nan, 0.], [0., 0., 0., 1.]),
    NodePose(4., [0., 0., 0.], [0., 0., 0., 0.]),
    NodePose(4., [0., 0., 0.], [0., 0., math.inf, 1.]),
    NodePose(0., [0., 0., 0.], [0., 0., 0., 1.]),
])
def test_batch_validation_keeps_previous_snapshot_on_invalid_node(bad):
    g = graph()
    previous = g.nodes[1]
    with pytest.raises(ValueError):
        g.replace(1, [node(3.5), bad])
    assert g.nodes[1] is previous
    assert g.stamps[1] == [1., 1.5, 2., 2.5, 3.]


def test_batch_validation_rejects_duplicates_and_accepts_nonunit_quaternion():
    g = graph()
    with pytest.raises(ValueError):
        g.replace(1, [node(4.), node(4.)])
    g.replace(1, [NodePose(4., [0., 0., 0.], [0., 0., 0., 2.])])
    assert g.stamps[1] == [4.]


def test_tolerance_boundary_and_pose_storage_are_independent():
    g = graph()
    memory = ObjectMap('lab', graph_id=g.graph_id)
    memory.update([observe(g, 1.1, x=2.), observe(g, 1.1, x=5.)])
    memory.reproject(g)
    g.replace(1, [node(t, .5e-6) for t in (1., 1.5, 2., 2.5, 3.)])
    assert not memory.reproject(g)
    g.replace(1, [node(t, 2e-6) for t in (1., 1.5, 2., 2.5, 3.)])
    assert memory.reproject(g)
    records = list(memory.objects.values())
    assert records[0].anchors[0]['pose'] is not records[1].anchors[0]['pose']


def test_optimized_translation_rotation_rebuilds_same_id_without_fake_motion(tmp_path):
    g = graph()
    memory = ObjectMap('lab', str(tmp_path/'db.json'), graph_id=g.graph_id)
    for stamp in (1.1, 1.6, 2.1):
        memory.update([observe(g, stamp)])
        memory.reproject(g)
    object_id = next(iter(memory.objects))
    memory.set_alias(object_id, 'window chair')
    before = memory.objects[object_id]
    raw = json.dumps(before.anchors[0]['points'])
    g.replace(1, [node(t, 1.0, math.pi/2) for t in (1.0, 1.5, 2.0, 2.5, 3.0)])
    assert memory.reproject(g)
    after = memory.objects[object_id]
    assert np.allclose(after.position, [1.0, 2.0, 0.5])
    assert np.allclose(np.diag(np.asarray(after.covariance).reshape(3, 3)), [0.04, 0.01, 0.09])
    assert after.alias == 'window chair' and after.last_seen == 2.1
    assert after.count == 3 and after.status == 1 and after.movement_votes == 0
    assert json.dumps(after.anchors[0]['points']) == raw
    assert memory.revision == 1
    projected = np.array(after.points)
    assert not memory.reproject(g)
    assert np.allclose(after.points, projected) and memory.revision == 1
    memory.save()
    loaded = ObjectMap('lab', str(tmp_path/'db.json'), graph_id=g.graph_id)
    assert loaded.revision == 1 and loaded.objects[object_id].alias == 'window chair'
    assert not loaded.reproject(g)
    with pytest.raises(ValueError, match='session mismatch'):
        ObjectMap('lab', str(tmp_path/'db.json'), graph_id='new-session')


def test_each_observation_uses_its_own_nodes_and_each_trajectory_its_own_correction():
    g = graph()
    memory = ObjectMap('lab', graph_id=g.graph_id)
    first = memory.update([observe(g, 1.1)])[0]
    g.active = 2
    g.replace(2, [node(t) for t in (1.0, 1.5)])
    second = memory.update([observe(g, 1.1, x=5.0)])[0]
    g.replace(1, [node(t, 0.4) for t in (1.0, 1.5, 2.0, 2.5, 3.0)])
    g.replace(2, [node(t, -0.2) for t in (1.0, 1.5)])
    memory.reproject(g)
    assert np.allclose(memory.objects[first].position, [2.4, 0, 0.5])
    assert np.allclose(memory.objects[second].position, [4.8, 0, 0.5])


def test_nonuniform_correction_interpolates_nodes_and_shortest_rotation():
    g = graph()
    obs = observe(g, 1.25)
    g.replace(1, [node(1.0, 0.0), node(1.5, 1.0)])
    points, _, _ = g.project(obs.anchor)
    assert np.allclose(np.median(points, axis=0), [2.5, 0, 0.5])
    g.replace(1, [node(1.0, 0, math.radians(179)), node(1.5, 0, math.radians(-179))])
    points, _, _ = g.project(obs.anchor)
    assert np.allclose(np.median(points, axis=0), [-2, 0, 0.5], atol=1e-8)


def test_discontinuous_snapshot_invalidates_instead_of_applying_current_pose():
    """A restarted trajectory (no overlap with the last snapshot) is not trimming."""
    g = graph()
    memory = ObjectMap('lab', graph_id=g.graph_id)
    for stamp in (1.1, 1.6, 2.1):
        memory.update([observe(g, stamp)])
    record = next(iter(memory.objects.values()))
    position = list(record.position)
    g.replace(1, [node(10.0, 100), node(10.5, 100)])
    assert memory.reproject(g)
    assert record.position == position and record.status == 2 and not record.support_valid
    assert memory.revision == 1
    # Polling the restarted trajectory again must not turn old evidence into trimming.
    g.replace(1, [node(10.0, 100), node(10.5, 100)])
    assert not memory.reproject(g)
    assert not any(a.get('frozen') for a in record.anchors) and record.status == 2
    g.replace(1, [node(t, 0.1) for t in (1.0, 1.5, 2.0, 2.5, 3.0)])
    assert memory.reproject(g) and record.support_valid and record.status == 1
    assert memory.revision == 2


def test_front_trimmed_nodes_freeze_their_last_optimized_pose(tmp_path):
    """pure_localization_trimmer drops old nodes; their last pose is final, not invalid."""
    g = graph()
    memory = ObjectMap('lab', str(tmp_path / 'frozen.json'), graph_id=g.graph_id)
    for stamp in (1.1, 1.6, 2.1):
        memory.update([observe(g, stamp)])
    record = next(iter(memory.objects.values()))
    g.replace(1, [node(t, 0.2) for t in (1.0, 1.5, 2.0, 2.5, 3.0)])
    assert memory.reproject(g)
    corrected = list(record.position)
    assert np.isclose(corrected[0], 2.2)
    # Later nodes keep moving, but the trimmed evidence keeps its final pose.
    g.replace(1, [node(2.5, 100), node(3.0, 100), node(3.5, 100)])
    assert not memory.reproject(g)
    assert record.position == corrected and record.support_valid and record.status == 1
    assert all(a['frozen'] for a in record.anchors) and memory.dirty
    assert memory.revision == 1
    memory.save()
    memory = ObjectMap('lab', str(memory.path), graph_id=g.graph_id)
    record = next(iter(memory.objects.values()))
    assert all(a['frozen'] for a in record.anchors) and record.position == corrected
    # A deleted trajectory voids even frozen evidence.
    g.replace(1, [])
    assert memory.reproject(g) and not record.support_valid and record.status == 2


def test_restart_voids_previously_frozen_evidence_but_not_later_evidence(tmp_path):
    g = graph()
    memory = ObjectMap('lab', str(tmp_path / 'restart.json'), graph_id=g.graph_id)
    for stamp in (1.1, 1.6, 2.1):
        memory.update([observe(g, stamp)])
    record = next(iter(memory.objects.values()))
    g.replace(1, [node(2.5), node(3.0), node(3.5)])
    memory.reproject(g)
    assert all(a['frozen'] for a in record.anchors) and record.support_valid
    memory.save()
    memory = ObjectMap('lab', str(memory.path), graph_id=g.graph_id)
    record = next(iter(memory.objects.values()))
    g.replace(1, [node(10.0), node(10.5), node(11.0)])
    assert memory.reproject(g) and not record.support_valid and record.status == 2
    # Evidence trimmed within the new epoch is frozen and valid again.
    fresh = ObjectMap('lab', graph_id=g.graph_id)
    fresh.update([observe(g, 10.2)])
    g.replace(1, [node(10.5), node(11.0)])
    fresh.reproject(g)
    other = next(iter(fresh.objects.values()))
    assert other.anchors[0]['frozen'] and other.support_valid


def test_middle_gap_is_not_trimming():
    g = graph()
    memory = ObjectMap('lab', graph_id=g.graph_id)
    for stamp in (1.1, 1.6):
        memory.update([observe(g, stamp)])
    record = next(iter(memory.objects.values()))
    g.replace(1, [node(1.0), node(1.5), node(2.5), node(3.0)])
    assert memory.reproject(g) and not record.support_valid and record.status == 2
    assert not any(a.get('frozen') for a in record.anchors)


def test_returning_to_an_object_after_trimming_recovers_immediately():
    g = PoseGraph('s')
    g.active = 1
    g.replace(1, [node(t / 10) for t in range(10, 211)])
    memory = ObjectMap('lab', graph_id='s', confirm_count=3)
    for stamp in (1.5, 2.0, 2.5, 3.0):
        memory.update([observe(g, stamp)])
        memory.reproject(g)
    record = next(iter(memory.objects.values()))
    assert record.status == 1
    # Away for a while: the trimmer keeps only the newest nodes.
    g.replace(1, [node(t / 10) for t in range(150, 421)])
    memory.reproject(g)
    assert record.support_valid and record.status == 1
    memory.update([observe(g, 40.0)])
    memory.reproject(g)
    assert record.support_valid and record.status == 1 and record.count == 5


def test_no_extrapolation_gap_session_validation_and_atomic_invalid_snapshot():
    g = graph()
    for stamp in (0.9, 3.1):
        with pytest.raises(LookupError):
            observe(g, stamp)
    g.replace(1, [node(1.0), node(3.0)])
    with pytest.raises(LookupError, match='gap'):
        observe(g, 2.0)
    with pytest.raises(ValueError):
        g.replace(1, [node(1.0), node(1.0)])
    assert len(g.nodes[1]) == 2
    with pytest.raises(ValueError):
        g.replace(1, [NodePose(1, [0, 0, 0], [0, 0, 0, 0])])
    bad = observe(g, 1.0).anchor
    bad['graph_id'] = 'different'
    with pytest.raises(ValueError, match='mismatch'):
        g.project(bad)


def test_retained_raw_evidence_is_bounded_and_legacy_db_cannot_be_corrected(tmp_path):
    g = graph()
    memory = ObjectMap('lab', graph_id=g.graph_id, max_anchors=3)
    for stamp in (1.0, 1.4, 1.8, 2.2, 2.6):
        memory.update([observe(g, stamp)])
        memory.reproject(g)
    record = next(iter(memory.objects.values()))
    assert len(record.anchors) == 3 and record.count == 5
    assert len(record.points) <= 512
    legacy = ObjectMap('lab', str(tmp_path/'legacy.json'))
    plain = observe(g, 1.0)
    plain.anchor = None
    legacy.update([plain])
    legacy.save()
    with pytest.raises(ValueError, match='session mismatch'):
        ObjectMap('lab', str(tmp_path/'legacy.json'), graph_id=g.graph_id)
