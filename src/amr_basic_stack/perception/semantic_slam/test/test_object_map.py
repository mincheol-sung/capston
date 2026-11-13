"""Object identity, ambiguous association, aging, and persistence contracts."""

from types import SimpleNamespace as NS

import numpy as np

import pytest

from semantic_slam.navigation import approach_candidates
from semantic_slam.navigation import at_approach
from semantic_slam.navigation import select_object
from semantic_slam.object_map import ObjectMap
from semantic_slam.object_map import Observation


@pytest.mark.parametrize('offset, scale', [(0., 1.), (10000., 1.), (0., 1e6), (1e16, 1e6)])
def test_fast_voxel_reduction_matches_numeric_row_unique(offset, scale):
    memory = ObjectMap('lab', max_points=512)
    rng = np.random.default_rng(42)
    points = np.concatenate((offset + scale * rng.uniform(-2, 2, (9000, 3)),
                             [[-0., 0., 0.], [0., -0., 0.], [-.05, .05, .1]]))
    _, indices = np.unique(np.floor(points / memory.voxel), axis=0, return_index=True)
    expected = points[np.sort(indices)]
    if len(expected) > memory.max_points:
        expected = expected[np.linspace(0, len(expected) - 1, memory.max_points, dtype=int)]
    assert np.array_equal(memory._bounded_points(points), expected)


def test_voxel_reduction_keeps_first_point_in_dense_duplicate_cells():
    memory = ObjectMap('lab', max_points=512)
    rng = np.random.default_rng(51)
    points = (rng.integers(-3, 4, (8192, 3)) + rng.uniform(.1, .9, (8192, 3))) * .05
    _, indices = np.unique(np.floor(points / memory.voxel), axis=0, return_index=True)
    assert np.array_equal(memory._bounded_points(points), points[np.sort(indices)])
    assert np.array_equal(memory._bounded_points([[0., 0., 0.]] * 5), [[0., 0., 0.]])
    assert memory._bounded_points([]).shape == (0, 3)


def test_batch_relations_preserve_pair_order_status_filter_and_distance():
    memory = ObjectMap('lab')
    rng = np.random.default_rng(82)
    records = [NS(object_id=str(i), position=position.tolist(), count=i % 5, status=i % 4)
               for i, position in enumerate(rng.uniform(-2., 2., (40, 3)))]
    memory.objects = {record.object_id: record for record in records}
    selected = [record for record in records if record.count >= 3 and record.status != 3]
    expected = []
    for i, first in enumerate(selected):
        for second in selected[i + 1:]:
            distance = np.linalg.norm(np.array(first.position[:2]) - second.position[:2])
            if distance <= 2.:
                expected.append((first.object_id, second.object_id, 'near', distance))
    actual = memory.relations()
    assert [row[:3] for row in actual] == [row[:3] for row in expected]
    assert [row[3] for row in actual] == pytest.approx([row[3] for row in expected], abs=1e-12)
    assert memory.relations(-1.) == []


def observation(x=0, y=0, stamp=1, cls=56):
    points = [
        [x + dx, y + dy, 0.5 + dz]
        for dx in (-0.1, 0.0, 0.1)
        for dy in (-0.1, 0.1)
        for dz in (0.0, 0.1)
    ]
    return Observation(
        cls,
        'chair' if cls == 56 else 'dining table',
        stamp,
        points,
        (np.eye(3) * 0.03).ravel().tolist(),
    )


def test_two_chairs_revisit_persist_and_ambiguous_observation(tmp_path):
    path = str(tmp_path / 'objects.json')
    memory = ObjectMap('lab-v1', path, gate=1.2, ambiguity_margin=0.2)
    for stamp in (1, 2, 3):
        ids = memory.update([observation(0, stamp=stamp), observation(1, stamp=stamp)])
        assert len(ids) == 2
    original = set(memory.objects)
    assert all(r.status == 1 and r.count == 3 for r in memory.objects.values())
    assert memory.update([observation(0.5, stamp=4)]) == []
    assert set(memory.objects) == original
    object_id = sorted(original)[0]
    memory.set_alias(object_id, 'window chair')
    memory.save()
    loaded = ObjectMap('lab-v1', path)
    assert set(loaded.objects) == original
    assert loaded.objects[object_id].alias == 'window chair'
    assert loaded.relations()[0][2] == 'near'
    with pytest.raises(ValueError, match='mismatch'):
        ObjectMap('lab-v2', path)
    loaded.age(100)
    assert len(loaded.objects) == 2
    assert all(r.status == 2 for r in loaded.objects.values())


def test_duplicate_out_of_order_capacity_and_moving_object():
    memory = ObjectMap('lab', max_objects=1, max_points=10)
    memory.update([observation(stamp=1)])
    record = next(iter(memory.objects.values()))
    memory.update([observation(stamp=1), observation(stamp=0.8), observation(5, stamp=2)])
    assert len(memory.objects) == 1 and record.count == 1
    assert len(record.points) <= 10
    memory.update([observation(0.3, stamp=1.5)])
    memory.update([observation(0.6, stamp=2.0)])
    assert record.status == 3


def test_alias_query_requires_disambiguation_and_near_reference():
    def record(i, x, alias=''):
        return NS(object_id=i, class_name='chair', alias=alias, status=1, position=NS(x=x, y=0))

    chairs = [record('a', 0, 'window'), record('b', 1), record('c', 6)]
    with pytest.raises(ValueError, match='Ambiguous'):
        select_object(chairs, class_name='chair')
    assert select_object(chairs, alias='window').object_id == 'a'
    assert select_object(chairs, class_name='chair', near_object_id='a').object_id == 'b'
    chairs[1].status = 3
    with pytest.raises(LookupError):
        select_object(chairs, object_id='b')


def test_candidates_outside_box_and_arrival_requires_heading_and_stop():
    candidates = approach_candidates([2, 2, 0.5], [1, 0.6, 1], 0.8)
    assert len(candidates) == 16
    for x, y, angle in candidates:
        distance = np.linalg.norm(np.maximum(np.abs(np.array([x, y]) - [2, 2]) - [0.5, 0.3], 0))
        assert distance >= 0.5
        assert np.allclose(
            [np.cos(angle), np.sin(angle)],
            (np.array([2, 2]) - [x, y]) / np.linalg.norm(np.array([2, 2]) - [x, y]),
        )
    assert at_approach((1, 1, 0), (1, 1, 0), 0, 0)
    assert not at_approach((1, 1, 1), (1, 1, 0), 0, 0)
    assert not at_approach((1, 1, 0), (1, 1, 0), 0.2, 0)


def test_new_nearby_chair_can_be_registered_on_a_later_visit():
    """A new surface does not disappear merely because a nearby chair is saved."""
    memory = ObjectMap('lab-v1')
    memory.update([observation(0.0, stamp=1)])
    first = set(memory.objects)
    memory.update([observation(0.65, stamp=2)])
    assert len(memory.objects) == 2 and first < set(memory.objects)
    for stamp in (3, 4):
        memory.update([observation(0.0, stamp=stamp), observation(0.65, stamp=stamp)])
    assert all(record.status == 1 for record in memory.objects.values())
    memory.update([observation(0.325, stamp=5)])
    assert len(memory.objects) == 2 and memory.ambiguous_count == 1


def _chair(rng, center):
    half = np.array([0.225, 0.225, 0.4])
    return (np.asarray(center) + rng.uniform(-half, half, size=(300, 3))).tolist()


def _observe_static_chair(memory, rng, map_offsets, start=100.0):
    """Observe a chair fixed at odom (3, 1) at 2 Hz while map->odom shifts by x offsets."""
    cov = (np.eye(3) * 0.03 ** 2).ravel().tolist()
    truth = np.array([3.0, 1.0, 0.4])
    for k, offset in enumerate(map_offsets):
        points = _chair(rng, truth)
        memory.update([Observation(
            56, 'chair', start + 0.5 * k, (np.asarray(points) + [offset, 0, 0]).tolist(),
            cov, 0.9, 0.9, odom_position=np.median(points, axis=0).tolist())])


def test_map_correction_jump_beyond_gate_keeps_one_object():
    # 1.14 m: largest map->odom step measured in the 2026-09-11 bag, beyond gate 0.8.
    memory = ObjectMap('lab', confirm_count=3)
    _observe_static_chair(memory, np.random.default_rng(3), [0.0] * 5 + [1.14] * 5)
    (record,) = memory.objects.values()
    assert record.status == 1 and record.movement_votes == 0
    # Re-anchored at the latest map alignment, without blending both alignments.
    assert abs(record.position[0] - 4.14) < 0.05
    assert record.extent[0] < 0.6


def test_map_correction_oscillation_inside_gate_is_not_movement():
    memory = ObjectMap('lab', confirm_count=3)
    _observe_static_chair(memory, np.random.default_rng(4),
                          [0.0] * 4 + [0.5, 0.0, 0.5, 0.0] + [0.0] * 4)
    (record,) = memory.objects.values()
    assert record.status == 1 and record.movement_votes == 0


def test_odometry_motion_still_marks_moving_and_window_falls_back_to_map():
    rng = np.random.default_rng(5)
    cov = (np.eye(3) * 0.03 ** 2).ravel().tolist()
    memory = ObjectMap('lab', confirm_count=3)
    _observe_static_chair(memory, rng, [0.0] * 4)
    # The chair itself is dragged 0.5 m in odometry; the map is steady.
    for k in range(3):
        center = np.array([3.0 + 0.5 * (k + 1), 1.0, 0.4])
        memory.update([Observation(56, 'chair', 102.5 + 0.5 * k, _chair(rng, center), cov,
                                   0.9, 0.9, odom_position=center.tolist())])
    (record,) = memory.objects.values()
    assert record.status == 3
    # Past the window the stored odometry position is not trusted.
    obs = Observation(56, 'chair', 200.0, _chair(rng, [4.5, 1.0, 0.4]), cov, 0.9, 0.9,
                      odom_position=[0.0, 0.0, 0.0])
    assert memory._odom_motion(record.object_id, obs) is None


def test_odometry_positions_are_not_persisted(tmp_path):
    path = tmp_path / 'objects.json'
    memory = ObjectMap('lab', str(path), confirm_count=3)
    _observe_static_chair(memory, np.random.default_rng(6), [0.0] * 4)
    assert memory.odom_positions
    memory.save()
    assert ObjectMap('lab', str(path), confirm_count=3).odom_positions == {}


def _seen(memory, rng, center, stamps, class_id=56, odom_shift=0.0):
    """Feed one chair at map ``center``; odometry is the map shifted by ``odom_shift``."""
    cov = (np.eye(3) * 0.03 ** 2).ravel().tolist()
    for stamp in stamps:
        points = _chair(rng, center)
        memory.update([Observation(
            class_id, 'chair', stamp, points, cov, 0.9, 0.9,
            odom_position=(np.median(points, axis=0) - [odom_shift, 0, 0]).tolist())])


def test_jump_while_out_of_view_merges_duplicate_into_original_id():
    # Seen at map x=3, unseen for 30 s (past the odometry window) while map->odom
    # stepped 1.14 m, then seen again at x=4.14: one object, original ID.
    memory = ObjectMap('lab', confirm_count=3)
    rng = np.random.default_rng(7)
    _seen(memory, rng, [3.0, 1.0, 0.4], [100.0 + 0.5 * k for k in range(5)])
    (original,) = memory.objects
    memory.set_alias(original, 'desk chair')
    _seen(memory, rng, [4.14, 1.0, 0.4], [132.0 + 0.5 * k for k in range(5)], odom_shift=1.14)
    assert list(memory.objects) == [original]
    record = memory.objects[original]
    assert abs(record.position[0] - 4.14) < 0.05 and record.extent[0] < 0.6
    assert record.alias == 'desk chair' and record.count == 10 and record.status == 1


def test_chairs_seen_in_one_frame_are_never_merged():
    memory = ObjectMap('lab', confirm_count=3)
    rng = np.random.default_rng(8)
    cov = (np.eye(3) * 0.03 ** 2).ravel().tolist()
    for k in range(5):
        memory.update([Observation(56, 'chair', 100.0 + 0.5 * k, _chair(rng, [x, 1.0, 0.4]),
                                   cov, 0.9, 0.9) for x in (3.0, 4.2)])
    _seen(memory, rng, [3.0, 1.0, 0.4], [140.0 + 0.5 * k for k in range(4)])
    assert len(memory.objects) == 2


def test_chairs_seen_alternately_are_never_merged():
    # Lifetimes overlap: neither stopped being seen when the other appeared.
    memory = ObjectMap('lab', confirm_count=3)
    rng = np.random.default_rng(9)
    for k in range(4):
        _seen(memory, rng, [3.0, 1.0, 0.4], [100.0 + 10 * k])
        _seen(memory, rng, [4.2, 1.0, 0.4], [105.0 + 10 * k])
    assert len(memory.objects) == 2


def test_distinct_aliases_and_ambiguous_originals_block_merge():
    memory = ObjectMap('lab', confirm_count=3)
    rng = np.random.default_rng(10)
    _seen(memory, rng, [3.0, 1.0, 0.4], [100.0 + 0.5 * k for k in range(4)])
    _seen(memory, rng, [3.0, 2.6, 0.4], [110.0 + 0.5 * k for k in range(4)])
    # Both older chairs are 1.2 m from the new one, inside merge distance: do not guess.
    _seen(memory, rng, [3.9, 1.8, 0.4], [150.0 + 0.5 * k for k in range(4)])
    assert len(memory.objects) == 3
    memory = ObjectMap('lab', confirm_count=3)
    _seen(memory, rng, [3.0, 1.0, 0.4], [100.0 + 0.5 * k for k in range(4)])
    memory.set_alias(next(iter(memory.objects)), 'left')
    _seen(memory, rng, [4.14, 1.0, 0.4], [150.0 + 0.5 * k for k in range(2)])
    memory.set_alias(next(reversed(memory.objects)), 'right')
    _seen(memory, rng, [4.14, 1.0, 0.4], [151.0 + 0.5 * k for k in range(3)])
    assert len(memory.objects) == 2


def test_first_seen_persists_and_merge_can_be_disabled(tmp_path):
    path = tmp_path / 'objects.json'
    rng = np.random.default_rng(11)
    memory = ObjectMap('lab', str(path), confirm_count=3)
    _seen(memory, rng, [3.0, 1.0, 0.4], [100.0 + 0.5 * k for k in range(4)])
    memory.save()
    reloaded = ObjectMap('lab', str(path), confirm_count=3)
    assert next(iter(reloaded.objects.values())).first_seen == 100.0
    # A restart cannot reuse odometry, so the merge is what prevents the duplicate.
    _seen(reloaded, rng, [4.14, 1.0, 0.4], [300.0 + 0.5 * k for k in range(4)])
    assert len(reloaded.objects) == 1
    disabled = ObjectMap('lab', str(path), confirm_count=3, merge_distance=0.0)
    _seen(disabled, rng, [4.14, 1.0, 0.4], [400.0 + 0.5 * k for k in range(4)])
    assert len(disabled.objects) == 2
