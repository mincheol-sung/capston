"""OccupancyGrid 거리장과 STATIC 트랙 억제 정책 테스트."""
from types import SimpleNamespace

import numpy as np
import pytest

from object_tracker.core.grid_filter import (
    GridFilterConfig,
    OccupancyDistanceGrid,
    StaticTrackSuppressor,
)

STATIC = 1
DYNAMIC = 2


def make_track(track_id, motion):
    return SimpleNamespace(id=track_id, motion=motion)


def make_grid(data, width=5, height=4, resolution=0.1,
              origin=(0.0, 0.0, 0.0), threshold=65):
    grid = OccupancyDistanceGrid()
    grid.update(data, width, height, resolution, origin[0], origin[1],
                origin[2], 'map', threshold)
    return grid


def test_distance_to_occupied_cell_and_unknown_is_not_wall():
    data = np.full(20, -1, dtype=np.int8)
    data[1 * 5 + 2] = 100
    grid = make_grid(data)

    distances = grid.distances([
        [0.25, 0.15],       # occupied cell (2,1)
        [0.35, 0.15],       # one cell away
        [10.0, 10.0],       # outside is fail-open
    ])
    assert distances[0] == pytest.approx(0.0)
    assert distances[1] == pytest.approx(0.1)
    assert np.isinf(distances[2])


def test_rotated_origin_is_applied_before_cell_lookup():
    data = np.zeros(20, dtype=np.int8)
    data[1 * 5 + 2] = 100
    # Grid local cell center (0.25, 0.15), rotated +90 degrees and translated.
    grid = make_grid(data, origin=(1.0, 2.0, np.pi / 2.0))
    point = [1.0 - 0.15, 2.0 + 0.25]
    assert grid.distances([point])[0] == pytest.approx(0.0)


def test_empty_map_has_infinite_distance():
    grid = make_grid(np.zeros(20, dtype=np.int8))
    assert np.isinf(grid.distances([[0.1, 0.1]])[0])


def test_static_requires_consecutive_near_frames():
    cfg = GridFilterConfig(
        enter_distance=0.15, exit_distance=0.20, confirm_frames=3)
    suppressor = StaticTrackSuppressor(cfg)
    track = make_track(7, STATIC)

    for _ in range(2):
        kept, suppressed = suppressor.select([track], [0.10], STATIC)
        assert kept == [track]
        assert suppressed == []
    kept, suppressed = suppressor.select([track], [0.10], STATIC)
    assert kept == []
    assert suppressed == [track]


def test_hysteresis_and_dynamic_immediate_release():
    cfg = GridFilterConfig(
        enter_distance=0.15, exit_distance=0.20, confirm_frames=1)
    suppressor = StaticTrackSuppressor(cfg)
    track = make_track(3, STATIC)

    assert suppressor.select([track], [0.10], STATIC)[1] == [track]
    # enter 밖이어도 exit 안이면 계속 억제한다.
    assert suppressor.select([track], [0.18], STATIC)[1] == [track]
    # 움직이기 시작하면 벽 가까이에서도 즉시 통과시킨다.
    track.motion = DYNAMIC
    assert suppressor.select([track], [0.0], STATIC)[0] == [track]


def test_grid_filter_config_validation():
    with pytest.raises(ValueError):
        GridFilterConfig(enter_distance=0.2, exit_distance=0.1).validate()
    with pytest.raises(ValueError):
        GridFilterConfig(confirm_frames=0).validate()


def test_default_node_policy_keeps_stationary_semantic_objects_near_walls():
    from object_tracker.object_tracker_node import ObjectTrackerNode

    tracks = [make_track(1, STATIC)]
    node = SimpleNamespace(grid_cfg=GridFilterConfig())
    assert not node.grid_cfg.enabled
    # Disabled filter must not even require a map or a TF lookup.
    assert ObjectTrackerNode._filter_tracks(node, tracks, None) is tracks
