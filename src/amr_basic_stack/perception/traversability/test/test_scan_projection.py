"""LaserScan → 셀 인덱스 투영 테스트. 특히 row 순서(상하 반전) 검증."""

import math

import numpy as np

from traversability import scan_projection


def test_world_to_flat_cells_uses_occupancy_grid_row_order():
    """Row 0 은 최소 y 다. 이미지처럼 뒤집히면 안 된다."""
    width, height, resolution = 4, 3, 0.5
    origin_x, origin_y = 0.0, 0.0

    # y = 0.25 m -> cell_y 0 (최소 y 행), x = 0.25 m -> cell_x 0
    bottom_left = np.array([[0.25, 0.25]])
    assert scan_projection.world_to_flat_cells(
        bottom_left, width, height, resolution, origin_x, origin_y,
    ).tolist() == [0]

    # y = 1.25 m -> cell_y 2 (최대 y 행) -> flat 2*4 + 3 = 11
    top_right = np.array([[1.75, 1.25]])
    assert scan_projection.world_to_flat_cells(
        top_right, width, height, resolution, origin_x, origin_y,
    ).tolist() == [11]


def test_world_to_flat_cells_honours_origin_offset():
    """격자 origin 은 셀 (0,0) 의 좌하단이다."""
    cells = scan_projection.world_to_flat_cells(
        np.array([[-6.75, -7.45]]),
        width=224, height=286, resolution=0.05,
        origin_x=-6.786268, origin_y=-7.478293,
    )
    assert cells.tolist() == [0]


def test_points_outside_the_grid_are_dropped():
    cells = scan_projection.world_to_flat_cells(
        np.array([[-1.0, -1.0], [0.25, 0.25], [99.0, 99.0]]),
        width=4, height=3, resolution=0.5, origin_x=0.0, origin_y=0.0,
    )
    assert cells.tolist() == [0]


def test_a_cell_counts_once_per_scan():
    """같은 셀에 여러 빔이 맞아도 한 스캔에서는 한 번만 센다."""
    cells = scan_projection.world_to_flat_cells(
        np.array([[0.10, 0.10], [0.20, 0.20], [0.30, 0.30]]),
        width=4, height=3, resolution=0.5, origin_x=0.0, origin_y=0.0,
    )
    assert cells.tolist() == [0]


def test_laser_endpoints_filter_invalid_ranges():
    points = scan_projection.laser_endpoints(
        ranges=[float('nan'), 0.05, 1.0, 100.0],
        angle_min=0.0, angle_increment=math.pi / 2,
        minimum=0.15, maximum=8.0,
    )
    assert points.shape == (1, 2)
    # index 2 -> angle = pi, range 1.0  ->  (-1, 0)
    assert points[0, 0] == -1.0
    assert abs(points[0, 1]) < 1.0e-12


def test_transform_points_applies_rotation_then_translation():
    points = np.array([[1.0, 0.0]])
    out = scan_projection.transform_points(points, 2.0, 3.0, math.pi / 2)
    assert abs(out[0, 0] - 2.0) < 1.0e-9
    assert abs(out[0, 1] - 4.0) < 1.0e-9


def test_accumulated_hit_mask_needs_repeated_observations():
    batches = [np.array([0]), np.array([0]), np.array([0, 5])]
    mask = scan_projection.accumulated_hit_mask(batches, (2, 3), minimum_hits=3)
    assert mask[0, 0]
    assert not mask[1, 2]


def _cells_to_xy(cells, width, resolution, origin_x, origin_y):
    return [((c % width) * resolution + origin_x + resolution / 2,
             (c // width) * resolution + origin_y + resolution / 2)
            for c in cells]


def test_raycast_clears_cells_between_sensor_and_endpoint():
    """센서와 끝점 사이가 해제되고, 끝점 자신은 해제되지 않는다."""
    width, height, res = 20, 5, 0.5
    origin = (0.25, 0.25)                       # 셀 (0, 0) 중심
    endpoint = np.array([[4.75, 0.25]])         # 셀 (9, 0) 중심, 4.5 m 앞
    cleared = scan_projection.raycast_free_cells(
        origin, endpoint, width, height, res, 0.0, 0.0,
        step_ratio=0.5, stop_margin=1.0)

    columns = sorted(int(c % width) for c in cleared)
    assert columns[0] == 0, '센서 셀부터 해제된다'
    assert 9 not in columns, '끝점(장애물) 은 해제하면 안 된다'
    assert 8 not in columns, 'stop_margin 1 셀이므로 그 앞도 남긴다'
    assert columns == list(range(0, 8))


def test_raycast_never_clears_beyond_the_endpoint():
    width, height, res = 20, 5, 0.5
    cleared = scan_projection.raycast_free_cells(
        (0.25, 0.25), np.array([[2.75, 0.25]]), width, height, res, 0.0, 0.0)
    assert max(int(c % width) for c in cleared) < 5


def test_raycast_drops_endpoints_that_are_too_close():
    """센서와 붙어 있는 끝점은 해제할 구간이 없다."""
    cleared = scan_projection.raycast_free_cells(
        (0.25, 0.25), np.array([[0.30, 0.25]]), 20, 5, 0.5, 0.0, 0.0)
    assert cleared.size == 0


def test_raycast_clips_to_the_grid():
    cleared = scan_projection.raycast_free_cells(
        (0.25, 0.25), np.array([[99.0, 0.25]]), 20, 5, 0.5, 0.0, 0.0)
    assert cleared.size > 0
    assert max(int(c % 20) for c in cleared) < 20
    assert max(int(c // 20) for c in cleared) < 5


def test_raycast_handles_an_empty_scan():
    assert scan_projection.raycast_free_cells(
        (0.0, 0.0), np.empty((0, 2)), 20, 5, 0.5, 0.0, 0.0).size == 0


def test_larger_step_ratio_never_clears_more_cells():
    """표본을 성기게 잡으면 해제가 줄어들 뿐 늘어나지 않는다 (안전한 방향)."""
    rng = np.random.default_rng(3)
    angles = rng.uniform(-np.pi, np.pi, 200)
    radii = rng.uniform(1.0, 6.0, 200)
    endpoints = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    fine = scan_projection.raycast_free_cells(
        (0.0, 0.0), endpoints, 224, 286, 0.05, -5.6, -7.15, step_ratio=0.25)
    coarse = scan_projection.raycast_free_cells(
        (0.0, 0.0), endpoints, 224, 286, 0.05, -5.6, -7.15, step_ratio=1.0)
    assert set(coarse.tolist()) <= set(fine.tolist())
