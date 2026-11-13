"""D455 z-band 필터·투영 테스트. ROS 없이 돈다."""

import math

import numpy as np

from traversability import depth_projection as dp


def test_finite_points_drops_invalid_depth():
    """깊이 없는 픽셀의 NaN 을 버린다 — RealSense 가 그렇게 낸다."""
    raw = np.array([
        [1.0, 0.0, 0.1],
        [np.nan, 0.0, 0.1],
        [1.0, np.inf, 0.1],
        [2.0, 0.0, 0.2],
    ])
    assert dp.finite_points(raw).tolist() == [[1.0, 0.0, 0.1], [2.0, 0.0, 0.2]]


def test_finite_points_handles_empty():
    assert dp.finite_points(np.empty((0, 3))).shape == (0, 3)


def test_select_height_band_is_inclusive_at_both_ends():
    points = np.array([
        [0.0, 0.0, 0.000],     # 바닥. 밴드 아래
        [0.0, 0.0, 0.033],     # 하한 정확히
        [0.0, 0.0, 0.150],
        [0.0, 0.0, 0.310],     # 상한 정확히
        [0.0, 0.0, 0.500],     # 로봇보다 높다. 밴드 위
    ])
    band = dp.select_height_band(points, 0.033, 0.31)
    assert band[:, 2].tolist() == [0.033, 0.150, 0.310]


def test_height_band_uses_vertical_z_not_optical_z():
    """Optical 좌표의 z 는 전방 깊이다. 변환 없이 밴드에 넣으면 안 된다.

    optical 규약: x 오른쪽, y 아래쪽, z 전방.
    카메라 앞 2 m, 광축과 같은 높이에 있는 점은 optical 에서 (0, 0, 2) 다.
    그 z=2 를 그대로 밴드 [0.033, 0.31] 에 넣으면 탈락하지만, 지면 기준으로
    변환하면 카메라 높이 0.1328 m 라 밴드 안이다.
    """
    optical_point = np.array([[0.0, 0.0, 2.0]])

    # 변환 없이 넣으면 사라진다 - 이게 바로 피해야 할 실수다
    assert dp.select_height_band(optical_point, 0.033, 0.31).shape[0] == 0

    # optical -> base_footprint: URDF camera_depth_optical_joint 의
    # rpy = (-pi/2, 0, -pi/2) 역변환. optical z(전방) -> base x(전방),
    # optical y(아래) -> base -z. 카메라는 지면 위 0.132772 m.
    rotation = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    translation = np.array([0.11398, 0.0, 0.132772])
    in_base = dp.transform_points(optical_point, translation, rotation)

    assert abs(in_base[0, 0] - 2.11398) < 1e-9, '전방 2 m 가 x 로 와야 한다'
    assert abs(in_base[0, 2] - 0.132772) < 1e-9, '높이는 카메라 높이 그대로'
    assert dp.select_height_band(in_base, 0.033, 0.31).shape[0] == 1


def test_floor_is_excluded_by_the_chosen_z_min():
    """z_min = 0.033 m 가 실제로 바닥면을 뺀다.

    설계문서 §4.1 의 후보값 -0.15 m(카메라 기준)를 그대로 쓰면 지면 기준
    0.1328 - 0.15 = -0.017 m 라 바닥이 통째로 밴드에 들어온다.
    """
    floor = np.column_stack([
        np.linspace(0.5, 3.0, 20),
        np.zeros(20),
        np.zeros(20),                      # 지면 z = 0
    ])
    assert dp.select_height_band(floor, 0.033, 0.31).shape[0] == 0

    camera_height = 0.04 + 0.092772
    bad_z_min = camera_height + (-0.15)
    assert bad_z_min < 0.0
    assert dp.select_height_band(floor, bad_z_min, 0.31).shape[0] == 20, (
        '문서 후보값을 그대로 쓰면 바닥 전체가 장애물이 된다'
    )


def test_drop_self_observations_keeps_external_obstacles():
    points = np.array([
        [0.10, 0.00, 0.10],     # 차체 안
        [0.19, 0.14, 0.27],     # 차체 안 (모서리)
        [0.25, 0.00, 0.10],     # 차체 앞 - 남아야 한다
        [0.00, 0.00, 0.40],     # 차체 위 - 남아야 한다
    ])
    kept = dp.drop_self_observations(
        points, [-0.20, -0.15, -0.05], [0.20, 0.15, 0.28])
    assert kept.tolist() == [[0.25, 0.0, 0.10], [0.0, 0.0, 0.40]]


def test_limit_range_uses_horizontal_distance():
    points = np.array([
        [0.10, 0.0, 0.2],      # 0.10 m - 너무 가깝다
        [1.00, 0.0, 0.2],
        [0.0, 5.99, 0.2],
        [7.00, 0.0, 0.2],      # 너무 멀다
    ])
    kept = dp.limit_range(points, 0.30, 6.0)
    assert kept.shape[0] == 2
    assert abs(np.hypot(kept[0, 0], kept[0, 1]) - 1.0) < 1e-9


def test_voxel_downsample_keeps_one_point_per_leaf():
    points = np.array([
        [0.00, 0.00, 0.10],
        [0.01, 0.01, 0.10],     # 같은 3 cm leaf
        [0.02, 0.02, 0.10],     # 같은 leaf
        [0.05, 0.00, 0.10],     # 다른 leaf
    ])
    kept = dp.voxel_downsample(points, 0.03)
    assert kept.shape[0] == 2


def test_voxel_downsample_is_deterministic():
    """같은 입력이면 같은 출력이어야 octomap 광선 통과가 안정적이다."""
    rng = np.random.default_rng(0)
    points = rng.random((500, 3))
    first = dp.voxel_downsample(points, 0.05)
    second = dp.voxel_downsample(points, 0.05)
    assert np.array_equal(first, second)


def test_to_flat_cells_uses_occupancy_grid_row_order():
    points = np.array([
        [0.25, 0.25, 0.1],     # cell (0, 0) -> flat 0
        [1.75, 1.25, 0.1],     # cell (3, 2) -> flat 2*4 + 3 = 11
    ])
    cells = dp.to_flat_cells(points, 4, 3, 0.5, 0.0, 0.0)
    assert cells.tolist() == [0, 11]


def test_to_flat_cells_drops_points_outside_the_grid():
    points = np.array([[-9.0, -9.0, 0.1], [0.25, 0.25, 0.1]])
    assert dp.to_flat_cells(points, 4, 3, 0.5, 0.0, 0.0).tolist() == [0]


def test_quaternion_to_matrix_matches_a_known_rotation():
    """Z 축 90도 회전이면 x 가 y 로 간다."""
    half = math.sqrt(0.5)
    rotation = dp.quaternion_to_matrix(0.0, 0.0, half, half)
    rotated = dp.transform_points(np.array([[1.0, 0.0, 0.0]]),
                                  np.zeros(3), rotation)
    assert abs(rotated[0, 0]) < 1e-9
    assert abs(rotated[0, 1] - 1.0) < 1e-9


def test_quaternion_to_matrix_is_orthonormal():
    rotation = dp.quaternion_to_matrix(0.13, -0.25, 0.41, 0.87)
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(rotation) - 1.0) < 1e-12


def test_every_stage_survives_an_empty_cloud():
    empty = np.empty((0, 3))
    assert dp.limit_range(empty, 0.3, 6.0).shape[0] == 0
    assert dp.drop_self_observations(empty, [-1, -1, -1], [1, 1, 1]).shape[0] == 0
    assert dp.select_height_band(empty, 0.0, 1.0).shape[0] == 0
    assert dp.voxel_downsample(empty, 0.03).shape[0] == 0
    assert dp.to_flat_cells(empty, 4, 3, 0.5, 0.0, 0.0).shape[0] == 0


# ── 2D 밴드 전용 상한 (limit_range_from) ───────────────────────────────────
#
# 왜 필요한가: 2D 격자는 셀에 점이 하나만 떨어져도 그 프레임의 hit 으로 센다.
# 깊이오차가 거리의 2 % 라 2.5 m 에서 이미 1 셀(0.05 m)을 넘고, 셀을 덮는
# 픽셀도 1.5 m 197 px² -> 6 m 12 px² 로 16 배 줄어든다.

CAMERA_XY = (0.11398, 0.0)     # URDF base_link -> camera_link 의 x


def test_limit_range_from_measures_from_the_given_origin():
    """카메라 원점 기준이다. base_footprint 원점으로 재면 0.114 m 어긋난다."""
    # 카메라 정면 1.4 m 지점 = base_footprint 기준 1.51398 m
    inside = np.array([[CAMERA_XY[0] + 1.4, 0.0, 0.1]])
    kept = dp.limit_range_from(inside, CAMERA_XY, 1.5)
    assert kept.shape[0] == 1

    # base_footprint 원점으로 재면 1.5 m 를 넘어 잘려 나간다 — 이 차이가 요점
    kept_wrong_origin = dp.limit_range_from(inside, (0.0, 0.0), 1.5)
    assert kept_wrong_origin.shape[0] == 0


def test_limit_range_from_cuts_beyond_the_limit():
    points = np.array([
        [CAMERA_XY[0] + 0.5, 0.0, 0.1],    # 0.5 m  남는다
        [CAMERA_XY[0] + 1.5, 0.0, 0.1],    # 1.5 m  경계 포함
        [CAMERA_XY[0] + 1.6, 0.0, 0.1],    # 1.6 m  잘린다
        [CAMERA_XY[0] + 6.0, 0.0, 0.1],    # 6.0 m  잘린다
    ])
    kept = dp.limit_range_from(points, CAMERA_XY, 1.5)
    assert kept.shape[0] == 2


def test_limit_range_from_ignores_height():
    """수평거리만 본다. 밴드 안에서 z 가 달라도 같은 취급이다."""
    points = np.array([
        [CAMERA_XY[0] + 1.0, 0.0, 0.033],
        [CAMERA_XY[0] + 1.0, 0.0, 0.310],
    ])
    assert dp.limit_range_from(points, CAMERA_XY, 1.5).shape[0] == 2


def test_limit_range_from_is_radial_not_forward_only():
    """반경 상한이다. D455 화각이 전방이라 뒤쪽 점은 애초에 없다."""
    lateral = np.array([[CAMERA_XY[0], 1.0, 0.1]])
    assert dp.limit_range_from(lateral, CAMERA_XY, 1.5).shape[0] == 1


def test_limit_range_from_handles_empty_input():
    """빈 입력에서 죽지 않는다. 깊이 결측이 많은 프레임에서 실제로 발생한다."""
    empty = np.empty((0, 3), dtype=np.float64)
    assert dp.limit_range_from(empty, CAMERA_XY, 1.5).shape[0] == 0


def test_limit_range_from_does_not_mutate_input():
    """호출자가 같은 배열을 3D 출력에도 쓰므로 원본을 건드리면 안 된다."""
    points = np.array([[CAMERA_XY[0] + 1.0, 0.0, 0.1],
                       [CAMERA_XY[0] + 9.0, 0.0, 0.1]])
    before = points.copy()
    dp.limit_range_from(points, CAMERA_XY, 1.5)
    assert points.tolist() == before.tolist()


def test_band_limit_is_tighter_than_the_3d_limit():
    """밴드 상한(1.5)이 전체 상한(6.0)보다 작아야 의미가 있다.

    두 값이 같아지면 이 필터는 아무 일도 하지 않는다 — 설정을 잘못 고쳤을 때
    조용히 무력화되는 것을 막는다.
    """
    import yaml
    import pathlib
    cfg = yaml.safe_load(
        (pathlib.Path(__file__).resolve().parent.parent
         / 'config' / 'depth_preprocess.yaml').read_text())
    params = cfg['depth_preprocess_node']['ros__parameters']
    assert params['band_max_range'] < params['max_range']
    # 깊이오차가 1 셀을 넘는 거리 0.05 / 0.02 = 2.50 m 안쪽이어야 한다
    assert params['band_max_range'] <= 2.50
