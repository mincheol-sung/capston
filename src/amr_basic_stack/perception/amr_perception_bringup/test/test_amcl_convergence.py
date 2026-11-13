"""AMCL 수렴 판정 테스트. ROS 없이 돈다."""

import math

from amr_perception_bringup.amcl_convergence import (
    ConvergenceTracker, pose_standard_deviations)

import pytest


def covariance(std_x, std_y, std_yaw):
    values = [0.0] * 36
    values[0] = std_x ** 2
    values[7] = std_y ** 2
    values[35] = std_yaw ** 2
    return values


def test_standard_deviations_read_the_right_diagonal_entries():
    """ROS 규약: 6x6 row-major 의 0, 7, 35 가 xx, yy, yaw-yaw 다."""
    std = pose_standard_deviations(covariance(0.2, 0.3, 0.1))
    assert abs(std[0] - 0.2) < 1e-12
    assert abs(std[1] - 0.3) < 1e-12
    assert abs(std[2] - 0.1) < 1e-12


def test_standard_deviations_reject_a_wrong_sized_covariance():
    with pytest.raises(ValueError):
        pose_standard_deviations([0.0] * 9)


def test_negative_diagonal_does_not_produce_nan():
    """수치 오차로 대각선이 살짝 음수가 되어도 죽지 않아야 한다."""
    values = covariance(0.1, 0.1, 0.1)
    values[0] = -1e-18
    assert pose_standard_deviations(values)[0] == 0.0


def _tracker(**kwargs):
    defaults = {'position_std': 0.15, 'yaw_std': math.radians(8.0),
                'required_samples': 3, 'min_travel': 1.0}
    defaults.update(kwargs)
    return ConvergenceTracker(**defaults)


def test_tight_covariance_alone_is_not_convergence():
    """움직이지 않았으면 공분산이 작아도 수렴으로 보지 않는다.

    AMCL 은 로봇이 움직여야 파티클을 갱신한다. 한 자리에서 본 스캔만으로는
    대칭적인 복도에서 엉뚱한 곳에 자신 있게 수렴한다.
    """
    tracker = _tracker()
    for _ in range(10):
        assert tracker.update(covariance(0.01, 0.01, 0.01)) is False
    assert tracker.streak == 10
    assert not tracker.travel_satisfied


def test_travel_alone_is_not_convergence():
    tracker = _tracker()
    tracker.add_travel(5.0)
    assert tracker.update(covariance(2.0, 2.0, 1.0)) is False
    assert tracker.streak == 0


def test_convergence_needs_both_travel_and_a_streak():
    tracker = _tracker()
    tracker.add_travel(1.5)
    assert tracker.update(covariance(0.01, 0.01, 0.01)) is False   # 1/3
    assert tracker.update(covariance(0.01, 0.01, 0.01)) is False   # 2/3
    assert tracker.update(covariance(0.01, 0.01, 0.01)) is True    # 3/3


def test_a_single_bad_sample_resets_the_streak():
    tracker = _tracker()
    tracker.add_travel(1.5)
    tracker.update(covariance(0.01, 0.01, 0.01))
    tracker.update(covariance(0.01, 0.01, 0.01))
    tracker.update(covariance(9.0, 9.0, 3.0))          # 튐
    assert tracker.streak == 0
    assert tracker.update(covariance(0.01, 0.01, 0.01)) is False


def test_each_axis_can_veto_on_its_own():
    for bad in ('x', 'y', 'yaw'):
        tracker = _tracker()
        tracker.add_travel(2.0)
        std = {'x': 0.01, 'y': 0.01, 'yaw': 0.01}
        std[bad] = 9.0
        for _ in range(5):
            result = tracker.update(
                covariance(std['x'], std['y'], std['yaw']))
        assert result is False, f'{bad} 축이 커도 수렴이라고 했다'


def test_add_travel_ignores_non_positive_distance():
    tracker = _tracker()
    tracker.add_travel(-3.0)
    tracker.add_travel(0.0)
    assert tracker.travel == 0.0


def test_reset_clears_everything():
    tracker = _tracker()
    tracker.add_travel(2.0)
    tracker.update(covariance(0.01, 0.01, 0.01))
    tracker.reset()
    assert tracker.travel == 0.0
    assert tracker.streak == 0
    assert tracker.last_std is None


def test_thresholds_are_inclusive_at_the_boundary():
    tracker = _tracker(required_samples=1)
    tracker.add_travel(1.0)
    assert tracker.update(covariance(0.15, 0.15, math.radians(8.0))) is True


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError):
        _tracker(position_std=0.0)
    with pytest.raises(ValueError):
        _tracker(yaw_std=-1.0)
    with pytest.raises(ValueError):
        _tracker(required_samples=0)
    with pytest.raises(ValueError):
        _tracker(min_travel=-1.0)


def test_status_is_readable_before_any_pose():
    tracker = _tracker()
    assert '아직' in tracker.status()


def _corridor_map(width=40, height=20, resolution=0.1):
    """위아래에 벽이 있고 왼쪽 끝이 막힌 복도. 좌우가 비대칭이다."""
    data = [0] * (width * height)

    def put(cx, cy):
        data[cy * width + cx] = 100

    for x in range(width):
        put(x, 0)
        put(x, height - 1)
    for y in range(height):
        put(0, y)                      # 왼쪽만 막혀 있다 -> 비대칭
    return data, width, height, resolution


def _matcher(**kwargs):
    from amr_perception_bringup.amcl_convergence import MapMatcher
    data, w, h, res = _corridor_map()
    defaults = {'occupancy': data, 'width': w, 'height': h,
                'resolution': res, 'origin_x': 0.0, 'origin_y': 0.0,
                'match_distance': 0.2}
    defaults.update(kwargs)
    return MapMatcher(**defaults)


def test_matcher_scores_points_on_walls_as_hits():
    matcher = _matcher()
    # 아래 벽(y=0 행)의 셀 중심들
    on_wall = [(x * 0.1 + 0.05, 0.05) for x in range(5, 15)]
    score, used = matcher.score(on_wall)
    assert used == 10
    assert score == 1.0


def test_matcher_scores_points_in_open_space_as_misses():
    matcher = _matcher()
    middle = [(x * 0.1 + 0.05, 1.0) for x in range(5, 15)]   # 벽에서 멀다
    score, used = matcher.score(middle)
    assert used == 10
    assert score == 0.0


def test_matcher_ignores_points_outside_the_grid():
    matcher = _matcher()
    score, used = matcher.score([(-99.0, -99.0), (0.5, 0.05)])
    assert used == 1
    assert score == 1.0


def test_matcher_returns_zero_for_an_empty_scan():
    assert _matcher().score([]) == (0.0, 0)


def test_matcher_rejects_a_map_without_obstacles():
    from amr_perception_bringup.amcl_convergence import MapMatcher
    with pytest.raises(ValueError):
        MapMatcher([0] * 100, 10, 10, 0.1, 0.0, 0.0)


def test_match_distance_controls_tolerance():
    tight = _matcher(match_distance=0.05)
    loose = _matcher(match_distance=0.35)
    # 벽에서 0.25 m 떨어진 점
    point = [(0.5, 0.25 + 0.05)]
    assert tight.score(point)[0] == 0.0
    assert loose.score(point)[0] == 1.0


def test_a_flipped_pose_scores_worse_than_the_true_pose():
    """180 도 뒤집힌 pose 는 벽 모양이 안 맞아 점수가 떨어진다.

    AMCL 이 자신 있게 틀린 곳에 수렴했을 때 이걸로 걸러낸다.
    """
    matcher = _matcher()
    # 진짜 pose: 로봇이 (1.0, 1.0) 에 있고 왼쪽 막힌 벽(x=0)과 아래 벽을 본다
    true_points = [(0.05, 1.0), (0.05, 1.2), (1.0, 0.05), (1.5, 0.05)]
    # 뒤집힌 pose 라면 같은 스캔이 오른쪽 열린 공간으로 간다 (벽이 없다)
    flipped_points = [(3.9, 1.0), (3.9, 1.2), (2.0, 1.0), (2.5, 1.0)]
    assert matcher.score(true_points)[0] > matcher.score(flipped_points)[0]
    assert matcher.score(true_points)[0] == 1.0


def test_match_distance_inflates_the_baseline_score():
    """허용 거리를 키우면 엉뚱한 pose 도 점수를 얻는다.

    2026-09-09 실측(test_imu_scan): 지도에서 "장애물 근처" 인 셀의 비율이
    match_distance 0.05 m 에서 5.7 %, 0.10 m 에서 12.1 %, 0.20 m 에서 23.2 %
    였다. 이 비율이 무작위 pose 의 기저 점수이므로, 허용 거리를 키울수록
    구분력이 떨어진다. min_match_score 를 정할 때 같이 봐야 한다.
    """
    import numpy as np
    tight = _matcher(match_distance=0.05)
    loose = _matcher(match_distance=0.5)
    near_tight = float((tight.distance <= 0.05).mean())
    near_loose = float((loose.distance <= 0.5).mean())
    assert near_loose > near_tight
    assert np.isfinite(near_loose)
