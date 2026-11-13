"""amcl_wiggle_node 의 기구학과 이동 계획. ROS 없이 돈다.

가장 중요한 것은 첫 테스트다 — 바퀴 부호가 amr_mpc 의 T_inv 와 다르면 로봇이
엉뚱한 방향으로 움직인다. 두 구현을 직접 비교해 박제한다.
"""

import math

from amr_perception_bringup.amcl_wiggle_node import (
    FORWARD_BACK, PATTERNS, ROTATE, ROTATE_ONLY, ROTATION, SQUARE, TRANSLATE,
    mecanum_inverse)

import numpy as np

import pytest


# plant_wheel.yaml geometry 실측값
R = 0.04
WHEEL_BASE = 0.1897
ROBOT_WIDTH = 0.207
K = (WHEEL_BASE + ROBOT_WIDTH) / 2.0


def _reference_t_inv():
    """amr_mpc.core.kinematics.mecanum_matrices 와 같은 식을 여기 다시 쓴다.

    amr_mpc 를 import 하면 인지 패키지가 제어 패키지에 build 의존을 갖게 되므로
    식을 복제하고 그 복제가 맞는지를 이 파일이 지킨다. amr_mpc 가 설치돼 있으면
    아래 test_matches_installed_amr_mpc 가 진짜 원본과 비교한다.
    """
    return (1.0 / R) * np.array([
        [1.0, -1.0, -K],
        [1.0, 1.0, K],
        [1.0, 1.0, -K],
        [1.0, -1.0, K],
    ])


@pytest.mark.parametrize('body', [
    (0.10, 0.0, 0.0),
    (0.0, 0.10, 0.0),
    (0.0, 0.0, 0.5),
    (-0.07, 0.03, -0.2),
])
def test_wheels_match_the_amr_mpc_convention(body):
    """[FL, FR, RL, RR] 순서와 부호가 T_inv 와 같아야 한다."""
    expected = _reference_t_inv() @ np.array(body)
    actual = mecanum_inverse(body[0], body[1], body[2], R, K)
    assert actual == pytest.approx(expected, abs=1.0e-12)


def test_matches_installed_amr_mpc():
    """amr_mpc 가 설치돼 있으면 원본과 직접 비교한다."""
    kinematics = pytest.importorskip('amr_mpc.core.kinematics')
    t_inv, _, _, k = kinematics.mecanum_matrices(R, WHEEL_BASE, ROBOT_WIDTH)
    assert k == pytest.approx(K)
    body = np.array([0.08, -0.05, 0.3])
    assert mecanum_inverse(body[0], body[1], body[2], R, K) == pytest.approx(
        t_inv @ body, abs=1.0e-12)


def test_pure_forward_spins_every_wheel_the_same_way():
    """전진은 네 바퀴가 같은 부호·같은 크기다. 부호가 섞이면 제자리에서 돈다."""
    wheels = mecanum_inverse(0.10, 0.0, 0.0, R, K)
    assert wheels == pytest.approx(np.full(4, 0.10 / R))


def test_pure_strafe_alternates_in_the_mecanum_pattern():
    """좌측 이동은 [-, +, +, -] 다. 메카넘 롤러 배치가 그렇다."""
    wheels = mecanum_inverse(0.0, 0.10, 0.0, R, K)
    magnitude = 0.10 / R
    assert wheels == pytest.approx(
        np.array([-magnitude, magnitude, magnitude, -magnitude]))


def test_square_returns_to_the_start_point():
    """네 구간의 방향 합이 0 이어야 출발점으로 돌아온다.

    돌아오지 않으면 반복할 때마다 로봇이 흘러가 주변 여유를 넘어선다.
    """
    total = np.zeros(3)
    for _, _, direction in SQUARE:
        total += np.array(direction)
    assert total == pytest.approx(np.zeros(3))


def test_square_covers_both_planar_axes():
    """전후만 흔들면 그 축 방향 모호성이 안 풀린다. 두 축을 다 써야 한다."""
    axes = {tuple(np.nonzero(np.array(d))[0]) for _, _, d in SQUARE}
    assert (0,) in axes and (1,) in axes


def test_square_uses_no_rotation():
    """회전이 섞이면 구간마다 몸체 자세가 달라져 방향 합이 0 이어도 안 돌아온다."""
    assert all(direction[2] == 0.0 for _, _, direction in SQUARE)


@pytest.mark.parametrize('pattern,laps,expected', [
    (SQUARE, 1, 0.80),
    (SQUARE, 2, 1.60),
    (FORWARD_BACK, 1, 0.40),
    (FORWARD_BACK, 3, 1.20),
])
def test_planned_path_length(pattern, laps, expected):
    """누적량은 경로 길이다(순변위가 아니다). 기본 구성이 min_travel 을 넘는가."""
    assert len(pattern) * laps * 0.20 == pytest.approx(expected)


def test_square_two_laps_clears_a_one_metre_min_travel():
    """패턴이 square 일 때 두 바퀴가 min_travel 1.0 m 를 넘어야 한다.

    현재 기본 pattern 은 rotate_only 이고 그때 min_travel 은 0 이다
    (test_config_pattern_and_min_travel_agree 가 그 짝을 지킨다).
    """
    min_travel = 1.0
    assert len(SQUARE) * 2 * 0.20 > min_travel


def test_forward_back_one_lap_does_not_clear_min_travel():
    """20 cm 왕복 한 번(0.4 m)은 부족하다 — 이걸 오해하면 영원히 안 수렴한다."""
    assert len(FORWARD_BACK) * 1 * 0.20 < 1.0


def test_wheel_speed_stays_inside_the_hardware_limit():
    """기본 speed 0.10 m/s 는 한계 30 rad/s 안에 넉넉히 들어간다."""
    wheels = mecanum_inverse(0.10, 0.0, 0.0, R, K)
    assert float(np.max(np.abs(wheels))) == pytest.approx(2.5)
    assert float(np.max(np.abs(wheels))) < 30.0


def test_strafe_and_rotation_together_stay_bounded():
    """사각형에는 없지만, 파라미터로 회전을 섞었을 때의 상한을 확인한다."""
    wheels = mecanum_inverse(0.10, 0.10, 0.5, R, K)
    assert float(np.max(np.abs(wheels))) < 30.0


def test_leg_completion_uses_displacement_not_path_length():
    """구간 완료는 구간 시작점으로부터의 변위로 판정한다.

    경로 길이로 판정하면 슬립·떨림이 누적되어 20 cm 를 못 가고 구간이 끝난다.
    노드가 쓰는 식을 여기 박제한다.
    """
    start = (1.0, 2.0)
    # 0.15 m 전진 후 0.05 m 후진: 경로 길이 0.20 m, 변위 0.10 m
    end = (1.10, 2.0)
    moved = math.hypot(end[0] - start[0], end[1] - start[1])
    assert moved == pytest.approx(0.10)
    assert moved < 0.20


# ── 병진 뒤 제자리 회전 ────────────────────────────────────────────────────
#
# 왜 넣었나: 사각형은 헤딩을 고정한 채 돌아서 yaw 공분산이 잘 안 줄어든다.
# ConvergenceTracker 가 위치와 yaw 를 함께 보므로(yaw_std_degrees 8.0),
# position_std 만 내려가고 수렴 판정이 안 떨어질 수 있다.

def test_rotation_phase_is_rotation_only():
    """회전 구간은 병진 성분이 0 이어야 제자리에서 돈다."""
    for _, kind, direction in ROTATION:
        assert kind == ROTATE
        assert direction[0] == 0.0 and direction[1] == 0.0
        assert direction[2] != 0.0


def test_translate_legs_carry_no_rotation():
    """병진 구간에 회전이 섞이면 출발점으로 안 돌아온다."""
    for pattern in (SQUARE, FORWARD_BACK):
        for _, kind, direction in pattern:
            assert kind == TRANSLATE
            assert direction[2] == 0.0


def test_rotation_comes_after_translation():
    """순서가 뒤집히면 '제자리 회전만으로는 엉뚱하게 붙는다' 가 된다.

    노드는 legs = 병진 × laps + 회전 으로 만든다. 그 순서를 박제한다.
    """
    legs = list(SQUARE) * 2 + list(ROTATION)
    kinds = [kind for _, kind, _ in legs]
    assert kinds.count(ROTATE) == 1
    assert kinds.index(ROTATE) == len(kinds) - 1
    assert set(kinds[:-1]) == {TRANSLATE}


def test_rotate_only_pattern_has_no_translation_legs():
    """rotate_only 는 병진 구간이 비어 있다. legs = [] * laps + ROTATION."""
    assert ROTATE_ONLY == ()
    assert PATTERNS['rotate_only'] is ROTATE_ONLY
    for laps in (1, 2, 5):
        legs = list(ROTATE_ONLY) * laps + list(ROTATION)
        kinds = [kind for _, kind, _ in legs]
        assert kinds == [ROTATE], 'laps 와 무관하게 회전 한 구간만 남아야 한다'


def test_patterns_table_covers_every_accepted_name():
    """노드의 검증이 PATTERNS 를 그대로 쓴다. 표와 상수가 어긋나면 안 된다."""
    assert PATTERNS == {
        'square': SQUARE,
        'forward_back': FORWARD_BACK,
        'rotate_only': ROTATE_ONLY,
    }


def test_rotation_does_not_count_toward_min_travel():
    """누적 이동은 x·y 경로 길이다. 제자리 회전은 0 을 더한다.

    그래서 사각형이 min_travel 1.0 m 를 먼저 채워야 한다 — 회전으로는 못 채운다.
    """
    # 제자리 회전 = 위치 변화 없음
    before = after = (1.0, 2.0)
    assert math.hypot(after[0] - before[0], after[1] - before[1]) == 0.0


@pytest.mark.parametrize('revolutions,expected_rad', [
    (1.0, 2.0 * math.pi),
    (2.0, 4.0 * math.pi),
    (0.5, math.pi),
])
def test_rotation_target_in_radians(revolutions, expected_rad):
    assert 2.0 * math.pi * revolutions == pytest.approx(expected_rad)


def test_accumulated_yaw_survives_the_pi_boundary():
    """누적은 wrap 한 증분의 절댓값 합이다.

    시작 yaw 와의 차이로 재면 한 바퀴마다 0 으로 돌아와 영원히 안 끝나고,
    wrap 을 안 하면 ±pi 경계에서 2pi 가 튀어 즉시 달성으로 읽힌다.
    노드가 쓰는 식을 여기 박제한다.
    """
    # +0.4 rad 씩 32 스텝 = 12.8 rad. 두 바퀴(4pi = 12.566) 를 넘겨 ±pi 경계를
    # 두 번 지난다. 누적이 실제 회전량과 같아야 한다.
    step = 0.4
    steps = 32
    yaw = 0.0
    turned = 0.0
    crossings = 0
    for _ in range(steps):
        new_yaw = math.atan2(math.sin(yaw + step), math.cos(yaw + step))
        if abs(new_yaw - yaw) > math.pi:      # 경계를 넘은 스텝
            crossings += 1
        delta = math.atan2(math.sin(new_yaw - yaw), math.cos(new_yaw - yaw))
        turned += abs(delta)
        yaw = new_yaw
    assert crossings >= 2, '경계를 안 지나면 이 테스트가 아무것도 검증하지 않는다'
    assert turned == pytest.approx(steps * step, abs=1e-9)
    assert turned > 4.0 * math.pi


def test_rotation_wheel_speed_stays_inside_the_limit():
    """yaw_rate 0.5 rad/s 에서 바퀴는 병진 0.10 m/s 와 같은 크기다."""
    wheels = mecanum_inverse(0.0, 0.0, 0.5, R, K)
    peak = float(np.max(np.abs(wheels)))
    assert peak == pytest.approx(0.5 * K / R)
    assert peak == pytest.approx(2.479, abs=1e-3)
    assert peak < 30.0


def test_pure_rotation_alternates_sign_across_the_diagonal():
    """제자리 회전은 [-, +, -, +] 다. 부호가 섞이면 옆으로 흐른다."""
    wheels = mecanum_inverse(0.0, 0.0, 0.5, R, K)
    magnitude = 0.5 * K / R
    assert wheels == pytest.approx(
        np.array([-magnitude, magnitude, -magnitude, magnitude]))


def _load_config():
    import pathlib
    import yaml
    return yaml.safe_load(
        (pathlib.Path(__file__).resolve().parent.parent
         / 'config' / 'amcl.yaml').read_text())


def test_config_rotation_matches_the_documented_plan():
    """amcl.yaml 의 값이 주석에 적은 계획 시간과 맞는가."""
    cfg = _load_config()
    p = cfg['amcl_wiggle_node']['ros__parameters']
    translate_legs = len(PATTERNS[p['pattern']]) * p['laps']
    translate_seconds = translate_legs * p['leg_distance'] / p['speed']
    rotate_seconds = 2.0 * math.pi * p['rotate_revolutions'] / p['yaw_rate']
    assert rotate_seconds == pytest.approx(25.13, abs=0.01)
    if p['pattern'] == 'square':
        assert translate_seconds == pytest.approx(16.0)
    else:
        assert translate_seconds == 0.0
    # 여유가 2 배 이상이어야 max_duration 이 정상 동작을 자르지 않는다
    assert p['max_duration'] > 2.0 * (translate_seconds + rotate_seconds)


def test_config_pattern_and_min_travel_agree():
    """설정의 pattern 과 min_travel 은 한 쌍이다. 어긋나면 안 수렴한다.

    ConvergenceTracker.converged = travel_satisfied and streak >= N 이고,
    travel 은 x·y 경로 길이라 제자리 회전은 0 을 더한다. 그래서 병진이 없는
    패턴에서 min_travel > 0 이면 파티클이 아무리 모여도 판정이 안 난다.

    반대로 병진이 있는 패턴에서 계획 경로가 min_travel 에 못 미치면
    wiggle 이 끝나도 카운터가 안 찬다.
    """
    cfg = _load_config()
    w = cfg['amcl_wiggle_node']['ros__parameters']
    min_travel = cfg['amcl_bootstrap_node']['ros__parameters']['min_travel']
    planned = len(PATTERNS[w['pattern']]) * w['laps'] * w['leg_distance']
    if planned == 0.0:
        assert min_travel == 0.0, (
            f"pattern {w['pattern']!r} 는 병진이 없는데 min_travel 이 "
            f'{min_travel} 이다. 0 이어야 수렴 판정이 난다.')
    else:
        assert planned >= min_travel, (
            f'계획 경로 {planned:.2f} m 가 min_travel {min_travel} m 에 '
            f'못 미친다.')


def test_config_rotate_only_needs_a_nonzero_rotation():
    """rotate_only 인데 회전까지 0 이면 할 일이 하나도 없다."""
    w = _load_config()['amcl_wiggle_node']['ros__parameters']
    if w['pattern'] == 'rotate_only':
        assert w['rotate_revolutions'] > 0.0
