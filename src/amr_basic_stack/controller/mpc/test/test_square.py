# pytest -q controller/mpc/test   (ROS 불필요) — core/square.py (2026-11-20)
import math

import numpy as np
import pytest

from amr_mpc.core.square import Square, wrap

MODES = ("tangent", "fixed")
O = (0.4, -0.3, 0.6)


def body_fd(sq, t, h=1e-5):
    """위치·헤딩의 중앙 차분을 몸체 좌표로."""
    X1, _ = sq.at(t, O)
    X2, _ = sq.at(t + h, O)
    Xm, _ = sq.at(t - h, O)
    d = (X2 - Xm) / (2 * h)
    d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
    c, s = math.cos(X1[2]), math.sin(X1[2])
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])


def corner_times(sq):
    return [t0 for t0 in sq.t_start[1:]]


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("accel", [0.4, 1.0])
def test_velocity_matches_position_derivative(mode, accel):
    """발행하는 몸체 속도가 위치·헤딩의 미분과 같아야 MPC 의 기준 가속도(속도 차분)·목표 입력이 맞다. 꼭짓점 계단 ±1 ms 는 뺀다."""
    sq = Square(accel=accel, yaw_mode=mode)
    cts = corner_times(sq)
    for t in np.linspace(0.002, sq.T_total - 0.002, 4000):
        if min(abs(t - c) for c in cts) < 1e-3:
            continue
        assert np.allclose(sq.at(t, O)[1], body_fd(sq, t), atol=1e-4), t


@pytest.mark.parametrize("accel, side", [(0.4, 1.5), (1.0, 1.5), (0.4, 2.0)])
def test_peak_at_middle_of_interior_edge(accel, side):
    """가운데 변: 꼭짓점 v_c 에서 가속해 가운데에서 √(v_c² + aL), 그 뒤 감속."""
    sq = Square(side=side, accel=accel, corner_speed=0.1, yaw_mode="fixed")
    e = sq.edges[1]
    vp = math.sqrt(0.1 ** 2 + accel * side)
    assert e.vp == pytest.approx(vp)
    assert e.d2 == pytest.approx(0.0, abs=1e-12)                 # 등속 구간 없음 (삼각형)
    assert e.d1 == pytest.approx(side / 2)                       # 최고가 변 가운데
    s_mid, v_mid = e.at(e.t1)
    assert s_mid == pytest.approx(side / 2) and v_mid == pytest.approx(vp)


def test_v_max_clips_to_trapezoid():
    sq = Square(accel=0.4, v_max=0.5, yaw_mode="fixed")
    assert all(e.vp == pytest.approx(0.5) for e in sq.edges)
    assert sq.edges[1].d2 > 0.0
    with pytest.raises(ValueError):
        Square(corner_speed=0.3, v_max=0.2)                      # 꼭짓점보다 낮은 상한


@pytest.mark.parametrize("mode", MODES)
def test_corner_speeds_start_and_end_at_rest(mode):
    """첫 출발·마지막 도착은 0, 사이 꼭짓점은 속도 크기 v_c (방향만 꺾임)."""
    sq = Square(corner_speed=0.1, yaw_mode=mode, laps=2.0)
    assert np.allclose(sq.at(0.0, O)[1], 0.0)
    for c in corner_times(sq):
        for tt in (c - 1e-9, c + 1e-9):
            assert np.hypot(*sq.at(tt, O)[1][:2]) == pytest.approx(0.1, abs=1e-6)
    assert np.allclose(sq.at(sq.T_path, O)[1][:2], 0.0)


@pytest.mark.parametrize("mode", MODES)
def test_path_stays_on_square_and_returns(mode):
    """위치는 늘 정사각형 변 위 (헤딩 완화가 경로를 안 바꾼다), 한 바퀴 뒤 출발 포즈로."""
    sq = Square(side=1.5, yaw_mode=mode)
    x0, y0, a0 = O
    c0, s0 = math.cos(a0), math.sin(a0)
    for t in np.linspace(0.0, sq.T_total, 3000):
        X, _ = sq.at(t, O)
        px, py = c0 * (X[0] - x0) + s0 * (X[1] - y0), -s0 * (X[0] - x0) + c0 * (X[1] - y0)
        assert -1e-9 <= px <= 1.5 + 1e-9 and -1e-9 <= py <= 1.5 + 1e-9
        assert min(abs(px), abs(px - 1.5), abs(py), abs(py - 1.5)) < 1e-9     # 안쪽이 아니라 변 위
    Xe, ue = sq.at(sq.T_total + 1.0, O)
    assert np.allclose(Xe, O, atol=1e-9) and np.allclose(ue, 0.0)


def test_tangent_heading_is_continuous_and_follows_edges():
    """tangent: 헤딩에 계단이 없고 (완화), 완화 구간 밖에서는 변 방향."""
    sq = Square(yaw_mode="tangent", yaw_blend=0.3)
    ts = np.linspace(0.0, sq.T_total, 20000)
    yaws = np.array([sq.at(t, O)[0][2] for t in ts])
    jumps = np.abs([wrap(b - a) for a, b in zip(yaws[:-1], yaws[1:])])
    assert jumps.max() < math.radians(1.0)
    for i in range(4):                                          # 변 가운데에서는 변 방향
        e = sq.edges[i]
        X, u = sq.at(sq.t_start[i] + e.t1, O)
        assert wrap(X[2] - (O[2] + i * math.pi / 2)) == pytest.approx(0.0, abs=1e-9)
        assert u[1] == pytest.approx(0.0, abs=1e-9)            # 옆 속도 없음


def test_fixed_heading_never_rotates():
    sq = Square(yaw_mode="fixed")
    for t in np.linspace(0.0, sq.T_total, 2000):
        X, u = sq.at(t, O)
        assert X[2] == pytest.approx(O[2]) and u[2] == 0.0


def test_peak_load_is_peak_speed_for_fixed():
    """fixed: 변이 몸체 축과 나란해 부하 = 속도. tangent 는 꼭짓점 회전이 더해진다."""
    k = 0.1932
    run, spin = Square(yaw_mode="fixed", accel=0.4).peak_load(k)
    assert run == pytest.approx(math.sqrt(0.01 + 0.6), rel=1e-3) and spin == 0.0
    run_t, spin_t = Square(yaw_mode="tangent", accel=0.4).peak_load(k)
    assert run_t >= run - 1e-6 and spin_t > 0.0


@pytest.mark.parametrize("kw", [dict(yaw_mode="spin"), dict(side=0.0), dict(accel=0.0), dict(laps=0.1),
                                dict(yaw_mode="tangent", yaw_blend=0.0), dict(yaw_mode="tangent", yaw_blend=2.0)])
def test_rejects_bad_parameters(kw):
    with pytest.raises(ValueError):
        Square(**kw)


# ── 꼭짓점 정지 corner_mode stop (2026-11-21) ──────────────────────────────
STOP = dict(corner_mode="stop", accel=1.5, v_max=1.5, rot_s=1.0, dwell_s=0.1)


@pytest.mark.parametrize("mode", MODES)
def test_stop_velocity_matches_position_derivative(mode):
    """stop 은 속도 계단이 없다 — 어디서나 발행 속도 = 위치·헤딩의 미분."""
    sq = Square(yaw_mode=mode, **STOP)
    for t in np.linspace(0.002, sq.T_total - 0.002, 4000):
        assert np.allclose(sq.at(t, O)[1], body_fd(sq, t), atol=1e-4), t


def test_stop_timeline_and_corner_rotation():
    """변 4 개 × 2 s (0 → 1.5 → 0, a 1.5) + 사이 꼭짓점 3 개 × (0.1 + 1.0 + 0.1). 꼭짓점 블록 동안 위치 고정·속도 0, 헤딩 +90°."""
    sq = Square(yaw_mode="tangent", **STOP)
    assert sq.vc == 0.0 and sq.T_corner == pytest.approx(1.2)
    assert sq.T_path == pytest.approx(4 * 2.0 + 3 * 1.2)
    for i in range(3):
        t_stop = sq.t_start[i] + sq.edges[i].T
        X0, u0 = sq.at(t_stop + 1e-6, O)
        X1, u1 = sq.at(t_stop + sq.T_corner - 1e-6, O)
        assert np.allclose(u0, 0.0, atol=1e-5) and np.allclose(u1, 0.0, atol=1e-5)
        assert np.allclose(X0[:2], X1[:2], atol=1e-12)
        assert wrap(X1[2] - X0[2]) == pytest.approx(math.pi / 2, abs=1e-6)
        Xm, um = sq.at(t_stop + sq.dwell + 0.5 * sq.rot, O)    # 회전 가운데: 병진 0, ω = 1.5·(π/2)/rot_s
        assert np.allclose(um[:2], 0.0) and um[2] == pytest.approx(1.5 * 0.5 * math.pi / sq.rot)


def test_stop_tangent_has_no_sideways_velocity():
    """stop + tangent: 몸체 v_y = 0 이 어디서나 — 퓨어 퍼슛 (v_y = 0) 이 낼 수 있는 기준."""
    sq = Square(yaw_mode="tangent", **STOP)
    vy = [sq.at(t, O)[1][1] for t in np.linspace(0.0, sq.T_total, 4000)]
    assert np.abs(vy).max() < 1e-9


def test_stop_fixed_never_rotates_and_pass_unchanged():
    sq = Square(yaw_mode="fixed", **STOP)
    assert sq.rot == 0.0 and sq.T_corner == pytest.approx(0.2)
    assert all(sq.at(t, O)[1][2] == 0.0 for t in np.linspace(0.0, sq.T_total, 500))
    a, b = Square(), Square(corner_mode="pass")                 # 기본은 pass — 예전 기준 그대로
    assert a.T_path == b.T_path and a.t_start == b.t_start


@pytest.mark.parametrize("kw", [dict(corner_mode="halt"), dict(corner_mode="stop", rot_s=0.0),
                                dict(corner_mode="stop", dwell_s=-0.1)])
def test_stop_rejects_bad_parameters(kw):
    with pytest.raises(ValueError):
        Square(**kw)
