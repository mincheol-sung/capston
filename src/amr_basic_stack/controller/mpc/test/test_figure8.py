# pytest -q controller/mpc/test   (ROS 불필요)
import math

import numpy as np
import pytest

from amr_mpc.core.figure8 import Figure8, wrap

MODES = ("tangent", "fixed")


def make(mode="tangent", **kw):
    d = dict(width=1.4, height=0.8, speed=0.25, accel=0.4, yaw_mode=mode, align_s=1.5)
    return Figure8(**{**d, **kw})


@pytest.mark.parametrize("mode", MODES)
def test_bounding_box_is_exactly_width_by_height(mode):
    """경계상자가 기동 포즈 중심의 W × H 여야 방 크기 검토가 성립한다."""
    f = make(mode)
    xy = np.array(f.path_xy(n=4001))
    assert xy[:, 0].min() == pytest.approx(-f.W / 2, abs=1e-9)
    assert xy[:, 0].max() == pytest.approx(+f.W / 2, abs=1e-9)
    assert xy[:, 1].min() == pytest.approx(-f.H / 2, abs=1e-4)
    assert xy[:, 1].max() == pytest.approx(+f.H / 2, abs=1e-4)


@pytest.mark.parametrize("mode", MODES)
def test_returns_to_start_pose_and_stops(mode):
    """제자리 복귀: 위치·헤딩 둘 다. 8자는 회전지수 0 이라 접선각도 제자리로 온다."""
    f = make(mode)
    assert f.th_tab[-1] - f.th_tab[0] == pytest.approx(0.0, abs=1e-9)
    origin = np.array([1.0, -2.0, 0.7])
    X0, u0 = f.at(0.0, origin)
    Xe, ue = f.at(f.T_total + 1.0, origin)
    assert np.allclose(X0, origin) and np.allclose(u0, 0.0)
    assert np.allclose(Xe, origin, atol=1e-9) and np.allclose(ue, 0.0)


@pytest.mark.parametrize("mode", MODES)
def test_published_twist_matches_derivative_of_published_pose(mode):
    """발행하는 몸체속도가 발행하는 포즈의 도함수인지 유한차분으로 독립 검산.
    둘이 어긋나면 MPC 의 피드포워드(ur)와 오차항(Xr)이 서로 다른 궤적을 가리킨다."""
    f = make(mode)
    origin = np.array([0.3, 0.4, -1.1])
    h = 1e-4
    for t in np.linspace(h, f.T_total - h, 1500):
        Xm, _ = f.at(t - h, origin)
        Xp, _ = f.at(t + h, origin)
        X, u = f.at(t, origin)
        c, s = math.cos(X[2]), math.sin(X[2])
        assert np.allclose([c * u[0] - s * u[1], s * u[0] + c * u[1]],
                           (Xp[:2] - Xm[:2]) / (2 * h), atol=2e-3)
        assert u[2] == pytest.approx(wrap(Xp[2] - Xm[2]) / (2 * h), abs=5e-3)


def test_tangent_mode_drives_forward_only():
    """접선 추종이면 주행 중 몸체 y 속도가 0 — 메카넘 횡슬립을 안 쓴다."""
    f = make("tangent")
    for t in np.linspace(f.T_align, f.T_align + f.T_path, 500):
        u = f.at(t)[1]
        assert abs(u[1]) < 1e-9
        assert u[0] >= -1e-9


def test_speed_profile_is_trapezoidal_and_covers_arc_length():
    f = make("fixed")
    assert f.V == pytest.approx(0.25) and f.t_cru > 0.0
    dt = 1e-3
    dist = sum(f._profile(t)[1] for t in np.arange(0.0, f.T_path, dt)) * dt
    assert dist == pytest.approx(f.L, rel=1e-3)          # ∫v dt = 호길이


def test_triangular_profile_when_path_too_short_to_cruise():
    f = Figure8(width=0.2, height=0.1, speed=2.0, accel=0.4, yaw_mode="fixed")
    assert f.V < 2.0 and f.t_cru == pytest.approx(0.0, abs=1e-9)
    assert f.V == pytest.approx(math.sqrt(0.4 * f.L))


def test_rejects_bad_parameters():
    for kw in ({"width": 0.0}, {"height": -1.0}, {"speed": 0.0}, {"accel": -0.1},
               {"laps": 0.0}, {"yaw_mode": "spin"}):
        with pytest.raises(ValueError):
            make(**kw)


# ── 무장 재개 배속 램프 (2026-11-11) ───────────────────────────────────────────────
from amr_mpc.core.figure8 import ramp_elapsed, ramp_rate


def test_ramp_from_zero_is_quadratic_then_linear():
    Tr = 1.25
    assert ramp_elapsed(0.0, 0.0, Tr) == 0.0
    assert math.isclose(ramp_elapsed(0.5, 0.0, Tr), 0.25 / (2 * Tr))          # τ²/(2Tr)
    assert math.isclose(ramp_elapsed(Tr, 0.0, Tr), Tr / 2)                     # 배속 1 에 닿을 때 절반만 흐름
    assert math.isclose(ramp_elapsed(Tr + 2.0, 0.0, Tr), Tr / 2 + 2.0)         # 그 뒤로는 1 배속
    assert ramp_rate(Tr / 2, 0.0, Tr) == 0.5 and ramp_rate(5.0, 0.0, Tr) == 1.0


def test_ramp_is_continuous_and_composable():
    # 0.3 s 씩 끊어 적분해도 한 번에 적분한 것과 같아야 한다 (노드는 틱마다 끊어서 쌓는다)
    Tr, r, t = 1.25, 0.0, 0.0
    for _ in range(10):
        t += ramp_elapsed(0.3, r, Tr)
        r = ramp_rate(0.3, r, Tr)
    assert math.isclose(t, ramp_elapsed(3.0, 0.0, Tr), rel_tol=1e-12)


def test_ramp_disabled_or_full_rate_is_identity():
    assert ramp_elapsed(2.0, 1.0, 1.25) == 2.0
    assert ramp_elapsed(2.0, 0.0, 0.0) == 2.0


# ── 지도 고정 출발의 진입 구간 (2026-11-11) ────────────────────────────────────────
from amr_mpc.core.figure8 import lead_in, lead_in_duration


def test_lead_in_endpoints_and_rest():
    start, goal, T = [0.2, -0.1, 0.3], [0.0, 0.0, 0.0], 3.0
    X0, u0 = lead_in(0.0, T, start, goal)
    X1, u1 = lead_in(T, T, start, goal)
    assert np.allclose(X0, start) and np.allclose(u0, 0.0)      # 출발: 로봇 포즈, 정지
    assert np.allclose(X1, goal) and np.allclose(u1, 0.0)       # 끝: 교차점, 정지 → 8자로 이어진다
    Xa, _ = lead_in(-1.0, T, start, goal); Xb, _ = lead_in(T + 1.0, T, start, goal)
    assert np.allclose(Xa, start) and np.allclose(Xb, goal)


def test_lead_in_velocity_is_derivative_of_pose():
    start, goal, T, h = [0.25, 0.1, -0.4], [1.0, 2.0, 0.2], 4.0, 1e-5
    for t in np.linspace(0.1, T - 0.1, 25):
        Xm, _ = lead_in(t - h, T, start, goal); Xp, _ = lead_in(t + h, T, start, goal)
        X, u = lead_in(t, T, start, goal)
        c, s = math.cos(X[2]), math.sin(X[2])
        dxy = (Xp[:2] - Xm[:2]) / (2 * h)
        assert np.allclose([c * u[0] - s * u[1], s * u[0] + c * u[1]], dxy, atol=1e-6)   # 몸체 속도 → 세계
        assert math.isclose(u[2], (Xp[2] - Xm[2]) / (2 * h), abs_tol=1e-6)


def test_lead_in_duration_respects_limits():
    assert lead_in_duration([0, 0, 0], [0.3, 0.4, 0], 0.1, 0.5) == pytest.approx(5.0)       # 0.5 m / 0.1 m/s
    assert lead_in_duration([0, 0, 0], [0, 0, 1.0], 0.1, 0.5) == pytest.approx(2.0)         # 1 rad / 0.5 rad/s
    assert lead_in_duration([0, 0, 0], [0.01, 0, 0], 0.1, 0.5, min_s=3.0) == 3.0            # 최소 start_delay


# ── 랩별 속도 (2026-11-11) ───────────────────────────────────────────────────
def test_lap_speeds_profile():
    f = Figure8(width=3.0, height=1.5, accel=0.4, lap_speeds=[0.3, 0.6, 0.45])
    assert f.laps == 3.0
    summ = f.lap_profile_summary()
    prev = 0.0
    for k, V, vmax, _ in summ:
        assert vmax <= max(V, prev) + 1e-6       # 목표를 넘지 않는다. 느려지는 랩은 경계를 지나며 내려오므로 직전 속도까지
        prev = V
    assert summ[1][2] == pytest.approx(0.6, abs=1e-3)               # 둘째 랩은 0.6 까지 올라간다
    X0, _ = f.at(0.0); Xe, ue = f.at(f.T_total + 1.0)
    assert np.hypot(*(Xe[:2] - X0[:2])) < 1e-6 and np.abs(ue).max() < 1e-9   # 출발점으로 돌아와 선다
    # 가속도가 accel 을 넘지 않는다 (호길이 축)
    dv = np.abs(np.diff(f._lap_v))
    assert dv[:-30].max() / 1e-3 <= 0.4 * 1.05                # 가감속 ≤ accel (끝 제동 곡선은 1 ms 격자라 3 % 넘칠 수 있다)
    assert dv[-30:].max() < 0.005                             # 끝에서 0 으로 맞추는 마지막 몇 ms 의 계단은 5 mm/s 미만


def test_lap_speeds_absent_keeps_old_profile():
    a = Figure8(width=3.0, height=1.5, speed=0.5, accel=0.4, laps=2)
    b = Figure8(width=3.0, height=1.5, speed=0.5, accel=0.4, laps=2, lap_speeds=None)
    for t in np.linspace(0, a.T_total, 50):
        assert np.allclose(a.at(t)[0], b.at(t)[0]) and np.allclose(a.at(t)[1], b.at(t)[1])
