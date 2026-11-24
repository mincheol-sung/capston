"""지그재그 기준 (core/zigzag.py, 2026-11-24) — 모양·헤딩·전진/후진·속도 파형·유한차분 검산."""
import math

import numpy as np
import pytest

from amr_mpc.core.zigzag import Zigzag, wrap


def _fd_body(z, t, o, h=1e-5):
    X1, _ = z.at(t, o)
    X2, _ = z.at(t + h, o)
    Xm, _ = z.at(t - h, o)
    d = (X2 - Xm) / (2 * h)
    d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
    c, s = math.cos(X1[2]), math.sin(X1[2])
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])


def test_default_geometry():
    """기본 5 × 1 m, 사이각 10°: 꼭짓점이 왼쪽으로 sin 10° 씩 나아가고 끝점 (1 + 2(1 − cos10°), 2 sin10°)."""
    z = Zigzag()
    a = math.radians(10.0)
    assert z.n == 5 and z.length == 1.0
    assert np.allclose(z.corners[2], [1 - math.cos(a), math.sin(a)])
    assert np.allclose(z.corners[-1], [1 + 2 * (1 - math.cos(a)), 2 * math.sin(a)])
    for k in range(1, z.n):                             # 꼭짓점 k 에서 두 직선이 나가는 방향 사이각 = 10°
        u, w = -z.dirs[k - 1], z.dirs[k]
        assert math.degrees(math.acos(np.clip(u @ w, -1, 1))) == pytest.approx(10.0)


def test_heading_along_line_and_direction_alternates():
    """직선 위 헤딩 = 직선 방향, 짝수 직선 전진 (v_x > 0), 홀수 후진 (v_x < 0), v_y = ω = 0."""
    z = Zigzag(n=4, angle_deg=-15.0)
    for k, g in enumerate(z.legs):
        X, u = z.at(z.t_start[k] + 0.5 * g.T)
        assert np.sign(u[0]) == (1 if k % 2 == 0 else -1)
        assert u[1] == 0.0 and u[2] == 0.0
        assert X[2] == pytest.approx(0.0 if k % 2 == 0 else math.radians(15.0))   # 음수 각 → 오른쪽, 홀수 직선 헤딩 +15°
    assert z.corners[-1][1] < 0.0


@pytest.mark.parametrize("profile", ["sine", "square"])
@pytest.mark.parametrize("v_max", [0.0, 0.6])
def test_speed_profile_ends_zero_and_peak_accel(profile, v_max):
    A, L = 1.7, 1.0
    z = Zigzag(accel=A, profile=profile, v_max=v_max)
    g = z.legs[0]
    ts = np.linspace(0.0, g.T, 4001)
    v = np.array([g.at(t)[1] for t in ts])
    acc = np.array([g.at(t)[2] for t in ts])
    assert v[0] == 0.0 and abs(v[-1]) < 1e-9
    assert g.at(g.T)[0] == pytest.approx(L)
    assert np.trapz(v, ts) == pytest.approx(L, rel=1e-5)
    if profile == "sine":
        vp = math.sqrt(A * L / 2.0) if v_max == 0.0 else v_max
        assert g.vp == pytest.approx(vp) and np.max(v) == pytest.approx(vp, rel=1e-5)
        assert g.a_peak == pytest.approx(A if v_max == 0.0 else 2 * vp * vp / L)
    else:
        vp = math.sqrt(A * L) if v_max == 0.0 else v_max
        assert g.vp == pytest.approx(vp) and np.max(np.abs(acc)) == pytest.approx(A)
    assert np.max(np.abs(acc)) <= A + 1e-9


@pytest.mark.parametrize("kw", [dict(), dict(profile="square", accel=1.7), dict(angle_deg=-30.0, n=4, v_max=0.5)])
def test_velocity_matches_finite_difference(kw):
    z = Zigzag(**kw)
    o = (0.3, -0.2, 0.7)
    for t in np.linspace(0.003, z.T_total - 0.003, 3000):
        _, u = z.at(t, o)
        assert np.abs(_fd_body(z, t, o) - u).max() < 1e-4


def test_corner_rotation_in_place():
    """꼭짓점: 쉼 → 회전 (θ_k → θ_{k+1}, 위치 그대로) → 쉼. 주행 시간 = Σ 직선 + (n−1)(rot + 2 dwell)."""
    z = Zigzag(rot_s=0.6, dwell_s=0.1)
    g = z.legs[0]
    t0 = z.t_start[0] + g.T
    for dt in (0.05, 0.4, 0.75):
        X, u = z.at(t0 + dt)
        assert np.allclose(X[:2], z.corners[1]) and u[0] == 0.0
    assert z.at(t0 + 0.05)[1][2] == 0.0 and z.at(t0 + 0.4)[1][2] < 0.0   # 왼쪽 지그재그: 첫 꼭짓점 시계 방향 회전
    assert z.at(t0 + z.T_corner)[0][2] == pytest.approx(-math.radians(10.0))
    assert z.T_path == pytest.approx(sum(x.T for x in z.legs) + 4 * 0.8)
    assert z.T_align == 0.0 and z.T_total == z.T_path


@pytest.mark.parametrize("kw", [dict(yaw_mode="fixed"), dict(profile="tri"), dict(n=0), dict(n=2.5), dict(angle_deg=0.0),
                                dict(angle_deg=180.0), dict(length=0.0), dict(accel=0.0), dict(rot_s=0.0)])
def test_rejects_bad_params(kw):
    with pytest.raises(ValueError):
        Zigzag(**kw)
