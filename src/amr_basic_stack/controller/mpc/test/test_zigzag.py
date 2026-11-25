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


@pytest.mark.parametrize("kw", [dict(), dict(profile="square", accel=1.7), dict(angle_deg=-30.0, n=4, v_max=0.5),
                                dict(yaw_mode="fixed"), dict(yaw_mode="spin"), dict(yaw_mode="spin", spin_rate=3.0, profile="square", accel=1.7),
                                dict(n=4, align_s=1.5), dict(reverse=False, rot_s=1.2), dict(reverse=False, rot_s=1.2, n=4, align_s=2.0)])
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


@pytest.mark.parametrize("kw", [dict(yaw_mode="bad"), dict(yaw_mode="spin", spin_rate=0.0), dict(profile="tri"), dict(n=0), dict(n=2.5), dict(angle_deg=0.0),
                                dict(angle_deg=180.0), dict(length=0.0), dict(accel=0.0), dict(rot_s=0.0)])
def test_rejects_bad_params(kw):
    with pytest.raises(ValueError):
        Zigzag(**kw)


def test_fixed_heading_and_body_velocity():
    """2026-11-24 (9) fixed: 헤딩 0 고정, 몸체 속도 = 진행 방향 그대로 (홀수 직선은 170° 방향 → v_x < 0, v_y > 0), 꼭짓점 회전 없음 (쉼 2·dwell)."""
    z = Zigzag(yaw_mode="fixed", dwell_s=0.1)
    assert z.T_corner == pytest.approx(0.2) and z.rot == 0.0
    for k, g in enumerate(z.legs):
        X, u = z.at(z.t_start[k] + 0.5 * g.T)
        assert X[2] == 0.0 and u[2] == 0.0
        assert np.allclose(u[:2] / np.hypot(*u[:2]), z.dirs[k])
    X, u = z.at(z.t_start[1] + 0.5 * z.legs[1].T)
    assert u[0] < 0.0 and u[1] > 0.0


def test_spin_integer_turns_and_rate():
    """spin: 헤딩 = κ_s·호길이, n_s = round(spin_rate·L/V/2π) ≥ 1, ω = κ_s·v, 끝 헤딩 = 출발."""
    z = Zigzag(yaw_mode="spin", spin_rate=3.0, accel=1.7, profile="square")
    assert z.spin_turns == round(3.0 * 5.0 / z.V / (2 * math.pi)) == 2
    assert z.kappa_spin == pytest.approx(2 * math.pi * 2 / 5.0)
    k, g = 2, z.legs[2]
    t = z.t_start[k] + 0.3 * g.T
    X, u = z.at(t)
    d, v, _ = g.at(0.3 * g.T)
    assert u[2] == pytest.approx(z.kappa_spin * v) and X[2] == pytest.approx(wrap(z.kappa_spin * (k * z.length + d)))
    assert abs(wrap(z.at(z.T_total + 1.0)[0][2])) < 1e-9
    assert z.at(z.t_stop[0] + 0.05)[1][2] == 0.0                         # 꼭짓점 (v = 0) 에서 ω = 0


def test_tangent_end_alignment():
    """tangent, n 짝수: 끝 헤딩 −α → align_s 동안 출발 헤딩으로. n 홀수·align_s 0 이면 정렬 없음. 주행 구간 = (0, T_path)."""
    z = Zigzag(n=4, align_s=1.5)
    assert z.T_align == 1.5 and z.T_total == pytest.approx(z.T_path + 1.5) and z.drive_window == (0.0, z.T_path)
    assert z.at(z.T_path)[0][2] == pytest.approx(-math.radians(10.0))
    assert abs(z.at(z.T_total)[0][2]) < 1e-12 and z.at(z.T_path + 0.75)[1][2] > 0.0
    assert Zigzag(n=5, align_s=1.5).T_align == 0.0 and Zigzag(n=4, align_s=0.0).T_align == 0.0


def test_segment_window():
    """path following: 직선 j 위면 (t_start[j], t_stop[j]), 꼭짓점 (쉼·회전)·주행 밖이면 None (시계대로)."""
    z = Zigzag()
    assert z.segment_window(0.0) == pytest.approx((0.0, z.t_stop[0]))
    assert z.segment_window(z.t_stop[0] - 0.01) == pytest.approx((0.0, z.t_stop[0]))
    assert z.segment_window(z.t_stop[0] + 0.05) is None
    assert z.segment_window(z.t_start[1] + 0.5) == pytest.approx((z.t_start[1], z.t_stop[1]))
    assert z.segment_window(z.T_path + 1.0) is None


def test_forward_only_same_shape_always_forward():
    """2026-11-24 (10) reverse=False: 꼭짓점·모양은 같고, 직선마다 v_x > 0 (늘 전진), 헤딩 = 진행 방향 (0 | s·(π − α)),
    꼭짓점 회전 ±(π − α) 가 번갈아 (헤딩이 쌓이지 않음)."""
    a = math.radians(10.0)
    z0, z1 = Zigzag(reverse=True), Zigzag(reverse=False, rot_s=1.2)
    assert all(np.allclose(p, q) for p, q in zip(z0.corners, z1.corners))
    for k, g in enumerate(z1.legs):
        X, u = z1.at(z1.t_start[k] + 0.5 * g.T)
        assert u[0] > 0.0 and u[1] == 0.0 and u[2] == 0.0
        assert X[2] == pytest.approx(0.0 if k % 2 == 0 else math.pi - a)
    d = [z1.heads[k + 1] - z1.heads[k] for k in range(z1.n - 1)]
    assert d == pytest.approx([math.pi - a, -(math.pi - a)] * 2)
    assert z1.at(z1.t_stop[0] + z1.dwell + 0.5 * z1.rot)[1][2] > 0.0     # 첫 꼭짓점은 반시계 (왼쪽 지그재그)


def test_forward_only_end_alignment():
    """늘 전진, n 짝수: 끝 헤딩 π − α → align_s 동안 출발 헤딩으로 (−(π − α))."""
    z = Zigzag(n=4, reverse=False, rot_s=1.2, align_s=2.0)
    assert z.dtheta == pytest.approx(-(math.pi - math.radians(10.0))) and z.T_align == 2.0
    assert abs(z.at(z.T_total)[0][2]) < 1e-12


def test_pass_sharp_no_stop_same_shape():
    """2026-11-25 corner_mode pass, turn_m 0 (꺾인 채): 꼭짓점·모양은 stop 과 같고, 출발·끝 말고는 서지 않는다 (꼭짓점 속력 = corner_speed),
    쉼·제자리 회전 없음 (ω = 0), 직선 사이 시간 틈 없음, 늘 전진 (reverse False)."""
    kw = dict(n=10, angle_deg=30.0, accel=1.15, profile="square", reverse=False)
    z0, z = Zigzag(rot_s=1.1, **kw), Zigzag(corner_mode="pass", corner_speed=0.5, turn_m=0.0, **kw)
    assert all(np.allclose(p, q) for p, q in zip(z0.corners, z.corners))
    assert z.T_corner == 0.0 and z.rot == 0.0 and z.dwell == 0.0
    assert z.t_start[1:] == pytest.approx(z.t_stop[:-1])
    ts = np.linspace(1e-3, z.T_path - 1e-3, 5000)
    sp = np.array([z.at(t)[1] for t in ts])
    assert sp[:, 0].min() > 0.0 and np.abs(sp[:, 1]).max() < 1e-12 and np.all(sp[:, 2] == 0.0)
    for k in range(1, z.n):                                             # 꼭짓점 k 를 지나는 속력
        assert z.at(z.t_start[k])[1][0] == pytest.approx(0.5)
        assert np.allclose(z.at(z.t_start[k])[0][:2], z.corners[k])
    assert z.T_path < z0.T_path
    assert np.hypot(*z.at(0.0)[1][:2]) == 0.0 and np.hypot(*z.at(z.T_path)[1][:2]) == 0.0


def test_pass_sharp_velocity_matches_finite_difference_off_corners():
    z = Zigzag(corner_mode="pass", corner_speed=0.4, n=4, angle_deg=-60.0, accel=1.15, profile="square", v_max=0.8, turn_m=0.0)
    o = (0.3, -0.2, 0.7)
    for t in np.linspace(0.01, z.T_path - 0.01, 3000):
        if min(abs(t - x) for x in z.t_start[1:]) < 1e-4:               # 꼭짓점 계단은 빼고
            continue
        assert np.allclose(_fd_body(z, t, o), z.at(t, o)[1], atol=1e-4)


@pytest.mark.parametrize("kw", [dict(n=10, angle_deg=30.0, turn_m=0.5, corner_speed=0.3),
                                dict(n=4, angle_deg=-60.0, turn_m=0.3, corner_speed=0.4, v_max=0.8),
                                dict(n=3, angle_deg=90.0, turn_m=0.2, corner_speed=0.5, dwell_s=0.0)])
def test_pass_round_continuous_curvature_and_geometry(kw):
    """turn_m > 0 (cubic spiral): 위치·속도·heading·ω 가 이어지고 (ω 계단 없음), v_y = 0, 출발·끝 말고 v_x ≥ corner_speed 아래로 안 떨어짐,
    turn 이 꼭짓점 앞뒤 turn_m 지점에서 시작·끝나고 heading 이 다음 직선 방향, 최대 곡률 1.5·(π − α)/L_t, 속도 vs 위치 유한차분."""
    z = Zigzag(corner_mode="pass", accel=1.15, profile="square", **kw)
    assert z.round_ and not z.reverse and z.max_curvature() == pytest.approx(1.5 * (math.pi - z.alpha) / z.Lt)
    ts = np.linspace(0.0, z.T_path, 20001)
    XU = [z.at(t) for t in ts]
    X = np.array([a for a, _ in XU]); U = np.array([b for _, b in XU])
    assert np.abs(U[:, 1]).max() < 1e-12
    inner = (ts > z.legs[0].t1 + 0.01) & (ts < z.T_path - z.legs[-1].t3 - 0.01)
    assert U[inner, 0].min() >= z.vc - 1e-9
    dt = ts[1] - ts[0]
    assert np.abs(np.diff(U[:, 2])).max() < 0.05 and np.abs(np.diff(U[:, 0])).max() <= z.A * dt + 1e-9
    assert np.abs(np.diff(X[:, :2], axis=0)).max() < 2.0 * z.V * dt
    assert np.abs([wrap(x) for x in np.diff(X[:, 2])]).max() < 0.05
    for s0, T, kind, k in z.seg:
        if kind != "turn":
            continue
        Xa, _ = z.at(s0 + 1e-9); Xb, _ = z.at(s0 + T - 1e-9)
        assert np.allclose(Xa[:2], z.corners[k + 1] - z.turn_m * z.dirs[k], atol=1e-6)
        assert np.allclose(Xb[:2], z.corners[k + 1] + z.turn_m * z.dirs[k + 1], atol=1e-6)
        assert wrap(Xb[2] - z.heads[k + 1]) == pytest.approx(0.0, abs=1e-6)
    o = (0.3, -0.2, 0.7)
    for t in np.linspace(0.003, z.T_path - 0.003, 3000):
        assert np.allclose(_fd_body(z, t, o), z.at(t, o)[1], atol=1e-5)


def test_pass_round_segment_window_covers_drive():
    z = Zigzag(corner_mode="pass", accel=1.15, profile="square", n=5, angle_deg=30.0, turn_m=0.4, corner_speed=0.3)
    assert z.t_start[0] == 0.0 and z.t_stop[-1] == pytest.approx(z.T_path) and z.t_start[1:] == pytest.approx(z.t_stop[:-1])
    for t in np.linspace(0.0, z.T_path, 500):
        assert z.segment_window(t) is not None


@pytest.mark.parametrize("kw", [dict(profile="sine"), dict(profile="square", corner_speed=0.0),
                                dict(profile="square", corner_speed=3.0, turn_m=0.0), dict(profile="square", corner_mode="glide"),
                                dict(profile="square", turn_m=0.6), dict(profile="square", turn_m=-0.1),
                                dict(profile="square", yaw_mode="fixed")])
def test_pass_rejects(kw):
    with pytest.raises(ValueError):
        Zigzag(**{"corner_mode": "pass", **kw})
