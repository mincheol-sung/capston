# pytest -q controller/mpc/test   (ROS 불필요) — core/path_follow.py (2026-11-23, figure8_reference path_follow_lead_s)
import numpy as np
import pytest

from amr_mpc.core import path_follow as pf
from amr_mpc.core.figure8 import Figure8

O = np.zeros(3)


@pytest.fixture(scope="module")
def fig():
    """실차 계단 변조 8자 (circles8 width 1.6, speed 0.69 · accel 3.0, 2 바퀴)."""
    return Figure8(width=1.6, height=0.8, speed=0.69, accel=3.0, laps=2, yaw_mode="tangent", align_s=1.5, curve="circles",
                   speed_mod_amp=0.5, speed_mod_cycles=2, speed_mod_phase_deg=-90, speed_mod_shape="square")


def _xy(f):
    return lambda t: f.at(t, O)[0][:2]


def _normal(f, t):
    X, u = f.at(t, O)
    c, s = np.cos(X[2]), np.sin(X[2])
    vw = np.array([c * u[0] - s * u[1], s * u[0] + c * u[1]])
    tg = vw / np.linalg.norm(vw)
    return np.array([-tg[1], tg[0]])


@pytest.mark.parametrize("t_true", [2.0, 5.3, 9.9, 14.0])
def test_project_finds_foot_of_perpendicular(fig, t_true):
    """경로 점에서 법선으로 4 cm 옆에 둔 로봇 → 정사영 시각이 그 점 (0.001 s 격자, 앞뒤 창 안의 anchor 에서)."""
    pose = _xy(fig)(t_true) + 0.04 * _normal(fig, t_true)
    tp = pf.project(_xy(fig), pose, t_true - 0.15, 0.0, fig.T_total)
    assert tp == pytest.approx(t_true, abs=0.003)


def test_project_stays_on_this_lap_at_crossing(fig):
    """교차점 (원점) 은 바퀴마다 지나간다 — 창이 직전 정사영 근처라 다른 바퀴 시각으로 안 튄다."""
    ts = np.arange(0.5, fig.T_total - 0.5, 0.01)
    t_cross = [t for t in ts if np.hypot(*_xy(fig)(t)) < 0.01]
    assert len(t_cross) > 3                                   # 2 바퀴 × 원 2 개마다
    for tc in t_cross[::5]:
        tp = pf.project(_xy(fig), np.zeros(2), tc - 0.1, 0.0, fig.T_total)
        assert abs(tp - tc) < 0.3


def test_govern_clamps():
    assert pf.govern(5.0, 4.0, 0.05, 1.0) == (4.05, 1)        # 로봇이 늦다 → 기다림
    assert pf.govern(3.9, 4.0, 0.05, 1.0) == (4.0, -1)        # 로봇이 앞섰다 → 앞당김
    assert pf.govern(4.03, 4.0, 0.05, 1.0) == (4.03, 0)       # lead 안 → 그대로 (시간 추종)
    assert pf.govern(5.0, 4.0, 0.05, 0.05) == (5.0, 0)        # 기준 속력 < 0.1 (출발·정지 근처) → 시계대로
    assert pf.govern(5.0, 4.0, 0.05, 1.0, decel=True) == (5.0, 0)   # 앞에 감속 → 위쪽 상한 안 걸고 시계대로
    assert pf.govern(3.9, 4.0, 0.05, 1.0, decel=True) == (4.0, -1)  # 앞섬은 그대로 당긴다


def test_decel_ahead_sees_end_stop_and_step_down(fig):
    """끝 정지 (실차 시뮬 멈춤 자리: 끝 0.24 s 전) 와 계단 내림 앞에서 참, 등속 구간 가운데에서 거짓."""
    speed = lambda t: float(np.hypot(*fig.at(t, O)[1][:2]))
    assert pf.decel_ahead(speed, fig.T_path - 0.24, 0.05)
    ts = np.arange(0.5, fig.T_path - 0.5, 0.01); v = np.array([speed(t) for t in ts])
    k = int(np.argmax(np.diff(v) < -0.01))                    # 첫 계단 내림 시작
    assert pf.decel_ahead(speed, ts[k] - 0.2, 0.05)
    flat = [t for t in ts if abs(speed(t) - speed(t + 0.4)) < 1e-6 and abs(speed(t) - speed(t - 0.05)) < 1e-6]
    assert flat and not pf.decel_ahead(speed, flat[len(flat) // 2], 0.05)


def test_slow_robot_drive_takes_longer_but_never_lags_more_than_lead(fig):
    """프로파일의 70 % 로만 진행하는 로봇 (경로 위, 4 cm 옆) — 기준이 기다려 앞섬 ≤ lead, 주행은 ≈ 1/0.7 배 걸린다."""
    dt, lead = 0.02, 0.05
    t_ref, t_rob, t_proj, wall = 0.0, 0.0, 0.0, 0.0
    lag_max = 0.0
    while t_rob < fig.T_path - 0.2 and wall < 60.0:
        wall += dt
        t_rob += 0.7 * dt                                      # 로봇은 경로 시각으로 0.7 배속
        pose = _xy(fig)(t_rob) + 0.04 * _normal(fig, max(t_rob, 0.3))
        t_proj = pf.project(_xy(fig), pose, t_proj, 0.0, fig.T_total)
        speed = lambda x: float(np.hypot(*fig.at(x, O)[1][:2]))
        v = speed(t_proj); dec = pf.decel_ahead(speed, t_proj, lead)
        t_ref, _ = pf.govern(t_ref + dt, t_proj, lead, v, decel=dec)
        if v >= pf.V_MIN and not dec:
            lag_max = max(lag_max, t_ref - t_proj)
    assert lag_max <= lead + 2 * 0.7 * dt + 1e-9              # 감속 구간을 시계대로 지난 직후 한두 스텝 몫
    assert wall == pytest.approx(fig.T_path / 0.7, rel=0.05)
