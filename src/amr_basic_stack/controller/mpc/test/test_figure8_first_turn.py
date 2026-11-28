"""figure8·circles8 first_turn (2026-11-28): right = 출발 헤딩 축에 대해 거울. left (기본) 는 예전 출력 그대로."""
import math

import numpy as np
import pytest

from amr_mpc.core.figure8 import Figure8
from amr_mpc.core import ref_config

CFGS = [dict(curve="circles", width=2.0, speed=1.0, accel=1.0, yaw_mode="tangent"),
        dict(curve="circles", width=2.0, speed=0.8, accel=1.0, yaw_mode="tangent", crossing_blend=0.6, laps=2),
        dict(curve="circles", width=3.0, speed=1.0, accel=1.5, yaw_mode="fixed"),
        dict(curve="gerono", width=1.4, height=0.8, speed=0.25, accel=0.4, yaw_mode="tangent"),
        dict(curve="gerono", width=1.4, height=0.8, speed=0.25, accel=0.4, yaw_mode="spin")]


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _traj(f, origin=(0.0, 0.0, 0.0), n=600):
    return np.array([np.concatenate(f.at(t, origin=origin)) for t in np.linspace(0.0, f.T_total + 0.5, n)])


@pytest.mark.parametrize("cfg", CFGS)
def test_left_is_default(cfg):
    assert np.array_equal(_traj(Figure8(**cfg)), _traj(Figure8(**cfg, first_turn="left")))


@pytest.mark.parametrize("cfg", CFGS)
def test_right_is_mirror_of_left(cfg):
    L, R = _traj(Figure8(**cfg)), _traj(Figure8(**cfg, first_turn="right"))
    exp = np.column_stack([L[:, 0], -L[:, 1], -L[:, 2], L[:, 3], -L[:, 4], -L[:, 5]])   # [x, y, yaw, vx, vy, ω]
    d = np.abs(R - exp)
    d[:, 2] = np.abs(_wrap(R[:, 2] - exp[:, 2]))
    assert d.max() < 1e-12
    pl, pr = np.array(Figure8(**cfg).path_xy(200)), np.array(Figure8(**cfg, first_turn="right").path_xy(200))
    assert np.allclose(pr, pl * [1.0, -1.0], atol=1e-12)
    assert Figure8(**cfg).peak_load(0.1932) == Figure8(**cfg, first_turn="right").peak_load(0.1932)


def test_circles_right_turns_clockwise_first():
    f = Figure8(curve="circles", width=2.0, speed=1.0, accel=1.0, yaw_mode="tangent", first_turn="right")
    _, u = f.at(f.T_align + 1.0)
    assert u[2] < 0.0                               # 첫 원에서 요레이트 음수 (오른쪽, 시계)
    X, _ = f.at(f.T_align + 1.0)
    assert X[1] < 0.0                               # 첫 원은 출발 헤딩의 오른쪽 (로컬 y < 0)


def test_bad_value():
    with pytest.raises(ValueError):
        Figure8(curve="circles", width=2.0, speed=1.0, accel=1.0, first_turn="up")


def test_ref_config_passes_first_turn():
    assert "first_turn" in ref_config.LIVE and ref_config.CHOICES["first_turn"] == ("left", "right")
    assert "first_turn" in ref_config.RELEVANT["circles8"]
    cfg = dict(shape="circles8", width=2.0, height=1.0, speed=1.0, accel=1.0, laps=1.0, yaw_mode="tangent", align_s=1.5,
               lap_speeds=[0.0], crossing_blend_m=0.0, spin_rate=1.0, speed_mod_amp=0.0, speed_mod_cycles=2.0,
               speed_mod_phase_deg=0.0, speed_mod_shape="sin", first_turn="right")
    p = ref_config.make_path(cfg, 1.55, 1.0)
    assert p.mirror and math.isclose(p.R, 0.5)
