# pytest -q controller/mpc/test/test_seq_plan.py   (ROS 불필요) — 2026-11-28 amrseq 순서·출발점
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, os.path.join(HERE, ".."))
from seq import compose, default_n, end_offset_of, leg_origin, origin_params, plan  # noqa: E402
from amr_mpc.core import presets, ref_config  # noqa: E402
from amr_mpc.body import mpc as mpc_mod  # noqa: E402

CONFIG = os.path.join(HERE, "..", "config")
V_LIMIT = mpc_mod.R_WHEEL * 38.75          # 기준 노드 = R_WHEEL × launch max_wheel_rad_per_sec (실차 기본 38.75 → 1.55 m/s)


def test_plan_closed_paths_interleave():
    legs = plan("circles8", 6)
    assert len(legs) == 18 and all(g["kind"] == "data" and g["dir"] is None for g in legs)
    assert [g["ctrl"] for g in legs[:6]] == ["mpc", "pp", "adpp", "mpc", "pp", "adpp"]
    assert [g["i"] for g in legs] == list(range(1, 19))
    assert default_n("circles8") == default_n("polyline") == default_n("zigzag") == 5   # 2026-11-28 세 경로 모두 5


def test_plan_zigzag_returns_and_direction_balance():
    """zigzag 6 회차: data 18 + 복귀 18, 제어기마다 fwd 3 · rev 3 (사용자 결정 2026-11-28)."""
    legs = plan("zigzag", 6)
    data = [g for g in legs if g["kind"] == "data"]
    assert len(data) == 18 and len(legs) == 36
    assert all(legs[2 * k + 1]["kind"] == "return" and legs[2 * k + 1]["ctrl"] == "mpc" for k in range(18))
    for c in ("mpc", "pp", "adpp"):
        dirs = [g["dir"] for g in data if g["ctrl"] == c]
        assert dirs.count("fwd") == 3 and dirs.count("rev") == 3, c
    assert data[-1]["dir"] == "rev"                     # 끝나면 출발점 A 로 돌아와 있다


def _cfg(name):
    base = {k: v for k, v in presets.track_base(CONFIG).items()}
    cfg = dict(base, **presets.reference_values(CONFIG, name))
    for k in ref_config.LIVE:                           # track_base 가 안 채운 값 (origin_* 등) 은 기본값으로
        cfg.setdefault(k, {"origin_mode": "robot", "origin_x": 0.0, "origin_y": 0.0, "origin_yaw_deg": 0.0,
                           "start_tolerance_xy": 0.3, "start_tolerance_yaw_deg": 30.0, "lead_speed": 0.1, "lead_yaw_rate": 0.5}.get(k))
    return cfg


@pytest.mark.parametrize("A", [(0.0, 0.0, 0.0), (1.2, -0.7, math.radians(37.0))])
def test_zigzag_anchor_chain_returns_to_A(A):
    """s2 zigzag: fwd (A 출발) 끝 = 복귀 다리 출발점, 180° 돈 뒤 rev 출발점, rev 끝 = A. 경로 at() 으로 검산."""
    cfg = _cfg("s2")
    path, _, _ = ref_config.build(cfg, V_LIMIT, 1.0, mpc_mod.K_GEOM)
    end = end_offset_of(cfg, V_LIMIT, 1.0, mpc_mod.K_GEOM)
    assert math.hypot(end[0], end[1]) > 1.0 and abs(end[2]) < 1e-9          # 끝은 출발점에서 떨어져 있고 방향은 같다
    legs = plan("zigzag", 1)
    o = [leg_origin(g, A, end) for g in legs]          # data fwd, return, data rev, return, data fwd, return
    fin = lambda origin: path.at(path.T_total + 1.0, origin)[0]
    e1 = fin(o[0])
    assert np.allclose(e1[:2], o[1][:2], atol=1e-9) and abs(math.remainder(e1[2] - o[1][2], 2 * math.pi)) < 1e-9
    assert np.allclose(o[2][:2], o[1][:2]) and abs(math.remainder(o[2][2] - o[1][2] - math.pi, 2 * math.pi)) < 1e-9
    e2 = fin(o[2])
    assert np.allclose(e2[:2], A[:2], atol=1e-9) and abs(math.remainder(e2[2] - A[2] - math.pi, 2 * math.pi)) < 1e-9
    assert np.allclose(o[3][:2], A[:2]) and abs(math.remainder(o[4][2] - A[2], 2 * math.pi)) < 1e-9


def test_closed_paths_end_at_start():
    """s1 circles8 · s3 polyline 은 출발점에서 끝난다 → 복귀 없이 다음 회차 (anchor = A 그대로)."""
    for name in ("s1", "s3"):
        cfg = _cfg(name)
        assert end_offset_of(cfg, V_LIMIT, 1.0, mpc_mod.K_GEOM) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert leg_origin(plan("circles8", 1)[0], (1.0, 2.0, 0.3), (0, 0, 0)) == (1.0, 2.0, 0.3)


def test_origin_params_wraps_yaw():
    p = origin_params((1.0, 2.0, math.radians(190.0)))
    assert p["origin_mode"] == "fixed" and p["origin_yaw_deg"] == pytest.approx(-170.0)
    assert compose((0, 0, math.pi / 2), (1, 0, 0))[:2] == pytest.approx((0.0, 1.0), abs=1e-12)


@pytest.mark.parametrize("name", ["s1", "s2", "s3"])
def test_presets_pass_budget_and_use_known_names(name):
    """s1·s2·s3 기준 값이 부하 검사를 통과하고, pp·fault·mpc 이름이 실행 중 바꿀 수 있는 값이다."""
    p = presets.load_presets(CONFIG)[name]
    ref_config.build(_cfg(name), V_LIMIT, 1.0, mpc_mod.K_GEOM)
    assert set(p["pp"]) == {"lookahead_time", "lookahead_min", "lookahead_max",
                            "adpp_lookahead_time", "adpp_lookahead_min", "adpp_lookahead_max"}
    assert set(p["fault"]) == {"max_track_error"}
    assert set(p["mpc"]) <= {"W", "w_dudt", "r_u", "q_null", "mu_hat", "slip_fb_enable", "slip_fb_scale", "slip_eta_enable",
                             "disturbance_enable", "disturbance_alpha", "roller_rr", "inertia_z"}


@pytest.mark.parametrize("n, same_floor", [(5, True), (3, True), (4, False)])
def test_zigzag_rev_floor_needs_odd_n(n, same_floor):
    """180° 뒤 rev 회차가 fwd 와 같은 바닥 선을 지나는 것은 zigzag_n 홀수일 때만 (seq.py 가 짝수를 막는 근거)."""
    cfg = dict(_cfg("s2"), zigzag_n=n, zigzag_length=2.0)
    path, _, _ = ref_config.build(cfg, V_LIMIT, 1.0, mpc_mod.K_GEOM)
    end = end_offset_of(cfg, V_LIMIT, 1.0, mpc_mod.K_GEOM)
    ts = np.linspace(0.0, path.T_total, 1500)
    X = np.array([path.at(t)[0][:2] for t in ts])
    Y = np.array([path.at(t, leg_origin(plan("zigzag", 1)[2], (0.0, 0.0, 0.0), end))[0][:2] for t in ts])
    d = np.min(np.hypot(Y[:, None, 0] - X[None, :, 0], Y[:, None, 1] - X[None, :, 1]), axis=1).max()
    assert (d < 0.01) == same_floor
