# pytest -q controller/mpc/test   (ROS 불필요) — core/fig8_limits.py (2026-11-23, speed:=max·accel:=max)
import os

import pytest

from amr_mpc.core import fig8_limits as fl

CFG = os.path.join(os.path.dirname(__file__), "..", "config")
# 2026-11-23 실차 circles8 (114635·114459) 설정. μ̂·하중 몫은 그날 기동 로그 값으로 고정 (yaml 이 바뀌어도 시험이 그대로)
TRACK = dict(shape="circles8", width=2.0, height=1.5, laps=2, yaw_mode="tangent", align_s=1.5, crossing_blend_m=0.0, spin_rate=1.0,
             speed_mod_amp=0.5, speed_mod_cycles=2, speed_mod_phase_deg=-90, speed_mod_shape="sin")
C = fl.plant_constants(CFG, mu_hat_wheels=[0.598, 0.489, 0.547, 0.703], wheel_load_share=[0.2388, 0.2392, 0.2608, 0.2612], use_w=True,
                       com_height=0.0)
CAP = 1.0 * C["r"] * 370.0 * 2 * 3.141592653589793 / 60.0          # load_budget 1 × V_LIMIT 1.55


@pytest.fixture(scope="module")
def best():
    return fl.max_speed_accel(fl.figure8_maker(TRACK), C, CAP, C["k"])


def test_tau_bar_matches_controller_log():
    """τ̄_i = (r/√2)·μ̂_i·N_i·c_s — 그날 torque_mpc 기동 로그 [0.1445 0.1184 0.1444 0.1859]."""
    assert C["tau_bar"] == pytest.approx([0.1445, 0.1184, 0.1444, 0.1859], abs=1e-4)


def test_reproduces_2026_11_23(best):
    """analysis/torque_mpc_sim/2026-11-23_fig8_max 의 손 격자 계산 (speed 0.65 · accel 1.3, 앞먹임 0.996, 부하 87 %)."""
    v, a, info = best
    assert (v, a) == (pytest.approx(0.65), pytest.approx(1.30))
    assert info["ff"] == pytest.approx(0.996, abs=0.003) and info["load"] == pytest.approx(0.87, abs=0.01)


def test_result_is_the_largest_on_grid(best):
    """찾은 값은 두 상한 안이고, speed 를 한 칸 (0.01) 또는 accel 을 한 칸 (0.05) 올리면 넘는다."""
    v, a, _ = best
    make = fl.figure8_maker(TRACK)
    assert fl.ff_ratio(make(v, a), C) <= 1.0
    assert fl.ff_ratio(make(v + fl.V_STEP, a), C) > 1.0
    assert fl.ff_ratio(make(v, a + fl.A_STEP), C) > 1.0


def test_fixed_speed_only_raises_accel_and_margin_lowers():
    v, a, info = fl.max_speed_accel(fl.figure8_maker(TRACK), C, CAP, C["k"], speed=0.6)
    assert v == 0.6 and info["ff"] <= 1.0 and a >= 1.3
    v9, a9, info9 = fl.max_speed_accel(fl.figure8_maker(TRACK), C, CAP, C["k"], max_ff_ratio=0.9)
    assert v9 < 0.65 and info9["ff"] <= 0.9


def test_load_budget_binds_when_friction_is_generous():
    """μ̂ 를 크게 주면 C 상한이 안 걸리고 부하 예산 (speed ≤ 0.745) 이 묶는다."""
    c = fl.plant_constants(CFG, mu_hat_wheels=[2.0] * 4, wheel_load_share=[0.25] * 4, use_w=True)
    v, _, info = fl.max_speed_accel(fl.figure8_maker(TRACK), c, CAP, c["k"])
    assert v == pytest.approx(0.74) and 0.98 < info["load"] <= 1.0


SQ = dict(shape="square", laps=1, yaw_mode="tangent", align_s=1.5, square_side=1.5, square_corner_speed=0.1, square_yaw_blend=0.3,
          square_corner_mode="pass", square_rot_s=1.0, square_dwell_s=0.1, square_v_max=0.0)


def test_square_accel_is_edge_limit_not_corner():
    """정사각형 max: 변 직선 등가속 U_s = m_eff·a·r/4 가 min τ̄ (FR 0.1184) 안 → 4·0.1184/(6.785·0.04) = 1.745 → 1.70.
    꼭짓점 앞먹임 (pass 2.4 대) 은 기준에서 뺀다 — 실차 114214 (accel 1.5, v_max 1.5) 가 그걸로 완주."""
    v, a, info = fl.square_max(C, CAP, SQ)
    assert a == pytest.approx(1.70) and info["edge_ff"] == pytest.approx(C["me"] * 1.70 * C["r"] / 4 / 0.1184, rel=1e-3)
    assert info["edge_ff"] <= 1.0 < C["me"] * (a + fl.A_STEP) * C["r"] / 4 / min(C["tau_bar"])
    assert v == pytest.approx(CAP) and info["ff"] > 2.0                   # 변 최고는 부하 예산 1.55 에서 잘림, 꼭짓점은 상한 밖
    _, _, stop = fl.square_max(C, CAP, dict(SQ, square_corner_mode="stop"))
    assert stop["ff"] <= 1.0                                              # stop 은 꼭짓점까지 상한 안


def test_square_given_accel_keeps_it():
    v, a, info = fl.square_max(C, CAP, dict(SQ, square_v_max=1.5), accel=1.5)
    assert (v, a) == (pytest.approx(1.5), 1.5) and info["edge_ff"] < 1.0


def _lt(h, **kw):
    return fl.plant_constants(CFG, mu_hat_wheels=[0.598, 0.489, 0.547, 0.703], wheel_load_share=[0.2388, 0.2392, 0.2608, 0.2612],
                              use_w=True, com_height=h, **kw)


def test_load_transfer_off_switches_equal_static():
    """하중 이동 (2026-11-23): h > 0 이어도 가중 배분·스텝별 상한을 둘 다 끄면 N 이 아무 데도 안 들어가 h 0 과 같다."""
    f = fl.figure8_maker(TRACK)(0.5, 0.6)
    assert fl.ff_ratio(f, _lt(0.2, load_weighted_alloc=False, load_dynamic_cap=False)) == pytest.approx(fl.ff_ratio(f, C), rel=1e-12)


def test_load_transfer_ratio_matches_per_row_formula():
    """_ratio 의 벡터 계산 = 행마다 torque_mpc.cpp 식: N_i = 몫 m g − s_x m a_x h/2L − s_y m a_y h/2W (≥ 0.2 정지),
    cap = τ̄ N/N0, U = W⁻¹Fᵀ(FW⁻¹Fᵀ)⁻¹ Fb, 비 = max |U_i|/cap_i. 몸체 힘도 그대로 (F U = Fb)."""
    import numpy as np
    c = _lt(0.2)
    rng = np.random.default_rng(1)
    acc = rng.normal(scale=3.0, size=(20, 3)); th = rng.uniform(-3, 3, 20); cs, sn = np.cos(th), np.sin(th)
    Fb = rng.normal(scale=5.0, size=(20, 3))
    got = fl._ratio(Fb, acc, cs, sn, c)
    for n in range(20):
        ax, ay = cs[n] * acc[n, 0] + sn[n] * acc[n, 1], -sn[n] * acc[n, 0] + cs[n] * acc[n, 1]
        N = c["n0"] - np.array([1, 1, -1, -1]) * c["m"] * ax * 0.2 / (2 * c["L"]) - np.array([1, -1, 1, -1]) * c["m"] * ay * 0.2 / (2 * c["W"])
        N = np.maximum(N, 0.2 * c["n0"]); cap = c["tau_bar"] * N / c["n0"]
        Wi = np.diag(cap ** 2); U = Wi @ c["F"].T @ np.linalg.solve(c["F"] @ Wi @ c["F"].T, Fb[n])
        assert c["F"] @ U == pytest.approx(Fb[n], abs=1e-9)
        assert got[n] == pytest.approx(np.max(np.abs(U) / cap), rel=1e-9)
