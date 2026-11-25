"""amr_mpc/torque/wheel_dynamics.py · tools/wheel_param_id.py — 바퀴 회전자 식별 지시서 (2026-11-24) 4절 시험 I1~I6.

I1 합성 자료 (잡음 없음) 에서 회귀가 J_w·τ_c·b_v 를 되찾는다 (상대 1e-3)
I2 ω 에 잡음을 넣고 차단 주파수별 오차를 표로 (보고용 — 통과 기준 없음, 표가 나오는지만)
I3 τ 0.03 단계의 일정 속도 20 rad/s · 시정수 1.94 s · 자유 감속 시작 ω̇ −30.9 rad/s² (상대 1e-2)
I4 견인 토크 함수가 지시서 1.3절 검산표 두 행과 같다 (1e-5)
I5 Σ τ_g / r = m·a (1e-6)
I6 주행 저항 표 모드에서는 τ_c·b_v 항이 더해지지 않는다 (1e-12)
"""
import importlib.util
import math
import os

import numpy as np
import pytest

from amr_mpc.torque import wheel_dynamics as wd

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("wheel_param_id", os.path.join(HERE, "..", "tools", "wheel_param_id.py"))
wpi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wpi)

# 지시서의 예시값 (검산용 — 파라미터 기본값이 아니다)
J, TC, BV = 9.7e-4, 0.020, 5e-4
TAUS, HOLDS = [0.03, 0.035, 0.025, 0.03, 0.0], [8.0, 8.0, 8.0, 8.0, 3.0]


def synth_runs(noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for s in (1.0, -1.0):
        t, tau, w = wd.simulate_lifted([s * x for x in TAUS], HOLDS, J, TC, BV)
        out.append({"t": t, "mode": np.array(["torque"] * len(t)), "v": w + rng.normal(0.0, noise, len(w)),
                    "tau_q": tau, "tau_reply": tau, "tau_cmd": tau})
    return out


def test_I1_regression_recovers_parameters():
    res = wpi.identify([wpi.prepare(r) for r in synth_runs()])
    for key in (("cmd", "both"), ("cmd", "fwd"), ("cmd", "rev"), ("fb", "both"),
                ("cmd", "both", "acc"), ("cmd", "fwd", "acc"), ("cmd", "rev", "acc")):      # 지시서 식 / ω̇ 를 왼쪽에 둔 회귀
        f = res[key]
        assert f["J_w"] == pytest.approx(J, rel=1e-3), key
        assert f["tau_c"] == pytest.approx(TC, rel=1e-3), key
        assert f["b_v"] == pytest.approx(BV, rel=1e-3), key
        assert f["r2"] > 0.999999 and f["rms"] < 1e-6
    # 교차 확인: 일정 속도 직선 — 8 s 유지가 시정수 1.94 s 의 4.1 배뿐이라 끝 1 s 평균이 아직 덜 자리 잡았다 (b_v +3 %)
    ln = res[("cmd", "fwd", "line")]
    assert ln["n"] == 4 and ln["tau_c"] == pytest.approx(TC, rel=0.03) and ln["b_v"] == pytest.approx(BV, rel=0.05)
    assert res[("cmd", "fwd", "coast_fit")]["J_w"] == pytest.approx(J, rel=1e-3)
    assert res[("cmd", "rev", "coast_line")]["J_w"] == pytest.approx(J, rel=0.03)


def test_single_transient_cannot_separate_inertia_from_viscous():
    """과도 구간 하나만 있으면 ω̇ = −(b/J)(ω − ω_ss) 라 회귀 행렬의 열이 서로 종속 — J_w 와 b_v 가 안 갈린다 (조건수로 드러난다)."""
    t, tau, w = wd.simulate_lifted([0.03], [8.0], J, TC, BV, w0=5.0)
    one = wd.fit_wheel_params(tau, w, np.gradient(w, 0.01), np.abs(w) > 3.0)
    res = wpi.identify([wpi.prepare(r) for r in synth_runs()])
    assert one["cond"] > 1e4 and res[("cmd", "both")]["cond"] < 50.0


def test_I2_noise_table():
    rows = wpi.noise_study(sigmas=(0.0064, 0.5), fcs=(5.0, 10.0), trials=3)
    assert len(rows) == 4 and all(r["mean"].shape == (3,) for r in rows)
    quiet = next(r for r in rows if r["sigma"] == 0.0064 and r["fc"] == 10.0)
    assert np.all(np.abs(quiet["mean"]) < 1.0)                 # 기록 한 칸 수준 잡음이면 세 값 모두 1 % 안
    loud = next(r for r in rows if r["sigma"] == 0.5 and r["fc"] == 10.0)
    assert loud["mean"][0] < -20.0                             # 지시서 식 (ω̇ 가 설명 변수): 속도 잡음만큼 J_w 가 작게 나온다
    assert abs(loud["mean_acc"][0]) < 5.0                      # ω̇ 를 왼쪽에 두면 치우치지 않는다


def test_I3_step_and_coast_numbers():
    t, tau, w = wd.simulate_lifted([0.03, 0.0], [20.0, 3.0], J, TC, BV)
    i = int(round(20.0 / 0.01))
    assert w[i - 1] == pytest.approx(20.0, rel=1e-2)                           # ω_ss = (τ − τ_c)/b_v
    k = int(np.argmax(w >= 20.0 * (1.0 - math.exp(-1.0))))                     # 63.2 % 에 닿는 시각 = 시정수
    assert t[k] == pytest.approx(1.94, rel=1e-2) and J / BV == pytest.approx(1.94, rel=1e-2)
    assert (w[i + 1] - w[i]) / 0.01 == pytest.approx(-30.9, rel=1e-2)          # −(τ_c + b_v·20)/J_w
    assert w[-1] == 0.0                                                        # 서면 그대로 선다


def _table_row(tau, tau_res, m=4.56, r=0.04):
    m_eq = m + 4.0 * J / r ** 2
    a = 4.0 * (tau - tau_res) / (r * m_eq)
    return a, a / r, m, r


@pytest.mark.parametrize("tau_res, a_ref, wd_ref, jwd_ref, tg_ref, f_ref", [
    (0.0, 1.4316, 35.791, 0.03472, 0.06528, 6.528),
    (0.02, 1.1453, 28.633, 0.02777, 0.05223, 5.223),
])
def test_I4_I5_traction_torque_matches_document_table(tau_res, a_ref, wd_ref, jwd_ref, tg_ref, f_ref):
    a, wdot, m, r = _table_row(0.1, tau_res)
    assert a == pytest.approx(a_ref, abs=5e-5) and wdot == pytest.approx(wd_ref, abs=5e-4)
    fr = wd.CoulombViscous(tau_res, 0.0)
    tg = wd.traction_torque(0.1, 10.0, wdot, J, fr)
    assert J * wdot == pytest.approx(jwd_ref, abs=1e-5)                        # I4
    assert float(tg) == pytest.approx(tg_ref, abs=1e-5)
    chk = wd.body_force_consistency(np.full(4, float(tg)), r, m, a)            # I5: Σ τ_g / r = m a
    assert chk["F_x"][0] == pytest.approx(m * a, abs=1e-6) and abs(chk["mean_diff"]) < 1e-6
    assert chk["F_x"][0] == pytest.approx(f_ref, abs=5e-4)


def test_I6_resistance_table_is_not_double_subtracted():
    w = np.array([-20.0, -5.0, 0.0, 5.0, 12.5, 40.0])
    tab = wd.ResistanceTable([0.0, 10.0, 20.0], [0.020, 0.030, 0.045])
    expect_fric = np.sign(w) * np.array([0.045, 0.025, 0.020, 0.025, 0.03375, 0.045])       # 표 밖은 끝 값
    np.testing.assert_allclose(tab(w), expect_fric, atol=1e-12)
    tau, wdot = 0.1, np.array([3.0, -2.0, 0.0, 1.0, 4.0, 0.5])
    tg = wd.traction_torque(tau, w, wdot, J, tab)
    np.testing.assert_allclose(tg, tau - expect_fric - J * wdot, atol=1e-12)               # τ_c·b_v 항이 더해지지 않는다
    cv = wd.traction_torque(tau, w, wdot, J, wd.CoulombViscous(TC, BV))
    np.testing.assert_allclose(cv, tau - (TC * np.sign(w) + BV * w) - J * wdot, atol=1e-12)
    with pytest.raises(ValueError):
        wd.ResistanceTable([10.0, 5.0], [0.03, 0.02])


def test_traction_torque_broadcasts_per_wheel_and_smooth_sign():
    tau = np.full((3, 4), 0.05)
    w = np.array([[10.0, -10.0, 0.2, -0.2]] * 3)
    fr = wd.CoulombViscous([0.0174, 0.0206, 0.0155, 0.0157], [0.001066, 0.001158, 0.001112, 0.001177], smooth=0.5)
    tg = wd.traction_torque(tau, w, np.zeros((3, 4)), 8.9e-4, fr)
    assert tg.shape == (3, 4)
    assert tg[0, 0] == pytest.approx(0.05 - 0.0174 - 0.01066, abs=1e-9)        # tanh(20) = 1
    assert tg[0, 1] == pytest.approx(0.05 + 0.0206 + 0.01158, abs=1e-9)
    assert tg[0, 2] == pytest.approx(0.05 - 0.0155 * math.tanh(0.4) - 0.001112 * 0.2, abs=1e-12)


def test_body_force_consistency_reads_mass_error_as_slope():
    """τ_g 를 질량 5 kg 로 만든 자료를 4.56 kg 로 견주면 차이가 가속도에 비례 (기울기 = 질량 차), 일정한 힘 0.5 N 은 평균 차로."""
    a = np.linspace(-1.5, 1.5, 31)
    r = 0.04
    tg = np.repeat(((5.0 * a + 0.5) * r / 4.0)[:, None], 4, axis=1)
    chk = wd.body_force_consistency(tg, r, 4.56, a)
    assert chk["slope"] == pytest.approx(0.44, abs=1e-9) and chk["mean_diff"] == pytest.approx(0.5, abs=1e-9)


def test_resample_flags_gaps():
    t = np.r_[np.arange(0.0, 1.0, 0.0102), np.arange(1.2, 2.0, 0.0102)]        # 0.2 s 응답 누락
    tu, (x,), ok = wd.resample_uniform(t, 2.0 * t)
    assert np.allclose(x, 2.0 * tu) and abs((tu[1] - tu[0]) - 0.0102) < 1e-9
    gap = (tu > 1.0) & (tu < 1.19)
    assert not ok[gap].any() and ok[~gap & (tu < 0.98)].all()


def test_end_to_end_bench_csv_to_identification(tmp_path):
    """ak40_bench.py lifted 가 쓴 CSV (가짜 모터, 승인 순서, 전진·후진) → wheel_param_id 가 가짜 모터의 값을 되찾는다.
    속도 12 비트 기록 (0.022 rad/s)·토크 12 비트 (2.44 mN·m, "0" = ±1.22 mN·m)·바퀴 기준 부호가 모두 들어간 경로다."""
    import test_ak40_bench_lifted as tl
    for direction, tag in ((1.0, "fwd_r1"), (-1.0, "rev_r1")):
        a = tl._args(id=1, taus=",".join(str(direction * x) for x in tl.TAUS), holds=",".join(str(h) for h in tl.HOLDS),
                     csv="", out_dir=str(tmp_path), tag=tag, start_min=3.0, max_miss=10)
        m = tl.FakeMotor()
        m.lim, m.period = tl.LIM, 0.01
        assert tl.bench.lifted_main(m, a, tl.bench.lifted_prepare(a), clock=m.clock, sleep=m.sleep)["stop"] == "완료"
    files = sorted(str(p) for p in tmp_path.glob("*_lifted_FL_*.csv"))
    assert len(files) == 2
    runs = [wpi.load_csv(f) for f in files]
    assert all(r["meta"]["wheel"] == "FL" and r["meta"]["frame"] == "wheel" for r in runs)
    res = wpi.identify([wpi.prepare(r) for r in runs])
    for key in (("cmd", "both"), ("cmd", "fwd"), ("cmd", "rev"), ("cmd", "both", "acc"), ("cmd", "fwd", "acc"), ("cmd", "rev", "acc")):
        f = res[key]
        assert f["J_w"] == pytest.approx(tl.JW, rel=0.02), key
        assert f["tau_c"] == pytest.approx(tl.TC, rel=0.02), key
        assert f["b_v"] == pytest.approx(tl.BV, rel=0.02), key
    for d in ("fwd", "rev"):
        ln = res[("cmd", d, "line")]
        assert ln["n"] == 5 and ln["tau_c"] == pytest.approx(tl.TC, rel=0.03) and ln["b_v"] == pytest.approx(tl.BV, rel=0.03)
        assert res[("cmd", d, "coast_line")]["J_w"] == pytest.approx(tl.JW, rel=0.05)
    from amr_mpc.core import config
    txt = wpi.report({0: res}, config.load("torque_plant.yaml"), 10.0, str(tmp_path))
    assert "## FL" in txt and "W5 영향표" in txt and (tmp_path / "wheel_param_id.md").exists()
    wpi.plot_wheel("FL", [wpi.prepare(r) for r in runs], res, str(tmp_path / "fig.png"))
    assert (tmp_path / "fig.png").stat().st_size > 10000
