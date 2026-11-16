"""tools/jw_step_analysis.py — 토크 계단 앞뒤 가속도 차로 J 를 되찾는지 (해석해 합성 자료, 2026-11-16)."""
import importlib.util
import os

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("jw_step_analysis", os.path.join(HERE, "..", "tools", "jw_step_analysis.py"))
jwa = importlib.util.module_from_spec(spec)
spec.loader.exec_module(jwa)


def synth(J, taus, hold, tc=0.022, bv=0.0018, w0=0.0, q=0.022, seed=0):
    """마찰 τc + bv·ω (ω > 0) 의 구간별 해석해 ω(t) = ω_ss + (ω0 − ω_ss)·e^(−t·bv/J), 100 Hz, 속도 양자화 q."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, len(taus) * hold, 0.01) + rng.uniform(-0.001, 0.001, int(round(len(taus) * hold / 0.01)))
    t = np.sort(np.clip(t, 0.0, len(taus) * hold - 1e-6))
    v = np.empty_like(t); tau_cmd = np.empty_like(t)
    w_start = w0
    for k, tau in enumerate(taus):
        sel = (t >= k * hold) & (t < (k + 1) * hold)
        w_ss = max((tau - tc) / bv, 0.0)
        tt = t[sel] - k * hold
        v[sel] = w_ss + (w_start - w_ss) * np.exp(-tt * bv / J)
        tau_cmd[sel] = tau
        w_start = w_ss + (w_start - w_ss) * np.exp(-hold * bv / J)
    v = np.round(v / q) * q
    mode = np.array(["torque"] * len(t))
    return t, mode, v, tau_cmd.copy(), tau_cmd


@pytest.mark.parametrize("J", [1.0e-3, 8.9e-4, 6.0e-4])
def test_recovers_inertia_without_friction_model(J):
    """마찰 계수를 모르는 채로 Δτ/Δω̇ 가 정답 J 를 4 % 안에서 되찾는다."""
    taus = [0.08, 0.05, 0.08, 0.05, 0.08, 0.05, 0.08]
    est = jwa.step_estimates(*synth(J, taus, 1.0, w0=30.0))
    Js = [e["J_cmd"] for e in est if "skip" not in e]
    assert len(Js) == 6
    assert np.mean(Js) == pytest.approx(J, rel=0.04)


def test_skips_steps_near_standstill_and_voltage_limit():
    """|ω| < 3 rad/s (정지 마찰·0 지남) 과 > 39 rad/s (전압 한계) 계단은 뺀다."""
    est = jwa.step_estimates(*synth(1e-3, [0.025, 0.05], 2.0, w0=0.0))       # 0.025 정상속도 1.7 rad/s
    assert "skip" in est[0] and "3.0" in est[0]["skip"]
    est = jwa.step_estimates(*synth(1e-3, [0.1, 0.05], 3.0, w0=40.0))        # 0.1 정상속도 43 rad/s
    assert "skip" in est[0] and "39" in est[0]["skip"]


def test_old_csv_without_tau_cmd(tmp_path):
    """τ_명령 열이 없는 옛 CSV 는 --taus·--hold 로 계단 순서를 되살린다."""
    p = tmp_path / "old.csv"
    p.write_text("t,mode,p,v,tau_reply,temp,error\n0.0,damping,0,0,0,30,0\n0.5,torque,0,1,0.1,30,0\n"
                 "1.0,torque,0,2,0.1,30,0\n1.6,torque,0,3,0.06,30,0\n")
    t, mode, v, tr, tc = jwa.read_csv(str(p), taus=[0.1, 0.06], hold=1.0)
    assert list(tc) == [0.0, 0.1, 0.1, 0.06]
    with pytest.raises(SystemExit):
        jwa.read_csv(str(p))
