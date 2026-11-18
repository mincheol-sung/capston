"""노면별 μ̂ (amr_mpc/core/surfaces.py, tools/mu_surface.py) — 2026-11-17.

가짜 램프 기록(알고 있는 μ 로 만든 슬립)에서 μ̂ 를 되찾는지, 대표값 규칙, 표 읽기·쓰기, launch 조회를 본다.
"""
import csv
import math

import numpy as np
import pytest

from amr_mpc.core import surfaces as S
from amr_mpc.torque.model import TorquePlant

G = 9.81


def fake_ramp_csv(path, mu, direction="fwd", rate=1.0, dt=0.01, T=1.5, eff=1.0, noise=0.0, seed=0):
    """torque_ramp_node 꼴 CSV. 몸체 가속 = min(토크가 내는 가속, μ g/√2) (45° 롤러 모델).
    미끄러지기 전에는 바퀴 = 몸체/eff (옆걸음 롤러 손실 흉내), 미끄러지면 바퀴는 토크대로 더 빨리 돈다."""
    p = TorquePlant.from_yaml()
    rng = np.random.default_rng(seed)
    a_max = mu * G / math.sqrt(2.0)
    v_b = v_w = 0.0
    rows = []
    for k in range(int(T / dt)):
        t = k * dt
        tau = rate * t
        a_cmd = 4.0 * tau / (p.r * p.mass)
        a_b = min(a_cmd, a_max)
        v_b += a_b * dt
        v_w = v_b / eff if a_cmd <= a_max else v_w + a_cmd * dt
        rows.append({"t": f"{t:.4f}", "phase": "ramp", "tau_g": f"{tau:.5f}",
                     "v_wheel_dir": f"{v_w:.5f}", "v_est_dir": f"{v_b + noise * rng.standard_normal():.5f}"})
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return path


@pytest.mark.parametrize("mu", [0.3, 0.5, 0.8])
def test_ramp_node_recovers_mu(tmp_path, mu):
    p = TorquePlant.from_yaml()
    f = fake_ramp_csv(tmp_path / f"ramp_fwd_{int(mu*100)}_run1.csv", mu, rate=1.0, T=2.0)
    r = S.analyze_ramp_node_csv(str(f), p)
    assert r["dir"] == "fwd"
    assert r["result"] == "slip"
    assert abs(r["mu"] - mu) / mu < 0.03
    # 슬립 시작 토크는 실제 시작보다 늦게 잡힌다 (문턱 0.12 + 0.15·|v| 를 50 ms 넘어야 해서). μ 가 낮으면 바퀴–몸체 차가
    # 천천히 벌어져 더 늦다 (μ 0.3 에서 약 3 배). 그래서 μ̂ 는 토크가 아니라 미끄러지는 동안의 몸체 가속으로 낸다 —
    # 슬립 시작 토크는 참고값이고, 여기서는 실제보다 이르지 않은지만 본다
    tau_true = mu * G / math.sqrt(2.0) * p.r * p.mass / 4.0
    assert r["tau_slip"] >= tau_true


def test_ramp_node_no_slip_gives_lower_bound(tmp_path):
    p = TorquePlant.from_yaml()
    f = fake_ramp_csv(tmp_path / "ramp_fwd_hi_run1.csv", mu=3.0, rate=0.3, T=1.0)
    r = S.analyze_ramp_node_csv(str(f), p)
    assert r["result"] == "no_slip"
    assert r["mu"] < 3.0                    # 하한일 뿐


def test_ramp_node_left_with_roller_loss_is_not_fooled(tmp_path):
    """옆걸음: 미끄러지지 않아도 바퀴가 몸체보다 빠르다(효율 0.6). 슬립 전엔 차이가 일정 비율이라 판정이 안 걸리거나,
    걸려도 '더 벌어짐' 확인에서 μ 가 크게 틀리지 않아야 한다."""
    p = TorquePlant.from_yaml()
    f = fake_ramp_csv(tmp_path / "ramp_left_x_run1.csv", mu=0.4, direction="left", rate=0.6, T=2.5, eff=0.6)
    r = S.analyze_ramp_node_csv(str(f), p)
    assert r["dir"] == "left"
    if r["result"] == "slip":
        assert r["mu"] < 0.4 * 1.1            # 거짓 판정이 μ 를 부풀리지는 않는다 (상한 쪽 위험이 아님)


def test_direction_from_filename_required(tmp_path):
    f = fake_ramp_csv(tmp_path / "noname.csv", 0.4)
    with pytest.raises(ValueError):
        S.analyze_ramp_node_csv(str(f), TorquePlant.from_yaml())


def fake_step_csv(path, mu, v_cap=1.6, spin_up=0.15, dt=0.01, T=1.5, noise=0.0, seed=0):
    """torque_step_node 꼴 CSV: 바퀴는 spin_up 초에 속도 한계 v_cap 까지 헛돌고, 몸체는 μ g/√2 로 따라붙는다 (11-17 실차 모양)."""
    rng = np.random.default_rng(seed)
    a = mu * G / math.sqrt(2.0)
    rows, vb = [], 0.0
    for k in range(int(T / dt)):
        t = k * dt
        vw = min(v_cap, v_cap * t / spin_up)
        vb = min(vw, vb + a * dt)
        rows.append({"t": f"{t:.4f}", "phase": "step", "tau_g": "0.30000", "v_wheel_dir": f"{vw:.5f}",
                     "v_est_dir": f"{vb + noise * rng.standard_normal():.5f}"})
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return path


@pytest.mark.parametrize("mu", [0.3, 0.5])
def test_sliding_step_recovers_mu(tmp_path, mu):
    p = TorquePlant.from_yaml()
    f = fake_step_csv(tmp_path / f"step_fwd_0.3Nm_{int(mu*100)}_run1.csv", mu, noise=0.005)
    r = S.analyze_sliding_csv(str(f), p)
    assert r["result"] == "slip" and r["dir"] == "fwd"
    assert abs(r["mu"] - mu) / mu < 0.05
    assert r["r2"] > 0.95 and r["n"] >= 8


def test_sliding_no_slide_is_unconfirmed(tmp_path):
    p = TorquePlant.from_yaml()
    f = fake_step_csv(tmp_path / "step_left_0.1Nm_x_run1.csv", mu=50.0)       # 몸체가 바퀴를 바로 따라간다
    r = S.analyze_sliding_csv(str(f), p)
    assert r["dir"] == "left" and r["result"] == "unconfirmed" and r["n"] == 0


def _runs(d, mus, result="slip"):
    return [{"source": "x", "dir": d, "result": result, "mu": m} for m in mus]


def test_summarize_rule_min_of_main_medians():
    runs = _runs("fwd", [0.50, 0.52, 0.48, 0.51, 0.49]) + _runs("back", [0.46, 0.47, 0.45, 0.48, 0.44]) \
        + _runs("left", [0.30, 0.32])
    stats, rec = S.summarize(runs)
    assert rec["status"] == "확정"
    assert rec["mu_hat"] == pytest.approx(0.46)
    assert stats["fwd"]["n"] == 5 and stats["left"]["median"] == pytest.approx(0.31)
    assert any(w.startswith("left") for w in rec["warnings"])     # 옆이 확실히 낮다 → 경고


def test_summarize_insufficient_and_lower_bound_warning():
    runs = _runs("fwd", [0.5, 0.5, 0.5]) + _runs("back", [0.5] * 5) + _runs("fwd", [0.9], result="no_slip") \
        + _runs("back", [float("nan")], result="unconfirmed")
    stats, rec = S.summarize(runs, min_runs=5)
    assert rec["status"] == "부족" and "fwd 3 판" in rec["reason"]
    assert stats["back"]["unconfirmed"] == 1
    assert any("하한" in w for w in rec["warnings"])


def test_table_roundtrip_and_lookup(tmp_path, monkeypatch):
    path = tmp_path / "surfaces.yaml"
    stats, rec = S.summarize(_runs("fwd", [0.5] * 5) + _runs("back", [0.45] * 5))
    S.save({"lab_floor": S.make_entry(stats, rec, ["a.csv"], "mu_ramp_test", date="2026-11-17"),
            "rug": {"mu_hat": None, "status": "미측정"}}, path)
    monkeypatch.setenv("AMR_MPC_SURFACES_YAML", str(path))
    assert S.lookup("lab_floor") == pytest.approx(0.45)
    assert S.load()["lab_floor"]["measured"] == "2026-11-17"
    with pytest.raises(ValueError, match="미측정"):
        S.lookup("rug")
    with pytest.raises(ValueError, match="없다"):
        S.lookup("ice")


def test_wheel_scale_from_loads_and_motor_gain():
    rec = S.wheel_scale(loads=[1000, 1200, 1200, 1160])
    assert rec["scale"][0] == pytest.approx(1000 / 1140, abs=1e-3)
    assert rec["method"] == "하중"
    rec2 = S.wheel_scale(loads=[1140] * 4, motor_gain=[1.05, 1.0, 1.0, 1.0])     # FL 모터가 5 % 세다 → FL 만 낮춘다
    assert rec2["scale"][0] == pytest.approx(1 / 1.05, abs=1e-3) and rec2["scale"][1] == 1.0
    with pytest.raises(ValueError):
        S.wheel_scale()
    with pytest.raises(ValueError):
        S.wheel_scale(loads=[1, 2, 3])


def test_plane_loads_from_pair_sums():
    L = S.plane_loads(4560, front=2160, left=2380)            # 뒤·왼쪽이 무겁다
    assert sum(L) == pytest.approx(4560)
    assert L[0] + L[1] == pytest.approx(2160) and L[0] + L[2] == pytest.approx(2380)
    assert L[0] + L[3] == pytest.approx(L[1] + L[2])          # 대각 쏠림 0
    assert np.argmax(L) == 2                                  # RL 이 가장 무겁다
    assert S.plane_loads(4000, 2000, 2000) == [1000.0] * 4
    with pytest.raises(ValueError):
        S.plane_loads(4560, 4560, 2280)


def test_wheel_scale_from_onsets_only_lowers_and_warns_on_mismatch():
    on = [{"FL": {"t": 0.3, "tau_fb": 0.16}, "FR": {"t": 0.4, "tau_fb": 0.24}, "RL": None, "RR": {"t": 0.5, "tau_fb": 0.22}}] * 3
    rec = S.wheel_scale(onsets=on)
    assert rec["scale"][0] == pytest.approx(0.16 / 0.22, abs=1e-3)              # 기준 = 잡힌 바퀴 중앙 0.22
    assert rec["scale"][1] == 1.0 and rec["scale"][2] == 1.0                    # 높이지는 않는다 / 안 미끄러진 바퀴는 1
    rec2 = S.wheel_scale(loads=[1200, 1000, 1200, 1160], onsets=on)             # 하중으로는 FR 이 약한데 FL 이 미끄러짐
    assert any("FR" in w and "FL" in w for w in rec2["warnings"])


def test_wheel_slip_onsets_finds_the_light_wheel(tmp_path):
    """몸체는 무슬립으로 가는데 FL 만 바퀴가 앞서 돈다 → FL 만 잡힌다."""
    p = TorquePlant.from_yaml()
    rows, dt, v = [], 0.01, 0.0
    for k in range(80):
        t = k * dt
        v += 1.0 * dt
        om = np.full(4, v / p.r)
        if t > 0.3:
            om[0] += (t - 0.3) * 60.0                                           # FL 헛돎
        rows.append({"t": t, "vx_body": v, "vy_body": 0.0, "wz": 0.0,
                     **{f"omega_{w}": om[i] for i, w in enumerate(S.WHEELS)},
                     **{f"tau_fb_{w}": 0.5 * t for w in S.WHEELS}})
    f = tmp_path / "ramp_fwd_x_run1.csv"
    with open(f, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    o = S.wheel_slip_onsets(str(f), p)
    assert o["FL"] is not None and 0.3 < o["FL"]["t"] < 0.45
    assert o["FR"] is None and o["RL"] is None and o["RR"] is None


def test_plant_per_wheel_friction_limit():
    p = TorquePlant.from_yaml(mu_hat=0.4, wheel_limit_scale=[0.8, 1.0, 1.2, 1.0])
    tb = p.tau_bar_friction()
    assert tb[0] == pytest.approx(0.8 * p.tau_friction_limit()) and tb[2] == pytest.approx(1.2 * p.tau_friction_limit())
    assert np.allclose(TorquePlant.from_yaml().wheel_limit_scale, 1.0)          # 리포 기본값은 1 (예전과 같다)
    with pytest.raises(ValueError):
        TorquePlant.from_yaml(wheel_limit_scale=[0.0, 1, 1, 1])


def test_repo_table_loads_and_lab_floor_is_unmeasured():
    table = S.load()
    assert "lab_floor" in table
    with pytest.raises(ValueError, match="미측정"):
        S.lookup("lab_floor")
