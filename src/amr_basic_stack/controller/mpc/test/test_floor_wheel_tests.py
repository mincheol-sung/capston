"""tools/floor_wheel_tests.py — 바닥 시험 분석 (2026-11-24). 참값을 아는 합성 기록에서 값을 되찾는지 본다 (ROS·bag 없이 dict 로).

C1 등속 왕복: 전진·후진 구간을 하나씩 찾고, 네 바퀴 평균 주행 저항 − 띄운 마찰 = 넣은 구름 손실 (0.3 mN·m 안). 옆 보정 토크는 평균에 안 섞인다
C2 등속 구간에 가속이 남아 있으면 (m r²/4 + J_w)·ω̇ 를 빼야 맞는다 (안 빼면 0.8 mN·m 어긋남)
C3 출발 직후 속도가 넘쳤다 가라앉아도 넘친 곳이 아니라 오래 머문 속도를 등속으로 잡는다
S1 단계 토크, 헛돎 없음: 바닥 전달 토크 (3 %), 몸체 가속 (IMU 바이어스 제거), 바퀴 쪽 힘 = 몸체 쪽 힘 (5 %), 선 거리
S2 앞바퀴만 한계를 넘김: 앞 두 바퀴를 헛돎으로 잡고, 그 바퀴의 μ 가 넣은 값 (5 %), 뒷바퀴는 헛돎 아님
S3 단계 앞에 0 토크 명령이 깔려 있어도 단계 구간만 잡는다
S4 하중 이동: 무게중심 높이를 주면 앞바퀴 하중이 m·a·h/(2L) 만큼 줄고 네 바퀴 합 μ 는 그대로
"""
import importlib.util
import math
import os

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("floor_wheel_tests", os.path.join(HERE, "..", "tools", "floor_wheel_tests.py"))
fw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fw)

PV = fw.plant_values()
NAN4 = [math.nan] * 4


def fric(om):
    return PV["tc"] * np.tanh(om / 0.5) + PV["bv"] * om


def pack(rows_ws, rows_cmd, rows_od, rows_sl, rows_imu):
    return {"ws": np.array(rows_ws), "cmd": np.array(rows_cmd), "odom": np.array(rows_od), "slam": np.array(rows_sl),
            "imu": np.array(rows_imu), "status": np.zeros((0, 1)), "cap": np.zeros((0, 1)), "args": "", "path": "synthetic"}


def synth_cv(V=0.4, roll=0.004, lat=0.008, drift=0.0, overshoot=0.0, noise=0.25, seed=0):
    """왕복 직선: 1 s 정지 → 가속 1.5 → 등속 1.6 s → 감속 → 1 s 쉼 → 같은 꼴로 후진. 모터 토크 = 마찰 + 구름 + (m r²/4 + J)·ω̇ + 옆 보정."""
    rng = np.random.default_rng(seed)
    r, dt, acc = PV["r"], 0.001, 1.5
    eq = PV["m"] * r ** 2 / 4.0 + PV["J"]
    ta, tc_ = V / acc, 1.6
    prof = []
    for sgn in (1.0, -1.0):
        prof += [(1.0, 0.0, 0.0), (ta, sgn * acc, 0.0), (tc_, sgn * drift * r, sgn * overshoot), (ta, -sgn * acc, 0.0)]
    prof.append((1.0, 0.0, 0.0))
    t, v, x = 0.0, 0.0, 0.0
    ws, cmd, od, sl, imu = [], [], [], [], []
    k = 0
    for dur, a, ov in prof:
        n = int(round(dur / dt))
        if a == 0.0 and abs(v) < 1e-9:
            v = 0.0
        v_start = v
        for i in range(n):
            a_i = a
            v_ov = ov * V * math.exp(-(i * dt) / 0.15) if ov else 0.0          # 등속 초반 넘침 (0.15 s 로 가라앉음)
            v_now = v + v_ov
            om = np.full(4, v_now / r)
            wdot = a_i / r - (v_ov / 0.15) / r if ov else a_i / r
            tau = fric(om) + roll * np.sign(om) + eq * wdot + (lat * fw.PAT_LAT if abs(v_now) > 0.01 else 0.0)
            if k % 10 == 0:
                ws.append([t, *(np.full(4, x / r)), *(om + rng.normal(0, noise, 4)), *tau, 34, 34, 34, 34])
                cmd.append([t, 0.0, *(tau + rng.normal(0, 0.0015, 4))])          # 제어기 명령의 잔떨림 (12 비트 격자 2.44 mN·m 를 고르게 밟는다)
                od.append([t, x, 0.0, 0.0, v_now, 0.0, 0.0])
                if k % 100 == 0:
                    sl.append([t, x, 0.0])
            if k % 3 == 0:
                imu.append([t, a_i, 0.0, 0.0])
            v += a_i * dt
            x += v_now * dt
            t += dt
            k += 1
        if a != 0.0 and abs(v) < abs(a) * dt:
            v = 0.0
        del v_start
    return pack(ws, cmd, od, sl, imu)


def synth_step(tau=0.12, mu=None, imu_bias=0.3, decel=3.0, pre_zero_cmd=False, seed=0):
    """정지 2 s → 네 바퀴 같은 모터 토크 → 몸체 0.3 m 에서 끊음 (DAMPING) → 일정 감속으로 섬 → 2 s 정지.
    mu = 바퀴별 마찰계수 (None 이면 한계 없음). 한계 r·μ·N/√2 를 넘는 바퀴는 헛돈다 (그 바퀴의 바닥 전달 토크 = 한계)."""
    rng = np.random.default_rng(seed)
    r, J, m, dt = PV["r"], PV["J"], PV["m"], 0.0005
    N = PV["share"] * m * PV["g"]
    cap = r * np.asarray(mu, float) * N / math.sqrt(2.0) if mu is not None else np.full(4, np.inf)
    tq = fw.quantize(tau)
    t, v, x = 0.0, 0.0, 0.0
    om, th = np.zeros(4), np.zeros(4)
    ws, cmd, od, sl, imu = [], [], [], [], []
    truth = {"tg": [], "a": [], "spin": np.zeros(4, bool)}
    phase, k, t_cut, t_still = "pre", 0, None, None
    while True:
        if phase == "pre" and t >= 2.0:
            phase = "step"
        a, tau_i, mode = 0.0, np.zeros(4), 1.0
        if phase == "step":
            mode, tau_i = 0.0, np.full(4, tq)
            f = fric(np.maximum(om, v / r))
            a_ns = float(np.sum(tq - fric(np.full(4, v / r)))) / (m * r + 4.0 * J / r)
            tg_ns = tq - fric(np.full(4, v / r)) - J * a_ns / r
            slip = (tg_ns > cap) | (om > v / r + 1e-6)
            n_g = int((~slip).sum())
            a = float(np.sum(cap[slip]) + np.sum((tq - fric(np.full(4, v / r)))[~slip])) / (m * r + n_g * J / r)
            tg = np.where(slip, cap, tq - fric(np.full(4, v / r)) - J * a / r)
            wdot = np.where(slip, (tq - f - np.where(slip, cap, 0.0)) / J, a / r)
            truth["tg"].append(tg)
            truth["a"].append(a)
            truth["spin"] |= slip
            if x >= 0.3:
                phase, t_cut = "brake", t
                truth["v_cut"] = v
        if phase == "brake":
            a = -decel if v > 0 else 0.0
            wdot = (v / r - om) / 0.02 + a / r                                   # 바퀴는 0.02 s 로 몸체 속도에 붙는다
            if v <= 0.0:
                v, a, phase, t_still = 0.0, 0.0, "post", t
        if phase in ("pre", "post"):
            wdot = -om / 0.02
            if phase == "post" and t >= t_still + 2.0:
                break
        if k % 20 == 0:                                                          # 100 Hz
            ws.append([t, *th, *(om + rng.normal(0, 0.05, 4)), *tau_i, 33, 33, 33, 33])
            if mode == 0.0:
                cmd.append([t, 0.0, *tau_i])
            elif pre_zero_cmd and phase == "pre":
                cmd.append([t, 0.0, 0.0, 0.0, 0.0, 0.0])
            else:
                cmd.append([t, 1.0, *NAN4])
            od.append([t, x, 0.0, 0.0, v, 0.0, 0.0])
            if k % 200 == 0:
                sl.append([t, x, 0.0])
        if k % 5 == 0:                                                           # 400 Hz
            imu.append([t, a + imu_bias + rng.normal(0, 0.3), 0.0, 0.0])
        om = np.maximum(om + wdot * dt, 0.0)
        th = th + om * dt
        v = max(v + a * dt, 0.0)
        x += v * dt
        t += dt
        k += 1
    truth["tg"], truth["a"] = np.mean(truth["tg"], axis=0), float(np.mean(truth["a"]))
    truth["stop"] = truth["v_cut"] ** 2 / (2.0 * decel)
    return pack(ws, cmd, od, sl, imu), truth


# ── 등속 ─────────────────────────────────────────────────────────
@pytest.mark.parametrize("V", [0.2, 0.4, 0.8])
def test_cv_recovers_rolling_loss_both_directions(V):
    segs = fw.analyze_cv(synth_cv(V=V, roll=0.004, lat=0.008), PV)
    assert sorted(a["dir"] for a in segs) == [-1.0, 1.0]
    for a in segs:
        lift = (PV["tc"] + PV["bv"] * a["omega"]).mean()
        assert a["omega"].mean() == pytest.approx(V / PV["r"], rel=0.02)
        assert (a["tau_res"].mean() - lift) * 1e3 == pytest.approx(4.0, abs=0.3)          # 넣은 구름 손실 4 mN·m
        assert a["lat"] * 1e3 == pytest.approx(8.0, abs=0.3)                              # 옆 보정은 따로 잡힌다
        assert abs(a["yaw"]) < 3e-4 and abs(a["null"]) < 3e-4                             # 띄운 마찰의 바퀴별 차이는 미리 빠진다


def test_cv_residual_acceleration_is_corrected():
    eq = PV["m"] * PV["r"] ** 2 / 4.0 + PV["J"]
    segs = fw.analyze_cv(synth_cv(V=0.4, roll=0.004, drift=0.3), PV)
    assert len(segs) == 2
    for a in segs:
        lift = (PV["tc"] + PV["bv"] * a["omega"]).mean()
        assert a["wdot"] == pytest.approx(0.3, abs=0.05)
        assert a["accel_corr"] == pytest.approx(eq * 0.3, rel=0.2)                        # 0.81 mN·m
        assert (a["tau_res"].mean() - lift) * 1e3 == pytest.approx(4.0, abs=0.3)
        assert (a["tau_raw"].mean() - lift) * 1e3 > 4.5                                   # 안 고치면 크게 나온다


def test_cv_uses_dwell_speed_not_overshoot_peak():
    segs = fw.analyze_cv(synth_cv(V=0.2, roll=0.004, overshoot=0.25), PV)
    assert len(segs) == 2
    for a in segs:
        assert a["omega"].mean() == pytest.approx(5.0, rel=0.03)                          # 6.25 (넘친 값) 가 아니라 5.0
        lift = (PV["tc"] + PV["bv"] * a["omega"]).mean()
        assert (a["tau_res"].mean() - lift) * 1e3 == pytest.approx(4.0, abs=0.4)


def test_cv_report_has_mean_rows():
    res = [(f"v{V}", fw.analyze_cv(synth_cv(V=V), PV)) for V in (0.2, 0.4, 0.6)]
    txt = fw.report_cv(res, PV, lifted=(PV["tc"] * 1.05, PV["bv"] * 1.05))
    assert "네 바퀴 평균" in txt and "| 모두 | 6 |" in txt and "다른 띄운 시험 직선" in txt


# ── 단계 토크 ─────────────────────────────────────────────────────
def test_step_no_slip():
    run, tr = synth_step(tau=0.12)
    a = fw.analyze_step(run, PV)
    assert a is not None and not a["spin"].any()
    assert np.allclose(a["tau"], fw.quantize(0.12))
    assert np.allclose(a["tau_g"], tr["tg"], rtol=0.03)
    assert a["a_imu"] == pytest.approx(tr["a"], abs=0.1)                                  # 바이어스 0.3 m/s² 가 빠졌다
    assert a["a_est"] == pytest.approx(tr["a"], abs=0.1)
    assert a["F_wheel"] / a["F_body"] == pytest.approx(1.0, abs=0.05)
    assert a["dist_ratio"] == pytest.approx(1.0, abs=0.03)
    assert a["body_dist"] == pytest.approx(0.3, abs=0.02)
    assert a["stop_dist"] == pytest.approx(tr["stop"], abs=0.02)
    assert a["total_dist"] == pytest.approx(0.3 + tr["stop"], abs=0.03)
    assert a["total_dist_slam"] == pytest.approx(a["total_dist"], abs=0.01)
    assert a["a_noslip"] == pytest.approx(tr["a"], rel=0.03) and a["mu_body"] == pytest.approx(math.sqrt(2.0) * tr["a"] / PV["g"], abs=0.02)
    assert a["mu_total"] == pytest.approx(math.sqrt(2.0) * tr["tg"].sum() / (PV["r"] * PV["m"] * PV["g"]), rel=0.03)


def test_step_front_wheels_spin():
    run, tr = synth_step(tau=0.28, mu=[0.45, 0.45, 0.7, 0.7])
    assert list(tr["spin"]) == [True, True, False, False]
    a = fw.analyze_step(run, PV, com_height=0.0)
    assert list(a["spin"]) == [True, True, False, False]
    assert np.allclose(a["mu_static"][:2], 0.45, rtol=0.05)                               # 헛돈 바퀴의 μ = 그 바퀴의 한계
    assert (a["mu_static"][2:] < 0.7).all() and (a["mu_static"][2:] > 0.45).all()         # 뒷바퀴는 하한 (한계 0.7 아래)
    assert np.allclose(a["tau_g"], tr["tg"], rtol=0.05)
    assert a["dist_ratio"] > 1.1                                                          # 굴린 거리가 몸체 거리보다 길다
    assert a["a_imu"] == pytest.approx(tr["a"], abs=0.15)


def test_step_window_ignores_idle_zero_commands():
    run, _ = synth_step(tau=0.16, pre_zero_cmd=True)
    t0, t1 = fw.step_window(run)
    assert t0 == pytest.approx(2.0, abs=0.02) and 0.3 < t1 - t0 < 1.0
    assert fw.torque_window(run)[0] < 0.1                                                 # 토크 모드 전체 구간은 0 토크부터다


def test_step_load_transfer_changes_wheel_mu_not_total():
    run, tr = synth_step(tau=0.20)
    a0 = fw.analyze_step(run, PV, com_height=0.0)
    a1 = fw.analyze_step(run, PV, com_height=0.105)
    dN = PV["m"] * a1["a_body"] * 0.105 / (2.0 * PV["L"])
    assert np.allclose(a0["N_shift"], a0["N_static"])
    assert np.allclose(a1["N_shift"], a1["N_static"] - np.array([1, 1, -1, -1]) * dN)
    assert a1["N_shift"].sum() == pytest.approx(PV["m"] * PV["g"], rel=1e-9)
    assert (a1["mu_shift"][:2] > a1["mu_static"][:2]).all() and (a1["mu_shift"][2:] < a1["mu_static"][2:]).all()
    assert a1["mu_total"] == pytest.approx(a0["mu_total"], rel=1e-12)


def test_step_report_and_quick_run():
    runs = [(f"t{int(t * 100):03d}", fw.analyze_step(synth_step(tau=t)[0], PV)) for t in (0.08, 0.16)]
    txt = fw.report_step(runs + [("빈 판", None)], PV)
    assert "헛돈 바퀴" in txt and "단계 구간을 못 찾음" in txt and "실측이 아니다" in txt
    assert "토크 모드 구간" in fw.quick(synth_cv(V=0.4), PV)


# ── 제자리 회전 (2026-11-25, rot) ─────────────────────────────────────────────
# R1 참값 I_eff·M_c·B 로 만든 회전 단계 네 번 (반시계 0.06 · 0.10, 시계 0.08 · 0.07) 에서 세 값을 되찾는다 (표본 전부 3 %, 회 평균 10 %)
# R2 바퀴 각가속도 몫을 바퀴 쪽에서 뺀 검산이 I_z = I_eff − J_w·Σc² 를 낸다
# R3 정지 마찰을 못 넘은 판은 출발 nan, 맞춤에 표본을 안 보탠다
def _rot_run(levels, I=0.12, Mc=0.46, B=0.07, stuck=(), bias=0.01):
    rg = fw.rot_geometry()
    c = rg["c"]
    dt, t, w, ws, imu, cmd = 0.001, 0.0, 0.0, [], [], []
    pos = np.zeros(4)
    sched, tt = [], 1.0
    for k, tau in enumerate(levels):
        sched.append((tt, tt + 0.8, fw.quantize(abs(tau)) * np.sign(tau) * np.sign(c), k in stuck))
        tt += 3.0
    while t < tt:
        on = [s for s in sched if s[0] <= t < s[1]]
        if on:
            tq, stk = on[0][2], on[0][3]
            M = float(c @ (tq - fric(c * w))) - Mc * math.tanh(w / 0.05) - B * w
            w = 0.0 if stk else w + M / I * dt
        else:
            tq = None
            w = 0.0 if abs(w) < 0.05 else w - math.copysign(20.0, w) * dt       # DAMPING 으로 선다
        om = c * w
        pos = pos + om * dt
        n = int(round(t / dt))
        if n % 5 == 0:
            imu.append([t, 0.0, 0.0, w + bias])
        if n % 10 == 0:
            ws.append([t, *pos, *om, *(tq if tq is not None else np.zeros(4)), *[30.0] * 4])
            cmd.append([t, 0.0, *tq] if tq is not None else [t, 2.0, *NAN4])
        t += dt
    E = np.zeros((0, 3))
    return {"ws": np.array(ws), "imu": np.array(imu), "cmd": np.array(cmd), "odom": np.zeros((0, 7)), "slam": E}, rg


def test_rot_recovers_inertia_and_resistance():
    run, rg = _rot_run([0.06, -0.08, 0.10, -0.07])                       # 회마다 평균 맞춤은 회 4 개 이상 (계수 3 개)
    rows = fw.analyze_rots(run, PV, rg)
    assert len(rows) == 4 and [r["sg"] for r in rows] == [1.0, -1.0, 1.0, -1.0]
    assert fw.fit_rot_runs(rows[:3]) is None
    assert abs(rows[0]["bias"] - 0.01) < 1e-6
    x, se, rms, n = fw.fit_rot(rows, "M")
    assert x[0] == pytest.approx(0.12, rel=0.03) and x[1] == pytest.approx(0.46, rel=0.03) and x[2] == pytest.approx(0.07, rel=0.1)
    xr, *_ = fw.fit_rot_runs(rows)
    assert xr[0] == pytest.approx(0.12, rel=0.1) and xr[1] == pytest.approx(0.46, rel=0.1)
    txt = fw.report_rot([("synthetic", rows)], PV, rg)
    assert "맞춘 값" in txt and "검산" in txt


def test_rot_wheel_side_check_gives_body_inertia():
    rg = fw.rot_geometry()
    run, _ = _rot_run([0.06, -0.08, 0.10], I=0.06 + rg["I_w"])
    x, *_ = fw.fit_rot(fw.analyze_rots(run, PV, rg), "M_wh")
    assert x[0] == pytest.approx(0.06, rel=0.05)


def test_rot_stuck_run_has_no_samples():
    run, rg = _rot_run([0.06, -0.08, 0.10], stuck=(0,))
    rows = fw.analyze_rots(run, PV, rg)
    assert math.isnan(rows[0]["breakaway"]) and not rows[0]["moved"] and len(rows[0]["samples"]["M"]) == 0
    assert math.isnan(rows[0]["alpha_fit"]) and math.isfinite(rows[1]["breakaway"])
    x, *_ = fw.fit_rot(rows, "M")
    assert x[0] == pytest.approx(0.12, rel=0.05)


# ── 옆걸음 (2026-11-25, lat) ─────────────────────────────────────────────────
# L1 참값 옆 저항 F_c 로 만든 왼쪽·오른쪽 단계 네 번에서 회마다 저항 (m_eff 고정) 과 회들끼리 맞춤 (m·F_c) 이 참값 (3 %)
# L2 옆걸음 중 몸체가 돌아도 (ψ̈ 일정) IMU 위치 몫을 빼서 같은 값 — 안 빼면 어긋난다
def _lat_run(levels, Fc=4.6, alpha=0.0, tilt=(0.05, -0.03)):
    lg = fw.lat_geometry()
    c, m = lg["c"], lg["m_eff"]
    rx, ry = lg["r_imu"]
    dt, t, v, y, w = 0.001, 0.0, 0.0, 0.0, 0.0
    ws, imu, cmd, od = [], [], [], []
    pos = np.zeros(4)
    sched, tt = [], 1.0
    for tau in levels:
        sched.append((tt, tt + 1.0, fw.quantize(abs(tau)) * np.sign(tau) * np.sign(c)))
        tt += 3.0
    while t < tt:
        on = [s for s in sched if s[0] <= t < s[1]]
        al = 0.0 if on or abs(w) < 0.02 else -math.copysign(10.0, w)
        if on:
            tq = on[0][2]
            F = float(c @ (tq - fric(c * v))) - Fc * math.tanh(v / 0.02)
            a = F / m
            al = alpha
        else:
            tq = None
            a = 0.0 if abs(v) < 0.02 else -math.copysign(3.0, v)
            if abs(v) < 0.02:
                v = 0.0
        v += a * dt
        w = w + al * dt if on else (0.0 if abs(w) < 0.02 else w - math.copysign(10.0, w) * dt)   # 끊은 뒤 회전도 감속해 선다
        y += v * dt
        om = c * v
        pos = pos + om * dt
        n = int(round(t / dt))
        if n % 5 == 0:
            imu.append([t, tilt[0] - al * ry - w ** 2 * rx, tilt[1] + a + al * rx - w ** 2 * ry, w])
        if n % 10 == 0:
            ws.append([t, *pos, *om, *(tq if tq is not None else np.zeros(4)), *[30.0] * 4])
            cmd.append([t, 0.0, *tq] if tq is not None else [t, 2.0, *NAN4])
            od.append([t, 0.0, y, 0.0, 0.0, v, w])
        t += dt
    return {"ws": np.array(ws), "imu": np.array(imu), "cmd": np.array(cmd), "odom": np.array(od), "slam": np.zeros((0, 3))}, lg


def test_lat_recovers_resistance():
    run, lg = _lat_run([0.10, -0.10, 0.12, -0.12])
    rows = fw.analyze_lats(run, PV, lg)
    assert [r["sg"] for r in rows] == [1.0, -1.0, 1.0, -1.0]
    for r in rows:
        assert r["F_res"] == pytest.approx(4.6, rel=0.03) and r["F_res_est"] == pytest.approx(4.6, rel=0.05)
    x, *_ = fw.fit_lat_runs(rows)
    assert x[0] == pytest.approx(lg["m_eff"], rel=0.03) and x[1] == pytest.approx(4.6, rel=0.03)
    assert "롤러 구름 저항 가설" in fw.report_lat([("synthetic", rows)], PV, lg)


def test_lat_imu_lever_arm_removed_when_body_rotates():
    run, lg = _lat_run([0.10, -0.12], alpha=3.0)
    rows = fw.analyze_lats(run, PV, lg)
    for r in rows:
        assert r["F_res"] == pytest.approx(4.6, rel=0.03)
    lg_bad = dict(lg, r_imu=np.zeros(2))                                     # IMU 를 중심에 있다고 보면 α·r_x = 0.33 m/s² 만큼 틀린다
    bad = fw.analyze_lats(run, PV, lg_bad)
    assert abs(bad[0]["F_res"] - 4.6) > 1.0
