#!/usr/bin/python3
"""analyze.py 결과의 세 가지 확인.
 1. contour error 를 추정기 포즈 (MPC 가 쓰는 /state_estimator/odom) 로도 잰다 → SLAM 과 다르면 측위 쪽 어긋남.
 2. λ 를 기준 속력 v_ref > 0.3 m/s 인 구간만으로 다시 잰다 — 꼭짓점 저속에서는 분모 max(|ω|, |ω_nom|, ω_ε 2.5 rad/s) 가 작아 v̂ 잡음이 λ 를 키운다.
 3. C 직선 가속 (기준 가속 > 1.3, |ω_r| < 0.05) 의 실제 가속 a_meas (SLAM 속력 차분, 구간 처음·끝) vs MPC 토크로 모델이 내는 가속
    a_model = Σ_i τ_i sgn(U_s,i) / (m_eff r) (τ = MPC 출력, 마찰 보상 전). 비율 < 1 이면 모델 밖 손실 (바닥 구름 저항 등) 또는 질량 오차.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/check.py"""
import importlib.util, json, math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("an", os.path.join(HERE, "analyze.py"))
import matplotlib
matplotlib.use("Agg")
an = importlib.util.module_from_spec(spec)
import sys
_argv = sys.argv; sys.argv = [sys.argv[0]]
spec.loader.exec_module(an)                       # analyze.py 를 한 번 돌려 R·summ 을 얻는다 (그림도 다시 쓴다)
sys.argv = _argv
cs, w, WS = an.cs, an.w, an.WS

res = {}
for b, name, _, _, tb in an.BAGS:
    path = f"{WS}/bags/{b}"; r = an.R[b]
    S, E, P, A = w.load(path) if tb is not None else (None, None, None, None)
    D = cs.load(path)
    # 1. 추정기 포즈 contour — 추정기 odom 은 로봇 중심
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/state_estimator/odom"]))
    EO = []
    while rd.has_next():
        _, data, _ = rd.read_next(); m = deserialize_message(data, get_message(types["/state_estimator/odom"]))
        EO.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y])
    EO = np.array(EO)
    # analyze 의 로컬 좌표: t (SLAM 시각, T0 기준) 와 lx, ly. 추정기 위치를 같은 변환으로 — 변환을 다시 만든다
    Pp = np.array(D["pose"]); xc, yc = cs.imu_to_center(Pp[:, 1], Pp[:, 2], Pp[:, 3], an.off)
    rt = np.array([a for a, _ in D["ref"]])
    ok = np.searchsorted(rt, Pp[:, 0], side="right") - 1 >= 0
    tP = Pp[ok, 0]; T0 = tP[0] + 0.0
    # 같은 T0: analyze 의 t 는 T0 를 뺀 값 → 절대 T0 = tP 중 r['t'][0] 에 대응
    T0 = tP[0] - r["t"][0]
    # 로컬 변환 계수: SLAM 절대 (xc, yc) ↔ (lx, ly) 를 최소제곱으로 (회전 + 이동) 맞춘다
    X = np.c_[xc[ok], yc[ok]]; Y = np.c_[r["lx"], r["ly"]]
    Xm, Ym = X.mean(0), Y.mean(0); U_, _, Vt = np.linalg.svd((X - Xm).T @ (Y - Ym)); Rm = (U_ @ Vt).T; tv = Ym - Rm @ Xm
    el = (Rm @ np.c_[np.interp(tP, EO[:, 0], EO[:, 1]), np.interp(tP, EO[:, 0], EO[:, 2])].T).T + tv
    ce = an.contour(el[:, 0], el[:, 1]); mv = r["mv"]
    dif = el - Y
    res[b] = dict(name=name, contour_slam=1e3 * float(np.sqrt(np.mean(r["contour"][mv] ** 2))), contour_est=1e3 * float(np.sqrt(np.mean(ce[mv] ** 2))),
                  est_minus_slam_rms=1e3 * float(np.sqrt(np.mean(np.sum(dif[mv] ** 2, 1)))), est_minus_slam_mean=[1e3 * float(x) for x in dif[mv].mean(0)])
    # 2. λ, v_ref > 0.3
    L = an.read_topic(path, "/slip_logger/slip"); L = L[np.all(np.isfinite(L[:, 1:5]), axis=1)]
    tl = L[:, 0] - T0; vr = np.interp(tl, r["td"], r["vref"]); g = (tl >= 0) & (tl <= r["td"][-1]) & (vr > 0.3)
    lam = L[g, 1:5]; al = np.abs(lam); mx = al.max(1); big = mx > 0.5; sgn = np.sign(lam[np.arange(len(lam)), al.argmax(1)])
    res[b].update(lam_fast=dict(share_time=100 * float(g.sum() / ((tl >= 0) & (tl <= r["td"][-1])).sum()), big=100 * float(big.mean()),
                                lmax=float(mx.max()), lrms=float(np.sqrt(np.mean(lam ** 2))), spin=100 * float((sgn[big] > 0).mean()) if big.any() else float("nan")))
    # 3. C 직선 가속: a_meas vs a_model
    if tb is not None:
        tS = S[:, 0]; tau = S[:, 4:8]; us = S[:, 22:26]; hold = S[:, 20] > 0.5
        runm = cs.armed_mask(tS, A) & ~hold
        a_true, om = [], []
        for x in tS:
            R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]
            a_true.append((np.hypot(*R_.at(x + 0.15)[1][:2]) - np.hypot(*R_.at(x + 0.05)[1][:2])) / 0.1); om.append(abs(R_.at(x)[1][2]))
        sel = runm & (np.array(a_true) > 1.3) & (np.array(om) < 0.05)
        idx = np.flatnonzero(sel); segs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        rows = []
        for sg in segs:
            if len(sg) < 20:
                continue
            ta, tb_ = tS[sg[0]] - T0, tS[sg[-1]] - T0
            va, vb = np.interp([ta, tb_], r["ts"], r["speed"])
            a_meas = (vb - va) / (tb_ - ta)
            tf = (tau[sg] * np.sign(us[sg])).sum(1).mean()
            rows.append(dict(t=[float(ta), float(tb_)], a_meas=float(a_meas), a_model=float(tf / (an.m_eff * an.r_w)), a_ref=float(np.mean(np.array(a_true)[sg])),
                             tau_mean=float(tf / 4), sat=100 * float(((np.abs(tau[sg]) / tb).max(1) >= 0.98).mean())))
        res[b]["accel"] = rows

print("| 판 | contour SLAM / 추정기 [mm] | 추정기 − SLAM RMS (평균 x, y) [mm] |")
for b, s in res.items():
    print(f"| {s['name']} | {s['contour_slam']:.0f} / {s['contour_est']:.0f} | {s['est_minus_slam_rms']:.0f} ({s['est_minus_slam_mean'][0]:+.0f}, {s['est_minus_slam_mean'][1]:+.0f}) |")
print("| 판 | v_ref>0.3 시간 몫 % | 큰 슬립 % | λ 최대 | λ RMS | 큰 슬립 중 헛돎 % |")
for b, s in res.items():
    l = s["lam_fast"]; print(f"| {s['name']} | {l['share_time']:.0f} | {l['big']:.1f} | {l['lmax']:.2f} | {l['lrms']:.3f} | {l['spin']:.0f} |")
for b, s in res.items():
    if "accel" in s:
        print(f"{s['name']}: 직선 가속 구간")
        for a in s["accel"]:
            print(f"   t {a['t'][0]:.2f}~{a['t'][1]:.2f} s  a_ref {a['a_ref']:.2f}  a_model (MPC τ) {a['a_model']:.2f}  a_meas (SLAM) {a['a_meas']:.2f}  "
                  f"비율 {a['a_meas']/a['a_model']:.2f}  τ 평균 {a['tau_mean']:.4f}  포화 {a['sat']:.0f} %")
json.dump(res, open(os.path.join(HERE, "check.json"), "w"), ensure_ascii=False, indent=1)
