#!/usr/bin/python3
"""바퀴–몸체 시간차가 slip_logger λ 에 얼마나 섞이나 (2026-11-22).

λ_i = (ω_i − ω_nom,i)·sgn(ω_nom,i) / max(|ω_i|, |ω_nom,i|, 2.5),  ω_nom = T⁻¹ [v_x, v_y, ω_z] (추정기, slip_logger 와 같은 식).
바퀴 ω 를 τ 만큼 늦춰 (ω_i(t − τ) 와 추정 속도(t) 비교) 다시 계산 → 지표 변화 = 시간차 몫의 상한.
  τ = 0 은 slip_logger 기록과 같아야 한다 (검산). 창: /torque_mpc/status hold 0, 추정 속도 > 0.05 m/s.
  지표: 큰 슬립 % (max_i|λ_i| > 0.5), 미끄럼 > 0.1 %, λ RMS, λ 최대, 큰 슬립 중 헛돎(λ > 0) 몫 — 메모리 analysis-include-slip.
"""
import math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WR, KG, EPS = 0.04, 0.1932, 2.5
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
BAGS = {"circles8 0.7 (A/B 지금)": ["225012", "225104", "225157"], "circles8 0.7 (A/B 후보)": ["225302", "225349", "225435"],
        "line 가속 5.0·3.5": ["223401", "223553"], "square 가속 1.5": ["195636", "201411"]}
TAUS = (0, 20, 40, 60, 80)


def read(b):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/wheel_states", "/state_estimator/odom", "/slip_logger/slip", "/torque_mpc/status"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, d, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    return D


def lam(w, vx, vy, wz):
    wn = np.stack([vx - vy - KG * wz, vx + vy + KG * wz, vx + vy - KG * wz, vx - vy + KG * wz], 1) / WR
    return (w - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(w), np.abs(wn)), EPS)


def stats(L):
    L = L[np.all(np.isfinite(L), 1)]; a = np.abs(L); mx = a.max(1); big = mx > 0.5
    sg = np.sign(L[np.arange(len(L)), a.argmax(1)])
    return np.array([100 * big.mean(), 100 * (mx > 0.1).mean(), math.sqrt(np.mean(L ** 2)), mx.max(),
                     100 * (sg[big] > 0).mean() if big.any() else np.nan])


def run(b):
    D = read(b)
    S = np.array([[t, m.data[20] if len(m.data) > 20 else 0] for t, m in D["/torque_mpc/status"]]); act = S[S[:, 1] == 0, 0]
    E = np.array([[st(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, m in D["/state_estimator/odom"]])
    W = np.array([[st(m.header), *m.velocity] for _, m in D["/wheel_states"]])
    G = np.array([[t, *m.data[1:5]] for t, m in D["/slip_logger/slip"] if len(m.data) >= 5])
    # 평가 시각 = 추정기 출력 시각 (100 Hz), 창 안
    sp = np.hypot(E[:, 1], E[:, 2])
    m = (E[:, 0] >= act[0]) & (E[:, 0] <= act[-1]) & (sp > 0.05)
    te = E[m, 0]; vx, vy, wz = E[m, 1], E[m, 2], E[m, 3]
    out = {}
    for tau in TAUS:
        w = np.stack([np.interp(te - tau * 1e-3, W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
        out[tau] = stats(lam(w, vx, vy, wz))
    # 검산: slip_logger 기록 (수신 시각) 과 τ = 0 재계산 (추정기 stamp) 의 같은 창 지표
    g = G[(G[:, 0] >= act[0]) & (G[:, 0] <= act[-1])]
    spg = np.hypot(np.interp(g[:, 0], E[:, 0], E[:, 1]), np.interp(g[:, 0], E[:, 0], E[:, 2]))
    out["logger"] = stats(g[spg > 0.05, 1:5])
    return out


if __name__ == "__main__":
    lab = ["큰 슬립 %", "미끄럼>0.1 %", "λ RMS", "λ 최대", "큰 슬립 중 헛돎 %"]
    fmt = ["{:.1f}", "{:.1f}", "{:.3f}", "{:.2f}", "{:.0f}"]
    for name, runs in BAGS.items():
        R = [run(f"bags/2026-11-21/{r}") for r in runs]
        print(f"\n### {name} ({', '.join(runs)}) — 판 평균")
        print("| 지표 | logger 기록 | " + " | ".join(f"τ {t} ms" for t in TAUS) + " |")
        print("|---|---|" + "---|" * len(TAUS))
        for i, (l, f) in enumerate(zip(lab, fmt)):
            row = [np.nanmean([r["logger"][i] for r in R])] + [np.nanmean([r[t][i] for r in R]) for t in TAUS]
            print(f"| {l} | " + " | ".join(f.format(x) for x in row) + " |")
