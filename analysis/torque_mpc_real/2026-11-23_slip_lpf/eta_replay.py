#!/usr/bin/python3
"""η 시그모이드를 기존 실차 λ 에 씌우면 얼마나 깎였을까 (2026-11-23, 열린 고리 재생 — 실제로는 η 가 토크를 바꿔 λ 도 바뀐다).
λ̃ = 실시간과 같은 인과 1 차 LPF (f_c 10 Hz, y += α(x − y), α = Δt/(τ + Δt), τ = 1/(2π f_c)).
η(λ̃) = 1 / (1 + exp((|λ̃| − λ_c)/s)).  창·판 선택은 all_runs.py 와 같다 (플래그 없는 판).
  nice -n 19 ionice -c3 /usr/bin/python3 analysis/torque_mpc_real/2026-11-23_slip_lpf/eta_replay.py   → out_eta.md"""
import csv, os
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(H, "../../.."))
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
FC, S = 10.0, 0.0167
LC = [0.20, 0.25, 0.30]
GROUPS = [("square a1.5 vmax1.5", "C", ""), ("square a1.5 vmax1.5", "PP", "고정 0.30(기본)"),
          ("line dir0 v1.0 a3.0", "C", ""), ("circles8 v[0.43,0.61,0.8]", "C", ""), ("circles8 v[0.43,0.61,0.8]", "PP", "고정 0.30(기본)"),
          ("circles8 v0.7", "C", ""), ("line dir90 v0.3 a?", "C", "")]
runs = [r for r in csv.DictReader(open(os.path.join(H, "runs.csv"))) if not r["flags"]]


def load(bag):
    d = os.path.join(ROOT, "bags", bag)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=d, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/slip_logger/slip", "/state_estimator/odom", "/torque_mpc/status", "/pure_pursuit/status"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, dd, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(dd, ty[n])))
    pp = len(D["/pure_pursuit/status"]) > len(D["/torque_mpc/status"])
    act = np.array([t for t, mm in D["/pure_pursuit/status"] if len(mm.data) > 6 and mm.data[6] != 2]) if pp else \
        np.array([t for t, mm in D["/torque_mpc/status"] if not (len(mm.data) > 20 and mm.data[20] == 1)])
    Sg = np.array([[t, *mm.data[1:5]] for t, mm in D["/slip_logger/slip"] if len(mm.data) >= 5])
    E = np.array([[st(mm.header), np.hypot(mm.twist.twist.linear.x, mm.twist.twist.linear.y)] for _, mm in D["/state_estimator/odom"]])
    L = np.where(np.isfinite(Sg[:, 1:5]), Sg[:, 1:5], 0.0)
    dt = np.diff(Sg[:, 0], prepend=Sg[0, 0]); tau = 1 / (2 * np.pi * FC)
    Lf = np.zeros_like(L); y = np.zeros(4)
    for k in range(len(L)):
        y = y + dt[k] / (tau + dt[k]) * (L[k] - y); Lf[k] = y
    win = (Sg[:, 0] >= act[0]) & (Sg[:, 0] <= act[-1]) & (np.interp(Sg[:, 0], E[:, 0], E[:, 1]) > 0.05)
    return Lf[win], float(1 / np.median(np.diff(Sg[:, 0])))


out = ["| 시나리오 | 제어기 | 판 | \\|λ̃\\| 중앙값 | \\|λ̃\\| 90 % | " + " | ".join(f"λ_c {c}: η<0.5 시간 % / τ̄ 평균 배율" for c in LC) + " |",
       "|---|---|---|---|---|" + "---|" * len(LC)]
for scen, ctrl, la in GROUPS:
    sel = [r for r in runs if r["scen"] == scen and r["ctrl"] == ctrl and r["lookahead"] == la]
    per = []
    for r in sel:
        Lf, fs = load(r["bag"])
        a = np.abs(Lf)
        row = [np.median(a), np.percentile(a, 90)]
        for c in LC:
            eta = 1 / (1 + np.exp((a - c) / S))
            row += [100 * np.mean(eta < 0.5), float(np.mean(eta))]
        per.append(row)
        print(r["bag"], ctrl, scen, f"fs {fs:.0f}", np.round(row, 3), flush=True)
    m = np.median(np.array(per), 0)
    out.append(f"| {scen} | {ctrl} {la} | {len(sel)} | {m[0]:.3f} | {m[1]:.3f} | " +
               " | ".join(f"{m[2 + 2 * i]:.1f} / {m[3 + 2 * i]:.2f}" for i in range(len(LC))) + " |")
open(os.path.join(H, "out_eta.md"), "w").write(
    f"# η 열린 고리 재생 (인과 LPF {FC} Hz, s {S}) — 판별 중앙값, 바퀴 4 개 × 시간 모두 모음\n\n" + "\n".join(out) + "\n")
print("\n".join(out))
