#!/usr/bin/python3
"""LPF 슬립 λ̃ 의 분포와 "바닥" (지면 힘이 거의 없을 때의 λ̃), μ–λ 곡선 (2026-11-23).
η 의 꺾임 시작점 (η 가 1 에서 내려가기 시작하는 λ) 을 정하는 근거를 찾는다.

  λ̃_c  : 인과 1 차 LPF 10 Hz — 실시간 η 가 보게 될 값 (분포·바닥)
  λ̃_z, u: filtfilt 10 Hz (위상 0) — μ–λ 곡선 (지연 어긋남 없이)
  τ_g,i = τ_meas,i − J_w ω̇_i − (sgn(ω)·coulomb_i + viscous_i·ω)   (지면이 바퀴에 준 토크의 반작용 = 바퀴가 지면에 낸 토크)
  u_i   = τ_g,i·sgn(ω_nom,i) / ((r/√2)·N_i),  N_i = 몫_i·m g        (λ 와 같은 부호 규약: 구동 +, 제동 −. 단위는 μ 와 같다)
창·판 선택은 all_runs.py 와 같다 (플래그 없는 판).
  nice -n 19 ionice -c3 /usr/bin/python3 analysis/torque_mpc_real/2026-11-23_slip_lpf/lam_floor.py   → out_floor.md, floor_samples.npz"""
import csv, os
import numpy as np, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(H, "../../.."))
P = yaml.safe_load(open(os.path.join(ROOT, "src/amr_basic_stack/controller/mpc/config/torque_plant.yaml")))
r = P["geometry"]["wheel_radius_m"]; m = P["body"]["mass_kg"]; share = np.array(P["body"]["wheel_load_share"])
Jw = P["wheel"]["inertia_J_w_kgm2"]; CO = np.array(P["friction"]["coulomb_Nm"]); VI = np.array(P["friction"]["viscous_Nms_per_rad"])
UNIT = r / np.sqrt(2) * share * m * 9.81            # (r/√2)·N_i [N·m per μ]
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
FC = 10.0
EPSS = (5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0)   # η 전용 ω_ε 후보 (정확한 λ, 인과 LPF) — 2026-11-23 비교
runs = [x for x in csv.DictReader(open(os.path.join(H, "runs.csv"))) if not x["flags"]]


def load(bag):
    d = os.path.join(ROOT, "bags", bag)
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=d, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/slip_logger/slip", "/state_estimator/odom", "/torque_mpc/status", "/pure_pursuit/status", "/wheel_states"]
    rd.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in rd.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while rd.has_next():
        n, dd, t = rd.read_next(); D[n].append((t * 1e-9, deserialize_message(dd, ty[n])))
    pp = len(D["/pure_pursuit/status"]) > len(D["/torque_mpc/status"])
    act = np.array([t for t, mm in D["/pure_pursuit/status"] if len(mm.data) > 6 and mm.data[6] != 2]) if pp else \
        np.array([t for t, mm in D["/torque_mpc/status"] if not (len(mm.data) > 20 and mm.data[20] == 1)])
    S = np.array([mm.data[:9] for _, mm in D["/slip_logger/slip"] if len(mm.data) >= 9])
    W = {round(st(mm.header), 6): (mm.velocity, mm.torque) for _, mm in D["/wheel_states"]}
    keep = np.array([round(t, 6) in W for t in S[:, 0]]); S = S[keep]
    om = np.array([W[round(t, 6)][0] for t in S[:, 0]]); tq = np.array([W[round(t, 6)][1] for t in S[:, 0]])
    E = np.array([[st(mm.header), np.hypot(mm.twist.twist.linear.x, mm.twist.twist.linear.y)] for _, mm in D["/state_estimator/odom"]])
    t = S[:, 0]; L = S[:, 1:5]; wn = S[:, 5:9]
    ok = np.isfinite(L).all(1); L = np.where(np.isfinite(L), L, 0.0)
    fs = 1 / np.median(np.diff(t)); bb, aa = butter(1, FC / (fs / 2))
    dt = np.diff(t, prepend=t[0]); tau = 1 / (2 * np.pi * FC)
    Lc = np.zeros_like(L); y = np.zeros(4)
    for k in range(len(L)):
        y = y + dt[k] / (tau + dt[k]) * (L[k] - y); Lc[k] = y
    Lz = filtfilt(bb, aa, L, axis=0)
    sg = np.where(wn >= 0, 1.0, -1.0)
    def causal(X):
        Y = np.zeros_like(X); y = np.zeros(X.shape[1])
        for k in range(len(X)):
            y = y + dt[k] / (tau + dt[k]) * (X[k] - y); Y[k] = y
        return Y
    alt = {e: causal((om - wn) * sg / np.maximum(np.maximum(np.abs(om), np.abs(wn)), e)) for e in EPSS}
    dv = causal(r * (om - wn) * sg)
    omf = filtfilt(bb, aa, om, axis=0); domf = np.gradient(omf, t, axis=0)
    tg = filtfilt(bb, aa, tq, axis=0) - Jw * domf - (np.sign(omf) * CO + VI * omf)
    u = tg * np.where(wn >= 0, 1.0, -1.0) / UNIT
    win = ok & (t >= act[0]) & (t <= act[-1]) & (np.interp(t, E[:, 0], E[:, 1]) > 0.05)
    return Lc[win], Lz[win], u[win], np.abs(wn[win]), alt[5.0][win], alt[10.0][win], dv[win], np.stack([alt[e][win] for e in EPSS], 1)


pct = lambda a, q=(50, 75, 90, 95, 99): " | ".join(f"{np.percentile(a, x):.3f}" for x in q) if len(a) else " | ".join("-" for _ in q)
A = {}
for x in runs:
    R_ = load(x["bag"]); Lc = R_[0]
    key = (x["scen"], x["ctrl"] + (" " + x["lookahead"] if x["lookahead"] else ""))
    A.setdefault(key, []).append(R_)
    print(x["bag"], key, len(Lc), f"|λ̃| 90 % {np.percentile(np.abs(Lc), 90):.3f}", flush=True)
cat = lambda g, i: np.concatenate([z[i] for z in g])
lines = ["# λ̃ 분포·바닥·μ–λ 곡선 (플래그 없는 실차 판, 바퀴 4 개 × 시간 모음)\n",
         "## 1. 인과 10 Hz λ̃ 의 |λ̃| 백분위 (시나리오별)\n",
         "| 시나리오 | 제어기 | 판 | 표본 | 50 % | 75 % | 90 % | 95 % | 99 % |", "|---|---|---|---|---|---|---|---|---|"]
for k in sorted(A):
    lines.append(f"| {k[0]} | {k[1]} | {len(A[k])} | {len(cat(A[k], 0))} | {pct(np.abs(cat(A[k], 0)).ravel())} |")
Lc = np.concatenate([cat(g, 0) for g in A.values()]); Lz = np.concatenate([cat(g, 1) for g in A.values()])
U = np.concatenate([cat(g, 2) for g in A.values()]); WN = np.concatenate([cat(g, 3) for g in A.values()])
A5 = np.concatenate([cat(g, 4) for g in A.values()]); A10 = np.concatenate([cat(g, 5) for g in A.values()]); DV = np.concatenate([cat(g, 6) for g in A.values()])
LINE = np.concatenate([np.full(cat(g, 0).shape, k[0].startswith(("line dir0", "square"))) for k, g in A.items()])
C8 = np.concatenate([np.full(cat(g, 0).shape, k[0].startswith("circles8")) for k, g in A.items()])
np.savez_compressed(os.path.join(H, "floor_samples.npz"), Lc=Lc, Lz=Lz, U=U, WN=WN, A5=A5, A10=A10, DV=DV, LINE=LINE, C8=C8,
                    AE=np.concatenate([cat(g, 7) for g in A.values()]), EPSS=np.array(EPSS))
lines += ["\n## 2. 바닥: 지면 힘이 작을 때 (|u| < 문턱) 의 |λ̃| 백분위 (전체 판)\n",
          "| 조건 | 표본 % | 50 % | 75 % | 90 % | 95 % | 99 % |", "|---|---|---|---|---|---|---|"]
for th in (0.02, 0.05, 0.10):
    sel = np.abs(U) < th
    lines.append(f"| \\|u\\| < {th} | {100 * sel.mean():.1f} | {pct(np.abs(Lc[sel]))} |")
for lo, hi in ((2.5, 10), (10, 20), (20, 40)):
    sel = (np.abs(U) < 0.05) & (WN >= lo) & (WN < hi)
    lines.append(f"| \\|u\\| < 0.05, ω_nom {lo}~{hi} rad/s | {100 * sel.mean():.1f} | {pct(np.abs(Lc[sel]))} |")
sel = np.abs(U) < 0.05
lines += ["\n바퀴별 바닥 λ̃ (부호 있음, |u| < 0.05) 중앙값 [FL, FR, RL, RR]: " +
          str(np.round([np.median(Lc[sel[:, i], i]) for i in range(4)], 3)) + "\n",
          "\n## 3. μ–λ 곡선: λ̃ (filtfilt) 구간별 u 중앙값 (전체 판, 바퀴 모음)\n",
          "| λ̃ 구간 | 표본 | u 중앙값 | u 25 % | u 75 % |", "|---|---|---|---|---|"]
edges = np.r_[-1.0, -0.6, -0.4, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.6, 1.0]
for a, b in zip(edges[:-1], edges[1:]):
    sel = (Lz >= a) & (Lz < b)
    if sel.sum() > 50:
        lines.append(f"| {a:+.2f} ~ {b:+.2f} | {sel.sum()} | {np.median(U[sel]):+.3f} | {np.percentile(U[sel], 25):+.3f} | {np.percentile(U[sel], 75):+.3f} |")
open(os.path.join(H, "out_floor.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
