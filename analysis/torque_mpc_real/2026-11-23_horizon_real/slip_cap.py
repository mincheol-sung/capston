"""bag 별 슬립 λ 지표 + (C 만) 마찰 상한·slip guard·계산 시간·Δτ.
창 = 무장 중 & 기준 속도 > 0.05 m/s (moving_only.py 와 같음). λ 식 = slip_timing.py lam() (추정기 v̂ 기준, τ = 0).
쓰는 법: python3 slip_cap.py <bag dir>...
"""
import glob, math, re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WR, KG, EPS = 0.04, 0.1932, 2.5
W4 = ["FL", "FR", "RL", "RR"]
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
TOP = ["/mpc/armed", "/dra_mppi/reference_trajectory", "/state_estimator/odom", "/wheel_states",
       "/torque_mpc/status", "/torque_mpc/cmd_raw", "/torque_output/slip_guard"]


def read(b):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOP if t in ty]))
    D = {k: [] for k in TOP}
    while r.has_next():
        n, d, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    return D


def lam(w, vx, vy, wz):
    wn = np.stack([vx - vy - KG * wz, vx + vy + KG * wz, vx + vy - KG * wz, vx - vy + KG * wz], 1) / WR
    return (w - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(w), np.abs(wn)), EPS)


def tau_bar_from_log(b):
    for f in glob.glob(f"{b}/run_info/ros_log/torque_mpc_node_*.log"):
        m = re.search(r"쓰는 τ̄ \[([0-9. ]+)\]", open(f, encoding="utf-8").read())
        if m:
            return np.array([float(x) for x in m.group(1).split()])
    return None


print("bag | 창 s | 큰 슬립 % (max|λ|>0.5) | 0.5 s 평균 뒤 | λ RMS | λ 최대 | 큰 슬립 중 헛돎 % | 큰 슬립 바퀴 몫 FL/FR/RL/RR % | 큰 슬립 구간 가속/감속/그 밖 %")
C = {}
for b in sys.argv[1:]:
    D = read(b); name = b.rstrip("/").split("/")[-1]
    s0 = next(t for t, m in D["/mpc/armed"] if m.data); s1 = next((t for t, m in D["/mpc/armed"] if not m.data and t > s0), 1e18)
    R = np.array([[t, math.hypot(x.vx, x.vy)] for t, m in D["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for x in [m.references[0]]])
    a_ref = np.gradient(np.convolve(R[:, 1], np.ones(5) / 5, "same"), R[:, 0])
    E = np.array([[st(m.header), t, m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for t, m in D["/state_estimator/odom"]])
    W = np.array([[st(m.header), *m.velocity] for _, m in D["/wheel_states"]])
    tb = E[:, 1]                                                     # 창 판정은 수신 시각 (R 과 같은 시계)
    m = (tb >= R[0, 0]) & (tb <= R[-1, 0]) & (np.interp(tb, R[:, 0], R[:, 1]) > 0.05)
    te = E[m, 0]
    w = np.stack([np.interp(te, W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    L = lam(w, E[m, 2], E[m, 3], E[m, 4]); ok = np.all(np.isfinite(L), 1); L = L[ok]
    a = np.abs(L); mx = a.max(1); big = mx > 0.5; arg = a.argmax(1)
    k = max(1, int(round(0.5 / np.median(np.diff(te)))))
    Ls = np.stack([np.convolve(L[:, i], np.ones(k) / k, "same") for i in range(4)], 1)
    big_s = np.abs(Ls).max(1) > 0.5
    sg = np.sign(L[np.arange(len(L)), arg])
    ar = np.interp(tb[m][ok], R[:, 0], a_ref)
    ph = [100 * (big & (ar > 0.1)).sum() / max(big.sum(), 1), 100 * (big & (ar < -0.1)).sum() / max(big.sum(), 1),
          100 * (big & (np.abs(ar) <= 0.1)).sum() / max(big.sum(), 1)]
    share = [100 * (arg[big] == i).mean() if big.any() else 0 for i in range(4)]
    print(f"{name} | {te[-1]-te[0]:.1f} | {100*big.mean():.1f} | {100*big_s.mean():.1f} | {math.sqrt(np.mean(L**2)):.3f} | {mx.max():.2f} | "
          f"{100*(sg[big] > 0).mean() if big.any() else float('nan'):.0f} | {'/'.join(f'{x:.0f}' for x in share)} | {'/'.join(f'{x:.0f}' for x in ph)}")

    if D["/torque_mpc/status"]:
        S = np.array([m.data for t, m in D["/torque_mpc/status"] if s0 <= t <= s1 and len(m.data) >= 21])
        S = S[S[:, 20] == 0]                                          # hold 아닌 (추종 중) 스텝
        T = np.array([[t, *m.torque] for t, m in D["/torque_mpc/cmd_raw"] if s0 <= t <= s1 and m.mode == m.MODE_TORQUE])
        tbar = tau_bar_from_log(b)
        G = np.array([m.data for t, m in D["/torque_output/slip_guard"] if T[0, 0] <= t <= T[-1, 0]])
        gd = {}
        if G.ndim == 2 and len(G):
            dtg = np.diff(G[:, 0], append=G[-1, 0]); Tg = dtg.sum()
            flag = G[:, 9].astype(int)
            gd = {W4[i]: (100 * dtg[flag == i].sum() / Tg, int(np.sum((flag[1:] == i) & (flag[:-1] != i)) + (flag[0] == i)), G[:, 5 + i].min())
                  for i in range(4)}
        C[name] = dict(ms=S[:, 12], act=S[:, 9], fb=S[:, 11], dtau=np.diff(T[:, 1:], axis=0), cap=np.abs(T[:, 1:]) / tbar, tbar=tbar, gd=gd)

if C:
    print("\nC 판 | τ̄_i [N·m] | |τ|/τ̄ ≥ 0.98 % FL/FR/RL/RR | 계산 ms 평균 / 99 % / 최대 | active 평균 | fallback 수 | Δτ RMS [N·m/스텝] | slip guard 판정 % (횟수, g 최소) FL / FR / RL / RR")
    for n, c in C.items():
        cap = "/".join(f"{100*(c['cap'][:, i] >= 0.98).mean():.1f}" for i in range(4))
        gd = " / ".join(f"{v[0]:.1f} ({v[1]}, {v[2]:.2f})" for v in c["gd"].values())
        print(f"{n} | {np.round(c['tbar'], 4)} | {cap} | {c['ms'].mean():.2f} / {np.percentile(c['ms'], 99):.2f} / {c['ms'].max():.2f} | "
              f"{c['act'].mean():.1f} | {int((c['fb'] > 0).sum())} | {math.sqrt(np.mean(c['dtau']**2)):.4f} | {gd}")
