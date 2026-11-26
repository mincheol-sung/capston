#!/usr/bin/python3
"""SLAM (/tracked_pose) 은 왜 흔들리나 (2026-11-25, bag 164118_sq2p_mpcwd_r4).
SLAM 포즈 두 개 사이 (≈ 0.1 s) 이동을 바퀴 기구학 (T ω 적분) 과 견준다 — 몸체 앞 (x) · 옆 (y) 로.
  잔차 = SLAM 이동 − 바퀴 이동. 바퀴 미끄럼은 천천히 변하는 몫이라 0.5 s 이동중앙값을 뺀 몫을 "SLAM 흔들림" 으로 본다.
  속도 구간·가감속 여부별, /submap_list (전역 최적화로 지도↔궤적 다시 맞춤) 직후인지별, stamp 간격 (스캔 시각) 별로.
  정지 구간 (무장 전) SLAM 떨림도.
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/slam_noise.py [bag]"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(H, "../../.."))
BAG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "bags/2026-11-25/164118_sq2p_mpcwd_r4")
OFF = np.array([0.1093, 0.017]); R_W, K = 0.04, (0.1724 + 0.214) / 2
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
T = ["/tracked_pose", "/wheel_states", "/submap_list", "/state_estimator/odom"]
r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
S, W, SM, E = [], [], [], []
while r.has_next():
    n, d, tb = r.read_next(); m = deserialize_message(d, ty[n])
    if n == "/tracked_pose":
        p = m.pose; ps = yaw_of(p.orientation); c, s = math.cos(ps), math.sin(ps)
        S.append((st(m.header), p.position.x - (c * OFF[0] - s * OFF[1]), p.position.y - (s * OFF[0] + c * OFF[1]), ps, tb * 1e-9))
    elif n == "/wheel_states":
        W.append((st(m.header), *m.velocity))
    elif n == "/submap_list":
        SM.append(tb * 1e-9)
    elif n == "/state_estimator/odom":
        E.append((st(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y))
S, W, SM, E = map(np.array, (S, W, SM, E))
om = W[:, 1:5]; tw = W[:, 0]
vxw = R_W / 4 * om.sum(1); vyw = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
cx, cy = np.concatenate([[0], np.cumsum(vxw[1:] * np.diff(tw))]), np.concatenate([[0], np.cumsum(vyw[1:] * np.diff(tw))])
ts = S[:, 0]; dt = np.diff(ts)
ps = np.unwrap(S[:, 3]); ym = 0.5 * (ps[1:] + ps[:-1])
dx, dy = np.diff(S[:, 1]), np.diff(S[:, 2])
bx = np.cos(ym) * dx + np.sin(ym) * dy; by = -np.sin(ym) * dx + np.cos(ym) * dy   # SLAM 이동, 몸체 좌표
wx = np.diff(np.interp(ts, tw, cx)); wy = np.diff(np.interp(ts, tw, cy))       # 바퀴 이동 (몸체 좌표 적분)
rx, ry = bx - wx, by - wy
tm = 0.5 * (ts[1:] + ts[:-1]); v = np.interp(tm, tw, vxw)
vf = np.interp(tm, E[:, 0], np.hypot(E[:, 1], E[:, 2])); acc = np.gradient(vf, tm)


def detrend(x, win=0.5):
    out = np.empty_like(x)
    for i in range(len(x)):
        m = np.abs(tm - tm[i]) <= win / 2
        out[i] = x[i] - np.median(x[m])
    return out


hx, hy = detrend(rx), detrend(ry)
mov = np.abs(v) > 0.1; still = (np.abs(v) < 0.005) & (np.interp(tm, tw, np.abs(vyw)) < 0.005)
print(f"SLAM {len(S)} 개, stamp 간격 중앙값 {1e3 * np.median(dt):.1f} ms (5 % {1e3 * np.percentile(dt, 5):.1f}, 95 % {1e3 * np.percentile(dt, 95):.1f}), "
      f"도착 − stamp 중앙값 {1e3 * np.median(S[:, 4] - ts):.0f} ms, /submap_list {len(SM)} 개")
R = lambda a: f"{1e3 * np.sqrt(np.mean(a ** 2)):.1f}"
print("\n| 구간 | 간격 수 | 앞 흔들림 RMS mm | 옆 흔들림 RMS mm | 앞 잔차 평균 mm (미끄럼·시각 치우침) |\n|---|---|---|---|---|")
print(f"| 정지 (무장 전·후) | {still.sum()} | {R(hx[still])} | {R(hy[still])} | {1e3 * rx[still].mean():+.2f} |")
for lo, hi in ((0.1, 0.5), (0.5, 1.0), (1.0, 1.6)):
    m = mov & (np.abs(v) >= lo) & (np.abs(v) < hi)
    if m.sum() > 3:
        print(f"| {lo}~{hi} m/s | {m.sum()} | {R(hx[m])} | {R(hy[m])} | {1e3 * rx[m].mean():+.2f} |")
for lab, m in (("가감속 |a| > 0.5 m/s²", mov & (np.abs(acc) > 0.5)), ("등속 |a| < 0.3", mov & (np.abs(acc) < 0.3))):
    print(f"| {lab} | {m.sum()} | {R(hx[m])} | {R(hy[m])} | {1e3 * rx[m].mean():+.2f} |")
after = np.zeros(len(tm), bool)   # /submap_list 는 주기 발행이라 (3.3 Hz) 최적화 시점을 못 가른다 — 대신 아래 자기상관
idx = np.where(mov)[0]; cont = idx[np.diff(idx, prepend=-9) == 1]
for lab, h in (("앞", hx), ("옆", hy)):
    a, b = h[cont - 1], h[cont]
    print(f"{lab} 흔들림의 이웃 간격 상관 {np.corrcoef(a, b)[0, 1]:+.2f} (위치 잡음이 되돌아오면 −0.5, 한 번 옮겨지고 남으면 0)")
# stamp 간격과 앞 잔차: 시각이 틀리면 앞 잔차 ≈ v × (간격 오차)
m = mov
c = np.corrcoef(rx[m], v[m] * (dt[m] - np.median(dt)))[0, 1]
print(f"\n앞 잔차 vs v × (stamp 간격 − 중앙값) 상관 {c:+.2f} (시각 흔들림이 원인이면 1 에 가깝다)")
big = mov & (np.hypot(hx, hy) > 0.02)
print(f"흔들림 > 2 cm 인 간격 {big.sum()} / {mov.sum()} 개 — 그중 가감속 중 {int((big & (np.abs(acc) > 0.5)).sum())}")
np.savez_compressed(os.path.join(H, "slam_noise.npz"), tm=tm, rx=rx, ry=ry, hx=hx, hy=hy, v=v, acc=acc, after=after)
