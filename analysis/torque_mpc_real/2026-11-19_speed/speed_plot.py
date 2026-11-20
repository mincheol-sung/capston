"""speed_check.py 와 같은 출처로 시간축 그림 + 진행 방향 뒤처짐 (SLAM 중심 vs 기준 위치, 기준 방향 성분). 2026-11-19"""
import sys, math, numpy as np, rosbag2_py, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1724 + 0.2140) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]); P = np.linalg.pinv(Tinv); OFF = np.array([0.1093, 0.0173])
path, out = sys.argv[1], sys.argv[2]
T = ["/dra_mppi/reference_trajectory", "/wheel_states", "/tracked_pose", "/pure_pursuit/cmd_vel"]
rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=[t for t in T if t in types]))
R, W, S, C = [], [], [], []
while rd.has_next():
    tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
    if tp == T[0] and m.references:
        q = m.references[0]; R.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + m.dt, q.x, q.y, q.yaw, math.hypot(q.vx, q.vy)])
    elif tp == T[1]: W.append([tr, *m.velocity])
    elif tp == T[2]:
        th = 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w); c, s = math.cos(th), math.sin(th)
        S.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x - (c * OFF[0] - s * OFF[1]), m.pose.position.y - (s * OFF[0] + c * OFF[1])])
    elif tp == T[3]: C.append([tr, m.twist.linear.x])
R, W, S, C = map(np.array, (R, W, S, C)); R = R[np.argsort(R[:, 0])]
mov = R[R[:, 4] > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
vb = (P @ W[:, 1:5].T).T; ve = np.hypot(vb[:, 0], vb[:, 1])
h = 0.25; ts = np.arange(S[0, 0] + h, S[-1, 0] - h, 0.01)
sx = lambda tt: np.interp(tt, S[:, 0], S[:, 1]); sy = lambda tt: np.interp(tt, S[:, 0], S[:, 2])
vs = np.hypot(sx(ts + h) - sx(ts - h), sy(ts + h) - sy(ts - h)) / (2 * h)
# 진행 방향 뒤처짐: 기준 위치 → SLAM 중심, 기준 진행 방향(경로 접선 = 기준 속도 방향, tangent 라 yaw) 성분
tt = ts[(ts >= t0) & (ts <= t1)]
ex, ey = sx(tt) - np.interp(tt, R[:, 0], R[:, 1]), sy(tt) - np.interp(tt, R[:, 0], R[:, 2]); yw = np.interp(tt, R[:, 0], np.unwrap(R[:, 3]))
ea = np.cos(yw) * ex + np.sin(yw) * ey; vref = np.interp(tt, R[:, 0], R[:, 4])
cr = vref >= 0.65
print(f"진행 방향 오차 (SLAM 중심 − 기준, + = 앞섬): 주행 평균 {100*ea.mean():+.1f} cm, 순항(기준 ≥ 0.65) 평균 {100*ea[cr].mean():+.1f} cm, "
      f"최소 {100*ea.min():+.1f} 최대 {100*ea.max():+.1f} cm → 시간으로 순항 평균 {ea[cr].mean()/0.7*1e3:+.0f} ms")
sel = (ts >= t0) & (ts <= t1)
print(f"SLAM 0.5 s 차분 속도: 순항 구간 중앙 {np.median(vs[sel][np.interp(ts[sel], R[:,0], R[:,4]) >= 0.65]):.3f}, "
      f"10–90 % [{np.percentile(vs[sel][np.interp(ts[sel], R[:,0], R[:,4]) >= 0.65], 10):.3f}, {np.percentile(vs[sel][np.interp(ts[sel], R[:,0], R[:,4]) >= 0.65], 90):.3f}]")
fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
z = R[0, 0]
ax[0].plot(R[:, 0] - z, R[:, 4], "k", lw=2, label="reference |v|")
ax[0].plot(W[:, 0] - z, ve, lw=0.6, alpha=0.8, label="encoder pinv(T^-1)w")
ax[0].plot(ts - z, vs, lw=1.2, label="SLAM center, 0.5 s diff")
if len(C): ax[0].plot(C[:, 0] - z, C[:, 1], lw=0.8, label="PP command v_x")
ax[0].axhline(0.7, color="r", ls=":", lw=0.8); ax[0].set_ylabel("speed [m/s]"); ax[0].legend(fontsize=8, loc="upper right"); ax[0].set_ylim(0, 1.2)
ax[0].set_xlim(t0 - z - 1, t1 - z + 1); ax[0].set_title(f"{path.split('/')[-1]}  speed vs reference")
ax[1].plot(tt - z, 100 * ea, lw=1); ax[1].axhline(0, color="k", lw=0.5); ax[1].set_ylabel("along-track error [cm]\n(+ = ahead)"); ax[1].set_xlabel("time [s]")
fig.tight_layout(); fig.savefig(out, dpi=110)
