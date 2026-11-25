# 지도 위 그림이 실제 움직임과 맞나: 기준 경로 (/figure8/path) · 추정기 위치 (/state_estimator/odom, RViz 지나간 경로와 같은 값) ·
#   SLAM 위치 (/tracked_pose, IMU 점 → 로봇 중심, compare_slam.imu_to_center) 를 같은 판에서 겹쳐 본다.  /usr/bin/python3 est_vs_slam.py <bag> ...
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
from compare_slam import imu_to_center
for f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"):
    font_manager.fontManager.addfont(f)
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False})
HERE = os.path.dirname(os.path.abspath(__file__))
R_IMU = np.array([0.1093, 0.017])        # config/torque_estimator.yaml imu_offset_x/y (analyze.py 와 같은 값)
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9      # 메시지 시각 (스캔·추정 시각). bag 받은 시각과 섞지 않는다
yaw_of = lambda q: np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
bags = sys.argv[1:]
fig, axes = plt.subplots(1, len(bags), figsize=(5.2 * len(bags), 5.8), facecolor="#fcfcfb")
for ax, bag in zip(np.atleast_1d(axes), bags):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    T = ["/figure8/path", "/state_estimator/odom", "/tracked_pose", "/mpc/armed"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    paths, od, sl, arm = [], [], [], []
    while r.has_next():
        n, d, t = r.read_next(); m = deserialize_message(d, ty[n]); t *= 1e-9
        if n == "/figure8/path": paths.append((t, np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])))
        elif n == "/state_estimator/odom": od.append([st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation), t])
        elif n == "/tracked_pose": sl.append([st(m.header), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation), t])
        elif n == "/mpc/armed": arm.append([t, float(m.data)])
    od, sl, arm = np.array(od), np.array(sl), np.array(arm)
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]; t_off = td[td > t_on][0] if (td > t_on).any() else od[-1, 4]
    off_od = np.median(od[:, 4] - od[:, 0]); off_sl = np.median(sl[:, 4] - sl[:, 0])          # bag 받은 시각 − 메시지 시각 (늦음)
    t_on, t_off = t_on - off_od, t_off - off_od                                                 # 무장 구간을 메시지 시각 축으로
    sx, sy = imu_to_center(sl[:, 1], sl[:, 2], sl[:, 3], R_IMU)
    w = (od[:, 0] >= t_on) & (od[:, 0] <= t_off); ws = (sl[:, 0] >= t_on) & (sl[:, 0] <= t_off)
    ex = np.interp(sl[ws, 0], od[:, 0], od[:, 1]) - sx[ws]; ey = np.interp(sl[ws, 0], od[:, 0], od[:, 2]) - sy[ws]
    d = np.hypot(ex, ey)
    lab = os.path.basename(bag.rstrip("/"))
    pt = [round(t - t_on, 1) for t, _ in paths]
    ex2 = np.interp(sl[ws, 4] - off_od, od[:, 0], od[:, 1]) - sx[ws]; ey2 = np.interp(sl[ws, 4] - off_od, od[:, 0], od[:, 2]) - sy[ws]
    d2 = np.hypot(ex2, ey2)
    print(f"  (메시지 늦음: 추정기 {1e3*off_od:.0f} ms, SLAM {1e3*off_sl:.0f} ms.  bag 받은 시각끼리 맞추면 RMS {100*np.sqrt(np.mean(d2**2)):.1f} · 최대 {100*d2.max():.1f} cm)")
    print(f"{lab}: /figure8/path 메시지 {len(paths)} 개 (무장 기준 시각 {pt}), 점 {[len(p) for _, p in paths]}  |  추정기 − SLAM 중심 거리 RMS {100*np.sqrt(np.mean(d**2)):.1f} · 최대 {100*d.max():.1f} cm")
    P = paths[-1][1]
    ax.plot(P[:, 0], P[:, 1], color="#52514e", lw=1.3, ls=(0, (4, 3)), label="기준 경로 (/figure8/path 마지막)")
    if len(paths) > 1:
        ax.plot(paths[0][1][:, 0], paths[0][1][:, 1], color="#eda100", lw=1.0, ls=":", label="기준 경로 (첫 메시지)")
    ax.plot(od[w, 1], od[w, 2], color="#2a78d6", lw=1.5, label="추정기 (RViz 지나간 경로)")
    ax.plot(sx[ws], sy[ws], color="#eb6834", lw=1.2, alpha=0.9, label="SLAM (로봇 중심)")
    ax.set_aspect("equal"); ax.grid(color="#e6e5e1", lw=0.6); ax.set_facecolor("#fcfcfb")
    # 기준 경로까지 거리 (SLAM 중심, 무장 구간) — 그림에서 주황 선과 점선 사이로 보이는 거리
    seg = np.hypot(*np.diff(P, axis=0).T); k = np.maximum(1, np.ceil(seg / 0.01).astype(int))
    Dp = np.vstack([np.column_stack([np.linspace(P[i, 0], P[i + 1, 0], k[i], endpoint=False), np.linspace(P[i, 1], P[i + 1, 1], k[i], endpoint=False)]) for i in range(len(P) - 1)])
    dp = np.sqrt(((np.column_stack([sx[ws], sy[ws]])[:, None] - Dp[None]) ** 2).sum(-1)).min(1)
    ax.set_title(f"{lab}\nSLAM ↔ 기준 경로 (주황 ↔ 점선): RMS {100*np.sqrt(np.mean(dp**2)):.1f} · 최대 {100*dp.max():.1f} cm\n"
                 f"추정기 ↔ SLAM (파랑 ↔ 주황, 같은 시각): RMS {100*np.sqrt(np.mean(d**2)):.1f} · 최대 {100*d.max():.1f} cm", fontsize=9.5, loc="left")
    ax.legend(fontsize=7.5, loc="best")
fig.tight_layout()
out = os.path.join(HERE, "est_vs_slam.png"); fig.savefig(out, dpi=140, facecolor="#fcfcfb"); print(out)
