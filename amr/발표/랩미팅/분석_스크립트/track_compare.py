# 세 판 추종 오차를 같은 방식으로: ① 제어기 status 의 [t, e_x, e_y, e_θ] (fault_monitor 가 보는 값, 시간 기준 오차)
#                                   ② 추정기 위치에서 /figure8/path 까지 최단 거리 (경로 기준, 타이밍 무관) + 경로 진행률
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def read(b, want):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    out = {k: [] for k in want}
    while r.has_next():
        n, d, t = r.read_next(); out[n].append((t * 1e-9, deserialize_message(d, types[n])))
    return out
for b in sys.argv[1:]:
    raw = read(b, ("/torque_mpc/status", "/pure_pursuit/status", "/state_estimator/odom", "/figure8/path", "/mpc/armed"))
    arm = [(t, m.data) for t, m in raw["/mpc/armed"]]
    s0 = next(t for t, on in arm if on); s1 = next((t for t, on in arm if not on and t > s0), 1e18)
    st_name = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    St = np.array([[t, *m.data[1:4]] for t, m in raw[st_name] if s0 <= t <= s1])
    exy = np.hypot(St[:, 1], St[:, 2]); eth = np.degrees(np.abs(np.arctan2(np.sin(St[:, 3]), np.cos(St[:, 3]))))
    P = [m for t, m in raw["/figure8/path"] if t <= s1]
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in P[-1].poses])
    Od = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1])
    D = np.sqrt(((Od[:, None, :] - path[None, :, :]) ** 2).sum(-1)); dmin = D.min(1); idx = D.argmin(1)
    seg = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
    print(f"{b.split('/')[-1]} ({st_name.split('/')[1]}): arm {s1 - s0 if s1 < 1e17 else St[-1,0]-s0:.1f} s, 경로 길이 {seg[-1]:.2f} m, 진행 최대 {seg[idx].max():.2f} m ({100*seg[idx].max()/seg[-1]:.0f} %)")
    print(f"   시간 기준 |e_xy| RMS {100*np.sqrt(np.mean(exy**2)):.1f} cm, 최대 {100*exy.max():.1f} cm   |e_θ| RMS {np.sqrt(np.mean(eth**2)):.1f}°, 최대 {eth.max():.1f}°")
    print(f"   경로 기준 거리 RMS {100*np.sqrt(np.mean(dmin**2)):.1f} cm, 최대 {100*dmin.max():.1f} cm")
