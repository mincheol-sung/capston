"""기준 속도가 0.7 이면 로봇이 실제로 0.7 로 갔나 — 속도 출처 네 개를 기준 순항 구간에서 비교. 2026-11-19
  기준      /dra_mppi/reference_trajectory 첫 행 |v| (목표 시각 = stamp + dt)
  엔코더    v = pinv(T⁻¹)·ω  (r 0.04, k 0.1932 — 2026-11-19 (7) 새 기하). 바퀴가 헛돌면 실제보다 크다
  SLAM      /tracked_pose (IMU 점) → 몸체 중심 = p − R(θ)·[0.1093, 0.0173] 로 옮긴 뒤 0.2 s 중앙 차분
            (원 위에서 ω 1.4 rad/s 면 IMU 점은 중심보다 ω·0.11 ≈ 0.15 m/s 빠르게 돈다 — 옮기지 않으면 과대)
  추정기    /state_estimator/odom twist |v|
  PP 명령   /pure_pursuit/cmd_vel linear.x (v_ref + k_along·뒤처짐, 상한 v_max)"""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1724 + 0.2140) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]); P = np.linalg.pinv(Tinv)
OFF = np.array([0.1093, 0.0173])
T = ["/dra_mppi/reference_trajectory", "/wheel_states", "/tracked_pose", "/state_estimator/odom", "/pure_pursuit/cmd_vel", "/torque_mpc/status"]
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=[t for t in T if t in types]))
    R, W, S, E, C = [], [], [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references:
            q = m.references[0]; R.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + m.dt, math.hypot(q.vx, q.vy), q.yaw_rate])
        elif tp == T[1]: W.append([tr, *m.velocity])
        elif tp == T[2]:
            th = 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w); c, s = math.cos(th), math.sin(th)
            S.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x - (c * OFF[0] - s * OFF[1]),
                      m.pose.position.y - (s * OFF[0] + c * OFF[1])])
        elif tp == T[3]: E.append([tr, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)])
        elif tp == T[4]: C.append([tr, m.twist.linear.x])
    R, W, S, E = map(np.array, (R, W, S, E)); C = np.array(C) if C else None
    R = R[np.argsort(R[:, 0])]
    cr = R[R[:, 1] >= 0.69 * R[:, 1].max()]                    # 기준 순항 구간
    t0, t1 = cr[0, 0], cr[-1, 0]
    vb = (P @ W[:, 1:5].T).T; ve = np.hypot(vb[:, 0], vb[:, 1])
    h = 0.1; ts = np.arange(S[0, 0] + h, S[-1, 0] - h, 0.01)
    sx = lambda tt: np.interp(tt, S[:, 0], S[:, 1]); sy = lambda tt: np.interp(tt, S[:, 0], S[:, 2])
    vs = np.hypot(sx(ts + h) - sx(ts - h), sy(ts + h) - sy(ts - h)) / (2 * h)
    print(f"== {path.split('/')[-1]}  기준 최대 |v| {R[:,1].max():.3f} m/s, 순항(≥ 69 %) {t0-R[0,0]:.1f} ~ {t1-R[0,0]:.1f} s ({t1-t0:.1f} s), "
          f"기준 이동 {np.trapz(R[:,1], R[:,0]):.2f} m")
    def row(name, t, v):
        sel = (t >= t0) & (t <= t1); x = v[sel]
        if not len(x): print(f"   {name:8s} 표본 없음"); return
        print(f"   {name:8s} 순항 구간 평균 {x.mean():.3f}  중앙 {np.median(x):.3f}  10–90 % [{np.percentile(x,10):.3f}, {np.percentile(x,90):.3f}]  "
              f"최소 {x.min():.3f} 최대 {x.max():.3f}  ≥0.65 시간 {100*np.mean(x>=0.65):.0f} %   (전체 최대 {v.max():.3f})")
    row("기준", R[:, 0], R[:, 1]); row("엔코더", W[:, 0], ve); row("SLAM", ts, vs); row("추정기", E[:, 0], E[:, 1])
    if C is not None: row("PP 명령", C[:, 0], C[:, 1])
    # 이동 거리로 한 번 더: 기준 순항 구간 동안 간 거리
    sel = (ts >= t0) & (ts <= t1)
    print(f"   순항 {t1-t0:.2f} s 동안 기준 {np.trapz(cr[:,1], cr[:,0]):.2f} m, SLAM 중심 {np.trapz(vs[sel], ts[sel]):.2f} m, "
          f"엔코더 {np.trapz(ve[(W[:,0]>=t0)&(W[:,0]<=t1)], W[(W[:,0]>=t0)&(W[:,0]<=t1),0]):.2f} m")
