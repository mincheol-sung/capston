"""추정기 출력 품질 (est_mode kf vs slam): 추정기 포즈 − SLAM 중심 (SLAM stamp 에서), 추정기 속도 vs 엔코더 속도 (지연·차이), 한 주기 포즈 튐. 2026-11-19
SLAM /tracked_pose 는 IMU 점 → 중심 = p − R(θ)·[0.1093, 0.0173]. 엔코더 v = pinv(T⁻¹)ω (k 0.1932). 주행 구간 = 바퀴 |ω| > 3 rad/s."""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r, k = 0.04, (0.1724 + 0.2140) / 2
Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]); P = np.linalg.pinv(Tinv); OFF = np.array([0.1093, 0.0173])
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/state_estimator/odom", "/tracked_pose", "/wheel_states"]))
    E, S, W = [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == "/state_estimator/odom":
            q = m.pose.pose; th = 2 * math.atan2(q.orientation.z, q.orientation.w)
            E.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, tr, q.position.x, q.position.y, th,
                      m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z])
        elif tp == "/tracked_pose":
            th = 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w); c, s = math.cos(th), math.sin(th)
            S.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x - (c * OFF[0] - s * OFF[1]),
                      m.pose.position.y - (s * OFF[0] + c * OFF[1]), th])
        else: W.append([tr, *m.velocity])
    E, S, W = map(np.array, (E, S, W))
    mv = W[np.abs(W[:, 1:5]).max(1) > 3.0, 0]; t0, t1 = mv.min(), mv.max()
    Sm = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]
    ex, ey = np.interp(Sm[:, 0], E[:, 0], E[:, 2]) - Sm[:, 1], np.interp(Sm[:, 0], E[:, 0], E[:, 3]) - Sm[:, 2]
    th = Sm[:, 3]; ea = np.cos(th) * ex + np.sin(th) * ey; ec = -np.sin(th) * ex + np.cos(th) * ey
    eth = np.degrees(np.angle(np.exp(1j * (np.interp(Sm[:, 0], E[:, 0], np.unwrap(E[:, 4])) - th))))
    # 속도: 추정기 몸체 v_x (수신 시각) vs 엔코더 v_x — 1 s 이동평균 뺀 빠른 몫 상호상관으로 지연, 그리고 차이 RMS
    g = np.arange(t0, t1, 0.002)
    vb = (P @ W[:, 1:5].T).T
    out = [f"== {path.split('/')[-1]}  주행 {t1-t0:.1f} s  추정기 {len(E)/(E[-1,1]-E[0,1]):.0f} Hz, 수신−stamp 중앙 {1e3*np.median(E[:,1]-E[:,0]):.1f} ms"]
    out.append(f"   포즈 − SLAM 중심 (SLAM stamp {len(Sm)} 개): 앞뒤 평균 {1e3*ea.mean():+.1f} RMS {1e3*np.sqrt(np.mean(ea**2)):.1f} mm, "
               f"옆 평균 {1e3*ec.mean():+.1f} RMS {1e3*np.sqrt(np.mean(ec**2)):.1f} mm, 요 평균 {eth.mean():+.2f} RMS {np.sqrt(np.mean(eth**2)):.2f}°")
    for j, nm in ((0, "v_x"), (2, "ω_z")):
        a = np.interp(g, W[:, 0], vb[:, j]); b = np.interp(g, E[:, 1], E[:, 5 + (0 if j == 0 else 2)])
        diff = b - a
        ah, bh = a - np.convolve(a, np.ones(500) / 500, "same"), b - np.convolve(b, np.ones(500) / 500, "same")
        L = np.arange(-25, 151); c = [np.corrcoef(ah[300:-400], bh[300 + l:len(bh) - 400 + l])[0, 1] for l in L]
        i = int(np.argmax(c))
        out.append(f"   {nm}: 추정기 − 엔코더 평균 {diff.mean():+.3f} RMS {np.sqrt(np.mean(diff**2)):.3f}, 빠른 몫 지연 {2*L[i]:+d} ms (상관 {c[i]:.2f})")
    Em = E[(E[:, 0] >= t0) & (E[:, 0] <= t1)]
    th_e = Em[:, 4]; vx_w = np.cos(th_e) * Em[:, 5] - np.sin(th_e) * Em[:, 6]; vy_w = np.sin(th_e) * Em[:, 5] + np.cos(th_e) * Em[:, 6]
    dp = np.diff(Em[:, 2:4], axis=0) - np.stack([vx_w[:-1], vy_w[:-1]], 1) * np.diff(Em[:, 0])[:, None]; j = np.hypot(dp[:, 0], dp[:, 1])
    out.append(f"   포즈 한 주기 튐 (|Δp − v·Δt|): > 5 mm {np.sum(j > 0.005)} 번, > 1 cm {np.sum(j > 0.01)} 번, 최대 {1e3*j.max():.1f} mm, 합 {100*j.sum():.0f} cm")
    print("\n".join(out))
