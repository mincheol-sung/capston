"""바퀴가 0 에 붙은 사건: 필요 바퀴속도 |ω_ref| > 1.5 rad/s 인데 측정 |ω| < 0.3 rad/s, 모터 명령이 필요 방향으로 0.15 s 넘게."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r_w, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r_w) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
NAMES = ["FL", "FR", "RL", "RR"]
T = ["/dra_mppi/reference_trajectory", "/wheel_states", "/mpc/armed", "/wheel_torque_cmd", "/torque_mpc/status"]
print("판      | τ̄ N·m  | 주행 s | 붙음 사건 (바퀴@시작 t s, 붙은 시간 s, |τ_motor| 평균 N·m, 풀림?)")
for path in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=T))
    R, W, A, C, SB = [], [], [], [], []
    while r.has_next():
        tp, d, tr = r.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references: q = m.references[0]; R.append([tr, q.vx, q.vy, q.yaw_rate])
        elif tp == T[1]: W.append([tr, *m.velocity])
        elif tp == T[2]: A.append([tr, float(m.data)])
        elif tp == T[3]: C.append([tr, float(m.mode), *m.torque])
        elif tp == T[4] and m.data[8] > 0: SB.append(m.data[8])
    R, W, C = map(np.array, (R, W, C)); TB = float(np.median(SB))
    arm = [a[0] for a in A if a[1] > 0.5][0]
    dis = [a[0] for a in A if a[1] < 0.5 and a[0] > arm]; end = dis[0] if dis else W[-1, 0]
    Rm = R[(R[:, 0] > arm) & (R[:, 0] < end)]; mov = Rm[np.hypot(Rm[:, 1], Rm[:, 2]) > 1e-3]
    t0, t1 = mov[0, 0], mov[-1, 0]
    Wm = W[(W[:, 0] > t0) & (W[:, 0] < min(t1, end))]
    iR = np.clip(np.searchsorted(R[:, 0], Wm[:, 0]), 0, len(R) - 1); iC = np.clip(np.searchsorted(C[:, 0], Wm[:, 0]), 0, len(C) - 1)
    need = (Tinv @ R[iR, 1:4].T).T; tau = C[iC, 2:6]
    ev = []
    for w in range(4):
        om = Wm[:, 1 + w]
        st = (np.abs(om) < 0.3) & (np.abs(need[:, w]) > 1.5) & (np.sign(tau[:, w]) == np.sign(need[:, w]))
        i, n = 0, len(st)
        while i < n:
            if not st[i]: i += 1; continue
            j = i
            while j + 1 < n and st[j + 1]: j += 1
            dur = Wm[j, 0] - Wm[i, 0]
            if dur >= 0.15:
                post = om[j + 1:j + 20]; freed = bool(len(post) and np.any(post * np.sign(need[j, w]) > 1.0))
                ev.append(f"{NAMES[w]}@{Wm[i,0]-t0:.2f} {dur:.2f}s {np.mean(np.abs(tau[i:j+1, w])):.3f} {'풀림' if freed else ('끝' if j + 1 >= n else '안풀림')}")
            i = j + 1
    print(f"{path.split('/')[-1]}  | {TB:.4f} | {t1-t0:5.1f}  | {', '.join(ev) if ev else '없음'}")
