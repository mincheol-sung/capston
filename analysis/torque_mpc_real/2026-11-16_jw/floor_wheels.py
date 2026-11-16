"""바닥 직선 판(shape:=line, yaw 고정): 순항 구간 바퀴별 MPC 순토크·모터 명령·바퀴 속도 평균. 그리고 벤치 CSV 로 출발(0.08 N·m) 걸린 시간."""
import sys, csv, glob, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
N = ["FL", "FR", "RL", "RR"]
T = ["/dra_mppi/reference_trajectory", "/torque_mpc/cmd_raw", "/wheel_torque_cmd", "/wheel_states", "/mpc/armed"]
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=T))
    R, Cn, Cm, W, A = [], [], [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references: q = m.references[0]; R.append([tr, q.vx, q.vy])
        elif tp == T[1] and m.mode == 0: Cn.append([tr, *m.torque])
        elif tp == T[2] and m.mode == 0: Cm.append([tr, *m.torque])
        elif tp == T[3]: W.append([tr, *m.velocity])
        elif tp == T[4]: A.append((tr, m.data))
    R, Cn, Cm, W = map(np.array, (R, Cn, Cm, W)); arm = [a for a, v in A if v][0]; R = R[R[:, 0] > arm]
    sp = np.hypot(R[:, 1], R[:, 2]); vmax = sp.max()
    cruise = R[sp > 0.8 * vmax]
    # 순항 구간을 방향별로 (갔다 오기면 앞·뒤 두 덩어리)
    print(f"== {path.split('/')[-1]}  순항 {vmax:.2f} m/s")
    for sgn, name in ((1, "앞(+)"), (-1, "뒤(−)")):
        c = cruise[np.sign(cruise[:, 1] if np.abs(cruise[:, 1]).max() > np.abs(cruise[:, 2]).max() else cruise[:, 2]) == sgn]
        if len(c) < 5: continue
        a, b = c[0, 0], c[-1, 0] + 0.1
        sel = lambda X: X[(X[:, 0] > a) & (X[:, 0] < b)]
        n, mm, w = sel(Cn)[:, 1:].mean(0), sel(Cm)[:, 1:].mean(0), sel(W)[:, 1:].mean(0)
        print(f"   {name} {b-a:.1f} s | MPC 순토크 " + " ".join(f"{N[i]} {n[i]:+.4f}" for i in range(4)) + " | 모터 명령 " + " ".join(f"{mm[i]:+.4f}" for i in range(4))
              + " | 바퀴 속도 " + " ".join(f"{w[i]:+.1f}" for i in range(4)))
# 벤치 출발: 0.08 N·m 첫 계단에서 |ω| > 1 rad/s 될 때까지 걸린 시간
D = "/home/smoc/capston/capston/analysis/torque_mpc_real/2026-11-16_jw"
print("\n벤치 출발 (첫 계단 0.08 N·m, 정지에서): |ω| > 1 rad/s 까지 걸린 시간 [s]")
for wid in (1, 2, 3, 4):
    out = []
    for f in sorted(glob.glob(f"{D}/jw_{wid}_*.csv")):
        rows = [r for r in csv.DictReader(open(f)) if r["mode"] == "torque"]
        t = np.array([float(r["t"]) for r in rows]); v = np.array([float(r["v"]) for r in rows])
        i = np.argmax(np.abs(v) > 1.0); out.append(t[i] - t[0] if np.abs(v[i]) > 1 else np.nan)
    print(f"   {N[wid-1]}: " + ", ".join(f"{x:.2f}" for x in out))
