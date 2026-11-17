import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
path = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
T = ["/state_estimator/status", "/torque_mpc/status", "/mpc/armed", "/dra_mppi/reference_trajectory"]; r.set_filter(rosbag2_py.StorageFilter(topics=T))
E, M, A, R = [], [], [], []
while r.has_next():
    tp, d, tr = r.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
    if tp == T[0]: E.append([tr, *m.data[:3]])
    elif tp == T[1]: M.append([tr, m.data[1], m.data[2]])
    elif tp == T[2]: A.append((tr, m.data))
    elif m.references: q = m.references[0]; R.append([tr, q.vx, q.vy])
E, M, R = map(np.array, (E, M, R)); arm = [a for a, v in A if v][0]
Rm = R[R[:, 0] > arm]; t0 = Rm[np.hypot(Rm[:, 1], Rm[:, 2]) > 1e-3][0, 0]
res = np.hypot(E[:, 2], E[:, 3]); sel = E[:, 0] > arm - 1
for i in np.argsort(-res * sel)[:4]:
    print(f"  SLAM 보정 잔차 {100*res[i]:.1f} cm @ 기준 출발 {E[i,0]-t0:+.2f} s (무장 {E[i,0]-arm:+.2f} s)")
mm = np.hypot(M[:, 1], M[:, 2]); j = np.argmax(mm * (M[:, 0] > arm))
print(f"  MPC 위치 오차 최대 {100*mm[j]:.1f} cm @ 기준 출발 {M[j,0]-t0:+.2f} s")
