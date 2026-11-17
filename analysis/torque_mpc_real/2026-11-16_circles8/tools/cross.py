import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r_w, k = 0.04, (0.1897 + 0.207) / 2
Tinv = (1 / r_w) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])
T = ["/dra_mppi/reference_trajectory", "/wheel_states", "/mpc/armed", "/wheel_torque_cmd", "/torque_mpc/cmd_raw"]
path = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}; r.set_filter(rosbag2_py.StorageFilter(topics=T))
R, W, A, C, N = [], [], [], [], []
while r.has_next():
    tp, d, tr = r.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
    if tp == T[0] and m.references: q = m.references[0]; R.append([tr, q.vx, q.vy, q.yaw_rate])
    elif tp == T[1]: W.append([tr, *m.velocity])
    elif tp == T[2]: A.append([tr, float(m.data)])
    elif tp == T[3]: C.append([tr, *m.torque])
    elif tp == T[4]: N.append([tr, *m.torque])
R, W, C, N = map(np.array, (R, W, C, N))
arm = [a[0] for a in A if a[1] > 0.5][0]; Rm = R[R[:, 0] > arm]; t0 = Rm[np.hypot(Rm[:, 1], Rm[:, 2]) > 1e-3][0, 0]
at = lambda X, t: X[min(np.searchsorted(X[:, 0], t), len(X) - 1)]
print(f"== {path.split('/')[-1]}   FL (RR 도 비슷)   t: 기준 출발 후 [s]")
print("  t   | ω_need FL | ω_meas FL | τ_net(MPC) FL | τ_motor FL | 정지 마찰 몫 (τ_motor − τ_net − 운동 마찰 근사)")
for t in np.arange(1.4, 2.61, 0.1):
    q = at(R, t0 + t); w = at(W, t0 + t); c = at(C, t0 + t); nn = at(N, t0 + t)
    need = Tinv @ q[1:4]
    kin = 0.9 * 0.02142 * np.tanh(w[1] / 0.5) + 0.9 * 0.002062 * w[1]
    print(f" {t:4.1f} |  {need[0]:+6.2f}   |  {w[1]:+6.2f}   |   {nn[1]:+.4f}     |  {c[1]:+.4f}   | {c[1]-nn[1]-kin:+.4f}")
