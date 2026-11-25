# 시뮬 bag: 참값 슬립 (바퀴 ω vs 참값 몸체 속도의 기구학 T⁻¹·v) 바퀴별, 가드 판정·g
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
Tinv = np.array([[25,-25,-4.83],[25,25,4.83],[25,25,-4.83],[25,-25,4.83]], float)
def read(b, want):
    import glob, os
    u = b if os.path.exists(os.path.join(b, "metadata.yaml")) else glob.glob(os.path.join(b, "*.mcap"))[0]
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=u, storage_id="" if u == b else "mcap"), rosbag2_py.ConverterOptions("cdr","cdr"))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(ty))); out = {k: [] for k in want}
    while r.has_next():
        n, d, t = r.read_next(); out[n].append((t*1e-9, deserialize_message(d, ty[n])))
    return out
for b in sys.argv[1:]:
    raw = read(b, ("/wheel_states","/sim/ground_truth","/torque_output/slip_guard"))
    G = raw["/sim/ground_truth"]; m0 = G[0][1]
    if hasattr(m0, "twist"):
        V = np.array([[t, m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for t,m in G])
    W = np.array([[t,*m.velocity] for t,m in raw["/wheel_states"]])
    vb = np.array([np.interp(W[:,0], V[:,0], V[:,1+k]) for k in range(3)])
    wk = (Tinv @ vb).T                                  # 참값 몸체 → 바퀴 기구학 속도
    mv = np.abs(wk).max(1) > 2.0
    lam = (W[mv,1:] - wk[mv]) / np.maximum(np.maximum(np.abs(W[mv,1:]), np.abs(wk[mv])), 5.0)
    line = f"{b.split('/')[-1]}: 참값 |λ| 최대 [FL FR RL RR] {np.abs(lam).max(0).round(2)} · |λ|>0.2 시간 % {(100*(np.abs(lam)>0.2).mean(0)).round(1)} · 몸체 최고 {np.abs(V[:,1]).max():.2f} m/s"
    if raw["/torque_output/slip_guard"]:
        Q = np.array([m.data for t,m in raw["/torque_output/slip_guard"]])
        fl = Q[:,9]
        line += f" | 가드 판정 % [FL FR RL RR] {[round(100*float((fl==i).mean()),1) for i in range(4)]} · g 최소 {Q[:,5:9].min(0).round(2)}"
    print(line)
