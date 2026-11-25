# 시뮬 bag 비교 (참값 /sim/ground_truth 기준, 주행 중 = 기준 속도 > 0.05):
#   경로 거리 RMS / 최대, 교차점 0.15 m 안 경로 거리, 시간 기준 오차 (status), 헤딩, 참값 ω_z 흔들림 (0.5 s 이동평균 뺀 나머지), lag·원 안쪽 평균
import sys, glob, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def read(b, want):
    u = b if os.path.exists(os.path.join(b, "metadata.yaml")) else glob.glob(os.path.join(b, "*.mcap"))[0]
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=u, storage_id="" if u == b else "mcap"), rosbag2_py.ConverterOptions("cdr","cdr"))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(ty))); out = {k: [] for k in want}
    while r.has_next():
        n, d, t = r.read_next(); out[n].append((t*1e-9, deserialize_message(d, ty[n])))
    return out
print("회차 | 경로 거리 RMS / 최대 [mm] | 교차점 RMS / 최대 | 시간 기준 RMS / 최대 | 헤딩 RMS [°] | ω_z 흔들림 [rad/s] | lag 평균 · 원 안쪽 평균 [mm]")
for b in sys.argv[1:]:
    raw = read(b, ("/sim/ground_truth","/figure8/path","/mpc/armed","/dra_mppi/reference_trajectory","/torque_mpc/status","/pure_pursuit/status"))
    s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    R = np.array([[t, np.hypot(x.vx,x.vy), x.yaw, x.yaw_rate] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    mvR = R[:,1] > 0.05; t0, t1 = R[mvR,0][0], R[mvR,0][-1]
    G = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.angular.z] for t,m in raw["/sim/ground_truth"] if t0<=t<=t1])
    path = np.array([[p.pose.position.x,p.pose.position.y] for p in raw["/figure8/path"][-1][1].poses])
    d = np.sqrt(((G[:,None,1:3]-path[None])**2).sum(-1)).min(1)
    near = np.hypot(G[:,1]-path[0,0], G[:,2]-path[0,1]) < 0.15
    st = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    X = np.array([[t,*m.data[1:4]] for t,m in raw[st] if t0<=t<=t1])
    e = np.hypot(X[:,1],X[:,2]); th = np.degrees(np.arctan2(np.sin(X[:,3]),np.cos(X[:,3])))
    yaw = np.interp(X[:,0], R[:,0], np.unwrap(R[:,2])); wr = np.interp(X[:,0], R[:,0], R[:,3])
    lag = np.cos(yaw)*X[:,1]+np.sin(yaw)*X[:,2]; inn = -(-np.sin(yaw)*X[:,1]+np.cos(yaw)*X[:,2])*np.sign(wr)
    dt = np.median(np.diff(G[:,0])); w = int(round(0.5/dt)) | 1
    wz = G[:,3] - np.convolve(G[:,3], np.ones(w)/w, "same"); wz = wz[w:-w]
    r = lambda a: np.sqrt((a**2).mean())
    print(f"{os.path.basename(b).replace('_bag','')} | {1e3*r(d):.1f} / {1e3*d.max():.1f} | {1e3*r(d[near]):.1f} / {1e3*d[near].max():.1f} | {1e3*r(e):.1f} / {1e3*e.max():.1f} | {r(th):.2f} | {r(wz):.3f} | {1e3*lag.mean():+.1f} · {1e3*inn.mean():+.1f}")
