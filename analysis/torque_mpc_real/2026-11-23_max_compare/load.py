#!/usr/bin/python3
"""실차 bag 한 판 → 시계열 dict (무장 구간). SLAM 은 IMU 점 → 로봇 중심 (compare_slam.imu_to_center)."""
import importlib.util, math
import numpy as np, yaml
WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]; OFF = (est["imu_offset_x"], est["imu_offset_y"])
R_W, K = 0.04, 0.1932
TINV = np.array([[1, -1, -K], [1, 1, K], [1, 1, -K], [1, -1, K]]) / R_W      # 몸체 [vx, vy, ω] → 바퀴 ω (FL FR RL RR)


def load(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    D = cs.load(bag)                                                       # ref, pose, armed, tau, wheel
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    want = ["/torque_mpc/status", "/state_estimator/odom", "/imu/base_link", "/wheel_states", "/slip_logger/slip", "/pure_pursuit/status",
            "/pure_pursuit/cmd_vel", "/wheel_pi/status"]
    rd.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    S, E, G, W, L, PS, PC = [], [], [], [], [], [], []
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp]))
        if tp == "/torque_mpc/status": S.append(list(m.data))
        elif tp == "/state_estimator/odom":
            E.append([st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation),
                      m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z])
        elif tp == "/imu/base_link": G.append([st(m.header), m.angular_velocity.z])
        elif tp == "/wheel_states": W.append([st(m.header), *m.velocity, *m.torque])
        elif tp == "/slip_logger/slip": L.append(list(m.data[:9]))
        elif tp == "/pure_pursuit/status": PS.append(list(m.data))
        elif tp == "/pure_pursuit/cmd_vel": PC.append([tb * 1e-9, m.linear.x, m.angular.z] if hasattr(m, "linear") else [tb * 1e-9, m.twist.linear.x, m.twist.angular.z])
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], OFF)
    out = dict(D=D, slam=np.stack([P[:, 0], xc, yc, P[:, 3]], 1), S=np.array([s for s in S if len(s) == 26]), E=np.array(E), G=np.array(G),
               W=np.array(W), L=np.array(L), PS=PS, PC=np.array(PC))
    on = [t for t, v in D["armed"] if v]; off = [t for t, v in D["armed"] if not v and on and t > on[0]]
    out["arm"] = (on[0], off[0] if off else 1e18)
    return out


def ref_at(D, t):
    rt = np.array([a for a, _ in D["ref"]]); i = max(0, np.searchsorted(rt, t, side="right") - 1)
    return D["ref"][i][1].at(t)
