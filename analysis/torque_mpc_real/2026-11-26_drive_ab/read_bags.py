# 2026-11-25 롤러 구름 저항 A/B (tools/roller_ab.sh) — ../2026-11-25_polyline_pass_real/read_bags.py 복사, 기준열에 yaw·yaw_rate 열 추가
#   bag 을 읽어 판마다 npz 로 저장한다.   source install/setup.bash 뒤  /usr/bin/python3 read_bags.py bags/<날짜>/*_rr_*
import math, os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
WANT = {"/torque_mpc/status": "st", "/torque_mpc/cap": "cap", "/torque_mpc/cmd_raw": "mpc", "/wheel_torque_cmd": "mot",
        "/wheel_states": "ws", "/state_estimator/odom": "od", "/tracked_pose": "sl", "/mpc/armed": "arm", "/slip_logger/slip": "slip",
        "/dra_mppi/reference_trajectory": "ref", "/imu/base_link": "imu"}

for path in sys.argv[1:]:
    path = path.rstrip("/")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in WANT}
    d = {v: [] for v in WANT.values()}
    t0 = None
    while r.has_next():
        name, data, ns = r.read_next()
        if name not in types:
            continue
        if t0 is None:
            t0 = ns
        t = (ns - t0) * 1e-9
        m = deserialize_message(data, types[name]); k = WANT[name]
        if k in ("st", "cap", "slip"):
            d[k].append([t, *m.data])
        elif k in ("mpc", "mot"):
            d[k].append([t, float(m.mode), *(list(m.torque) if len(m.torque) == 4 else [math.nan] * 4)])
        elif k == "ws":
            d[k].append([t, *m.position, *m.velocity, *m.torque, *m.temperature])
        elif k == "od":
            q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            d[k].append([t, p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z])
        elif k == "sl":
            d[k].append([t, m.pose.position.x, m.pose.position.y])
        elif k == "arm":
            d[k].append([t, float(m.data)])
        elif k == "ref" and m.references:
            p = m.references[0]
            d[k].append([t, p.x, p.y, p.vx, p.vy, p.yaw, p.yaw_rate])      # 2026-11-25 roller_ab: 헤딩·회전 속도 열 추가 (끝에)
        elif k == "imu":
            a, w = m.linear_acceleration, m.angular_velocity
            d[k].append([t, a.x, a.y, a.z, w.z])
    out = os.path.join(HERE, os.path.basename(path) + ".npz")
    np.savez(out, **{k: np.array(v, float) for k, v in d.items() if v})
    print(out, {k: len(v) for k, v in d.items()})
