"""추정기 포즈의 한 주기 튐: |Δp − v·Δt| > 1 cm 인 주기 수와 합 (실제 주행으로는 10 ms 에 0.7 cm 밖에 못 간다). 2026-11-17"""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/state_estimator/odom", "/dra_mppi/reference_trajectory"]))
    O, R = [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp]))
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if tp == "/state_estimator/odom":
            th = 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w); vx, vy = m.twist.twist.linear.x, m.twist.twist.linear.y
            O.append([st, m.pose.pose.position.x, m.pose.pose.position.y, math.cos(th) * vx - math.sin(th) * vy, math.sin(th) * vx + math.cos(th) * vy])
        elif m.references and abs(m.references[0].vx) > 1e-3: R.append(st + m.dt)
    O = np.array(O); t0, t1 = min(R), max(R); O = O[(O[:, 0] >= t0) & (O[:, 0] <= t1)]
    dp = np.diff(O[:, 1:3], axis=0) - O[:-1, 3:5] * np.diff(O[:, 0])[:, None]; j = np.hypot(dp[:, 0], dp[:, 1])
    big = j > 0.01
    print(f"{path.split('/')[-1]}: 주행 {t1-t0:.1f} s, 1 cm 넘는 한 주기 튐 {big.sum()} 번 (합 {100*j[big].sum():.0f} cm, 최대 {100*j.max():.1f} cm)")
