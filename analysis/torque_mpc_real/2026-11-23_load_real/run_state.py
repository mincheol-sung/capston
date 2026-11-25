#!/usr/bin/python3
"""판별 무장 시점 상태: 모터 온도 (/wheel_states.temperature, 바퀴별), 출발 포즈 (/tracked_pose), 주행 중 바퀴 전류 대신 |τ| 평균.
판 사이 편차가 온도·출발 위치로 흘러가는지 보려는 것 (2026-11-23 h 스윕)."""
import math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B = "/home/smoc/capston/capston/bags/2026-11-23"
for bag in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/wheel_states", "/tracked_pose", "/mpc/armed"]))
    T, P, A = [], [], []
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp])); t = tb * 1e-9
        if tp == "/wheel_states": T.append([t, *m.temperature])
        elif tp == "/tracked_pose":
            q = m.pose.orientation; P.append([t, m.pose.position.x, m.pose.position.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))])
        elif tp == "/mpc/armed": A.append([t, m.data])
    T, P = np.array(T), np.array(P); ta = next(t for t, v in A if v)
    i = np.searchsorted(T[:, 0], ta); j = np.searchsorted(P[:, 0], ta)
    print(f"{bag:>14}  arm 시 모터 온도 {T[i, 1:5].round(0)} 끝 {T[-1, 1:5].round(0)}  출발 포즈 x {P[j,1]:+.3f} y {P[j,2]:+.3f} yaw {P[j,3]:+.1f}°")
