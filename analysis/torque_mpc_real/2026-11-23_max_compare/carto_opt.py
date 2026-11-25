#!/usr/bin/python3
"""Cartographer 전역 최적화 시각 찾기: /submap_list 에서 같은 (trajectory, submap_index) 서브맵의 포즈가 바뀐 메시지 = 그 사이에 최적화가 돌았다.
 지금 (가장 큰) trajectory 의 서브맵 포즈 변화 = local_to_map 보정량 (서브맵은 로컬 프레임에서 고정, 최적화가 map 에서 옮긴다).
 /tracked_pose 점프 (SLAM − 바퀴·자이로 추측항법, slam_jumps 와 같은 식) 시각과 맞춘다. 시각은 기준 출발 기준."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from load import load, ref_at
B = "/home/smoc/capston/capston/bags/2026-11-23"
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
for bag in sys.argv[1:]:
    d = load(f"{B}/{bag}"); D = d["D"]; a0, a1 = d["arm"]
    ts = np.arange(a0, min(a1, a0 + 60), 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts])
    tr0, tr1 = ts[np.argmax(sp > 0.02)], ts[len(sp) - 1 - np.argmax(sp[::-1] > 0.02)]
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/submap_list"]))
    prev = {}; events = []
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp]))
        tmsg = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 if m.header.stamp.sec else tb * 1e-9
        trajs = [e.trajectory_id for e in m.submap]; tmax = max(trajs) if trajs else -1
        moved = []
        for e in m.submap:
            key = (e.trajectory_id, e.submap_index); p = e.pose
            cur = np.array([p.position.x, p.position.y, yaw(p.orientation)])
            if key in prev and e.trajectory_id == tmax:
                dd = cur - prev[key]; moved.append((math.hypot(dd[0], dd[1]), math.degrees(dd[2]), e.submap_index))
            prev[key] = cur
        if moved and max(x[0] for x in moved) > 1e-4:
            mx = max(moved, key=lambda x: x[0]); events.append((tb * 1e-9 - tr0, mx[0], mx[1], len(moved)))
    sel = [e for e in events if -3.0 < e[0] < tr1 - tr0 + 3.0]
    print(f"{bag}: 최적화 (서브맵 포즈 바뀜) 전체 {len(events)} 번, 주행 ±3 s 안: " +
          ", ".join(f"t {e[0]:+.2f} s {e[1]*1e3:.0f} mm {e[2]:+.1f}°" for e in sel))
    gaps = np.diff([e[0] for e in events]); print(f"   최적화 간격 중앙 {np.median(gaps):.1f} s (최소 {gaps.min():.1f}, 최대 {gaps.max():.1f})" if len(gaps) else "")
