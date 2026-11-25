#!/usr/bin/python3
"""bag 시작 기준 타임라인: /initialpose (측위 맞추기), 새 trajectory 의 첫 서브맵, 최적화 (서브맵 포즈 바뀜 = 보정량), 무장, 기준 출발.
 노드 수는 /submap_list 의 num_range_data (서브맵마다 들어간 노드 수) 합으로 센다."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from load import load, ref_at
B = "/home/smoc/capston/capston/bags/2026-11-23"
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
for bag in sys.argv[1:]:
    d = load(f"{B}/{bag}"); D = d["D"]; a0 = d["arm"][0]
    ts = np.arange(a0, a0 + 30, 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts]); tr0 = ts[np.argmax(sp > 0.02)]
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/submap_list", "/initialpose", "/rosout"])); t_bag0 = None; ev = []; prev = {}; seen_traj = set()
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp])); tb *= 1e-9
        t_bag0 = t_bag0 or tb
        if tp == "/initialpose":
            ev.append((tb, f"/initialpose ({m.pose.pose.position.x:.3f}, {m.pose.pose.position.y:.3f})"))
        elif tp == "/rosout":
            if m.name == "check_localization" and ("맞음" in m.msg or "겹침" in m.msg or "%" in m.msg):
                ev.append((tb, "check_localization: " + m.msg.replace("\n", " ")[:120]))
        else:
            trajs = sorted({e.trajectory_id for e in m.submap}); tmax = trajs[-1] if trajs else -1
            if tmax not in seen_traj:
                seen_traj.add(tmax); ev.append((tb, f"trajectory {tmax} 첫 등장 (서브맵 {sum(1 for e in m.submap if e.trajectory_id == tmax)} 개)"))
            moved = 0.0; nodes = sum(e.num_range_data for e in m.submap if e.trajectory_id == tmax) if hasattr(m.submap[0], "num_range_data") else -1
            for e in m.submap:
                key = (e.trajectory_id, e.submap_index); p = e.pose; cur = np.array([p.position.x, p.position.y])
                if key in prev and e.trajectory_id == tmax:
                    moved = max(moved, float(np.linalg.norm(cur - prev[key])))
                prev[key] = cur
            if moved > 1e-4:
                ev.append((tb, f"최적화: 서브맵 {moved*1e3:.0f} mm 옮김 (trajectory {tmax} 노드 누적 {nodes})"))
    ev.append((a0, "무장")); ev.append((tr0, "기준 출발"))
    print(f"===== {bag}")
    for t, s in sorted(ev):
        print(f"  {t - t_bag0:6.2f} s (출발 {t - tr0:+6.2f})  {s}")
