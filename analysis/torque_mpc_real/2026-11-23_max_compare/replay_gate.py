#!/usr/bin/python3
"""check_localization.optimization_moved 를 bag 의 /submap_list 로 시험: /initialpose 직전 가장 큰 trajectory id 를 잡고, 그 뒤 메시지를
도구와 같은 규칙 (traj_min = 그 id + 1, 1e-4 m 넘게 옮겨지면 끝) 으로 넣어 '기다림이 끝났을 시각' 과 무장·출발 시각을 비교."""
import importlib.util, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
spec = importlib.util.spec_from_file_location("cl", "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools/check_localization.py")
cl = importlib.util.module_from_spec(spec); spec.loader.exec_module(cl)
B = "/home/smoc/capston/capston/bags/2026-11-23"
print("| bag | /initialpose | 기다림 끝 (첫 최적화) | 보정 | 실제 무장 | 기준 출발 |")
for bag in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/submap_list", "/initialpose", "/mpc/armed"])); t0 = None
    last_ids, t_init, traj_before, prev, done, arm = [], None, None, {}, None, None
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp])); tb *= 1e-9; t0 = t0 or tb
        if tp == "/initialpose" and t_init is None:
            t_init = tb; traj_before = max(last_ids) if last_ids else None
        elif tp == "/mpc/armed" and m.data and arm is None:
            arm = tb
        elif tp == "/submap_list":
            if t_init is None:
                last_ids = [e.trajectory_id for e in m.submap]
            elif done is None:
                mv = cl.optimization_moved(prev, m, traj_before + 1)
                if mv > 1e-4:
                    done = (tb, mv)
    print(f"| {bag} | {t_init - t0:.1f} s | {done[0] - t0:.1f} s (+{done[0] - t_init:.1f}) | {done[1] * 1e3:.0f} mm | {arm - t0:.1f} s | {arm - t0 + 3.5:.1f} s 쯤 |")
