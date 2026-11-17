import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool
KEEP = ("fault_monitor", "figure8", "torque_mpc_node", "torque_output", "torque_command_gate")
for path in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/rosout", "/mpc/armed"]))
    t0 = None; prev = None; print("=====", path.split('/')[-1])
    while r.has_next():
        tp, d, t = r.read_next(); t0 = t0 or t; rel = (t - t0) / 1e9
        if tp == "/mpc/armed":
            m = deserialize_message(d, Bool); line = f"ARMED={m.data}"
        else:
            m = deserialize_message(d, Log)
            if not any(k in m.name for k in KEEP): continue
            if "상태 추정값 대기" in m.msg or "기준열" in m.msg and "없음" in m.msg: continue
            txt = m.msg.replace("\n", " | ")
            if "토크 MPC (C++" in txt: txt = txt[txt.find("m 4.56"):][:200]
            elif "torque_output:" in txt: txt = "정지 마찰 " + txt[txt.find("정지 마찰") + 6:][:60]
            else: txt = txt[:170]
            line = f"[{m.level}] {m.name}: {txt}"
        if line != prev: print(f"{rel:7.2f}  {line}")
        prev = line
