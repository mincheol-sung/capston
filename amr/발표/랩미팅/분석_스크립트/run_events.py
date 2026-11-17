import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
want = ("/mpc/armed", "/fault_monitor/fault", "/rosout")
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
t0 = None
while r.has_next():
    n, d, t = r.read_next(); m = deserialize_message(d, types[n]); t = t * 1e-9
    if n == "/mpc/armed":
        print(f"{t:.1f} armed={m.data}")
        if m.data and t0 is None: t0 = t
    elif n == "/fault_monitor/fault":
        print(f"{t:.1f} FAULT {getattr(m, 'data', m)}")
    elif n == "/rosout" and m.level >= 30 and any(k in m.name for k in ("fault", "figure8", "gate", "torque_mpc")):
        print(f"{t:.1f} [{m.name}] {m.msg[:150]}")
