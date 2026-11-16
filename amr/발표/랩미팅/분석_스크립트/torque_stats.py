import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for b in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    want = ("/wheel_torque_cmd", "/mpc/armed")
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    T, arm = [], None; armed = False
    while r.has_next():
        n, d, t = r.read_next(); m = deserialize_message(d, types[n])
        if n == "/mpc/armed": armed = m.data
        elif armed:
            T.append(list(getattr(m, "torque", getattr(m, "torques", []))))
    T = np.abs(np.array(T))
    print(f"{b.split('/')[-1]}: arm 중 명령 {len(T)} 개, |τ| 중앙값 {np.median(T):.3f}, 95 % {np.percentile(T,95):.3f}, 최대 {T.max():.3f} N·m, |τ|>0.15 비율 {100*np.mean(T>0.15):.1f} %")
