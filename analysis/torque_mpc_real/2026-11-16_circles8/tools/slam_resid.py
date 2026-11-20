"""주행 구간 SLAM 보정 잔차 (추정기 예측 − SLAM 포즈, 보정 때마다): 보정 주기, 잔차 RMS/최대, 포즈 나이. 2026-11-17"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/state_estimator/status", "/dra_mppi/reference_trajectory"]))
    E, R = [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == "/state_estimator/status": E.append([tr, *m.data[:10]])
        elif m.references and abs(m.references[0].vx) > 1e-3: R.append(tr)
    E = np.array(E); t0, t1 = min(R), max(R); Em = E[(E[:, 0] >= t0) & (E[:, 0] <= t1)]
    res = 100 * np.hypot(Em[:, 2], Em[:, 3])
    print(f"{path.split('/')[-1]}: 보정 {len(Em)} 번 / {t1-t0:.1f} s ({len(Em)/(t1-t0):.1f} Hz), 잔차 RMS {np.sqrt(np.mean(res**2)):.1f} cm, 최대 {res.max():.1f} cm, "
          f"ψ 잔차 RMS {np.degrees(np.sqrt(np.mean(Em[:,4]**2))):.2f}°, 포즈 나이 중앙 {1e3*np.median(Em[:,10]):.0f} ms")
