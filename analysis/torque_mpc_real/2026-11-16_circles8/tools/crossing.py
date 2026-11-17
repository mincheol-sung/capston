"""circles8 tangent: 교차점(기준 ω 부호 바뀜) 주변 ±0.6 s 와 나머지 구간의 추종 오차 비교, 오차 최대 시각."""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
T = ["/dra_mppi/reference_trajectory", "/torque_mpc/status", "/mpc/armed"]
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=T))
    R, S, A = [], [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        if tp == T[0] and m.references: q = m.references[0]; R.append([tr, q.vx, q.yaw_rate])
        elif tp == T[1]: S.append([tr, m.data[1], m.data[2], m.data[3], *m.data[4:8], m.data[8], m.data[20] if len(m.data) > 20 else 0])
        elif tp == T[2]: A.append((tr, m.data))
    R, S = np.array(R), np.array(S)
    arm = [a for a, v in A if v][0]; R = R[R[:, 0] > arm]
    mov = R[np.abs(R[:, 1]) > 1e-3]; t0, t1 = mov[0, 0], mov[-1, 0]
    w = R[:, 2]; flips = R[1:, 0][(np.sign(w[1:]) != np.sign(w[:-1])) & (np.abs(w[1:]) > 0.2) & (np.abs(w[:-1]) > 0.2)]
    S = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]
    e = np.hypot(S[:, 1], S[:, 2]); eth = np.degrees(np.abs(S[:, 3]))
    near = np.zeros(len(S), bool)
    for f in flips: near |= np.abs(S[:, 0] - f) < 0.6
    sat = np.max(np.abs(S[:, 4:8]), axis=1) >= 0.98 * S[:, 8]
    i = np.argmax(e)
    print(f"== {path.split('/')[-1]}: 주행 {t1-t0:.1f} s, 교차점 {len(flips)} 개 @ " + ", ".join(f"{f-t0:.1f}" for f in flips) + " s")
    print(f"   위치 오차 최대 {100*e.max():.1f} cm @ {S[i,0]-t0:.1f} s ({'교차점 ±0.6 s 안' if near[i] else '교차점 밖'}) | 헤딩 오차 최대 {eth.max():.1f}° @ {S[np.argmax(eth),0]-t0:.1f} s")
    for name, msk in (("교차점 ±0.6 s", near), ("나머지", ~near)):
        print(f"   {name:12s}: 위치 RMS {100*np.sqrt(np.mean(e[msk]**2)):.1f} cm, 최대 {100*e[msk].max():.1f} cm | 헤딩 RMS {np.sqrt(np.mean(eth[msk]**2)):.1f}°, 최대 {eth[msk].max():.1f}° | 상한 닿음 {100*sat[msk].mean():.0f} % | 시간 {msk.mean()*100:.0f} %")
    # 출발·끝 (가감속) 구간
    for name, msk in (("출발 2 s", S[:, 0] - t0 < 2.0), ("끝 2 s", t1 - S[:, 0] < 2.0)):
        print(f"   {name:12s}: 위치 최대 {100*e[msk].max():.1f} cm, 헤딩 최대 {eth[msk].max():.1f}°")
