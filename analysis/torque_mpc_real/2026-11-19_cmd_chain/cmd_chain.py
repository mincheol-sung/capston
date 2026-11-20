"""MPC 순토크 τ* → 모터 명령 τ_cmd → 모터가 잰 토크 τ_meas, 단계마다 차이 (실차 bag, 무장 주행 구간). 2026-11-19
① τ_cmd − τ* : torque_output 이 더한 몫. 설계식 0.9·[C_i tanh(ω/0.5) + V_i ω] (운동 마찰 보상) 로 얼마나 설명되나, 나머지는 정지 마찰 보상·클램프
② τ_meas − τ_cmd : 모터 쪽. /wheel_states torque (모터 전류로 잰 값) 와 τ_cmd 를 지연 L (0~60 ms 중 상관 최대) 로 맞춰 기울기·RMS·|ω| 구간별
   /torque_mpc/cmd_raw 와 /wheel_torque_cmd 는 header.stamp 가 같다 (torque_output·게이트가 stamp 를 안 바꾼다)."""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
C = np.array([0.0174, 0.0206, 0.0155, 0.0157]); V = np.array([0.001066, 0.001158, 0.001112, 0.001177]); SC, WS = 0.9, 0.5
W = ("FL", "FR", "RL", "RR")
for path in sys.argv[1:]:
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    T = ["/torque_mpc/cmd_raw", "/wheel_torque_cmd", "/wheel_states"]; rd.set_filter(rosbag2_py.StorageFilter(topics=[t for t in T if t in types]))
    raw, cmd, ws = {}, [], []
    while rd.has_next():
        tp, d, tr = rd.read_next(); m = deserialize_message(d, get_message(types[tp])); tr *= 1e-9
        st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if tp == T[0] and m.mode == 0: raw[st] = np.array(m.torque)
        elif tp == T[1] and m.mode == 0: cmd.append((st, tr, np.array(m.torque)))
        elif tp == T[2]: ws.append([tr, *m.velocity, *m.torque])
    ws = np.array(ws); pr = [(tr, raw[st], tc) for st, tr, tc in cmd if st in raw]
    if len(pr) < 200: print(path, f"짝 {len(pr)} — 건너뜀"); continue
    tt = np.array([p[0] for p in pr]); ts = np.array([p[1] for p in pr]); tc = np.array([p[2] for p in pr])
    iw = np.clip(np.searchsorted(ws[:, 0], tt) - 1, 0, len(ws) - 1); om = ws[iw, 1:5]
    add = tc - ts; fr = SC * (C * np.tanh(om / WS) + V * om); rest = add - fr
    print(f"== {path.split('/')[-1]}  짝 {len(pr)} ({tt[-1]-tt[0]:.1f} s)  |τ*| RMS {np.sqrt(np.mean(ts**2)):.4f}  |τ_cmd| RMS {np.sqrt(np.mean(tc**2)):.4f}")
    print(f"   ① τ_cmd − τ*  RMS {np.round(np.sqrt(np.mean(add**2,0)),4)}  평균|·| {np.round(np.mean(np.abs(add),0),4)}  "
          f"운동 마찰 보상식 뺀 나머지 RMS {np.round(np.sqrt(np.mean(rest**2,0)),4)}  최대 {np.round(np.abs(rest).max(0),4)}")
    # ② 모터: 명령(수신 시각) → 측정. 2 ms 격자에 올려 지연 찾기
    g = np.arange(tt[0], tt[-1], 0.002)
    out = []
    for i, w in enumerate(W):
        c = np.interp(g, tt, tc[:, i]); best = None
        for L in range(0, 31):
            mm = np.interp(g + 0.002 * L, ws[:, 0], ws[:, 5 + i])
            r = np.corrcoef(c, mm)[0, 1]
            if best is None or r > best[0]: best = (r, L, mm)
        r, L, mm = best; A_ = np.vstack([c, np.ones_like(c)]).T; k, b = np.linalg.lstsq(A_, mm, rcond=None)[0]
        e = mm - c; wv = np.abs(np.interp(g, ws[:, 0], ws[:, 1 + i]))
        bins = [(0, 10), (10, 25), (25, 35), (35, 99)]
        by = " ".join(f"{lo}-{hi}:{np.mean(e[(wv>=lo)&(wv<hi)]):+.3f}/{np.sqrt(np.mean(e[(wv>=lo)&(wv<hi)]**2)):.3f}({100*np.mean((wv>=lo)&(wv<hi)):.0f}%)"
                      if np.any((wv >= lo) & (wv < hi)) else f"{lo}-{hi}:-" for lo, hi in bins)
        out.append(f"   ② {w}: 지연 {2*L} ms, 상관 {r:.3f}, τ_meas ≈ {k:.3f}·τ_cmd {b:+.4f}, 차 RMS {np.sqrt(np.mean(e**2)):.4f} | |ω| 구간 평균/RMS(시간%) {by}")
    print("\n".join(out))
