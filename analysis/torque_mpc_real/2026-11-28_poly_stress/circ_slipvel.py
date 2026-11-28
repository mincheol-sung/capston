"""원 8 자 · 꺾은선에서 세 제어기 미끄럼 속도 비교 — 슬립률 % 는 바퀴 속도 (분모) 에 따라 부풀거나 줄어 원끼리·제어기끼리 비교가 기운다.
미끄럼 속도 = r · (ω − ω_nom)  [m/s], r 0.04 m. ω = /wheel_states velocity, ω_nom = /slip_logger/slip 6~9 열 (추정기 몸체 속도가 시키는 무슬립 바퀴 속도),
두 토픽은 slip 첫 열 (= wheel_states header 시각) 로 맞춘다. 10 Hz 1차 Butterworth filtfilt. 창 = 무장 중 · 기준 속력 > 0.05 m/s.
지표: 네 바퀴 중 최대 |미끄럼 속도| 의 RMS, 그것이 > 0.10 / > 0.20 m/s 인 시간 %, 큰 미끄럼 (> 0.10) 중 헛돎 (바퀴가 빠름) 몫.
※ 추정기가 바퀴보다 ~40 ms 늦은 몫이 세 제어기에 똑같이 들어간다 (11-27 분해).
source install/setup.bash 뒤 /usr/bin/python3 circ_slipvel.py → 표 출력, circ_slipvel_out.txt"""
import os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
B = "/home/smoc/capston/capston/bags/"
R_W = 0.04
WANT = ["/wheel_states", "/slip_logger/slip", "/mpc/armed", "/dra_mppi/reference_trajectory"]
G = [
    ("원 11-26 R0.50 · 1.0 blend0.6 (요구 109 %)", "2026-11-26",
     {"MPC": ["184758_s2_mpc_r1", "184937_s2_mpc_r2", "185209_s2_mpc_r3", "185937_s2_mpc_r4", "190248_s2_mpc_r5", "190352_s2_mpc_r6"],
      "PP": ["185321_s2_pp_r1", "185437_s2_pp_r2", "185532_s2_pp_r3"], "ADPP": ["185636_s2_adpp_r1", "185736_s2_adpp_r2", "185834_s2_adpp_r3"]}),
    ("원 11-27 R0.50 · 1.0 blend0 · MPC 되먹임 1.0 (요구 109 %)", "2026-11-27",
     {"MPC": ["192519_yarr_mpc", "193234_yarr_mpc", "193851_yarr_mpc"], "MPC 되먹임 끔": ["165139_sfbw_lqr025_r3"],
      "PP": ["165935_pp_r1", "173001_pp_r1"], "ADPP": ["173512_pp_r1"]}),
    ("원 11-27 R0.50 · 0.7±50 % (요구 최고 120 %)", "2026-11-27", {"MPC": ["174149_circle1"], "PP": ["174419_pp_r1"]}),
    ("원 11-22 R0.50 · 0.6±50 % (요구 최고 88 %)", "2026-11-22",
     {"MPC": ["002707", "002931", "003146", "212703"], "PP": ["002822", "003041", "003451", "212837"]}),
    ("원 11-23 R0.75 · 0.8±50 % (요구 최고 105 %)", "2026-11-23", {"MPC": ["120345", "144243"], "PP": ["120544"]}),
    ("꺾은선 11-26 1.5/1.5 통과", "2026-11-26",
     {"MPC": ["201259_s2pl_mpc_r1", "201604_s2pl_mpc_r2", "201944_s2pl_mpc_r3"], "PP": ["201404_s2pl_pp_r1", "201704_s2pl_pp_r2", "202049_s2pl_pp_r3"],
      "ADPP": ["201501_s2pl_adpp_r1", "201804_s2pl_adpp_r2", "202150_s2pl_adpp_r3"]}),
]


def find(day, name):
    for d in os.listdir(B + day):
        if d.startswith(name):
            return os.path.join(B + day, d)


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def one(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    have = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in WANT if t in have]))
    ty = {n: get_message(have[n]) for n in WANT if n in have}
    ws, sl, arm, ref = {}, [], [], []
    while r.has_next():
        n, data, ns = r.read_next(); m = deserialize_message(data, ty[n])
        if n == "/wheel_states":
            ws[round(stamp(m.header) * 1e6)] = list(m.velocity)
        elif n == "/slip_logger/slip":
            sl.append(list(m.data[:9]))
        elif n == "/mpc/armed":
            arm.append((ns * 1e-9, m.data))
        elif len(m.references):
            p = m.references[0]; ref.append((stamp(m.header), float(np.hypot(p.vx, p.vy))))
    S = np.array(sl); ref = np.array(ref)
    t_on = next(t for t, v in arm if v); t_off = next((t for t, v in arm if not v and t > t_on), S[-1, 0])
    W = np.array([ws.get(round(t * 1e6), [np.nan] * 4) for t in S[:, 0]])
    ok = np.isfinite(W).all(1) & np.isfinite(S[:, 5:9]).all(1)
    t, e = S[ok, 0], R_W * (W[ok] - S[ok, 5:9])
    sg = np.sign(S[ok, 5:9])
    e = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(t)))), e, axis=0)
    win = (t >= t_on) & (t <= t_off) & (np.interp(t, ref[:, 0], ref[:, 1]) > 0.05)
    a = np.abs(e[win]); i = a.argmax(1); mx = a.max(1)
    spin = (e[win] * sg[win])[np.arange(len(i)), i] > 0
    big = mx > 0.10
    return dict(rms=100 * float(np.sqrt(np.mean(mx ** 2))), p10=100 * float(big.mean()), p20=100 * float((mx > 0.20).mean()),
                spin=100 * float(spin[big].mean()) if big.any() else np.nan, n=int(win.sum()), match=float(ok.mean()))


out = open(os.path.join(HERE, "circ_slipvel_out.txt"), "w")
print("| 묶음 | 제어기 | 판 | 최대 바퀴 미끄럼 속도 RMS cm/s | > 0.10 m/s 시간 % | > 0.20 m/s 시간 % | 큰 미끄럼 중 헛돎 % |")
print("|---|---|---|---|---|---|---|")
for g, day, cs in G:
    for c, names in cs.items():
        rows = []
        for n in names:
            p = find(day, n)
            try:
                r = one(p); rows.append(r)
                print(f"[{g}·{c}] {os.path.basename(p)}: RMS {r['rms']:.1f} · >0.1 {r['p10']:.1f} · >0.2 {r['p20']:.1f} · 헛돎 {r['spin']:.0f} · 맞춘 비율 {r['match']:.3f}", file=out)
            except Exception as ex:
                print(f"[{g}·{c}] {n}: 실패 {type(ex).__name__} {ex}", file=out)
        if rows:
            f = lambda k: (lambda v: f"{v.mean():.1f}" + (f" ({v.min():.1f}~{v.max():.1f})" if len(v) > 1 else ""))(np.array([r[k] for r in rows]))
            print(f"| {g} | {c} | {len(rows)} | {f('rms')} | {f('p10')} | {f('p20')} | {f('spin')} |")
out.close()
