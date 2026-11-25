#!/usr/bin/python3
"""실차 h 스윕 (2026-11-23, 계단 변조 8자 width 1.6 · speed 0.69 · accel 3.0) — 판마다 load_compare.py 와 같은 지표 + 동적 상한 기준 포화.
판 = bags/2026-11-23/<폴더>. 기본: 예전 세 판 (C 끔 151038 · C h0.2 154431 · PP 151249) + 폴더 이름이 _h 로 끝나는 새 판.
  sat_dyn = max_i |τ_i| / τ̄_i(t) ≥ 0.98, τ̄(t) 는 /torque_mpc/load 의 [t, N×4, τ̄×4] (h 0 이면 고정 상한과 같다).
  a_y 구간별 안쪽/바깥 바퀴 |λ| > 0.5 %, N 범위 (안쪽 최소 / 바깥 최대 ÷ 정적).
출력: sweep.json, sweep.npz, 표 (stdout).  사용: sweep_compare.py [라벨:폴더 ...]"""
import glob, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
import compare as cm
B = cm.B; TB = cm.TB


def load_topic(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    if "/torque_mpc/load" not in {t.name for t in rd.get_all_topics_and_types()}:
        return None
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/torque_mpc/load"])); X = []
    while rd.has_next():
        _, data, tb = rd.read_next(); m = list(deserialize_message(data, Float64MultiArray).data); X.append(m[0:9])   # m[0] = 노드 시각 (헤더 시각과 같은 시계)
    return np.array(X) if X else None


def runs_default():
    r = [("C_h0", "151038"), ("C_h0.2", "154431"), ("PP", "151249")]
    for p in sorted(glob.glob(f"{B}/*_h*") + glob.glob(f"{B}/*_pp*")):
        suf = os.path.basename(p).split("_", 1)[1]
        r.append((("PP_" if suf.startswith("pp") else "C_") + suf, os.path.basename(p)))
    return r


RUNS = [tuple(a.split(":", 1)) for a in sys.argv[1:]] or runs_default()
out, store = {}, {}
for lab, bag in RUNS:
    o = cm.series(bag); r = cm.metrics(o, "fig8"); r["bag"] = bag
    mv = o["moving"]; ref = o["ref"]; sp = np.hypot(ref[:, 3], ref[:, 4]); om = ref[:, 5]; ay = sp * np.abs(om)
    lam = o["lam"]; left = om > 0
    inf = np.where(left, lam[:, 0], lam[:, 1]); inr = np.where(left, lam[:, 2], lam[:, 3])
    outf = np.where(left, lam[:, 1], lam[:, 0]); outr = np.where(left, lam[:, 3], lam[:, 2])
    bins = {}
    for lo, hi in ((0.0, 0.6), (0.6, 2.2), (2.2, 3.5)):
        m = mv & (np.abs(om) > 0.05) & (ay >= lo) & (ay < hi)
        pc = lambda x: 100 * float(np.mean(np.abs(x[m]) > 0.5)) if m.any() else float("nan")
        bins[f"{lo}-{hi}"] = dict(share=100 * float(m.sum() / mv.sum()), in_f=pc(inf), in_r=pc(inr), out_f=pc(outf), out_r=pc(outr))
    r["bins"] = bins
    r["big_hi_in"] = bins["2.2-3.5"]["in_f"], bins["2.2-3.5"]["in_r"]
    if "hold" in o:
        r["hold_pct"] = 100 * float(np.mean(o["hold"][mv]))
    # 2026-11-23 path following 비교용: 주행 시간 (기준 활성 구간 = series 의 T) · contouring (참값 대신 SLAM 중심 → 기준 경로 점 집합 최단 거리)
    from scipy.spatial import cKDTree
    u = np.linspace(0, len(ref) - 1, 10 * (len(ref) - 1) + 1)
    Pd = np.stack([np.interp(u, np.arange(len(ref)), ref[:, 0]), np.interp(u, np.arange(len(ref)), ref[:, 1])], 1)
    ec = cKDTree(Pd).query(np.stack([o["x"], o["y"]], 1))[0][mv]
    r["e_cont_rms"], r["e_cont_max"] = 1e3 * float(np.sqrt(np.mean(ec ** 2))), 1e3 * float(ec.max())
    # 18 s 근처 (세 판 공통 최대 오차 자리) 의 최대 오차
    w = (o["t"] > 16) & (o["t"] < 21); r["e_18"] = 1e3 * float(o["e"][w].max()) if w.any() else float("nan")
    Lt = load_topic(f"{B}/{bag}")
    if Lt is not None and "tau_mpc" in o:
        t_abs = o["t"] + o["t0"]                                   # series 의 t 는 기준 출발 기준 → 절대 시각
        tbar = np.stack([np.interp(t_abs, Lt[:, 0], Lt[:, 5 + i]) for i in range(4)], 1)
        n0 = np.median(Lt[:, 1:5], axis=0)
        r["N_ratio_min"] = float((Lt[:, 1:5] / n0).min()); r["N_ratio_max"] = float((Lt[:, 1:5] / n0).max())
        r["tbar_min_frac"] = float((Lt[:, 5:9] / TB).min())
        ok = mv & ~o["hold"]; r["sat_dyn"] = 100 * float(((np.abs(o["tau_mpc"][ok]) / tbar[ok]).max(1) >= 0.98).mean())
    out[lab] = r; store.update({f"{lab}_{k}": v for k, v in o.items() if isinstance(v, np.ndarray)})
json.dump(out, open(os.path.join(HERE, "sweep.json"), "w"), indent=1, ensure_ascii=False)
np.savez(os.path.join(HERE, "sweep.npz"), **store)
print("| 판 | 주행 시간 [s] | contouring RMS / 최대 | 위치 RMS / 최대 | 18 s 최대 | 요 RMS | 큰 슬립 (헛돎/끌림) | λ RMS | 센 선회 안쪽 앞/뒤 | 포화 고정/동적 | hold % | N 최소~최대 ÷ 정적 |")
for lab, r in out.items():
    print(f"| {lab} {r['bag']} | {r['T']:.2f} | {r['e_cont_rms']:.0f} / {r['e_cont_max']:.0f} | {r['e_rms']:.0f} / {r['e_max']:.0f} | {r['e_18']:.0f} | {r['h_rms']:.1f} | {r['big']:.1f} ({r['drive']:.1f}/{r['brake']:.1f}) | "
          f"{r['lam_rms']:.3f} | {r['big_hi_in'][0]:.0f} / {r['big_hi_in'][1]:.0f} | {r.get('sat', float('nan')):.0f} / {r.get('sat_dyn', float('nan')):.0f} | "
          f"{r.get('hold_pct', float('nan')):.1f} | {r.get('N_ratio_min', float('nan')):.2f} ~ {r.get('N_ratio_max', float('nan')):.2f} | fault {r['fault']}")
