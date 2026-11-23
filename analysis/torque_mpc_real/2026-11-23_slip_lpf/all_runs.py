#!/usr/bin/python3
"""11-21~23 실차 판 전체: slip_logger λ (추정 속도 기준) 원신호 vs 10 Hz LPF (1 차 filtfilt, 위상 0) 슬립 지표 (2026-11-23).
창: 제어기 추종 중 (MPC hold 0 / PP mode ≠ 2) 이고 추정 속도 > 0.05 m/s.
지표: max_i|λ̃_i| > 0.25 / > 0.5 시간 %, λ̃ RMS, max|λ̃|, > 0.5 중 헛돎 (λ > 0) 몫. 판별 조건은 run_info/args.txt.
  nice -n 19 ionice -c3 /usr/bin/python3 analysis/torque_mpc_real/2026-11-23_slip_lpf/all_runs.py   → runs.csv, summary.md"""
import csv, glob, os, re
import numpy as np, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__))
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
FC = 10.0
KEYS = ["controller", "limit_mode", "shape", "speed", "accel", "lap_speeds", "speed_mod_amp", "speed_mod_shape", "square_v_max", "square_corner_mode",
        "seg_dir_deg", "pp_lookahead_min", "pp_lookahead_max", "slip_guard", "disturbance", "velocity_source", "est_gains", "use_w", "stiction_comp",
        "mpc_com_height", "path_follow_lead", "width", "yaw_mode"]
rows = []
for meta in sorted(glob.glob("bags/2026-11-2[123]/*/metadata.yaml")):
    d = os.path.dirname(meta)
    try:
        m = yaml.safe_load(open(meta))["rosbag2_bagfile_information"]
        line = open(os.path.join(d, "run_info", "args.txt")).read().splitlines()[3]
    except Exception:
        continue
    c = {t["topic_metadata"]["name"]: t["message_count"] for t in m["topics_with_message_count"]}
    if c.get("/slip_logger/slip", 0) < 200 or (c.get("/torque_mpc/status", 0) < 100 and c.get("/pure_pursuit/status", 0) < 100):
        continue
    args = dict(re.findall(r"(\w+):=(\S+)", line))
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=d, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/slip_logger/slip", "/state_estimator/odom", "/torque_mpc/status", "/pure_pursuit/status"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, dd, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(dd, ty[n])))
    pp = len(D["/pure_pursuit/status"]) > len(D["/torque_mpc/status"])
    if pp:
        act = np.array([t for t, mm in D["/pure_pursuit/status"] if len(mm.data) > 6 and mm.data[6] != 2])
    else:
        act = np.array([t for t, mm in D["/torque_mpc/status"] if not (len(mm.data) > 20 and mm.data[20] == 1)])
    if len(act) < 100 or not D["/state_estimator/odom"]:
        continue
    S = np.array([[t, *mm.data[1:5]] for t, mm in D["/slip_logger/slip"] if len(mm.data) >= 5])
    E = np.array([[st(mm.header), np.hypot(mm.twist.twist.linear.x, mm.twist.twist.linear.y)] for _, mm in D["/state_estimator/odom"]])
    fs = 1 / np.median(np.diff(S[:, 0]))
    L = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0)
    bb, aa = butter(1, FC / (fs / 2)); Lf = filtfilt(bb, aa, L, axis=0)
    win = (S[:, 0] >= act[0]) & (S[:, 0] <= act[-1]) & (np.interp(S[:, 0], E[:, 0], E[:, 1]) > 0.05)
    if win.sum() < 200:
        continue
    def met(X):
        a = np.abs(X[win]); mx = a.max(1); big = mx > 0.5
        sg = np.sign(X[win][np.arange(win.sum()), a.argmax(1)])
        return 100 * np.mean(mx > 0.25), 100 * big.mean(), float(np.sqrt(np.mean(X[win] ** 2))), float(mx.max()), (100 * (sg[big] > 0).mean() if big.any() else np.nan)
    raw, lp = met(L), met(Lf)
    lim = args.get("limit_mode", "")
    ctrl = "PP" if (pp or args.get("controller") == "pure_pursuit") else ("C" if lim == "friction" else "B")
    if ctrl == "PP":
        lo, hi = args.get("pp_lookahead_min"), args.get("pp_lookahead_max")
        la = f"비례 {lo}~{hi}" if lo and hi and lo != hi else (f"고정 {lo}" if lo else "고정 0.30(기본)")
    else:
        la = ""
    scen = args.get("shape", "?")
    if scen == "circles8":
        scen += f" v{args.get('speed', args.get('lap_speeds', '?'))}" + (f" 변조{args.get('speed_mod_amp')}{args.get('speed_mod_shape','sin')[:2]}" if args.get("speed_mod_amp") else "")
    elif scen == "square":
        scen += f" a{args.get('accel', '?')} vmax{args.get('square_v_max', '?')}"
    elif scen == "line":
        scen += f" dir{args.get('seg_dir_deg', '0')} v{args.get('speed', '?')} a{args.get('accel', '?')}"
    flags = ",".join(k for k in ("slip_guard", "disturbance") if args.get(k) == "true") + (",wheels" if args.get("velocity_source") == "wheels" else "") + \
            (f",h{args['mpc_com_height']}" if "mpc_com_height" in args else "") + (f",pfl{args['path_follow_lead']}" if "path_follow_lead" in args else "")
    rows.append(dict(bag=d.replace("bags/", ""), ctrl=ctrl, lookahead=la, scen=scen, flags=flags.strip(","), T=round(win.sum() / fs, 1),
                     raw25=round(raw[0], 1), raw50=round(raw[1], 1), lp25=round(lp[0], 1), lp50=round(lp[1], 1),
                     lp_rms=round(lp[2], 3), lp_max=round(lp[3], 2), lp_spin=round(lp[4], 0) if lp[4] == lp[4] else ""))
    print(rows[-1]["bag"], ctrl, la, scen, rows[-1]["flags"], rows[-1]["lp25"], rows[-1]["lp50"], flush=True)
with open(os.path.join(H, "runs.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
# 묶음 요약: (시나리오, 제어기, 룩어헤드, 플래그) → 판 수, 중앙값
groups = {}
for r_ in rows:
    groups.setdefault((r_["scen"], r_["ctrl"], r_["lookahead"], r_["flags"]), []).append(r_)
with open(os.path.join(H, "summary.md"), "w") as f:
    f.write("| 시나리오 | 제어기 | PP 룩어헤드 | 플래그 | 판 | 10 Hz > 0.25 % | 10 Hz > 0.5 % | 원신호 > 0.5 % | λ̃ RMS | λ̃ 최대 | 헛돎 몫 % |\n|---|---|---|---|---|---|---|---|---|---|---|\n")
    for k in sorted(groups, key=lambda k: (k[0], k[1], k[2], k[3])):
        g = groups[k]; med = lambda key: np.median([x[key] for x in g if x[key] != ""]) if any(x[key] != "" for x in g) else float("nan")
        f.write(f"| {k[0]} | {k[1]} | {k[2]} | {k[3] or '-'} | {len(g)} | {med('lp25'):.1f} | {med('lp50'):.1f} | {med('raw50'):.1f} | {med('lp_rms'):.3f} | {med('lp_max'):.2f} | {med('lp_spin'):.0f} |\n")
print(f"\n판 {len(rows)} 개 → runs.csv, summary.md")
