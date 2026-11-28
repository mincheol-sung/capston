# 2026-11-26 구동 방향 보정 MPC 실차 A/B (tools/drive_ab.sh: 끔 = 지금 MPC 기본 (롤러 항 켬), 켬 = drive_yaw_deg 2.4 · drive_slip_coef −2.5), 꺾은선 통과 3 판씩 — ../2026-11-25_roller_ab/summary.py 복사
#   지표: analyze.py (지난 꺾은선 통과 비교와 같은 잣대) + 11-23 회의 슬립 지표 (10 Hz LPF) + 회전 지표
#     회전 속도 비 = 기준 회전 속도 > 0.3 rad/s 인 표본에서 Σ sgn(ω_r)·ω_자이로 / Σ|ω_r|  (자이로는 서 있을 때 평균을 0 점으로)
#     출발 회전 끝 헤딩 오차 = 기준이 처음 움직이기 시작할 때 (|v_r| > 0.02) 추정기 헤딩 − 기준 헤딩 [°]
#   source install/setup.bash 뒤  /usr/bin/python3 read_bags.py bags/<날짜>/*_rr_*  →  /usr/bin/python3 summary.py
import glob, json, os, re, sys
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import analyze as AZ
WS = os.path.realpath(os.path.join(HERE, "..", "..", ".."))
EXCLUDE = {"180604_dy_off_r1", "180802_dy_off_r1", "181021_dy_off_r1"}   # 2026-11-26 CPU 포화 (안 꺼진 시뮬 스택) 판 — 버림
bags = sorted(b for b in glob.glob(os.path.join(WS, "bags", "*", "*_dy_*_r*")) if os.path.isdir(b) and not b.endswith(".tmp") and os.path.basename(b) not in EXCLUDE
              and os.path.exists(os.path.join(HERE, os.path.basename(b) + ".npz")))
RUNS = {"보정 끔": [b for b in bags if "_dy_off_" in b], "2.4° · −2.5 (11-23)": [b for b in bags if "_dy_on_" in b],
        "A: 2.4° · 0": [b for b in bags if "_dy_a_" in b], "B: 3.1° · +0.8": [b for b in bags if "_dy_b_" in b]}
RUNS = {k: v for k, v in RUNS.items() if v}
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi


def extra(lab):
    A = np.load(os.path.join(HERE, lab + ".npz")); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    fs = 1 / np.median(np.diff(t)); bb, aa = butter(1, 10.0 / (fs / 2)); Lf = filtfilt(bb, aa, L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05); mx = np.abs(Lf[w]).max(1)
    out = {"slip25": 100 * np.mean(mx > 0.25), "slip50": 100 * np.mean(mx > 0.5)}
    R, imu = A["ref"], A["imu"]
    if R.shape[1] >= 7:
        ws = A["ws"]; still = np.interp(imu[:, 0], ws[:, 0], np.abs(ws[:, 5:9]).max(1)) < 0.2
        bias = float(imu[still & (imu[:, 0] < t_on), 4].mean()) if (still & (imu[:, 0] < t_on)).sum() > 50 else 0.0
        rr = (R[:, 0] >= t_on) & (R[:, 0] <= t_off) & (np.abs(R[:, 6]) > 0.3)
        g = np.interp(R[rr, 0], imu[:, 0], imu[:, 4]) - bias
        out["rot_ratio"] = float(np.sum(np.sign(R[rr, 6]) * g) / np.sum(np.abs(R[rr, 6])))
        mv = np.flatnonzero((R[:, 0] >= t_on) & (np.hypot(R[:, 3], R[:, 4]) > 0.02))
        if len(mv):
            t1 = R[mv[0], 0]
            out["start_rot_err"] = float(np.degrees(wrap(np.interp(t1, od[:, 0], np.unwrap(od[:, 3])) - R[mv[0], 5])))
        rot = (R[:, 0] >= t_on) & (R[:, 0] <= t_off) & (np.abs(R[:, 6]) > 0.3)
        e = np.degrees(wrap(np.interp(R[rot, 0], od[:, 0], np.unwrap(od[:, 3])) - R[rot, 5]))
        out["rot_eth_rms"], out["rot_eth_max"] = float(np.sqrt(np.mean(e ** 2))), float(np.abs(e).max())
    return out


ROWS = [("위치 오차 RMS cm", "exy_rms", 100), ("위치 오차 최대 cm", "exy_max", 100), ("SLAM 기준 위치 RMS cm", "exy_slam_rms", 100),
        ("경로까지 거리 RMS cm", "dpath_rms", 100), ("경로까지 거리 최대 cm", "dpath_max", 100),
        ("헤딩 오차 RMS °", "eth_rms", 1), ("헤딩 오차 최대 °", "eth_max", 1), ("회전 중 헤딩 오차 RMS °", "rot_eth_rms", 1), ("회전 중 헤딩 오차 최대 °", "rot_eth_max", 1),
        ("회전 속도 비 (자이로 / 기준)", "rot_ratio", 1), ("출발 회전 끝 헤딩 오차 °", "start_rot_err", 1),
        ("속력 오차 RMS m/s", "v_err_rms", 1), ("토크 변화 RMS mN·m/주기", "dtau_rms", 1000),
        ("슬립 10 Hz LPF > 25 % 시간 %", "slip25", 1), ("슬립 10 Hz LPF > 50 % 시간 %", "slip50", 1)]
res = {}
for ctrl, paths in RUNS.items():
    res[ctrl] = []
    for b in paths:
        lab = os.path.basename(b)
        r, _ = AZ.analyze(b)
        res[ctrl].append(dict(r, **extra(lab), lab=lab))
json.dump({k: [{kk: vv for kk, vv in d.items() if isinstance(vv, (int, float, str))} for d in v] for k, v in res.items()},
          open(os.path.join(HERE, "summary.json"), "w"), ensure_ascii=False, indent=1)
fmt = lambda a: f"{np.mean(a):.2f} ({np.min(a):.2f}~{np.max(a):.2f})" if abs(np.mean(a)) < 1 else f"{np.mean(a):.1f} ({np.min(a):.1f}~{np.max(a):.1f})"
print("| 지표 (판 평균, 최소~최대) | " + " | ".join(f"{k} ({len(v)} 판)" for k, v in res.items()) + " |")
print("|---|" + "---|" * len(RUNS))
for name, key, sc in ROWS:
    cells = []
    for ctrl in RUNS:
        vals = [d[key] * sc for d in res[ctrl] if key in d and d[key] == d[key]]
        cells.append(fmt(vals) if vals else "—")
    print(f"| {name} | " + " | ".join(cells) + " |")
for ctrl in RUNS:
    print(ctrl, [d["lab"] for d in res[ctrl]])
