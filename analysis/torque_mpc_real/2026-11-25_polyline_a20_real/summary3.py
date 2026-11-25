# 꺾은선 accel 2.0 세 제어기 × 3 판: analyze.py 의 지표 (같은 잣대) + 11-23 회의 슬립 지표 (10 Hz LPF) → 제어기별 평균 (최소~최대)
#   source install/setup.bash 뒤  /usr/bin/python3 summary3.py   (read_bags.py npz 가 있어야 한다)
import json, os, sys
import numpy as np
from scipy.signal import butter, filtfilt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import analyze as AZ
B = "/home/smoc/capston/capston/bags/2026-11-25/"
RUNS = {"MPC (바퀴 속도)": ["151542_sq2_a20_mpcw_r1", "152823_sq2_a20_mpcw_r2", "152926_sq2_a20_mpcw_r3"],
        "PP": ["151644_sq2_a20_pp_r1", "153051_sq2_a20_pp_r2", "153213_sq2_a20_pp_r3"],
        "어댑티브 PP": ["151817_sq2_a20_adpp_r1", "153342_sq2_a20_adpp_r2", "153500_sq2_a20_adpp_r3"]}

def slip(lab):
    A = np.load(os.path.join(HERE, lab + ".npz")); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    fs = 1 / np.median(np.diff(t)); bb, aa = butter(1, 10.0 / (fs / 2)); Lf = filtfilt(bb, aa, L, axis=0)
    od = A["od"]; sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    w = (t >= t_on) & (t <= t_off) & (sp > 0.05); mx = np.abs(Lf[w]).max(1)
    return 100 * np.mean(mx > 0.25), 100 * np.mean(mx > 0.5)

ROWS = [("위치 오차 RMS cm", "exy_rms", 100), ("위치 오차 최대 cm", "exy_max", 100), ("SLAM 기준 위치 RMS cm", "exy_slam_rms", 100),
        ("진행 방향 오차 RMS cm", "along_rms", 100), ("경로까지 거리 RMS cm", "dpath_rms", 100), ("경로까지 거리 최대 cm", "dpath_max", 100),
        ("헤딩 오차 RMS °", "eth_rms", 1), ("헤딩 오차 최대 °", "eth_max", 1), ("속력 오차 RMS m/s", "v_err_rms", 1),
        ("가속도 오차 RMS (IMU) m/s²", "a_imu_err_rms", 1), ("토크 변화 RMS mN·m/주기", "dtau_rms", 1000)]
res = {}
for ctrl, labs in RUNS.items():
    res[ctrl] = []
    for lab in labs:
        r, _ = AZ.analyze(B + lab)
        s25, s50 = slip(lab)
        res[ctrl].append(dict(r, slip25=s25, slip50=s50, lab=lab))
json.dump({k: [{kk: vv for kk, vv in d.items() if isinstance(vv, (int, float, str))} for d in v] for k, v in res.items()},
          open(os.path.join(HERE, "summary3.json"), "w"), ensure_ascii=False, indent=1)
fmt = lambda a: f"{np.mean(a):.2f} ({np.min(a):.2f}~{np.max(a):.2f})" if np.mean(a) < 1 else f"{np.mean(a):.1f} ({np.min(a):.1f}~{np.max(a):.1f})"
print("| 지표 (3 판 평균, 최소~최대) | " + " | ".join(RUNS) + " |")
print("|---|" + "---|" * len(RUNS))
for name, key, sc in ROWS + [("슬립 10 Hz LPF > 25 % 시간 %", "slip25", 1), ("슬립 10 Hz LPF > 50 % 시간 %", "slip50", 1)]:
    cells = []
    for ctrl in RUNS:
        vals = [d[key] * sc for d in res[ctrl] if key in d and d[key] == d[key]]
        cells.append(fmt(vals) if vals else "—")
    print(f"| {name} | " + " | ".join(cells) + " |")
for ctrl in RUNS:
    print(ctrl, [(d["lab"][-2:], d.get("fault")) for d in res[ctrl]])
