#!/usr/bin/python3
"""앞방향 어긋남이 속도에 따라 바뀌나 (2026-11-22). 기계적 원인이면 속도와 무관, 라이다 스캔 움직임 왜곡이면 속도에 비례.
β_SLAM − β_바퀴 를 속력 구간별로 (정속 표본, fit_kinematics.bag_samples). 좌·우회전 평균으로 선회 몫을 뺀 값도."""
import glob, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit_kinematics as F
rows = []
for meta in sorted(glob.glob("bags/2026-11-2[0-2]/*/metadata.yaml")):
    d = os.path.dirname(meta)
    try:
        args = open(os.path.join(d, "run_info", "args.txt")).read().splitlines()[3]
    except (OSError, IndexError):
        continue
    shape = (re.findall(r"shape:=(\w+)", args) or ["?"])[0]
    if shape not in ("circles8", "square", "line"):
        continue
    if shape == "line" and "seg_dir_deg:=90" in args:
        continue
    try:
        S = F.bag_samples(d)
    except Exception:
        continue
    if S is None or len(S) < 10:
        continue
    for x in S:
        rows.append((shape, *x[:7]))
sh = np.array([r[0] for r in rows]); X = np.array([r[1:] for r in rows], float)
vx, vy, wz, W = X[:, 0], X[:, 1], X[:, 2], X[:, 3:7]
vxw = 0.01 * W.sum(1); vyw = 0.01 * (-W[:, 0] + W[:, 1] + W[:, 2] - W[:, 3])
sp = np.hypot(vx, vy)
db = np.degrees(np.angle(np.exp(1j * (np.arctan2(vy, vx) - np.arctan2(vyw, vxw)))))
print("| 속력 구간 m/s | 표본 | β 차 중앙값 ° (전체) | 직진 |ω|<0.3 | 좌회전 | 우회전 | (좌+우)/2 |")
print("|---|---|---|---|---|---|---|")
for lo, hi in ((0.15, 0.3), (0.3, 0.45), (0.45, 0.6), (0.6, 0.75), (0.75, 0.9), (0.9, 1.2), (1.2, 1.8)):
    m = (sp >= lo) & (sp < hi) & (vx > 0)
    if m.sum() < 15:
        continue
    st_ = m & (np.abs(wz) < 0.3); L = m & (wz > 0.3); R = m & (wz < -0.3)
    f = lambda s: f"{np.median(db[s]):+.2f} ({s.sum()})" if s.sum() >= 10 else "—"
    lr = f"{(np.median(db[L]) + np.median(db[R])) / 2:+.2f}" if L.sum() >= 10 and R.sum() >= 10 else "—"
    print(f"| {lo:.2f}~{hi:.2f} | {m.sum()} | {np.median(db[m]):+.2f} | {f(st_)} | {f(L)} | {f(R)} | {lr} |")
