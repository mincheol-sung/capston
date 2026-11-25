#!/usr/bin/python3
"""SLAM 기준 진행 방향 vs 바퀴 기구학 진행 방향의 각 차 (2026-11-22).
β_s = atan2(v_y, v_x) (SLAM 중심 중앙 차분, 몸체), β_w = atan2(v_y,w, v_x,w) (바퀴 FK, 같은 창 평균). 정속·|v| > 0.3 m/s 표본.
좌회전(ω > 0.3) / 우회전(ω < −0.3) / 직진(|ω| < 0.3) 으로 나눔 — 일정한 좌표 어긋남이면 셋이 같고, 선회 옆미끄럼이면 부호가 바뀐다."""
import glob, math, os, re, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit_kinematics as F
rows = []
for meta in sorted(glob.glob("bags/2026-11-2[12]/*/metadata.yaml")):
    d = os.path.dirname(meta)
    try:
        args = open(os.path.join(d, "run_info", "args.txt")).read().splitlines()[3]
    except (OSError, IndexError):
        continue
    shape = (re.findall(r"shape:=(\w+)", args) or ["?"])[0]
    if shape not in ("circles8", "line", "square"):
        continue
    ctrl = "A" if "pure_pursuit" in args else "C"
    try:
        S = F.bag_samples(d)
    except Exception:
        continue
    if S is None or len(S) < 20:
        continue
    for r in S:
        rows.append((shape, ctrl, os.path.basename(d), *r[:7]))
shape = np.array([r[0] for r in rows]); ctrl = np.array([r[1] for r in rows]); name = np.array([r[2] for r in rows])
X = np.array([r[3:] for r in rows], float)
vx, vy, wz, W = X[:, 0], X[:, 1], X[:, 2], X[:, 3:7]
vxw = 0.01 * W.sum(1); vyw = 0.01 * (-W[:, 0] + W[:, 1] + W[:, 2] - W[:, 3])
m = np.hypot(vx, vy) > 0.3
bs, bw = np.degrees(np.arctan2(vy, vx)), np.degrees(np.arctan2(vyw, vxw))
print("| 묶음 | 표본 | β_SLAM 평균 ° | β_바퀴 평균 ° | β_SLAM − β_바퀴 중앙값 ° [사분위] |")
print("|---|---|---|---|---|")
for lab, sel in (("좌회전 ω>0.3", m & (wz > 0.3)), ("우회전 ω<−0.3", m & (wz < -0.3)), ("직진 |ω|<0.3", m & (np.abs(wz) < 0.3)),
                 ("C 만", m & (ctrl == "C")), ("A(PP) 만", m & (ctrl == "A")), ("circles8", m & (shape == "circles8")),
                 ("square", m & (shape == "square")), ("line", m & (shape == "line")), ("전체", m)):
    if sel.sum() < 10:
        continue
    d_ = bs[sel] - bw[sel]
    print(f"| {lab} | {sel.sum()} | {bs[sel].mean():+.2f} | {bw[sel].mean():+.2f} | {np.median(d_):+.2f} [{np.percentile(d_,25):+.2f}~{np.percentile(d_,75):+.2f}] |")
# bag 별 (주행 표본 30 개 이상)
per = []
for nm in sorted(set(name[m])):
    s = m & (name == nm)
    if s.sum() >= 30:
        per.append(np.median(bs[s] - bw[s]))
per = np.array(per)
print(f"\nbag 별 중앙값 ({len(per)} bag): 중앙값 {np.median(per):+.2f}°, 사분위 {np.percentile(per,25):+.2f}~{np.percentile(per,75):+.2f}°, 최소 {per.min():+.2f}, 최대 {per.max():+.2f}")
print(f"속력 배율 |v_SLAM| / |v_바퀴| 중앙값 {np.median(np.hypot(vx,vy)[m] / np.hypot(vxw,vyw)[m]):.4f}")
