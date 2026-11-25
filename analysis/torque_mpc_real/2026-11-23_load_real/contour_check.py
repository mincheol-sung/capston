#!/usr/bin/python3
"""시간 추종 오차를 둘로 나눈다 (2026-11-23 자동 17 판, all_traj.npz — plot_all_traj.py 가 만든 것).
  e_time   = |p − p_r(t)|                        (지금 쓰는 위치 오차, trajectory tracking)
  e_lag    = (p − p_r(t))·t̂ (기준 진행 방향 성분)    (+ 앞섬 / − 뒤처짐 — 시간에 묶여서 생기는 몫)
  e_cont   = 경로 (기준 위치 전체를 0.002 s 간격으로 선형 보간한 점 집합) 까지 최단 거리  (contouring error, 시간 무관)
출력: 판별 e_time RMS · e_cont RMS/최대 · e_lag 평균/최소 [mm], e_cont² / e_time² 몫."""
import os
import numpy as np
from scipy.spatial import cKDTree
HERE = os.path.dirname(os.path.abspath(__file__))
z = dict(np.load(os.path.join(HERE, "all_traj.npz"), allow_pickle=True))
bags = sorted({k.rsplit("_", 1)[0] for k in z if k.endswith("_x")})
print("| 판 | e_time RMS | e_cont RMS / 최대 | e_lag 평균 / 가장 뒤처짐 | e_cont² ÷ e_time² | 큰 슬립 % |")
for bag in bags:
    x, y, ref, mv = z[f"{bag}_x"], z[f"{bag}_y"], z[f"{bag}_ref"], z[f"{bag}_mv"]
    c, s = np.cos(ref[:, 2]), np.sin(ref[:, 2])
    vw = np.stack([c * ref[:, 3] - s * ref[:, 4], s * ref[:, 3] + c * ref[:, 4]], 1)
    tg = vw / np.maximum(np.linalg.norm(vw, axis=1), 1e-9)[:, None]
    d = np.stack([x - ref[:, 0], y - ref[:, 1]], 1)
    e_t = np.linalg.norm(d, axis=1); lag = (d * tg).sum(1)
    u = np.linspace(0, len(ref) - 1, 10 * (len(ref) - 1) + 1)                   # 0.02 s → 0.002 s
    P = np.stack([np.interp(u, np.arange(len(ref)), ref[:, 0]), np.interp(u, np.arange(len(ref)), ref[:, 1])], 1)
    e_c = cKDTree(P).query(np.stack([x, y], 1))[0]
    m = mv
    rt, rc = np.sqrt(np.mean(e_t[m] ** 2)), np.sqrt(np.mean(e_c[m] ** 2))
    print(f"| {bag} | {1e3 * rt:.0f} | {1e3 * rc:.0f} / {1e3 * e_c[m].max():.0f} | {1e3 * lag[m].mean():+.0f} / {1e3 * lag[m].min():+.0f} | "
          f"{100 * rc ** 2 / rt ** 2:.0f} % | {z[f'{bag}_m'][3]:.0f} |")
