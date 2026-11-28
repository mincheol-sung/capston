# 2026-11-26 구간별 경로 옆 오차 경향 — 꺾은선 6 직선 · 6 꼭짓점 (출발 회전 포함), 진행 방향 왼쪽 + [cm], SLAM 중심 위치
#   판 묶음: 롤러 끔 · 롤러 켬 (= 보정 끔) · 구동 방향 보정 켬. 기준 경로 꼭짓점 (출발 좌표): (0,0)→(0,−1)→(1,−1)→(1,1)→(2,1)→(2,0)→(0,0)
import os, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
RA = os.path.join(HERE, "..", "2026-11-25_roller_ab")
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
C = np.array([[0, 0], [0, -1], [1, -1], [1, 1], [2, 1], [2, 0], [0, 0]], float)
GROUPS = {"롤러 끔": (RA, "_rr_off_"), "롤러 켬 · 보정 끔": (HERE, "_dy_off_"), "보정 2.4° · −2.5": (HERE, "_dy_on_"), "A 2.4° · 0": (HERE, "_dy_a_"), "B 3.1° · +0.8": (HERE, "_dy_b_")}
EXC = {"180604_dy_off_r1", "180802_dy_off_r1", "181021_dy_off_r1"}


def local_path(folder, lab):
    A = np.load(os.path.join(folder, lab + ".npz")); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]
    od = A["od"]; i0 = np.searchsorted(od[:, 0], t_on); x0, y0, a0 = od[i0, 1], od[i0, 2], od[i0, 3]
    z = np.load(os.path.join(folder, "slam_cache", lab + ".npz"), allow_pickle=True); T = z["T"]
    c, s = np.cos(a0), np.sin(a0)
    return c * (T[:, 1] - x0) + s * (T[:, 2] - y0), -s * (T[:, 1] - x0) + c * (T[:, 2] - y0)


def seg_stats(X, Y):
    """직선 i: 구간 가운데 80 % 의 옆 오차 평균·끝 옆 오차, 꼭짓점 i: 꼭짓점 0.15 m 안 최대 바깥쪽 오차."""
    out = []
    for i in range(6):
        a, b = C[i], C[i + 1]; d = b - a; L = np.linalg.norm(d); u = d / L; n = np.array([-u[1], u[0]])      # n = 진행 방향 왼쪽
        P = np.column_stack([X, Y]) - a
        s, e = P @ u, P @ n
        sel = (s > 0.1 * L) & (s < 0.9 * L) & (np.abs(e) < 0.25)
        end = (s > 0.8 * L) & (s < 0.95 * L) & (np.abs(e) < 0.25)
        out.append((e[sel].mean() * 100 if sel.any() else np.nan, e[end].mean() * 100 if end.any() else np.nan))
    return out


print("직선 (출발 → 끝, 길이, 앞 꼭짓점 회전): 1 (0,0)→(0,−1) 1 m 출발 오른쪽 · 2 →(1,−1) 1 m 왼쪽 · 3 →(1,1) 2 m 왼쪽 · 4 →(2,1) 1 m 오른쪽 · 5 →(2,0) 1 m 오른쪽 · 6 →(0,0) 2 m 오른쪽")
for g, (folder, key) in GROUPS.items():
    labs = sorted(os.path.basename(f)[:-4] for f in glob.glob(os.path.join(folder, f"*{key}r*.npz")) if os.path.basename(f)[:-4] not in EXC)
    if not labs or not all(os.path.exists(os.path.join(folder, "slam_cache", l + ".npz")) for l in labs):
        continue
    S = np.array([seg_stats(*local_path(folder, lab)) for lab in labs])          # (판, 직선, 2)
    m = np.nanmean(S, axis=0)
    print(f"{g} ({len(labs)} 판)  직선 옆 오차 평균 [cm, 왼쪽 +]: " + "  ".join(f"{k + 1}: {m[k, 0]:+.1f}" for k in range(6))
          + "   |  끝 쪽 (80~95 %): " + "  ".join(f"{k + 1}: {m[k, 1]:+.1f}" for k in range(6)))
