# 구름 손실 시뮬 C 조건: 참값 x(τ) 를 기준 출발 시각에 맞춰 묶음 평균 → MPC 값 0 과의 차가 어느 구간에서 생기나 (기준은 모든 판이 같다)
import glob, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
def load(tag):
    out = []
    for f in sorted(glob.glob(os.path.join(H, "runs", f"C_{tag}_*.npz"))):
        tr = np.load(f)["tr"]; i0 = np.argmax(tr[:, 3] > 0.5)
        out.append((tr[:, 0] - tr[i0, 0], tr[:, 1]))
    return out
g = np.arange(0.0, 4.4, 0.01)
X = {k: np.array([np.interp(g, r[0], r[1]) for r in load(k)]) for k in ("r00", "r27", "r51", "r80")}
x0 = X["r00"].mean(0); v0 = np.gradient(x0, 0.01)
print("| 기준 출발 뒤 s | 값 0 의 x m (속도 m/s) | 2.7 − 0 mm (최소~최대) | 5.1 − 0 mm | 8.0 − 0 mm | 값 0 판 사이 표준편차 mm |")
print("|---|---|---|---|---|---|")
for a in np.arange(0.0, 4.4 - 1e-9, 0.2):
    s = (g >= a) & (g < a + 0.2)
    cell = []
    for k in ("r27", "r51", "r80"):
        d = (X[k][:, s].mean(1) - x0[s].mean()) * 1e3
        cell.append(f"{d.mean():+.1f} ({d.min():+.1f}~{d.max():+.1f})")
    print(f"| {a:.1f} | {x0[s].mean():.3f} ({v0[s].mean():+.2f}) | " + " | ".join(cell) + f" | {X['r00'][:, s].mean(1).std()*1e3:.1f} |")
print("\n가는 쪽 끝 넘어감 (x 최대 − 1.0) · 돌아온 쪽 끝 넘어감 (0 − x 최소) mm, 판별:")
for k in ("r00", "r27", "r51", "r80"):
    print(f"  {k}: 가는 쪽 {np.round((X[k].max(1) - 1.0) * 1e3, 1)}   돌아온 쪽 {np.round(-X[k][:, g > 2.5].min(1) * 1e3, 1)}")
