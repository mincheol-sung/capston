# 한 판의 0.2 s 간격 시계열:  /usr/bin/python3 timeseries.py 125014_cap033_r1 [간격 s]
#   기준 속도 · 몸체 속도 · 오차 · 모터 토크 · 모델 바닥 토크 / 상한 · 슬립 λ · 외란 추정
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
r, k, m, Jw, Iz = 0.04, 0.1932, 4.56, 8.9e-4, 0.06
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
mw = 4 * Jw / r**2
P = np.eye(4) - Jw * F.T @ np.diag(1 / np.array([m + mw, m + mw, Iz + mw * k**2])) @ F
tbar = r / np.sqrt(2) * 0.33 * np.array([0.2388, 0.2392, 0.2608, 0.2612]) * m * 9.81 * 0.8
lab = sys.argv[1]; dt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
A = np.load(os.path.join(HERE, lab + ".npz"))
arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
t_off = td[td > t_on][0] if (td > t_on).any() else A["st"][-1, 0]
st, cap = A["st"], A["cap"]; s = (st[:, 0] >= t_on) & (st[:, 0] <= t_off); S, C = st[s], cap[s][:, 1:]; t = S[:, 0]; D = S[:, 1:]
tau, fs = D[:, 4:8], C[:, 2:6]; g = (P @ (tau - fs).T).T
ip = lambda X, cols: np.column_stack([np.interp(t, X[:, 0], X[:, c]) for c in cols])
od = A["od"]; vb = ip(od, [4, 5, 6]); ref = A["ref"]; vr = ip(ref, [3, 4]); L = ip(A["slip"], range(2, 6)); om = ip(A["ws"], range(5, 9))
ad = D[:, 17:20] / 0.01
yaw = np.unwrap(od[:, 3]); y0 = np.interp(t_on, od[:, 0], yaw)
print(f"{lab}: 출발 yaw {np.degrees(y0):.1f}°, 기준 첫 점 vx vy {ref[np.searchsorted(ref[:, 0], t_on + 0.5), 3:5].round(3)}  (기준 속도는 월드 좌표, 몸체 속도는 추정기 twist)")
print("| s | 기준 v월드 x y | 몸체 vx vy ωz | yaw 변화 ° | 오차 x y mm · θ° | 모터 τ FL FR RL RR mN·m | 바닥/τ̄ FL FR RL RR | 바퀴 ω rad/s | λ FL FR RL RR | 외란 추정 앞 옆 회전 |")
print("|---|---|---|---|---|---|---|---|---|---|")
f = lambda v, n=2: " ".join(f"{x:+.{n}f}" for x in v)
for a in np.arange(0, t[-1] - t_on, dt):
    q = (t - t_on >= a) & (t - t_on < a + dt)
    if q.sum() < 3:
        continue
    yq = np.degrees(np.interp(t[q], od[:, 0], yaw).mean() - y0)
    print(f"| {a:4.1f} | {f(vr[q].mean(0))} | {f(vb[q].mean(0))} | {yq:+5.1f} | {1e3 * D[q, 1].mean():+4.0f} {1e3 * D[q, 2].mean():+4.0f} · {np.degrees(D[q, 3].mean()):+5.1f} | "
          f"{f(1e3 * tau[q].mean(0), 0)} | {f((g[q] / tbar).mean(0))} | {f(om[q].mean(0), 1)} | {f(np.nanmean(L[q], 0))} | {f(ad[q].mean(0))} |")
