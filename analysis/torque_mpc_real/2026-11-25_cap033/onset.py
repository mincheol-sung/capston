# 헛돎·끌림이 시작한 순간의 바닥 토크 → 바퀴별 마찰계수:  /usr/bin/python3 onset.py
#   바닥 토크 두 가지 (헛돌기 전에는 바퀴 속도 = 몸체가 시키는 속도라고 본다):
#     모델 값  = P (τ − f),   f = 상한 옮김 양 (기준 바퀴 속도의 마찰, /torque_mpc/cap)
#     실제 추정 = P (τ − 마찰(ω_nom)),  ω_nom = 몸체 속도가 시키는 바퀴 속도 (/slip_logger/slip),  마찰 = torque_plant.yaml 띄운 값
#   마찰계수 = √2 · |바닥 토크| / (r · N_i),  N_i = 정지 하중 몫 × m g  (MPC 상한과 같은 정의)
#   시작 = 그 바퀴가 0.15 s 동안 |λ| < 0.15 였다가 |λ| > 0.25 가 된 첫 표본 (λ 부호 = 바닥 토크 부호: 헛돎 / 반대: 끌림)
import glob, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
r, k, m, Jw, Iz, gr = 0.04, 0.1932, 4.56, 8.9e-4, 0.06, 9.81
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
mw = 4 * Jw / r**2
P = np.eye(4) - Jw * F.T @ np.diag(1 / np.array([m + mw, m + mw, Iz + mw * k**2])) @ F
share = np.array([0.2388, 0.2392, 0.2608, 0.2612]); N = share * m * gr
tc = np.array([0.0174, 0.0206, 0.0155, 0.0157]); bv = np.array([0.001066, 0.001158, 0.001112, 0.001177])
fric = lambda w: tc * np.tanh(w / 0.5) + bv * w
mu = lambda g: np.sqrt(2) * np.abs(g) / (r * N)
W = ["FL", "FR", "RL", "RR"]
ev = []
for f in sorted(glob.glob(os.path.join(HERE, "*.npz"))):
    lab = os.path.basename(f)[:-4]; A = np.load(f)
    arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["st"][-1, 0]
    st, cap = A["st"], A["cap"]; s = (st[:, 0] >= t_on) & (st[:, 0] <= t_off)
    t = st[s, 0]; D = st[s, 1:]; C = cap[s, 1:]; tau, fs = D[:, 4:8], C[:, 2:6]
    slp = A["slip"]; lam = np.column_stack([np.interp(t, slp[:, 0], slp[:, c]) for c in range(2, 6)])
    wn = np.column_stack([np.interp(t, slp[:, 0], slp[:, c]) for c in range(6, 10)])
    gm = (P @ (tau - fs).T).T
    ga = (P @ (tau - np.array([fric(w) for w in wn])).T).T
    for i in range(4):
        for j in range(15, len(t)):
            sg = np.sign(gm[j, i])
            if abs(lam[j, i]) > 0.25 and np.all(np.abs(lam[j - 15:j, i]) < 0.15) and abs(gm[j, i]) > 0.03:
                kind = "헛돎" if np.sign(lam[j, i]) == sg else "끌림"
                q = slice(j - 3, j)                                       # 시작 직전 30 ms
                ev.append((lab, W[i], t[j] - t_on, kind, 1e3 * gm[q, i].mean(), 1e3 * ga[q, i].mean(), mu(gm[q, i].mean())[i] if False else
                           np.sqrt(2) * abs(gm[q, i].mean()) / (r * N[i]), np.sqrt(2) * abs(ga[q, i].mean()) / (r * N[i]), wn[j, i], 1e3 * tau[q, i].mean()))
print("| 판 | 바퀴 | 무장 뒤 s | 종류 | 모델 바닥 토크 mN·m | 실제 추정 바닥 토크 | 마찰계수 (모델 · 실제 추정) | 몸체가 시키는 바퀴 속도 rad/s | 모터 토크 |")
print("|---|---|---|---|---|---|---|---|---|")
for e in ev:
    print(f"| {e[0]} | {e[1]} | {e[2]:.2f} | {e[3]} | {e[4]:+.1f} | {e[5]:+.1f} | {e[6]:.3f} · {e[7]:.3f} | {e[8]:+.1f} | {e[9]:+.0f} |")
print("\n바퀴별 헛돎·끌림 시작의 마찰계수 (실제 추정, 최소 · 중앙 · 최대, 횟수):  MPC 상한은 0.33 × 0.8 = 0.264")
for w in W:
    for kind in ("헛돎", "끌림"):
        v = np.array([e[7] for e in ev if e[1] == w and e[3] == kind])
        if len(v):
            print(f"  {w} {kind}: {v.min():.3f} · {np.median(v):.3f} · {v.max():.3f}  ({len(v)} 회)")
