# floor_step.md 의 "토크 단계별" 표 두 개 → fig_step.png   (/usr/bin/python3 plot_step.py)
import os, re
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__))
rows = [l for l in open(os.path.join(H, "floor_step.md")).read().split("\n")]
def table(title):
    i = next(k for k, l in enumerate(rows) if l.startswith(title))
    out = []
    for l in rows[i + 4:]:
        if not l.startswith("|"): break
        out.append([c.strip() for c in l.strip("|").split("|")])
    return out
T1, T2 = table("## 토크 단계별 (회들의 중앙값)"), table("## 토크 단계별 바닥 전달 토크")
tau = np.array([float(r[0]) for r in T1]) * 1e-3
a_imu = np.array([float(r[3].split()[0]) for r in T1])
a_lo, a_hi = (np.array([float(x) for x in re.findall(r"\(([\d.]+)~([\d.]+)\)", r[3])[0]])[k] for k in (0, 1)), None
rng = np.array([[float(x) for x in re.findall(r"\(([\d.]+)~([\d.]+)\)", r[3])[0]] for r in T1])
a_ns = np.array([float(r[4]) for r in T1])                      # 구름 손실 5 mN·m 를 뺀 예상
a_w = np.array([[float(x) for x in r[5].split()] for r in T1])
ratio = np.array([[float(x) for x in r[7].split()] for r in T1])
spin = np.array([[int(x) for x in r[6].split()] for r in T1]); n = np.array([int(r[1]) for r in T1])
tg = np.array([[float(x) for x in r[2].split()] for r in T2])
W, C = ["FL", "FR", "RL", "RR"], ["C3", "C1", "C0", "C2"]
fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
ax[0].plot(tau, a_ns + 4 * 0.005 / 0.2714, "k--", lw=1, label="no-slip model (lifted friction only)")
ax[0].plot(tau, a_ns, "k:", lw=1, label="no-slip model (+5 mN.m rolling loss)")
ax[0].errorbar(tau, a_imu, yerr=[a_imu - rng[:, 0], rng[:, 1] - a_imu], fmt="ko-", capsize=3, label="body accel (IMU, median, min-max)")
for i in range(4): ax[0].plot(tau, a_w[:, i], C[i] + "s-", ms=4, lw=0.8, label=f"{W[i]} wheel accel r*wdot")
ax[0].set_ylim(0, 8.2); ax[0].set_xlabel("motor torque per wheel [N.m]"); ax[0].set_ylabel("acceleration [m/s^2]"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
ax[0].set_title("body vs wheel acceleration")
for i in range(4): ax[1].plot(a_imu, 100 * (ratio[:, i] - 1), C[i] + "o-", label=W[i])
ax[1].axhline(25, color="gray", ls="--", lw=0.8); ax[1].text(0.8, 26, "25 %", color="gray", fontsize=8)
ax[1].set_xlabel("body acceleration (IMU) [m/s^2]"); ax[1].set_ylabel("wheel speed over body speed at cut-off [%]"); ax[1].legend(); ax[1].grid(alpha=0.3)
ax[1].set_title("how much faster each wheel ends than the body")
for i in range(4): ax[2].plot(a_imu, tg[:, i], C[i] + "o-", label=W[i])
ax[2].plot(a_imu, 4.56 * a_imu * 0.04 / 4 * 1e3, "k--", lw=1, label="m*a*r/4 (equal share)")
ax[2].set_xlabel("body acceleration (IMU) [m/s^2]"); ax[2].set_ylabel("ground torque per wheel [mN.m]"); ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)
ax[2].set_title("torque passed to the floor (wheel side, eq. W1)")
fig.tight_layout(); fig.savefig(os.path.join(H, "fig_step.png"), dpi=85)
print("tau", tau, "\na_imu", a_imu, "\nno-slip(roll 0)", np.round(a_ns + 4 * 0.005 / 0.2714, 2), "\nmeasured/model(roll 0)", np.round(a_imu / (a_ns + 4 * 0.005 / 0.2714), 3), "\nsqrt2*a/g", np.round(np.sqrt(2) * a_imu / 9.81, 3))
print("spin count", spin.tolist(), "of", n.tolist())
