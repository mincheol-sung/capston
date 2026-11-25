"""정사각형: 기준 가속 구간(진행 방향 기준 가속도 > +0.5) / 감속(< −0.5) / 등속 으로 나눠
   (추정−참) 진행 방향 속도 평균, (참−기준) 진행 방향 위치 평균, 상한 닿음 %."""
import sys, re, os, numpy as np
sys.path.insert(0, "analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam")
import analyze as A
def world(Y):
    c, s = np.cos(Y[:, 3]), np.sin(Y[:, 3]); return np.stack([c * Y[:, 4] - s * Y[:, 5], s * Y[:, 4] + c * Y[:, 5]], 1)
for lab in sys.argv[1:]:
    D = A.read(A.RUNS + "/" + lab)
    G, E = A.odom_arr(D["/sim/ground_truth"]), A.odom_arr(D["/state_estimator/odom"])
    arm = next(t for t, m in D["/mpc/armed"] if m.data); t1 = max(t for t, _ in D["/torque_mpc/status"])
    R = np.array(sorted((A.st(m.header) + m.dt, r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for _, m in D["/dra_mppi/reference_trajectory"] if m.references for r in m.references[:1]))
    tg = np.arange(arm, t1, 0.01)
    ip = lambda A_, c: np.interp(tg, A_[:, 0], A_[:, c] if c != 3 else np.unwrap(A_[:, 3]))
    Rr = np.stack([tg] + [ip(R, c) for c in range(1, 6)], 1); vr = world(Rr); sp = np.linalg.norm(vr, axis=1)
    Gi = np.stack([tg] + [ip(G, c) for c in range(1, 7)], 1); Ei = np.stack([tg] + [ip(E, c) for c in range(1, 7)], 1)
    u = vr / np.maximum(sp, 1e-9)[:, None]
    acc = np.gradient(np.convolve(sp, np.ones(10) / 10, "same"), tg)
    moving = sp > 0.05
    T = np.array([[t, *m.torque] for t, m in D["/torque_mpc/cmd_raw"] if arm <= t <= t1])
    tb = A.tau_bar_from_log(lab); satT = np.any(np.abs(T[:, 1:5]) >= 0.98 * tb[None], axis=1)
    accT = np.interp(T[:, 0], tg, acc); spT = np.interp(T[:, 0], tg, sp)
    out = []
    for name, mk, mkT in (("가속", (acc > 0.5) & moving, (accT > 0.5) & (spT > 0.05)), ("감속", (acc < -0.5) & moving, (accT < -0.5) & (spT > 0.05)),
                          ("등속", (np.abs(acc) <= 0.5) & moving, (np.abs(accT) <= 0.5) & (spT > 0.05))):
        dv = ((world(Ei) - world(Gi)) * u).sum(1)[mk].mean() * 100
        dp = ((Gi[:, 1:3] - Rr[:, 1:3]) * u).sum(1)[mk].mean() * 100
        out.append(f"{name}({mk.mean()*100:.0f}%): 추정−참 v {dv:+.1f} cm/s, 참−기준 along {dp:+.1f} cm, 상한(한 바퀴라도) {satT[mkT].mean()*100:.0f}%")
    print(lab, " | ".join(out))
