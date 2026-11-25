"""기구 확인: 곡선 구간에서 (참 속도 − 기준 속도), (추정 속도 − 기준 속도) 를 기준 진행 방향 t·안쪽 n 성분 평균으로.
   가설: slam 은 추정 속도가 ~90 ms 늦게 돈다 → 제어기는 추정 속도를 기준 속도에 맞춘다 → 참 속도는 기준보다 ωτ 만큼 앞서 돈다
   → 참값이 기준보다 앞섬 (along +).  참값−기준 along 평균, 안쪽 평균도 같이 (곡선 = |기준 yaw_rate| > 0.2)."""
import sys, numpy as np
sys.path.insert(0, "analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam")
import analyze as A
def world(Y):  # 몸체 속도 → 월드
    c, s = np.cos(Y[:, 3]), np.sin(Y[:, 3]); return np.stack([c * Y[:, 4] - s * Y[:, 5], s * Y[:, 4] + c * Y[:, 5]], 1)
for lab in sys.argv[1:]:
    D = A.read(A.RUNS + "/" + lab)
    G, E = A.odom_arr(D["/sim/ground_truth"]), A.odom_arr(D["/state_estimator/odom"])
    arm = next(t for t, m in D["/mpc/armed"] if m.data); t1 = max(t for t, _ in D["/torque_mpc/status"])
    R = np.array(sorted((A.st(m.header) + m.dt, r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for _, m in D["/dra_mppi/reference_trajectory"] if m.references for r in m.references[:1]))
    g = G[(G[:, 0] >= arm) & (G[:, 0] <= t1) & (np.hypot(G[:, 4], G[:, 5]) > 0.05)]
    ip = lambda c, A_=R: np.interp(g[:, 0], A_[:, 0], A_[:, c])
    ryaw = np.interp(g[:, 0], R[:, 0], np.unwrap(R[:, 3]))
    Rr = np.stack([g[:, 0], ip(1), ip(2), ryaw, ip(4), ip(5)], 1)
    vr = world(Rr); sp = np.linalg.norm(vr, axis=1)
    ok = (sp > 0.05) & (np.abs(ip(6)) > 0.2)
    t_ = vr / np.maximum(sp, 1e-9)[:, None]; n_ = np.stack([-t_[:, 1], t_[:, 0]], 1) * np.sign(ip(6))[:, None]  # n = 곡률 중심 쪽
    Ei = np.stack([g[:, 0], *(np.interp(g[:, 0], E[:, 0], E[:, c] if c != 3 else np.unwrap(E[:, 3])) for c in range(1, 7))], 1)
    vt, ve = world(g), world(Ei)
    dp = g[:, 1:3] - Rr[:, 1:3]
    f = lambda v, u: float(np.mean((v * u).sum(1)[ok]) * 100)
    print(f"{lab}: 참−기준 위치 along {f(dp, t_):+.2f} 안쪽 {f(dp, n_):+.2f} cm | 참−기준 속도 along {f(vt - vr, t_):+.2f} 안쪽 {f(vt - vr, n_):+.2f} cm/s | "
          f"추정−기준 속도 along {f(ve - vr, t_):+.2f} 안쪽 {f(ve - vr, n_):+.2f} cm/s | 추정−참 속도 안쪽 {f(ve - vt, n_):+.2f} cm/s | 곡선 {ok.mean()*100:.0f}%")
