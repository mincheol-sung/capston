import sys, numpy as np
sys.path.insert(0, "analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam")
import analyze as A
for lab in sys.argv[1:]:
    D = A.read(A.RUNS + "/" + lab)
    G, E = A.odom_arr(D["/sim/ground_truth"]), A.odom_arr(D["/state_estimator/odom"])
    msgs = [m for _, m in D["/dra_mppi/reference_trajectory"] if m.references]
    dts = sorted({round(m.dt, 4) for m in msgs})
    st = np.array([A.st(m.header) for m in msgs]); rate = 1 / np.median(np.diff(st))
    arm = next(t for t, m in D["/mpc/armed"] if m.data); t1 = max(t for t, _ in D["/torque_mpc/status"])
    # 방법 1: references[0] 을 stamp(잘못) / stamp+dt(맞음)
    out = {}
    for name, off in (("stamp", 0.0), ("stamp+dt", 1.0)):
        R = np.array(sorted((A.st(m.header) + off * m.dt, m.references[0].x, m.references[0].y) for m in msgs))
        g = G[(G[:, 0] >= arm) & (G[:, 0] <= t1) & (np.hypot(G[:, 4], G[:, 5]) > 0.05)]
        e = np.hypot(g[:, 1] - np.interp(g[:, 0], R[:, 0], R[:, 1]), g[:, 2] - np.interp(g[:, 0], R[:, 0], R[:, 2]))
        out[name] = np.sqrt(np.mean(e**2))
    # 방법 2: status 로 직접 — Xr = Xe + est,  참값 오차 = Xr − truth  (같은 틱)
    S = np.array([[t, *m.data[1:3]] for t, m in D["/torque_mpc/status"] if arm <= t <= t1])
    v = np.interp(S[:, 0], G[:, 0], np.hypot(G[:, 4], G[:, 5])); S = S[v > 0.05]
    ex, ey = np.interp(S[:, 0], E[:, 0], E[:, 1]), np.interp(S[:, 0], E[:, 0], E[:, 2])
    gx, gy = np.interp(S[:, 0], G[:, 0], G[:, 1]), np.interp(S[:, 0], G[:, 0], G[:, 2])
    ec = S[:, 1:3]; ee = np.stack([ex - gx, ey - gy], 1); et = ec + ee
    r = lambda a: np.sqrt(np.mean((a**2).sum(1)))
    corr = np.mean((ec * ee).sum(1)) / (r(ec) * r(ee))
    print(f"{lab}: dt={dts} rate={rate:.1f}Hz | ref0@stamp {out['stamp']*100:.2f} | ref0@stamp+dt {out['stamp+dt']*100:.2f} | "
          f"status 직접: 참값 {r(et)*100:.2f} = 제어기 {r(ec)*100:.2f} ⊕ 추정 {r(ee)*100:.2f} (상관 {corr:+.2f})")
