"""슬립 때 속도 출처별 오차: 바퀴 기구학 (r/4)Σ(±ω_i) vs 추정기 (kf/slam) vs 참값 몸체 속도.
   참 λ (/slip_logger/slip 9~12) 의 max|λ| 로 구간을 나눈다. A·엔코더 판 (/wheel_states 기록됨)."""
import sys, numpy as np
sys.path.insert(0, "analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam")
import analyze as A
r = 0.04
print("| 판 | 구간 (max|λ|) | 시간 % | 바퀴 기구학 속도 오차 RMS cm/s | 추정기 속도 오차 RMS cm/s |")
print("|---|---|---|---|---|")
for lab in sys.argv[1:]:
    D = A.read(A.RUNS + "/" + lab)
    G, E = A.odom_arr(D["/sim/ground_truth"]), A.odom_arr(D["/state_estimator/odom"])
    arm = next(t for t, m in D["/mpc/armed"] if m.data)
    W = np.array([[t, *m.velocity] for t, m in D["/wheel_states"] if t >= arm])
    S = np.array([[t, *m.data[9:13]] for t, m in D["/slip_logger/slip"] if len(m.data) >= 13])
    t = W[:, 0]; w = W[:, 1:5]
    vw = np.stack([r / 4 * w.sum(1), r / 4 * (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3])], 1)
    vg = np.stack([np.interp(t, G[:, 0], G[:, 4]), np.interp(t, G[:, 0], G[:, 5])], 1)
    ve = np.stack([np.interp(t, E[:, 0], E[:, 4]), np.interp(t, E[:, 0], E[:, 5])], 1)
    lam = np.abs(np.stack([np.interp(t, S[:, 0], S[:, 1 + i]) for i in range(4)], 1)).max(1)
    mov = np.hypot(vg[:, 0], vg[:, 1]) > 0.05
    for name, m in (("< 0.1 (거의 무슬립)", lam < 0.1), ("0.1–0.5", (lam >= 0.1) & (lam <= 0.5)), ("> 0.5 (큰 슬립)", lam > 0.5)):
        m = m & mov
        if m.sum() < 10:
            continue
        rw = np.sqrt(np.mean(np.sum((vw[m] - vg[m]) ** 2, 1))) * 100
        re = np.sqrt(np.mean(np.sum((ve[m] - vg[m]) ** 2, 1))) * 100
        print(f"| {lab} | {name} | {100 * m.sum() / mov.sum():.0f} | {rw:.1f} | {re:.1f} |")
