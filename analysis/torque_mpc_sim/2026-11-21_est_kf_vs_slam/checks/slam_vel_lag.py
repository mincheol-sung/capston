"""slam 속도 오차를 참값 궤적만으로 예측 (독립 경로): 10 Hz 스캔 시각 포즈 → 두 포즈 차분 → 1차 LPF(2 Hz) 의 결정적 오차,
   + 포즈 잡음(σ 2 mm/축) 이 차분·LPF 를 지난 몫 (해석식). 측정 추정 속도 오차와 비교."""
import sys, math, numpy as np
sys.path.insert(0, "analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam")
import analyze as A
Dt, f, sig = 0.1, 2.0, 0.002
al = 1 - math.exp(-2 * math.pi * f * Dt); be = 1 - al
sig_v_axis = sig / Dt * math.sqrt(al**2 * 2 / (1 + be))
print(f"α={al:.4f}  잡음 몫 (축당) {sig_v_axis*100:.2f} cm/s → 2D RMS {math.sqrt(2)*sig_v_axis*100:.2f} cm/s")
for lab in sys.argv[1:]:
    D = A.read(A.RUNS + "/" + lab)
    G, E = A.odom_arr(D["/sim/ground_truth"]), A.odom_arr(D["/state_estimator/odom"])
    arm = next(t for t, m in D["/mpc/armed"] if m.data); t1 = max(t for t, _ in D["/torque_mpc/status"])
    c, s = np.cos(G[:, 3]), np.sin(G[:, 3])
    VW = np.stack([c * G[:, 4] - s * G[:, 5], s * G[:, 4] + c * G[:, 5]], 1)    # 참 월드 속도
    ts = np.arange(G[0, 0], G[-1, 0], Dt)
    P = np.stack([np.interp(ts, G[:, 0], G[:, 1]), np.interp(ts, G[:, 0], G[:, 2])], 1)
    Vt = np.stack([np.interp(ts, G[:, 0], VW[:, 0]), np.interp(ts, G[:, 0], VW[:, 1])], 1)
    y = None; err = []
    for k in range(1, len(ts)):
        vr = (P[k] - P[k - 1]) / Dt
        y = vr if y is None else y + al * (vr - y)
        err.append((ts[k], *(y - Vt[k])))
    err = np.array(err)
    vt = np.interp(err[:, 0], G[:, 0], np.hypot(G[:, 4], G[:, 5]))
    m = (err[:, 0] >= arm) & (err[:, 0] <= t1) & (vt > 0.05)
    det = np.sqrt(np.mean((err[m, 1:] ** 2).sum(1)))
    w = np.abs(np.interp(err[m, 0], G[:, 0], G[:, 6])); v = vt[m]
    # 해석식: 회전하는 위상자 v e^{jωt} 에 차분 D(z)=(1-z^-1)/(jωΔt) 와 LPF H(z)=α/(1-βz^-1) 를 거친 오차 |1 - H·D|·v
    z = np.exp(1j * w * Dt)
    Dn = np.where(w > 1e-6, (1 - 1 / z) / (1j * w * Dt + 1e-12), 1.0)
    H = al / (1 - be / z)
    ana = np.sqrt(np.mean((np.abs(1 - H * Dn) * v) ** 2))
    print(f"{lab}: 참 |ω| RMS {np.sqrt(np.mean(w**2)):.2f} rad/s, v RMS {np.sqrt(np.mean(v**2)):.2f} m/s | "
          f"결정적 오차 (참값 궤적 재현) {det*100:.2f} cm/s, 해석식 {ana*100:.2f} cm/s | "
          f"⊕ 잡음 → {math.hypot(det, math.sqrt(2)*sig_v_axis)*100:.2f} cm/s")
