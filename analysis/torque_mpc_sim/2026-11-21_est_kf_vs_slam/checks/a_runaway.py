"""A + PI 추정기 되먹임 발산 확인: /wheel_pi/status [t, ω_ref×4, ω_fb×4, τ×4, 포화 수] 와 엔코더 /wheel_states 를 나란히.
   발산 시작 = 엔코더 |ω_i| 가 처음 정격 38.746 rad/s 를 넘은 시각. 그 전후 0.5 s 의 바퀴별 값."""
import sys, numpy as np
sys.path.insert(0, "analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam")
import analyze as A
for lab in sys.argv[1:]:
    D = A.read(A.RUNS + "/" + lab)
    arm = next(t for t, m in D["/mpc/armed"] if m.data)
    P = np.array([[t, *m.data[1:14]] for t, m in D["/wheel_pi/status"]])
    W = np.array([[t, *m.velocity] for t, m in D["/wheel_states"]])
    G = A.odom_arr(D["/sim/ground_truth"])
    k = np.argmax(np.abs(W[:, 1:5]).max(1) > 38.746); t_on = W[k, 0]; i = int(np.argmax(np.abs(W[k, 1:5])))
    S = np.array([[t, *m.data[9:13]] for t, m in D["/slip_logger/slip"] if len(m.data) >= 13])
    print(f"{lab}: 무장 뒤 {t_on - arm:.2f} s 에 바퀴 {['FL','FR','RL','RR'][i]} |ω| > 정격, 몸체 참 속도 {np.interp(t_on, G[:,0], np.hypot(G[:,4],G[:,5])):.2f} m/s")
    print("   t−t_on |  ω_ref  ω_fb(추정기→T⁻¹)  ω_엔코더 |  τ_i  | 참 λ_i")
    for dt in (-0.4, -0.2, -0.1, 0.0, 0.1, 0.2, 0.4, 0.8):
        t = t_on + dt
        j = np.argmin(np.abs(P[:, 0] - t))
        print(f"   {dt:+.1f} s | {P[j,1+i]:7.1f} {P[j,5+i]:12.1f} {np.interp(t, W[:,0], W[:,1+i]):12.1f} | {P[j,9+i]:+.3f} | {np.interp(t, S[:,0], S[:,1+i]):+.2f}")
