# 직선 급가속 (shape:=line, 1 m, 1.0 m/s): 명령 가속 vs 실제 몸체 가속 vs 바퀴 가속, 가속 구간 슬립, 멈춘 자리
#   몸체 가속: /imu/base_link a_x (무장 전 정지 1 s 평균을 바이어스로 뺌), 몸체 속도: 그 적분 (가속 구간 0.5 s 안이라 표류 작음) · SLAM 차분
#   바퀴 속도: /wheel_states velocity → v_w = r·mean(ω) (r 0.04), 가속 구간 비교 λ = (v_w − v_b)/max(v_w, v_b)
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r = 0.04
print("bag | 명령 accel | IMU a_x 최대 / 가속 구간 평균 | 바퀴 v 기울기 최대 | 0.4 s 시점 v: 기준 / 바퀴 / IMU 적분 / SLAM | λ 최대 (가속 구간) | 바퀴별 τ 최대 | 멈춘 자리 (SLAM, 진행 방향) | 옆 벗어남 최대")
for b in sys.argv[1:]:
    raw = read("bags/2026-11-21/" + b, ("/imu/base_link", "/wheel_states", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose", "/wheel_torque_cmd"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data)
    R = np.array([[t, r_.vx, r_.vy] for t, m in raw["/dra_mppi/reference_trajectory"] if t >= s0 and len(m.references) for r_ in [m.references[0]]])
    vref = np.hypot(R[:, 1], R[:, 2]); t_go = R[np.argmax(vref > 0.01), 0]
    # 기준열 첫 행은 "지금" → 기준 속도가 처음 오르는 시각을 출발로
    acc_cmd = np.max(np.diff(vref) / np.diff(R[:, 0]))
    I = np.array([[t, m.linear_acceleration.x, m.linear_acceleration.y] for t, m in raw["/imu/base_link"]])
    bias = I[(I[:, 0] > t_go - 1.2) & (I[:, 0] < t_go - 0.2), 1].mean()
    W = np.array([[t, *m.velocity, *m.torque] for t, m in raw["/wheel_states"]])
    vw = r * W[:, 1:5].mean(1)
    P = np.array([[t, m.pose.position.x, m.pose.position.y] for t, m in raw["/tracked_pose"]])
    p0 = P[np.argmin(np.abs(P[:, 0] - t_go)), 1:]
    win = (I[:, 0] >= t_go - 0.05) & (I[:, 0] <= t_go + 0.6)
    ax = np.convolve(I[:, 1] - bias, np.ones(5) / 5, "same")
    tI = I[win, 0]; vI = np.concatenate([[0], np.cumsum(0.5 * (ax[win][1:] + ax[win][:-1]) * np.diff(tI))])
    w2 = (W[:, 0] >= t_go - 0.05) & (W[:, 0] <= t_go + 0.6)
    vb_on_w = np.interp(W[w2, 0], tI, vI)
    lam = (vw[w2] - vb_on_w) / np.maximum(np.maximum(np.abs(vw[w2]), np.abs(vb_on_w)), 0.05)
    dvw = np.convolve(np.gradient(vw, W[:, 0]), np.ones(3) / 3, "same")
    acc_phase = (I[:, 0] >= t_go) & (I[:, 0] <= t_go + 1.0 / acc_cmd)
    t4 = t_go + 0.4
    vref4 = np.interp(t4, R[:, 0], vref); vw4 = np.interp(t4, W[:, 0], vw); vI4 = np.interp(t4, tI, vI)
    k = (P[:, 0] > t4 - 0.15) & (P[:, 0] < t4 + 0.15); vS4 = np.hypot(*np.polyfit(P[k, 0], P[k, 1:], 1)[0]) if k.sum() > 2 else np.nan
    # 진행 방향 = 출발점에서 끝점 방향
    after = P[P[:, 0] > R[-1, 0] - 0.5]
    pe = after[-1, 1:] if len(after) else P[-1, 1:]
    d = pe - p0; u = d / np.linalg.norm(d)
    mv = P[(P[:, 0] >= t_go) & (P[:, 0] <= R[-1, 0])]
    lat = np.abs((mv[:, 1:] - p0) @ np.array([-u[1], u[0]]))
    print(f"{b} | {acc_cmd:.1f} | {ax[acc_phase].max():.2f} / {ax[acc_phase].mean():.2f} m/s² | {dvw[(W[:,0]>=t_go)&(W[:,0]<=t_go+1.0/acc_cmd)].max():.2f} | "
          f"{vref4:.2f} / {vw4:.2f} / {vI4:.2f} / {vS4:.2f} m/s | {lam.max():.2f} | {np.abs(W[w2, 5:9]).max(0).round(2)} | {np.linalg.norm(d):.3f} m | {100*lat.max():.1f} cm")
