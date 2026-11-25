# 주행 중 (기준 속도 > 0.05 m/s) 만 — track_compare 는 무장 ~ 해제 전체라 완주 뒤 무장된 채 서 있던 시간이 RMS 를 묽힌다 (205044: 78 % 가 정지)
# 2026-11-23 정정: SLAM 열을 로봇 중심으로. /tracked_pose 는 Cartographer tracking frame (D455 IMU) 포즈라 그대로 쓰면
#   IMU 오프셋 (torque_estimator.yaml imu_offset) 몫이 섞인다 — R 0.3 원·접선 헤딩에서 한쪽 원 +35 mm, 헤딩 오차 Δθ 면 옆으로 0.11·sin Δθ.
#   compare_slam.imu_to_center (p_c = p_imu − R(ψ) r_imu) 로 옮긴다. 근거·옛 값 analysis/torque_mpc_real/2026-11-23_horizon_real/RESULTS_KO.md
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))
print("bag | 경로 거리 RMS / 최대 (추정기 = RViz 궤적) | 경로 거리 RMS (SLAM 중심) | 시간 기준 RMS / 최대 | 헤딩 RMS / 최대   [cm, °]")
for b in sys.argv[1:]:
    raw = read(b, ("/state_estimator/odom","/figure8/path","/mpc/armed","/dra_mppi/reference_trajectory","/tracked_pose","/torque_mpc/status","/pure_pursuit/status"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    mv = lambda t: (np.interp(t, R[:, 0], R[:, 1]) > 0.05) & (t >= R[0, 0]) & (t <= R[-1, 0])
    dist = lambda P: np.sqrt(((P[:, None, :] - path[None]) ** 2).sum(-1)).min(1)
    O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1]); O = O[mv(O[:, 0])]
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)]
                  for t, m in raw["/tracked_pose"] if s0 <= t <= s1]); S = S[mv(S[:, 0])]
    S[:, 1], S[:, 2] = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off)          # IMU 점 → 로봇 중심
    st = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    X = np.array([[t, *m.data[1:4]] for t, m in raw[st] if s0 <= t <= s1]); X = X[mv(X[:, 0])]
    dO, dS = dist(O[:, 1:]), dist(S[:, 1:3]); e = np.hypot(X[:, 1], X[:, 2]); th = np.degrees(np.abs(np.arctan2(np.sin(X[:, 3]), np.cos(X[:, 3]))))
    r = lambda a: 100 * np.sqrt((a ** 2).mean())
    print(f"{b.split('/')[-1]} {st.split('/')[1]:13s} | {r(dO):.1f} / {100*dO.max():.1f} | {r(dS):.1f} | {r(e):.1f} / {100*e.max():.1f} | {np.sqrt((th**2).mean()):.1f} / {th.max():.1f}")
