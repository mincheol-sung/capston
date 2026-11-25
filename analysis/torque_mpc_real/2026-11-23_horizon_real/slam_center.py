"""moving_only.py 의 SLAM 경로 거리를 IMU 점 그대로 / 로봇 중심으로 옮겨서 (compare_slam.imu_to_center) 두 번 계산.
창·경로·거리 정의는 moving_only.py 와 같다. 추정기 − SLAM(중심) 위치 RMS 도 같이.
"""
import importlib.util, sys, numpy as np, yaml
WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]; off = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))
exec(open(f"{WS}/amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
print(f"IMU 오프셋 {off}")
# 원 위 IMU 점 반지름 (접선 헤딩, R 0.5): 왼쪽 원 / 오른쪽 원
R0 = 0.5
print(f"계산: IMU 점 반지름 − R = 왼쪽 원 {1e3*(np.hypot(R0-off[1], off[0])-R0):+.1f} mm, 오른쪽 원 {1e3*(np.hypot(R0+off[1], off[0])-R0):+.1f} mm")
print("bag | SLAM IMU 점 RMS / 최대 [cm] | SLAM 중심 RMS / 최대 | 추정기 − SLAM 중심 RMS [cm]")
for b in sys.argv[1:]:
    raw = read(b, ("/state_estimator/odom", "/figure8/path", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    mv = lambda t: (np.interp(t, R[:, 0], R[:, 1]) > 0.05) & (t >= R[0, 0]) & (t <= R[-1, 0])
    dist = lambda P: np.sqrt(((P[:, None, :] - path[None]) ** 2).sum(-1)).min(1)
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t, m in raw["/tracked_pose"] if s0 <= t <= s1]); S = S[mv(S[:, 0])]
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off)
    O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1])
    d0, d1 = dist(S[:, 1:3]), dist(np.c_[xc, yc])
    es = np.hypot(np.interp(S[:, 0], O[:, 0], O[:, 1]) - xc, np.interp(S[:, 0], O[:, 0], O[:, 2]) - yc)
    print(f"{b.split('/')[-1]} | {100*np.sqrt((d0**2).mean()):.1f} / {100*d0.max():.1f} | {100*np.sqrt((d1**2).mean()):.1f} / {100*d1.max():.1f} | {100*np.sqrt((es**2).mean()):.1f}")
