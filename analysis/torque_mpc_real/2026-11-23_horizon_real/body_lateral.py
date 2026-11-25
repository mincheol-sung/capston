"""8자 경로 왼쪽 법선 성분 lat 을 두 원으로 나눠 분해 (교차점 0.15 m 안 뺌):
   lat_CCW = y_b + r_in,  lat_CW = y_b − r_in   →   y_b = (lat_CCW + lat_CW)/2 (몸체 기준 왼쪽 치우침), r_in = (lat_CCW − lat_CW)/2 (원 안쪽 치우침)
   접선 헤딩 (yaw_mode tangent) 이라 경로 왼쪽 = 몸체 왼쪽. 기준 속도 구간별로 내서 y_b 가 속도에 비례하는지 (구동계 방향 어긋남 꼴) 일정한지 (위치 오프셋 꼴) 본다.
실행 (작업공간 루트): python3 <이 파일> <bag>...
"""
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))
BINS = [(0.30, 0.45), (0.45, 0.60), (0.60, 0.75), (0.75, 0.91)]
print("bag | 제어기 | 출처 | y_b [mm] 기준 속도 " + " / ".join(f"{a:.2f}–{b:.2f}" for a, b in BINS) + " | r_in 같은 순서 | y_b = a + b·v 적합 (a mm, b mm/(m/s))")
for b in sys.argv[1:]:
    raw = read(b, ("/state_estimator/odom", "/figure8/path", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose", "/pure_pursuit/status"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    mv = lambda t: (np.interp(t, R[:, 0], R[:, 1]) > 0.05) & (t >= R[0, 0]) & (t <= R[-1, 0])
    tan = np.gradient(path, axis=0); tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    turn = np.sign(np.cross(np.roll(tan, 3, 0), np.roll(tan, -3, 0)))
    O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1]); O = O[mv(O[:, 0])]
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t, m in raw["/tracked_pose"] if s0 <= t <= s1]); S = S[mv(S[:, 0])]
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off)
    ctl = "PP" if raw["/pure_pursuit/status"] else "C"
    for nm, t, P in (("추정기", O[:, 0], O[:, 1:]), ("SLAM 중심", S[:, 0], np.c_[xc, yc])):
        j = np.sqrt(((P[:, None, :] - path[None]) ** 2).sum(-1)).argmin(1)
        lat = 1e3 * ((P - path[j]) * np.c_[-tan[j, 1], tan[j, 0]]).sum(1); tr = turn[j]
        far = np.hypot(*(P - path[0]).T) > 0.15; v = np.interp(t, R[:, 0], R[:, 1])
        yb, ri, vb = [], [], []
        for lo, hi in BINS:
            m = far & (v >= lo) & (v < hi)
            a, c = lat[m & (tr > 0)].mean(), lat[m & (tr < 0)].mean()
            yb.append((a + c) / 2); ri.append((a - c) / 2); vb.append(v[m].mean())
        A = np.c_[np.ones(4), vb]; (a0, b0), *_ = np.linalg.lstsq(A, np.array(yb), rcond=None)
        print(f"{b.split('/')[-1]} | {ctl} | {nm} | " + " / ".join(f"{x:+.1f}" for x in yb) + " | " + " / ".join(f"{x:+.1f}" for x in ri) + f" | a {a0:+.1f}, b {b0:+.1f}")
