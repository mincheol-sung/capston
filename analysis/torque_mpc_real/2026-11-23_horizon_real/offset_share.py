"""몸체 왼쪽 치우침 y_b 가 경로 거리에서 차지하는 몫 (추정기 = 제어기가 보는 위치, 로봇 중심). body_lateral.py 와 같은 분해:
   lat = 경로 왼쪽 법선 성분, y_b = (평균 lat_CCW + 평균 lat_CW)/2, r_in = (평균 lat_CCW − 평균 lat_CW)/2. 교차점 0.15 m 안 뺌.
   RMS(lat) ≈ 경로 거리 RMS. y_b 를 뺀 RMS, 원마다 평균 (y_b ± r_in) 을 뺀 RMS (= 흔들림·구간 변화만) 를 같이.
실행 (작업공간 루트): python3 <이 파일> <bag>...
"""
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))
print("bag | 제어기 | R m | y_b 추정기 / SLAM 중심 [mm] | r_in 추정기 [mm] | lat RMS [mm] | y_b 뺀 RMS | 원마다 평균 뺀 RMS | y_b 몫 (평균제곱 %)")
for b in sys.argv[1:]:
    try:
        raw = read(b, ("/state_estimator/odom", "/figure8/path", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose", "/pure_pursuit/status"))
        s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    except (StopIteration, IndexError) as e:
        print(f"{b.split('/')[-1]} | 건너뜀 ({type(e).__name__})"); continue
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    mv = lambda t: (np.interp(t, R[:, 0], R[:, 1]) > 0.05) & (t >= R[0, 0]) & (t <= R[-1, 0])
    tan = np.gradient(path, axis=0); tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    turn = np.sign(np.cross(np.roll(tan, 3, 0), np.roll(tan, -3, 0)))
    curv = np.abs(np.cross(tan[:-1], tan[1:])) / np.linalg.norm(np.diff(path, axis=0), axis=1)
    O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1]); O = O[mv(O[:, 0])]
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t, m in raw["/tracked_pose"] if s0 <= t <= s1]); S = S[mv(S[:, 0])]
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off)
    out = {}
    for nm, P in (("E", O[:, 1:]), ("S", np.c_[xc, yc])):
        j = np.sqrt(((P[:, None, :] - path[None]) ** 2).sum(-1)).argmin(1)
        lat = 1e3 * ((P - path[j]) * np.c_[-tan[j, 1], tan[j, 0]]).sum(1); tr = turn[j]; far = np.hypot(*(P - path[0]).T) > 0.15
        lat, tr = lat[far], tr[far]
        a, c = lat[tr > 0].mean(), lat[tr < 0].mean(); yb, ri = (a + c) / 2, (a - c) / 2
        circ = np.where(tr > 0, a, c)
        out[nm] = (yb, ri, np.sqrt((lat ** 2).mean()), np.sqrt(((lat - yb) ** 2).mean()), np.sqrt(((lat - circ) ** 2).mean()))
    yb, ri, r0, r1, r2 = out["E"]
    ctl = "PP" if raw["/pure_pursuit/status"] else "C"
    print(f"{b.split('/')[-2][-5:]}/{b.split('/')[-1]} | {ctl} | {1/np.median(curv[curv > 0.3]):.2f} | {yb:+.1f} / {out['S'][0]:+.1f} | {ri:+.1f} | {r0:.1f} | {r1:.1f} | {r2:.1f} | {100*(1 - r1**2/r0**2):.0f}")
