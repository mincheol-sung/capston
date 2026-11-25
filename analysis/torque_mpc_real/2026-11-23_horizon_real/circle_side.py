"""8자 두 원별 안쪽(+)/바깥(−) 치우침 — 위치 출처 3 가지: 추정기 (/state_estimator/odom, 로봇 중심 = RViz 궤적),
SLAM 중심 (/tracked_pose → compare_slam.imu_to_center), SLAM IMU 점 (/tracked_pose 그대로).
치우침 = 가장 가까운 경로 점에서 경로 왼쪽 법선 성분 × 그 점의 회전 방향 부호 (+ = 원 안쪽). 교차점 0.15 m 안은 뺀다.
창 = 무장 & 기준 속도 > 0.05 m/s (moving_only.py 와 같음). 실행 (작업공간 루트): python3 <이 파일> <bag>...
"""
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))


def side(P, path, tan, turn):
    j = np.sqrt(((P[:, None, :] - path[None]) ** 2).sum(-1)).argmin(1)
    nrm = np.c_[-tan[j, 1], tan[j, 0]]
    lat = ((P - path[j]) * nrm).sum(1)
    return lat * turn[j], turn[j], np.hypot(*(P - path[0]).T) > 0.15


print("bag | 제어기 | 경로 진행 방향 확인 | 왼쪽 회전 원 (CCW) 안쪽 평균 [mm]: 추정기 / SLAM 중심 / SLAM IMU 점 | 오른쪽 회전 원 (CW): 같은 순서 | R [m]")
for b in sys.argv[1:]:
    raw = read(b, ("/state_estimator/odom", "/figure8/path", "/mpc/armed", "/dra_mppi/reference_trajectory", "/tracked_pose", "/pure_pursuit/status"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    mv = lambda t: (np.interp(t, R[:, 0], R[:, 1]) > 0.05) & (t >= R[0, 0]) & (t <= R[-1, 0])
    tan = np.gradient(path, axis=0); tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    k = 3; turn = np.sign(np.cross(np.roll(tan, k, 0), np.roll(tan, -k, 0)))           # + = 왼쪽 회전
    curv = np.abs(np.cross(tan[:-1], tan[1:])) / np.linalg.norm(np.diff(path, axis=0), axis=1)
    O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1]); O = O[mv(O[:, 0])]
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t, m in raw["/tracked_pose"] if s0 <= t <= s1]); S = S[mv(S[:, 0])]
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off)
    # 경로 점 순서 = 진행 방향인지: 추정기 이동 방향과 접선의 내적이 양수인 비율
    j = np.sqrt(((O[:-1, None, 1:] - path[None]) ** 2).sum(-1)).argmin(1)
    fwd = 100 * ((np.diff(O[:, 1:], axis=0) * tan[j]).sum(1) > 0).mean()
    res = {}
    for nm, P in (("E", O[:, 1:]), ("Sc", np.c_[xc, yc]), ("Si", S[:, 1:3])):
        inn, tr, far = side(P, path, tan, turn)
        res[nm] = [1e3 * inn[far & (tr > 0)].mean(), 1e3 * inn[far & (tr < 0)].mean()]
    ctl = "PP" if raw["/pure_pursuit/status"] else "C"
    print(f"{b.split('/')[-1]} | {ctl} | {fwd:.0f} % | {res['E'][0]:+.1f} / {res['Sc'][0]:+.1f} / {res['Si'][0]:+.1f} | {res['E'][1]:+.1f} / {res['Sc'][1]:+.1f} / {res['Si'][1]:+.1f} | {1/np.median(curv[curv > 0.5]):.2f}")
R0 = (0.3, 0.5)
for r in R0:
    print(f"계산 (경로 정확히 따를 때 IMU 점 안쪽 치우침, R {r}): 왼쪽 원 {-1e3*(np.hypot(r-off[1], off[0])-r):+.1f} mm, 오른쪽 원 {-1e3*(np.hypot(r+off[1], off[0])-r):+.1f} mm")
