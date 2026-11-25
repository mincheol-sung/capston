# 눈에 띄는 것: ① 교차점(경로 출발점 0.15 m 안) 오차 ② 흔들림 = 추정기 ω_z·몸체 v_y 의 2 Hz 위 성분 RMS (0.5 s 이동평균을 뺀 나머지) ③ 바퀴 토크 Δτ
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
print("bag | 교차점 근처 경로 거리 RMS / 최대 | 그 밖 RMS | ω_z 흔들림 RMS [rad/s] | v_y(몸체) 흔들림 RMS [m/s] | 경로 거리 2 Hz 위 RMS [cm]")
for b in sys.argv[1:]:
    raw = read(b if "/" in b else "bags/2026-11-21/"+b, ("/state_estimator/odom","/figure8/path","/mpc/armed","/dra_mppi/reference_trajectory"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= s1][-1].poses])
    R = np.array([[t, np.hypot(r.vx, r.vy)] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.y, m.twist.twist.angular.z] for t, m in raw["/state_estimator/odom"] if s0 <= t <= s1])
    O = O[(np.interp(O[:,0], R[:,0], R[:,1]) > 0.05) & (O[:,0] <= R[-1,0])]
    D = np.sqrt(((O[:, None, 1:3] - path[None]) ** 2).sum(-1)); d = D.min(1)
    near = np.hypot(*(O[:, 1:3] - path[0]).T) < 0.15
    hp = lambda x: x - np.convolve(x, np.ones(51) / 51, "same")
    k = slice(25, -25)
    r = lambda a: np.sqrt((a ** 2).mean())
    print(f"{b} | {100*r(d[near]):.1f} / {100*d[near].max():.1f} cm ({near.mean()*100:.0f} % 시간) | {100*r(d[~near]):.1f} | {r(hp(O[:,4])[k]):.3f} | {r(hp(O[:,3])[k]):.3f} | {100*r(hp(d)[k]):.2f}")
