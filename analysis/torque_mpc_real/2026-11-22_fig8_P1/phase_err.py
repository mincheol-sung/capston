# P1 (sin 속도 변조 8자): 기준 접선 가속 부호로 구간을 나눠 오차 — 가속 (a_ref > 0.1) / 감속 (< −0.1) / 그 밖, 주행 중만
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
print("bag | 구간 | 시간 % | 시간 기준 RMS [cm] | lag 평균 (+ 뒤처짐) | 원 안쪽 평균 | 헤딩 RMS [°] | 헤딩 평균 (+ 덜 돎)")
for b in sys.argv[1:]:
    raw = read(b, ("/torque_mpc/status","/pure_pursuit/status","/mpc/armed","/dra_mppi/reference_trajectory"))
    s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    R = np.array([[t, np.hypot(x.vx,x.vy), x.yaw, x.yaw_rate] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    mv = R[:,1] > 0.05; t0, t1 = R[mv,0][0], R[mv,0][-1]
    k = 5; a = np.gradient(np.convolve(R[:,1], np.ones(k)/k, "same"), R[:,0])
    st = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    X = np.array([[t,*m.data[1:4]] for t,m in raw[st] if t0+1.0<=t<=t1-1.0])         # 출발·정지 1 s 뺌
    ar = np.interp(X[:,0], R[:,0], a); yaw = np.interp(X[:,0], R[:,0], np.unwrap(R[:,2])); wr = np.interp(X[:,0], R[:,0], R[:,3])
    lag = np.cos(yaw)*X[:,1]+np.sin(yaw)*X[:,2]; inn = -(-np.sin(yaw)*X[:,1]+np.cos(yaw)*X[:,2])*np.sign(wr)
    th = np.degrees(np.arctan2(np.sin(X[:,3]),np.cos(X[:,3]))); e = np.hypot(X[:,1],X[:,2])
    for nm, m in (("가속", ar > 0.1), ("감속", ar < -0.1), ("그 밖", np.abs(ar) <= 0.1)):
        print(f"{b.split('/')[-1]} {st.split('/')[1][:5]} | {nm} | {100*m.mean():.0f} | {100*np.sqrt((e[m]**2).mean()):.1f} | {100*lag[m].mean():+.1f} | {100*inn[m].mean():+.1f} | {np.sqrt((th[m]**2).mean()):.1f} | {(th[m]*np.sign(wr[m])).mean():+.1f}")
