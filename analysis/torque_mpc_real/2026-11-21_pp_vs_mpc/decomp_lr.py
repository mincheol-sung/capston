# 시간 기준 오차 Xe = X_r − X (status, 월드) 를 기준 접선 t·바깥 법선 n 으로 돌려 랩 순항별 평균(편향)·표준편차(진동)
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
for b in sys.argv[1:]:
    raw = read(b, ("/torque_mpc/status","/pure_pursuit/status","/mpc/armed","/dra_mppi/reference_trajectory"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data); s1 = next((t for t, m in raw["/mpc/armed"] if not m.data and t > s0), 1e18)
    st = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    St = np.array([[t, *m.data[1:4]] for t, m in raw[st] if s0 <= t <= s1])
    R = np.array([[t, r.yaw, np.hypot(r.vx, r.vy), r.yaw_rate] for t, m in raw["/dra_mppi/reference_trajectory"] if s0 <= t <= s1 and len(m.references) for r in [m.references[0]]])
    yaw = np.interp(St[:,0], R[:,0], np.unwrap(R[:,1])); vr = np.interp(St[:,0], R[:,0], R[:,2]); wr = np.interp(St[:,0], R[:,0], R[:,3])
    c, s = np.cos(yaw), np.sin(yaw)
    # Xe = X_r − X.  along_lag = +t·Xe (양수 = 로봇이 기준보다 뒤), 왼쪽 = n_L·Xe (양수 = 로봇이 기준 오른쪽)
    lag = c*St[:,1] + s*St[:,2]; left = -s*St[:,1] + c*St[:,2]
    inn = -left*np.sign(wr)       # left>0 = 로봇이 기준 오른쪽. 원 중심이 왼쪽(ω>0)이면 그건 바깥 → 부호 뒤집어 양수 = 원 안쪽
    th = np.degrees(np.arctan2(np.sin(St[:,3]), np.cos(St[:,3])))
    print(b.split('/')[-1], st.split('/')[1])
    for v0, sgn in [(v, g) for v in (0.43, 0.61, 0.80) for g in (1, -1)]:
        m = (np.abs(vr - v0) < 0.015) & (np.abs(wr) > 0.3) & (np.sign(wr) == sgn)
        if m.sum() < 20: continue
        print(f"   {v0:.2f} {"왼원" if sgn>0 else "오른원"}: lag 평균 {100*lag[m].mean():+5.1f} σ {100*lag[m].std():4.1f} | 안쪽 평균 {100*inn[m].mean():+5.1f} σ {100*inn[m].std():4.1f} cm | e_θ 평균 {(th[m]*np.sign(wr[m])).mean():+5.1f}° σ {th[m].std():4.1f}°  ({m.sum()} 행)")
