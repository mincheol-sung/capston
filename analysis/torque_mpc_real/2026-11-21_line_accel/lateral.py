# 직선에서 옆으로 벗어나는 원인: 옆 위치 (SLAM, 출발 방향 기준) · 헤딩 변화 (IMU 자이로 적분) · 바퀴별 ω 와 평균의 차 (대각선 쌍)
#   메카넘 기구학: v_y ∝ (−ω_FL + ω_FR + ω_RL − ω_RR)/4,  ω_z ∝ (−ω_FL + ω_FR − ω_RL + ω_RR)/4  (X 배치, 몸체 기준)
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r = 0.04
for b in sys.argv[1:]:
    raw = read("bags/2026-11-21/"+b, ("/imu/base_link","/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory","/tracked_pose","/pure_pursuit/status"))
    s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    R = np.array([[t, np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    tg = R[np.argmax(R[:,1]>0.01),0]; tend = R[np.where(R[:,1]>0.01)[0][-1],0]
    P = np.array([[t, m.pose.position.x, m.pose.position.y, 2*np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t,m in raw["/tracked_pose"]])
    i0 = np.argmin(np.abs(P[:,0]-tg)); p0 = P[i0,1:3]; th0 = P[i0,3]; u = np.array([np.cos(th0), np.sin(th0)]); n = np.array([-u[1], u[0]])
    G = np.array([[t, m.angular_velocity.z] for t,m in raw["/imu/base_link"]]); gb = G[(G[:,0]>tg-1.2)&(G[:,0]<tg-0.2),1].mean()
    W = np.array([[t,*m.velocity] for t,m in raw["/wheel_states"]])
    St = np.array([[t,*m.data[1:4]] for t,m in raw["/pure_pursuit/status"]]) if raw["/pure_pursuit/status"] else None
    print(f"== {b}")
    for t_rel in (0.15, 0.3, 0.45, 0.7, 1.0, (tend-tg), (tend-tg)+1.0):
        t = tg + t_rel
        p = np.array([np.interp(t, P[:,0], P[:,1]), np.interp(t, P[:,0], P[:,2])]) - p0
        k = (G[:,0]>=tg)&(G[:,0]<=t); dth = np.degrees(np.trapz(G[k,1]-gb, G[k,0])) if k.sum()>1 else 0
        kw = (W[:,0]>=tg)&(W[:,0]<=t)
        vy_w = r*np.trapz((-W[kw,1]+W[kw,2]+W[kw,3]-W[kw,4])/4, W[kw,0]) if kw.sum()>1 else 0   # 바퀴가 "말한" 옆 이동 [m]
        print(f"   t {t_rel:4.2f} s: 진행 {p@u:+.3f} m, 옆 {100*(p@n):+5.1f} cm | 헤딩 변화 (자이로) {dth:+5.1f}° | 바퀴 기구학 옆 이동 누적 {100*vy_w:+5.1f} cm")
