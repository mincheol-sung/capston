# 바퀴가 굴린 거리 (엔코더 각도 × r, 네 바퀴 각각) vs 몸체가 간 거리 (SLAM, 출발 −0.5 s → 정지 뒤) — 적분·지연과 무관한 슬립 총량
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r = 0.04
print("bag | 비교군 accel | 바퀴가 굴린 거리 [FL FR RL RR] cm | 몸체 (SLAM) cm | 바퀴/몸체 [FL FR RL RR] | 0.3 s 시점 바퀴/몸체 (IMU)")
for b in sys.argv[1:]:
    try:
        raw = read("bags/2026-11-21/"+b, ("/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory","/tracked_pose","/imu/base_link"))
        s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    except StopIteration:
        print(f"{b} | 무장 안 됨"); continue
    a = open(f"bags/2026-11-21/{b}/run_info/args.txt").read()
    grp = "A" if "pure_pursuit" in a else "C"; acc = a.split("accel:=")[1].split()[0]
    R = np.array([[t, np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    tg = R[np.argmax(R[:,1]>0.01),0]; te = R[np.where(R[:,1]>0.01)[0][-1],0] + 1.0
    W = np.array([[t,*m.position] for t,m in raw["/wheel_states"]]); P = np.array([[t,m.pose.position.x,m.pose.position.y] for t,m in raw["/tracked_pose"]])
    at = lambda A, t: A[np.argmin(np.abs(A[:,0]-t)), 1:]
    dw = r*(at(W,te) - at(W,tg-0.5)); ds = np.linalg.norm(at(P,te) - at(P,tg-0.5))
    # 0.3 s 시점 (IMU 적분과 엔코더 각도)
    I = np.array([[t,m.linear_acceleration.x] for t,m in raw["/imu/base_link"]]); ba = I[(I[:,0]>tg-1.5)&(I[:,0]<tg-0.6),1].mean()
    k = (I[:,0]>=tg-0.5)&(I[:,0]<=tg+0.3); xI = np.trapz(np.concatenate([[0],np.cumsum(0.5*((I[k,1]-ba)[1:]+(I[k,1]-ba)[:-1])*np.diff(I[k,0]))]), I[k,0])
    dw3 = r*(at(W,tg+0.3) - at(W,tg-0.5))
    print(f"{b} | {grp} {acc} | {np.round(100*dw,1)} | {100*ds:.1f} | {np.round(dw/ds,2)} | {np.round(dw3/xI,2) if xI>0.01 else '-'}")
