# 바퀴가 굴린 거리 / 몸체 거리 (SLAM) 를 가속 (출발 −0.5 s ~ 감속 시작) 과 감속 (감속 시작 ~ 정지 +1 s) 으로 나눠서.
# SLAM 지연 (~0.1 s) 때문에 가속 구간은 모두 > 1, 감속 구간은 모두 < 1 로 치우친다 — 바퀴끼리 견준다.
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r = 0.04
print("bag | 구간 | 바퀴/몸체 [FL FR RL RR]")
for b in sys.argv[1:]:
    raw = read("bags/2026-11-21/"+b, ("/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory","/tracked_pose"))
    s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    R = np.array([[t,np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    mv = np.where(R[:,1]>0.01)[0]; tg, te = R[mv[0],0], R[mv[-1],0]
    top = R[(R[:,0]>=tg)&(R[:,0]<=te)]; tdec = top[np.where(top[:,1]>=0.99)[0][-1],0]
    W = np.array([[t,*m.position] for t,m in raw["/wheel_states"]]); P = np.array([[t,m.pose.position.x,m.pose.position.y] for t,m in raw["/tracked_pose"]])
    at = lambda A,t: A[np.argmin(np.abs(A[:,0]-t)),1:]
    for a,c,nm in ((tg-0.5,tdec,"가속~순항"),(tdec,te+1.0,"감속~정지")):
        dw = r*(at(W,c)-at(W,a)); ds = np.linalg.norm(at(P,c)-at(P,a))
        print(f"{b} | {nm} ({100*ds:.0f} cm) | {np.round(dw/ds,2)}")
