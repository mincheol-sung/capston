# 출발 전후 50 ms 칸: 기준 속도 · MPC/PI 출력 · 최종 토크 · 바퀴 속도·위치 변화 (엔코더 각도로 바퀴가 실제로 돈 거리) · 몸체 (IMU 적분, SLAM)
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r = 0.04
for b in sys.argv[1:]:
    raw = read("bags/2026-11-21/"+b, ("/imu/base_link","/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory","/torque_mpc/cmd_raw","/wheel_pi/cmd_raw","/wheel_torque_cmd","/tracked_pose","/torque_mpc/status"))
    arm = [(t,m.data) for t,m in raw["/mpc/armed"]]; s0 = next(t for t,d in arm if d)
    R = np.array([[t, np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    Rh = [(t, m) for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references)]
    tg = R[np.argmax(R[:,1]>0.01),0]
    # 기준열 미리보기: 각 시각 기준열 안에서 속도가 처음 0.01 넘는 행이 몇 s 뒤인가
    I = np.array([[t,m.linear_acceleration.x] for t,m in raw["/imu/base_link"]]); ba = I[(I[:,0]>tg-1.5)&(I[:,0]<tg-0.6),1].mean()
    k = I[:,0]>=tg-0.6; vI = np.concatenate([[0],np.cumsum(0.5*((I[k,1]-ba)[1:]+(I[k,1]-ba)[:-1])*np.diff(I[k,0]))])
    W = np.array([[t,*m.position,*m.velocity] for t,m in raw["/wheel_states"]])
    top = "/torque_mpc/cmd_raw" if raw["/torque_mpc/cmd_raw"] else "/wheel_pi/cmd_raw"
    U = np.array([[t,*m.torque] for t,m in raw[top]]); Fq = np.array([[t,*m.torque] for t,m in raw["/wheel_torque_cmd"]])
    P = np.array([[t,m.pose.position.x,m.pose.position.y] for t,m in raw["/tracked_pose"]])
    hold = np.array([[t,m.data[20]] for t,m in raw["/torque_mpc/status"]]) if raw["/torque_mpc/status"] else None
    print(f"== {b} ({'C' if 'mpc' in top else 'A'})  무장 {s0-tg:+.2f} s, 기준 출발 0 s   [바퀴 FL FR RL RR]")
    p_w0 = W[np.argmin(np.abs(W[:,0]-(tg-0.4))),1:5]; pS0 = P[np.argmin(np.abs(P[:,0]-(tg-0.4))),1:]
    for c in np.arange(-0.4, 0.31, 0.05):
        t = tg + c
        def at(A, cols): return A[np.argmin(np.abs(A[:,0]-t)), cols]
        u = at(U, slice(1,5)); f = at(Fq, slice(1,5)); w = at(W, slice(5,9)); dp = r*(at(W, slice(1,5)) - p_w0)
        vb = np.interp(t, I[k,0], vI); h = at(hold, 1) if hold is not None else np.nan
        print(f"  {c:+.2f} | ref {np.interp(t,R[:,0],R[:,1]):.2f} | hold {h:.0f} | 출력 {np.round(u,3)} | 최종 {np.round(f,3)} | 바퀴 r·ω {np.round(r*w,2)} | 바퀴가 굴린 거리 {np.round(100*dp,1)} cm | 몸체 v {vb:.2f}")
