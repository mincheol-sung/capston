# 직선 출발 1.2 s: 속도 (기준·바퀴·IMU 적분) / 헤딩 (자이로 적분 vs 바퀴 기구학) / 제어기 토크 바퀴별
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r, k = 0.04, 0.1932; out = sys.argv[1]; bags = sys.argv[2:]
fig, ax = plt.subplots(3, len(bags), figsize=(4.6*len(bags), 9), squeeze=False)
for j, b in enumerate(bags):
    raw = read("bags/2026-11-21/"+b, ("/imu/base_link","/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory","/torque_mpc/cmd_raw","/wheel_pi/cmd_raw"))
    s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    R = np.array([[t,np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    tg = R[np.argmax(R[:,1]>0.01),0]; T = lambda t: t-tg
    I = np.array([[t,m.linear_acceleration.x,m.angular_velocity.z] for t,m in raw["/imu/base_link"]]); pre=(I[:,0]>tg-1.2)&(I[:,0]<tg-0.2)
    ba, bg = I[pre,1].mean(), I[pre,2].mean(); kI=(I[:,0]>=tg-0.05)&(I[:,0]<=tg+1.2)
    vI = np.concatenate([[0],np.cumsum(0.5*((I[kI,1]-ba)[1:]+(I[kI,1]-ba)[:-1])*np.diff(I[kI,0]))])
    yI = np.degrees(np.concatenate([[0],np.cumsum(0.5*((I[kI,2]-bg)[1:]+(I[kI,2]-bg)[:-1])*np.diff(I[kI,0]))]))
    W = np.array([[t,*m.velocity] for t,m in raw["/wheel_states"]]); kW=(W[:,0]>=tg-0.05)&(W[:,0]<=tg+1.2)
    yW = np.degrees(np.concatenate([[0],np.cumsum(np.diff(W[kW,0])*0.5*((r*(-W[kW,1]+W[kW,2]-W[kW,3]+W[kW,4])/(4*k))[1:]+(r*(-W[kW,1]+W[kW,2]-W[kW,3]+W[kW,4])/(4*k))[:-1]))]))
    top = "/torque_mpc/cmd_raw" if raw["/torque_mpc/cmd_raw"] else "/wheel_pi/cmd_raw"
    U = np.array([[t,*m.torque] for t,m in raw[top]])
    acc = open(f"bags/2026-11-21/{b}/run_info/args.txt").read().split("accel:=")[1].split()[0]
    nm = ("C" if "torque_mpc" in top else "A") + f" {b} accel {acc}"
    A=ax[0,j]; A.plot(T(R[:,0]),R[:,1],"k",label="ref"); A.plot(T(W[kW,0]),r*W[kW,1:5].mean(1),label="wheel"); A.plot(T(I[kI,0]),vI,label="IMU"); A.set_xlim(-.1,1.2); A.set_title(nm+"  v [m/s]"); A.legend(fontsize=7); A.grid(alpha=.3)
    A=ax[1,j]; A.plot(T(I[kI,0]),yI,label="gyro"); A.plot(T(W[kW,0]),yW,label="wheel kin."); A.set_xlim(-.1,1.2); A.set_title("heading change [deg]"); A.legend(fontsize=7); A.grid(alpha=.3)
    A=ax[2,j]
    for i,n in enumerate(("FL","FR","RL","RR")): A.plot(T(U[:,0]),U[:,1+i],label=n)
    A.set_xlim(-.1,1.2); A.set_title(top+" [N·m]"); A.legend(fontsize=7); A.grid(alpha=.3); A.set_xlabel("t after start [s]")
fig.tight_layout(); fig.savefig(out, dpi=90); print(out)
