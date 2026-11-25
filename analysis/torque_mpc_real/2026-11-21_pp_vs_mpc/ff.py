# 랩 순항(원 위)에서 바퀴 토크: A 는 /wheel_pi/cmd_raw, C 는 /torque_mpc/status 의 τ(4~7) 와 앞먹임 U_s(22~25). 원 방향(ω_r 부호)별로 평균
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
for b in sys.argv[1:]:
    raw = read(b, ("/torque_mpc/status","/wheel_pi/cmd_raw","/mpc/armed","/dra_mppi/reference_trajectory"))
    s0 = next(t for t, m in raw["/mpc/armed"] if m.data)
    R = np.array([[t, np.hypot(r.vx, r.vy), r.yaw_rate] for t, m in raw["/dra_mppi/reference_trajectory"] if t >= s0 and len(m.references) for r in [m.references[0]]])
    if raw["/wheel_pi/cmd_raw"]:
        T = np.array([[t, *m.torque] for t, m in raw["/wheel_pi/cmd_raw"] if t >= s0]); U = None; name = "A τ_PI"
    else:
        X = np.array([[t, *m.data[4:8], *m.data[22:26]] for t, m in raw["/torque_mpc/status"] if t >= s0 and len(m.data) >= 26 and m.data[20] == 0])
        T = X[:, :5]; U = X[:, 5:]; name = "C τ"
    vr = np.interp(T[:,0], R[:,0], R[:,1]); wr = np.interp(T[:,0], R[:,0], R[:,2])
    print(b, name, "  [FL FR RL RR] N·m, 원 위 순항 평균")
    for v0 in (0.43, 0.61):
        for sg, nm in ((1, "왼원(반시계)"), (-1, "오른원(시계)")):
            m = (np.abs(vr - v0) < 0.015) & (np.sign(wr) == sg) & (np.abs(wr) > 0.3)
            line = f"   {v0:.2f} {nm}: τ {T[m,1:].mean(0).round(4)}"
            if U is not None:
                line += f"  U_s {U[m].mean(0).round(4)}  τ−U_s {(T[m,1:]-U[m]).mean(0).round(4)}  |τ−U_s| RMS / |U_s| RMS {np.sqrt(((T[m,1:]-U[m])**2).mean()):.4f} / {np.sqrt((U[m]**2).mean()):.4f}"
            print(line)
