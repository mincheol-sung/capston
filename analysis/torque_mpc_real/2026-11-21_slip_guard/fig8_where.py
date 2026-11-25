# 8자 bag 에 감지 규칙 (문턱 0.35 · 50 ms) — 어느 바퀴가, 어느 랩·교차점 근처·가감속 중 판정되나. 판정된 순간 그 바퀴 σ 크기도.
import sys, os, numpy as np
os.environ.setdefault("TH", "0.35"); os.environ.setdefault("MINON", "5")
argv = sys.argv; sys.argv = [argv[0]]
src = open("analysis/torque_mpc_real/2026-11-21_slip_guard/detect_rule.py").read().split('print("bag')[0]
exec(src); sys.argv = argv
print("bag | 설명 | 판정 % [FL FR RL RR] | 랩별 판정 % (0.43 / 0.61 / 0.80) | 교차점 0.15 m 안 몫 | 판정 중 σ 중앙값")
for b, nm in [x.split(":") for x in sys.argv[1:]]:
    t, sig, E, T = run(b)
    f = rule(t, sig)
    raw = read("bags/2026-11-21/"+b, ("/mpc/armed","/dra_mppi/reference_trajectory","/figure8/path","/state_estimator/odom"))
    s0 = next(tt for tt,m in raw["/mpc/armed"] if m.data)
    R = np.array([[tt,np.hypot(x.vx,x.vy)] for tt,m in raw["/dra_mppi/reference_trajectory"] if tt>=s0 and len(m.references) for x in [m.references[0]]])
    tg = R[np.argmax(R[:,1]>0.01),0]
    vr = np.interp(t+tg, R[:,0], R[:,1])
    path = np.array([[p.pose.position.x,p.pose.position.y] for p in raw["/figure8/path"][-1][1].poses])
    O = np.array([[tt,m.pose.pose.position.x,m.pose.pose.position.y] for tt,m in raw["/state_estimator/odom"]])
    px = np.interp(t+tg,O[:,0],O[:,1]); py = np.interp(t+tg,O[:,0],O[:,2]); near = np.hypot(px-path[0,0],py-path[0,1]) < 0.15
    anyf = f.any(1)
    laps = [100*anyf[np.abs(vr-v0)<0.015].mean() if (np.abs(vr-v0)<0.015).any() else np.nan for v0 in (0.43,0.61,0.80)]
    sel = sig[f]
    print(f"{b} | {nm} | {(100*f.mean(0)).round(1)} | {np.round(laps,1)} | {100*(anyf&near).sum()/max(1,anyf.sum()):.0f} % (교차점 시간 몫 {100*near.mean():.0f} %) | {np.median(sel):.2f}" if len(sel) else f"{b} | {nm} | 판정 없음")
