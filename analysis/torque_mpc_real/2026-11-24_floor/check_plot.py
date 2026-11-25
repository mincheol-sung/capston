# check_run.npz → fig_fricmodel_check.png + 초 단위 요약:  /usr/bin/python3 check_plot.py
import os
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
S = os.path.dirname(os.path.abspath(__file__))
A = np.load(S + "/check_run.npz")
t_on = float(A["t_on"]); t_off = float(A["t_off"])
ws, mot, st, cap, od, ref = A["ws"], A["mot"], A["st"], A["cap"], A["od"], A["ref"]
W = ["FL", "FR", "RL", "RR"]; col = ["C0", "C1", "C2", "C3"]
fig, ax = plt.subplots(4, 1, figsize=(13, 13), sharex=True)
m = (ws[:,0] > t_on-1) & (ws[:,0] < t_off+3)
for i in range(4): ax[0].plot(ws[m,0]-t_on, ws[m,5+i], col[i], lw=0.8, label=W[i])
ax[0].set_ylabel("wheel speed [rad/s]"); ax[0].legend(ncol=4); ax[0].grid(alpha=0.3)
mm = (mot[:,0] > t_on-1) & (mot[:,0] < t_off+3)
for i in range(4):
    y = np.where(mot[mm,1]==0, mot[mm,2+i], np.nan)
    ax[1].plot(mot[mm,0]-t_on, y, col[i], lw=0.8, label=W[i]+" motor cmd")
    ax[1].plot(cap[:,0]-t_on, cap[:,1+10+i], col[i], lw=0.6, ls="--")
    ax[1].plot(cap[:,0]-t_on, cap[:,1+6+i], col[i], lw=0.6, ls="--")
    ax[1].plot(cap[:,0]-t_on, cap[:,1+15+i], col[i], lw=0.6, ls=":")
ax[1].set_ylabel("torque [N.m]  (dashed: bounds, dotted: model friction)"); ax[1].legend(ncol=4); ax[1].grid(alpha=0.3)
ax[2].plot(st[:,0]-t_on, 1e3*st[:,2], label="ex [mm]"); ax[2].plot(st[:,0]-t_on, 1e3*st[:,3], label="ey [mm]"); ax[2].plot(st[:,0]-t_on, 10*np.degrees(st[:,4]), label="10 x etheta [deg]")
ax[2].legend(ncol=3); ax[2].grid(alpha=0.3); ax[2].set_ylabel("tracking error")
mo = (od[:,0] > t_on-1) & (od[:,0] < t_off+3)
ax[3].plot(od[mo,0]-t_on, od[mo,4], label="est vx body"); ax[3].plot(od[mo,0]-t_on, od[mo,5], label="est vy body")
ax[3].plot(ws[m,0]-t_on, 0.04*ws[m,5:9].mean(1), "k", lw=0.6, label="r x mean wheel speed")
if len(ref): ax[3].plot(ref[:,0]-t_on, np.hypot(ref[:,3], ref[:,4]), "r--", lw=0.8, label="|ref v| (first of horizon)")
for k in range(6): ax[3].plot(st[:,0]-t_on, st[:,1+14+k], lw=0.5, alpha=0.6)
ax[3].legend(ncol=4); ax[3].grid(alpha=0.3); ax[3].set_ylabel("speed [m/s] (+ thin: d-hat x6)"); ax[3].set_xlabel("time since arm [s]")
fig.tight_layout(); fig.savefig(S + "/fig_fricmodel_check.png", dpi=80)
# 수치: 구간별
om = ws[:,5:9].mean(1); tt = ws[:,0]-t_on
print("바퀴 평균 속도 요약 (무장 뒤 초 단위):")
for a in range(0, 19):
    s = (tt >= a) & (tt < a+1)
    ms_ = (mot[:,0]-t_on >= a) & (mot[:,0]-t_on < a+1) & (mot[:,1]==0)
    ss = (st[:,0]-t_on >= a) & (st[:,0]-t_on < a+1)
    if s.any():
        print(f"  {a:2d}~{a+1:2d} s  ω {om[s].mean():6.2f}  모터 명령 평균 {np.round(mot[ms_,2:6].mean(0),3) if ms_.any() else '  토크 모드 아님'}  명령 표준편차 {np.round(mot[ms_,2:6].std(0),3) if ms_.any() else ''}  ex {1e3*st[ss,2].mean() if ss.any() else float('nan'):6.1f} ey {1e3*st[ss,3].mean() if ss.any() else float('nan'):6.1f} mm  stopped {st[ss,1+21].max() if ss.any() else float('nan')}")
# 응답 누락: 같은 값 반복 (오래된 값) 비율
ma = (ws[:,0] >= t_on) & (ws[:,0] <= t_off)
P = ws[ma,1:5]; V = ws[ma,5:9]
print("무장 중 위치·속도가 직전과 완전히 같은 표본 % (움직일 때만):", 100*np.mean((np.diff(P,axis=0)==0)[np.abs(V[1:]).mean(1)>1.0], axis=0))
dtw = np.diff(ws[ma,0]); print(f"wheel_states 주기 중앙 {1e3*np.median(dtw):.1f} ms 최대 {1e3*dtw.max():.1f} ms, 25 ms 넘는 간격 {int((dtw>0.025).sum())} 번")
