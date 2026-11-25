# 확인 주행 (check_run.npz) 의 정지 구간: 외란 추정이 얼마나 쌓였나, 그동안 로봇이 움직였나, 멈춤 조건이 언제 참인가
#   source install/setup.bash 뒤  /usr/bin/python3 freeze_cost.py
# /torque_mpc/status 17–19 ÷ Ts = 외란 추정 가속 (몸체 좌표). 20 = hold 래치, 21 = stopped (몸체 속도 < hold.stopped_speed 0.04 m/s)
# 외란 추정이 시키는 토크 = pinv(G(0))·(−가속)  (최소 노름 환산 — MPC 가 실제로 고른 값과 null 성분만큼 다를 수 있다)
import os
import numpy as np
from amr_mpc.torque.model import TorquePlant

np.set_printoptions(precision=1, suppress=True, linewidth=200)
A = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_run.npz"))
st, ws, od, sl, ref = A["st"], A["ws"], A["od"], A["sl"], A["ref"]
t_on, t_off = float(A["t_on"]), float(A["t_off"])
Ts = 0.01
P = TorquePlant.from_yaml()
Gp = np.linalg.pinv(P.G(0.0))                       # 4×3: 몸체 가속 → 바퀴 토크 (최소 노름)
print("몸체 가속 1 단위가 바퀴 토크로 [FL FR RL RR] mN·m:  앞", Gp[:, 0] * 1e3, " 옆", Gp[:, 1] * 1e3, " 회전", Gp[:, 2] * 1e3)

m = (st[:, 0] >= t_on) & (st[:, 0] <= t_off)
S = st[m]; t = S[:, 0] - t_on; D = S[:, 1:]
ad = D[:, 17:20] / Ts                               # 외란 추정 가속 [앞, 옆, 회전]
tau = D[:, 4:8]; hold, stopped = D[:, 20], D[:, 21]
tau_d = -(Gp @ ad.T).T                              # 외란 추정이 시키는 토크
om = np.column_stack([np.interp(S[:, 0], ws[:, 0], ws[:, 5 + i]) for i in range(4)])
ang = np.column_stack([np.interp(S[:, 0], ws[:, 0], ws[:, 1 + i]) for i in range(4)])
vb = np.hypot(np.interp(S[:, 0], od[:, 0], od[:, 4]), np.interp(S[:, 0], od[:, 0], od[:, 5]))
vref = np.interp(S[:, 0], ref[:, 0], np.hypot(ref[:, 3], ref[:, 4]))
slx, sly = np.interp(S[:, 0], sl[:, 0], sl[:, 1]), np.interp(S[:, 0], sl[:, 0], sl[:, 2])
e_mm, eth = D[:, 1:3] * 1e3, np.degrees(D[:, 3])

print(f"\n무장 {t[-1]:.1f} s.  stopped 깃발 참 {100*stopped.mean():.1f} %,  hold 래치 참 {100*hold.mean():.1f} %  (hold 진입 조건: 오차 < 10 mm · 2°)")
print("\n| 무장 뒤 s | 기준 속도 m/s | 몸체 속도 m/s | 바퀴 속도 rad/s | stopped % | 오차 x · y mm · 회전 ° | 외란 추정 앞 · 옆 m/s² · 회전 rad/s² | 명령 FL FR RL RR mN·m | 그중 외란 추정 몫 FL FR RL RR |")
print("|---|---|---|---|---|---|---|---|---|")
for a in np.arange(0.0, t[-1], 0.5):
    s = (t >= a) & (t < a + 0.5)
    if s.sum() < 5: continue
    print(f"| {a:.1f} | {vref[s].mean():.2f} | {vb[s].mean():.3f} | {om[s].mean():+.2f} | {100*stopped[s].mean():.0f} | {e_mm[s,0].mean():+.0f} · {e_mm[s,1].mean():+.0f} · {eth[s].mean():+.2f} | "
          f"{ad[s,0].mean():+.2f} · {ad[s,1].mean():+.2f} · {ad[s,2].mean():+.2f} | {tau[s].mean(0)*1e3} | {tau_d[s].mean(0)*1e3} |")

# 정지 구간: 네 바퀴 모두 |속도| < 0.3 rad/s 가 0.3 s 넘게
still = (np.abs(om) < 0.3).all(1)
edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
print("\n정지 구간별 (네 바퀴 |속도| < 0.3 rad/s):")
for i0, i1 in zip(edges[::2], edges[1::2]):
    if t[i1 - 1] - t[i0] < 0.3: continue
    a, b = slice(i0, i0 + 10), slice(i1 - 10, i1)
    roll = (ang[b].mean(0) - ang[a].mean(0)) * P.r * 1e3
    dsl = np.hypot(slx[b].mean() - slx[a].mean(), sly[b].mean() - sly[a].mean()) * 1e3
    print(f"  {t[i0]:.2f} ~ {t[i1-1]:.2f} s ({t[i1-1]-t[i0]:.2f} s)   기준 속도 평균 {vref[i0:i1].mean():.3f}   stopped 참 {100*stopped[i0:i1].mean():.0f} %   hold 참 {100*hold[i0:i1].mean():.0f} %")
    print(f"    오차 x·y mm · 회전 °   처음 {e_mm[a].mean(0)} {eth[a].mean():+.2f}  →  끝 {e_mm[b].mean(0)} {eth[b].mean():+.2f}")
    print(f"    바퀴가 굴러간 거리 mm [FL FR RL RR] {roll}   SLAM 위치 변화 {dsl:.1f} mm")
    print(f"    외란 추정 가속   처음 {ad[a].mean(0).round(2)}  →  끝 {ad[b].mean(0).round(2)}")
    print(f"    명령 토크 mN·m   처음 {tau[a].mean(0)*1e3}  →  끝 {tau[b].mean(0)*1e3}")
    print(f"    외란 추정 몫     처음 {tau_d[a].mean(0)*1e3}  →  끝 {tau_d[b].mean(0)*1e3}")
    print(f"    나머지 (피드백)  처음 {(tau[a]-tau_d[a]).mean(0)*1e3}  →  끝 {(tau[b]-tau_d[b]).mean(0)*1e3}")

# 멈춤 조건 후보: 기준 속도 0 그리고 stopped 깃발
cond = (vref < 1e-3) & (stopped > 0.5)
mov = np.abs(om).mean(1) > 1.0
print(f"\n멈춤 조건 (기준 속도 0 · stopped) 참인 시간 {cond.sum()*Ts:.2f} s / {len(t)*Ts:.2f} s.  그중 바퀴가 1 rad/s 넘게 돌던 스텝 {int((cond & mov).sum())}")
print(f"달리는 중 (바퀴 > 1 rad/s) 몸체 속도 최소 {vb[mov].min():.3f} m/s,  정지 구간 몸체 속도 최대 {vb[still].max():.3f} m/s  (문턱 0.04)")

# 검산: 정지 중에는 잰 가속이 0 이라 잔차 = −G·(낸 토크).  낸 토크 = 피드백 + 외란 추정 몫 (G·몫 = −외란 추정) 이므로
#   식-11 은  d̂(k+1) = d̂(k) − α·G·τ_피드백  →  변화율 = −(α/Ts)·G·τ_피드백  (α/Ts = 1 /s).  외란 추정이 "피드백의 적분기" 가 된다.
G0 = P.G(0.0)
f3 = lambda v: '[' + ' '.join(f'{x:+.3f}' for x in v) + ']'
for lab, a, b in (("마지막 정지, 상한 닿기 전", 10.5, 12.0), ("마지막 정지, 상한 닿은 뒤 (앞·옆은 잘림)", 12.1, 15.5)):
    s = (t >= a) & (t <= b)
    pred = -(0.01 / Ts) * (G0 @ (tau[s] - tau_d[s]).mean(0))
    obs = np.array([np.polyfit(t[s], ad[s, i], 1)[0] for i in range(3)])
    print(f"{lab} {a}~{b} s: 외란 추정 변화율 [앞 옆 m/s² 매초, 회전 rad/s² 매초]  식으로 {f3(pred)}   기록에서 {f3(obs)}")
print(f"외란 추정 앞·옆 크기 (상한 0.5): 12.0 s {np.hypot(ad[:,0],ad[:,1])[np.searchsorted(t,12.0)]:.3f},  끝 {np.hypot(ad[:,0],ad[:,1])[-1]:.3f},  0.49 넘은 첫 시각 {t[np.argmax(np.hypot(ad[:,0],ad[:,1])>0.49)]:.2f} s")
for lab, tt in (("가운데 쉼 시작 6.27", 6.27), ("가운데 쉼 끝 7.43", 7.43), ("마지막 정지 시작 10.22", 10.22), ("끝", t[-1] - 0.05)):
    i = np.searchsorted(t, tt); s = slice(max(i - 5, 0), i + 5)
    print(f"{lab} s: 외란 추정 {f3(ad[s].mean(0))}  몫 [FL FR RL RR] mN·m {(tau_d[s].mean(0)*1e3).round(1)}  명령 {(tau[s].mean(0)*1e3).round(1)}")
