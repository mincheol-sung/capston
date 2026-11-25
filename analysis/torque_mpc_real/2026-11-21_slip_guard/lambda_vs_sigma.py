# 같은 bag 에서 두 감지 방식 비교 (제어에 안 씀)
#   λ 방식: /slip_logger/slip 의 λ_i (ω_nom = T⁻¹·v̂, v̂ = 상태 추정기 몸체 속도) — |λ_i| > TH_L 가 50 ms 지속이면 판정 (바퀴마다 따로)
#   σ 방식: slip_guard 의 leave-one-out 잔차 (자이로 + 다른 세 바퀴) — σ 가장 큰 바퀴가 0.35 를 50 ms (detect_rule.py 와 같다)
import sys, os, numpy as np
os.environ.setdefault("TH", "0.35"); os.environ.setdefault("MINON", "5")
argv = sys.argv; sys.argv = [argv[0]]
exec(open("analysis/torque_mpc_real/2026-11-21_slip_guard/detect_rule.py").read().split('print("bag')[0]); sys.argv = argv
TH_L = float(os.environ.get("THL", "0.3"))
def persist(x, n=5):
    out = np.zeros_like(x, bool); run = np.zeros(x.shape[1], int)
    for k in range(len(x)):
        run = np.where(x[k], run + 1, 0); out[k] = run >= n
    return out
print(f"bag | 설명 | λ (|λ|>{TH_L}) 판정 % [FL FR RL RR] · 출발 전후 첫 판정 | σ (>0.35) 판정 % · 첫 판정 | 주행 중 |λ| 중앙값 [FL FR RL RR]")
for b, nm in [x.split(":") for x in sys.argv[1:]]:
    t, sig, E, T = run(b)
    fs = rule(t, sig)
    raw = read("bags/2026-11-21/"+b, ("/slip_logger/slip","/mpc/armed","/dra_mppi/reference_trajectory"))
    s0 = next(tt for tt,m in raw["/mpc/armed"] if m.data)
    R = np.array([[tt,np.hypot(x.vx,x.vy)] for tt,m in raw["/dra_mppi/reference_trajectory"] if tt>=s0 and len(m.references) for x in [m.references[0]]])
    mv = np.where(R[:,1]>0.01)[0]; tg, te = R[mv[0],0], R[mv[-1],0]
    L = np.array([m.data[:5] for tt,m in raw["/slip_logger/slip"]]); L = L[(L[:,0]>=tg-0.3)&(L[:,0]<=te+0.5)]
    lam = np.nan_to_num(L[:,1:5]); fl = persist(np.abs(lam) > TH_L)
    tl = L[:,0]-tg
    def first(f, tt):
        c = [(tt[np.argmax(f[:,i])], i) for i in range(4) if f[:,i].any()]
        return f"{'FL FR RL RR'.split()[min(c)[1]]} {min(c)[0]:+.2f} s" if c else "없음"
    print(f"{b} | {nm} | {(100*fl.mean(0)).round(0)} · {first(fl, tl)} | {(100*fs.mean(0)).round(0)} · {first(fs, t)} | {np.median(np.abs(lam),0).round(2)}")
