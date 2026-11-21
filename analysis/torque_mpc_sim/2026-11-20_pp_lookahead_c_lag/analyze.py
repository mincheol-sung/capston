#!/usr/bin/python3
"""정사각형 시뮬 (바닥 μ 0.45, 최고 1.5 m/s): PP 룩어헤드 가변 vs 상수 L, MPC C 기준 가속 1.0 / 1.15 / 1.5. 참값 기준."""
import json, glob, os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, "runs"); SIDE = 1.5
def contour(px, py):
    def seg(ax, ay, bx, by):
        ux, uy = bx-ax, by-ay; t = np.clip(((px-ax)*ux+(py-ay)*uy)/(ux*ux+uy*uy), 0, 1); return np.hypot(px-ax-t*ux, py-ay-t*uy)
    P=[(0,0),(SIDE,0),(SIDE,SIDE),(0,SIDE),(0,0)]; return np.min([seg(*P[i],*P[i+1]) for i in range(4)],axis=0)
def met(lab):
    s = json.loads(open(f"{R}/{lab}.json").read().strip().splitlines()[-1]); z = np.load(f"{R}/{lab}.npz"); raw, lam = z["raw"], z["lam"]
    t = raw[:,0]; vr = np.hypot(raw[:,7], raw[:,8]); mv = vr > 0.02; t0, t1 = t[mv][0], t[mv][-1]
    d = contour(raw[:,1], raw[:,2])
    # 진행 방향 오차 e_t: 기준 진행 방향 단위벡터 (몸체 → 세계: 기준 헤딩으로 회전)
    c, s_ = np.cos(raw[:,6]), np.sin(raw[:,6]); vw = np.stack([c*raw[:,7]-s_*raw[:,8], s_*raw[:,7]+c*raw[:,8]], 1)
    tt = vw / np.maximum(np.linalg.norm(vw, axis=1), 1e-9)[:, None]; et = ((raw[:,1:3] - raw[:,4:6]) * tt).sum(1)
    h = 0.05; v = np.hypot(np.gradient(raw[:,1], t), np.gradient(raw[:,2], t))
    # 꼭짓점 머뭇: 주행 중, 기준 속도 > 0.3 인데 실제 속도 < 0.25 인 시간 (기준은 꼭짓점 근처만 0.1)
    stall = float(np.sum(np.diff(t)[mv[:-1] & (v[:-1] < 0.25) & (vr[:-1] > 0.3)]))
    mk = (lam[:,0] >= t0) & (lam[:,0] <= t1); mx = np.nanmax(np.abs(lam[mk,1:5]), axis=1)
    return dict(contour=1e3*np.sqrt(np.mean(d[mv]**2)), cmax=1e3*d[mv].max(), et=1e3*np.mean(et[mv]), track=s["moving_rms_mm"],
                vmax=float(v[mv].max()), stall=stall, spin=100*np.mean(mx>0.5), lmax=float(mx.max()), tau=s["tau_max"], dtau=s["dtau_rms"],
                fault=s.get("fault") or "")
groups = [("pp_var_a1.5", ["pp_var_a1.5_r1", "pp_var_a1.5_r2"]), ("ppL0.2_a1.5", ["ppL0.2_a1.5"]), ("ppL0.3_a1.5", ["ppL0.3_a1.5"]), ("ppL0.4_a1.5", ["ppL0.4_a1.5"]),
          ("pp_var_a1.0", ["pp_var_a1.0_r1"]), ("ppL0.2_a1.0", ["ppL0.2_a1.0"]), ("ppL0.3_a1.0", ["ppL0.3_a1.0"]), ("ppL0.4_a1.0", ["ppL0.4_a1.0"]),
          ("mpcC_a1.5", ["mpcC_a1.5_r1", "mpcC_a1.5_r2"]), ("mpcC_a1.15", ["mpcC_a1.15_r1", "mpcC_a1.15_r2"]), ("mpcC_a1.0", ["mpcC_a1.0_r1", "mpcC_a1.0_r2"])]
out = {}
print("| 판 | 윤곽 RMS / 최대 | 진행 방향 평균 (− 뒤처짐) | 시간 추종 RMS | 최고 속도 | 꼭짓점 머뭇 [s] | 헛돎 % | λ max | τ max | Δτ RMS |")
for g, labs in groups:
    ms = [met(l) for l in labs if os.path.exists(f"{R}/{l}.json")]
    out[g] = ms
    f = lambda k, fm: " / ".join(fm.format(m[k]) for m in ms)
    print(f"| {g} | {f('contour','{:.0f}')} / {f('cmax','{:.0f}')} | {f('et','{:+.0f}')} | {f('track','{:.0f}')} | {f('vmax','{:.2f}')} | {f('stall','{:.2f}')} | {f('spin','{:.1f}')} | {f('lmax','{:.2f}')} | {f('tau','{:.2f}')} | {f('dtau','{:.4f}')} |" + (" ★" + ms[0]['fault'][:30] if any(m['fault'] for m in ms) else ""))
json.dump(out, open(os.path.join(HERE, "summary.json"), "w"), indent=1)
