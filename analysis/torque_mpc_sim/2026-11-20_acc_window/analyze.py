#!/usr/bin/python3
"""창 밀기 켬(shift) vs 끔(old): 직선 가속 앞먹임 비율 ρ, 진행 방향 평균 오차 (뒤처짐), 윤곽, 슬립. 참값 기준 (시뮬 바닥 μ 0.45)."""
import json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, "runs"); SIDE = 1.5
r, m, Jw = 0.04, 4.56, 8.9e-4; mw = 4 * Jw / r**2
def contour(px, py):
    def seg(ax, ay, bx, by):
        ux, uy = bx-ax, by-ay; t = np.clip(((px-ax)*ux+(py-ay)*uy)/(ux*ux+uy*uy), 0, 1); return np.hypot(px-ax-t*ux, py-ay-t*uy)
    P=[(0,0),(SIDE,0),(SIDE,SIDE),(0,SIDE),(0,0)]; return np.min([seg(*P[i],*P[i+1]) for i in range(4)],axis=0)
def met(lab):
    s = json.loads(open(f"{R}/{lab}.json").read().strip().splitlines()[-1]); z = np.load(f"{R}/{lab}.npz"); raw, st, lam = z["raw"], z["st"], z["lam"]
    t = raw[:, 0]; vr = np.hypot(raw[:, 7], raw[:, 8]); mv = vr > 0.02
    c, s_ = np.cos(raw[:, 6]), np.sin(raw[:, 6]); vw = np.stack([c*raw[:, 7]-s_*raw[:, 8], s_*raw[:, 7]+c*raw[:, 8]], 1)
    tt = vw / np.maximum(np.linalg.norm(vw, axis=1), 1e-9)[:, None]; et = ((raw[:, 1:3] - raw[:, 4:6]) * tt).sum(1)
    d = contour(raw[:, 1], raw[:, 2])
    # 직선 가속: 기준 속력 기울기 > 1.3 m/s², ω_r ≈ 0 (raw 의 기준을 status 시각에 보간)
    ts = st[:, 0]; v_s = np.interp(ts, t, vr); om_s = np.interp(ts, t, np.abs(raw[:, 9]))
    acc = np.gradient(np.convolve(v_s, np.ones(5) / 5, mode="same"), ts)
    sel = (acc > 1.3) & (om_s < 0.05) & (np.abs(st[:, 22:26]).max(1) > 0.01)
    rho = np.abs(st[sel, 22:26]).mean(1) / ((m + mw) * 1.5 * r / 4) if sel.any() else np.array([np.nan])
    mk = (lam[:, 0] >= t[mv][0]) & (lam[:, 0] <= t[mv][-1]); mx = np.nanmax(np.abs(lam[mk, 1:5]), axis=1)
    return dict(rho=float(np.mean(rho)), rho_sd=float(np.std(rho)), et=1e3*float(np.mean(et[mv])), track=s["moving_rms_mm"],
                contour=1e3*float(np.sqrt(np.mean(d[mv]**2))), spin=100*float(np.mean(mx > 0.5)), lmax=float(mx.max()),
                tau=s["tau_max"], dtau=s["dtau_rms"], fault=s.get("fault") or "")
out = {}
print("| 판 | 직선 가속 앞먹임 ρ (평균 ± 표준편차) | 진행 방향 평균 (− 뒤처짐) | 시간 추종 RMS | 윤곽 RMS | 헛돎 % | λ max | Δτ RMS |")
for g in ["C_old", "C_shift", "B_old", "B_shift"]:
    ms = [met(f"{g}_r{k}") for k in (1, 2) if os.path.exists(f"{R}/{g}_r{k}.json")]
    out[g] = ms
    f = lambda k, fm: " / ".join(fm.format(x[k]) for x in ms)
    print(f"| {g} | {' / '.join(f'{x[chr(114)+chr(104)+chr(111)]:.2f}±{x[chr(114)+chr(104)+chr(111)+chr(95)+chr(115)+chr(100)]:.2f}' for x in ms)} | {f('et','{:+.0f}')} | {f('track','{:.0f}')} | {f('contour','{:.0f}')} | {f('spin','{:.1f}')} | {f('lmax','{:.2f}')} | {f('dtau','{:.4f}')} |"
          + (" ★" + ms[0]['fault'][:30] if any(x['fault'] for x in ms) else ""))
json.dump(out, open(os.path.join(HERE, "summary.json"), "w"), indent=1)
