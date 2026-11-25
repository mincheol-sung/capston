#!/usr/bin/python3
"""circles8 1.1 m/s 실차: PP (200720) vs MPC C (200303) — 왜 PP 가 더 잘 따라가나.
  SLAM 중심 vs 기준 (위치·요), 바퀴 ω 측정 vs ω_nom = T⁻¹·[SLAM 몸체 속도 (0.2 s 중앙 차분), 자이로 ω_z],
  바퀴 슬립 λ = (ω − ω_nom)·sgn(ω_nom) / max(|ω|, |ω_nom|, 1)   (slip_logger 와 같은 꼴, 지면 속도는 SLAM+자이로)
  MPC: /torque_mpc/status 의 τ·U_s 를 C 상한 τ̄ (기동 로그 [0.1230 0.1253 0.1298 0.1187]) 으로 나눔.
출력: 표 (stdout), why.json, fig_why_pp_vs_mpc.png"""
import json, math, os
import numpy as np
from load import load, ref_at, TINV
HERE = os.path.dirname(os.path.abspath(__file__)); B = "/home/smoc/capston/capston/bags/2026-11-21"
TB = np.array([0.1230, 0.1253, 0.1298, 0.1187]); NM = ["FL", "FR", "RL", "RR"]
wrap = lambda a: np.arctan2(np.sin(a), np.cos(a))


def series(bag):
    d = load(f"{B}/{bag}"); D = d["D"]; a0, a1 = d["arm"]; sl, W, G = d["slam"], d["W"], d["G"]
    ts = np.arange(a0, min(a1, a0 + 30), 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts]); tr0 = ts[np.argmax(sp > 0.02)]
    t_end = min(a1, tr0 + 6.4)
    t = np.arange(tr0, t_end, 0.02)
    ref = np.array([np.r_[ref_at(D, x)[0], ref_at(D, x)[1]] for x in t])            # [x, y, ψ, vx, vy, ω] (몸체 속도)
    x = np.interp(t, sl[:, 0], sl[:, 1]); y = np.interp(t, sl[:, 0], sl[:, 2]); psi = np.interp(t, sl[:, 0], np.unwrap(sl[:, 3]))
    xa = np.interp(t + 0.1, sl[:, 0], sl[:, 1]) - np.interp(t - 0.1, sl[:, 0], sl[:, 1])
    ya = np.interp(t + 0.1, sl[:, 0], sl[:, 2]) - np.interp(t - 0.1, sl[:, 0], sl[:, 2])
    gz = np.interp(t, G[:, 0], G[:, 1]); c, s = np.cos(psi), np.sin(psi)
    vb = np.stack([(c * xa + s * ya) / 0.2, (-s * xa + c * ya) / 0.2, gz], 1); wn = vb @ TINV.T
    wm = np.stack([np.interp(t, W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    lam = (wm - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(wm), np.abs(wn)), 1.0)
    T = np.array(D["tau"]); tau = np.stack([np.interp(t, T[:, 0], T[:, 2 + i]) for i in range(4)], 1)
    out = dict(t=t - tr0, x=x, y=y, ref=ref, e=np.hypot(x - ref[:, 0], y - ref[:, 1]), eh=np.degrees(wrap(psi - ref[:, 2])),
               gz=gz, wm=wm, wn=wn, lam=lam, tau=tau, fault=a1 < t_end + 0.05)
    S = d["S"]
    if len(S):
        out["tau_mpc"] = np.stack([np.interp(t, S[:, 0], S[:, 4 + i]) for i in range(4)], 1)
        out["us"] = np.stack([np.interp(t, S[:, 0], S[:, 22 + i]) for i in range(4)], 1)
    # 전체 기준 경로 (그림용)
    tt = np.arange(tr0, tr0 + 6.4, 0.02); out["ref_full"] = np.array([ref_at(D, q)[0][:2] for q in tt])
    return out


if __name__ == "__main__":
    P, M = series("200720"), series("200303")
    res = {}
    print("| 판 | 달린 시간 | 위치 오차 RMS / 최대 [mm] | 요 오차 RMS / 최대 [°] | 큰 슬립 % (max|λ|>0.5) | 헛돎 λ>0.5 / 끌림 λ<−0.5 % | 바퀴별 λ 평균 FL/FR/RL/RR | 바퀴별 |τ| 최대 [N·m] |")
    print("|---|---|---|---|---|---|---|---|")
    for lab, z in (("A PP (200720)", P), ("C MPC (200303)", M)):
        mx = np.abs(z["lam"]).max(1); big = mx > 0.5
        sg = np.sign(z["lam"][np.arange(len(mx)), np.abs(z["lam"]).argmax(1)])
        r = dict(T=float(z["t"][-1]), e_rms=1e3 * float(np.sqrt(np.mean(z["e"] ** 2))), e_max=1e3 * float(z["e"].max()),
                 h_rms=float(np.sqrt(np.mean(z["eh"] ** 2))), h_max=float(np.abs(z["eh"]).max()), big=100 * float(big.mean()),
                 drive=100 * float((big & (sg > 0)).mean()), brake=100 * float((big & (sg < 0)).mean()),
                 lam_mean=[float(v) for v in z["lam"].mean(0)], tau_max=[float(v) for v in np.abs(z["tau"]).max(0)])
        res[lab] = r
        print(f"| {lab} | {r['T']:.1f} s{' (fault 정지)' if z['fault'] else ''} | {r['e_rms']:.0f} / {r['e_max']:.0f} | {r['h_rms']:.1f} / {r['h_max']:.0f} | "
              f"{r['big']:.0f} | {r['drive']:.0f} / {r['brake']:.0f} | {' / '.join(f'{v:+.2f}' for v in r['lam_mean'])} | {' / '.join(f'{v:.3f}' for v in r['tau_max'])} |")
    sat = (np.abs(M["tau_mpc"]) / TB).max(1) >= 0.98; ff = (np.abs(M["us"]) / TB).max(1) > 1.0
    fl_rev = M["wm"][:, 0] < 0
    print(f"\nMPC: 포화 (max|τ_i|/τ̄_i ≥ 0.98) {100*sat.mean():.0f} %, 앞먹임 U_s 가 상한 밖 {100*ff.mean():.0f} % (등속 원 구간 U_s/τ̄ {np.median((np.abs(M['us'])/TB).max(1)[M['t']>1.0]):.2f})")
    print(f"MPC: FL 역회전 (ω_FL < 0) {100*fl_rev.mean():.0f} % — 시작 t {M['t'][np.argmax(fl_rev)]:.2f} s, 최저 {M['wm'][:,0].min():.1f} rad/s (지면 기준 ω_nom 은 {M['wn'][fl_rev,0].mean():+.1f} 평균)")
    print(f"MPC: FL MPC 토크 평균 (역회전 중) {M['tau_mpc'][fl_rev,0].mean():+.3f}, 최종 (보상 포함) {M['tau'][fl_rev,0].mean():+.3f} N·m")
    print(f"자이로 ω 평균 (t 1~2.8 s, 기준 2.20): PP {P['gz'][(P['t']>1)&(P['t']<2.8)].mean():.2f}, MPC {M['gz'][(M['t']>1)&(M['t']<2.8)].mean():.2f} rad/s")
    print(f"PP FR 최종 토크 최대 {np.abs(P['tau'][:,1]).max():.3f} N·m = C 상한의 {np.abs(P['tau'][:,1]).max()/TB[1]:.1f} 배")
    json.dump(res, open(os.path.join(HERE, "why.json"), "w"), indent=1, ensure_ascii=False)
    np.savez(os.path.join(HERE, "why.npz"), **{f"P_{k}": v for k, v in P.items() if isinstance(v, np.ndarray)}, **{f"M_{k}": v for k, v in M.items() if isinstance(v, np.ndarray)})
