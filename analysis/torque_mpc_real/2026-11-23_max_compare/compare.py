#!/usr/bin/python3
"""2026-11-23 speed:=max accel:=max 실차 4 판 — circles8 width 3.0 (C 144243 · PP 144542 — 둘 다 첫 최적화 뒤 무장, 예전 120345·120544 는 compare_fig8_120345_120544.json), square (C 143041 — 첫 최적화 뒤 무장, 예전 121319 는 compare_121319.json · PP 121404).
SLAM 은 로봇 중심 (compare_slam.imu_to_center). 기준 활성 = 기준 속력 > 0.02 또는 |ω_r| > 0.02 인 구간.
  위치 오차 |p − p_r|, 진행 방향 e_t (+ 앞섬), 요 오차.  꼭짓점 (square) = 기준 |ω_r| > 0.05.
  슬립 λ_i = (ω_i − ω_nom,i)·sgn(ω_nom,i) / max(|ω_i|, |ω_nom,i|, 2.5),  ω_nom = T⁻¹·[SLAM 몸체 속도 (0.2 s 중앙 차분), 자이로 ω_z] (slip_logger 와 같은 꼴).
    큰 슬립 = max_i |λ_i| > 0.5 시간 %, 헛돎 (λ > 0) / 끌림 (λ < 0). 공칭 T 의 바퀴별 치우침 0.03~0.05 (SLAM↔바퀴 2.7°) 는 0.5 문턱에 비해 작다.
  C: 포화 = max_i |τ_i|/τ̄_i ≥ 0.98, 앞먹임 초과 = max_i |U_s,i|/τ̄_i > 1 (τ̄ 는 기동 로그 값).
출력: 표 (stdout), compare.json, compare.npz"""
import json, math, os
import numpy as np
from load import load, ref_at, TINV
HERE = os.path.dirname(os.path.abspath(__file__)); B = "/home/smoc/capston/capston/bags/2026-11-23"
TB = np.array([0.1445, 0.1184, 0.1444, 0.1859])
RUNS = [("fig8", "C", "144243"), ("fig8", "PP", "144542"), ("square", "C", "143041"), ("square", "PP", "121404")]
wrap = lambda a: np.arctan2(np.sin(a), np.cos(a))


def series(bag):
    d = load(f"{B}/{bag}"); D = d["D"]; a0, a1 = d["arm"]; sl, W, G = d["slam"], d["W"], d["G"]
    tt = np.arange(a0, min(a1, a0 + 60), 0.02)
    R = np.array([np.r_[ref_at(D, x)[0], ref_at(D, x)[1]] for x in tt])
    act = (np.hypot(R[:, 3], R[:, 4]) > 0.02) | (np.abs(R[:, 5]) > 0.02)
    i0, i1 = np.argmax(act), len(act) - 1 - np.argmax(act[::-1]); t, ref = tt[i0:i1 + 1], R[i0:i1 + 1]
    x = np.interp(t, sl[:, 0], sl[:, 1]); y = np.interp(t, sl[:, 0], sl[:, 2]); psi = np.interp(t, sl[:, 0], np.unwrap(sl[:, 3]))
    xa = np.interp(t + 0.1, sl[:, 0], sl[:, 1]) - np.interp(t - 0.1, sl[:, 0], sl[:, 1])
    ya = np.interp(t + 0.1, sl[:, 0], sl[:, 2]) - np.interp(t - 0.1, sl[:, 0], sl[:, 2])
    gz = np.interp(t, G[:, 0], G[:, 1]); c, s = np.cos(psi), np.sin(psi)
    wn = np.stack([(c * xa + s * ya) / 0.2, (-s * xa + c * ya) / 0.2, gz], 1) @ TINV.T
    wm = np.stack([np.interp(t, W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    lam = (wm - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(wm), np.abs(wn)), 2.5)
    T = np.array(D["tau"]); tau = np.stack([np.interp(t, T[:, 0], T[:, 2 + i]) for i in range(4)], 1)
    cr, sr = np.cos(ref[:, 2]), np.sin(ref[:, 2]); vw = np.stack([cr * ref[:, 3] - sr * ref[:, 4], sr * ref[:, 3] + cr * ref[:, 4]], 1)
    vn = np.linalg.norm(vw, axis=1); tg = vw / np.maximum(vn, 1e-9)[:, None]
    dxy = np.stack([x - ref[:, 0], y - ref[:, 1]], 1)
    o = dict(t0=float(t[0]), t=t - t[0], x=x, y=y, ref=ref, e=np.linalg.norm(dxy, axis=1), et=(dxy * tg).sum(1), eh=np.degrees(wrap(psi - ref[:, 2])),
             lam=lam, tau=tau, corner=np.abs(ref[:, 5]) > 0.05, moving=vn > 0.02, fault=bool(a1 < tt[i1] - 0.05))
    S = d["S"]
    if len(S):
        o["tau_mpc"] = np.stack([np.interp(t, S[:, 0], S[:, 4 + i]) for i in range(4)], 1)
        o["us"] = np.stack([np.interp(t, S[:, 0], S[:, 22 + i]) for i in range(4)], 1)
        o["hold"] = np.interp(t, S[:, 0], S[:, 20]) > 0.5
    return o


def metrics(o, shape):
    mv = o["moving"]; a = np.abs(o["lam"][mv]); mx = a.max(1); big = mx > 0.5
    sg = np.sign(o["lam"][mv][np.arange(len(mx)), a.argmax(1)])
    r = dict(T=float(o["t"][-1]), e_rms=1e3 * float(np.sqrt(np.mean(o["e"][mv] ** 2))), e_max=1e3 * float(o["e"][mv].max()),
             et=1e3 * float(o["et"][mv].mean()), h_rms=float(np.sqrt(np.mean(o["eh"] ** 2))), h_max=float(np.abs(o["eh"]).max()),
             big=100 * float(big.mean()), drive=100 * float((big & (sg > 0)).mean()), brake=100 * float((big & (sg < 0)).mean()),
             lam_rms=float(np.sqrt(np.mean(o["lam"][mv] ** 2))), wheel_max=[float(v) for v in a.max(0)],
             tau_max=float(np.abs(o["tau"]).max()), dtau=float(np.sqrt(np.mean(np.diff(o["tau"], axis=0) ** 2))), fault=o["fault"])
    if shape == "square":
        cm = o["corner"] & mv; sm = ~o["corner"] & mv
        r["e_corner"] = 1e3 * float(np.sqrt(np.mean(o["e"][cm] ** 2))); r["e_edge"] = 1e3 * float(np.sqrt(np.mean(o["e"][sm] ** 2)))
        bc = (np.abs(o["lam"][cm]).max(1) > 0.5); r["big_in_corner"] = 100 * float(bc.sum() / max(big.sum(), 1))
    if "tau_mpc" in o:
        ok = mv & ~o["hold"]; sat = (np.abs(o["tau_mpc"][ok]) / TB).max(1) >= 0.98; ff = (np.abs(o["us"][ok]) / TB).max(1) > 1.0
        r["sat"] = 100 * float(sat.mean()); r["ff_over"] = 100 * float(ff.mean())
        if shape == "square":
            cm = o["corner"][ok]; r["sat_corner"] = 100 * float(sat[cm].mean()); r["sat_edge"] = 100 * float(sat[~cm].mean())
    return r


if __name__ == "__main__":
    res, store = {}, {}
    for shape, ctl, bag in RUNS:
        o = series(bag); res[f"{shape}_{ctl}"] = dict(bag=bag, **metrics(o, shape))
        store.update({f"{shape}_{ctl}_{k}": v for k, v in o.items() if isinstance(v, np.ndarray)})
    json.dump(res, open(os.path.join(HERE, "compare.json"), "w"), indent=1, ensure_ascii=False)
    np.savez(os.path.join(HERE, "compare.npz"), **store)
    for k, r in res.items():
        print(k, json.dumps({a: (round(b, 3) if isinstance(b, float) else b) for a, b in r.items()}, ensure_ascii=False))
