#!/usr/bin/python3
"""경로를 따라 C 마찰 상한 (바퀴별 |τ_i| ≤ τ̄_i) · 부하 예산 (|v| + k|ω| ≤ 1.55) 안에서 가장 빠른 속도 프로파일
 — 시간 최적 경로 매개화 (time-optimal path parameterization; Bobrow·Dubowsky·Gibson 1985 의 속도 한계 곡선 + 앞뒤 적분).
 tangent 헤딩 (몸체 v_y = 0) 에서 몸체 힘:  f = [m_eff·a_t,  m·v²κ (w 항 켬),  I_eff·(a_t κ + v² dκ/ds)],  τ = F⁺ f.
 그 다음: 같은 랩타임의 등속 (출발·정지 가속은 같은 accel) 이 경로 위에서 내는 max_i |τ_i|/τ̄_i — 등속 PP 가 마찰을 어디서 넘나.
 ※ 노드의 0.2 s 차분 창·교차점 ω 계단의 앞당김은 넣지 않은 해석값이다 (경향 확인용)."""
import math, sys
import numpy as np
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core import fig8_limits as fl
from amr_mpc.core.figure8 import Figure8
C = fl.plant_constants("/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/config")
TB, Fp, me, m, Ie, k = C["tau_bar"], C["Fp"], C["me"], C["m"], C["Ie"], C["k"]
VCAP = 1.55


def path(f, ds=0.005):
    """Figure8 를 느린 등속으로 뽑아 호길이 격자의 (x, y, ψ_tangent) → κ, dκ/ds."""
    t = np.arange(0, f.T_total, 0.002); X = np.array([f.at(q)[0] for q in t])
    d = np.r_[0, np.cumsum(np.hypot(np.diff(X[:, 0]), np.diff(X[:, 1])))]
    keep = np.r_[True, np.diff(d) > 1e-6]; d, X = d[keep], X[keep]
    s = np.arange(0, d[-1], ds); x, y = np.interp(s, d, X[:, 0]), np.interp(s, d, X[:, 1])
    th = np.unwrap(np.arctan2(np.gradient(y, ds), np.gradient(x, ds)))
    kap = np.gradient(th, ds); kap = np.convolve(kap, np.ones(5) / 5, mode="same"); dk = np.gradient(kap, ds)
    return s, kap, dk


def ratio(v, at, kap, dk):
    f = np.stack([me * at, m * v * v * kap, Ie * (at * kap + v * v * dk)], -1)
    return (np.abs(f @ Fp.T) / TB).max(-1)


def topp(s, kap, dk, target=1.0, a_hi=3.0):
    ds = s[1] - s[0]; n = len(s)
    # 속도 한계 곡선 (a_t = 0): 마찰·부하
    vg = np.linspace(0.02, VCAP, 300)
    vlim = np.array([vg[np.nonzero((ratio(vg, 0 * vg, kp, dkk) <= target) & (vg * (1 + k * abs(kp)) <= VCAP))[0][-1]] for kp, dkk in zip(kap, dk)])
    ag = np.linspace(0, a_hi, 121)
    def amax(v, kp, dkk, sign):
        ok = ratio(np.full_like(ag, v), sign * ag, kp, dkk) <= target
        return ag[np.nonzero(ok)[0][-1]] if ok[0] else 0.0
    v = np.zeros(n)
    for i in range(n - 1):                                     # 앞으로: 출발 0, 최대 가속
        v[i + 1] = min(vlim[i + 1], math.sqrt(v[i] ** 2 + 2 * amax(v[i], kap[i], dk[i], +1) * ds))
    v[-1] = 0.0
    for i in range(n - 1, 0, -1):                              # 뒤로: 정지 0, 최대 감속
        v[i - 1] = min(v[i - 1], math.sqrt(v[i] ** 2 + 2 * amax(v[i], kap[i], dk[i], -1) * ds))
    T = float(np.sum(2 * ds / np.maximum(v[1:] + v[:-1], 1e-6)))
    return v, vlim, T


def const_profile(s, vc, a):
    """등속 vc, 출발·정지 가속 a (사다리꼴)."""
    L = s[-1]; v = np.minimum(vc, np.minimum(np.sqrt(2 * a * s), np.sqrt(2 * a * np.maximum(L - s, 0))))
    return v


def at_of(s, v):
    return np.gradient(v * v / 2, s)


if __name__ == "__main__":
    print(f"τ̄ {np.round(TB, 4)}  (μ̂ 넘는 선 = 1/c_s = 1.25)")
    for name, f in [("circles8 width 3.0 (지름 1.5)", Figure8(width=3.0, height=1.5, speed=0.2, accel=1.0, laps=1, yaw_mode="tangent", curve="circles")),
                    ("circles8 width 1.6 (지름 0.8)", Figure8(width=1.6, height=0.8, speed=0.2, accel=1.0, laps=1, yaw_mode="tangent", curve="circles")),
                    ("gerono 8 width 3.0 · height 1.5", Figure8(width=3.0, height=1.5, speed=0.2, accel=1.0, laps=1, yaw_mode="tangent", curve="gerono"))]:
        s, kap, dk = path(f)
        v, vlim, T = topp(s, kap, dk)
        L = s[-1]
        # 같은 랩타임의 등속 (출발·정지 가속 1.3): vc 를 이분법으로
        lo, hi = 0.05, VCAP
        for _ in range(40):
            vc = 0.5 * (lo + hi); vv = const_profile(s, vc, 1.3); Tc = float(np.sum(2 * (s[1] - s[0]) / np.maximum(vv[1:] + vv[:-1], 1e-6)))
            lo, hi = (vc, hi) if Tc > T else (lo, vc)
        vv = const_profile(s, vc, 1.3); rc = ratio(vv, at_of(s, vv), kap, dk); ro = ratio(v, at_of(s, v), kap, dk)
        cruise = (v > 0.3 * v.max())
        print(f"\n== {name}: 길이 {L:.2f} m, |κ| {np.abs(kap).min():.2f}~{np.abs(kap).max():.2f} 1/m")
        print(f"   시간 최적 (C 상한·부하 안): 랩타임 {T:.2f} s, 속도 {v[cruise].min():.2f}~{v.max():.2f} m/s, 상한 비 최대 {ro.max():.2f}")
        print(f"   같은 랩타임 등속: {vc:.2f} m/s → 상한 비 최대 {rc.max():.2f} (μ̂ 넘음 {100*np.mean(rc > 1.25):.0f} % 구간), "
              f"1 넘음 {100*np.mean(rc > 1.0):.0f} %")
        np.savez(f"topp_{name.split()[0]}_{name.split()[2]}.npz", s=s, kap=kap, v=v, vlim=vlim, vv=vv, rc=rc, ro=ro, T=T, vc=vc)
