#!/usr/bin/env python3
"""Python 판(amr_mpc)으로 기준 입출력을 만들어 C++ 판(wheel_mpc_replay)과 맞춰 본다.

  python3 make_parity_fixture.py <out_dir>          (controller/mpc 를 PYTHONPATH 에 두거나 amr_mpc 설치본으로)
  wheel_mpc_replay mpc <out_dir>/mpc_*.bin
  wheel_mpc_replay ref <out_dir>/ref.bin

mpc_*.bin  폐루프(바퀴별 FOPDT + 측정잡음 + 50 Hz 측정 갱신)에서 Python WheelMPC.step 의 입력과 출력.
           포화가 걸리게 0.8 m/s 원궤적 + 초기오차 35 cm·23°, 중간에 reset 2 회.
ref.bin    무작위 기준열(±π 를 넘는 yaw, 정지 기준열 포함)에서 ReferenceHorizon.at / is_hold, HoldLatch.update.

형식: 전부 little-endian float64. 앞에 정수 필드도 float64 로 넣는다 (replay.cpp 와 같은 순서).
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np

from amr_mpc.wheel import model as wm
import amr_mpc.wheel.mpc as M
from amr_mpc.core.reference import ReferenceHorizon, HoldLatch, wrap

CASES = [   # 이름, Ts, Np, Nc, theta
    ("25hz_d1", 0.04, 20, 5, 0.040),     # 현재 운용 구성 (Python fast path d=1)
    ("25hz_d0", 0.04, 20, 5, 0.000),
    ("100hz_d0", 0.01, 80, 20, 0.000),   # Python 은 d ≤ 1 만 받으므로 100 Hz 는 d=0 으로 비교
]


def put(f, *vals):
    for v in vals:
        a = np.asarray(v, float).ravel()
        f.write(struct.pack(f"<{a.size}d", *a))


def mpc_case(path, Ts, Np, Nc, theta, T=8.0, seed=0):
    wm.THETA = float(theta)
    s = Ts / 0.04
    q, r, qn, ko = (6.0 * s, 6.0 * s, 0.5 * s), 0.0016 / s, 0.0001 * s, 0.15
    m = M.WheelMPC(Ts=Ts, Np=Np, Nc=Nc, q=q, r=(r,) * 4, q_null=qn, k_obs=ko)
    a = np.exp(-Ts / wm.TAU); b = wm.K_DC * (1 - a); dp = int(round(0.040 / Ts))   # 플랜트는 항상 θ 40 ms
    V, R0 = 0.8, 1.0
    om = V / R0
    rf = lambda t: (np.array([R0 * math.sin(om * t), R0 * (1 - math.cos(om * t)), om * t]), np.array([V, 0.0, om]))
    rng = np.random.default_rng(seed)
    X = np.array([0.0, -0.35, -0.40]); w = np.zeros(4); w_prev = np.zeros(4); hist = [np.zeros(4)] * (dp + 1)
    hold = max(1, int(round(0.02 / Ts))); w_meas = np.zeros(4)
    steps = int(T / Ts); resets = {steps // 3, 2 * steps // 3}
    recs = []
    for k in range(steps):
        t = k * Ts
        reset = k in resets
        if reset:
            m.reset(); w_prev = np.zeros(4)
        if k % hold == 0:
            w_meas = w + rng.normal(0, 0.05, 4)
        Xr, ur = rf(t)
        ref = [(rf(t + i * Ts)[0][2], *rf(t + i * Ts)[1][:2]) for i in range(Np)]
        wc = m.step(X, w_meas, w_prev, ref, Xr, ur)
        recs.append((reset, X.copy(), w_meas.copy(), w_prev.copy(), Xr, ur, np.array(ref), wc.copy(), m.stats["fallback"]))
        hist.append(wc); w_prev = wc
        w = a * w + b * hist[-1 - dp]
        X = X + Ts * wm.f(X, wm.T_FWD @ w); X[2] = math.atan2(math.sin(X[2]), math.cos(X[2]))
    with open(path, "wb") as f:
        put(f, Ts, Np, Nc, theta, q[0], q[2], r, qn, ko, wm.TAU, wm.K_DC,
            wm.R_W, float(wm.CFG["geometry"]["wheel_base_m"]), float(wm.CFG["geometry"]["robot_width_m"]),
            wm.W_MAX, wm.ALPHA_MAX, len(recs))
        for reset, X_, wm_, wp_, Xr, ur, ref, wc, fb in recs:
            put(f, float(reset), X_, wm_, wp_, Xr, ur, ref, wc, fb)
    sat = np.mean([np.abs(r[7]).max() >= wm.W_MAX - 1e-9 for r in recs]) * 100
    print(f"{path.name}: {len(recs)} 스텝, 바퀴 한계에 닿은 스텝 {sat:.1f} %, fallback {sum(r[8] for r in recs)}")


def ref_case(path, n_cases=300, seed=1):
    rng = np.random.default_rng(seed)
    with open(path, "wb") as f:
        put(f, n_cases)
        for c in range(n_cases):
            T = int(rng.integers(1, 25)); dt = float(rng.uniform(0.05, 0.3)); t0 = float(rng.uniform(0, 100))
            if c % 10 == 0:                      # 정지 기준열
                p = rng.normal(0, 2, 3); rows = [(p[0], p[1], p[2], 0.0, 0.0, 0.0)] * T
            else:
                yaw = np.cumsum(rng.normal(0, 1.2, T)) + rng.uniform(-math.pi, math.pi)
                yaw = np.array([wrap(y) for y in yaw])        # 랩된 yaw 가 ±π 를 넘나든다
                rows = [(*rng.normal(0, 2, 2), yaw[i], *rng.normal(0, 0.6, 3)) for i in range(T)]
            h = ReferenceHorizon.from_rows(t0, dt, rows)
            ts = list(rng.uniform(t0 - 0.6, t0 + (T + 3) * dt, 40)) + list(h.times) + [h.times[0] - 1e-12]
            put(f, T, dt, t0, np.array(rows), float(h.is_hold), len(ts))
            for t in ts:
                Xr, ur = h.at(t)
                put(f, t, Xr, ur)
        # HoldLatch 무작위 열
        L = HoldLatch(0.01, math.radians(2), 0.03, math.radians(5))
        n = 2000
        put(f, 0.01, math.radians(2), 0.03, math.radians(5), n)
        for _ in range(n):
            ih = bool(rng.random() < 0.8); st = bool(rng.random() < 0.7)
            Xe = np.array([rng.normal(0, 0.02), rng.normal(0, 0.02), rng.normal(0, math.radians(4))])
            put(f, float(ih), Xe, float(st), float(L.update(ih, Xe, stopped=st)))
    print(f"{path.name}: 기준열 {n_cases} 개, HoldLatch 2000 스텝")


if __name__ == "__main__":
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
    for name, Ts, Np, Nc, th in CASES:
        mpc_case(out / f"mpc_{name}.bin", Ts, Np, Nc, th)
    ref_case(out / "ref.bin")
