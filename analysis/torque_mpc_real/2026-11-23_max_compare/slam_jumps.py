#!/usr/bin/python3
"""SLAM 포즈 점프 찾기: SLAM 표본마다 (≈0.1 s) SLAM 변위 − 바퀴(T⁺ω)+자이로 추측항법 변위 (월드, 같은 구간) 의 크기.
 정상이면 구동계 2.7° 어긋남 (속도의 4.7 %) 수준 = 1.5 m/s 에서 0.1 s 에 7 mm 정도. 40 mm 넘으면 점프로 본다. 무장·기준 이동 중만."""
import math, sys
import numpy as np
from load import load, ref_at
B = "/home/smoc/capston/capston/bags/2026-11-23"; r, k = 0.04, 0.1932
Tp = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / k, 1 / k, -1 / k, 1 / k]]) * r / 4
for bag in sys.argv[1:]:
    d = load(f"{B}/{bag}"); D = d["D"]; sl, W, G = d["slam"], d["W"], d["G"]; a0, a1 = d["arm"]
    ts = np.arange(a0, min(a1, a0 + 60), 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts])
    tr0, tr1 = ts[np.argmax(sp > 0.02)], ts[len(sp) - 1 - np.argmax(sp[::-1] > 0.02)]
    q = sl[(sl[:, 0] >= tr0) & (sl[:, 0] <= tr1)]; res = []
    for a, b in zip(q[:-1], q[1:]):
        tt = np.arange(a[0], b[0], 0.002); psi = a[3]; p = np.zeros(2)
        for t in tt:
            w = np.array([np.interp(t, W[:, 0], W[:, 1 + j]) for j in range(4)]); vb = Tp @ w
            psi += np.interp(t, G[:, 0], G[:, 1]) * 0.002
            p += 0.002 * np.array([math.cos(psi) * vb[0] - math.sin(psi) * vb[1], math.sin(psi) * vb[0] + math.cos(psi) * vb[1]])
        res.append((b[0] - tr0, np.linalg.norm(b[1:3] - a[1:3] - p), np.hypot(*ref_at(D, b[0])[1][:2])))
    res = np.array(res); big = res[res[:, 1] > 0.04]
    print(f"{bag}: SLAM 표본 {len(res)}, 잔차 중앙 {np.median(res[:, 1]) * 1e3:.0f} mm · 95 % {np.percentile(res[:, 1], 95) * 1e3:.0f} mm, "
          f"40 mm 넘는 점프 {len(big)} 번: " + ", ".join(f"t {x[0]:.2f} s {x[1] * 1e3:.0f} mm (v_r {x[2]:.2f})" for x in big))
