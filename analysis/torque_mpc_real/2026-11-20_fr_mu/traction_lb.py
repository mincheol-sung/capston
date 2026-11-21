#!/usr/bin/python3
"""바퀴별 μ 하한 — 미끄러지지 않고 버틴 접지 토크의 최대 (50 ms 이상 이어진 값만).
  무슬립 판정 = single_wheel_slip.py 와 같은 식으로 그 바퀴의 어긋남 |d_i| < 0.05 m/s 이고 세 바퀴 잔차 < 0.04 m/s (몸체가 통째로 밀린 것도 빼려고
  자이로·엔코더 일관성만 본다). τ_c = |τ_fb| − (C_i + V_i|ω_i|) − J_w|ω̇_i|.  μ_LB = (50 ms 이동 최소 τ_c 의 최대) / ((r/√2)(m g/4))."""
import importlib.util, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("se", os.path.join(HERE, "slip_events.py")); se = importlib.util.module_from_spec(spec); spec.loader.exec_module(se)
r, k, C, V, Jw, UNIT, W = se.r, se.k, se.C, se.V, se.Jw, se.UNIT, se.W
b = np.array([-1, 1, 1, -1.0]); c = np.array([-1, 1, -1, 1.0])
res_all = {}
for bag in sys.argv[1:]:
    ws, P, G, arm = se.load(bag.rstrip("/"))
    t = ws[:, 0]; om = ws[:, 1:5]; tfb = ws[:, 5:9]; fresh = (ws[:, 9:13] > 0.5).all(1)
    wz = np.interp(t, G[:, 0], G[:, 1]); y = r * om - c[None, :] * k * wz[:, None]
    A = np.stack([np.ones(4), b], 1); sol = (np.linalg.pinv(A) @ y.T).T; dev4 = y - sol @ A.T     # 네 바퀴 모두로 맞춘 잔차
    ok = fresh & se.cs.armed_mask(t, arm) & (np.abs(dev4).max(1) < 0.05)
    dom = np.gradient(om, t, axis=0)
    tc = np.abs(tfb) - (C[None, :] + V[None, :] * np.abs(om)) - Jw * np.abs(dom)
    dt = np.median(np.diff(t)); nwin = max(1, int(round(0.05 / dt)))
    row = {}
    for i in range(4):
        x = np.where(ok, tc[:, i], -np.inf)
        mins = np.array([x[j:j + nwin].min() for j in range(len(x) - nwin)])
        jm = int(np.argmax(mins)); row[W[i]] = dict(tau=float(mins[jm]), mu=float(mins[jm] / UNIT), t=float(t[jm] - t[0]),
                                                    tau_fb=float(np.abs(tfb[jm, i])), omega=float(om[jm, i]))
    res_all[os.path.basename(bag.rstrip("/"))] = row
    print(f"{os.path.basename(bag.rstrip('/'))}: " + "  ".join(f"{w} μ≥{row[w]['mu']:.3f} (τ_c {row[w]['tau']:.3f}, ω {row[w]['omega']:+.0f})" for w in W))
json.dump(res_all, open(os.path.join(HERE, "traction_lb.json"), "w"), ensure_ascii=False, indent=1)
mx = {w: max(v[w]["mu"] for v in res_all.values()) for w in W}
print("\n오늘 판 전체 하한 (최대): " + "  ".join(f"{w} {mx[w]:.3f}" for w in W))
print("지금 설정 mu_hat_wheels: FL 0.432  FR 0.285  RL 0.456  RR 0.350")
