#!/usr/bin/python3
"""포화·앞먹임 초과가 언제 생기나 — 기준 상태로 구간을 나눈다: 가속 (v̇_r > 0.2), 감속 (v̇_r < −0.2), 꼭짓점 (|ω_r| > 0.3), 등속·그 밖."""
import importlib.util, json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", os.path.join(HERE, "why_c.py")); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
res = {}
for bag, which in [("171531", "old"), ("190506", "new"), ("190642", "new")]:
    S, E, P, A = w.load(f"{WS}/bags/2026-11-20/{bag}"); D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    t = S[:, 0]; tau = S[:, 4:8]; us = S[:, 22:26]; hold = S[:, 20] > 0.5; tb = w.TB[which]
    run = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    rt = np.array([a for a, _ in D["ref"]]); ur = []
    for x in t:
        i = np.searchsorted(rt, x, side="right") - 1
        ur.append(D["ref"][i][1].at(x)[1] if i >= 0 else np.full(3, np.nan))
    ur = np.array(ur); v = np.hypot(ur[:, 0], ur[:, 1]); vd = np.gradient(v, t); om = ur[:, 2]
    corner = np.abs(om) > 0.3; acc = ~corner & (vd > 0.2); dec = ~corner & (vd < -0.2); cru = ~corner & ~acc & ~dec
    sat = (np.abs(tau) / tb).max(1) >= 0.98; ff = (np.abs(us) / tb).max(1) > 1.0
    row = {}
    for name, m in [("가속", acc), ("감속", dec), ("꼭짓점 (헤딩 회전)", corner), ("그 밖", cru)]:
        mm = run & m
        row[name] = dict(share=100 * mm.sum() / run.sum(), sat=100 * sat[mm].mean() if mm.any() else float("nan"), ff=100 * ff[mm].mean() if mm.any() else float("nan"))
    res[bag] = row
    print(f"== {bag} ({which})  구간: 시간 비율 / 그 구간에서 상한 붙음 / 앞먹임이 상한 넘음")
    for k, r in row.items():
        print(f"   {k:14s} {r['share']:4.0f} %  |  포화 {r['sat']:4.0f} %  |  앞먹임 초과 {r['ff']:4.0f} %")
json.dump(res, open(os.path.join(HERE, "phase.json"), "w"), ensure_ascii=False, indent=1)
