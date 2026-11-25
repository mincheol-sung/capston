#!/usr/bin/python3
"""실차 계단 변조 8자 (circles8 width 1.6, speed 0.69 → 0.345/1.035, accel 3.0, laps 2) — C 151038 vs PP 151249.
지표·슬립 정의는 ../2026-11-23_max_compare/compare.py 와 같다 (SLAM 로봇 중심, λ = SLAM 속도 + 자이로 기준)."""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
import compare as cm
TB = cm.TB
out, store = {}, {}
for ctl, bag in (("C", "151038"), ("PP", "151249")):
    o = cm.series(bag); r = cm.metrics(o, "fig8"); r["bag"] = bag
    mv = o["moving"]; a = np.abs(o["lam"][mv]); w = a.argmax(1); big = a.max(1) > 0.5
    tr = np.abs(o["tau"][mv][np.arange(len(w)), w]) / TB[w]
    r["big_loaded"] = 100 * float((big & (tr >= 0.5)).mean()); r["wheel_share"] = [int(((w == i) & big).sum()) for i in range(4)]
    # 가감속 계단 구간 (기준 접선 가속 |a| > 1) vs 나머지
    ref = o["ref"]; sp = np.hypot(ref[:, 3], ref[:, 4]); acc = np.gradient(sp, o["t"])
    st = (np.abs(acc) > 1.0)[mv]
    r["big_in_step"] = 100 * float(big[st].mean()) if st.any() else float("nan"); r["big_outside"] = 100 * float(big[~st].mean())
    r["e_step"] = 1e3 * float(np.sqrt(np.mean(o["e"][mv][st] ** 2))); r["e_out"] = 1e3 * float(np.sqrt(np.mean(o["e"][mv][~st] ** 2)))
    out[ctl] = r; store.update({f"{ctl}_{k}": v for k, v in o.items() if isinstance(v, np.ndarray)})
json.dump(out, open(os.path.join(HERE, "step_compare.json"), "w"), indent=1, ensure_ascii=False)
np.savez(os.path.join(HERE, "step_compare.npz"), **store)
for k, r in out.items():
    print(k, json.dumps({a: (round(b, 3) if isinstance(b, float) else b) for a, b in r.items()}, ensure_ascii=False))
