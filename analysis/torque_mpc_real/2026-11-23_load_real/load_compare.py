#!/usr/bin/python3
"""실차 계단 변조 8자 — C 하중 이동 끔 (151038) · 켬 h 0.2 (154431) · PP (151249). 지표는 ../2026-11-23_max_compare/compare.py 와 같다.
더해: 원심 가속 a_y = v·|ω| 구간별 안쪽/바깥 바퀴 |λ| > 0.5, hold (DAMPING) 구간, /torque_mpc/load 의 수직력."""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
import compare as cm
TB = cm.TB
RUNS = [("C_off", "151038"), ("C_on", "154431"), ("PP", "151249")]
out, store = {}, {}
for lab, bag in RUNS:
    o = cm.series(bag); r = cm.metrics(o, "fig8"); r["bag"] = bag
    mv = o["moving"]; ref = o["ref"]; sp = np.hypot(ref[:, 3], ref[:, 4]); om = ref[:, 5]; ay = sp * np.abs(om)
    lam = o["lam"]; left = om > 0
    inf = np.where(left, lam[:, 0], lam[:, 1]); inr = np.where(left, lam[:, 2], lam[:, 3])
    outf = np.where(left, lam[:, 1], lam[:, 0]); outr = np.where(left, lam[:, 3], lam[:, 2])
    tin_f = np.where(left, o["tau"][:, 0], o["tau"][:, 1])
    bins = {}
    for lo, hi in ((0.0, 0.6), (0.6, 2.2), (2.2, 3.5)):
        m = mv & (np.abs(om) > 0.05) & (ay >= lo) & (ay < hi)
        pc = lambda x: 100 * float(np.mean(np.abs(x[m]) > 0.5)) if m.any() else float("nan")
        bins[f"{lo}-{hi}"] = dict(share=100 * float(m.sum() / mv.sum()), in_f=pc(inf), in_r=pc(inr), out_f=pc(outf), out_r=pc(outr),
                                  tau_in_f=float(np.median(tin_f[m])) if m.any() else float("nan"))
    r["bins"] = bins
    if "hold" in o:
        r["hold_pct"] = 100 * float(np.mean(o["hold"][mv]))
    out[lab] = r; store.update({f"{lab}_{k}": v for k, v in o.items() if isinstance(v, np.ndarray)})
json.dump(out, open(os.path.join(HERE, "load_compare.json"), "w"), indent=1, ensure_ascii=False)
np.savez(os.path.join(HERE, "load_compare.npz"), **store)
print("| 판 | 위치 RMS / 최대 | 요 RMS | 큰 슬립 (헛돎/끌림) | λ RMS | |τ| 최대 | 포화 % | hold % |")
for lab, r in out.items():
    print(f"| {lab} {r['bag']} | {r['e_rms']:.0f} / {r['e_max']:.0f} | {r['h_rms']:.1f} | {r['big']:.1f} ({r['drive']:.1f}/{r['brake']:.1f}) | {r['lam_rms']:.3f} | {r['tau_max']:.3f} | {r.get('sat', float('nan')):.0f} | {r.get('hold_pct', float('nan')):.1f} |")
print("\n원심 가속 구간별 |λ| > 0.5 % — 안쪽 앞 / 안쪽 뒤 / 바깥 앞 / 바깥 뒤 (구간 몫), 안쪽 앞 토크 중앙")
for lab, r in out.items():
    print(lab, " | ".join(f"a_y {k}: {b['in_f']:.0f}/{b['in_r']:.0f}/{b['out_f']:.0f}/{b['out_r']:.0f} ({b['share']:.0f} %) τ_in_f {b['tau_in_f']:+.3f}" for k, b in r["bins"].items()))
