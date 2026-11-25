#!/usr/bin/python3
"""all_bags.json 합치기: 배율별 주행 innovation RMS (포즈 수 가중), ρ₁ (가중 평균), bag 별 부호 검정, 제어기별."""
import json, math, os
import numpy as np
D = json.load(open(os.path.join(os.path.dirname(__file__), os.environ.get("OUT", "all_bags.json"))))
MUL = tuple(f"{float(x):.1f}" for x in os.environ.get("MUL", "0.5,1,2,3,4").split(","))
use = {k: v for k, v in D.items() if all(m in v["res"] for m in MUL)}
print(f"bag {len(D)} 개 중 주행 포즈 30 개 이상 {len(use)} 개 (A {sum(v['ctrl']=='A' for v in use.values())}, "
      f"C {sum(v['ctrl']=='C' for v in use.values())}, B {sum(v['ctrl']=='B' for v in use.values())}), "
      f"est_mode 가 kf 아닌 bag {sum(v['est_mode']!='kf' for v in use.values())} 개")
chk = [v.get("check_mm", float("nan")) for v in use.values()]
print(f"검산 (재생 vs 기록 잔차) 최대 차: 중앙값 {np.nanmedian(chk):.3f} mm, 최대 {np.nanmax(chk):.3f} mm, 0.1 mm 넘는 bag {sum(c > 0.1 for c in chk)} 개")
def pooled(sel):
    rows = []
    for m in MUL:
        n = np.array([v["res"][m]["n"] for v in sel]); w = n / n.sum()
        rows.append((m, int(n.sum()), math.sqrt(np.sum(w * [v["res"][m]["sxy"] for v in sel])) * 1e3,
                     float(np.sum(w * [v["res"][m]["rxy"] for v in sel])),
                     math.degrees(math.sqrt(np.sum(w * [v["res"][m]["syaw"] for v in sel]))),
                     float(np.sum(w * [v["res"][m]["ryaw"] for v in sel]))))
    return rows
for lab, sel in (("전체", list(use.values())), ("A", [v for v in use.values() if v["ctrl"] == "A"]),
                 ("C", [v for v in use.values() if v["ctrl"] == "C"])):
    if not sel:
        continue
    print(f"\n### {lab} ({len(sel)} bag)")
    print("| 배율 (σ_a, σ_g) | 주행 포즈 | ν_xy RMS mm | ρ₁ xy | ν_ψ RMS ° | ρ₁ yaw |")
    print("|---|---|---|---|---|---|")
    for m, n, sx, rx, sy, ry in pooled(sel):
        print(f"| ×{float(m):g} | {n} | {sx:.2f} | {rx:+.2f} | {sy:.3f} | {ry:+.2f} |")
sel = list(use.values())
def frac(f):
    return f"{sum(f(v) for v in sel)}/{len(sel)}"
print("\n부호 검정 (bag 별)")
print(f"  ×1 에서 ρ₁ xy > 0: {frac(lambda v: v['res']['1.0']['rxy'] > 0)}, ρ₁ yaw > 0: {frac(lambda v: v['res']['1.0']['ryaw'] > 0)}")
print(f"  ×2 가 ×1 보다 ν_xy RMS 작음: {frac(lambda v: v['res'][MUL[2]]['sxy'] < v['res']['1.0']['sxy'])}, ν_ψ: {frac(lambda v: v['res'][MUL[2]]['syaw'] < v['res']['1.0']['syaw'])}")
print(f"  ×4 가 ×2 보다 ν_xy RMS 작음: {frac(lambda v: v['res'][MUL[-1]]['sxy'] < v['res'][MUL[2]]['sxy'])}, ν_ψ: {frac(lambda v: v['res'][MUL[-1]]['syaw'] < v['res'][MUL[2]]['syaw'])}")
best = [MUL[int(np.argmin([v['res'][m]['sxy'] for m in MUL]))] for v in sel]
besty = [MUL[int(np.argmin([v['res'][m]['syaw'] for m in MUL]))] for v in sel]
print(f"  RMS 가 가장 작은 배율 (bag 수) xy: " + ", ".join(f"×{float(m):g} {best.count(m)}" for m in MUL) +
      " | yaw: " + ", ".join(f"×{float(m):g} {besty.count(m)}" for m in MUL))
