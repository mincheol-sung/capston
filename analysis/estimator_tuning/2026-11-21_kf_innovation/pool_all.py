#!/usr/bin/python3
"""all_bags.json (×0.5~4) + all_bags_hi.json (×1, 6~16) 합치기. 같은 bag 의 ×1 은 결정적 재생이라 같다.
   제외: 재생이 노드 기록 잔차와 1 mm 넘게 다른 bag (SLAM 초기화·저주기 등으로 재생이 노드와 다른 경로를 탄 것), 배율 하나라도 주행 포즈 < 30."""
import json, math, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
A = json.load(open(os.path.join(H, "all_bags.json"))); B = json.load(open(os.path.join(H, "all_bags_hi.json")))
MUL = ("0.5", "1.0", "2.0", "3.0", "4.0", "6.0", "8.0", "12.0", "16.0")
use, drop = {}, []
for k in sorted(set(A) | set(B)):
    v = dict(A.get(k) or B[k]); res = dict((A.get(k) or {}).get("res", {})); res.update((B.get(k) or {}).get("res", {}))
    chk = max(x for x in ((A.get(k) or {}).get("check_mm", 0.0), (B.get(k) or {}).get("check_mm", 0.0)) if x == x) if True else 0
    if not all(m in res for m in MUL):
        continue
    if chk > 1.0:
        drop.append(f"{k} (검산 {chk:.0f} mm)"); continue
    v["res"] = res; use[k] = v
print(f"쓴 bag {len(use)} 개 (A {sum(v['ctrl']=='A' for v in use.values())}, C {sum(v['ctrl']=='C' for v in use.values())}), 뺀 bag: {', '.join(drop) or '없음'}")
def table(sel, lab):
    print(f"\n### {lab} ({len(sel)} bag)")
    print("| 배율 | 주행 포즈 | ν_xy RMS mm | ρ₁ xy | ν_ψ RMS ° | ρ₁ yaw |")
    print("|---|---|---|---|---|---|")
    for m in MUL:
        n = np.array([v["res"][m]["n"] for v in sel]); w = n / n.sum()
        sx = math.sqrt(np.sum(w * [v["res"][m]["sxy"] for v in sel])) * 1e3
        sy = math.degrees(math.sqrt(np.sum(w * [v["res"][m]["syaw"] for v in sel])))
        rx = float(np.sum(w * [v["res"][m]["rxy"] for v in sel])); ry = float(np.sum(w * [v["res"][m]["ryaw"] for v in sel]))
        print(f"| ×{float(m):g} | {n.sum()} | {sx:.2f} | {rx:+.2f} | {sy:.3f} | {ry:+.2f} |")
sel = list(use.values())
table(sel, "전체")
table([v for v in sel if v["ctrl"] == "A"], "A"); table([v for v in sel if v["ctrl"] == "C"], "C")
cnt = lambda f: f"{sum(f(v) for v in sel)}/{len(sel)}"
r = lambda v, m, k: v["res"][m][k]
print("\n부호 검정 (bag 별)")
print(f"  ×1 ρ₁ > 0  xy {cnt(lambda v: r(v,'1.0','rxy') > 0)}, yaw {cnt(lambda v: r(v,'1.0','ryaw') > 0)}")
for a, b in (("1.0", "2.0"), ("2.0", "3.0"), ("3.0", "4.0"), ("4.0", "6.0"), ("6.0", "8.0"), ("8.0", "12.0")):
    print(f"  ×{float(b):g} 이 ×{float(a):g} 보다 RMS 작음  xy {cnt(lambda v: r(v,b,'sxy') < r(v,a,'sxy'))}, yaw {cnt(lambda v: r(v,b,'syaw') < r(v,a,'syaw'))}")
bx = [MUL[int(np.argmin([r(v, m, 'sxy') for m in MUL]))] for v in sel]; by = [MUL[int(np.argmin([r(v, m, 'syaw') for m in MUL]))] for v in sel]
print("  RMS 최소 배율 (bag 수)  xy: " + ", ".join(f"×{float(m):g} {bx.count(m)}" for m in MUL if bx.count(m)) +
      " | yaw: " + ", ".join(f"×{float(m):g} {by.count(m)}" for m in MUL if by.count(m)))
