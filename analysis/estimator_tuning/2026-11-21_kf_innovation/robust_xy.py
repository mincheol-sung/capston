#!/usr/bin/python3
"""xy 만 (σ_a 배율, σ_g 그대로 — xy 와 yaw 는 서로 안 섞임을 확인함). 큰 잔차에 덜 휘둘리는 지표:
   |ν| ≤ 50 mm 만 쓴 RMS, |ν| 중앙값, 그리고 50 mm 넘는 잔차 수. 재생 검산 1 mm 넘는 bag 은 뺀다.
     nice -n 19 ionice -c3 /usr/bin/python3 analysis/estimator_tuning/2026-11-21_kf_innovation/robust_xy.py bags/2026-11-21"""
import glob, json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import replay_tune as RT
from amr_mpc.core import config
MUL = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0)
base = dict(config.section("estimator", "torque_estimator.yaml")); base["mode"] = "kf"
J = json.load(open(os.path.join(os.path.dirname(__file__), "all_bags_hi.json")))
keep = [k for k, v in J.items() if v["res"].get("1.0", {}).get("n", 0) >= 30 and v.get("check_mm", 9) <= 1.0]
out = {}
for k in sorted(keep):
    ev, _ = RT.load(os.path.join(sys.argv[1], k)); out[k] = {"ctrl": J[k]["ctrl"]}
    for m in MUL:
        cfg = dict(base); cfg["accel_noise_density"] = base["accel_noise_density"] * m
        R, _ = RT.replay(ev, cfg); e = R[(R[:, 4] > 0.05) & (R[:, 5] > 0)]
        mag = np.hypot(e[:, 1], e[:, 2]); ok = mag <= 0.05
        rho = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
        out[k][str(m)] = dict(n=int(ok.sum()), big=int((~ok).sum()), s2=float(np.mean(e[ok, 1:3] ** 2)), med=float(np.median(mag)),
                              rho=0.5 * (rho(e[ok, 1]) + rho(e[ok, 2])))
json.dump(out, open(os.path.join(os.path.dirname(__file__), "robust_xy.json"), "w"), indent=1)
ks = list(out)
print(f"{len(ks)} bag (A {sum(out[k]['ctrl']=='A' for k in ks)}, C {sum(out[k]['ctrl']=='C' for k in ks)})")
print("| σ_a 배율 | 절단 RMS 비 중앙값 [사분위] | 합친 절단 RMS mm | \\|ν\\| 중앙값 mm (합침) | ρ₁ 중앙값 | 50 mm 넘는 잔차 (전체) |")
print("|---|---|---|---|---|---|")
for m in map(str, MUL):
    r = np.array([math.sqrt(out[k][m]["s2"] / out[k]["1.0"]["s2"]) for k in ks])
    n = np.array([out[k][m]["n"] for k in ks]); w = n / n.sum()
    print(f"| ×{float(m):g} | {np.median(r):.3f} [{np.percentile(r,25):.3f}~{np.percentile(r,75):.3f}] | {math.sqrt(np.sum(w*[out[k][m]['s2'] for k in ks]))*1e3:.2f} | "
          f"{np.sum(w*[out[k][m]['med'] for k in ks])*1e3:.2f} | {np.median([out[k][m]['rho'] for k in ks]):+.2f} | {sum(out[k][m]['big'] for k in ks)} |")
best = [str(MUL[int(np.argmin([out[k][str(m)]["s2"] for m in MUL]))]) for k in ks]
print("절단 RMS 최소 배율 (bag 수): " + ", ".join(f"×{float(m):g} {best.count(str(m))}" for m in MUL if best.count(str(m))))
