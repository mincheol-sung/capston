"""11-21 정사각형 compare_all.py 와 같은 잣대로 114214 (+ 검산용 201411) 지표.
compare_all.py 의 정의 부분(run·extra·COND)만 실행하고 판 목록만 바꾼다.
"""
import os, numpy as np
CA = "/home/smoc/capston/capston/analysis/torque_mpc_real/2026-11-21_square/compare_all.py"
src = open(CA).read()
g = {"__file__": CA, "__name__": "ca_head"}
exec(src[:src.index("rows = []; TRAJ = {}")], g)
run, extra = g["run"], g["extra"]
TB_NEW = np.array([0.1445, 0.1184, 0.1444, 0.1859])
print("| 판 | 측위 | SLAM 튐 (횟수/최대 mm) | 추정기−SLAM RMS | contour RMS / 최대 | lag 평균 | 시간 추종 | yaw RMS | 큰 슬립 % (v_ref>0.3) | 헛돎 몫 | λ RMS | 슬립 크기 x·y | Δτ RMS | τ max | 변 최고 속도 | 포화 % |")
for bag, tb in [("2026-11-21/201411", g["TB"]), ("2026-11-23/114214", TB_NEW)]:
    r = run(bag, tb); c2 = r["cs"]; L = r["lag"][np.isfinite(r["lag"])]
    x = dict(contour=1e3 * float(np.sqrt(np.mean(r["contour"][r["mv"]] ** 2))), contour_max=1e3 * float(r["contour"][r["mv"]].max()),
             lag=1e3 * float(L.mean()), yaw=float(c2["yaw"]), dtau=float(c2["dtau"]), tau_max=float(c2["tau_max"]),
             slip_xy=[float(c2["slip_x"]), float(c2["slip_y"])], big=r["lam"]["big_fast"], lrms=r["lam"]["rms_fast"],
             spin=r["lam"]["spin_fast"], peak=[p for p, _ in r["side_peak"]], track=1e3 * float(c2["rms"]))
    x.update(extra(bag, tb, None))
    print(f"| {bag[-6:]} | {x['loc']} | {x['jumps']} / {x['jump_max']:.0f} | {x['est_slam']:.0f} | {x['contour']:.0f} / {x['contour_max']:.0f} | "
          f"{x['lag']:+.0f} | {x['track']:.0f} | {x['yaw']:.1f} | {x['big']:.1f} | {x['spin']:.0f} | {x['lrms']:.3f} | {x['slip_xy'][0]:.3f}·{x['slip_xy'][1]:.3f} | "
          f"{x['dtau']:.4f} | {x['tau_max']:.2f} | {' '.join(f'{p:.2f}' for p in x['peak'])} | {x.get('sat', float('nan')):.1f} |")
    if "c" in r:
        print("   ρ (직선 가속 feedforward 비율)", round(r["c"]["rho"], 3), f"(n {r['c']['rho_n']}),", "구간별 포화 %",
              {k: round(v["sat"], 1) for k, v in r["c"]["phase"].items()})
