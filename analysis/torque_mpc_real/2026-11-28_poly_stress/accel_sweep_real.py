"""꺾은선에서 가속 조건별 세 제어기 슬립 격차 (11-25·26 실차 판) — poly_run.py analyze 그대로.
source install/setup.bash 뒤 /usr/bin/python3 accel_sweep_real.py → accel_sweep_real_out.txt"""
import os, sys, warnings
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.argv = sys.argv[:1]
exec(compile(open(os.path.join(HERE, "poly_run.py")).read().split("# ── 글꼴·색")[0], "poly_run_head", "exec"))
warnings.filterwarnings("ignore")
B = "/home/smoc/capston/capston/bags/"
G = {
  "가속 1.15 · 꼭짓점 정지": {"MPC": ["2026-11-25/143238_sq2_mpc_r1", "2026-11-25/144114_sq2_mpc_wheels_r1", "2026-11-25/144240_sq2_mpc_wheels_r2"],
                          "PP": ["2026-11-25/143351_sq2_pp_r1"], "ADPP": []},
  "가속 1.5 · 최고 1.5 · 통과": {"MPC": ["2026-11-25/154715_sq2p_mpcw_r1", "2026-11-25/155359_sq2p_mpcw_r2", "2026-11-25/155804_sq2p_mpcw_r3",
                                   "2026-11-26/201259_s2pl_mpc_r1", "2026-11-26/201604_s2pl_mpc_r2", "2026-11-26/201944_s2pl_mpc_r3"],
                           "PP": ["2026-11-25/154855_sq2p_pp_r1", "2026-11-25/155511_sq2p_pp_r2", "2026-11-25/155933_sq2p_pp_r3",
                                  "2026-11-26/201404_s2pl_pp_r1", "2026-11-26/201704_s2pl_pp_r2", "2026-11-26/202049_s2pl_pp_r3"],
                           "ADPP": ["2026-11-25/155018_sq2p_adpp_r1", "2026-11-25/155632_sq2p_adpp_r2", "2026-11-25/160100_sq2p_adpp_r3",
                                    "2026-11-26/201501_s2pl_adpp_r1", "2026-11-26/201804_s2pl_adpp_r2", "2026-11-26/202150_s2pl_adpp_r3"]},
  "가속 2.0 · 꼭짓점 정지": {"MPC": ["2026-11-25/151542_sq2_a20_mpcw_r1", "2026-11-25/152823_sq2_a20_mpcw_r2", "2026-11-25/152926_sq2_a20_mpcw_r3"],
                          "PP": ["2026-11-25/151644_sq2_a20_pp_r1", "2026-11-25/153051_sq2_a20_pp_r2", "2026-11-25/153213_sq2_a20_pp_r3"],
                          "ADPP": ["2026-11-25/151817_sq2_a20_adpp_r1", "2026-11-25/153342_sq2_a20_adpp_r2", "2026-11-25/153500_sq2_a20_adpp_r3"]},
}
K = [("m50", "큰 슬립 %"), ("m25", "중간 슬립 %"), ("m50_line", "직선 안 큰 슬립 %"), ("m50_corner", "꼭짓점 안 큰 슬립 %"), ("slam_pos", "위치 SLAM cm"),
     ("vmax", "최고 속력 m/s"), ("dtick", "떨림 mN·m")]
out = open(os.path.join(HERE, "accel_sweep_real_out.txt"), "w")
for g, cs in G.items():
    print(f"\n## {g}")
    print("| 제어기 | 판 | " + " | ".join(l for _, l in K) + " |"); print("|---|---|" + "---|" * len(K))
    for c, names in cs.items():
        rows = []
        for n in names:
            try:
                r = analyze(B + n); rows.append(r)
                print(f"[{g}·{c}] {n}: " + " · ".join(f"{l} {r[k]:.2f}" for k, l in K) + f" · 끝까지 {r['done']} · fault {r['faults']}", file=out)
            except Exception as e:
                print(f"[{g}·{c}] {n}: 실패 {e}", file=out)
        if not rows:
            print(f"| {c} | 0 | " + " | ".join("-" for _ in K) + " |"); continue
        cell = lambda k: (lambda v: f"{np.nanmean(v):.1f} ({np.nanmin(v):.1f}~{np.nanmax(v):.1f})")(np.array([r[k] for r in rows], float))
        print(f"| {c} | {len(rows)} | " + " | ".join(cell(k) for k, _ in K) + " |")
out.close()
