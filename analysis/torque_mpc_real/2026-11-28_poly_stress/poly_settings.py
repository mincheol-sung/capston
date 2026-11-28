"""11-28 꺾은선 MPC 설정 비교 (가속 1.5 · 2.0, W, 슬립 되먹임) + 11-26 기준 판 (W ×1 · 되먹임 없음) + PP·ADPP.
source install/setup.bash 뒤 /usr/bin/python3 poly_settings.py → poly_settings_out.txt, 표 출력.
지표는 poly_run.py analyze 그대로. CPU 유휴 = run_info/mpstat.txt 'all' 행 %idle 의 중앙값 (첫 2 줄 = 기동 순간 빼고)."""
import os, re, sys, warnings
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.argv = sys.argv[:1]
exec(compile(open(os.path.join(HERE, "poly_run.py")).read().split("# ── 글꼴·색")[0], "poly_run_head", "exec"))
warnings.filterwarnings("ignore")
B = "/home/smoc/capston/capston/bags/"
G = [
    ("1.5 · W 80,80 · 되먹임 1.0 (11-28)", ["2026-11-28/155209_stress15", "2026-11-28/160322", "2026-11-28/161335", "2026-11-28/162108"]),
    ("1.5 · W 20,20 · 되먹임 1.0 (11-28)", ["2026-11-28/162719"]),
    ("1.5 · W 20,40 · 되먹임 1.0 (11-28)", ["2026-11-28/164655"]),
    ("1.5 · W 20,20 · 되먹임 없음 (11-26)", ["2026-11-26/201259_s2pl_mpc_r1", "2026-11-26/201604_s2pl_mpc_r2", "2026-11-26/201944_s2pl_mpc_r3"]),
    ("1.5 · PP (11-26 + 11-28)", ["2026-11-26/201404_s2pl_pp_r1", "2026-11-26/201704_s2pl_pp_r2", "2026-11-26/202049_s2pl_pp_r3", "2026-11-28/162502_s2pl_pp_r1"]),
    ("1.5 · ADPP (11-26)", ["2026-11-26/201501_s2pl_adpp_r1", "2026-11-26/201804_s2pl_adpp_r2", "2026-11-26/202150_s2pl_adpp_r3"]),
    ("2.0 · W 80,80 · 되먹임 1.0 (11-28)", ["2026-11-28/163243"]),
    ("2.0 · W 20,20 · 되먹임 1.0 (11-28)", ["2026-11-28/163413"]),
    ("2.0 · W 20,40 · 되먹임 1.0 (11-28)", ["2026-11-28/163602", "2026-11-28/164535"]),
]
K = [("slam_pos", "위치 SLAM cm"), ("pos_max", "위치 최대 cm"), ("along_r", "진행 RMS"), ("cross_r", "옆 RMS"), ("head", "헤딩 °"),
     ("m50", "큰 슬립 %"), ("m25", "중간 슬립 %"), ("vcorner", "꼭짓점 m/s"), ("dtick", "떨림 mN·m")]


def cpu_idle(path):
    f = os.path.join(path, "run_info", "mpstat.txt")
    if not os.path.exists(f):
        return np.nan
    v = [float(l.split()[-1]) for l in open(f) if re.search(r"\sall\s", l) and not l.split()[-1].startswith("%")]
    return float(np.median(v[2:])) if len(v) > 3 else np.nan


out = open(os.path.join(HERE, "poly_settings_out.txt"), "w")
print("| 설정 | 판 | " + " | ".join(l for _, l in K) + " | CPU 유휴 % |"); print("|---|---|" + "---|" * (len(K) + 1))
for g, names in G:
    rows = []
    for n in names:
        try:
            r = analyze(B + n)
        except Exception as e:
            print(f"[{g}] {n}: 실패 {type(e).__name__} {e}", file=out); continue
        r["cpu"] = cpu_idle(B + n)
        if not r["done"] or r["faults"]:
            print(f"[{g}] {n}: 끝까지 {r['done']} · fault {r['faults']} — 표에서 뺌", file=out)
        else:
            rows.append(r)
        print(f"[{g}] {n}: " + " · ".join(f"{l} {r[k]:.2f}" for k, l in K) + f" · CPU 유휴 {r['cpu']:.0f} · 무장 {r['armed']:.1f} s", file=out)
    if not rows:
        print(f"| {g} | 0 | " + " | ".join("-" for _ in K) + " | - |"); continue
    cell = lambda k, f="{:.1f}": (lambda v: (f.format(np.nanmean(v)) + (f" ({f.format(np.nanmin(v))}~{f.format(np.nanmax(v))})" if len(v) > 1 else "")))(np.array([r[k] for r in rows], float))
    print(f"| {g} | {len(rows)} | " + " | ".join(cell(k, "{:.2f}" if k == "vcorner" else "{:.1f}") for k, _ in K) + f" | {cell('cpu', '{:.0f}')} |")
out.close()
