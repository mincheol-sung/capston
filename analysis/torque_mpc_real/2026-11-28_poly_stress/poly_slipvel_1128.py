"""11-28 꺾은선 1.5/1.5 (되먹임 켬) 과 같은 날 PP 의 미끄럼 속도 — circ_slipvel.py 의 one() 그대로."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "circ_slipvel.py")).read().split("out = open(")[0]
exec(compile(src, "circ_slipvel_head", "exec"))
for lab, day, names in (("MPC W ×4 · 되먹임 1.0", "2026-11-28", ["155209_stress15", "160322", "161335", "162108"]),
                        ("MPC W ×1 · 되먹임 1.0", "2026-11-28", ["162719"]), ("PP (11-28)", "2026-11-28", ["162502_s2pl_pp_r1"])):
    rows = [one(find(day, n)) for n in names]
    v = lambda k: np.array([r[k] for r in rows])
    print(f"| 꺾은선 11-28 1.5/1.5 | {lab} | {len(rows)} | {v('rms').mean():.1f} ({v('rms').min():.1f}~{v('rms').max():.1f}) | {v('p10').mean():.1f} | {v('p20').mean():.1f} | {v('spin').mean():.0f} |")
