"""원 8 자에서 MPC 슬립이 PP·ADPP 보다 작은 조건 찾기 — 세 제어기가 같은 날 같은 조건으로 돈 판 전부.
source install/setup.bash 뒤 /usr/bin/python3 circ_slip_map.py → circ_slip_map_out.txt, 표 출력.
지표: poly_run.py analyze (10 Hz LPF 슬립률 > 0.5 / > 0.25 시간 %, SLAM 중심 위치 RMS).
옆 힘 요구 = m v² / R ÷ 옆 힘 상한 8.35 N (μ̂ 0.33 · c_s 0.8 · m g / √2, 11-27 문서), m 4.56 kg (MPC 기동 로그).
v 는 기준 최고 속력 (속도 변조 판은 최고값), R = width / 4 (circles8)."""
import os, re, sys, warnings
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.argv = sys.argv[:1]
exec(compile(open(os.path.join(HERE, "poly_run.py")).read().split("# ── 글꼴·색")[0], "poly_run_head", "exec"))
warnings.filterwarnings("ignore")
B = "/home/smoc/capston/capston/bags/"
M, FLAT = 4.56, 8.35
G = [  # (이름, 날짜, width, MPC 판, PP 판, ADPP 판)
    ("11-21 R0.30 · 0.5", "2026-11-21", 1.2, ["201047", "201751", "201947", "203133", "205044", "205817"], ["200944", "205937", "210424", "210648", "213710"], []),
    ("11-21 R0.50 · 0.5", "2026-11-21", 2.0, ["203301", "203340", "204536", "212502", "213246", "213847"], ["203305", "204329", "212629"], []),
    ("11-21 R0.50 · 1.1 a1.5", "2026-11-21", 2.0, ["200303"], ["200720"], []),
    ("11-22 R0.50 · 0.6±50 %", "2026-11-22", 2.0, ["002707", "002931", "003146", "212703"], ["002822", "003041", "003451", "212837"], []),
    ("11-23 R0.50 · 0.65±50 % a1.3", "2026-11-23", 2.0, ["115407"], ["115556"], []),
    ("11-23 R0.75 · 0.8±50 % a1.4", "2026-11-23", 3.0, ["120345", "144243"], ["120544"], []),
    ("11-23 R0.40 · 0.69±50 % a3.0", "2026-11-23", 1.6, ["151038", "154431"], ["151249", "161802_ppb", "172102_pf_pp"], []),
    ("11-26 R0.50 · 1.0 blend0.6", "2026-11-26", 2.0, ["184758_s2_mpc_r1", "184937_s2_mpc_r2", "185209_s2_mpc_r3", "185937_s2_mpc_r4", "190248_s2_mpc_r5", "190352_s2_mpc_r6"],
     ["185321_s2_pp_r1", "185437_s2_pp_r2", "185532_s2_pp_r3"], ["185636_s2_adpp_r1", "185736_s2_adpp_r2", "185834_s2_adpp_r3"]),
    ("11-27 R0.50 · 1.0 blend0", "2026-11-27", 2.0, ["192519_yarr_mpc", "193234_yarr_mpc", "193851_yarr_mpc"], ["165935_pp_r1", "173001_pp_r1"], ["173512_pp_r1"]),
    ("11-27 R0.50 · 0.7±50 %", "2026-11-27", 2.0, ["174149_circle1"], ["174419_pp_r1"], []),
]


def find(day, name):
    for d in os.listdir(B + day):
        if d.startswith(name):
            return os.path.join(B + day, d)
    return None


out = open(os.path.join(HERE, "circ_slip_map_out.txt"), "w")
print("| 묶음 | R m | 최고 v | 옆 힘 요구 | 큰 슬립 % MPC / PP / ADPP | 중간 슬립 % MPC / PP / ADPP | 위치 SLAM cm MPC / PP / ADPP | 판 |")
print("|---|---|---|---|---|---|---|---|")
for name, day, w, mp, pp, ad in G:
    R = w / 4.0; res = {}; vmax = []
    for c, names in (("MPC", mp), ("PP", pp), ("ADPP", ad)):
        rows = []
        for n in names:
            p = find(day, n)
            if p is None:
                print(f"[{name}·{c}] {n}: bag 없음", file=out); continue
            try:
                r = analyze(p)
            except Exception as e:
                print(f"[{name}·{c}] {n}: 실패 {type(e).__name__} {e}", file=out); continue
            ok = r["done"] and not r["faults"]
            print(f"[{name}·{c}] {os.path.basename(p)}: 큰 {r['m50']:.1f} · 중간 {r['m25']:.1f} · 위치 SLAM {r['slam_pos']:.1f} · 끝까지 {r['done']} · fault {r['faults']}"
                  + ("" if ok else " — 뺌"), file=out)
            if ok:
                rows.append(r); vmax.append(float(np.hypot(r["_R"][:, 3], r["_R"][:, 4]).max()))
        res[c] = rows
    v = max(vmax) if vmax else np.nan
    dem = 100 * M * v * v / R / FLAT
    f = lambda c, k: "-" if not res[c] else f"{np.mean([r[k] for r in res[c]]):.1f}"
    print(f"| {name} | {R:.2f} | {v:.2f} | {dem:.0f} % | {f('MPC','m50')} / {f('PP','m50')} / {f('ADPP','m50')} | {f('MPC','m25')} / {f('PP','m25')} / {f('ADPP','m25')} | "
          f"{f('MPC','slam_pos')} / {f('PP','slam_pos')} / {f('ADPP','slam_pos')} | {len(res['MPC'])}·{len(res['PP'])}·{len(res['ADPP'])} |")
out.close()
