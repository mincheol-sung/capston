#!/usr/bin/python3
"""작은 circles8 + 순항 속도 sin 변조 실차 비교 (절차: amr/notes_amr/speed_mod_slip_experiment.md) — 회차를 자동 분류해 표로.

각 bag 의 run_info/args.txt 로 비교군(A pure_pursuit / B motor_only / C friction)·조건(등속 / 변조)을 가르고,
2026-11-16 비교와 같은 스크립트(amr/발표/랩미팅/분석_스크립트/)를 돌려 출력에서 숫자를 뽑는다. 계산식은 그 스크립트 그대로.

  cd ~/capston/capston && source install/setup.bash
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-20_speedmod/tools/speedmod_table.py bags/2026-11-20/1835* bags/2026-11-20/18[4-9]*
  (bag 을 여러 개 주면 회차별 표 + 조건·비교군별 최소–최대 표)

추가 열 (이 도구에서만):
  완주       track_compare 의 경로 진행 ≥ 98 % 이고 /fault_monitor/fault 가 비었으면 O, 아니면 정지 이유
  추정 튐    |ω_nom| 최대 / |ω| 최대 > 1.15 이면 ★ — 몸체가 바퀴보다 빠르다는 뜻이라 물리적이지 않다 (추정기 속도 튐).
             그 회차의 λ 는 슬립 근거로 약하다 (2026-11-16 B 회차·2026-11-20 S2 C 183042 와 같은 현상)
λ 는 slip_bins.py 의 |v̂| 0.30–0.60 m/s 칸 (이번 속도 0.20–0.60 m/s).
"""
import os
import re
import subprocess
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WS = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
Z = os.path.join(WS, "amr", "발표", "랩미팅", "분석_스크립트")


def classify(b):
    a = open(os.path.join(b, "run_info", "args.txt")).read()
    g = lambda k: (re.search(rf"\b{k}:=(\S+)", a) or [None, ""])[1]
    if "real_all" not in a or g("shape") != "circles8":
        return None
    grp = "A" if g("controller") == "pure_pursuit" else ("C" if g("limit_mode") == "friction" else
                                                         "B" if g("limit_mode") == "motor_only" else "?")
    eps = float(g("speed_mod_amp") or 0.0)
    cond = f"변조 {g('speed')} ε{eps:g} n{g('speed_mod_cycles')}" if eps > 0 else f"등속 {g('speed')}"
    return grp, cond, f"W{g('width')}"


def run(script, b):
    p = subprocess.run(["/usr/bin/python3", os.path.join(Z, script), b], capture_output=True, text=True, cwd=Z)
    return p.stdout


def fault_of(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if "/fault_monitor/fault" not in types:
        return ""
    r.set_filter(rosbag2_py.StorageFilter(topics=["/fault_monitor/fault"]))
    msgs = []
    while r.has_next():
        _, d, _ = r.read_next()
        msgs.append(deserialize_message(d, get_message(types["/fault_monitor/fault"])).data)
    return next((m for m in msgs if m), "")


def num(pat, s, k=1, cast=float):
    m = re.search(pat, s)
    return cast(m.group(k)) if m else np.nan


def one(b):
    b = os.path.abspath(b)                       # 스크립트는 분석_스크립트 폴더에서 돈다 (plot_slip import) → 상대 경로가 깨진다
    c = classify(b)
    if c is None:
        return None
    tc, ws, ts, sb = run("track_compare.py", b), run("wheel_sat.py", b), run("torque_stats.py", b), run("slip_bins.py", b)
    prog = num(r"진행 최대 [\d.]+ m \((\d+) %\)", tc)
    lam_line = re.search(r"0\.30–0\.60\s+(\d+)\s+([\d.na ]+?)\s+([\d.na]+)\s*$", sb, re.M)
    lam = [float(x) for x in lam_line.group(2).split()] if lam_line else [np.nan] * 4
    w = [float(x) for x in re.search(r"\|ω\| 최대 ([\d. ]+) rad/s", ws).group(1).split()] if "|ω| 최대" in ws else [np.nan]
    wnom = num(r"\|ω_nom\| 최대 ([\d.]+)", ws)
    f = fault_of(b)
    return dict(
        bag=os.path.basename(b.rstrip("/")), grp=c[0], cond=c[1], W=c[2],
        done="O" if (prog >= 98 and not f) else ("정지 " + f[:40] if f else f"진행 {prog:.0f} %"),
        e_rms=num(r"\|e_xy\| RMS ([\d.]+) cm", tc), e_max=num(r"\|e_xy\| RMS [\d.]+ cm, 최대 ([\d.]+) cm", tc),
        d_rms=num(r"경로 기준 거리 RMS ([\d.]+) cm", tc), th_rms=num(r"\|e_θ\| RMS ([\d.]+)°", tc),
        lam=float(np.sqrt(np.nanmean(np.square(lam)))), lam_max=float(np.nanmax(lam)),
        lam_frac=float(lam_line.group(3)) if lam_line else np.nan,
        tau95=num(r"95 % ([\d.]+)", ts), tau_max=num(r"최대 ([\d.]+) N·m", ts),
        sat=num(r"\|ω\|>38 비율 ([\d.]+) %", ws), spike=(wnom / max(w) > 1.15) if w and not np.isnan(wnom) else False,
        wmax=max(w), wnom=wnom)


def main():
    rows = [r for r in (one(b) for b in sys.argv[1:]) if r]
    print("bag | 비교군 | 조건 | 완주 | |e_xy| RMS / 최대 [cm] | 경로 거리 RMS [cm] | 헤딩 RMS [°] | λ RMS (0.3–0.6) 4 바퀴 / 최악 | |λ|>0.1 [%] | |τ| 95 % / 최대 | |ω|max / |ω_nom|max | 추정 튐")
    for r in rows:
        print(f"{r['bag']} | {r['grp']} | {r['cond']} {r['W']} | {r['done']} | {r['e_rms']:.1f} / {r['e_max']:.1f} | {r['d_rms']:.1f} | {r['th_rms']:.1f} | "
              f"{r['lam']:.3f} / {r['lam_max']:.2f} | {r['lam_frac']:.1f} | {r['tau95']:.3f} / {r['tau_max']:.3f} | {r['wmax']:.1f} / {r['wnom']:.1f} | {'★' if r['spike'] else ''}")
    groups = {}
    for r in rows:
        groups.setdefault((r["cond"], r["grp"]), []).append(r)
    if any(len(v) > 1 for v in groups.values()):
        print("\n조건·비교군별 최소–최대 (완주한 회차만, ★ 회차는 λ 범위에서 뺌)")
        print("조건 | 비교군 | 완주/전체 | |e_xy| RMS | |e_xy| 최대 | 경로 거리 RMS | 헤딩 RMS | λ RMS (★ 뺌) | |τ| 95 %")
        rng = lambda v, f="{:.1f}": (f.format(min(v)) + "–" + f.format(max(v))) if v else "—"
        for (cond, grp), v in sorted(groups.items()):
            ok = [r for r in v if r["done"] == "O"]
            g = lambda k, src=ok: [r[k] for r in src if not np.isnan(r[k])]
            print(f"{cond} | {grp} | {len(ok)}/{len(v)} | {rng(g('e_rms'))} | {rng(g('e_max'))} | {rng(g('d_rms'))} | {rng(g('th_rms'))} | "
                  f"{rng(g('lam', [r for r in ok if not r['spike']]), '{:.3f}')} | {rng(g('tau95'), '{:.3f}')}")


if __name__ == "__main__":
    main()
