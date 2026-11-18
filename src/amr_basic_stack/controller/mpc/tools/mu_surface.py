#!/usr/bin/python3
"""노면별 μ̂ 정리 도구 — 램프 시험 결과를 모아 대표값을 정하고 config/surfaces.yaml 표에 남긴다.

절차서: amr/notes_amr/mu_surface_procedure.md.  계산·규칙: amr_mpc/core/surfaces.py (ROS 없음).

  # 표 보기
  python3 tools/mu_surface.py list

  # 한 노면 정리 (mu_ramp_test.py 의 _summary.csv 여러 개, torque_ramp_node CSV 도 섞어도 된다)
  python3 tools/mu_surface.py summarize --surface lab_floor \\
      --summary analysis/torque_mpc_real/2026-11-18_mu/lab_*_summary.csv \\
      [--ramp-node analysis/torque_mpc_real/2026-11-17_ramp/ramp_*.csv] [--min-runs 5] [--main-dirs fwd,back] \\
      [--note "…"] [--write]

  # 바퀴별 마찰 상한 배율 (차체 값, torque_plant.yaml surface.wheel_limit_scale)
  python3 tools/mu_surface.py wheels --loads 1050,1180,1220,1110 \\
      [--ramp-node analysis/torque_mpc_real/2026-11-18_ramp/ramp_*.csv] [--motor-gain …] [--mu 0.4]

  --write 가 없으면 결과만 보여 준다. 쓸 때는 리포의 config/surfaces.yaml (설치본이 아니라) 에 쓰고, 다시 빌드해야 launch 가 읽는다.
"""
import argparse
import glob
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from amr_mpc.core import surfaces as S          # noqa: E402
from amr_mpc.torque.model import TorquePlant     # noqa: E402

SRC_TABLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", S.SURFACE_FILE)


def _expand(patterns):
    out = []
    for p in patterns or []:
        hits = sorted(glob.glob(os.path.expanduser(p)))
        if not hits:
            sys.exit(f"파일 없음: {p}")
        out += hits
    return out


def cmd_list(_a):
    table = S.load()
    if not table:
        print("표가 비어 있다 (config/surfaces.yaml)")
        return
    print(f"{'노면':16s} {'μ̂':>6s}  {'상태':4s}  {'측정일':10s}  방향별 (판 수·중앙값)")
    for name, e in sorted(table.items()):
        mu = "—" if e.get("mu_hat") is None else f"{e['mu_hat']:.3f}"
        dirs = "  ".join(f"{k} {v.get('n', 0)}·{v.get('median')}" for k, v in sorted((e.get("dirs") or {}).items()))
        print(f"{name:16s} {mu:>6s}  {str(e.get('status')):4s}  {str(e.get('measured')):10s}  {dirs}")
        if e.get("note"):
            print(f"{'':16s} └ {e['note']}")


def cmd_summarize(a):
    plant = TorquePlant.from_yaml()
    runs, sources, methods = [], [], []
    for f in _expand(a.summary):
        runs += S.read_mu_ramp_summary(f)
        sources.append(os.path.relpath(f))
        methods.append("mu_ramp_test")
    for f in _expand(a.ramp_node):
        runs.append(S.analyze_ramp_node_csv(f, plant))
        sources.append(os.path.relpath(f))
        methods.append("torque_ramp_node(오프라인 판정)")
    if not runs:
        sys.exit("--summary 나 --ramp-node 로 기록을 줄 것")
    main_dirs = tuple(x.strip() for x in a.main_dirs.split(",") if x.strip())
    stats, rec = S.summarize(runs, main_dirs=main_dirs, min_runs=a.min_runs)

    print(f"노면 {a.surface}: 판 {len(runs)} 개, 질량 {plant.mass:.2f} kg (torque_plant.yaml)")
    for r in runs:
        mu = "—" if not math.isfinite(r["mu"]) else f"{r['mu']:.3f}"
        tau = "" if not math.isfinite(r.get("tau_slip", float("nan"))) else f"  τ {r['tau_slip']:.3f}"
        tag = {"slip": "μ̂", "no_slip": "하한 ≥", "unconfirmed": "버림"}.get(r["result"], r["result"])
        print(f"  {r['source']:40s} {r['dir']:5s} {r['result']:11s} {tag} {mu}{tau}")
    print("방향별:")
    for k, s in sorted(stats.items()):
        print(f"  {k:5s} 판 {s['n']}  중앙 {s['median']}  범위 {s['min']}~{s['max']}"
              f"{'  하한 ≥ ' + str(s['lower_bound']) if s['lower_bound'] is not None else ''}"
              f"{'  버림 ' + str(s['unconfirmed']) if s['unconfirmed'] else ''}")
    print(f"추천 μ̂ = {rec['mu_hat']}  [{rec['status']}]  — {rec['reason']}")
    for w in rec["warnings"]:
        print(f"  ★ {w}")
    if rec["mu_hat"] is not None:
        tau_mu = plant.r / math.sqrt(2.0) * rec["mu_hat"] * plant.mass * plant.g / 4.0 * plant.c_s
        print(f"이 μ̂ 의 C 상한 τ_μ = {tau_mu:.4f} N·m (c_s {plant.c_s})")
    if a.write:
        path = os.path.abspath(SRC_TABLE)
        table = S.load()
        table[a.surface] = S.make_entry(stats, rec, sources, ", ".join(sorted(set(methods))), a.note)
        S.save(table, path)
        print(f"→ {path} 에 '{a.surface}' 를 썼다. 실행에 쓰려면 다시 빌드(amrb) 뒤  surface:={a.surface}")


def _four(text, name):
    if not text:
        return None
    v = [float(x) for x in text.strip("[] ").split(",")]
    if len(v) != 4:
        sys.exit(f"{name} 는 4 개 FL,FR,RL,RR — {text!r}")
    return v


def cmd_wheels(a):
    plant = TorquePlant.from_yaml()
    loads, gain = _four(a.loads, "--loads"), _four(a.motor_gain, "--motor-gain")
    onsets = []
    for f in _expand(a.ramp_node):
        o = S.wheel_slip_onsets(f, plant)
        onsets.append(o)
        cells = "  ".join(f"{w} {'—':>12s}" if o[w] is None else f"{w} {o[w]['t']:.2f}s {o[w]['tau_fb']:.3f}"
                          for w in S.WHEELS)
        print(f"  {os.path.basename(f):34s} {cells}")
    try:
        rec = S.wheel_scale(loads, onsets or None, gain)
    except ValueError as e:
        sys.exit(str(e))
    if onsets:
        print("바퀴별 슬립 시작 (모터 보고 토크, 판 수·중앙):  " +
              "  ".join(f"{w} {n}판·{'—' if m is None else f'{m:.3f}'}" for w, (n, m) in rec["onset"].items()))
    if loads:
        tot = sum(loads)
        print("하중:  " + "  ".join(f"{w} {x:g} ({x / tot:.1%})" for w, x in zip(S.WHEELS, loads)) + f"   합 {tot:g}")
    mu = a.mu if a.mu is not None else plant.mu_hat
    tmu = plant.r / math.sqrt(2.0) * mu * plant.mass * plant.g / 4.0 * plant.c_s
    print(f"추천 배율 ({rec['method']}):  " + "  ".join(f"{w} {s:.3f}" for w, s in zip(S.WHEELS, rec["scale"])))
    print(f"μ̂ {mu} 이면 바퀴별 τ̄ = " +
          "  ".join(f"{w} {min(plant.tau_motor_limit(), tmu * s):.4f}" for w, s in zip(S.WHEELS, rec["scale"])) +
          f" N·m  (같게 걸면 {tmu:.4f})")
    for w in rec["warnings"]:
        print(f"  ★ {w}")
    print(f"→ 쓰려면 torque_plant.yaml surface.wheel_limit_scale: {rec['scale']}"
          f"   또는 한 번만  wheel_limit_scale:={','.join(f'{s:g}' for s in rec['scale'])}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="표 보기")
    s = sub.add_parser("summarize", help="한 노면의 기록을 모아 대표값")
    s.add_argument("--surface", required=True, help="노면 이름 (영문·숫자·_ , 예: lab_floor, lab_carpet)")
    s.add_argument("--summary", nargs="*", help="mu_ramp_test.py 의 *_summary.csv (glob 가능)")
    s.add_argument("--ramp-node", nargs="*", dest="ramp_node", help="torque_ramp_node 의 ramp_*.csv (glob 가능)")
    s.add_argument("--main-dirs", default=",".join(S.MAIN_DIRS), help="대표값을 정하는 방향 (기본 fwd,back)")
    s.add_argument("--min-runs", type=int, default=5, help="주 방향마다 필요한 슬립 확인 판 수")
    s.add_argument("--note", default="", help="표에 남길 메모 (바닥 상태·날짜·배터리 등)")
    s.add_argument("--write", action="store_true", help="config/surfaces.yaml 에 쓴다")
    w = sub.add_parser("wheels", help="바퀴별 마찰 상한 배율 (하중 + 바퀴별 램프 슬립 시작)")
    w.add_argument("--loads", default="", help="바퀴별 하중 FL,FR,RL,RR (g 이든 kg 이든 같은 단위로)")
    w.add_argument("--ramp-node", nargs="*", dest="ramp_node", help="torque_ramp_node 의 ramp_*.csv (glob 가능) — 검산용")
    w.add_argument("--motor-gain", default="", dest="motor_gain", help="실제 토크/명령 FL,FR,RL,RR (모르면 비움 = 1)")
    w.add_argument("--mu", type=float, default=None, help="τ̄ 를 보여 줄 μ̂ (비우면 torque_plant.yaml)")
    a = ap.parse_args()
    if a.cmd == "summarize" and not all(c.isalnum() or c == "_" for c in a.surface):
        sys.exit("노면 이름은 영문·숫자·_ 만")
    {"list": cmd_list, "summarize": cmd_summarize, "wheels": cmd_wheels}[a.cmd](a)


if __name__ == "__main__":
    main()
