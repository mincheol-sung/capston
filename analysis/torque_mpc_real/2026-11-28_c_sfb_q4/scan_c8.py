"""모든 날짜의 같은 원 (circles8 width 2.0 · 1.0 m/s · 1 바퀴) 판 전수 — MPC 파라미터를 지금 기본값 (183517 seq params) 과 diff 해 설정별로 묶는다.
판 나누기: bag_runs.runs (pitwall bag 은 판마다, 옛 bag 은 무장 구간 전체 1 판). 위치 > 40 cm · 무장 < 3 s 판은 표에만 남기고 묶음 평균에서 뺀다.
diff 에서 빼는 키: armed_topic (controller:=all 배선), 로그·토픽 이름. pitwall bag 은 판의 live 값 (mu_hat·W·slip_fb_scale) 을 덮어쓴다.
nice -n 19 /usr/bin/python3 scan_c8.py → scan_c8_out.txt"""
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import seq_s1 as S  # noqa: E402
from bag_runs import runs  # noqa: E402

IGN = {"armed_topic", "use_sim_time", "log_dir", "log_prefix", "cmd_topic", "state_topic", "ref_topic", "start_type_description_service"}
REF = S.SEQ


def flat(path):
    f = os.path.join(path, "run_info", "params", "torque_mpc_node.yaml")
    if not os.path.exists(f):
        return None
    out, key, lst = {}, None, []
    for line in open(f):
        if re.match(r"^\s*#", line) or not line.strip():
            continue
        m = re.match(r"^\s{4}(\w+):\s*(.*)$", line)
        if m:
            if key and lst:
                out[key] = ",".join(lst); lst = []
            key, val = m.group(1), m.group(2).strip()
            if val:
                out[key] = val; key = None
            continue
        m = re.match(r"^\s*-\s*(\S+)", line)
        if m and key:
            lst.append(m.group(1))
    if key and lst:
        out[key] = ",".join(lst)
    return out


def norm(v):
    try:
        return ",".join(f"{float(x):g}" for x in v.split(","))
    except ValueError:
        return v.strip("'\"")


if __name__ == "__main__":
    base = {k: norm(v) for k, v in flat(REF).items()}
    rows = []
    for day in sorted(d for d in os.listdir(S.B) if d.startswith("2026-11-")):
        for d in sorted(os.listdir(S.B + day)):
            p = os.path.join(S.B + day, d); a = os.path.join(p, "run_info", "args.txt")
            if not os.path.isdir(p) or not os.path.exists(a):
                continue
            A = open(a).read()
            g = lambda k, dflt="": (re.search(rf"\b{k}:=(\S+)", A) or [None, dflt])[1]
            if "circles8" not in A and "seq" not in d and "mu041" not in d:
                continue
            try:
                RS = runs(p)
            except Exception:  # noqa: BLE001
                continue
            if not RS:
                continue
            fl = flat(p); D = None
            for r in RS:
                ref = (r.get("ref") or {}).get("cfg") or {}
                if ref:                                              # pitwall bag: 판의 기준 값
                    if not (ref.get("shape") == "circles8" and float(ref.get("width", 0)) == 2.0 and float(ref.get("speed", 0)) == 1.0 and float(ref.get("laps", 1)) == 1.0):
                        continue
                    blend = float(ref.get("crossing_blend_m", 0)); ctrl = (r["controller"] or g("controller", "mpc")).upper()
                else:
                    if not (g("width") == "2.0" and g("speed") == "1.0" and g("laps", "1") == "1"):
                        continue
                    blend = float(g("crossing_blend_m", "0") or 0); ctrl = g("controller", "mpc").upper()
                if not r["armed"]:
                    continue
                try:
                    D = D or S.read(p); t_on, t_off = r["armed"][0]; m = S.metrics(D, t_on, t_off)
                except Exception as ex:  # noqa: BLE001
                    print(f"{day} {d}: 실패 {type(ex).__name__}"); continue
                diff = "-"
                if ctrl == "MPC":
                    if fl is None:
                        diff = "params 없음"
                    else:
                        cur = {k: norm(v) for k, v in fl.items()}
                        live = r.get("mpc") or {}
                        for k, v in live.items():
                            kk = {"slip_fb_enable": "slip_fb_enable", "slip_fb_scale": "slip_fb_scale", "mu_hat": "mu_hat", "W": "W", "r_u": "r_u", "w_dudt": "w_dudt"}.get(k)
                            if kk:
                                cur[kk] = norm(",".join(map(str, v)) if isinstance(v, list) else str(v).lower() if isinstance(v, bool) else str(v))
                        ks = sorted(k for k in set(base) | set(cur) if k not in IGN and base.get(k) != cur.get(k))
                        diff = " ".join(f"{k}={cur.get(k, '∅')}" for k in ks) or "지금 기본값"
                m.update(day=day[5:], name=d[:6] + (f"#{r['run']}" if len(RS) > 1 else ""), ctrl=ctrl, blend=blend, diff=diff); rows.append(m)
                print(f"{day[5:]} {m['name']} {ctrl} blend {blend:g}: 위치 {m['pos']:.1f} · 미끄럼 {m['sv']:.1f} · 큰 슬립 {m['m50']:.1f} | {diff[:160]}", flush=True)
    out = open(os.path.join(HERE, "scan_c8_out.txt"), "w")
    print(f"같은 원 판 {len(rows)} 개. 기준 params = {REF}", file=out)
    G = {}
    for m in rows:
        G.setdefault((m["ctrl"], m["blend"], m["diff"]), []).append(m)
    print("| 제어기 | blend | 지금 기본값과 다른 MPC 파라미터 | 판 | 위치 RMS cm | 진행 cm | 미끄럼 속도 cm/s | 큰 슬립 % | 날짜·판 |", file=out)
    print("|---|---|---|---|---|---|---|---|---|", file=out)
    for k in sorted(G, key=lambda k: (k[0], np.mean([m["pos"] for m in G[k]]))):
        v = [m for m in G[k] if m["armed"] > 3 and m["pos"] < 40] or G[k]
        f = lambda q: (lambda a: f"{a.mean():.1f}" + (f" ({a.min():.1f}~{a.max():.1f})" if len(a) > 1 else ""))(np.array([m[q] for m in v]))
        print(f"| {k[0]} | {k[1]:g} | {k[2]} | {len(v)}/{len(G[k])} | {f('pos')} | {f('along')} | {f('sv')} | {f('m50')} | "
              + ", ".join(f"{m['day']} {m['name']}" for m in G[k][:8]) + (" …" if len(G[k]) > 8 else "") + " |", file=out)
    out.close()
    print("→", os.path.join(HERE, "scan_c8_out.txt"))
