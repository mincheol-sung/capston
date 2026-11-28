"""같은 원 (circles8 width 2.0 = 반지름 0.5 m · 1.0 m/s · 가속 1.0 · 1 바퀴) 에서 지금까지 돌린 MPC 설정별 위치–슬립 trade-off.
대상: 11-27 의 slip_fb 배율 판 전부 (0 ~ 5.0, W 20 · 80) + 11-28 170146 (W 80 · 1.0) + 11-28 seq MPC 5 판 (blend 0.6) + PP·ADPP 판.
뺀 판: 162239 (MPC 멈춤 → 발산), 193327·194141 (교차점 SLAM 튐, slip-fb-scale-sweep 메모). params 가 없는 판 (기동만 하고 끈 판) 은 건너뛴다.
조건은 판마다 run_info/params/torque_mpc_node.yaml (W·slip_fb_enable·scale·grip_speed) 와 args.txt (crossing_blend_m) 에서 읽는다.
지표: seq_s1.metrics (위치 RMS SLAM 중심·진행·옆·미끄럼 속도·큰 슬립 %).
nice -n 19 /usr/bin/python3 sfb_pareto.py → sfb_pareto_out.txt, sfb_pareto.png"""
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import seq_s1 as S  # noqa: E402
from bag_runs import runs  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

SKIP = {"162239_sfb_on_r1", "193327_yarr_mpc", "194141_yarr_mpc"}


def cond(path):
    a = open(os.path.join(path, "run_info", "args.txt")).read()
    g = lambda k, d="": (re.search(rf"\b{k}:=(\S+)", a) or [None, d])[1]
    if "circles8" not in a or g("speed") != "1.0" or g("width") != "2.0" or g("laps", "1") != "1":
        return None
    ctrl = g("controller", "mpc")
    c = dict(ctrl=ctrl.upper(), blend=float(g("crossing_blend_m", "0") or 0))
    if ctrl == "mpc":
        f = os.path.join(path, "run_info", "params", "torque_mpc_node.yaml")
        if not os.path.exists(f):
            return None
        pr = open(f).read()
        W = re.search(r"\n\s*W:\s*\n((?:\s*-\s*\S+\s*\n){6})", pr)
        if W is None:
            return None
        W = [float(x) for x in re.findall(r"-\s*(\S+)", W.group(1))]
        gv = lambda k, d: (lambda mm: mm.group(1) if mm else d)(re.search(rf"{k}:\s*(\S+)", pr))
        en = gv("slip_fb_enable", "false") == "true"; sc = float(gv("slip_fb_scale", "0")); gs = float(gv("slip_fb_grip_speed", "0"))
        lm = re.search(r"limit_mode:\s*(\S+)", pr)
        c.update(W=W[0], scale=sc if en else 0.0, grip=gs, limit=lm.group(1) if lm else "?")
    return c


if __name__ == "__main__":
    rows = []
    for day in ("2026-11-27", "2026-11-28"):
        for d in sorted(os.listdir(S.B + day)):
            p = os.path.join(S.B + day, d)
            if d in SKIP or not os.path.isdir(p) or not os.path.exists(os.path.join(p, "run_info", "args.txt")) or "seq" in d:
                continue
            c = cond(p)
            if c is None:
                continue
            try:
                rr = runs(p)
                if not rr or not rr[0]["armed"]:
                    continue
                t_on, t_off = rr[0]["armed"][0]; D = S.read(p); m = S.metrics(D, t_on, t_off)
            except Exception as ex:  # noqa: BLE001
                print(f"{day} {d}: 실패 {type(ex).__name__} {ex}"); continue
            m.update(c, name=f"{day[5:]} {d[:6]}"); rows.append(m)
    # 11-28 seq (blend 0.6)
    RS = runs(S.SEQ); D = S.read(S.SEQ)
    for r in RS:
        (t_on, t_off), = r["armed"]; m = S.metrics(D, t_on, t_off); c = r["controller"].upper()
        m.update(ctrl=c, blend=0.6, name=f"11-28 seq#{r['run']}")
        if c == "MPC":
            m.update(W=80.0, scale=1.0, grip=0.0, limit="friction")
        rows.append(m)
    # 11-26 같은 경로 (blend 0.6, MPC W 20 · 되먹임 없음)
    for c, names in S.OLD.items():
        for n in names:
            p = S.find("2026-11-26", n); (rr,) = runs(p); t_on, t_off = rr["armed"][0]; m = S.metrics(S.read(p), t_on, t_off)
            m.update(ctrl=c, blend=0.6, name=f"11-26 {n[:6]}")
            if c == "MPC":
                m.update(W=20.0, scale=0.0, grip=0.0, limit="friction")
            rows.append(m)

    out = open(os.path.join(HERE, "sfb_pareto_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    ok = [m for m in rows if m["armed"] > 3 and m["pos"] < 40]
    P(f"판 {len(rows)} 개 (위치 < 40 cm · 무장 > 3 s 남은 것 {len(ok)})")
    P("| 판 | 제어기 | blend | W | 배율 | grip | limit | 위치 RMS cm | 최대 | 진행 | 옆 RMS | 헤딩 ° | 미끄럼 속도 cm/s | 큰 슬립 % | 무장 s |")
    P("|" + "---|" * 15)
    for m in sorted(rows, key=lambda q: (q["ctrl"], q["blend"], q.get("W", 0), q.get("scale", -1), q["name"])):
        P(f"| {m['name']} | {m['ctrl']} | {m['blend']:g} | {m.get('W', '-')} | {m.get('scale', '-')} | {m.get('grip', '-')} | {m.get('limit', '-')} | {m['pos']:.1f} | {m['pos_max']:.1f} | "
          f"{m['along']:+.1f} | {m['cross_r']:.1f} | {m['head']:.1f} | {m['sv']:.1f} | {m['m50']:.1f} | {m['armed']:.1f} |")
    P("\n## 묶음 평균 (위치 < 40 cm 판)")
    keyf = lambda m: (m["ctrl"], m["blend"], m.get("W"), m.get("scale"), m.get("limit"))
    G = {}
    for m in ok:
        G.setdefault(keyf(m), []).append(m)
    P("| 제어기 | blend | W | 배율 | limit | 판 | 위치 RMS cm | 진행 cm | 옆 RMS cm | 미끄럼 속도 cm/s | 큰 슬립 % |")
    P("|" + "---|" * 11)
    for k in sorted(G, key=lambda k: (k[0], k[1], k[2] or 0, k[3] if k[3] is not None else -1)):
        v = G[k]; f = lambda key: (lambda a: f"{a.mean():.1f}" + (f" ({a.min():.1f}~{a.max():.1f})" if len(a) > 1 else ""))(np.array([m[key] for m in v]))
        P(f"| {k[0]} | {k[1]:g} | {k[2] if k[2] is not None else '-'} | {k[3] if k[3] is not None else '-'} | {k[4] or '-'} | {len(v)} | {f('pos')} | {f('along')} | {f('cross_r')} | {f('sv')} | {f('m50')} |")
    # blend 0 MPC 배율 회귀
    b0 = [m for m in ok if m["ctrl"] == "MPC" and m["blend"] == 0 and m.get("limit") == "friction"]
    if len(b0) > 3:
        x = np.array([m["scale"] for m in b0]); y1 = np.array([m["pos"] for m in b0]); y2 = np.array([m["sv"] for m in b0]); y3 = np.array([m["cross_r"] for m in b0])
        P(f"\nblend 0 · friction MPC {len(b0)} 판: 배율 0 / 0.25 / 0.5 / 1.0 / ≥ 1.5 의 위치·미끄럼 속도 평균")
        for lo, hi in ((-0.01, 0.01), (0.2, 0.3), (0.45, 0.55), (0.95, 1.05), (1.05, 1.45), (1.45, 6)):
            k = (x > lo) & (x <= hi)
            if k.any():
                P(f"  배율 ({lo:.2f}, {hi:.2f}] n {k.sum()}: 위치 {y1[k].mean():.1f} · 옆 RMS {y3[k].mean():.1f} · 미끄럼 {y2[k].mean():.1f} cm/s")
    out.close()

    fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
    if fp:
        font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
    INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(9.5, 6.4))
    cmap = plt.get_cmap("Blues")
    for m in ok:
        if m["ctrl"] == "MPC":
            sc = m.get("scale", 0); col = cmap(0.35 + 0.6 * min(sc, 2.5) / 2.5) if m.get("limit") == "friction" else "#9a9a96"
            mk = "o" if m["blend"] == 0 else "s"
            ax.scatter(m["sv"], m["pos"], s=60, marker=mk, facecolors=col if m.get("W") == 20 else "white", edgecolors=col, linewidths=1.8, zorder=3)
            ax.annotate(f"{sc:g}", (m["sv"], m["pos"]), xytext=(4, 3), textcoords="offset points", fontsize=7.5, color=INK2)
        else:
            col = "#eb6834" if m["ctrl"] == "PP" else "#1baf7a"
            ax.scatter(m["sv"], m["pos"], s=60, marker="o" if m["blend"] == 0 else "s", color=col, zorder=3)
    for lab, kw in (("MPC W 20 (채움) · 숫자 = 되먹임 배율", dict(color=cmap(0.8))), ("MPC W 80 (빈 점)", dict(facecolors="white", edgecolors=cmap(0.8))),
                    ("MPC motor_only (회색, 참고 — 추천안에서 뺌)", dict(color="#9a9a96")), ("PP", dict(color="#eb6834")), ("ADPP", dict(color="#1baf7a")), ("● blend 0 · ■ blend 0.6", dict(color="white"))):
        ax.scatter([], [], s=60, label=lab, **kw)
    ax.set_xlabel("미끄럼 속도 RMS [cm/s] (네 바퀴 중 최대 r·(ω − ω_nom))"); ax.set_ylabel("위치 RMS [cm] (SLAM 로봇 중심)")
    ax.grid(color=GRID, lw=0.5); ax.legend(fontsize=8.5, frameon=False, loc="upper right")
    ax.set_title("같은 원 (반지름 0.5 m · 1.0 m/s · 1 바퀴) — 지금까지 돌린 설정의 위치–슬립 (왼쪽 아래가 좋음)", loc="left", fontsize=10.5, color=INK)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "sfb_pareto.png"), dpi=130); plt.close(fig)
    print("그림:", os.path.join(HERE, "sfb_pareto.png"))
