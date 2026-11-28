"""zigzag · polyline 의 μ̂ × 되먹임 배율 후보 비교 (2026-11-28 밤). 원은 grid2.py.
mu055.rows_of 와 같은 지표인데, amrseq 복귀 다리 (kind return, 180° 회전) 는 빼고 data 다리만 쓴다. zigzag 는 방향 (fwd · rev) 도 남긴다.
비교 묶음: 같은 경로의 앞 세션 판 (zigzag = 194358 seq 의 MPC 0.33·1.0 · PP · ADPP 5 판씩).
/usr/bin/python3 scen.py <이름> <경로 모양 zigzag|polyline> <bag> [<앞 세션 bag> ...] → <이름>_out.txt
(모양이 다른 판 — 예: 194358 끝의 원 판 두 개, 모터 전원이 끊겨 바퀴가 안 돈 판 — 은 뺀다)"""
import os
import sys

import numpy as np
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mu055 as M  # noqa: E402
import seq_s1 as S  # noqa: E402
import seq_s1_cause as C  # noqa: E402
import sfb_modes as SM  # noqa: E402
from bag_runs import runs  # noqa: E402

KEYS = [("pos", "위치 RMS cm"), ("pos_max", "위치 최대 cm"), ("sv", "slip velocity cm/s"), ("m50", "큰 슬립 %"), ("dtick", "떨림 mN·m"), ("head", "헤딩 RMS °"), ("sat", "상한 붙음 %")]


def sat_moving(F, CAP, t_on, t_off, ts, vref, mu):
    """움직이는 동안 (기준 속력 > 0.05) 한 바퀴라도 바닥 토크 ≥ 0.95 τ̄(μ̂) 인 시간 %."""
    k = (F[:, 0] >= t_on) & (F[:, 0] <= t_off) & (np.interp(F[:, 0], ts, vref) > 0.05)
    f = np.column_stack([np.interp(F[k, 0], CAP[:, 0], CAP[:, 2 + i]) for i in range(4)]) if CAP is not None else 0
    g = np.abs((F[k, 10:14] - f) @ SM.PG.T) / (SM.TBAR * mu / 0.33)
    return 100 * float(np.mean(g.max(1) >= 0.95))


SHAPE = None


def rows_seq(tag, bag):
    RS = runs(bag); D = S.read(bag); F = C.read_sfb(bag); CAP = SM.read_cap(bag); miss, err, W, Mc = M.health(bag); out = []
    for r in RS:
        sq = r.get("seq") or {}
        shp = (r.get("ref") or {}).get("cfg", {}).get("shape")
        if not r["armed"] or sq.get("kind") == "return" or shp == "rotate" or (SHAPE and shp != SHAPE):
            continue
        (t_on, t_off), = r["armed"][:1]
        c = (r["controller"] or "mpc").upper(); live = r["mpc"] or {}
        mu, sc = live.get("mu_hat", 0.33), live.get("slip_fb_scale", 1.0)
        try:
            m = S.metrics(D, t_on, t_off)
        except Exception as ex:  # noqa: BLE001
            print(f"{tag} 판 {r['run']}: 지표 실패 {type(ex).__name__} {ex}"); continue
        sat = sat_moving(F, CAP, t_on, t_off, m["_ts"] + t_on, m["_vref"], mu) if c == "MPC" else np.nan
        kw = (W[:, 0] >= t_on) & (W[:, 0] <= t_off); km = (Mc[:, 0] >= t_on) & (Mc[:, 0] <= t_off)
        m.update(tag=tag, run=r["run"], ctrl=c, mu=mu, sc=sc, dir=sq.get("dir"), sat=sat,
                 err=int(((err >= t_on) & (err <= t_off)).sum()) if err.size else 0,
                 tq=float(np.abs(W[kw, 1:5]).mean() / max(np.abs(Mc[km, 1:5]).mean(), 1e-9)))
        out.append(m)
    return out


def group(rows):
    G = {}
    for m in rows:
        k = (round(m["mu"], 2), round(m["sc"], 2)) if m["ctrl"] == "MPC" else (m["ctrl"], None)
        G.setdefault(k, []).append(m)
    return G


def table(P, rows, title):
    P(f"\n## {title}")
    G = group(rows)
    order = sorted([k for k in G if k[1] is not None]) + [k for k in (("PP", None), ("ADPP", None)) if k in G]
    P("| 조건 | 판 (fwd/rev) | " + " | ".join(lab for _, lab in KEYS) + " |")
    P("|---|---|" + "---|" * len(KEYS))
    for k in order:
        v = G[k]; name = f"MPC μ̂ {k[0]:.2f} · {k[1]:g}" if k[1] is not None else k[0]
        nf = sum(1 for m in v if m["dir"] == "fwd"); nr = sum(1 for m in v if m["dir"] == "rev")
        cells = []
        for q, _ in KEYS:
            a = np.array([m[q] for m in v], float)
            cells.append("-" if not np.isfinite(a).any() else f"{np.nanmean(a):.1f}" + (f" ({np.nanmin(a):.1f}~{np.nanmax(a):.1f})" if len(a) > 1 else ""))
        P(f"| {name} | {len(v)} ({nf}/{nr}) | " + " | ".join(cells) + " |")
    return G


if __name__ == "__main__":
    name, SHAPE, bag, prev = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
    out = open(os.path.join(HERE, f"{name}_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    now = rows_seq(name, bag)
    P(f"{name}: data 판 {len(now)} 개 ({bag}) · 모터 잰/명령 {min(m['tq'] for m in now):.2f}~{max(m['tq'] for m in now):.2f} · 모터 에러 로그 {sum(m['err'] for m in now)}"
      f" · 모터 °C {min(m['temp'] for m in now):.1f}~{max(m['temp'] for m in now):.1f}")
    for m in now:
        P(f"  판 {m['run']:2d} {m['ctrl']:4s} {m['dir'] or '':3s} μ̂ {m['mu']:.2f} · {m['sc']:g} | 위치 {m['pos']:.1f} (최대 {m['pos_max']:.1f}) · slip v {m['sv']:.1f} · 큰 슬립 {m['m50']:.1f}"
          f" · 떨림 {m['dtick']:.1f} · 헤딩 {m['head']:.1f} · 상한 {m['sat']:.0f} %")
    Gn = table(P, now, f"1) {name} 안에서만 (같은 세션)")
    allr = now[:]
    for i, b in enumerate(prev):
        allr += rows_seq(f"앞{i + 1}", b)
    Ga = table(P, allr, "2) 앞 세션 판까지 합침") if prev else Gn
    P("\n## 3) 후보 대 PP · ADPP (같은 세션만 · 합침) Welch t — 음수 = MPC 가 작음 (좋음)")
    for lab_set, G in (("같은 세션", Gn), ("합침", Ga)):
        for k in sorted(k for k in G if k[1] is not None):
            for ref in ("PP", "ADPP"):
                A, Bv = G[k], G.get((ref, None), [])
                if len(A) > 1 and len(Bv) > 1:
                    s = []
                    for q, lab in KEYS[:6]:
                        x, y = np.array([m[q] for m in A]), np.array([m[q] for m in Bv])
                        s.append(f"{lab.split()[0]} {x.mean() - y.mean():+.1f} (p {stats.ttest_ind(x, y, equal_var=False).pvalue:.2f})")
                    P(f"- [{lab_set}] μ̂ {k[0]:.2f}·{k[1]:g} (n {len(A)}) − {ref} (n {len(Bv)}): " + " · ".join(s))
    out.close()
