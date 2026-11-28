"""2026-11-28 밤 튜닝 뒤 (C μ̂ 0.60 · 슬립 되먹임 배율 1.2, 기본값 9523fbb) 세 경로 × 세 제어기 — 튜닝 전과 같은 정의로.
지표는 ../2026-11-28_c_sfb_q4/seq_s1.py metrics() 그대로 (머리주석): 위치 = 기준 − SLAM 로봇 중심, 창 = 무장 · 기준 속력 > 0.05 m/s,
  큰 슬립 % = 네 바퀴 중 최대 |λ̃| > 0.5 인 시간, 미끄럼 속도 = r (ω − ω_nom) 네 바퀴 중 최대의 RMS, 떨림 = /wheel_torque_cmd 한 틱 차분 RMS,
  상한 붙음 = 한 바퀴라도 바닥 토크 ≥ 0.95 τ̄(μ̂) 인 시간 % (MPC 만, scen.sat_moving).
  여기서 더한 것: 중간 슬립 % = 같은 λ̃ 에서 최대 |λ̃| > 0.25 인 시간 (seq_s1 의 _Lf 창 그대로).
  토크 사용량 (모터 명령 /wheel_torque_cmd τ, 바퀴 /wheel_states ω, 창 = 무장 · 기준 속력 > 0.05):
    토크 RMS · 평균 |τ| · 최대 |τ| [N·m] = 네 바퀴 모두 모아서. 구동 일 = Σ_바퀴 Σ max(τ_i ω_i, 0) Δt, 제동 일 = Σ min(τ_i ω_i, 0) Δt, 순 일 = 합 [J]
    (바퀴마다 나눠 더한다 — 한 바퀴가 제동하는 동안 다른 바퀴가 구동하면 둘 다 센다). ω 는 명령 시각으로 선형 보간.
복귀 다리 (kind return · shape rotate) 는 뺀다.
source install/setup.bash 뒤 /usr/bin/python3 tuned3.py rows   → rows.json (bag 읽기, 오래 걸림)
                              /usr/bin/python3 tuned3.py report → tuned3_out.txt"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
Q4 = os.path.join(HERE, "..", "2026-11-28_c_sfb_q4")
sys.path.insert(0, Q4)
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
B = "/home/smoc/capston/capston/bags/2026-11-28/"

# (기간, 경로 모양, bag, 판 범위 또는 None)
SETS = [("후", "circles8", "220501_tuned_mu060", None), ("후", "zigzag", "220501_tuned_mu060", None), ("후", "polyline", "220501_tuned_mu060", None),
        ("전", "circles8", "183517_seq_sfbq4", None),
        ("전", "zigzag", "184556_seq_sfbq4", (37, 66)), ("전", "zigzag", "194358_seq_sfbq4", (1, 30)),
        ("전", "polyline", "213115_poly", None)]
KEYS = [("pos", "위치 RMS (SLAM 중심) [cm]"), ("pos_max", "위치 최대 [cm]"), ("along", "진행 평균 (+뒤처짐) [cm]"), ("cross_r", "옆 RMS [cm]"),
        ("head", "헤딩 RMS [°]"), ("m25", "슬립 > 25 % 시간 [%]"), ("m50", "슬립 > 50 % 시간 [%]"), ("spin", "큰 슬립 중 헛돎 [%]"),
        ("sv", "미끄럼 속도 RMS [cm/s]"), ("sv20", "미끄럼 속도 > 20 cm/s 시간 [%]"), ("dtick", "모터 명령 떨림 [mN·m]"),
        ("sat", "상한 붙음 [%] (C)"),
        ("tau_rms", "토크 RMS [N·m]"), ("tau_abs", "평균 |τ| [N·m]"), ("tau_pk", "최대 |τ| [N·m]"),
        ("w_drive", "구동 일 [J]"), ("w_brake", "제동 일 [J]"), ("w_net", "순 일 [J]"), ("vref_max", "기준 최고 속력 [m/s]"), ("temp", "모터 온도 [°C]")]


def cmd_rows():
    import seq_s1 as S
    import seq_s1_cause as C
    import sfb_modes as SM
    from scen import sat_moving
    from bag_runs import runs
    cache = {}
    out = []
    for period, shape, bag, rng in SETS:
        path = B + bag
        if bag not in cache:
            print(f"읽는 중 {bag} …", flush=True)
            cache = {bag: (runs(path), S.read(path), C.read_sfb(path), SM.read_cap(path))}
        RS, D, F, CAP = cache[bag]
        for r in RS:
            sq = r.get("seq") or {}
            shp = (r.get("ref") or {}).get("cfg", {}).get("shape") or (r.get("ref") or {}).get("shape")
            if not r["armed"] or sq.get("kind") == "return" or shp != shape or r["end"].startswith("버림"):
                continue
            if rng and not rng[0] <= r["run"] <= rng[1]:
                continue
            (t_on, t_off), = r["armed"][:1]
            c = (r["controller"] or "mpc").lower(); live = r["mpc"] or {}
            mu, sc = live.get("mu_hat", 0.33), live.get("slip_fb_scale", 1.0)
            if c == "mpc" and period == "전" and (round(mu, 2), round(sc, 2)) != (0.33, 1.0):
                continue                                  # 튜닝 전 C 는 0.33 · 1.0 만
            if c == "mpc" and period == "후" and (round(mu, 2), round(sc, 2)) != (0.6, 1.2):
                continue
            try:
                m = S.metrics(D, t_on, t_off)
            except Exception as ex:  # noqa: BLE001
                print(f"  {bag} 판 {r['run']}: 지표 실패 {ex}"); continue
            Lf = m["_Lf"]
            m25 = 100 * float(np.mean(np.abs(Lf).max(1) > 0.25)) if len(Lf) else np.nan
            sat = sat_moving(F, CAP, t_on, t_off, m["_ts"] + t_on, m["_vref"], mu) if c == "mpc" else np.nan
            M, W = D["mot"], D["ws"]
            km = (M[:, 0] >= t_on) & (M[:, 0] <= t_off) & (np.interp(M[:, 0], m["_ts"] + t_on, m["_vref"]) > 0.05)
            tq, tm = M[km, 1:5], M[km, 0]
            om = np.column_stack([np.interp(tm, W[:, 0], W[:, 1 + i]) for i in range(4)])
            pw = tq * om; dt = np.diff(tm, prepend=tm[0]) if len(tm) else tm
            dt = np.minimum(dt, 0.05)                    # 명령이 끊긴 틈 (정지·hold) 은 50 ms 로 자른다
            tor = dict(tau_rms=float(np.sqrt(np.mean(tq ** 2))), tau_abs=float(np.mean(np.abs(tq))), tau_pk=float(np.abs(tq).max()),
                       w_drive=float((np.clip(pw, 0, None) * dt[:, None]).sum()), w_brake=float((np.clip(pw, None, 0) * dt[:, None]).sum()))
            tor["w_net"] = tor["w_drive"] + tor["w_brake"]
            row = {k: (float(v) if np.ndim(v) == 0 else [float(x) for x in v]) for k, v in m.items() if not k.startswith("_")}
            row.update(period=period, shape=shape, bag=bag, run=r["run"], ctrl=c, mu=mu, sc=sc, dir=sq.get("dir"), m25=m25, sat=sat, **tor)
            out.append(row)
            print(f"  {period} {shape:8s} {bag} 판 {r['run']:2d} {c:4s} 위치 {row['pos']:.2f} 큰 {row['m50']:.1f} 중 {m25:.1f} τrms {tor['tau_rms']:.3f} 구동 {tor['w_drive']:.1f} J", flush=True)
    json.dump(out, open(os.path.join(HERE, "rows.json"), "w"), ensure_ascii=False, indent=0)
    print(f"rows.json: {len(out)} 판")


def stat(x, y):
    from scipy.stats import mannwhitneyu, ttest_ind
    x, y = np.asarray(x, float), np.asarray(y, float)
    x, y = x[np.isfinite(x)], y[np.isfinite(y)]
    if len(x) < 2 or len(y) < 2:
        return np.nan, np.nan
    return float(ttest_ind(x, y, equal_var=False).pvalue), float(mannwhitneyu(x, y).pvalue)


def cmd_report():
    rows = json.load(open(os.path.join(HERE, "rows.json")))
    f = open(os.path.join(HERE, "tuned3_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=f)

    for shape in ("circles8", "zigzag", "polyline"):
        G = {(p, c): [m for m in rows if m["shape"] == shape and m["period"] == p and m["ctrl"] == c] for p in ("후", "전") for c in ("mpc", "pp", "adpp")}
        P(f"\n━━ {shape} — 판 수: 후 C {len(G['후', 'mpc'])} · PP {len(G['후', 'pp'])} · ADPP {len(G['후', 'adpp'])} | "
          f"전 C {len(G['전', 'mpc'])} · PP {len(G['전', 'pp'])} · ADPP {len(G['전', 'adpp'])}")
        P(f"{'지표':32}{'후 C':>14}{'후 PP':>14}{'후 ADPP':>14}  {'C−PP p(W/MW)':>20}  {'C−ADPP p(W/MW)':>20}  {'전 C':>14}  {'후C−전C p(W)':>16}")
        for k, lab in KEYS:
            v = {g: np.array([m[k] for m in G[g]], float) for g in G}
            cell = lambda a: f"{np.nanmean(a):6.2f}±{np.nanstd(a, ddof=1) if np.isfinite(a).sum() > 1 else 0:5.2f}" if np.isfinite(a).any() else f"{'-':>12}"
            d = lambda a, b: np.nanmean(a) - np.nanmean(b) if np.isfinite(a).any() and np.isfinite(b).any() else np.nan
            pw1, pm1 = stat(v["후", "mpc"], v["후", "pp"]); pw2, pm2 = stat(v["후", "mpc"], v["후", "adpp"]); pw3, _ = stat(v["후", "mpc"], v["전", "mpc"])
            P(f"{lab:32}{cell(v['후', 'mpc']):>14}{cell(v['후', 'pp']):>14}{cell(v['후', 'adpp']):>14}"
              f"  {d(v['후', 'mpc'], v['후', 'pp']):+7.2f} ({pw1:.3f}/{pm1:.3f})  {d(v['후', 'mpc'], v['후', 'adpp']):+7.2f} ({pw2:.3f}/{pm2:.3f})"
              f"  {cell(v['전', 'mpc']):>14}  {d(v['후', 'mpc'], v['전', 'mpc']):+7.2f} ({pw3:.3f})")
        P("  판별 (후):")
        for c in ("mpc", "pp", "adpp"):
            P(f"   {c:5} " + " ".join(f"#{m['run']}{(m['dir'] or '')[:1]} {m['pos']:.1f}/{m['m25']:.0f}/{m['m50']:.0f}" for m in G["후", c]) + "   (위치 cm / >25 % / >50 %)")
    f.close()


if __name__ == "__main__":
    cmd_rows() if sys.argv[1] == "rows" else cmd_report()
