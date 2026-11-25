#!/usr/bin/python3
"""바퀴축 관성 J_w · 쿨롱 마찰 τ_c · 점성 마찰 b_v 식별 — 바퀴를 띄운 토크 단계 시험 (ak40_bench.py lifted) 의 CSV 로 (2026-11-24,
바퀴 회전자 식별 지시서 W2·W5). ROS 없이 돈다. 식·전처리는 amr_mpc/torque/wheel_dynamics.py 머리주석.

  /usr/bin/python3 tools/wheel_param_id.py analysis/torque_mpc_real/2026-11-24_lifted            # 폴더 안 *_lifted_*.csv 전부
  /usr/bin/python3 tools/wheel_param_id.py <csv…> --fc 10 --out analysis/torque_mpc_real/2026-11-24_lifted
  /usr/bin/python3 tools/wheel_param_id.py --noise-study                                          # 시험 I2: 잡음·차단 주파수별 오차표

하는 일 (바퀴마다)
  1. 전처리: 고른 격자로 옮기고, ω·τ 에 같은 영위상 저역통과 (--fc, 기본 10 Hz [구현 선택]), ω 중앙 차분 → ω̇
  2. 제외: |ω| < --w_min (정지 마찰·부호 전환), |ω| > --w_max (전압 한계 근처), 토크가 바뀐 시각의 앞 --smear/fc 초 ~ 뒤 --smear/fc + --guard 초
           (필터가 계단을 뭉개는 구간 + 명령 지연), 바퀴가 선 자리 근처, 응답 누락 구간, 토크 모드가 아닌 표본 (trip·damping)
  3. 회귀 (식-W2): [ω̇ sgn(ω) ω]·[J_w τ_c b_v]ᵀ = τ — 전진만 / 후진만 / 합쳐서. τ 는 실제 나간 명령값과 피드백 토크로 각각.
     같은 식을 ω̇ 에 대해 푼 회귀 (ω̇ 가 왼쪽) 도 같이 낸다 — 지시서 식 그대로는 속도 잡음이 크면 J_w 가 작게 나온다 (--noise-study)
  4. 교차 확인: 일정 속도 구간만의 직선 τ = τ_c + b_v·|ω|,  자유 감속 구간만의 J_w = (τ − τ_c·sgn ω − b_v·ω)/ω̇
                (τ_c·b_v 를 회귀 값으로 / 일정 속도 직선 값으로 — 뒤쪽은 회귀와 무관한 길이다)
  5. 출력: 바퀴별 표 (R², 잔차 RMS 포함), 설정값 (torque_plant.yaml) 과 비교, 측정 τ 와 식-W2 로 다시 만든 τ 를 겹친 그림,
           W5 영향표 (J_w 가 바뀌면 m_w·m_eq·a_max·μ 가 어떻게 달라지나). **설정 파일은 바꾸지 않는다** — 후보만 낸다.
기본값 출처: w_min 3 · w_max 30 · guard 0.02 는 tools/jw_step_analysis.py 와 같다 (2026-11-16 벤치에서 쓴 값).
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from amr_mpc.ak40 import protocol as P                   # noqa: E402
from amr_mpc.torque import wheel_dynamics as wd          # noqa: E402

WHEELS = ("FL", "FR", "RL", "RR")


def load_csv(path):
    """lifted CSV (+ 같은 이름 .json) → dict. 옛 벤치 CSV (tau_cmd_q·step 열 없음) 도 읽는다 — 그때는 모터 기준 부호이고
    실제 나간 값은 12 비트 반올림으로 되살린다."""
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise ValueError(f"{path}: 빈 파일")
    col = lambda k: np.array([float(r[k]) for r in rows])
    out = {"path": path, "t": col("t"), "mode": np.array([r["mode"] for r in rows]), "v": col("v"),
           "tau_reply": col("tau_reply"), "tau_cmd": col("tau_cmd"), "temp": col("temp")}
    if "tau_cmd_q" in rows[0]:
        out["tau_q"] = col("tau_cmd_q")
    else:
        lim = P.MitLimits()
        out["tau_q"] = np.array([P.uint_to_float(P.float_to_uint(x, -lim.t_max, lim.t_max, 12), -lim.t_max, lim.t_max, 12)
                                 for x in out["tau_cmd"]])
    meta_path = os.path.splitext(path)[0] + ".json"
    out["meta"] = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    return out


def prepare(run, fc=10.0, w_min=3.0, w_max=30.0, guard=0.02, smear=2.0, tail=1.0):
    """한 판 전처리 → dict(t, w, wd, tau_q, tau_fb, valid, coast, steady=[(ω, τ_q, τ_fb)…]).
    valid: 회귀에 쓰는 표본.  coast: 그중 명령이 0 인 (자유 감속) 표본.  steady: 0 이 아닌 단계마다 끝 tail 초 평균."""
    on = run["mode"] == "torque"
    t, (v, tq, tf, tc, onf), ok = wd.resample_uniform(run["t"], run["v"], run["tau_q"], run["tau_reply"], run["tau_cmd"],
                                                      on.astype(float))
    dt = float(t[1] - t[0])
    fs = 1.0 / dt
    w = wd.zero_phase_lowpass(v, fs, fc)
    wdot = wd.central_difference(w, dt)
    tqf, tff = wd.zero_phase_lowpass(tq, fs, fc), wd.zero_phase_lowpass(tf, fs, fc)
    x = smear / fc if fc > 0.0 else 0.0                     # 필터가 계단·정지 꺾임을 뭉개는 길이
    valid = ok & (onf > 0.5) & (np.abs(w) >= w_min) & (np.abs(w) <= w_max) & (t >= t[0] + x) & (t <= t[-1] - x)
    # 원래 표본에서 명령이 바뀐 시각 (고른 격자의 보간값이 아니라 원자료로 찾는다)
    ch = np.flatnonzero(np.abs(np.diff(run["tau_cmd"])) > 1e-9) + 1
    for i in ch:
        ts = run["t"][i]
        valid &= ~((t > ts - x - dt) & (t < ts + x + guard))
    stopped = np.abs(v) < min(1.0, 0.5 * w_min)             # 바퀴가 선 자리: 속도 꺾임이 뭉개지는 구간도 뺀다
    if stopped.any():
        k = int(math.ceil(x / dt))
        dil = np.convolve(stopped.astype(float), np.ones(2 * k + 1), mode="same") > 0.5
        valid &= ~dil
    coast = valid & (np.abs(tc) < 1e-9)
    steady = []
    edges = [0] + list(ch) + [len(run["t"])]
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a < 5 or not on[a:b].all() or abs(run["tau_cmd"][a]) < 1e-9:
            continue
        ta, tb = run["t"][a], run["t"][b - 1]
        if tb - ta < 2.0 * tail:                            # 짧은 단계 (출발용) 는 일정 속도로 안 본다
            continue
        sel = (run["t"][a:b] > tb - tail)
        wm = float(np.mean(run["v"][a:b][sel]))
        if w_min <= abs(wm) <= w_max:
            steady.append((wm, float(np.mean(run["tau_q"][a:b][sel])), float(np.mean(run["tau_reply"][a:b][sel]))))
    return {"t": t, "w": w, "wd": wdot, "tau_q": tqf, "tau_fb": tff, "valid": valid, "coast": coast, "steady": steady,
            "raw_w": v, "dt": dt}


def identify(preps):
    """한 바퀴의 판들 (prepare 결과 목록) → 식별 결과 dict. 키: (tau 종류, 방향) → fit, 그리고 일정 속도 직선·자유 감속."""
    cat = lambda k: np.concatenate([p[k] for p in preps])
    w, wdot, valid, coast = cat("w"), cat("wd"), cat("valid"), cat("coast")
    out = {"n_runs": len(preps)}
    for name, key in (("cmd", "tau_q"), ("fb", "tau_fb")):
        tau = cat(key)
        for dname, dm in (("fwd", w > 0.0), ("rev", w < 0.0), ("both", np.ones(len(w), bool))):
            out[(name, dname)] = wd.fit_wheel_params(tau, w, wdot, valid & dm)                 # 지시서 식 그대로 (τ 가 왼쪽)
            out[(name, dname, "acc")] = wd.fit_wheel_params_accel(tau, w, wdot, valid & dm)    # ω̇ 를 왼쪽에 (잡음에 덜 치우침)
        st = np.array([s for p in preps for s in p["steady"]]).reshape(-1, 3)
        col = 1 if name == "cmd" else 2
        for dname, dm in (("fwd", st[:, 0] > 0.0), ("rev", st[:, 0] < 0.0)):
            line = wd.steady_state_line(st[dm, 0], st[dm, col]) if dm.any() else wd.steady_state_line([], [])
            out[(name, dname, "line")] = line
            m = coast & ((w > 0.0) if dname == "fwd" else (w < 0.0))
            fit = out[(name, dname, "acc")]
            out[(name, dname, "coast_fit")] = wd.coast_inertia(tau, w, wdot, fit["tau_c"], fit["b_v"], m)
            out[(name, dname, "coast_line")] = wd.coast_inertia(tau, w, wdot, line["tau_c"], line["b_v"], m)
    out["valid"], out["n"] = valid, int(valid.sum())
    return out


def impact_table(J_now, J_new, plant):
    """W5: J_w 가 바뀌면 달라지는 값들 [(항목, 지금, 식별값)]. 식은 지시서 W5 표와 같다 — 예시 조건 τ̄ 0.12, τ_slip 0.12, τ_res 0.02."""
    r = float(plant["geometry"]["wheel_radius_m"]); m = float(plant["body"]["mass_kg"]); g = float(plant["model"]["gravity"])
    k = 0.5 * (float(plant["geometry"]["wheel_base_m"]) + float(plant["geometry"]["robot_width_m"]))
    rows = []
    f = lambda J: (4 * J / r ** 2, m + 4 * J / r ** 2, 4 * J * k * k / r ** 2, 4 * 0.12 / (r * (m + 4 * J / r ** 2)),
                   4 * math.sqrt(2) * (0.12 - 0.02) / ((m + 4 * J / r ** 2) * r * g))
    for name, a, b in zip(("바퀴 등가 질량 m_w = 4J_w/r² [kg]", "전체 등가 질량 m_eq = m + m_w [kg]",
                           f"회전 등가 관성 증가분 4J_w·k²/r² [kg·m²] (k {k:.4f})", "직진 최대 가속 4·0.12/(r·m_eq) [m/s²]",
                           "마찰계수 예시 4√2·(0.12 − 0.02)/(m_eq·r·g)"), f(J_now), f(J_new)):
        rows.append((name, a, b))
    return rows


def noise_study(sigmas=(0.0064, 0.3, 0.65), fcs=(5.0, 10.0, 20.0), trials=20, seed=0,
                J=9.7e-4, tau_c=0.020, b_v=5e-4, **prep_kw):
    """시험 I2: 합성 자료 (지시서 예시값) 의 ω 에 백색잡음 σ [rad/s] 를 넣고 차단 주파수별 상대 오차 [%] (평균 ± 표준편차).
    σ 0.0064 = 속도 기록 한 칸 0.0222/√12 (정지 구간), 0.3~0.65 = 2026-11-16 벤치에서 도는 중 실측한 흔들림."""
    rng = np.random.default_rng(seed)
    taus, holds = [0.03, 0.035, 0.025, 0.03, 0.0], [8.0, 8.0, 8.0, 8.0, 3.0]
    out = []
    for sg in sigmas:
        for fc in fcs:
            errs = []
            for _ in range(trials):
                preps = []
                for sgn in (1.0, -1.0):
                    t, tau, w = wd.simulate_lifted([sgn * x for x in taus], holds, J, tau_c, b_v)
                    run = {"t": t, "mode": np.array(["torque"] * len(t)), "v": w + rng.normal(0.0, sg, len(w)),
                           "tau_q": tau, "tau_reply": tau, "tau_cmd": tau}
                    preps.append(prepare(run, fc=fc, **prep_kw))
                res = identify(preps)
                f, g = res[("cmd", "both")], res[("cmd", "both", "acc")]
                errs.append([f["J_w"] / J - 1.0, f["tau_c"] / tau_c - 1.0, f["b_v"] / b_v - 1.0,
                             g["J_w"] / J - 1.0, g["tau_c"] / tau_c - 1.0, g["b_v"] / b_v - 1.0])
            e = 100.0 * np.array(errs)
            out.append({"sigma": sg, "fc": fc, "mean": e.mean(0)[:3], "std": e.std(0)[:3],
                        "mean_acc": e.mean(0)[3:], "std_acc": e.std(0)[3:]})
    return out


def _fmt_fit(f):
    return (f"{f['J_w']*1e4:6.2f} | {f['tau_c']*1e3:6.2f} | {f['b_v']*1e3:6.3f} | {f['r2']:.4f} | {f['rms']*1e3:5.2f} | {f['n']:5d}"
            if f["n"] >= 6 else "   —   |   —    |   —    |   —    |   —   | %5d" % f["n"])


def report(by_wheel, plant, fc, out_dir=""):
    """바퀴별 표·W5 영향표를 문자열로 (out_dir 이 있으면 wheel_param_id.md 로도 쓴다)."""
    L = []
    cfg_J = float(plant["wheel"]["inertia_J_w_kgm2"])
    cfg_c, cfg_b = plant["friction"]["coulomb_Nm"], plant["friction"]["viscous_Nms_per_rad"]
    L.append(f"# 바퀴 띄운 토크 단계 시험 식별 결과 (필터 {fc:g} Hz 영위상)\n")
    L.append("단위: J_w [×10⁻⁴ kg·m²], τ_c [mN·m], b_v [mN·m·s/rad], 잔차 RMS [mN·m]. 부호는 바퀴 기준 (전진 +).\n")
    Jc = []
    for wi in sorted(by_wheel):
        res, name = by_wheel[wi], WHEELS[wi]
        L.append(f"## {name}  (판 {res['n_runs']} 개, 쓴 표본 {res['n']})\n")
        L.append(f"설정값 (torque_plant.yaml): J_w {cfg_J*1e4:.2f}, τ_c {cfg_c[wi]*1e3:.2f}, b_v {cfg_b[wi]*1e3:.3f}\n")
        L.append("| τ | 방향 | 회귀 | J_w | τ_c | b_v | R² | 잔차 RMS | 표본 |")
        L.append("|---|---|---|---|---|---|---|---|---|")
        for tn, tl in (("cmd", "명령 (실제 나간 값)"), ("fb", "피드백")):
            for dn, dl in (("fwd", "전진"), ("rev", "후진"), ("both", "합쳐서")):
                L.append(f"| {tl} | {dl} | ω̇ 왼쪽 | " + _fmt_fit(res[(tn, dn, "acc")]) + " |")
                L.append(f"| {tl} | {dl} | 지시서 식 | " + _fmt_fit(res[(tn, dn)]) + " |")
        L.append("\n교차 확인 (명령 기준):\n")
        L.append("| 방향 | 일정 속도 직선 τ_c · b_v (점 수) | 자유 감속 J_w: 회귀 (ω̇ 왼쪽) 마찰로 (사분위) | 자유 감속 J_w: 직선 마찰로 (사분위) |")
        L.append("|---|---|---|---|")
        for dn, dl in (("fwd", "전진"), ("rev", "후진")):
            ln, c1, c2 = res[("cmd", dn, "line")], res[("cmd", dn, "coast_fit")], res[("cmd", dn, "coast_line")]
            L.append(f"| {dl} | {ln['tau_c']*1e3:.2f} · {ln['b_v']*1e3:.3f} ({ln['n']}) | "
                     f"{c1['J_w']*1e4:.2f} ({c1['q25']*1e4:.2f}~{c1['q75']*1e4:.2f}, {c1['n']}) | "
                     f"{c2['J_w']*1e4:.2f} ({c2['q25']*1e4:.2f}~{c2['q75']*1e4:.2f}, {c2['n']}) |")
        L.append("")
        if res[("cmd", "both", "acc")]["n"] >= 6 and math.isfinite(res[("cmd", "both", "acc")]["J_w"]):
            Jc.append(res[("cmd", "both", "acc")]["J_w"])
    if Jc:
        Jm = float(np.mean(Jc))
        L.append(f"## W5 영향표 — 식별값 = 바퀴별 '명령·합쳐서·ω̇ 왼쪽' J_w 의 평균 {Jm*1e4:.2f}×10⁻⁴ (설정은 바꾸지 않는다)\n")
        L.append("| 항목 | 지금 (설정 J_w %.2f×10⁻⁴) | 식별값 | 차 |" % (cfg_J * 1e4))
        L.append("|---|---|---|---|")
        for name, a, b in impact_table(cfg_J, Jm, plant):
            L.append(f"| {name} | {a:.4f} | {b:.4f} | {100*(b/a-1):+.1f} % |")
    txt = "\n".join(L) + "\n"
    if out_dir:
        with open(os.path.join(out_dir, "wheel_param_id.md"), "w") as f:
            f.write(txt)
    return txt


def plot_wheel(name, preps, res, path):
    """측정 τ (명령·피드백) 와 식-W2 로 다시 만든 τ = J_w·ω̇ + τ_c·sgn ω + b_v·ω 를 겹친 그림 (판마다 한 줄)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = [n for n in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic") if n in have] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    f = res[("cmd", "both", "acc")]
    n = len(preps)
    fig, ax = plt.subplots(n, 1, figsize=(10, 2.2 * n + 0.6), squeeze=False)
    for k, p in enumerate(preps):
        a = ax[k, 0]
        rec = f["J_w"] * p["wd"] + f["tau_c"] * np.sign(p["w"]) + f["b_v"] * p["w"]
        a.plot(p["t"], p["tau_q"] * 1e3, color="0.6", lw=1.0, label="명령 (필터 뒤)")
        a.plot(p["t"], p["tau_fb"] * 1e3, color="k", lw=0.8, label="피드백 (필터 뒤)")
        a.plot(p["t"], np.where(p["valid"], rec, np.nan) * 1e3, color="tab:red", lw=1.2, label="식-W2 로 다시 만든 τ (회귀에 쓴 구간)")
        a.set_ylabel("토크 [mN·m]")
        b = a.twinx()
        b.plot(p["t"], p["w"], color="tab:blue", lw=0.7, alpha=0.6)
        b.set_ylabel("ω [rad/s]", color="tab:blue")
        if k == 0:
            a.legend(loc="upper right", fontsize=7, ncol=3)
            a.set_title(f"{name}: J_w {f['J_w']*1e4:.2f}e-4, τ_c {f['tau_c']*1e3:.1f} mN·m, b_v {f['b_v']*1e3:.3f} mN·m·s/rad, "
                        f"R² {f['r2']:.3f}, 잔차 RMS {f['rms']*1e3:.2f} mN·m", fontsize=9)
    ax[-1, 0].set_xlabel("시간 [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="lifted CSV 파일들 또는 그 폴더")
    ap.add_argument("--fc", type=float, default=10.0, help="영위상 저역통과 차단 주파수 [Hz]")
    ap.add_argument("--w_min", type=float, default=3.0, help="이보다 느린 표본은 뺀다 [rad/s]")
    ap.add_argument("--w_max", type=float, default=30.0, help="이보다 빠른 표본은 뺀다 [rad/s]")
    ap.add_argument("--guard", type=float, default=0.02, help="토크가 바뀐 뒤 더 빼는 시간 [s] (명령 지연·전류 루프 과도)")
    ap.add_argument("--smear", type=float, default=2.0, help="토크가 바뀐 시각 앞뒤로 빼는 길이 = smear/fc [s]")
    ap.add_argument("--out", default="", help="결과 폴더 (wheel_param_id.md, fig_wheel_<바퀴>.png). 비우면 화면에만")
    ap.add_argument("--noise-study", dest="noise", action="store_true", help="시험 I2 의 잡음·차단 주파수별 오차표만 낸다")
    a = ap.parse_args()
    if a.noise:
        print("오차 % = 평균 ± 표준편차.  [지시서 식: τ 가 왼쪽]  /  [ω̇ 가 왼쪽]")
        print("σ_ω [rad/s] | 차단 [Hz] | J_w | τ_c | b_v  /  J_w | τ_c | b_v")
        for r in noise_study(w_min=a.w_min, w_max=a.w_max, guard=a.guard, smear=a.smear):
            f = lambda m, s: " | ".join(f"{m[i]:+7.2f} ± {s[i]:5.2f}" for i in range(3))
            print(f"{r['sigma']:10.4f} | {r['fc']:5.0f} | {f(r['mean'], r['std'])}  /  {f(r['mean_acc'], r['std_acc'])}")
        return
    files = []
    for p in a.paths:
        files += sorted(glob.glob(os.path.join(p, "*_lifted_*.csv"))) if os.path.isdir(p) else sorted(glob.glob(p) or [p])
    if not files:
        raise SystemExit("CSV 가 없다 — 파일이나 폴더를 준다")
    from amr_mpc.core import config
    plant = config.load("torque_plant.yaml")
    by_wheel, preps_by = {}, {}
    for f in files:
        run = load_csv(f)
        wi = run["meta"].get("wheel")
        wi = WHEELS.index(wi) if wi in WHEELS else next((i for i, n in enumerate(WHEELS) if f"_{n}_" in os.path.basename(f)), None)
        if wi is None:
            print(f"건너뜀 (어느 바퀴인지 모름 — .json 이 없고 파일 이름에도 없다): {f}")
            continue
        if run["meta"].get("frame", "wheel") != "wheel":
            print(f"주의: {os.path.basename(f)} 는 모터 기준 부호로 기록됐다 — 전진/후진 표시는 모터 +/− 다")
        preps_by.setdefault(wi, []).append(prepare(run, a.fc, a.w_min, a.w_max, a.guard, a.smear))
    for wi, preps in preps_by.items():
        by_wheel[wi] = identify(preps)
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        for wi, preps in preps_by.items():
            plot_wheel(WHEELS[wi], preps, by_wheel[wi], os.path.join(a.out, f"fig_wheel_{WHEELS[wi]}.png"))
    print(report(by_wheel, plant, a.fc, a.out))


if __name__ == "__main__":
    main()
