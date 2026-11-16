#!/usr/bin/python3
"""바퀴 축관성 J_w — 돌고 있는 중 토크 계단의 앞뒤 가속도 차로 잰다 (2026-11-16). ROS 없이 돈다.

  측정 (로봇을 받침대에 올려 네 바퀴를 띄우고, ROS 스택은 끈다. 바퀴마다, 3 회씩) — 2026-11-16 실측 순서:
    /usr/bin/python3 tools/ak40_bench.py steps --id 1 --taus 0.09,0.05 --hold 5                  # 워밍업 (0.09 로 출발 → 0.05 유지)
    /usr/bin/python3 tools/ak40_bench.py steps --id 1 --taus 0.08,0.035,0.055,0.035,0.055,0.035,0.055,0.035,0.055,0 --hold 1.0 --csv jw_1_1.csv
      ※ 워밍업을 torque --tau 0.06 으로 하면 FL 이 정지 마찰을 못 넘는다 (끝속도 0). 0.05/0.08 왕복은 0.08 에서 약 40 rad/s 로
        전압 한계에 닿아 응답 토크가 명령보다 0.010 낮다 → 계단 절반이 빠지고 흩어진다. 마찰이 낮은 바퀴는 0.03/0.045 도 된다.
  분석 (대표값은 중앙값 — 실제 속도 잡음이 0.3~0.65 rad/s 라 계단 하나의 J 가 ±30 % 흩어진다):
    /usr/bin/python3 tools/jw_step_analysis.py jw_1_*.csv
    /usr/bin/python3 tools/jw_step_analysis.py --taus 0.1,0.06,0.04,0.03,0.02 --hold 3 stepsdown_1.csv   # τ_명령 열이 없는 옛 CSV

  원리: 바퀴 하나의 운동  J·ω̇ = τ − τ_f(ω).  계단 순간 t_s 에서 ω 는 연속이라 앞뒤 마찰 τ_f(ω(t_s)) 가 같다 →
        J·(ω̇⁺ − ω̇⁻) = τ⁺ − τ⁻   →   J = Δτ / Δω̇
        ω̇⁻·ω̇⁺ 는 계단 앞·뒤 window 초를 ω̇ = α − β·ω 꼴(지수, β 앞뒤 공유)로 맞춰 t_s 에서의 기울기로 구한다 (guard 초는 명령 지연이라 뺀다).
        τc·sign(ω) + bv·ω 같은 마찰 모양을 가정하지 않는다 — 6e-4 ~ 1.0e-3 로 흩어진 예전 추정(2026-11-16)의 원인이 마찰 모델이었다.
  가정: 계단 앞뒤로 마찰이 토크 크기에 따라 바뀌지 않는다 (감속기 부하 의존 마찰이 크면 J 가 1/η 만큼 커 보인다).
        τ_명령은 드라이버가 Iq = τ/K_T 로 내는 값이다 — K_T 가 틀리면 같은 비율로 틀린다 (응답 토크도 같은 K_T 를 쓴다).
  검증 (합성 자료, Stribeck 정지 마찰 0.08·워밍업 15 %·명령 지연 5 ms·속도 양자화 0.022 rad/s·100 Hz ± 1 ms, 3 판):
        정답 1.0e-3 → +0.4 %, 8.9e-4 → +0.4 %, 6.0e-4 → +0.3 %. 부하 의존 마찰 10 % 를 넣으면 +11 %.
        합성의 속도 잡음은 실제(0.3~0.65 rad/s)보다 작다 — 실측은 계단 20 개 이상의 중앙값을 쓴다.
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def read_csv(path, taus=None, hold=None):
    """ak40_bench.py --csv 출력 → (t, mode, v, tau_reply, tau_cmd). τ_명령 열이 없으면 taus·hold 로 되살린다."""
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t"]) for r in rows])
    mode = np.array([r["mode"] for r in rows])
    v = np.array([float(r["v"]) for r in rows])
    tr = np.array([float(r["tau_reply"]) for r in rows])
    if rows and "tau_cmd" in rows[0]:
        tc = np.array([float(r["tau_cmd"]) for r in rows])
    else:
        if taus is None or hold is None:
            raise SystemExit(f"{path}: tau_cmd 열이 없다 — --taus 와 --hold 로 계단 순서를 준다")
        on = mode == "torque"
        t0 = t[on][0]
        idx = np.clip(((t - t0) // hold).astype(int), 0, len(taus) - 1)
        tc = np.where(on, np.asarray(taus, float)[idx], 0.0)
    return t, mode, v, tr, tc


def _g(beta, s):
    """(1 − e^(−β·s))/β — β → 0 이면 s. ω = ω_s + ω̇(t_s)·g(β, t − t_s) 는 ω̇ = α − β·ω 의 해다."""
    return s if beta < 1e-9 else -np.expm1(-beta * s) / beta


def _shared_beta_fit(sb, vb, sa, va, betas=np.linspace(0.0, 30.0, 301)):
    """계단 앞(sb, vb)·뒤(sa, va) [s = t − t_s] 를 ω̇ = α − β·ω 꼴(지수)로 맞춘다. β = b_loc/J 는 같은 ω 라 앞뒤 공유,
    절편·계단 순간 기울기는 앞뒤 따로 (명령 지연이 있어 이음 연속을 강제하지 않는다). 반환 (ω̇⁻, ω̇⁺, ω⁻, ω⁺, β)."""
    nb, na = len(sb), len(sa)
    y = np.concatenate([vb, va])

    def solve(beta):
        A = np.zeros((nb + na, 4))
        A[:nb, 0] = 1.0; A[nb:, 1] = 1.0
        A[:nb, 2] = _g(beta, sb); A[nb:, 3] = _g(beta, sa)
        c, *_ = np.linalg.lstsq(A, y, rcond=None)
        return float(np.sum((A @ c - y) ** 2)), c

    res = [solve(b)[0] for b in betas]
    k = int(np.argmin(res))
    lo, hi = betas[max(k - 1, 0)], betas[min(k + 1, len(betas) - 1)]
    for _ in range(30):                                # 황금분할로 다듬기
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        if solve(m1)[0] < solve(m2)[0]:
            hi = m2
        else:
            lo = m1
    beta = 0.5 * (lo + hi)
    c = solve(beta)[1]
    return c[2], c[3], c[0], c[1], beta


def step_estimates(t, mode, v, tau_reply, tau_cmd, window=0.15, guard=0.02, w_min=3.0, w_max=39.0, min_dtau=0.005):
    """계단마다 dict(ts, tau_before, tau_after, w, wdot_before, wdot_after, J_cmd, J_reply, gap). 조건이 안 맞는 계단은 이유와 함께."""
    out = []
    on = mode == "torque"
    change = np.flatnonzero(on[1:] & on[:-1] & (np.abs(np.diff(tau_cmd)) >= min_dtau)) + 1
    for i in change:
        ts = t[i]
        b = on & (t >= ts - guard - window) & (t < ts - guard)
        a = on & (t > ts + guard) & (t <= ts + guard + window)
        rec = {"ts": ts, "tau_before": tau_cmd[i - 1], "tau_after": tau_cmd[i]}
        if b.sum() < 6 or a.sum() < 6:
            rec["skip"] = "window 안 샘플 부족 (계단 사이가 짧거나 토크 모드가 아님)"
        elif np.min(np.abs(v[a | b])) < w_min:
            rec["skip"] = f"|ω| < {w_min} rad/s (정지 마찰·0 지남)"
        elif np.max(np.abs(v[a | b])) > w_max:
            rec["skip"] = f"|ω| > {w_max} rad/s (전압 한계 근처)"
        elif np.ptp(tau_cmd[b]) >= min_dtau or np.ptp(tau_cmd[a]) >= min_dtau:
            rec["skip"] = "window 안에 다른 계단 (hold 가 window + guard 보다 짧다)"
        else:
            wd_b, wd_a, w_b, w_a, beta = _shared_beta_fit(t[b] - ts, v[b], t[a] - ts, v[a])
            dtau = tau_cmd[i] - tau_cmd[i - 1]
            dtr = tau_reply[a].mean() - tau_reply[b].mean()
            rec.update(w=0.5 * (w_a + w_b), gap=w_a - w_b, wdot_before=wd_b, wdot_after=wd_a, beta=beta,
                       J_cmd=dtau / (wd_a - wd_b), J_reply=dtr / (wd_a - wd_b), dtau_reply=dtr)
        out.append(rec)
    return out


def summarize(J):
    J = np.asarray(J, float)
    return J.mean(), (J.std(ddof=1) if len(J) > 1 else math.nan), len(J)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="ak40_bench.py --csv 파일들 (glob 가능)")
    ap.add_argument("--window", type=float, default=0.15, help="계단 앞·뒤 맞춤 구간 [s]")
    ap.add_argument("--guard", type=float, default=0.02, help="계단 바로 앞뒤로 빼는 구간 [s] (명령 지연)")
    ap.add_argument("--w_min", type=float, default=3.0, help="이보다 느린 계단은 뺀다 [rad/s]")
    ap.add_argument("--w_max", type=float, default=30.0,
                    help="이보다 빠른 계단은 뺀다 [rad/s]. 2026-11-16 실차: 0.08 N·m 에서 약 40 rad/s 로 전압 한계에 닿아 응답 토크가 명령보다 0.010 낮았다")
    ap.add_argument("--taus", default="", help="τ_명령 열이 없는 옛 CSV 의 계단 순서 (쉼표)")
    ap.add_argument("--hold", type=float, default=None, help="옛 CSV 의 계단 길이 [s]")
    a = ap.parse_args()
    taus = [float(x) for x in a.taus.split(",")] if a.taus else None
    files = sorted({f for p in a.csv for f in (glob.glob(p) or [p])})
    sys.path.insert(0, os.path.join(HERE, ".."))
    try:
        from amr_mpc.torque.model import TorquePlant
        p = TorquePlant.from_yaml()
        J_cfg, r = p.J_w, p.r
    except Exception:                                  # 설정 없이도 돈다
        J_cfg, r = math.nan, 0.04
    allJ, allR = [], []
    for f in files:
        t, mode, v, tr, tc = read_csv(f, taus, a.hold)
        est = step_estimates(t, mode, v, tr, tc, a.window, a.guard, a.w_min, a.w_max)
        print(f"== {os.path.basename(f)}")
        print("   t[s]  | τ 명령 앞→뒤 | ω rad/s | ω̇ 앞 → 뒤 rad/s² | 이음 차 rad/s | J(명령) kg·m² | J(응답) kg·m²")
        Jf = []
        for e in est:
            if "skip" in e:
                print(f"  {e['ts']:6.2f} | {e['tau_before']:.3f}→{e['tau_after']:.3f} | 뺌: {e['skip']}")
                continue
            Jf.append(e["J_cmd"]); allR.append(e["J_reply"])
            print(f"  {e['ts']:6.2f} | {e['tau_before']:.3f}→{e['tau_after']:.3f} | {e['w']:6.1f}  | {e['wdot_before']:+7.1f} → {e['wdot_after']:+7.1f} "
                  f"| {e['gap']:+6.2f}        | {e['J_cmd']:.3e}     | {e['J_reply']:.3e}")
        if Jf:
            m, s, n = summarize(Jf)
            print(f"   → J = {m:.3e} ± {s:.1e} (계단 {n} 개)")
        allJ += Jf
    if allJ:
        m, s, n = summarize(allJ)
        mr, sr, _ = summarize(allR)
        med, q1, q3 = np.median(allJ), np.percentile(allJ, 25), np.percentile(allJ, 75)
        print(f"\n전체 J(명령 토크) 중앙값 = {med:.3e} kg·m² (사분위 {q1:.2e}~{q3:.2e}, 계단 {n} 개)   ← 대표값")
        print(f"             평균 ± 표준편차 = {m:.3e} ± {s:.1e} (평균의 표준오차 {s/math.sqrt(n):.1e})")
        print(f"전체 J(응답 토크) 중앙값 = {np.median(allR):.3e} (평균 {mr:.3e} ± {sr:.1e})")
        print(f"→ 바퀴 등가 질량 μ = 4J/r² = {4*med/r**2:.3f} kg (중앙값)   (지금 torque_plant.yaml J_w {J_cfg:.2e} → {4*J_cfg/r**2:.3f} kg)")


if __name__ == "__main__":
    main()
