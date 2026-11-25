#!/usr/bin/python3
"""계단 변조 8자 (circles8 width 1.6, speed 0.69 → 0.345/1.035 계단, accel 2.0, 바닥 μ 0.55) — C vs PP, 2 판씩.
npz: raw = [t, x, y, yaw, Xr(3), ur(3)] (참값 포즈 + 같은 시각 기준), lam = [t, 참값 λ×4], st = /torque_mpc/status (MPC 만).
  주행 창 = 기준 속력 > 0.02 인 첫~마지막 행.  e_t = 기준 속도 방향 성분 (+ 앞섬 / − 뒤처짐),  e_n = 그 왼쪽 법선 성분.
  큰 슬립 = max_i |λ_i| > 0.5 시간 %, 헛돎 (λ > 0) / 끌림 (λ < 0) 로 나눔.  MPC 포화 = max_i |τ_i|/τ̄_i ≥ 0.98."""
import json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, os.environ.get("RUNS", "runs069"))   # runs/ 는 speed 0.70 — 부하 100.47 % 로 기준 노드가 거부한 판들
TB = {"C": np.array([0.1093, 0.1113, 0.1154, 0.1055]), "B": np.full(4, 1.2)}


def met(lab):
    if not os.path.exists(f"{R}/{lab}.npz"):
        return None
    js = json.loads(open(f"{R}/{lab}.json").read().strip().splitlines()[-1])
    z = np.load(f"{R}/{lab}.npz"); raw, lam = z["raw"], z["lam"]; st = z["st"] if "st" in z.files else np.zeros((0, 26))
    t = raw[:, 0]; c, s = np.cos(raw[:, 6]), np.sin(raw[:, 6])
    vw = np.stack([c * raw[:, 7] - s * raw[:, 8], s * raw[:, 7] + c * raw[:, 8]], 1); vr = np.linalg.norm(vw, axis=1)
    mv = vr > 0.02; i0, i1 = np.argmax(mv), len(mv) - 1 - np.argmax(mv[::-1]); win = np.zeros(len(t), bool); win[i0:i1 + 1] = True
    d = raw[:, 1:3] - raw[:, 4:6]; tt = vw / np.maximum(vr, 1e-9)[:, None]; nn = np.stack([-tt[:, 1], tt[:, 0]], 1)
    et, en = (d * tt).sum(1), (d * nn).sum(1); e = np.linalg.norm(d, axis=1)
    eh = np.degrees(np.abs(np.arctan2(np.sin(raw[:, 3] - raw[:, 6]), np.cos(raw[:, 3] - raw[:, 6]))))
    lm = (lam[:, 0] >= t[i0]) & (lam[:, 0] <= t[i1]); L = lam[lm, 1:5]; L = L[np.all(np.isfinite(L), 1)]
    a = np.abs(L); mx = a.max(1); big = mx > 0.5; sg = np.sign(L[np.arange(len(L)), a.argmax(1)])
    out = dict(lab=lab, e_rms=1e3 * float(np.sqrt(np.mean(e[win] ** 2))), e_max=1e3 * float(e[win].max()),
               et=1e3 * float(np.mean(et[mv])), en=1e3 * float(np.sqrt(np.mean(en[mv] ** 2))), h_rms=float(np.sqrt(np.mean(eh[win] ** 2))),
               big=100 * float(big.mean()), drive=100 * float((big & (sg > 0)).mean()), brake=100 * float((big & (sg < 0)).mean()),
               lmax=float(mx.max()), lrms=float(np.sqrt(np.mean(L ** 2))), fault=js.get("fault") or "", sat=float("nan"), tau=float("nan"))
    g = lab.split("_")[0]
    if g in TB and len(st):
        sm = (st[:, 0] >= t[i0]) & (st[:, 0] <= t[i1]) & (st[:, 20] < 0.5)
        out["sat"] = 100 * float(np.mean((np.abs(st[sm, 4:8]) / TB[g]).max(1) >= 0.98))
        out["tau"] = float(np.abs(st[sm, 4:8]).mean())
    return out


if __name__ == "__main__":
    rows = {lab: met(lab) for lab in ("C_r1", "C_r2", "PP_r1", "PP_r2")}
    json.dump(rows, open(os.path.join(HERE, "summary.json"), "w"), indent=1)
    print("| 판 | 보상 | 위치 오차 RMS / 최대 [mm] | 진행 방향 평균 (− 뒤처짐) | 옆 방향 RMS | 헤딩 RMS [°] | 큰 슬립 % | 헛돎 / 끌림 % | λ 최대 | λ RMS "
          "| MPC 포화 % | MPC |τ| 평균 | fault |")
    print("|" + "---|" * 13)
    for lab, m in rows.items():
        if not m:
            print(f"| {lab} | (없음) |"); continue
        g, c = lab.split("_")
        print(f"| {'A' if g == 'PP' else g} | {c} | {m['e_rms']:.0f} / {m['e_max']:.0f} | {m['et']:+.0f} | {m['en']:.0f} | {m['h_rms']:.1f} "
              f"| {m['big']:.1f} | {m['drive']:.1f} / {m['brake']:.1f} | {m['lmax']:.2f} | {m['lrms']:.3f} | {m['sat']:.0f} | {m['tau']:.3f} | {m['fault'] or '-'} |")
