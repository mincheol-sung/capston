#!/usr/bin/python3
"""꼭짓점 pass vs stop — PP(A)·B·C, 시뮬 참값 (바닥 μ 0.45, accel 1.5, v_max 1.5, tangent).
npz: raw = [t, x, y, yaw, Xr(3), ur(3)] (참값 포즈 + 같은 시각 기준), lam = [t, λ×4], st = /torque_mpc/status (MPC 만).
  주행 창   : 기준 속력 > 0.02 또는 기준 |ω| > 0.02 인 첫 행 ~ 마지막 행 (stop 의 꼭짓점 회전 포함)
  시간 추종 : |p − p_r| RMS (주행 창)        윤곽 : 정사각형 변까지 거리 (주행 창 RMS·최대)
  꼭짓점    : 꼭짓점 0.3 m 안에서 윤곽 최대   진행 방향 : 기준 속도 방향 성분 평균 (− 뒤처짐, 기준 이동 중만)
  헤딩      : |ψ − ψ_r| RMS·최대 (주행 창)   큰 슬립 : 주행 창에서 max_i |λ_i| > 0.5 비율 (λ>0 헛돎 · λ<0 끌림, slip.py 가 나눈다), λ 최대
  MPC 포화  : max_i |τ_i| / τ̄_i ≥ 0.98 (C 는 바퀴별 τ̄, B 는 1.2),  앞먹임 초과 : max_i |U_s,i| / τ̄_i > 1."""
import json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, "runs"); SIDE = 1.5
CORNERS = np.array([(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE)], float)
TB = {"C": np.array([0.1093, 0.1113, 0.1154, 0.1055]), "B": np.full(4, 1.2)}


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def contour(px, py):
    def seg(a, b):
        u = b - a; t = np.clip(((px - a[0]) * u[0] + (py - a[1]) * u[1]) / (u @ u), 0, 1)
        return np.hypot(px - a[0] - t * u[0], py - a[1] - t * u[1])
    return np.min([seg(CORNERS[i], CORNERS[(i + 1) % 4]) for i in range(4)], axis=0)


def met(lab):
    path = f"{R}/{lab}.npz"
    if not os.path.exists(path):
        return None
    js = json.loads(open(f"{R}/{lab}.json").read().strip().splitlines()[-1])
    z = np.load(path); raw, lam = z["raw"], z["lam"]; st = z["st"] if "st" in z.files else np.zeros((0, 26))
    t = raw[:, 0]; p, pr = raw[:, 1:3], raw[:, 4:6]
    c, s = np.cos(raw[:, 6]), np.sin(raw[:, 6])
    vw = np.stack([c * raw[:, 7] - s * raw[:, 8], s * raw[:, 7] + c * raw[:, 8]], 1); vr = np.linalg.norm(vw, axis=1)
    act = (vr > 0.02) | (np.abs(raw[:, 9]) > 0.02)
    i0, i1 = np.argmax(act), len(act) - 1 - np.argmax(act[::-1]); win = np.zeros(len(t), bool); win[i0:i1 + 1] = True
    mv = vr > 0.02
    e = np.linalg.norm(p - pr, axis=1)
    tt = vw / np.maximum(vr, 1e-9)[:, None]; e_al = ((pr - p) * tt).sum(1)          # + 면 기준이 앞 (로봇이 뒤처짐)
    d = contour(p[:, 0], p[:, 1]); near = np.min(np.linalg.norm(p[:, None, :] - CORNERS[None], axis=2), axis=1) < 0.3
    eh = np.abs(wrap(raw[:, 3] - raw[:, 6]))
    lm = (lam[:, 0] >= t[i0]) & (lam[:, 0] <= t[i1]); mx = np.nanmax(np.abs(lam[lm, 1:5]), axis=1) if lm.any() else np.zeros(1)
    out = dict(lab=lab, T=float(t[i1] - t[i0]), e_rms=1e3 * float(np.sqrt(np.mean(e[win] ** 2))), e_max=1e3 * float(e[win].max()),
               c_rms=1e3 * float(np.sqrt(np.mean(d[win] ** 2))), c_max=1e3 * float(d[win].max()),
               c_corner=1e3 * float(d[win & near].max()) if (win & near).any() else float("nan"),
               lag=-1e3 * float(np.mean(e_al[mv])), h_rms=math.degrees(float(np.sqrt(np.mean(eh[win] ** 2)))),
               h_max=math.degrees(float(eh[win].max())), spin=100 * float(np.mean(mx > 0.5)), lmax=float(mx.max()),
               e_end=1e3 * float(e[-1]), fault=js.get("fault") or "", sat=float("nan"), ff=float("nan"))
    g = lab.split("_")[0]
    if g in TB and len(st):
        sm = (st[:, 0] >= t[i0]) & (st[:, 0] <= t[i1]) & (st[:, 20] < 0.5)
        out["sat"] = 100 * float(np.mean((np.abs(st[sm, 4:8]) / TB[g]).max(1) >= 0.98))
        out["ff"] = 100 * float(np.mean((np.abs(st[sm, 22:26]) / TB[g]).max(1) > 1.0))
    return out


if __name__ == "__main__":
    rows = {}
    for mode in ("pass", "stop"):
        for g in ("PP", "B", "C"):
            rows[(g, mode)] = [m for m in (met(f"{g}_{mode}_r{k}") for k in (1, 2)) if m]
    json.dump({f"{g}_{m}": v for (g, m), v in rows.items()}, open(os.path.join(HERE, "summary.json"), "w"), indent=1)
    f = lambda v, k, fmt="{:.0f}": "/".join(fmt.format(x[k]) for x in v) if v else "-"
    print("| 판 | 주행 | 시간 추종 RMS / 최대 [mm] | 윤곽 RMS / 최대 [mm] | 꼭짓점 0.3 m 안 윤곽 최대 | 진행 방향 평균 (− 뒤처짐) | 헤딩 RMS / 최대 [°] "
          "| 큰 슬립 % (|λ|>0.5) | λ 최대 | 포화 % | 앞먹임 초과 % | 끝 오차 | fault |")
    print("|" + "---|" * 13)
    for (g, mode), v in rows.items():
        if not v:
            print(f"| {g} {mode} | (없음) |"); continue
        print(f"| {g} {mode} | {f(v, 'T', '{:.1f}')} s | {f(v, 'e_rms')} / {f(v, 'e_max')} | {f(v, 'c_rms')} / {f(v, 'c_max')} | {f(v, 'c_corner')} "
              f"| {f(v, 'lag')} | {f(v, 'h_rms', '{:.1f}')} / {f(v, 'h_max', '{:.0f}')} | {f(v, 'spin', '{:.1f}')} | {f(v, 'lmax', '{:.2f}')} "
              f"| {f(v, 'sat')} | {f(v, 'ff')} | {f(v, 'e_end')} | {','.join(x['fault'] for x in v if x['fault']) or '-'} |")
