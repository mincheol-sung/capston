#!/usr/bin/python3
"""2026-11-21 실차 정사각형 조건별 비교 — P1 (PP 룩어헤드 0.30) · P2 (PP 룩어헤드 0.15~0.60) · C1 (C, Q11=Q22=5) · C2 (C, Q11=Q22=20).
판: run_all.log 의 "RESULT <라벨> try<n> rc=0 <bag>" + 앞선 판 (184008 = P1, 183827 = C1, 190553 = C2).
지표는 analyze.py 의 run() (contour·lag·yaw·Δτ·λ·변 최고 속도, 같은 잣대) 을 그대로 쓴다 — analyze.py 의 함수 정의 부분만 실행한다.
추가: SLAM 튐 (연속 /tracked_pose 이동이 추정기 이동과 3 cm 넘게 다름, 주행 −0.5~9.5 s) 횟수·최대, 추정기 − SLAM 위치 RMS,
      측위 맞추기 배너 (맞음 / 실패), C 는 옆 feedback 회귀 k_p·k_d (decomp.py 와 같은 식).
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/compare_all.py"""
import glob, json, math, os, re
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "analyze.py")).read()
g = {"__file__": os.path.join(HERE, "analyze.py"), "__name__": "analyze_head"}
exec(src[:src.index("R = {b: run(b, tb)")], g)          # 상수·contour·project·read_topic·run 만
run, cs, w, WS, plt = g["run"], g["cs"], g["w"], g["WS"], g["plt"]
TB = np.array([0.1230, 0.1253, 0.1298, 0.1187])
r_w, k_geo, m_eff = 0.04, 0.1932, 6.785
FM = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k_geo, k_geo, -k_geo, k_geo]]) / r_w
COND = {"P1": ("PP 룩어헤드 0.30", "#1baf7a", None), "P2": ("PP 룩어헤드 0.15~0.60", "#0f6e8c", None),
        "C1": ("C Q11=Q22=5", "#eb6834", TB), "C2": ("C Q11=Q22=20", "#6250d6", TB), "C3": ("C Q11=Q22=20 Q33=10", "#c43b8a", TB)}

runs = [("P1", "2026-11-21/184008", "P1_0"), ("C1", "2026-11-21/183827", "C1_0"), ("C2", "2026-11-21/190553", "C2_0")]
logf = os.path.join(HERE, "run_all.log")
if os.path.exists(logf):
    for line in open(logf):
        m = re.match(r"RESULT (\w+) try\d rc=0 (\S+)", line)
        if m:
            lab, bag = m.group(1), m.group(2)
            runs.append((lab[:2], os.path.relpath(bag, f"{WS}/bags"), lab))


def extra(bag, tb, T0_rel):
    path = f"{WS}/bags/{bag}"
    S, E, P, A = w.load(path); D = cs.load(path)
    Pp = np.array(D["pose"]); xc, yc = cs.imu_to_center(Pp[:, 1], Pp[:, 2], Pp[:, 3], w.off)
    ex, ey = np.interp(Pp[:, 0], E[:, 0], E[:, 1]), np.interp(Pp[:, 0], E[:, 0], E[:, 2])
    jump = np.linalg.norm(np.c_[np.diff(xc), np.diff(yc)] - np.c_[np.diff(ex), np.diff(ey)], axis=1)
    rt = np.array([a for a, _ in D["ref"]])
    # 주행 시작 = 무장 뒤 기준 속력 > 0.02 첫 SLAM 표본 (analyze.run 과 같은 정의)
    tP = Pp[:, 0]; arm = cs.armed_mask(tP, D["armed"])
    sp = []
    for x in tP:
        i = np.searchsorted(rt, x, side="right") - 1
        sp.append(np.hypot(*D["ref"][i][1].at(x)[1][:2]) if i >= 0 else 0.0)
    mv = arm & (np.array(sp) > 0.02); T0 = tP[np.flatnonzero(mv)[0]]
    tj = tP[1:] - T0; jm = (jump > 0.03) & (tj > -0.5) & (tj < 9.5)
    win = (tP >= T0) & (tP <= T0 + 9.5)
    es = 1e3 * float(np.sqrt(np.mean((ex[win] - xc[win]) ** 2 + (ey[win] - yc[win]) ** 2)))
    out = dict(jumps=int(jm.sum()), jump_max=1e3 * float(jump[jm].max()) if jm.any() else 0.0, est_slam=es)
    banner = ""
    for f in glob.glob(f"{path}/run_info/ros_log/*/launch.log"):
        txt = open(f, errors="ignore").read()
        banner = "맞음" if "측위 맞추기 끝: 맞음" in txt else ("실패" if "측위 맞추기 실패" in txt else banner)
    out["loc"] = banner
    if tb is not None and len(S):
        t = S[:, 0]; Xe = S[:, 1:4]; tau = S[:, 4:8]; us = S[:, 22:26]; hold = S[:, 20] > 0.5
        runm = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
        th = np.interp(t, E[:, 0], np.unwrap(E[:, 3])); tv, vr, om = [], [], []
        for x in t:
            R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
            c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]]); n = np.linalg.norm(vw)
            tv.append(vw / max(n, 1e-9)); vr.append(n); om.append(ur[2])
        tv = np.array(tv); nv = np.c_[-tv[:, 1], tv[:, 0]]; vr = np.array(vr); om = np.array(om)
        e_n = -(Xe[:, :2] * nv).sum(1)
        dF = (FM @ (tau - us).T).T; c, s = np.cos(th), np.sin(th)
        a_n = ((np.c_[c * dF[:, 0] - s * dF[:, 1], s * dF[:, 0] + c * dF[:, 1]]) * nv).sum(1) / m_eff
        sat = ((np.abs(tau) / tb).max(1) >= 0.98)
        sel = runm & ~sat & (np.abs(om) < 0.3) & (vr > 0.3)
        h = 5; ed = np.full(len(e_n), np.nan); ed[h:-h] = (e_n[2 * h:] - e_n[:-2 * h]) / (t[2 * h:] - t[:-2 * h])
        ok = sel & np.isfinite(ed)
        X = np.c_[-e_n[ok], -ed[ok]]; cf, *_ = np.linalg.lstsq(X, a_n[ok], rcond=None)
        r2 = 1 - np.sum((a_n[ok] - X @ cf) ** 2) / np.sum((a_n[ok] - a_n[ok].mean()) ** 2)
        out.update(kp=float(cf[0]), kd=float(cf[1]), r2=float(r2), sat=100 * float(sat[runm & (vr > 0.02)].mean()))
    return out


rows = []; TRAJ = {}
for cond, bag, lab in runs:
    tb = COND[cond][2]
    try:
        r = run(bag, tb)
    except Exception as e:                                       # 기준열이 없는 판 등
        print(f"건너뜀 {lab} {bag}: {e}"); continue
    c2 = r["cs"]; L = r["lag"][np.isfinite(r["lag"])]
    row = dict(cond=cond, lab=lab, bag=bag, contour=1e3 * float(np.sqrt(np.mean(r["contour"][r["mv"]] ** 2))),
               contour_max=1e3 * float(r["contour"][r["mv"]].max()), lag=1e3 * float(L.mean()), yaw=float(c2["yaw"]), dtau=float(c2["dtau"]),
               tau_max=float(c2["tau_max"]), slip_xy=[float(c2["slip_x"]), float(c2["slip_y"])], big=r["lam"]["big_fast"], lrms=r["lam"]["rms_fast"],
               spin=r["lam"]["spin_fast"], peak=[p for p, _ in r["side_peak"]], track=1e3 * float(c2["rms"]))
    row.update(extra(bag, tb, None))
    rows.append(row); TRAJ[lab] = (r["lx"][r["win"]], r["ly"][r["win"]])
json.dump(rows, open(os.path.join(HERE, "compare_all.json"), "w"), ensure_ascii=False, indent=1)

print("| 판 | 조건 | 측위 | SLAM 튐 (횟수/최대 mm) | 추정기−SLAM RMS | contour RMS / 최대 | lag 평균 | 시간 추종 | yaw RMS | 큰 슬립 % (v_ref>0.3) | 헛돎 몫 | λ RMS | 슬립 크기 x·y | Δτ RMS | τ max | 변 최고 속도 | 옆 k_p / k_d (R²) |")
for x in rows:
    kk = f"{x['kp']:.2f} / {x['kd']:.2f} ({x['r2']:.2f})" if "kp" in x else "–"
    print(f"| {x['lab']} {x['bag'][-6:]} | {COND[x['cond']][0]} | {x['loc']} | {x['jumps']} / {x['jump_max']:.0f} | {x['est_slam']:.0f} | {x['contour']:.0f} / {x['contour_max']:.0f} | "
          f"{x['lag']:+.0f} | {x['track']:.0f} | {x['yaw']:.1f} | {x['big']:.1f} | {x['spin']:.0f} | {x['lrms']:.3f} | {x['slip_xy'][0]:.3f}·{x['slip_xy'][1]:.3f} | "
          f"{x['dtau']:.4f} | {x['tau_max']:.2f} | {min(x['peak']):.2f}~{max(x['peak']):.2f} | {kk} |")
print("\n조건별 평균 [최소~최대] (판 수)")
for cond in COND:
    xs = [x for x in rows if x["cond"] == cond]
    if not xs:
        continue
    f = lambda k: f"{np.mean([x[k] for x in xs]):.1f} [{min(x[k] for x in xs):.1f}~{max(x[k] for x in xs):.1f}]"
    print(f"  {cond} {COND[cond][0]} ({len(xs)}): contour {f('contour')} | lag {f('lag')} | yaw {f('yaw')} | 큰 슬립 {f('big')} | Δτ×1000 "
          f"{np.mean([1e3*x['dtau'] for x in xs]):.1f} | SLAM 튐 {f('jumps')} | 추정기−SLAM {f('est_slam')}")

# 그림: 조건별 점 (판 하나 = 점 하나), 지표마다 패널
mets = [("contour RMS [mm]", "contour"), ("lag 평균 [mm] (− 뒤처짐)", "lag"), ("yaw 오차 RMS [°]", "yaw"),
        ("큰 슬립 시간 [%]\nmax|λ|>0.5, v_ref>0.3", "big"), ("토크 떨림 Δτ RMS [N·m]", "dtau"), ("SLAM 튐 횟수\n(> 3 cm, 주행 중)", "jumps")]
conds = [c for c in COND if any(x["cond"] == c for x in rows)]
fig, axs = plt.subplots(1, len(mets), figsize=(20, 4.6))
for ax, (lab, key) in zip(axs, mets):
    for i, cond in enumerate(conds):
        xs = [x[key] for x in rows if x["cond"] == cond]
        jit = np.linspace(-0.12, 0.12, len(xs)) if len(xs) > 1 else [0.0]
        ax.scatter(i + np.array(jit), xs, s=46, color=COND[cond][1], edgecolor="#fcfcfb", linewidth=1.5, zorder=3)
        ax.hlines(np.mean(xs), i - 0.25, i + 0.25, color=COND[cond][1], lw=2.2, zorder=2)
    SHORT = {"P1": "PP\nL 0.30", "P2": "PP\nL 비례", "C1": "C\nQ11=Q22\n=5", "C2": "C\nQ11=Q22\n=20", "C3": "C\nQ=20,20\nQ33=10"}
    ax.set_xticks(range(len(conds))); ax.set_xticklabels([f"{c}\n{SHORT[c]}" for c in conds], fontsize=7.5)
    ax.set_title(lab, loc="left", color="#0b0b0b", fontsize=9.5); ax.grid(axis="x", visible=False); ax.set_xlim(-0.6, len(conds) - 0.4)
    if key == "lag":
        ax.axhline(0, color="#52514e", lw=0.8)
fig.suptitle("2026-11-21 실차 정사각형 (accel 1.5 · v_max 1.5 · tangent) — 판 하나 = 점 하나, 가로선 = 조건 평균", x=0.01, ha="left", color="#0b0b0b", fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.92)); fig.savefig(os.path.join(HERE, "fig_compare_all.png"), dpi=130); plt.close(fig)

# 그림: 조건별 궤적 겹쳐 그리기
fig, axs = plt.subplots(1, len(conds), figsize=(4.3 * len(conds), 4.6))
for ax, cond in zip(np.atleast_1d(axs), conds):
    ax.plot([0, 1.5, 1.5, 0, 0], [0, 0, 1.5, 1.5, 0], color="#a9a8a2", lw=6, alpha=0.45, solid_capstyle="round")
    xs = [x for x in rows if x["cond"] == cond]
    for j, x in enumerate(xs):
        lx, ly = TRAJ[x["lab"]]
        ax.plot(lx, ly, color=COND[cond][1], lw=1.6, alpha=1.0 - 0.25 * j, label=f"{x['lab']} {x['bag'][-6:]}: {x['contour']:.0f} mm, 튐 {x['jumps']}")
    ax.set_aspect("equal"); ax.set_xlim(-0.45, 2.0); ax.set_ylim(-0.45, 2.0)
    ax.set_title(f"{cond} {COND[cond][0]}", loc="left", fontsize=10, color="#0b0b0b"); ax.legend(loc="center", frameon=False, fontsize=8)
fig.suptitle("조건별 궤적 (SLAM 중심, 출발 꼭짓점 기준) — 범례: contour RMS, SLAM 튐 횟수", x=0.01, ha="left", fontsize=11, color="#0b0b0b")
fig.tight_layout(rect=(0, 0, 1, 0.92)); fig.savefig(os.path.join(HERE, "fig_compare_traj.png"), dpi=130); plt.close(fig)
