# 2026-11-25 실차 4 판 (read_bags.py 의 npz) 분석:  /usr/bin/python3 analyze.py
#   ① 추종 오차  ② 슬립 λ  ③ 상한에 닿은 동안의 몸체 가속 — 구동 (바닥 토크가 바퀴 회전 방향) 과 제동 (반대) 따로
#      모델이 모르는 바퀴 회전 방향 저항 R 이 있으면  가속 = 4·(τ̄ ∓ R)/(r·m)  → 구동이 작고 제동이 크다. 차 = 8R/(r·m)
#   ④ 외란 추정  ⑤ 모터 온도
#   바닥 토크 (모델) = P (τ − f),  P = I − J_w Fᵀ M_eff⁻¹ F  (torque_mpc.hpp ground_map),  f = 상한 옮김 양 (/torque_mpc/cap)
import glob, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
r, k, m, Jw, Iz = 0.04, 0.1932, 4.56, 8.9e-4, 0.06                       # torque_plant.yaml
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
mw = 4 * Jw / r**2
P = np.eye(4) - Jw * F.T @ np.diag(1 / np.array([m + mw, m + mw, Iz + mw * k**2])) @ F
share = np.array([0.2388, 0.2392, 0.2608, 0.2612])
tbar = r / np.sqrt(2) * 0.33 * share * m * 9.81 * 0.8                     # 시작 로그 [0.0798 0.0799 0.0871 0.0872] 와 같아야 한다
a_cap = tbar.sum() / (r * m)
print(f"P 대각 {np.round(np.diag(P), 3)} (로그 0.691),  τ̄ {np.round(tbar * 1e3, 1)} mN·m,  네 바퀴가 다 상한일 때 몸체 가속 {a_cap:.3f} m/s²")
print(f"  모르는 저항 R 1 mN·m 당 구동·제동 가속 차 {8e-3 / (r * m):.3f} m/s²\n")

def interp(tq, A, cols):
    return np.column_stack([np.interp(tq, A[:, 0], A[:, c]) for c in cols])

rows = []
for f in sorted(glob.glob(os.path.join(HERE, "*.npz"))):
    lab = os.path.basename(f)[:-4]
    A = np.load(f)
    arm = A["arm"]; ta = arm[arm[:, 1] > 0.5, 0]; td = arm[arm[:, 1] < 0.5, 0]
    t_on = ta[0]; t_off = td[td > t_on][0] if (td > t_on).any() else A["st"][-1, 0]
    st, cap = A["st"], A["cap"]
    s = (st[:, 0] >= t_on) & (st[:, 0] <= t_off); S, C = st[s], cap[s][:, 1:]
    t = S[:, 0]; D = S[:, 1:]
    tau, fs, ad = D[:, 4:8], C[:, 2:6], D[:, 17:20] / 0.01
    g = (P @ (tau - fs).T).T                                               # 모델 바닥 토크
    om = interp(t, A["ws"], range(5, 9))
    od = A["od"]; vbx = np.interp(t, od[:, 0], od[:, 4]); vby = np.interp(t, od[:, 0], od[:, 5])
    ref = A["ref"]; vr = np.hypot(np.interp(t, ref[:, 0], ref[:, 3]), np.interp(t, ref[:, 0], ref[:, 4]))
    imu = A["imu"]; still = (imu[:, 0] > t_on - 1.5) & (imu[:, 0] < t_on - 0.2)
    ax0 = imu[still, 1].mean()
    k_s = 20                                                               # IMU 이동 평균 (약 50 ms)
    axs = np.convolve(imu[:, 1] - ax0, np.ones(k_s) / k_s, mode="same")
    ax = np.interp(t, imu[:, 0], axs)
    e = np.hypot(D[:, 1], D[:, 2]); mov = vr > 0.02
    at = np.abs(g) >= 0.97 * tbar                                          # 바퀴별 상한에 닿음
    drive = at & (np.sign(g) == np.sign(om)) & (np.abs(om) > 1.0)
    brake = at & (np.sign(g) != np.sign(om)) & (np.abs(om) > 1.0)
    sgn_v = np.sign(vbx)
    sel_d = drive.all(1); sel_b = brake.all(1)
    a_along = ax * sgn_v                                                   # 진행 방향 가속 (+ = 빨라짐)
    # 추정기 속도 기울기 (0.1 s 창 중앙 차분) — IMU 와 독립
    dv = np.gradient(np.convolve(vbx, np.ones(10) / 10, mode="same"), t) * sgn_v
    slp = A["slip"]; ms = (slp[:, 0] >= t_on) & (slp[:, 0] <= t_off)
    L = slp[ms][:, 2:6]; wn = slp[ms][:, 6:10]; movL = np.abs(wn).mean(1) > 2.0
    Lm = L[movL]
    ws = A["ws"]; mws = (ws[:, 0] >= t_on) & (ws[:, 0] <= t_off)
    temp = np.nanmean(ws[mws][:, 13:17], 0)
    nxy = np.hypot(ad[:, 0], ad[:, 1])
    rows.append(dict(lab=lab, T=t_off - t_on, rms=1e3 * np.sqrt(np.mean(e[mov] ** 2)), mx=1e3 * e.max(), end=1e3 * e[-50:].mean(),
                     vmax=np.abs(vbx).max(), vymax=np.abs(vby).max(), nd=sel_d.sum() * 0.01, nb=sel_b.sum() * 0.01,
                     ad_=a_along[sel_d].mean() if sel_d.any() else np.nan, ab_=-a_along[sel_b].mean() if sel_b.any() else np.nan,
                     dd_=dv[sel_d].mean() if sel_d.any() else np.nan, db_=-dv[sel_b].mean() if sel_b.any() else np.nan,
                     at_any=100 * at.any(1)[mov].mean(), l10=100 * np.nanmean(np.abs(Lm) > 0.1, 0), l25=100 * np.nanmean(np.abs(Lm) > 0.25, 0),
                     spin=100 * np.nanmean(Lm > 0.1, 0), drag=100 * np.nanmean(Lm < -0.1, 0), lmax=np.nanmax(np.abs(Lm), 0),
                     admax=nxy.max(), adcap=100 * (nxy >= 0.495).mean(), yawmax=np.abs(ad[:, 2]).max(), temp=temp,
                     fb=int((D[:, 11] > 0).sum()), msmax=D[:, 12].max(), taumax=1e3 * np.nanmax(np.abs(A["mot"][(A["mot"][:, 0] >= t_on) & (A["mot"][:, 0] <= t_off) & (A["mot"][:, 1] == 0), 2:6]))))
    # 구간별 (상한에 닿은 이어진 구간) 상세
    print(f"== {lab}: 무장 {t_off - t_on:.1f} s, 최고 속도 {np.abs(vbx).max():.2f} m/s (옆 최대 {np.abs(vby).max():.3f}), 온도 {np.round(temp, 1)} °C, IMU 0 점 {ax0:+.3f} m/s²")
    edges = np.flatnonzero(np.diff(np.r_[0, (sel_d | sel_b).astype(int), 0]))
    for i0, i1 in zip(edges[::2], edges[1::2]):
        if i1 - i0 < 5:
            continue
        kind = "구동" if sel_d[i0:i1].mean() > 0.5 else "제동"
        print(f"   {t[i0] - t_on:5.2f}~{t[i1 - 1] - t_on:5.2f} s {kind}  진행 속도 {vbx[i0]:+.2f}→{vbx[i1 - 1]:+.2f} m/s  "
              f"가속 크기 IMU {abs(a_along[i0:i1].mean()):.2f} · 추정기 {abs(dv[i0:i1].mean()):.2f} m/s²  "
              f"모델 바닥 토크 평균 {np.round(1e3 * np.abs(g[i0:i1]).mean(0), 1)}  모터 명령 평균 {np.round(1e3 * tau[i0:i1].mean(0), 1)}")

print("\n| 판 | 이동 중 오차 RMS mm | 최대 mm | 끝 오차 mm | 상한에 닿은 이동 시간 % | 네 바퀴 다 상한: 구동 s · 제동 s | 그때 가속 IMU 구동 · 제동 m/s² | 추정기 구동 · 제동 | 모터 명령 최대 mN·m |")
print("|---|---|---|---|---|---|---|---|---|")
for q in rows:
    print(f"| {q['lab']} | {q['rms']:.1f} | {q['mx']:.1f} | {q['end']:.1f} | {q['at_any']:.0f} | {q['nd']:.2f} · {q['nb']:.2f} | {q['ad_']:.2f} · {q['ab_']:.2f} | {q['dd_']:.2f} · {q['db_']:.2f} | {q['taumax']:.0f} |")
print(f"(모델: 네 바퀴가 다 상한이면 {a_cap:.2f} m/s². 모르는 저항 R 이 있으면 구동 {a_cap:.2f} − 0.055·R, 제동 {a_cap:.2f} + 0.055·R  [R 은 mN·m])")
print("\n| 판 | 슬립 |λ|>0.1 % FL FR RL RR | |λ|>0.25 % | 헛돎 (λ>0.1) % | 끌림 (λ<−0.1) % | |λ| 최대 | 외란 추정 앞·옆 최대 m/s² (상한 0.5 에 닿은 %) · 회전 최대 | QP fallback · 계산 최대 ms |")
print("|---|---|---|---|---|---|---|---|")
for q in rows:
    f1 = lambda v: " ".join(f"{x:.0f}" for x in v)
    print(f"| {q['lab']} | {f1(q['l10'])} | {f1(q['l25'])} | {f1(q['spin'])} | {f1(q['drag'])} | {' '.join(f'{x:.2f}' for x in q['lmax'])} | "
          f"{q['admax']:.2f} ({q['adcap']:.0f} %) · {q['yawmax']:.2f} | {q['fb']} · {q['msmax']:.1f} |")
