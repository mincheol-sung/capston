# 2026-11-26 구동 방향 보정값 다시 맞춤 — 바퀴 엔코더 기구학 이동 vs SLAM 이동 (제어기와 무관한 측정, UMBmark 꼴 기구학 보정, Borenstein & Feng 1996)
#   0.4 s 창 (0.1 s 씩 밀며) 마다: SLAM 중심 변위 Δp_SLAM (관성 좌표)  vs  바퀴 기구학 몸체 속도 (T ω, 보정 없는 원값) 를 SLAM 헤딩으로 돌려 적분한 Δp_바퀴
#   δ = ∠Δp_SLAM − ∠Δp_바퀴  (+ = 실제 이동이 바퀴 계산보다 반시계 = 왼쪽).  창 평균 구심 가속 a_c = |v|·ω_z (v 바퀴 속력, ω_z 추정기 = 자이로), 앞 가속 a_x
#   맞춤: δ = δ0 + c·a_c  (지금 설정 δ0 2.4°, c −2.5 °/(m/s²), 11-23 원운동 R 0.3·0.5·0.75 에서 잰 값)  — 국면별 (직선 순항·가속·감속·선회) 로도
#   판: 오늘 같은 꺾은선 12 판 (rr 6 · dy 6) — 엔코더 원값은 보정 켬/끔과 무관하게 같은 측정이라 다 쓴다
import os, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
RA = os.path.join(HERE, "..", "2026-11-25_roller_ab")
EXC = {"180604_dy_off_r1", "180802_dy_off_r1", "181021_dy_off_r1"}
r, k = 0.04, 0.1932
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
rows = []
runs = [(RA, os.path.basename(f)[:-4]) for f in glob.glob(os.path.join(RA, "*_rr_*_r*.npz"))] + \
       [(HERE, os.path.basename(f)[:-4]) for f in glob.glob(os.path.join(HERE, "*_dy_*_r*.npz"))]
runs = [x for x in sorted(runs, key=lambda x: x[1]) if x[1] not in EXC]
for folder, lab in runs:
    A = np.load(os.path.join(folder, lab + ".npz")); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]
    td = arm[arm[:, 1] < 0.5, 0]; t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    z = np.load(os.path.join(folder, "slam_cache", lab + ".npz"), allow_pickle=True); T = z["T"].copy(); T[:, 0] -= float(z["t_arm"]) - t_on
    ws, od = A["ws"], A["od"]
    om = ws[:, 5:9]
    vb = np.column_stack([r / 4 * om.sum(1), r / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3]), r / (4 * k) * (-om[:, 0] + om[:, 1] - om[:, 2] + om[:, 3])])
    tw = ws[:, 0]
    yaw_s = np.interp(tw, T[:, 0], np.unwrap(T[:, 3]))
    vx_w = np.cos(yaw_s) * vb[:, 0] - np.sin(yaw_s) * vb[:, 1]; vy_w = np.sin(yaw_s) * vb[:, 0] + np.cos(yaw_s) * vb[:, 1]
    dt = np.diff(tw, prepend=tw[0])
    Xw, Yw = np.cumsum(vx_w * dt), np.cumsum(vy_w * dt)
    wz = np.interp(tw, od[:, 0], od[:, 6]); sp = np.hypot(vb[:, 0], vb[:, 1])
    ax = np.gradient(np.convolve(sp, np.ones(10) / 10, mode="same"), tw)
    for t0 in np.arange(t_on + 0.5, t_off - 0.5, 0.1):
        t1 = t0 + 0.4
        ps = np.array([np.interp(t, T[:, 0], T[:, c]) for t in (t0, t1) for c in (1, 2)]).reshape(2, 2)
        pw = np.array([[np.interp(t, tw, Xw), np.interp(t, tw, Yw)] for t in (t0, t1)])
        ds, dw = ps[1] - ps[0], pw[1] - pw[0]
        if np.linalg.norm(dw) < 0.12:                       # 0.3 m/s 아래는 뺀다 (SLAM 잡음이 각을 키운다)
            continue
        m = (tw >= t0) & (tw <= t1)
        rows.append([np.degrees(wrap(np.arctan2(ds[1], ds[0]) - np.arctan2(dw[1], dw[0]))), np.mean(sp[m] * wz[m]), np.mean(ax[m]),
                     np.mean(np.abs(wz[m])), np.linalg.norm(ds) / np.linalg.norm(dw), np.mean(sp[m])])
R = np.array(rows)
d, ac, axx, w, sc, v = R.T
ok = np.abs(d) < 15
print(f"창 {len(R)} 개 (|δ| < 15° {ok.sum()}), 판 {len(runs)}")
def fit(sel, cols, name):
    Aa = np.column_stack([np.ones(sel.sum())] + [c[sel] for c in cols])
    x, *_ = np.linalg.lstsq(Aa, d[sel], rcond=None); res = d[sel] - Aa @ x
    se = np.sqrt(np.diag(np.sum(res ** 2) / max(1, len(res) - len(x)) * np.linalg.inv(Aa.T @ Aa)))
    print(f"  {name}: " + ", ".join(f"{n} {xx:+.2f} ± {s:.2f}" for n, xx, s in zip(["δ0 °", "c °/(m/s²)", "a_x 계수 °/(m/s²)"], x, se))
          + f"   잔차 RMS {np.sqrt(np.mean(res ** 2)):.2f}°  표본 {sel.sum()}")
    return x
st = ok & (w < 0.3)
print(f"직선 (|ω_z| < 0.3): δ 중앙 {np.median(d[st]):+.2f}°  평균 {d[st].mean():+.2f} ± {d[st].std() / np.sqrt(st.sum()):.2f}°  (표본 {st.sum()})  거리 비 SLAM/바퀴 {np.median(sc[st]):.3f}")
for nm, sel in (("  직선 가속 (a_x > 0.3)", st & (axx > 0.3)), ("  직선 감속 (a_x < −0.3)", st & (axx < -0.3)), ("  직선 순항", st & (np.abs(axx) <= 0.3))):
    print(f"{nm}: δ 중앙 {np.median(d[sel]):+.2f}°  (표본 {sel.sum()})")
tr = ok & (w >= 0.3)
print(f"선회 (|ω_z| ≥ 0.3): 왼쪽 (a_c > 0) δ 중앙 {np.median(d[tr & (ac > 0)]):+.2f}° (a_c 중앙 {np.median(ac[tr & (ac > 0)]):+.2f}), "
      f"오른쪽 δ 중앙 {np.median(d[tr & (ac < 0)]):+.2f}° (a_c 중앙 {np.median(ac[tr & (ac < 0)]):+.2f})  표본 {tr.sum()}")
print("맞춤 δ = δ0 + c·a_c (+ a_x):")
fit(ok, [ac], "전부")
fit(ok, [ac, axx], "전부 + 앞 가속")
fit(ok & (np.abs(ac) <= 2.0), [ac], "|a_c| ≤ 2 (지금 설정 slip_ac_max)")
print(f"지금 설정으로 본 잔차: 직선 {np.median(d[st] - 2.4):+.2f}°, 왼쪽 선회 {np.median(d[tr & (ac > 0)] - (2.4 - 2.5 * np.clip(ac[tr & (ac > 0)], -2, 2))):+.2f}°, "
      f"오른쪽 선회 {np.median(d[tr & (ac < 0)] - (2.4 - 2.5 * np.clip(ac[tr & (ac < 0)], -2, 2))):+.2f}°")
np.save(os.path.join(HERE, "kin_calib_rows.npy"), R)
