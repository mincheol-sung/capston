"""δ (SLAM 진행 방향 − 바퀴 기구학 진행 방향) 를 주행 중 온라인으로 추정했다면 — bag 재생.
표본: SLAM 중심 (/tracked_pose → imu_to_center, header stamp) ±0.1 s 중앙 차분 몸체 속도 vs 같은 창 바퀴 기구학 속도 (공칭 T, header stamp).
|v| > 0.3 m/s, |Δ| < 15° (SLAM 튐 뺌). 온라인 추정 = 시간 상수 τ 의 지수 가중 평균 (EWMA, 1차 저역 필터와 같음), 시작값 0.
출력: 표본 잡음 (MAD·1.4826), 전체 중앙값, 왼쪽(CCW)·오른쪽(CW) 선회 중앙값, τ 별 추정치가 최종값 ±0.5° 안에 드는 시각과 그 뒤 흔들림 범위.
실행 (작업공간 루트): python3 <이 파일> <bag>...
"""
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))
r, k = 0.04, 0.1932
Tinv = np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]) / r; T = np.linalg.pinv(Tinv)
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
TAUS = (2.0, 5.0, 20.0)
POOL = []
print("bag | 표본 (첫~끝 표본 s) | 표본 잡음 σ [°] | 전체 중앙값 | CCW / CW 중앙값 | " + " | ".join(f"τ {t:.0f} s: ±0.5° 도달 (첫 표본부터 s), 그 뒤 범위" for t in TAUS))
for b in sys.argv[1:]:
    raw = read(b, ("/tracked_pose", "/wheel_states", "/mpc/armed"))
    S = np.array([[st(m.header), m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for _, m in raw["/tracked_pose"]])
    S = S[np.argsort(S[:, 0])]
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off); th = np.unwrap(S[:, 3]); t = S[:, 0]
    Wv = np.array([[st(m.header), *m.velocity] for _, m in raw["/wheel_states"]]); Wv = Wv[np.argsort(Wv[:, 0])]
    rows = []
    for i in range(len(t)):
        a, c = np.searchsorted(t, t[i] - 0.1), np.searchsorted(t, t[i] + 0.1) - 1
        if a < 0 or c >= len(t) or c <= a or t[c] - t[a] < 0.15:
            continue
        dt = t[c] - t[a]; vw = np.array([xc[c] - xc[a], yc[c] - yc[a]]) / dt; wz = (th[c] - th[a]) / dt
        ps = th[i]; vb = np.array([np.cos(ps) * vw[0] + np.sin(ps) * vw[1], -np.sin(ps) * vw[0] + np.cos(ps) * vw[1]])
        m = (Wv[:, 0] >= t[a]) & (Wv[:, 0] <= t[c])
        if m.sum() < 5 or np.hypot(*vb) < 0.3:
            continue
        vwh = T @ Wv[m, 1:].mean(0)
        d = np.degrees(np.arctan2(vb[1], vb[0]) - np.arctan2(vwh[1], vwh[0])); d = (d + 180) % 360 - 180
        if abs(d) < 15:
            rows.append([t[i], d, wz, np.hypot(*vb)])
    A = np.array(rows); tt, dd, ww, vv = A[:, 0], A[:, 1], A[:, 2], A[:, 3]; POOL.append(A)
    sig = 1.4826 * np.median(np.abs(dd - np.median(dd)))
    ccw, cw = np.median(dd[ww > 0.3]) if (ww > 0.3).any() else np.nan, np.median(dd[ww < -0.3]) if (ww < -0.3).any() else np.nan
    cells = []
    for tau in TAUS:
        e, out = 0.0, []
        for j in range(len(tt)):
            h = tt[j] - tt[j - 1] if j else 0.1
            al = 1 - np.exp(-h / tau); e += al * (dd[j] - e); out.append(e)
        out = np.array(out); fin = np.median(dd)
        ok = np.abs(out - fin) <= 0.5
        # 처음으로 ±0.5° 안에 든 시각 (첫 표본부터 경과 시간)
        drive = tt - tt[0]
        j0 = np.argmax(ok) if ok.any() else None
        rng = (out[j0:].min(), out[j0:].max()) if j0 is not None else (np.nan, np.nan)
        cells.append(f"{drive[j0]:.1f} s, {rng[0]:+.2f}~{rng[1]:+.2f}" if j0 is not None else "안 듦")
    drive_total = tt[-1] - tt[0]
    print(f"{b.split('/')[-1]} | {len(dd)} ({drive_total:.0f} s) | {sig:.2f} | {np.median(dd):+.2f} | {ccw:+.2f} / {cw:+.2f} | " + " | ".join(cells))

# ── 선회 몫이 구심 가속 a_c = v·ω_z 에 비례하나 (모은 표본) ──
P = np.vstack(POOL); d, w, v = P[:, 1], P[:, 2], P[:, 3]; ac = v * w
for nm, X in (("δ0 만", np.c_[np.ones_like(d)]), ("δ0 + c·sign(ω_z)", np.c_[np.ones_like(d), np.sign(w)]), ("δ0 + c·a_c", np.c_[np.ones_like(d), ac])):
    cf, *_ = np.linalg.lstsq(X, d, rcond=None); res = d - X @ cf
    print(f"[모음 {len(d)}] {nm}: 계수 {np.round(cf, 3)}  잔차 σ (MAD) {1.4826*np.median(np.abs(res-np.median(res))):.2f}°")
edges = [-2.0, -1.0, -0.5, -0.2, 0.2, 0.5, 1.0, 2.0]
print("a_c 구간 [m/s²] | 표본 | Δ 중앙값 [°]")
for lo, hi in zip(edges[:-1], edges[1:]):
    m = (ac >= lo) & (ac < hi)
    if m.sum() >= 10:
        print(f"{lo:+.1f}~{hi:+.1f} | {m.sum()} | {np.median(d[m]):+.2f}")
