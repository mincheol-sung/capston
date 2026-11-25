"""2.67° 어긋남의 두 후보 가르기 — 옆걸음 판에서 바퀴 ω_i 를 SLAM 중심 몸체 속도로 회귀.
 (a) 바퀴 굴러가는 방향(toe) 이 모두 +δ 반시계: v_실제 = s·R(δ)·(T ω)  →  c_y = [−24.08, +26.44, +26.44, −24.08] (FL·RR/FR·RL = 0.91)
 (b) 대각선 짝 유효 반지름 차 (c_x 회귀의 r_i 37.8/41.5/41.9/37.8 mm) → c_y = [−26.46, +24.10, +23.87, −26.46] (= 1.10)
 앞으로 갈 때 (c_x) 는 두 모델이 같은 값을 내서 못 가른다. 옆으로 갈 때 c_y 는 반대로 갈린다.
SLAM 중심 = /tracked_pose → compare_slam.imu_to_center, 몸체 속도 = ±0.1 s 중앙 차분을 R(−ψ) 로. 바퀴 ω = 같은 ±0.1 s 평균.
표본: |v_y| > 0.15, |v_x| < 0.08, |ω_z| < 0.15 (옆걸음 정속), 헛돎 표본 (어느 바퀴든 |ω_i| > 1.6·|v_y|/r) 뺌.
실행 (작업공간 루트): python3 <이 파일> <bag>...
"""
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))
r = 0.04
W = ["FL", "FR", "RL", "RR"]
print("bag | 표본 | c_y FL / FR / RL / RR [rad/s per m/s] | FL·RR ÷ FR·RL | c_x 같은 순서 (참고)")
allX, allY = [], []
for b in sys.argv[1:]:
    raw = read(b, ("/tracked_pose", "/wheel_states"))
    S = np.array([[t, m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for t, m in raw["/tracked_pose"]])
    xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off); th = np.unwrap(S[:, 3]); t = S[:, 0]
    Wv = np.array([[tt, *m.velocity] for tt, m in raw["/wheel_states"]])
    rows = []
    for i in range(len(t)):
        a, c = np.searchsorted(t, t[i] - 0.1), np.searchsorted(t, t[i] + 0.1) - 1
        if a < 0 or c >= len(t) or c <= a or t[c] - t[a] < 0.15:
            continue
        dt = t[c] - t[a]; vw = np.array([xc[c] - xc[a], yc[c] - yc[a]]) / dt; wz = (th[c] - th[a]) / dt
        ps = th[i]; vb = np.array([np.cos(ps) * vw[0] + np.sin(ps) * vw[1], -np.sin(ps) * vw[0] + np.cos(ps) * vw[1]])
        m = (Wv[:, 0] >= t[a]) & (Wv[:, 0] <= t[c])
        if m.sum() < 5:
            continue
        rows.append([vb[0], vb[1], wz, *Wv[m, 1:].mean(0)])
    A = np.array(rows)
    sel = (np.abs(A[:, 1]) > 0.15) & (np.abs(A[:, 0]) < 0.08) & (np.abs(A[:, 2]) < 0.15)
    sel &= np.all(np.abs(A[:, 3:]) <= 1.6 * np.abs(A[:, 1:2]) / r, axis=1)
    X, Y = A[sel, :3], A[sel, 3:]
    allX.append(X); allY.append(Y)
    C = np.linalg.lstsq(X, Y, rcond=None)[0]                       # (3 × 4): 행 = v_x, v_y, ω_z
    cy, cx = C[1], C[0]
    print(f"{b.split('/')[-1]} | {sel.sum()} | " + " / ".join(f"{x:+.2f}" for x in cy) + f" | {(abs(cy[0]) + abs(cy[3])) / (abs(cy[1]) + abs(cy[2])):.3f} | " + " / ".join(f"{x:+.1f}" for x in cx))
X, Y = np.vstack(allX), np.vstack(allY); C = np.linalg.lstsq(X, Y, rcond=None)[0]; cy = C[1]
print(f"합침 | {len(X)} | " + " / ".join(f"{x:+.2f}" for x in cy) + f" | {(abs(cy[0]) + abs(cy[3])) / (abs(cy[1]) + abs(cy[2])):.3f} |")
d, s = np.radians(2.67), 0.9887
ra = np.array([np.sin(d) - np.cos(d), np.sin(d) + np.cos(d), np.sin(d) + np.cos(d), np.sin(d) - np.cos(d)]) / (s * r)
rb = np.array([-1 / 0.0378, 1 / 0.0415, 1 / 0.0419, -1 / 0.0378])
print("예측 (a) 회전 | " + " / ".join(f"{x:+.2f}" for x in ra) + f" | {(abs(ra[0]) + abs(ra[3])) / (abs(ra[1]) + abs(ra[2])):.3f}")
print("예측 (b) 반지름 | " + " / ".join(f"{x:+.2f}" for x in rb) + f" | {(abs(rb[0]) + abs(rb[3])) / (abs(rb[1]) + abs(rb[2])):.3f}")
print("공칭 | −25 / +25 / +25 / −25 | 1.000")


# ── 2. SLAM 없이: 정속 옆걸음 중 바퀴 속도 비율 (|ω_FL|+|ω_RR|)/(|ω_FR|+|ω_RL|) — 100 Hz 표본 ──
# 정속 = 기준 속력 0.25~0.35 m/s, 기준 가속 |a| < 0.2. 바퀴 기구학으로 옆 성분이 주 (|v_y,w| > 0.15, |v_x,w| < 0.3|v_y,w|, |ω_z,w| < 0.2).
# 헛돎 뺌: 한 바퀴의 |ω| 가 나머지 세 바퀴 평균의 1.4 배 넘으면 그 표본 뺌.
print("\n[2] bag | 표본 (100 Hz) | 비율 중앙값 [사분위] | 왼쪽(+y) / 오른쪽(−y) 따로")
for b in sys.argv[1:]:
    raw = read(b, ("/wheel_states", "/dra_mppi/reference_trajectory", "/mpc/armed"))
    Rf = np.array([[tt, np.hypot(x.vx, x.vy)] for tt, m in raw["/dra_mppi/reference_trajectory"] if len(m.references) for x in [m.references[0]]])
    ar = np.gradient(Rf[:, 1], Rf[:, 0])
    Wv = np.array([[tt, *m.velocity] for tt, m in raw["/wheel_states"]])
    w = Wv[:, 1:]; vx = r / 4 * w.sum(1); vy = r / 4 * (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3]); wz = r / 4 / 0.1932 * (-w[:, 0] + w[:, 1] - w[:, 2] + w[:, 3])
    sp = np.interp(Wv[:, 0], Rf[:, 0], Rf[:, 1]); aa = np.interp(Wv[:, 0], Rf[:, 0], ar)
    a_ = np.abs(w); others = (a_.sum(1, keepdims=True) - a_) / 3
    m = (sp > 0.25) & (sp < 0.35) & (np.abs(aa) < 0.2) & (np.abs(vy) > 0.15) & (np.abs(vx) < 0.3 * np.abs(vy)) & (np.abs(wz) < 0.2) & np.all(a_ < 1.4 * others, 1)
    q = (a_[:, 0] + a_[:, 3]) / (a_[:, 1] + a_[:, 2])
    lo, md, hi = np.percentile(q[m], [25, 50, 75]) if m.any() else (np.nan,) * 3
    L, Rr = q[m & (vy > 0)], q[m & (vy < 0)]
    print(f"{b.split('/')[-1]} | {m.sum()} | {md:.3f} [{lo:.3f}~{hi:.3f}] | {np.median(L) if len(L) else float('nan'):.3f} ({len(L)}) / {np.median(Rr) if len(Rr) else float('nan'):.3f} ({len(Rr)})")
print("예측: (a) 굴러가는 방향 회전 0.911 · (b) 대각선 반지름 차 1.103 · 공칭 1.000")
