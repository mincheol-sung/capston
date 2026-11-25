"""선회 방향 따라 갈리는 진행 방향 차가 (i) SLAM–바퀴 시간차인지 (Δ_err = −ω_z·Δt, ω 비례) (ii) 선회 옆미끄럼인지 (a_c = v·ω_z 비례) 가르기.
바퀴 창을 τ ms 옮겨 (ω(t − τ), +τ = 바퀴를 늦춤) 다시 계산 → CCW/CW 갈림과 회귀 계수가 τ 에 따라 어떻게 바뀌나. P1 은 R 0.5 고정이라 ω = v/R, a_c = v²/R —
속도 0.3~0.9 로 변해 두 회귀변수를 가를 수 있다. 실행 (작업공간 루트): python3 <이 파일> <bag>...
"""
import importlib.util, sys, numpy as np, yaml
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
_spec = importlib.util.spec_from_file_location("cs", "src/amr_basic_stack/controller/mpc/tools/compare_slam.py"); cs = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(cs)
_est = yaml.safe_load(open("src/amr_basic_stack/controller/mpc/config/torque_estimator.yaml"))["estimator"]; off = (float(_est["imu_offset_x"]), float(_est["imu_offset_y"]))
r, k = 0.04, 0.1932
Tinv = np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]) / r; T = np.linalg.pinv(Tinv)
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
DATA = []
for b in sys.argv[1:]:
    raw = read(b, ("/tracked_pose", "/wheel_states"))
    S = np.array([[st(m.header), m.pose.position.x, m.pose.position.y, 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)] for _, m in raw["/tracked_pose"]]); S = S[np.argsort(S[:, 0])]
    Wv = np.array([[st(m.header), *m.velocity] for _, m in raw["/wheel_states"]]); Wv = Wv[np.argsort(Wv[:, 0])]
    DATA.append((S, Wv))


def samples(shift):
    out = []
    for S, Wv in DATA:
        xc, yc = cs.imu_to_center(S[:, 1], S[:, 2], S[:, 3], off); th = np.unwrap(S[:, 3]); t = S[:, 0]
        for i in range(len(t)):
            a, c = np.searchsorted(t, t[i] - 0.1), np.searchsorted(t, t[i] + 0.1) - 1
            if a < 0 or c >= len(t) or c <= a or t[c] - t[a] < 0.15:
                continue
            dt = t[c] - t[a]; vw = np.array([xc[c] - xc[a], yc[c] - yc[a]]) / dt; wz = (th[c] - th[a]) / dt
            ps = th[i]; vb = np.array([np.cos(ps) * vw[0] + np.sin(ps) * vw[1], -np.sin(ps) * vw[0] + np.cos(ps) * vw[1]])
            m = (Wv[:, 0] >= t[a] - shift) & (Wv[:, 0] <= t[c] - shift)
            if m.sum() < 5 or np.hypot(*vb) < 0.3:
                continue
            vwh = T @ Wv[m, 1:].mean(0)
            d = np.degrees(np.arctan2(vb[1], vb[0]) - np.arctan2(vwh[1], vwh[0])); d = (d + 180) % 360 - 180
            if abs(d) < 15:
                out.append([d, wz, np.hypot(*vb)])
    return np.array(out)


mad = lambda x: 1.4826 * np.median(np.abs(x - np.median(x)))
print("τ [ms] | 표본 | CCW / CW 중앙값 [°] | 갈림 CW−CCW | Δ = δ0 + c_ω ω: δ0, c_ω [°/(rad/s)] (σ) | Δ = δ0 + c_a a_c: δ0, c_a [°/(m/s²)] (σ) | 둘 다: δ0, c_ω, c_a (σ)")
for sh in (-0.08, -0.04, 0.0, 0.04, 0.08, 0.12):
    A = samples(sh); d, w, v = A[:, 0], A[:, 1], A[:, 2]; ac = v * w; one = np.ones_like(d)
    res = []
    for X in (np.c_[one, w], np.c_[one, ac], np.c_[one, w, ac]):
        cf, *_ = np.linalg.lstsq(X, d, rcond=None); res.append((cf, mad(d - X @ cf)))
    ccw, cw = np.median(d[w > 0.3]), np.median(d[w < -0.3])
    print(f"{1000*sh:+.0f} | {len(d)} | {ccw:+.2f} / {cw:+.2f} | {cw-ccw:+.2f} | {res[0][0][0]:+.2f}, {res[0][0][1]:+.2f} ({res[0][1]:.2f}) | "
          f"{res[1][0][0]:+.2f}, {res[1][0][1]:+.2f} ({res[1][1]:.2f}) | {res[2][0][0]:+.2f}, {res[2][0][1]:+.2f}, {res[2][0][2]:+.2f} ({res[2][1]:.2f})")
print("시간차 해석: c_ω = −Δt·57.3 → Δt = −c_ω/57.3 s (+ = SLAM 이 바퀴보다 늦음)")
