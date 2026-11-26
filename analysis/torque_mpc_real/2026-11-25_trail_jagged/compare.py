#!/usr/bin/python3
"""RViz 초록 선 (= /state_estimator/odom) 이 직진 구간에서 들쭉날쭉한 이유: 추정기 vs SLAM vs 바퀴·IMU (2026-11-25).
bag: 2026-11-25/164118_sq2p_mpcwd_r4 (16:41:18, polyline 사각형 두 개, MPC velocity_source wheels).
  - 경로 횡오차: 추정기 중심 / SLAM 중심 (/tracked_pose 는 IMU 점 → imu_offset (0.1093, 0.017) 만큼 R(ψ) 로 중심으로)
    직선 구간 (변 끝 0.3 m 안 뺌) 에서 횡오차 − 0.5 s 이동평균 = 들쭉날쭉한 몫
  - 보정 잔차 (/state_estimator/status): SLAM 이 올 때마다 예측과 다른 정도. 위치는 K_p 0.711 배만큼 바로 옮겨진다
  - 몸체 옆방향 속도 v_y: 추정기 vs 바퀴 기구학 (T ω) — 1~5 Hz 성분
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/compare.py [bag]"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(H, "../../.."))
BAG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "bags/2026-11-25/164118_sq2p_mpcwd_r4")
OFF = np.array([0.1093, 0.017])                  # torque_estimator.yaml imu_offset_x/y (포즈 기준점 = IMU)
R_W, K = 0.04, (0.1724 + 0.214) / 2              # torque_plant.yaml
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
T = ["/state_estimator/odom", "/tracked_pose", "/state_estimator/status", "/figure8/path", "/wheel_states"]
r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
E, S, ST, W, path = [], [], [], [], None
while r.has_next():
    n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
    if n == "/state_estimator/odom":
        p = m.pose.pose
        E.append((st(m.header), p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x, m.twist.twist.linear.y))
    elif n == "/tracked_pose":
        p = m.pose; ps = yaw_of(p.orientation); c, s = math.cos(ps), math.sin(ps)
        S.append((st(m.header), p.position.x - (c * OFF[0] - s * OFF[1]), p.position.y - (s * OFF[0] + c * OFF[1]), ps))
    elif n == "/state_estimator/status":
        ST.append(list(m.data))
    elif n == "/figure8/path" and path is None:
        path = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])
    elif n == "/wheel_states":
        W.append((st(m.header), *m.velocity))
E, S, ST, W = map(np.array, (E, S, ST, W))
print(f"bag {os.path.basename(BAG)}: 추정기 {len(E)} ({1 / np.median(np.diff(E[:, 0])):.0f} Hz), SLAM {len(S)} ({1 / np.median(np.diff(S[:, 0])):.1f} Hz), "
      f"보정 {len(ST)}, 기준 경로 점 {len(path)}")

# 기준 꺾은선을 꼭짓점으로 줄인다 (방향이 5° 넘게 바뀌는 점)
d = np.diff(path, axis=0); keep = [0]
for i in range(1, len(path) - 1):
    a, b = d[i - 1], d[i]
    if np.linalg.norm(a) > 1e-6 and np.linalg.norm(b) > 1e-6 and abs(math.atan2(a[0] * b[1] - a[1] * b[0], a @ b)) > math.radians(5):
        keep.append(i)
keep.append(len(path) - 1)
V = path[keep]
segs = [(V[i], V[i + 1]) for i in range(len(V) - 1) if np.linalg.norm(V[i + 1] - V[i]) > 0.5]
print(f"기준 직선 변 {len(segs)} 개, 길이 {[round(float(np.linalg.norm(b - a)), 2) for a, b in segs]} m")


def xtrack(P):
    """각 점의 가장 가까운 변에 대한 횡오차 (왼쪽 +) 와 그 변 위 위치가 끝에서 0.3 m 안인지."""
    out = np.full(len(P), np.nan); inner = np.zeros(len(P), bool); best = np.full(len(P), np.inf); seg = np.full(len(P), -1)
    for k, (a, b) in enumerate(segs):
        u = b - a; L = np.linalg.norm(u); u = u / L
        rel = P - a; s = rel @ u; e = u[0] * rel[:, 1] - u[1] * rel[:, 0]
        dist = np.where((s >= 0) & (s <= L), np.abs(e), np.inf)
        better = dist < best
        best[better], out[better], inner[better], seg[better] = dist[better], e[better], ((s > 0.3) & (s < L - 0.3))[better], k
    return out, inner, seg


sp = np.hypot(E[:, 4], E[:, 5])
mv = np.where(sp > 0.1)[0]; t0, t1 = E[mv[0], 0], E[mv[-1], 0]
we = (E[:, 0] >= t0) & (E[:, 0] <= t1); ws = (S[:, 0] >= t0) & (S[:, 0] <= t1)
ee, ie, ke = xtrack(E[:, 1:3]); es, is_, ks = xtrack(S[:, 1:3])


def hf(t, e, inner, seg, win=0.5):
    """같은 변 안에서 횡오차 − 이동평균 (±win/2)."""
    out = np.full(len(e), np.nan)
    for k in np.unique(seg[seg >= 0]):
        idx = np.where((seg == k) & np.isfinite(e))[0]
        for j in idx:
            if not inner[j]: continue
            m = idx[np.abs(t[idx] - t[j]) <= win / 2]
            out[j] = e[j] - e[m].mean()
    return out


# (기록된 /figure8/path 는 무장 전 한 번이라 실제 기준과 어긋난다 → 각 경로 자신의 0.5 s 이동평균에서 진행 방향의 옆으로 벗어난 몫)
def lateral_hf(t, P, win=0.5):
    out = np.full(len(t), np.nan)
    for j in range(len(t)):
        m = np.abs(t - t[j]) <= win / 2
        if m.sum() < 3: continue
        a, b = np.where(m)[0][[0, -1]]
        u = P[b] - P[a]; L = np.linalg.norm(u)
        if L < 0.05: continue                      # 거의 안 움직인 창은 뺀다
        u /= L; rel = P[j] - P[m].mean(0)
        out[j] = u[0] * rel[1] - u[1] * rel[0]
    return out
he = lateral_hf(E[we, 0], E[we, 1:3]); hs = lateral_hf(S[ws, 0], S[ws, 1:3])
q = lambda a: a[np.isfinite(a)]
print("\n## 1. 들쭉날쭉한 몫: 자기 경로 0.5 s 이동평균에서 진행 방향 옆으로 벗어난 거리 (움직인 구간)")
print("| | 표본 | RMS mm | 95 % mm | 최대 mm |\n|---|---|---|---|---|")
for lab, h in (("추정기 (초록 선)", he), ("SLAM (/tracked_pose → 중심)", hs)):
    print(f"| {lab} | {len(q(h))} | {1e3 * np.sqrt(np.mean(q(h) ** 2)):.1f} | {1e3 * np.percentile(np.abs(q(h)), 95):.1f} | {1e3 * np.abs(q(h)).max():.1f} |")

# 추정기 위치를 SLAM 시각에 보간해 차이
ex = np.interp(S[ws, 0], E[:, 0], E[:, 1]); eyy = np.interp(S[ws, 0], E[:, 0], E[:, 2])
dd = np.hypot(ex - S[ws, 1], eyy - S[ws, 2])
print(f"\n## 2. 같은 시각 추정기 − SLAM 거리: 중앙값 {1e3 * np.median(dd):.1f} mm, 95 % {1e3 * np.percentile(dd, 95):.1f} mm, 최대 {1e3 * dd.max():.1f} mm")

# 보정 잔차와 추정기 한 스텝 위치 변화
res = ST[(ST[:, 0] >= t0) & (ST[:, 0] <= t1)]
rn = np.hypot(res[:, 1], res[:, 2])
print(f"\n## 3. 보정 잔차 |e| (SLAM − 예측, IMU 점): 중앙값 {1e3 * np.median(rn):.1f} mm, 95 % {1e3 * np.percentile(rn, 95):.1f} mm, "
      f"최대 {1e3 * rn.max():.1f} mm  (위치는 0.711 배 = 중앙값 {0.711e3 * np.median(rn):.1f} mm 를 한 번에 옮김), 포즈 튐 {int(res[-1, 7] - res[0, 7])} 회")
yaw_u = np.interp(res[:, 0], E[:, 0], np.unwrap(E[:, 3])); spd_u = np.interp(res[:, 0], E[:, 0], sp)
bx = np.cos(yaw_u) * res[:, 1] + np.sin(yaw_u) * res[:, 2]; by = -np.sin(yaw_u) * res[:, 1] + np.cos(yaw_u) * res[:, 2]
print(f"   잔차를 몸체 앞 (x) · 옆 (y) 로: |x| 95 % {1e3 * np.percentile(np.abs(bx), 95):.1f} mm, |y| 95 % {1e3 * np.percentile(np.abs(by), 95):.1f} mm")
print("   | 추정 속도 | 보정 수 | |잔차 앞| 중앙/95 % mm | |잔차 옆| 중앙/95 % mm |\n   |---|---|---|---|")
for lo, hi in ((0.1, 0.5), (0.5, 1.0), (1.0, 1.3), (1.3, 2.0)):
    m = (spd_u >= lo) & (spd_u < hi)
    if m.sum() > 3:
        print(f"   | {lo}~{hi} m/s | {m.sum()} | {1e3 * np.median(np.abs(bx[m])):.1f} / {1e3 * np.percentile(np.abs(bx[m]), 95):.1f} | "
              f"{1e3 * np.median(np.abs(by[m])):.1f} / {1e3 * np.percentile(np.abs(by[m]), 95):.1f} |")
E2 = E[we]; tE = E2[:, 0]
step = np.hypot(np.diff(E2[:, 1]), np.diff(E2[:, 2])); vstep = sp[we][1:] * np.diff(tE)   # 속도로 예상한 한 스텝 이동
extra = step - vstep
near = np.array([np.min(np.abs(res[:, 0] - t)) < 0.006 for t in tE[1:]])
print(f"   추정기 한 스텝 이동 − 속도×Δt: 보정 직후 스텝 중앙값 {1e3 * np.median(np.abs(extra[near])):.2f} mm (95 % {1e3 * np.percentile(np.abs(extra[near]), 95):.2f}), "
      f"나머지 {1e3 * np.median(np.abs(extra[~near])):.3f} mm (95 % {1e3 * np.percentile(np.abs(extra[~near]), 95):.3f})")

# 몸체 옆 속도: 추정기 vs 바퀴
Wt = W[:, 0]; om = W[:, 1:5]
vy_w = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
vx_w = R_W / 4 * om.sum(1)
vy_e = np.interp(Wt, E[:, 0], E[:, 5]); vx_e = np.interp(Wt, E[:, 0], E[:, 4])
wm = (Wt >= t0) & (Wt <= t1)
fs = 1 / np.median(np.diff(Wt)); bb, aa = butter(2, [1 / (fs / 2), 5 / (fs / 2)], btype="band")
band = lambda x: filtfilt(bb, aa, x)
print(f"\n## 4. 몸체 옆방향 속도 v_y (바퀴 표본 {fs:.0f} Hz, 움직인 구간)")
print("| | 평균 mm/s | 1~5 Hz 성분 RMS mm/s |\n|---|---|---|")
for lab, v in (("추정기", vy_e), ("바퀴 기구학 T ω", vy_w)):
    print(f"| {lab} | {1e3 * v[wm].mean():+.1f} | {1e3 * np.sqrt(np.mean(band(v)[wm] ** 2)):.1f} |")
c = np.corrcoef(band(vy_e)[wm], band(vy_w)[wm])[0, 1]
print(f"1~5 Hz 성분 상관 (추정기 vs 바퀴): {c:+.2f}.  참고 v_x 1~5 Hz RMS 추정기 {1e3 * np.sqrt(np.mean(band(vx_e)[wm] ** 2)):.1f} / 바퀴 {1e3 * np.sqrt(np.mean(band(vx_w)[wm] ** 2)):.1f} mm/s")
np.savez_compressed(os.path.join(H, "series.npz"), E=E, S=S, ST=ST, W=W, path=path, t0=t0, t1=t1)
