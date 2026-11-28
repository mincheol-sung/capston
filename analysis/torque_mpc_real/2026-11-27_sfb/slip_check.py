"""slip_logger 검산: bag(npz) 의 /wheel_states·/state_estimator/odom 으로 λ 를 다시 계산해 /slip_logger/slip 과 비교.
python3 -I slip_check.py <npz>"""
import sys
import numpy as np
from scipy.signal import butter, filtfilt, lfilter

R, K, EPS = 0.04, (0.1724 + 0.214) / 2, 2.5          # torque_plant.yaml wheel_radius_m, (wheel_base + robot_width)/2, slip_logger omega_eps
TINV = np.array([[1, -1, -K], [1, 1, K], [1, 1, -K], [1, -1, K]]) / R


def lam(w, wn):
    den = np.maximum(np.maximum(np.abs(w), np.abs(wn)), EPS)
    return (w - wn) * np.where(wn >= 0, 1.0, -1.0) / den


A = np.load(sys.argv[1])
ws, od, S, arm = A["ws"], A["od"], A["slip"], A["arm"]
print(f"K = {K:.4f} m, 행: wheel_states {len(ws)}, odom {len(od)}, slip {len(S)}")

# 1) 노드 재현: 각 slip 행의 ω_nom 은 노드가 쓴 v̂ 로 만든 것 → ω_nom 으로 v̂ 를 되풀어 odom 과 맞는지
Tpinv = np.linalg.pinv(TINV)
v_from_node = S[:, 6:10] @ Tpinv.T                      # 노드가 쓴 v̂ (최소제곱 역; T⁻¹ 열 독립이라 정확)
# 같은 행의 바퀴 ω: slip 행의 header 시각 = wheel_states header. bag 수신 시각으로 가장 가까운 ws 행을 짝짓는다
i = np.clip(np.searchsorted(ws[:, 0], S[:, 0]) - 1, 0, len(ws) - 1)
w = ws[i, 5:9]
lam_re = lam(w, S[:, 6:10])
ok = np.isfinite(S[:, 2:6])
d = np.abs(lam_re - S[:, 2:6])[ok]
print(f"[1] 노드 ω_nom + 짝지은 바퀴 ω 로 λ 재계산: 차이 중앙값 {np.median(d):.2e}, 99 % {np.quantile(d, .99):.2e}, "
      f"1e-6 안 {np.mean(d < 1e-6) * 100:.1f} %  (짝짓기 어긋남 탓이 나머지)")

# 노드가 쓴 v̂ 가 odom 의 몇 ms 전 메시지였나 (bag 수신 시각 기준)
j = np.clip(np.searchsorted(od[:, 0], S[:, 0]) - 1, 0, len(od) - 1)
err0 = np.abs(v_from_node - od[j, 4:7]).max(1)
err1 = np.abs(v_from_node - od[np.maximum(j - 1, 0), 4:7]).max(1)
print(f"    노드 v̂ = 직전 odom 과 같음 {np.mean(err0 < 1e-6) * 100:.1f} %, 그 전 odom 과 같음 {np.mean(err1 < 1e-6) * 100:.1f} %")
age = S[:, 0] - od[j, 0]
print(f"    직전 odom 과 slip 수신 시각 차 중앙값 {np.median(age) * 1e3:.1f} ms, 95 % {np.quantile(age, .95) * 1e3:.1f} ms")

# 2) 창 (summary.py 와 같게): 무장 ~ 해제, 추정 속력 > 0.05
t = S[:, 0]
t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
t_off = td[td > t_on][0] if (td > t_on).any() else ws[-1, 0]
sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
win = (t >= t_on) & (t <= t_off) & (sp > 0.05)
L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
fs = 1 / np.median(np.diff(t))
bb, aa = butter(1, 10.0 / (fs / 2))
Lf = filtfilt(bb, aa, L, axis=0)
Lc = lfilter(bb, aa, L, axis=0)
print(f"[2] 표본 주기 {1e3 / fs:.2f} ms, 창 {win.sum()} 행 ({win.sum() / fs:.2f} s), NaN 바퀴-행 {(~ok[win]).sum()}")


def metric(X, name):
    mx = np.abs(X[win]).max(1)
    per = (np.abs(X[win]) > 0.5).mean(0) * 100
    print(f"    {name:28s} 네 바퀴 중 최대 >25 % {np.mean(mx > .25) * 100:5.1f} · >50 % {np.mean(mx > .5) * 100:5.1f}"
          f"   | 바퀴별 >50 % FL {per[0]:.1f} FR {per[1]:.1f} RL {per[2]:.1f} RR {per[3]:.1f}")


metric(L, "원신호")
metric(Lf, "10 Hz filtfilt (발표 지표)")
metric(Lc, "10 Hz 인과 1차 (실시간 꼴)")

# 3) 구동 방향 보정 3.1° 를 넣으면 (slip_logger 는 일부러 안 넣음)
for deg in (3.1,):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    Tr = TINV @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    wn = v_from_node @ Tr.T
    Lr = lam(w, wn); Lr = np.where(ok, Lr, 0.0)
    metric(filtfilt(bb, aa, Lr, axis=0), f"filtfilt + T⁻¹R(−{deg}°)")

# 4) 큰 슬립 한 순간 예시 (FL)
k = np.where(win)[0][np.argmax(np.abs(Lf[win, 0]))]
print(f"[4] 예시 t={t[k] - t_on:.2f} s (무장 뒤): v̂ = [{v_from_node[k, 0]:+.3f} m/s, {v_from_node[k, 1]:+.3f} m/s, {v_from_node[k, 2]:+.3f} rad/s]")
for n, nm in enumerate(("FL", "FR", "RL", "RR")):
    print(f"    {nm}: 실제 ω {w[k, n]:+7.2f}  몸체가 시키는 ω_nom {S[k, 6 + n]:+7.2f} rad/s  → λ 원신호 {S[k, 2 + n]:+.3f}, 10 Hz {Lf[k, n]:+.3f}")
