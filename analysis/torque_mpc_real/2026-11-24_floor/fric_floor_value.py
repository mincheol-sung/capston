# 마찰 모델에 더할 값 (네 바퀴 공통, 바퀴 회전 방향을 따르는 일정 토크) 후보별: 속도·온도에 따라 "모델 − 실제" 가 모자란가 넘치는가
#   /usr/bin/python3 fric_floor_value.py      단위 mN·m (네 바퀴 평균), 속도 rad/s.  자료: floor_cv.md 12 구간 (34~36 °C)
import numpy as np
# (바퀴 속도, 주행 저항 전진, 후진, 띄운 마찰 모델 = 설정값 11-16 측정 38~39 °C) — fric_floor_candidates.py 와 같은 표
runs = [(5.1, 28.8, 30.1, 23.1), (5.1, 28.8, 29.0, 23.05), (10.2, 35.0, 33.3, 28.8), (10.2, 34.5, 33.6, 28.9), (15.5, 38.9, 39.3, 34.8), (15.35, 37.8, 39.0, 34.65)]
R = np.array(runs); W = np.r_[R[:, 0], R[:, 0]]; Y = np.r_[R[:, 1], R[:, 2]]; L = np.r_[R[:, 3], R[:, 3]]
short = Y - L                                                   # 모델이 모자란 양 (12 구간)
c, b = np.linalg.lstsq(np.column_stack([np.ones_like(W), W]), short, rcond=None)[0]
tc, bv = 17.3, 1.128                                            # 설정 띄운 마찰의 네 바퀴 평균
print(f"모자란 양 12 구간: 평균 {short.mean():.2f}, 최소 {short.min():.1f}, 최대 {short.max():.1f}   직선 {c:.2f} {b:+.3f}·ω")
for ws in (5.1, 10.2, 15.4):
    s = np.abs(W - ws) < 0.3
    print(f"  {ws:4.1f} rad/s: 평균 {short[s].mean():.1f}  (구간별 {np.round(np.sort(short[s]), 1)})")
print(f"  잰 범위 밖 (직선 연장): 17.5 rad/s (0.7 m/s) {c + b*17.5:.1f},  25 rad/s (1.0 m/s) {c + b*25:.1f}")
# 온도: 띄운 마찰은 약 1 %/°C (floor_cv.md — 26~34 °C 직선과 38~39 °C 직선의 차에서). 주행 저항 전체에 같은 비율을 쓴다고 보면
#   35 °C 에서 잰 값 Y 가 T °C 에서는 Y·(1 − 0.01·(T − 35))
print("\n| 더하는 값 | 남는 차 = 모델 − 실제 (− 모자람, + 넘침): 5 · 10 · 15 rad/s (35 °C, 잰 값) | 12 구간 중 넘치는 구간 | 17.5 · 25 rad/s (직선 연장) | 39 °C 라면 5 · 15 · 25 rad/s | 30 °C 라면 5 · 15 · 25 |")
print("|---|---|---|---|---|---|")
def at(ws, T):                                                  # 실제 주행 저항 추정 (잰 범위는 평균, 밖은 직선 연장) × 온도
    return (tc + bv * ws + c + b * ws) * (1 - 0.01 * (T - 35))
for x in (0.0, 2.0, 2.7, 3.1, 4.0, 5.1):
    e = L + x - Y
    row = " · ".join(f"{e[np.abs(W-ws)<0.3].mean():+.1f}" for ws in (5.1, 10.2, 15.4))
    ext = " · ".join(f"{tc + bv*ws + x - at(ws, 35):+.1f}" for ws in (17.5, 25))
    hot = " · ".join(f"{tc + bv*ws + x - at(ws, 39):+.1f}" for ws in (5.1, 15.4, 25))
    cold = " · ".join(f"{tc + bv*ws + x - at(ws, 30):+.1f}" for ws in (5.1, 15.4, 25))
    print(f"| {x:.1f} | {row} | {int((e > 0).sum())} / 12 | {ext} | {hot} | {cold} |")
# 상한과 견줌: τ_μ = (r/√2)·μ̂·(m g/4)·c_s
r, N, mu, cs = 0.04, 4.56 * 9.81 / 4, 0.33, 0.8
cap, lim = r / np.sqrt(2) * mu * N * cs, r / np.sqrt(2) * mu * N
m_eq_r4 = (4.56 + 4 * 8.9e-4 / r**2) * r / 4
print(f"\n바닥 토크 상한 (μ̂ 0.33 × 안전 계수 0.8) {cap*1e3:.1f} mN·m,  안전 계수 없이 {lim*1e3:.1f} → 여유 {1e3*(lim-cap):.1f} mN·m")
for d in (1.0, 2.7, 5.1):
    print(f"  모델 마찰이 {d:.1f} mN·m 모자라면: 상한에서 가속 {d/1e3/m_eq_r4:.3f} m/s² 덜 냄 (상한 가속 {4*cap/r/4.56:.2f} 의 {100*d/1e3/m_eq_r4/(4*cap/r/4.56):.1f} %).   "
          f"{d:.1f} 넘치면: 바닥 토크가 상한을 {d:.1f} 넘음 = 여유 {1e3*(lim-cap):.1f} 의 {100*d/1e3/(lim-cap):.0f} %, 마찰계수로 {mu*cs + mu*d/1e3/lim:.3f} 까지 씀 (상한 {mu*cs:.3f})")
