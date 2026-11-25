# 등속 직진 12 점 (floor_cv.md) 으로: 바닥에서 더 드는 마찰 중 "방향 따라 뒤집히는 몫" 과 "몸체 기준 일정한 몫", 마찰 모델 후보별 남는 차
#   /usr/bin/python3 fric_floor_candidates.py      단위 mN·m (네 바퀴 평균, 가는 방향 +), 속도 rad/s
import numpy as np
# (바퀴 속도, 주행 저항 전진, 주행 저항 후진, 띄운 마찰 모델) — floor_cv.md 표에서 판별로
runs = [(5.1, 28.8, 30.1, 23.1), (5.1, 28.8, 29.0, 23.05), (10.2, 35.0, 33.3, 28.8), (10.2, 34.5, 33.6, 28.9), (15.5, 38.9, 39.3, 34.8), (15.35, 37.8, 39.0, 34.65)]
R = np.array(runs); w, fwd, rev, lifted = R.T
flip, const = (fwd + rev) / 2, (fwd - rev) / 2       # 뒤집히는 몫 (방향 따라 부호가 바뀜) · 몸체 기준 일정한 몫 (앞쪽 +)
print("| 바퀴 속도 | 전진 | 후진 | 뒤집히는 몫 = (전진+후진)/2 | 일정한 몫 = (전진−후진)/2 | 띄운 마찰 모델 | 모델이 모자란 양 |")
print("|---|---|---|---|---|---|---|")
for ws in (5.1, 10.2, 15.4):
    s = np.abs(w - ws) < 0.3
    print(f"| {w[s].mean():.1f} | {fwd[s].mean():.1f} | {rev[s].mean():.1f} | {flip[s].mean():.1f} | {const[s].mean():+.1f} | {lifted[s].mean():.1f} | {(flip[s]-lifted[s]).mean():.1f} |")
print(f"일정한 몫: 판별 {np.round(const,2)}  평균 {const.mean():+.2f}, 최대 크기 {np.abs(const).max():.2f}")
# 마찰 모델 후보: 12 점 (전진·후진 따로) 에 대한 남는 차
W, Y, L = np.r_[w, w], np.r_[fwd, rev], np.r_[lifted, lifted]
s_ls = (L @ Y) / (L @ L)
A = np.column_stack([np.ones_like(W), W]); c, b = np.linalg.lstsq(A, Y - L, rcond=None)[0]
print(f"\n배율 하나 (최소제곱) {s_ls:.3f},  더하는 직선 (최소제곱) {c:+.2f} {b:+.3f}·ω")
print("| 후보 | 남는 차 5 · 10 · 15 rad/s (모델 − 잰 값) | 12 점 RMS |")
print("|---|---|---|")
for lab, pred in (("지금 (띄운 값 그대로, 배율 1.0)", L), (f"배율 {s_ls:.2f}", s_ls * L), ("배율 1.12 (15 rad/s 에 맞춤)", 1.12 * L), (f"띄운 값 + ({c:.1f} {b:+.2f}·ω)", L + c + b * W)):
    e = pred - Y
    row = " · ".join(f"{e[np.abs(W-ws)<0.3].mean():+.1f}" for ws in (5.1, 10.2, 15.4))
    print(f"| {lab} | {row} | {np.sqrt(np.mean(e**2)):.1f} |")
# 확인 주행 (0.2 m/s 왕복): 등속에서 실제 명령 전진 29.0 · 후진 31.3, 띄운 모델 22.9 (dhat_check.py)
cf, cr, cm = 29.0, 31.3, 22.9
print(f"\n확인 주행: 모델이 모자란 양 전진 {cf-cm:.1f} · 후진 {cr-cm:.1f} → 뒤집히는 몫 {(cf+cr)/2-cm:.1f}, 일정한 몫 {(cf-cr)/2:+.1f}")
print(f"  방향을 바꾼 순간 외란 추정이 틀려 있는 양 (바퀴당): 지금 = 전진 때 메우던 {cf-cm:.1f} + 후진에서 필요한 {cr-cm:.1f} = {cf-cm+cr-cm:.1f}")
for lab, add in ((f"띄운 값 + ({c:.1f} {b:+.2f}·ω)", c + b * 5.1), (f"배율 {s_ls:.2f}", (s_ls - 1) * cm), ("배율 1.12", 0.12 * cm)):
    print(f"  {lab}: 모델 {cm+add:.1f} → 모자란 양 전진 {cf-cm-add:+.1f} · 후진 {cr-cm-add:+.1f} → 방향 전환 때 틀린 양 {abs(cf-cm-add+cr-cm-add):.1f}")
