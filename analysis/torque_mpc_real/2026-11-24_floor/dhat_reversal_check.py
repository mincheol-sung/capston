# 식으로 계산한 외란 추정 (앞 성분) 을 확인 주행 기록과 견준다:  /usr/bin/python3 dhat_reversal_check.py
#   한 방향 직진으로 줄인 식 (바퀴 하나 기준, 토크 mN·m, 가속 m/s²):
#     k·가속       = 모터 토크 − 실제 마찰·(가는 방향 부호)          k = m_eff·r/4 = 67.8 mN·m per m/s²
#     k·예측 가속  = 모터 토크 − 모델 마찰·(가는 방향 부호)
#     잔차 = 가속 − 예측 가속 = −(실제 마찰 − 모델 마찰)·(방향 부호)/k
#     식-11:  외란 추정 ← 0.99·외란 추정 + 0.01·잔차      (0.01 s 마다)
#   "실제 마찰 − 모델 마찰" 은 다른 시험 (등속 직진 12 점, 5 rad/s) 의 6.1 mN·m 를 쓴다 — 확인 주행 기록에서 맞춘 값이 아니다.
import os
import numpy as np
A = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_run.npz"))
st, ws = A["st"], A["ws"]; t_on, t_off = float(A["t_on"]), float(A["t_off"])
Ts, alpha = 0.01, 0.01
k = (4.56 + 4 * 8.9e-4 / 0.04**2) * 0.04 / 4 * 1e3      # 67.8
S = st[(st[:, 0] >= t_on) & (st[:, 0] <= t_off)]; t = S[:, 0] - t_on
dx = S[:, 1 + 17] / Ts                                   # 기록된 외란 추정 앞 성분
om = np.interp(S[:, 0], ws[:, 0], ws[:, 5:9].mean(1))
df = 6.1
print(f"k = {k:.1f} mN·m per m/s²,  0.99^100 = {0.99**100:.3f} (1 s 뒤 남는 비율),  잔차 크기 = {df}/{k:.1f} = {df/k:.3f} m/s²")

def sim(t0, t1):
    i0, i1 = np.searchsorted(t, t0), np.searchsorted(t, t1)
    d = dx[i0]; out = []
    for i in range(i0, i1):
        out.append(d)
        d = (1 - alpha) * d + alpha * (-(df / k) * np.tanh(om[i] / 0.5))
    return np.arange(i0, i1), np.array(out)

for lab, t0, t1, marks in (("전진 (출발 3.4 s 의 기록값에서 시작)", 3.4, 6.0, (3.5, 4.0, 4.5, 5.0, 5.5)),
                           ("후진 (다시 출발 7.43 s 의 기록값에서 시작)", 7.43, 10.0, (7.5, 8.0, 8.5, 9.0, 9.5))):
    idx, d = sim(t0, t1)
    print(f"\n{lab}\n| 무장 뒤 s | 바퀴 속도 rad/s | 외란 추정 앞 성분: 식 | 기록 | 차 |\n|---|---|---|---|---|")
    for m in marks:
        j = np.searchsorted(t[idx], m); s = slice(max(j - 10, 0), j + 10)
        print(f"| {m:.1f} | {om[idx][s].mean():+.1f} | {d[s].mean():+.3f} | {dx[idx][s].mean():+.3f} | {d[s].mean()-dx[idx][s].mean():+.3f} |")
    print(f"  구간 전체 차 RMS {np.sqrt(np.mean((d - dx[idx])**2)):.3f} m/s² = {k*np.sqrt(np.mean((d - dx[idx])**2)):.1f} mN·m/바퀴")

print("\n방향을 바꾼 뒤 '틀려 있는 양' = k × (새 방향의 잔차 − 지금 외란 추정).  전진 값에 다 수렴한 채 뒤집히면 처음 2×6.1 = 12.2 mN·m:")
for sec in (0.0, 0.5, 1.0, 2.0, 3.0):
    print(f"  {sec:.1f} s 뒤 {12.2 * 0.99 ** (sec / Ts):.1f} mN·m", end="")
print()
