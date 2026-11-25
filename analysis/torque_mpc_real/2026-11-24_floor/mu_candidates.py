# 단일 마찰계수 후보 → 바퀴별 바닥 토크 상한·직진 최대 가속, 그 가속에서 단계 토크 시험이 본 것 (floor_step.md 의 단계별 중앙값을 가속에 대해 보간)
#   /usr/bin/python3 mu_candidates.py
# 상한 식 (torque_mpc_node, limit_mode friction): 상한_i = 안전 계수 × μ_i × 하중_i × r/√2   [바닥 전달 토크]
# 직진 최대 가속: 옆 힘 0 · 회전 모멘트 0 이면 앞 두 바퀴 토크가 같고 뒤 두 바퀴 토크가 같다 → 합 = 2·min(FL, FR) + 2·min(RL, RR)
#   cap_on ground: 합/(m r)             (상한을 바닥 전달 토크에 건다)
#   cap_on total : 합/(m r + 4 J_w/r)   (상한을 모터 토크 − 구동계 마찰에 건다 — 바퀴 관성 몫이 상한 안에 들어간다)
import math
import numpy as np
r, m, g, J = 0.04, 4.56, 9.81, 8.9e-4
share = np.array([0.2388, 0.2392, 0.2608, 0.2612]); N = share * m * g
# floor_step.md 단계별 중앙값 (2026-11-24, 70 회): 몸체 가속 IMU, 끊을 때 바퀴/몸체 속도 FL FR RL RR, 헛돈 회 수/회 수 FL·FR, 도는 각 최대
A = np.array([0.78, 1.39, 1.88, 2.27, 2.65, 2.94])
RATIO = np.array([[1.00, 0.98, 1.01, 0.98], [1.07, 1.01, 1.00, 0.98], [1.14, 1.05, 1.06, 1.04], [1.31, 1.13, 1.05, 1.07], [1.29, 1.24, 1.06, 1.13], [1.24, 1.25, 1.15, 1.10]])
SPIN_FL = np.array([0, 3, 2, 7, 11, 7]) / np.array([15, 13, 13, 11, 11, 7])
SPIN_FR = np.array([0, 0, 0, 2, 9, 7]) / np.array([15, 13, 13, 11, 11, 7])
YAW = np.array([1.4, 6.2, 6.5, 22.4, 15.2, 20.8])


def caps(mu, cs):
    return cs * np.asarray(mu, float) * N * r / math.sqrt(2.0)


def amax(tb):
    s = 2.0 * min(tb[0], tb[1]) + 2.0 * min(tb[2], tb[3])
    return s / (m * r), s / (m * r + 4.0 * J / r)


def seen(a):
    if a > A[-1]:
        return f"시험 범위 밖 (최고 {A[-1]} 에서 앞 둘 다 헛돎)"
    f = lambda y: float(np.interp(a, A, y))
    return f"FL {100*(f(RATIO[:,0])-1):.0f} % · FR {100*(f(RATIO[:,1])-1):.0f} % 빠름, FL 헛돈 회 {100*f(SPIN_FL):.0f} % · FR {100*f(SPIN_FR):.0f} %, 도는 각 최대 {f(YAW):.0f}°"


rows = [("지금 설정 (바퀴별)", [0.598, 0.489, 0.547, 0.703], 0.8)]
for mu in (0.27, 0.33, 0.38, 0.42):
    for cs in (0.8, 1.0):
        rows.append((f"단일 {mu:.2f}", [mu] * 4, cs))
print("| 후보 | 안전 계수 | 바퀴별 바닥 토크 상한 mN·m FL FR RL RR | 직진 최대 가속 m/s²: 바닥 기준 | 그 가속에서 시험이 본 것 | 직진 최대 가속: 모터 기준 | 그 가속에서 시험이 본 것 |")
print("|---|---|---|---|---|---|---|")
for name, mu, cs in rows:
    tb = caps(mu, cs)
    ag, at = amax(tb)
    print(f"| {name} | {cs:.1f} | {' '.join(f'{x*1e3:.0f}' for x in tb)} | {ag:.2f} | {seen(ag)} | {at:.2f} | {seen(at)} |")
print()
print("| 후보 (안전 계수 0.8) | 최대 가속 (바닥 기준) | 주 시나리오 0.8~0.9 배 | 스트레스 1.5~2 배 | 최대 가속 (모터 기준) | 주 시나리오 | 스트레스 |")
print("|---|---|---|---|---|---|---|")
for name, mu, cs in rows:
    if cs != 0.8:
        continue
    ag, at = amax(caps(mu, cs))
    print(f"| {name} | {ag:.2f} | {0.8*ag:.2f}~{0.9*ag:.2f} | {1.5*ag:.2f}~{2*ag:.2f} | {at:.2f} | {0.8*at:.2f}~{0.9*at:.2f} | {1.5*at:.2f}~{2*at:.2f} |")
print()
print("검산: 지금 설정의 최대 가속은 리포 계산값 2.88 (바닥 기준) · 1.94 (모터 기준) 과 같아야 한다 →", [round(x, 2) for x in amax(caps([0.598, 0.489, 0.547, 0.703], 0.8))])
print("몸체 가속 → 정지 하중 기준 마찰계수 √2·a/g:", {a: round(math.sqrt(2) * a / g, 3) for a in A})
