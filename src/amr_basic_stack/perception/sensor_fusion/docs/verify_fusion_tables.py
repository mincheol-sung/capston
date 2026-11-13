#!/usr/bin/env python3
"""SEMINAR_LATE_FUSION.md 의 표를 노드 기본 파라미터에서 재계산한다.

문서에 숫자를 손으로 적어 두면 파라미터가 바뀔 때 조용히 거짓말이 된다.
이 스크립트는 camera_lidar_fusion_node 의 모델 클래스를 그대로 import 하므로,
계수를 고치면 출력이 같이 바뀐다. 발표 직전에 한 번 돌려 문서와 대조할 것.

실행:
  source /opt/ros/jazzy/setup.bash
  source ~/2026amr_ws/install/setup.bash
  python3 verify_fusion_tables.py

C2 실측값(bias_CL_r, 인용 chi^2)만 EXPERIMENT_C2_RESULTS.txt 에서 옮겨 온
상수다. 그 외에는 전부 계산 결과다.
"""
import math

import numpy as np

from sensor_fusion.camera_lidar_fusion_node import (
    PolarNoiseModel, RepresentativeOffset)

# 노드 기본 파라미터와 같은 값. 바뀌면 여기도 같이 바꿔야 한다.
CAM = PolarNoiseModel(0.321e-3, 2.741, 0.560e-3, 1.450)
LID = PolarNoiseModel(0.331e-3, 0.354, 6.5e-3, 0.0,
                      r_floor=1.6e-3, r_break=4.5)
OFF = RepresentativeOffset(-10.98e-3, 1.281, 13.0e-3)
Q_R, Q_T = 5.08e-3, 6.95e-3
GATE = 9.21

# C2 실측 (EXPERIMENT_C2_RESULTS.txt 5절). 계산값이 아니라 측정값이다.
BIAS_CL_R = {1.10: -13.5e-3, 2.10: -22.9e-3, 3.09: -39.5e-3,
             4.09: -66.2e-3, 5.05: -96.5e-3}
DT_MEAN, DT_MAX = 0.0222, 0.0666   # C2 짝짓기 시각차 평균 / 최대 [s]


def gate_S(d):
    """정면(theta=0) 배치에서의 게이트 공분산 S = Rc + Rl + Q."""
    return CAM.cov(d, 0.0) + LID.cov(d, 0.0) + np.diag([Q_R ** 2, Q_T ** 2])


def chi2(delta, S):
    return float(delta @ np.linalg.solve(S, delta))


def table_weights():
    print('[4·8절] R(d) 성분과 공분산 가중 평균의 방향별 가중치')
    print(f'{"d[m]":>5} {"sC_r[mm]":>9} {"sL_r[mm]":>9} {"cam_r[%]":>9} '
          f'{"sC_t[mm]":>9} {"sL_t[mm]":>9} {"cam_t[%]":>9}')
    for d in (1, 2, 3, 4, 5, 6):
        cr, lr = CAM.sigma_r(d), LID.sigma_r(d)
        ct, lt = CAM.sigma_t(d), LID.sigma_t(d)
        # 카메라 가중 = (1/sC^2) / (1/sC^2 + 1/sL^2) = sL^2 / (sC^2 + sL^2).
        # 반경 가중은 4m 에서 0.14% 라 소수 첫째 자리로는 정보가 뭉개진다.
        wr = lr ** 2 / (cr ** 2 + lr ** 2) * 100
        wt = lt ** 2 / (ct ** 2 + lt ** 2) * 100
        print(f'{d:>5} {cr*1e3:>9.2f} {lr*1e3:>9.2f} {wr:>9.2f} '
              f'{ct*1e3:>9.2f} {lt*1e3:>9.2f} {wt:>9.1f}')

    # sigma_C,r = sigma_L,r 를 푸는 거리. 멱함수 두 개라 닫힌 해가 있다.
    d_star = (0.331 / 0.321) ** (1.0 / (2.741 - 0.354))
    print(f'  교차 거리 d* = {d_star:.4f} m  '
          f'(sigma = {CAM.sigma_r(d_star)*1e3:.3f} mm)')


def table_gate_radius():
    print()
    print(f'[6절] chi^2 = {GATE} 게이트 반경 (정면 배치)')
    print(f'{"d[m]":>5} {"radial[mm]":>11} {"tangential[mm]":>15} '
          f'{"sigma_comb_r[mm]":>17}')
    for d in (1, 2, 3, 4, 5, 6):
        S = gate_S(d)
        sc = math.hypot(CAM.sigma_r(d), LID.sigma_r(d))
        print(f'{d:>5} {math.sqrt(GATE*S[0,0])*1e3:>11.1f} '
              f'{math.sqrt(GATE*S[1,1])*1e3:>15.1f} {sc*1e3:>17.2f}')


def table_uncorrected():
    print()
    print('[5절] delta(d) 무보정 시 진짜 쌍의 chi^2')
    print('      = bias_CL_r^2 / (sC_r^2 + sL_r^2)   [Q 미포함, radial only]')
    print(f'{"d[m]":>6} {"bias[mm]":>9} {"s_comb[mm]":>11} {"chi^2":>8} {"판정":>6}')
    for d, b in BIAS_CL_R.items():
        s = math.hypot(CAM.sigma_r(d), LID.sigma_r(d))
        c = b ** 2 / s ** 2
        print(f'{d:>6.2f} {b*1e3:>9.1f} {s*1e3:>11.3f} {c:>8.0f} '
              f'{"기각" if c > GATE else "통과":>6}')
    print(f'  bias 는 d^1.281 로 커지는데 분모 sigma_C,r 은 d^2.741 로 커진다')
    print(f'  -> 계통 오차는 근거리에서 가장 치명적이다')


def table_dt_cost():
    print()
    print(f'[11-1절] Delta t 미보상의 대가 — 정지 장애물 + 로봇 주행')
    print(f'         (Delta t = {DT_MEAN*1e3:.1f} ms, 순수 radial, ! = 기각)')
    print(f'{"v[m/s]":>7} ' + ' '.join(f'{f"d={d}m":>9}' for d in (1, 2, 3, 4)))
    for v in (0.5, 0.8, 1.0, 1.5):
        cells = []
        for d in (1, 2, 3, 4):
            c = chi2(np.array([v * DT_MEAN, 0.0]), gate_S(d))
            cells.append(f'{c:>8.1f}{"!" if c > GATE else " "}')
        print(f'{v:>7} ' + ' '.join(cells))

    print()
    print('         게이트를 넘지 않는 최대 radial 속도')
    print(f'{"d[m]":>5} {"gate_r[mm]":>11} {"dt=22.2ms":>11} {"dt=66.6ms":>11}')
    for d in (1, 2, 3, 4):
        gr = math.sqrt(GATE * gate_S(d)[0, 0])
        print(f'{d:>5} {gr*1e3:>11.1f} {gr/DT_MEAN:>10.2f}m/s '
              f'{gr/DT_MAX:>10.2f}m/s')


def table_abd():
    print()
    print('[10절] ABD D_max vs 인접 점 간격 (lambda=20deg, k*sigma_r=0.09m, '
          'dphi=0.5deg)')
    lam, dphi, ks = math.radians(20.0), math.radians(0.5), 3 * 0.03
    print(f'{"r[m]":>5} {"점간격[mm]":>12} {"D_max[mm]":>11} {"배수":>7}')
    for r in (1.1, 2.1, 3.1, 4.1, 5.1, 7.1):
        gap = 2 * r * math.sin(dphi / 2) * 1e3
        dmax = (r * math.sin(dphi) / math.sin(lam - dphi) + ks) * 1e3
        print(f'{r:>5} {gap:>12.1f} {dmax:>11.1f} {dmax/gap:>7.1f}')


if __name__ == '__main__':
    table_weights()
    table_gate_radius()
    table_uncorrected()
    table_dt_cost()
    table_abd()
