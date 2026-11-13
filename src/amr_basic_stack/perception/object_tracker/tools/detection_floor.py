#!/usr/bin/env python3
"""동정 판별의 검출 하한 — "몇 m/s 부터 DYNAMIC 이라 부를 수 있나".

    python3 tools/detection_floor.py

sigma_a 를 정하려면 그 값이 무엇을 사는지 알아야 한다. 이 스크립트가 그
환산표를 만든다. 실험이 아니라 계산이다 — C2 실측 R(d) 와 CWNA Q(dt) 만
있으면 정상상태 P_vv 가 Riccati 반복으로 나오고, 거기서 하한이 닫힌 형태로
떨어진다.

유도:
  DYNAMIC 판정은  d^2 = v^T P_vv^-1 v > chi2_enter_dynamic  이다.
  P_vv 의 고유분해 P_vv = sum lambda_i u_i u_i^T 에서 속도가 u_i 방향이면

      v_min,i = sqrt(chi2_enter_dynamic * lambda_i)

  즉 방향별 하한은 그 방향 속도 표준편차의 sqrt(9.21) = 3.035 배다.
  P_vv 는 정상상태 값이므로 트랙이 충분히 오래 살았다고 가정한 하한이다.

P 는 실제 오차 공분산이 아니다:
  Riccati 가 주는 P 는 "가속도가 정말 sigma_a 세기로 흔들릴 때"의 오차
  공분산이다. 진짜 정지한 물체는 가속이 0 이라 실제 v_hat 산포가 P 보다
  작다 (sigma_a=1.0, d=3m 에서 P 는 178mm/s 를 주장하지만 실측은 96mm/s).
  그래서 표에 두 열을 나란히 둔다.

      v_min       P 로 계산한 판정 문턱.  판정식이 실제로 P 를 쓰므로 이 값이
                  "몇 m/s 에서 넘어가나"의 정답이다.
      오경보       진짜 정지 물체를 몬테카를로로 돌려 d^2 > 문턱이 되는 비율.
                  P 가 맞다면 1% 여야 한다. 그보다 훨씬 작으면 필터가
                  비관적(sigma_a 과대)이라는 뜻이고, 크면 그 반대다.

  이 괴리가 sigma_a 를 실측해야 하는 이유다. chi^2 문턱이 "0.99" 라는 이름값을
  하려면 P 가 실제 오차 공분산이어야 한다.

R(d) 계수 출처:
  sensor_fusion/camera_lidar_fusion_node.py 의 declare_parameter 기본값.
  C2 실측 2026-09-04 (EXPERIMENT_C2.md, 커밋 5bdd219). 여기 값을 바꾸면
  그쪽도 같이 바꿔야 한다 — 이 파일은 분석용 사본이다.

주의 — 이 표는 낙관적이다:
  C2 의 sigma 는 강체 정지 타깃(suitcase)을 반복 측정해 얻은 값이다. 보행자는
  대표점 자체가 팔·다리·몸통 흔들림으로 움직여서 실제 R 이 더 크다. 그 몫은
  sigma_a 가 흡수해야 하는데, 그래서 sigma_a 를 실측으로 정해야 한다
  (NIS 평균이 2 가 되도록 — core/config.py 독스트링).
"""
import math

import numpy as np

from object_tracker.core.config import TrackerConfig
from object_tracker.core.kalman import H, process_noise, transition

# ── C2 실측 R(d).  camera_lidar_fusion_node.py 의 기본 파라미터 사본 ──────── #
CAM_SIGMA_R = (0.321e-3, 2.741)     # sigma_r = coef * d^exp  [m]
CAM_SIGMA_T = (0.560e-3, 1.450)
LID_SIGMA_R = (0.331e-3, 0.354)
LID_SIGMA_R_FLOOR, LID_SIGMA_R_BREAK = 1.6e-3, 4.5
LID_SIGMA_T = 6.5e-3                # 거리 무관 (C2 관측 중앙값)

# 카메라 검출률. C2 실측: 6.05m 92.8%, 7.10m 0%.
CAM_MAX_RANGE = 6.5


def sensor_sigmas(d):
    """(카메라 sigma_r, sigma_t, 라이다 sigma_r, sigma_t). 단위 m."""
    cam_r = CAM_SIGMA_R[0] * d ** CAM_SIGMA_R[1]
    cam_t = CAM_SIGMA_T[0] * d ** CAM_SIGMA_T[1]
    lid_r = (LID_SIGMA_R_FLOOR if d >= LID_SIGMA_R_BREAK
             else LID_SIGMA_R[0] * d ** LID_SIGMA_R[1])
    return cam_r, cam_t, lid_r, LID_SIGMA_T


def fused_sigmas(d, camera=True):
    """융합 후 (반경, 횡) 표준편차. 극좌표축이 R 의 고유축이라 스칼라로 합쳐진다.

    카메라가 없는 거리(> CAM_MAX_RANGE)에서는 라이다 단독이다.
    """
    cam_r, cam_t, lid_r, lid_t = sensor_sigmas(d)
    if not camera:
        return lid_r, lid_t
    r = 1.0 / math.sqrt(1.0 / cam_r ** 2 + 1.0 / lid_r ** 2)
    t = 1.0 / math.sqrt(1.0 / cam_t ** 2 + 1.0 / lid_t ** 2)
    return r, t


def steady_state_P(R, dt, sigma_a, iters=4000):
    """CV Kalman 의 정상상태 P. Riccati 를 그냥 반복해서 수렴시킨다."""
    F = transition(dt)
    Q = process_noise(dt, sigma_a)
    P = np.eye(4) * 1.0
    I4 = np.eye(4)
    for _ in range(iters):
        P = F @ P @ F.T + Q
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        A = I4 - K @ H
        P = A @ P @ A.T + K @ R @ K.T
        P = 0.5 * (P + P.T)
    return P


def static_target_monte_carlo(R, dt, sigma_a, chi2_gate, n=6000, seed=0):
    """진짜 정지한 물체를 필터에 먹여 (실측 E|v_hat|, d^2 초과 비율)."""
    from object_tracker.core.kalman import KalmanFilter, velocity_chi2

    rng = np.random.default_rng(seed)
    L = np.linalg.cholesky(R)
    kf = KalmanFilter(np.zeros(2), R, sigma_v0=1.5)
    speeds, exceed = [], 0
    burn = 500
    for k in range(n):
        kf.predict(dt, sigma_a)
        kf.update(L @ rng.standard_normal(2), R)
        if k >= burn:
            speeds.append(kf.speed)
            exceed += velocity_chi2(kf) > chi2_gate
    return float(np.mean(speeds)), exceed / (n - burn)


def floor_for(d, dt, sigma_a, chi2_gate):
    camera = d <= CAM_MAX_RANGE
    sr, st = fused_sigmas(d, camera)
    # 물체가 정면(+x)에 있다고 두면 반경 = x, 횡 = y 로 축이 정렬된다.
    R = np.diag([sr ** 2, st ** 2])
    Pvv = steady_state_P(R, dt, sigma_a)[2:4, 2:4]
    v_min_r = math.sqrt(chi2_gate * Pvv[0, 0])
    v_min_t = math.sqrt(chi2_gate * Pvv[1, 1])
    v_claim = math.sqrt(0.5 * np.trace(Pvv)) * math.sqrt(math.pi / 2.0)
    v_real, fa = static_target_monte_carlo(R, dt, sigma_a, chi2_gate)
    return v_min_r, v_min_t, v_claim, v_real, fa, camera, sr, st


def main():
    cfg = TrackerConfig()
    dt = 1.0 / 12.0            # 라이다 12Hz. 융합은 ApproximateTime 이라 이 이하.
    # Historical chi-square diagnostic, no longer the motion classifier.
    gate = 9.21034
    print('Historical CV velocity diagnostic; not the 3-mode IMM decision rule.')

    print(f'검출 주기 dt = {dt*1000:.1f} ms (12 Hz)')
    print(f'DYNAMIC 진입 문턱 chi^2 = {gate:.3f}  '
          f'-> 하한은 방향별 sigma_v 의 {math.sqrt(gate):.3f} 배')
    print()

    for sigma_a in (0.15, 0.3, 1.0, 3.0):
        mark = '  <- 현재 기본값' if sigma_a == cfg.sigma_a else ''
        print(f'=== sigma_a = {sigma_a} m/s^1.5{mark} ===')
        print(f'{"d[m]":>5} {"센서":>7} {"sig_r":>8} {"sig_t":>8} '
              f'{"v_min 반경":>11} {"v_min 횡":>10} '
              f'{"정지|v| P주장":>13} {"실측":>8} {"오경보":>8}')
        for d in (1.0, 2.0, 3.0, 4.0, 6.0, 7.0, 9.0):
            vr, vt, vc, vreal, fa, cam, sr, st = floor_for(d, dt, sigma_a, gate)
            who = '융합' if cam else '라이다'
            print(f'{d:5.1f} {who:>7} {sr*1000:6.2f}mm {st*1000:6.2f}mm '
                  f'{vr*1000:9.0f}mm/s {vt*1000:8.0f}mm/s '
                  f'{vc*1000:11.0f}mm/s {vreal*1000:6.0f}mm/s {fa*100:7.2f}%')
        print()

    print('읽는 법:')
    print('  v_min 은 "이 속도면 정상상태에서 DYNAMIC 으로 넘어간다"의 하한이다.')
    print('  보행 1.2 m/s = 1200 mm/s 와 비교해 여유를 본다.')
    print('  "P주장" 과 "실측" 이 벌어질수록 필터가 비관적이고, 오경보가')
    print('  1% 보다 작아진다 — 문턱 이름값(0.99)이 그만큼 안 맞는다는 뜻이다.')
    print(f'  거리 의존성이 약한 이유: 융합 위치 잡음이 {CAM_MAX_RANGE}m 안에서')
    print('  1mm 이하 ~ 수 mm 라 P_vv 를 R 이 아니라 Q 가 지배한다.')
    print(f'  {CAM_MAX_RANGE}m 를 넘으면 카메라가 검출을 못 해(C2: 7.10m 0%)')
    print('  라이다 단독이 되고, 라이다 sigma_t 6.5mm 가 횡방향을 지배한다.')


if __name__ == '__main__':
    main()
