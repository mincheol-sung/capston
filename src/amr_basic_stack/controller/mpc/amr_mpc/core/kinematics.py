#!/usr/bin/env python3
"""메카넘 kinematics — 두 구조(A 몸체속도, B 바퀴속도)가 공유한다.

  Ẋ = f(X, v) = R(θ) v                        (world ← body)
  Ẋ_e = A_c X_e + B_c v_e,  A_c 의 3열 = 기준 속도벡터를 90° 회전,  B_c = R(θ_r)     (reference 주위 Jacobian)
  A_k = I + T_s A_c (exact — A_c nilpotent),  B_k = T_s B_c                            (ZOH)
  ω = T⁻¹ v,  v = T ω     T⁻¹ (4×3) 는 드라이버 twist_to_wheels 와 같은 식, T = pinv(T⁻¹)
"""
import numpy as np

WHEELS = ("FL", "FR", "RL", "RR")


def f(X, u):
    """몸체 kinematics. ROS REP-103 (x 전방, y 좌측, yaw 반시계)."""
    th = X[2]
    return np.array([u[0] * np.cos(th) - u[1] * np.sin(th),
                     u[0] * np.sin(th) + u[1] * np.cos(th),
                     u[2]])


def kinematic_jacobians(theta_r, vxr, vyr, Ts):
    """reference 주위 linearization 을 ZOH 이산화한 (A_k, B_k). A_c 의 3열은 기준 속도벡터를 90° 돌린 것."""
    Xr_dot = vxr * np.cos(theta_r) - vyr * np.sin(theta_r)
    Yr_dot = vxr * np.sin(theta_r) + vyr * np.cos(theta_r)
    A_c = np.array([[0.0, 0.0, -Yr_dot],
                    [0.0, 0.0,  Xr_dot],
                    [0.0, 0.0,  0.0]])
    B_c = np.array([[np.cos(theta_r), -np.sin(theta_r), 0.0],
                    [np.sin(theta_r),  np.cos(theta_r), 0.0],
                    [0.0, 0.0, 1.0]])
    return np.eye(3) + Ts * A_c, Ts * B_c


def mecanum_matrices(wheel_radius, wheel_base, robot_width):
    """(T_inv 4×3, T_fwd 3×4, null_dir 4, k).  k = (wheel_base + width)/2 — L1 부하식의 ω 팔 길이."""
    k = (wheel_base + robot_width) / 2.0
    T_inv = (1.0 / wheel_radius) * np.array([[1.0, -1.0, -k],     # FL
                                             [1.0,  1.0,  k],     # FR
                                             [1.0,  1.0, -k],     # RL
                                             [1.0, -1.0,  k]])    # RR
    T_fwd = np.linalg.pinv(T_inv)
    null_dir = np.linalg.svd(T_fwd)[2][-1]                         # 출력에 안 보이는 바퀴 조합 [1,1,−1,−1]/2
    return T_inv, T_fwd, null_dir, k
