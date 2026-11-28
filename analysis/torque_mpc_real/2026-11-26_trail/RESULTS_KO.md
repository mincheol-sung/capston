# RViz 지나간 경로의 좌우 흔들림 — 실제 운동인가 (2026-11-26)

질문: RViz `Trail (estimator)` 이 직선 구간에서 좌우로 꺾이는데, 눈으로 보면 로봇은 그렇게 움직이지 않는다.

## 결론

흔들림은 실제 운동이 아니다. state_estimator 가 SLAM 포즈를 받을 때마다 위치를 고치면서 생긴 계단이다.

| 근거 | 191922 꺾은선 1.5 m/s | 191432 지그재그 MPC | 190718 지그재그 PP |
|---|---|---|---|
| estimator 100 Hz 표본 사이, 바퀴 속도로 설명 안 되는 이동 최대 | **46.0 mm** | 16.0 mm | 17.7 mm |
| 같은 경로를 두 번 미분한 몸체 횡가속(1~5 Hz)과 IMU 의 상관 | **−0.22** | +0.08 | — |
| 위 횡가속 RMS (경로 → IMU) | 1.57 → 0.88 m/s² | 0.54 → 0.35 | 0.50 → 0.29 |

- 경로 횡가속이 IMU 보다 크고 IMU 와 상관이 없다 → 경로가 그린 좌우 운동을 몸체가 실제로 겪지 않았다.
- `/tracked_pose` 자체도 스캔 매칭 잡음(0.5 s 고주파 횡흔들림 RMS 0.55~0.85 cm)이 있지만 계단은 estimator 쪽이 크다.
- `/tracked_pose` 는 로봇 중심이 아니라 카메라 IMU 점이다. estimator − tracked_pose 의 몸체 좌표 차이 평균 (−10.9, −1.7) cm,
  표준편차 0.6~1.5 cm → `torque_estimator.yaml imu_offset (0.1093, 0.017)` 과 맞다. 그대로 그리면 11 cm 어긋난다.

## 바꾼 것 — `torque_viz_node`

| 토픽 | 내용 | RViz 기본 |
|---|---|---|
| `/torque_viz/trail_slam` | `/tracked_pose` 를 p − R(ψ)·r 로 base_footprint 로 옮긴 경로 (10 Hz) | 켬 (초록) |
| `/torque_viz/trail_smooth` | 바퀴 FK + 자이로 ω_z 100 Hz 적분, SLAM 을 2차 complementary filter (τ 0.5 s, 몸체 속도 편향 적분) 로 섞음. 보정을 바퀴 표본마다 나눠 적용 → 계단 없음 | 켬 (자홍) |
| `/torque_viz/trail` | 예전 estimator 경로 | 끔 (회색) |

코드 `amr_mpc/core/trail_fusion.py` (ROS 없음), 시험 `test/test_trail_fusion.py`.

## bag 재생 검증 (`validate.py`, 노드와 같은 코드·같은 수신 순서)

| 회차 | 경로 | 표본 간 속도 밖 이동 최대 | 0.5 s 횡흔들림 RMS / max | 횡가속 1~5 Hz RMS (IMU) | SLAM(base) 과 거리 RMS / max |
|---|---|---|---|---|---|
| 191922 | estimator | 46.0 mm | 1.17 / 4.18 cm | 1.57 (0.88) | 2.28 / 6.74 cm |
| | SLAM | 24.8 mm | 0.85 / 2.86 cm | 0.93 | 0 |
| | smooth | **1.5 mm** | **0.50** / 2.15 cm | 0.28 | 2.89 / 6.96 cm |
| 191432 | estimator | 16.0 mm | 0.70 / 2.14 | 0.54 (0.35) | 0.91 / 3.33 |
| | smooth | **0.6 mm** | 0.56 / 0.98 | 0.11 | 1.14 / 4.39 |
| 190718 | estimator | 17.7 mm | 0.63 / 1.76 | 0.50 (0.29) | 1.08 / 2.84 |
| | smooth | **0.4 mm** | 0.51 / 0.88 | 0.09 | 0.91 / 2.12 |

- smooth − SLAM 차이는 평균 편향 0 (몸체 x +0.01, y +0.03 cm), 시간 밀림 없음 (±50 ms 밀면 커짐). 1.5 m/s 꺾은선에서 남는 2.9 cm 는
  바퀴 미끄러짐 구간에서 dead reckoning 과 SLAM 이 갈라지는 몫 — 어느 쪽이 참인지는 이 자료로 못 가린다.
- 바퀴 FK 속도 / SLAM 속도 비 vx 0.94~0.96 이지만 상수 스케일 보정은 거리 RMS 를 줄이지 못해 넣지 않았다.
- 그림 `trail_191922.png` (오른쪽: estimator 계단이 가장 큰 곳 확대).

## 아닌 것

- 표시 전용. MPC·PP 가 쓰는 `/state_estimator/odom` 은 그대로다. 다만 MPC 는 위 46 mm 계단을 그대로 받는다 — 추종 오차 지표
  (`estimator` 기준) 에도 이 계단이 들어 있다. 추정기 쪽 수정은 이 작업 범위 밖.
