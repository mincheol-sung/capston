# kf 게인을 키울 때의 걸림돌 시험 (2026-11-21)

후보 `est_gains:=candidate_1121` (실차 bag 37 개 innovation 분석, `../2026-11-21_kf_innovation/README_KO.md`) 을 쓰기 전에 두 걸림돌을 확인한다.

| 걸림돌 | 무엇이 걱정인가 | 어디서 | 무엇으로 판단 |
|---|---|---|---|
| ① 주행 중 SLAM 오차의 시간 상관 | 게인을 키울수록 추정이 SLAM 오차를 따라간다. innovation whiteness 기준이 게인을 실제 최적보다 크게 고른다 | 시뮬 (참값 있음) | 같은 bag 을 배율 격자로 오프라인 재생 → **innovation 이 고른 배율** vs **참값 추정 오차가 가장 작은 배율** |
| ② 속도 상태로 들어가는 포즈 잡음 | K_v 4.16 → 7.89. MPC 속도 상태가 흔들려 토크가 떨린다 | 시뮬 + 실차 | 토크 떨림 RMS, 틱간 Δτ, 상한 닿음, 추종 오차, 슬립 λ — 지금 값 vs 후보 |

## 1. 새 launch 인자 (stack_changelog 2026-11-21 (13))

| 인자 | 값 | 하는 일 |
|---|---|---|
| `est_gains` | 비움·`yaml` (지금) \| `candidate_1121` | `torque_estimator.yaml estimator_presets` 의 잡음 값 묶음으로 게인 설계 |
| `est_noise` | `"키=값,…"` | 한 번만 덮어쓰기 (est_gains 뒤). 키: accel_noise_density accel_bias_walk pose_noise_xy gyro_noise_density gyro_bias_walk pose_noise_yaw_deg |
| `sim_noise` | `"키=값,…"` (시뮬만) | 시뮬 센서 잡음. 시간 상관 SLAM 오차 `slam_gm_xy` [m]·`slam_gm_yaw_deg`·`slam_gm_tau` [s] (1차 Gauss-Markov) |

추정기 기동 로그에 실제로 쓴 잡음 값과 게인이 찍히고, yaml 과 다르면 "← yaml 과 다름" 이 붙는다. ROS 파라미터라 실차 bag 의 `run_info/params` 에도 남는다.

## 2. 시뮬 판 구성 (`cases.txt`, 32 판)

| 환경 | `sim_noise` | 뜻 |
|---|---|---|
| E0 | 없음 | 지금 시뮬 (IMU 0.95 m/s² @200 Hz = 밀도 0.067, SLAM white 2 mm·0.1°) — 지금 게인이 설계된 잡음과 같다 |
| E1 | `accel_noise_moving=1.56, gyro_noise_moving=0.18, slam_noise_xy=0.0015, slam_noise_yaw_deg=0.027` | 오늘 실측: 주행 IMU 밀도 0.11 m/s²/√Hz·0.0128 rad/s/√Hz (×√200), 정지 SLAM 떨림 1.5 mm·0.027°. SLAM 오차 white |
| E2 | E1 + `slam_gm_xy=0.003, slam_gm_yaw_deg=0.05, slam_gm_tau=0.3` | 빠른 상관 SLAM 오차 (정지 실측 ρ(0.1 s) 0.4~0.76 → τ 0.1~0.36 s) |
| E3 | E1 + 같은 크기, `slam_gm_tau=1.5` | 느린 상관 SLAM 오차 (정지 실측 ρ(1 s) 0.4~0.6 → τ 1.1~2 s) |

× 게인 2 (yaml, candidate_1121) × 경로 2 (circles8 W2.0 0.7 m/s, 정사각형 1.5 m/s) × 2 판. C, `velocity_source:=estimator`, 바닥 μ 0.45.
시뮬 IMU·SLAM 포즈도 녹화해서 ① 의 오프라인 재생에 쓴다.

```bash
cd /home/smoc/capston/capston && bash analysis/estimator_tuning/2026-11-21_gain_risk/run_all.sh
```

실차 스택이 같은 NUC 에서 돌고 있으면 `NICE=19 bash …/run_all.sh` (시뮬이 느려져 끊김이 나면 그 판은 다시).

### 판단 기준

- ①: E1 에서 innovation 이 고른 배율 ≈ 참값 최적 배율이면 방법이 맞게 동작한다. E2·E3 에서 innovation 이 고른 배율이 참값 최적보다 크면
  걸림돌 ① 이 실제다 — 그때 후보의 참값 추정 오차가 지금 값보다 나쁜지로 크기를 본다.
- ②: 같은 환경에서 후보의 토크 떨림·Δτ·상한 닿음이 얼마나 느는지, 추종 오차·슬립 λ 와 같이 본다.

## 3. 실차 A/B

```bash
amrall shape:=circles8 width:=2.0 yaw_mode:=tangent speed:=0.7 bag:=true                              # 지금 값
amrall shape:=circles8 width:=2.0 yaw_mode:=tangent speed:=0.7 bag:=true est_gains:=candidate_1121    # 후보
```

번갈아 3 판씩. 다른 인자는 같게. 주의: `slip_logger` 의 λ 는 추정기 속도를 기준으로 계산하므로 게인이 바뀌면 λ 의 기준도 바뀐다
— 두 판의 슬립 비교는 SLAM 포즈 차분 속도 기준으로 다시 계산해서 본다.
