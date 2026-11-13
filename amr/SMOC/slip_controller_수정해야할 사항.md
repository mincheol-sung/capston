# AMR 슬립 제어기 구조도 — 수정할 것

- 대상: [AMR 슬립 제어기 구조도](https://claude.ai/artifact/X4fkdPvuHnDosLSQctFrQo)
- 검토일: 2026-10-23 ~ 10-26 (Claude Code 세션, NUC `2026amr_ws`)
- 근거 파일: `mapping_2d.lua`, `localization_2d.lua`, `ekf.yaml`, `dra_mppi.yaml`, `wheel.yaml`, `base.yaml`, `amr_basic.urdf`
- 실측 bag: `bags/mpc_chain_20260923_135325` (주기·지연), `bags/mpc_chain_20260911_012414` (측위 계단)
- 검산 스크립트: NUC scratchpad `check.py`, `check2.py`, `align.py`, `observer.py`

---

## 그대로 둬도 되는 것

| 항목 | 이유 |
|---|---|
| `keyboard_command_gate`를 최종 단에 | `command_type: wheels`가 Float32MultiArray 길이 4를 수정 없이 받음 |
| DRA_MPPI → MPC 인터페이스 | `/dra_mppi/reference_trajectory` 그대로 |
| `v = Rᵀ(ψ)·d/dt[X, Y]` | world 속도에 Rᵀ를 left-multiply → body 속도. 식 자체는 맞음 |
| torque MPC 동역학 | ω × v 항 포함, F = Bᵀτ/r (일률 보존 검산 오차 0.0) |
| `/wheel_states` = JointState | 표준 메시지 |
| scan 가공을 perception으로 이동 | 토픽 이름 그대로라 영향 없음 |

---

## 막히는 것 (반드시 고칠 것)

### ① `/odom` 제거 → PLANNER가 "변경 없음"이 안 됨  `[ ]`

- 현재: `/odom`(휠) → `ekf_filter_node` → `/odometry/filtered` + TF `odom→base_footprint`
- `ekf.yaml`: `odom0: /odom`에서 vx, vy만, `imu0`에서 yaw rate만 씀
- `/odom`이 없으면 EKF가 vx, vy 입력을 잃음 → `odom` 프레임 붕괴 → DRA_MPPI(`planning_frame: odom`), traversability, object_tracker(`track_frame: odom`), ekf_carto(map→odom)가 모두 멈춤
- **수정**: 구조도에 `ekf_filter_node` 블록을 추가하고, `odom0`을 LiDAR odometry 속도로 교체. 토픽 계약이 유지되므로 플래너는 실제로 변경 없음
- 참고: "LiDAR v → 역기구학 → ω_nom → 정기구학 → odom"은 T_FWD·T⁻¹ = I₃ 라서 v를 바로 넣는 것과 같음 (‖T_FWD T⁻¹ − I₃‖ = 1.6e-15)

### ② "scan-matched 포즈만"은 `/tracked_pose`에서 안 나옴  `[ ]`

- `mapping_2d.lua`: `use_pose_extrapolator = true`, `use_imu_data = true`, `use_odometry = true`
- `localization_2d.lua`: `pose_publish_period_sec = 1e-2`
- 실측: `/tracked_pose` 98.22 Hz, `/scan_front` 10.0 Hz → 10개 중 9개가 외삽값
- 실측 (09-11 bag): pose graph 최적화 때 `/tracked_pose` 계단 34회 / 157 s, 최대 953 mm → 10 Hz 미분 시 **9.53 m/s 스파이크**
- **수정**: `v_body`는 pose graph가 없는 LiDAR odometry(rf2o 등 scan-to-scan)에서 받음. cartographer는 localization 전용

### ③ 가짜 슬립 (시간 정렬)  `[ ]`

- 원인: LiDAR 차분 v = 과거 구간 [t₀, t₁]의 평균, 휠 ω = 현재 순간값. 가속 중엔 서로 다른 시점의 속도
- 실측 지연 (수신 − stamp): `/scan_front` p50 99.9 ms, `/scan` p50 135.4 ms / p90 167.2 ms / max 314.4 ms
- 현재 `/measured_wheel_speeds`는 Float32MultiArray → **stamp 필드 없음**
- 시뮬레이션 (슬립 0, 가속 진폭 0.94 m/s², 지연 135 ms):

  | 방법 | 가짜 λ 평균 | 최대 |
  |---|---|---|
  | 최신 ω, ZOH (stamp 없음) | 0.260 | 0.472 |
  | 도착시각 기준 구간 평균 | 0.166 | 0.268 |
  | **stamp 구간 [t₀, t₁] 평균** | **0.0004** | **0.0012** |

- **수정**:
  - `ak40_can_node`가 `/wheel_states`에 `header.stamp` = CAN 수신 시각을 넣음
  - EST ①에 휠 샘플 버퍼(~0.3 s)를 두고, LiDAR v의 stamp 구간과 같은 구간으로 ω̄를 계산
  - 가장 깔끔한 방법: 바퀴 각도 차분 ω̄ = (θ(t₁) − θ(t₀)) / (t₁ − t₀)
- 결과적으로 λ는 최소 ~135 ms 늦은 정보. 빠른 감지는 EST ②(dω/dt, 휠만, 100 Hz)가 담당

### ④ torque MPC 포즈와 reference의 프레임 불일치  `[ ]`

- 구조도: `torque_mpc_node` ← `/tracked_pose` (**map** 프레임)
- reference: DRA_MPPI `planning_frame: odom` (**odom** 프레임)
- X_e = X_r^odom − X^map = (진짜 추종 오차) − (map↔odom 오프셋)
- 실측 (09-11 bag): `map→odom` 누적 75.93 m / 157 s, 5 cm 넘는 계단 312회
- **수정**: MPC 포즈는 `/odometry/filtered`(odom 프레임)에서 받음
- 주의: 이건 제어 주기가 아니라 **좌표계** 문제

---

## 소소한 것

### a. tracking_frame 오프셋  `[ ]`
- `/tracked_pose`의 기준점은 `camera_gyro_frame` (`localization_2d.lua` 29행), base_link에서 앞쪽 0.116 m
- 그 점의 속도 = base 속도 + ω × r → ωz 1 rad/s에서 횡방향 0.116 m/s, 2 rad/s에서 0.232 m/s
- ②대로 LiDAR odometry를 쓰고 base frame을 `base_link`로 두면 사라짐

### b. `/wheel_torque_cmd` 단위  `[ ]`
- 구조도에 "N·m 또는 전류 A"로 둘 다 적혀 있음 → 하나로 고정
- 권장: N·m 고정, 전류 환산은 드라이버 안에서. 옛 `/wheel_cmd`(rad/s)는 재사용 금지

### c. fault → 안전정지  `[ ]`
- `/mpc/arm`에 false를 보내는 방식은 지금 게이트로 동작함
- 하지만 fault가 남아 있어도 키보드 `a`로 다시 arm 가능 → 게이트에 **latch되는 fault 입력** 추가

### d. 토크 모드의 "0"은 정지가 아님  `[ ]`
- 속도 모드 [0,0,0,0] = 멈춰라, 토크 모드 τ = 0 = 힘 빼라 (굴러감)
- timeout·disarm 동작을 새로 정의: 감속 토크 / damping 모드 / 속도 0 전환 중 선택

---

## 수정 흐름 재검토에서 나온 것

### N1. MPC 속도 상태 — 관측기 블록이 필요  `[ ]`
- MPC는 100 Hz로 v가 필요한데 LiDAR는 10 Hz + 지연 135 ms
- EKF는 등속 모델이라 vx, vy는 사실상 직전 값 유지(ZOH). IMU는 ωz만 갱신 (`imu0_config` 가속도 off)
- 원래 구조도의 `mpc_velocity_interp`가 이 역할 → **지우지 말고 토크 모델 관측기로 구현**
- 시뮬레이션 (가속 진폭 0.94 m/s², 모델 오차 = 힘의 10 %, LiDAR 10 Hz, 지연 135 ms):

  | 방법 | v 오차 평균 | 최대 |
  |---|---|---|
  | LiDAR 값 유지 (ZOH) | 97.8 mm/s | 190.0 mm/s |
  | 토크 모델 관측기, 지연 무시 | 64.3 mm/s | 105.4 mm/s |
  | **토크 모델 관측기 + 지연 보상** | **10.6 mm/s** | **21.1 mm/s** |

- 지연 보상 = 측정을 stamp 시점의 추정값과 비교하고, 그 오차를 현재로 넘김 (out-of-sequence measurement / re-propagation)

### N2. EKF 주기  `[ ]`
- `ekf.yaml` `frequency: 50.0` < MPC 100 Hz
- N1처럼 관측기를 MPC 안에 두면 상관없어짐. 아니면 EKF를 100 Hz로 (ekf_carto와 같은 값으로 맞추라는 주석 있음)

### N3. LiDAR odometry 입력 선택  `[ ]`
- 합친 `/scan`은 지연이 크고 지터가 큼 (max 314 ms), 두 LiDAR가 동기화돼 있지 않음
- rf2o류는 단일 스캐너용 → `/scan_front` 단독(지연 100 ms, 지터 작음)과 bag으로 비교
- 확인할 것: 속도가 매끄러운지, 정지 구간에서 0이 나오는지

### N4. LiDAR odometry 실패 시 대비  `[ ]`
- 휠이 odom에서 완전히 빠짐 → 특징 없는 복도에서 LiDAR odometry가 미끄러지면 받쳐줄 것이 없음
- `fault_monitor`에 LiDAR odometry 품질 감시 추가 (매칭 점수, 휠과의 괴리 등)

---

## 제어 주기에 대한 정리 (10-26 질문)

| 루프 | 실제 주기 한계 | 이유 |
|---|---|---|
| torque MPC | **100 Hz 가능** | 관측기가 자기가 낸 τ로 v를 예측하고, LiDAR는 10 Hz로 보정만 함 (N1) |
| 슬립 λ (EST ①) | **10 Hz, 135 ms 지연이 본질적 한계** | 휠과 무관한 기준 속도가 LiDAR뿐이라서 |
| 빠른 슬립 개입 (EST ②) | **100 Hz** | dω/dt는 휠만으로 계산 — 기준 속도가 필요 없음 |
| yaw rate | **200 Hz** | 자이로가 직접 측정 (외삽 아님) |

- 측정 주기 ≠ 제어 주기. 제어 성능을 좌우하는 것은 **모델 정확도와 측정 지연**이지 측정 주기 자체가 아님
- 휠 정보를 MPC용 추정기에만 넣는 것(슬립 기준에는 미사용)이 허용되는지는 **교수님께 확인할 사항**

---

## 수정 후 흐름 (N1~N4 반영)

```
rplidar ×2 ─▶ /scan_front (또는 /scan) ─▶ lidar_odometry (pose graph 없음, base_link 기준, stamp)
                                            │ v_body
                                            ├──────────────▶ slip EST ① (stamp 구간 ω̄ 와 비교)
                                            ▼
/camera/imu ─▶ ekf_filter_node (odom0 = LiDAR 속도, imu0 = ωz) ─▶ /odometry/filtered + TF odom→base
cartographer (localization) ─▶ ekf_carto ─▶ TF map→odom
                                                   │
                                   ┌───────────────┴──────────────┐
                                   ▼                              ▼
                       DRA_MPPI (변경 없음)          torque_mpc_node
                                                       └ 토크 모델 관측기 (100 Hz, 지연 보상)
ak40_can_node ─▶ /wheel_states (header.stamp) ─▶ 휠 버퍼 ─▶ slip EST ①②③
torque_mpc ─▶ τ_nom ──┐
slip CTL / CPS ─▶ Δτ_slip, T_max ──┴▶ torque_mixer ─▶ gate (+fault latch) ─▶ /wheel_torque_cmd [N·m] ─▶ ak40_can_node
fault_monitor ─▶ (모터 에러, timeout, LiDAR odom 품질, 추정값 범위) ─▶ gate fault latch
```

---

## 다음 순서

1. `[ ]` 교수님 확인: 휠 정보를 MPC 상태 추정에만 쓰는 것이 허용되는지
2. `[ ]` N3: 기존 bag으로 LiDAR odometry 후보 비교 (`/scan_front` vs `/scan`) — 로봇 없이 가능
3. `[ ]` 구조도 반영: ①②③④ + a~d + N1~N4
4. `[ ]` 스켈레톤: 노드 껍데기 + 토픽·타입·단위만 맞춰 `ros2 launch` 연결 확인
5. `[ ]` 동역학 모델에 넣을 항 결정: ω × v (필수), 바퀴·로터 반사관성 J_w (10:1 기어), 마찰
