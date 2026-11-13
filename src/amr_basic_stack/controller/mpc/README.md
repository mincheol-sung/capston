# amr_mpc — WP4 궤적추종 MPC

Wang, *MPC System Design and Implementation Using MATLAB* 1~2장 형식의 궤적추종 MPC.
DRA-MPPI 가 낸 기준열을 따라가며 `/cmd_vel` 을 소유한다.

```text
/dra_mppi/reference_trajectory ──┐   PlanarReferenceArray (10 Hz, dt 0.2 s × 20 step, odom 프레임)
                                 ├─▶ mpc_tracking_node ──▶ /cmd_vel (Twist, 25 Hz)
/odometry/filtered ──────────────┘   Odometry (EKF)          ├─▶ /mpc/reference_horizon (Path, 보간된 0.8 s 지평)
                                                             └─▶ /mpc/status (Float64MultiArray)
```

| 실행 파일 | 역할 |
|---|---|
| `mpc_tracking_node` | **MPC 를 소유하는 유일한 노드.** 기준열 구독 → 보간 → MPC → `/cmd_vel` |
| `fake_reference_node` | 원궤적 기준열 발행 (`circle.launch.py` / `run_circle.sh` 가 감쌈) |
| `figure8_reference_node` | 8자 경로를 같은 메시지로 발행. 한 바퀴 뒤 기동 포즈로 복귀·정지 |
| `sim_plant_node` | 식별한 구동계(FOPDT)+기구학 시뮬. `/cmd_vel` → `/odometry/filtered` |

## 패키지 구조

```text
amr_mpc/
├── core/      ROS 무관, 두 구조가 공유
│   ├── config.py        plant.yaml / plant_wheel.yaml 적재 (환경변수 AMR_MPC_PLANT_YAML 등으로 교체 가능)
│   ├── kinematics.py    f, kinematic_jacobians (A_k, B_k), mecanum_matrices (T⁻¹, T, null 방향)
│   ├── qp.py            Hildreth (교재 QPhild), Nesterov FGM — 교재 Ex 2.10/2.11 검산
│   ├── reference.py     기준열 → 임의 시각 보간 (ReferenceHorizon), 원궤적 생성
│   └── figure8.py       8자 궤적 (Gerono, 등속 호길이 재매개화)
├── body/      구조 A — 몸체속도 입력 (/cmd_vel).  plant.yaml
│   ├── model.py         FOPDT ZOH, (A_m, B_m, C_m) 9차, Wang 증강 12차
│   └── mpc.py           F_free·Φ, E·F, L1·rate·연성출력 제약, step()
├── wheel/     구조 B — 바퀴 각속도 입력 (/wheel_cmd).  plant_wheel.yaml
│   ├── model.py         바퀴별 FOPDT, (A_m, B_m, C_m) 11차, 증강 14차
│   └── mpc.py           바퀴 box·가속 제약, step()
└── nodes/     ROS 노드 (실행 파일 이름은 setup.py 의 console_scripts)
    ├── body_tracking.py       mpc_tracking_node
    ├── wheel_tracking.py      wheel_tracking_node
    ├── fake_reference.py      fake_reference_node
    ├── figure8_reference.py   figure8_reference_node
    ├── sim_plant_body.py      sim_plant_node
    ├── sim_plant_wheel.py     sim_plant_wheel_node
    └── keyboard_command_gate.py
config/   plant.yaml (A 설계값) · plant_wheel.yaml (B 설계값) · mpc.yaml (노드 배선)
launch/   mpc · circle · figure8 · wheel_circle
test/     pytest (core 만, ROS 불필요)
tools/    실차 진단 (measure_odom_lag, diag_fig8, combo_lag_test)
```

Import 는 전부 절대 경로(`from amr_mpc.core import config`)다. 자기검사는 `controller/mpc` 에서 모듈로 실행한다:

```bash
python3 -m amr_mpc.core.config      # 어떤 YAML 이 실제로 적재되는지
python3 -m amr_mpc.core.qp          # 교재 예제 검산
python3 -m amr_mpc.body.model       # A 모델 검산
python3 -m amr_mpc.body.mpc         # A 원궤적 폐루프
python3 -m amr_mpc.wheel.model      # B 모델 검산
python3 -m amr_mpc.wheel.mpc        # B 원궤적 폐루프
python3 -m pytest -q test
```

## 빌드·실행

```bash
cd ~/2026amr_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select amr_mpc --symlink-install && source install/setup.bash
ros2 launch amr_mpc mpc.launch.py                       # 플래너 + EKF 가 떠 있을 때
ros2 launch amr_mpc figure8.launch.py                    # 8자 경로 + 제자리 복귀
ros2 launch amr_mpc circle.launch.py sim:=true speed:=0.5   # 원궤적, 로봇 없이
./run_circle.sh 1.0 0.15 2                              # 원궤적 실차 (부하 예산 검사 포함)
pytest -q src/amr_basic_stack/controller/mpc/test       # 보간 단위검사
```

파라미터는 `config/mpc.yaml`. `max_wheel_rad_per_sec` 는 `amr_bringup/config/base.yaml` 과 같아야 한다.

## 설정 구조

설계 파라미터는 **`config/plant.yaml` 한 곳**에 있다. `body/model.py`·`body/mpc.py` 가 import 시점에
읽고, ROS 노드는 그 값을 파라미터 기본값으로 쓴다.

| 절 | 내용 | 언제 바꾸나 |
|---|---|---|
| `model` | 명령→응답 FOPDT (τ, K, θ) | **sys_id 재피팅** 하면 이 세 줄만 |
| `geometry` | 차체 치수 → `K_GEOM`, `V_LIMIT` 파생 | 차체가 바뀌면. `base.yaml` 과 같게 |
| `limits` | 바퀴 한계, 명령 가속도, 부하 예산 | 하드웨어 한계·안전 여유를 조정할 때 |
| `tuning` | `Ts`, `Np`, `Nc`, `q_pos`, `q_yaw`, `r_du` | 제어기 설계값 |
| `safety` | `odom_timeout`, `ref_timeout` | 끊김 판정 |

| 파일 | 역할 |
|---|---|
| `config/plant.yaml` | **설계값 단일 출처.** 순수 데이터 (ROS 파라미터 파일 아님) |
| `config/mpc.yaml` | 노드 배선만 — 토픽, 로깅. 숫자는 없다 |

```bash
python3 -m amr_mpc.core.config     # 지금 어떤 값이 실제로 적재되는지 (경로까지) 확인
```

**왜 이렇게 했나.** 2026-09-03 까지 값이 `model.py` 상수 / `mpc.py` 상수 / `mpc.yaml`
세 곳에 흩어져 있었고 실제로 갈라졌다 — `MPC` 기본값이 `r=(1,1,0.3)` 인데 노드는 `(1,1,1)`
을 넘기고, `|Δu|` 제약은 `mpc_tracking_node` 만 걸어서 자기시험은 **무제약으로 돌았다.**
즉 `run_circle.sh` 로 0.9 m/s 까지 확인한 실차 결과가 운용 구성의 결과가 아니었다.
지금은 `demo()`·자기시험·운용이 전부 같은 값을 쓴다.

`AMR_MPC_PLANT_YAML` 로 다른 파일을 꽂을 수 있다 (파라미터 스윕용).

## 문서

| 파일 | 내용 |
|---|---|
| `docs/signal_flow_servo.html` | 서보 신호 흐름 — 블록마다 펼치기 |
| `docs/signal_flow_globalpath.html` | Theta* 직결 제어 흐름 6단계 |

## 실시간 성능

주기 40 ms 에 `step()` 이 들어가야 한다. **2026-09-04 측정 결과 병목은 알고리즘이 아니라 CPU 경쟁이다.**

8자 sim, 682 틱, `step_ms` 분포:

| 조건 | p50 | p95 | p99 | max | 20 ms 초과 |
|---|---|---|---|---|---|
| 부하 없음 | 1.14 | 1.52 | 1.57 | 3.97 | 0 |
| **CPU 83 % 부하** | **16.61** | 23.52 | **25.88** | 31.60 | **116 틱** |
| **부하 + 코어 고정** | **1.81** | 2.49 | **3.34** | 12.59 | **0** |

부하는 12 코어 중 10 개를 점유시켜 만들었고, 고정은 남은 두 코어에 제어 노드만 올린 것이다.
**코드 한 줄 안 고치고 `p99` 가 25.9 → 3.3 ms (7.7 배).** 느린 틱의 대부분이 `QP 반복 0` 이라
솔버가 아니라 스케줄링이 원인이라는 것도 같이 확인됐다.

```bash
ros2 launch amr_mpc figure8.launch.py mpc_prefix:="taskset -c 10,11"
```

실차는 인지 스택(카메라·YOLO·트래커)이 같이 도므로 **고정을 쓰는 것이 기본값이어야 한다.**
`chrt` 로 실시간 우선순위까지 주면 더 낫지만 `CAP_SYS_NICE` 가 필요하다.

`step()` 자체의 비용 (조용한 기계, 중앙값):

| 항목 | 시간 | 비고 |
|---|---|---|
| `_F_Phi` | 0.45 ms | 증강 상수부 캐싱 후 (1.10 → 0.45) |
| `_constraints` | 0.007 ms | 상수 행 80/100 을 미리 채운다 (0.151 → 0.007) |
| QP | ~0 | 정상 주행에서 682 틱 중 2 회만 반복한다 |
| **`step()` 전체** | **1.1~1.5 ms** | 예산의 3~4 % |

외부 QP 솔버(OSQP·qpOASES·HPIPM)나 C++ 재작성은 **이 표가 가리키지 않는 방향**이다 —
솔버는 거의 돌지 않고, 꼬리는 스케줄링이며, 조용한 기계에서는 이미 예산의 4 % 다.

## 구조 B — 바퀴 각속도 입력 MPC (2026-09-04, 병행)

교수님 지적("출력이 액추에이터의 물리 입력 — 토크/전류/각속도 — 여야 controller") 에 따라 MPC 출력을
몸체속도 `/cmd_vel` 이 아니라 **바퀴 각속도 setpoint `/wheel_cmd` (FL, FR, RL, RR)** 로 내는 판.
기존 구조(A, `body/`)는 그대로 두고 `wheel/` 로 병행한다.

| | A `mpc_tracking_node` | **B `wheel_tracking_node`** |
|---|---|---|
| 출력 | `/cmd_vel` (3) → 드라이버가 `T⁻¹` | `/wheel_cmd` (4) → 드라이버는 한계·반전만 |
| 상태 `x_m` | `[X_e; v_e; z]` 9 | `[X_e; ω_e; z]` 11 (증강 14) |
| actuator 모델 | 축별 FOPDT ×3 (`plant.yaml`) | **바퀴별 FOPDT ×4** (`plant_wheel.yaml`: τ 101/98/102/93 ms, K 1.01, θ 40) |
| kinematics | 드라이버 안 | **MPC 소유** — `B_k = T_s R(θ_r) T`, `T` = pinv(`T⁻¹`) |
| 제약 | L1 마름모 (바퀴 한계의 대리) 40행 | 바퀴별 `\|ω_i\| ≤ 30` box 40행 + `\|Δω_i\| ≤ 4.5` 40행 — **정확한 하드웨어 한계** |
| 상태 측정 | odom twist | `/measured_wheel_speeds` (encoder) + odom pose |
| 내부 자유도 | — | 4−3 = 1 (null 방향 `[1,1,−1,−1]/2`, 출력에 안 보임, `a′` 로 감쇠 → detectable) |

| 파일 | 내용 |
|---|---|
| `config/plant_wheel.yaml` | 바퀴별 τ·K·θ, 기하, 바퀴 한계, `r_dw = r_du·r² = 0.0016` |
| `wheel/model.py` | `T⁻¹`, `T`, 바퀴 FOPDT ZOH, `plant()` 11차, `augment()` 14차. `python3 -m amr_mpc.wheel.model` 자기검사 6항 |
| `wheel/mpc.py` | `WheelMPC.step(X, ω_meas, ω_prev, ref, Xr, ur) → ω_cmd`. box+rate 제약 상수부 미리 조립. `python3 -m amr_mpc.wheel.mpc` 데모 |
| `nodes/wheel_tracking.py` | ROS 배선. `/cmd_vel` 발행자가 있으면 ERROR (명령 경쟁) |
| `nodes/sim_plant_wheel.py` | 바퀴별 FOPDT + `T` 시뮬 플랜트 → `/odom`, `/measured_wheel_speeds` |
| `launch/wheel_circle.launch.py` | `sim:=true` 로봇 없이 / `sim:=false` 실차 |
| `amr_base/roboclaw_mecanum_node.py` | `wheel_cmd` 구독 추가 (`cmd_vel` 경로 그대로, 같은 워치독) |

```bash
ros2 launch amr_mpc wheel_circle.launch.py sim:=true speed:=0.5 duration:=20.0
```

폐루프 sim (R 1 m, 0.5 m/s, 20 s): 정상상태 위치오차 RMS **0.07 cm** (A 는 0.06), 램프 구간 최대 2.5 cm,
헤딩 최대 1.1°, 바퀴 부하 최대 57 %, QP 활성 0, step 평균 1.3 / 최대 3.2 ms. 순수 파이썬 데모(초기오차 15 cm):
1 cm 진입 1.32 s, 바퀴 최대 26/30 rad/s.

`z` 는 여전히 있다 — lower loop(RoboClaw PI + 통신) 지연 40 ms 를 상태로 실현한 것이고, 토크/전류 입력으로
내려가면 사라진다. 그때 바뀌는 건 `wheel/model.py` 의 actuator 블록뿐이다.

## 시간축 보간 (`reference_horizon.py`)

`references[i]` 의 목표 시각은 `header.stamp + (i+1)·dt` (README 규약). MPC 는 매 틱 "지금 + i·Ts" (i = 0..Np−1) 의
기준을 요구하므로:

| 구간 | 처리 |
|---|---|
| 두 샘플 사이 | 포즈·속도 선형 보간, yaw 는 unwrap 후 보간 |
| 첫 샘플 이전 (stamp ~ stamp+dt) | 첫 샘플에서 그 몸체속도로 뒤로 외삽 — 플래너가 10 Hz 라 "지금" 은 거의 항상 이 구간 |
| 마지막 샘플 이후 | 끝 포즈 유지, 속도 0 |

DRA-MPPI 의 롤아웃(`mecanum_model.rollout`)은 dt=0.2 s 오일러라 샘플 사이가 원래 직선이다 — 선형 보간이 플래너 궤적을 정확히 재현한다.
`fake_reference_node` 의 정확한 원에 대해서는 현(chord) 오차 1.25 mm 이하 (R=1 m, 0.5 m/s; 테스트로 고정) —
시뮬 정상상태 RMS 0.11 cm 가 바로 이 값이다.

## 안전 동작

- 기준열이 `ref_timeout`(0.5 s) 동안 안 오거나 odom 이 `odom_timeout`(0.3 s) 동안 없으면 매 틱 0 을 낸다.
- 실제 베이스는 네 엔코더의 마지막 정상 측정이 `feedback_timeout_s`(0.25 s)를 넘으면
  명령을 거부하고 측정 속도·오도메트리·joint state 발행을 멈춘다. 따라서 보존한 마지막
  값이 MPC의 유효시간을 계속 갱신하지 않는다. 짧은 읽기 실패만 이 시간 안에서 허용한다.
  RoboClaw 정지 전송의 ACK가 확인되지 않으면 이후 update에서 두 보드에 정지를 재시도한다.
- 재개 시 MPC 내부(Δx_m, Hildreth λ)와 `u_prev` 를 리셋한다.
- 기준 부하 `|vx|+|vy|+0.198|ω|` 가 `load_budget·V_LIMIT` 를 넘으면 경고. **거부는 하지 않는다** — 플래너가 지켜야 한다.
  DRA-MPPI 의 `limits.control_max = [0.94, 0.56, 3.0]` 은 축별 상자라 세 축을 동시에 쓰면 L1 부하 2.1 m/s 로
  `V_LIMIT = 1.2` 를 넘는다. 플래너 한계를 L1 다면체 안으로 두거나(예: `[0.6, 0.3, 1.5]`), MPC 의 입력 제약이 잘라내는 것을 감수해야 한다.

## 8자 경로 (`figure8.py` / `figure8_reference_node`)

가로세로 2 m 공간용. 기동 포즈가 8자의 교차점이고, 한 바퀴 뒤 **위치·헤딩 모두** 그 포즈로 돌아와 멈춘다.

```text
p(s) = [ (W/2)·sin s ,  (H/2)·sin 2s ]        s ∈ [0, 2π)      Gerono 렘니스케이트
```

* **경계상자가 정확히 W × H**, 기동 포즈가 그 중심 — 방 크기 검토가 바로 된다.
  로봇 외접반경 0.18 m (베이스 0.2858 × 0.1884) 이므로 2 m 방이면 W ≤ 1.44.
* **회전지수 0** — 두 잎의 회전이 상쇄돼 한 바퀴 뒤 접선각이 제자리다 (원은 회전지수 1).
  그래서 헤딩까지 복귀한다.  `test_figure8.py` 가 수치로 확인한다.
* **κ 가 교차점에서 부호를 바꾸며 0 을 지난다** — 원 두 개를 접해 붙인 8자는 여기서 κ 가
  +1/R → −1/R 로 뛰어 ω 계단이 생긴다.  Gerono 는 C^∞ 라 그 계단이 없다.
* `s` 는 호길이가 아니다. 누적 사다리꼴로 σ(s) 표를 만들고 역보간해 **등속 재매개화**하고,
  σ(t) 는 사다리꼴 속도 프로파일(0→V, 순항, V→0)로 준다.  호길이가 짧으면 삼각형으로 자동 강등.

`height` 가 곡률을 정한다 (잎사귀 꼭대기 κ ≈ 16H/W²).  폐루프 시뮬 (10 Hz 발행 → 보간 → MPC
→ FOPDT 플랜트, W 1.4, v 0.25 m/s, 초기오차 10 cm):

| H [m] | κ_max [1/m] | ω_max [rad/s] | 위치오차 최대 | 헤딩오차 최대 | 기준 부하 | 한 바퀴 |
|---|---|---|---|---|---|---|
| 1.0 | 8.93 | 2.23 | 1.32 cm | 12.5 ° | 67 % | 24.5 s |
| **0.8** | **7.50** | **1.88** | **1.31 cm** | **9.8 °** | **59 %** | **21.9 s** |
| 0.6 | 6.25 | 1.56 | 1.29 cm | 7.7 ° | 53 % | 19.5 s |
| 0.4 | 5.72 | 1.43 | 1.28 cm | 6.0 ° | 49 % | 17.4 s |

위치오차는 H 에 거의 무관하고 **헤딩오차만 곡률을 따라간다** — `q_yaw` 0.5 가 `q_pos` 6.0 의
1/12 이라 급회전에서 헤딩이 뒤처진다.  기본값 0.8 은 잎사귀가 0.7 × 0.8 로 가장 둥근 지점.

`yaw_mode`

| | 헤딩 | 정렬 회전 | 부하 (v 0.25) | 헤딩오차 | 쓸 곳 |
|---|---|---|---|---|---|
| `tangent` (기본) | 접선 추종 | 앞뒤 `align_s` 초 제자리 회전 (θ(0)=48.8°) | 59 % | 최대 9.8 ° | 곡률·회전까지 거는 표준 시험 |
| `fixed` | 고정 (게걸음) | 없음 | 36 % | 0.03 ° | 메카넘 횡이동, 위치 추종만 보고 싶을 때 |

기준 부하 `|vx|+|vy|+K_GEOM·|ω|` 가 `load_budget`(0.65)·V_LIMIT 를 넘으면 노드가 뜨지 않고
낮출 속도를 알려준다 — `run_circle.sh` 의 예산 검사와 같은 규칙이다.

### 상태 원천은 `/odom` 이다 — EKF 를 끼우지 말 것

`figure8.launch.py` 의 `odom_topic` 기본값은 **`/odom`** 이다. `circle.launch.py`·`run_circle.sh`
와 같고, 이유가 있다: `model.py` 의 FOPDT(τ, K, **θ=40 ms**)는 `sys_id/wp2_step_logger.py` 가
`ODOM_TOPIC = "/odom"` 으로 식별한 값이고, `mpc.yaml` 이 `Ts=0.04` 를 쓰는 것도 그래야 `d=1`
(정확히 한 스텝 지연)이 되기 때문이다.

`/odometry/filtered` 를 물리면 그 사이에 robot_localization EKF 가 한 단 더 들어간다
(50 Hz, `process_noise_covariance` 미튜닝, yaw 는 절대기준 없이 자이로 vyaw 적분만).
그 추가 지연은 예측모델에 없으므로 MPC 가 과보정한다.

**2026-09-03 실차 발산 — 실측으로 확정.** `odom_topic:=/odometry/filtered` + 전체 브링업으로
8자를 돌렸더니 위치오차 RMS 43 → 115 cm, 최대 2.8 m 로 달아났다. `tools/diag_fig8.py` 의 ⑦:

```
   t[s]    오차cm    부하%  |u|vx  |v|vx  |u|wz  |v|wz    QP
   0-2      0.0     1   0.00   0.00   0.07   0.00      0     ← 기준 정지, 정상
   2-4      1.5   100   0.07   0.03   6.05   0.11    800     ← 정렬 회전 시작 = 즉시 포화
```

위치오차 1.5 cm 인데 MPC 가 최대 회전 6.05 rad/s(= V_LIMIT/K_GEOM)를 명령하고 실측은 0.11 이다.
**로봇이 명령을 실행하지 못하고 있었다.** 제자리 회전 시험(±0.5 rad/s 계단, `/cmd_vel` 대비
상호상관)으로 원인을 갈랐다:

| 신호 | joy_teleop 켬 | joy_teleop 끔 |
|---|---|---|
| `/odom` wz | 이득 0.83, 지연 120 ms | **이득 0.97**, 120 ms |
| `/odometry/filtered` wz | 이득 0.74, 지연 480 ms | 이득 0.93, **180 ms** |
| `/imu/base_link` wz | 이득 0.75, 지연 420 ms | 이득 0.95, **100 ms** |

**주원인은 `/cmd_vel` 경쟁이다.** `joy_teleop` 은 조이스틱이 없어도 0 을 20 Hz 로 계속 쏜다
(`bringup.launch.py`·`amr_system.launch.py` 의 `use_joystick` 기본값이 `true`). MPC 는 25 Hz 라
드라이버가 받는 명령의 절반 가까이가 0 이 된다. 로봇은 명령의 일부만 실행하고, MPC 는
"반응이 없다"로 읽어 적분기가 감기며 포화 → 발산한다.

IMU 는 멀쩡했다 — 480/420 ms 는 전부 경쟁이 만든 허상이고, 끄면 100~180 ms 로 돌아온다.
`mpc_tracking_node` 가 이제 `cmd_vel` 발행자 수를 세어 2 개 이상이면 ERROR 를 낸다.

**부차 원인은 `/odometry/filtered` 의 추가 지연 60 ms** (180 vs 120 ms). `model.py` 의 FOPDT
(θ=40 ms, d=1)는 `sys_id/wp2_step_logger.py` 가 `/odom` 으로 식별한 값이라 이 60 ms 는 모델에
없다. 시뮬에서 추가 지연 80 ms 면 직선·원까지 부하 100 %·QP 800 으로 간다:

| 기준 | 지연 0 ms | 80 ms | 120 ms |
|---|---|---|---|
| 직선 0.15 m/s | 2.0 cm, 부하 17 %, QP 0 | 3.8 cm, 48 %, QP 20 | 8.3 cm, 70 %, QP 40 |
| 원 R=1 m | 2.0 cm, 부하 20 %, QP 0 | 4.6 cm, 86 %, QP 80 | 10.4 cm, **100 %, QP 800** |
| 8자 1.4×0.8 | 1.3 cm, 부하 59 %, QP 0 | 5.1 cm, **100 %**, QP 640 | 11.0 cm, **100 %, QP 800** |

그래서 `figure8.launch.py` 기본값은 `/odom` 이다. 여유가 없는 쪽을 굳이 고를 이유가 없다.

**MPC 를 돌릴 때 지켜야 할 것 둘**
1. `/cmd_vel` 발행자는 **하나여야 한다.** `use_joystick:=false` 로 띄우거나
   `keyboard_command_gate` 를 써라 (`autonomy.launch.py` 가 그렇게 한다 — MPC 는
   `/mpc/cmd_vel_raw` 로 내고 게이트만 `/cmd_vel` 을 소유한다).
2. 상태 원천은 `/odom`. EKF 를 써야 하면 `tools/measure_odom_lag.py` 로 지연을 먼저 재라.

지연은 재서 확인한다 — `tools/measure_odom_lag.py` 가 `/cmd_vel` 대비 각 odom 토픽의
지연·이득을 상호상관으로 낸다. `/odom` 이 40~60 ms 여야 정상이고, `/odometry/filtered` 가
그보다 80 ms 이상 크면 MPC 를 거기 물리면 안 된다.

EKF 를 꼭 써야 하면 `model.py` 의 `THETA` 를 실측 지연으로 다시 잡고 `Ts`·`d` 를 맞춰야 한다
(그러면 `mpc.yaml` 의 `Ts=0.04` 전제가 깨지므로 재튜닝이 따라온다).

### 비상정지

`abort_error`(기본 0.35 m)만큼 기준에서 벌어지면 기준 발행을 끊는다 → MPC 가 `ref_timeout`
뒤 `/cmd_vel` 0. 위 발산에서 로봇이 2.8 m 를 달아났기 때문에 넣었다. 0 이하로 주면 끈다.

### 진단

```bash
python3 tools/diag_fig8.py fig8_real_YYYYmmdd_HHMMSS.csv
```
①기준열이 8자였나 ②MPC 가 따라갔나 ③명령이 포화했나 ④기준열이 끊겼나
⑤명령 대비 실측속도(이득·지연) ⑥폐합 — 순서대로 갈라준다. 마지막에 기준/실주행을 겹쳐 그린다.

```bash
ros2 launch amr_mpc figure8.launch.py sim:=true             # 로봇 없이 (sim_plant_node)
ros2 launch amr_mpc figure8.launch.py                       # 실차. 드라이버 + EKF 선행
ros2 launch amr_mpc figure8.launch.py height:=0.6 speed:=0.2 yaw_mode:=fixed laps:=2.0
python3 -m amr_mpc.core.figure8                                  # 궤적 자기검사 (닫힘·유한차분·부하)
```

RViz 는 `/figure8/path` (전체 경로, latched) 와 `/mpc/reference_horizon` (MPC 가 보는 0.8 s 지평)
을 보면 된다.
