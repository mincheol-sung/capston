# 2026-11-14 NUC — 시뮬 확인 + 체크리스트 0~2단계 (벤치)

- 장소·머신: 로봇 NUC (`smoc-NUC`), 커밋 `4c4ffff` (리뷰 R8~R10), 복사 빌드(`colcon build --symlink-install` 이지만 NUC 의 setuptools 83 때문에 ament_python 은 복사 설치)
- 모터: AK40-10 × 4, CANable2(slcan, `can0` 1 Mbps). ID 1 FL · 2 FR · 3 RL · 4 RR. **네 바퀴 모두 띄운 상태**
- 벤치 도구: `src/amr_basic_stack/controller/mpc/tools/ak40_bench.py` (ROS 없이)
- 모터 온도: 시작 21~22 °C → 끝 30~32 °C

## 1. NUC 빌드 시뮬 확인 (`torque_experiment.launch.py use_rviz:=false`, `tools/compare_run.py`)

| 비교군 | rms_mm | max_mm | moving_rms_mm | fault | PC 기준 |
|---|---|---|---|---|---|
| B (토크 MPC) | **12.16** | 35.9 | 13.71 | 없음 | 12.8 / 최대 35 |
| A (퓨어 퍼슛 + PI) | **5.34** | 15.5 | 6.35 | 없음 | 5.1~6.3 |

→ 코드가 NUC 에 제대로 옮겨졌다. 두 판 모두 31.3 s 에 완주, 종료 때 9개 노드 `finished cleanly`.

## 2. 발견 — `~/.bashrc` 의 `ROS_STATIC_PEERS` 가 로컬 토픽을 수 초씩 밀어낸다

NUC `~/.bashrc` 136행: `ROS_STATIC_PEERS="192.168.0.206;192.168.0.56;192.168.0.213"` (CycloneDDS, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`). 이날 206·56 은 ping 무응답.

같은 시뮬(도메인 88)에서 `ros2 topic hz /imu/base_link`:

| DDS 설정 | IMU 주기 | 최대 간격 | 추정기 경고 |
|---|---|---|---|
| 사용자 터미널과 같음 (Cyclone + LOCALHOST + URI + static peers) | **11.4 Hz** | 0.997 s | "IMU stamp +2.85 s 어긋남", "SLAM 포즈 2.9 s 반영 안 됨" |
| Fast DDS | 200.0 Hz | 6 ms | 0 |
| Cyclone 만 | 200.0 Hz | 6 ms | 0 |
| Cyclone + LOCALHOST + URI, **peers 만 뺌** | 200.0 Hz | 7 ms | 0 |

증상: 추정기 출력이 3 s 마다 끊김 → fault_monitor `state_timeout` → 무장 해제 → 8자가 안 끝남.
원인은 static peers 한 줄로 좁혀졌다(무응답 peer 로 가는 unicast 가 송신을 붙잡는다는 메커니즘은 추정).
**대응: 터미널마다 `unset ROS_STATIC_PEERS`.** `.bashrc` 는 공용 설정이라 고치지 않았다(팀 결정 필요). 실차 스택도 같은 환경으로 뜨므로 4단계 전에 필수.
같은 이유로 NUC 는 LOCALHOST 범위라, PC(192.168.0.209)의 RViz 로 NUC 토픽을 보려면 PC IP 가 peers 에 있어야 한다.

## 3. 0·1단계 — 팀원 벤치 기록으로 확인

| 항목 | 근거 | 판정 |
|---|---|---|
| XT30 극성, 입력 16~28 V | 팀원 2026-11-11 벤치에서 네 모터 동작 | 확인(간접) |
| CAN 종단 약 60 Ω | 직접 측정 안 함. FL·RR 111 s 동안 11100/11100 응답 | 간접. 3단계에서 늦은 응답이 늘면 직접 잰다 |
| MIT 모드, ID 1~4 | 팀원 벤치 + 이날 status | 확인 |
| CAN sudden interrupt 설정 | 아래 4절 시험 | **설정 안 됨** |

## 4. CAN 끊김 시험 — 모터는 명령이 끊겨도 마지막 토크를 계속 낸다

```bash
(/usr/bin/python3 tools/ak40_bench.py torque --id 1 --tau 0.05 --time 10 & P=$!; sleep 3; kill -9 $P; sleep 5; /usr/bin/python3 tools/ak40_bench.py status --id 1)
```

| 시점 | FL 응답 |
|---|---|
| 0 s | v −0.08, t −0.0012 (정지) |
| 3 s | `kill -9` — 이후 명령 없음 |
| **8 s (status 진입 응답)** | **v 11.59 rad/s, t 0.0525 N·m, error 0** |
| 8 s 이후 | status 의 DAMPING 으로 정지 (v 0.01 → −0.08) |

→ 공장값에는 끊김 보호가 없다(`ak40/bridge.py` 의 가정과 같다). `kill -9`·NUC 멈춤·USB-CAN 빠짐 때 마지막 토크가 계속 나간다.
CubeMars 매뉴얼 V1.0.17 4.1.1.4 "Application Function" 에 "CAN sudden interrupt" 설정이 있다(AK40-10 포함, 상위 프로그램은 Windows 로 보임, 쓰기 전에 Read Parameters 먼저 — 4.1.1.5).
**네 모터 설정 전에는 3단계 이후(노드로 토크)를 하지 않는다.** 설정 뒤 같은 시험에서 8 s 응답 t 가 ±0.0012 근처면 확인.

## 5. 2단계 status — 네 모터 응답, 12 비트 양자화 확인

네 모터 모두 응답, error 0. 정지 중 `v`·`t` 값은 12 비트 양자화와 맞다.

| 값 | 범위 | 한 칸 | "0" 표시 | 관측 |
|---|---|---|---|---|
| v | ±45.5 rad/s | 91/4095 = 0.0222 | ±0.0111 | 0.0111 의 홀수 배 |
| t | ±5 N·m | 10/4095 = 2.44 mN·m | ±1.22 mN·m | 0.0012 의 홀수 배 |

DAMPING 중 기어감: 없음. damping 시험의 DAMPING 3 s 동안 v 평균 −0.012~−0.016 rad/s, 위치 변화 ≤ 0.0004 rad.
0 명령의 +1.22 mN·m 가 쿨롱 마찰(약 20 mN·m)보다 훨씬 작아서다(시뮬 가짜 모터는 0.056 rad/s 로 기어갔다).

## 6. 2단계 damping — `damping --id N --spin 20` (`damping_N.csv`)

| 모터 | 시작 v | 5 % 까지 정지 시간 | 응답 τ/v 로 본 실제 kd | v 궤적으로 맞춘 J | 팀원 J |
|---|---|---|---|---|---|
| FL | 19.5 | 0.092 s | 0.0180 | 7.9e-4 | 8.65e-4 |
| FR | 19.6 | 0.102 s | 0.0186 | 8.9e-4 | 8.89e-4 |
| RL | 20.1 | 0.092 s | 0.0178 | 7.7e-4 | 9.18e-4 |
| RR | 19.4 | 0.101 s | 0.0189 | 8.0e-4 | 8.76e-4 |

- 예측: J/(kd+bv) = 0.041 s, 19.9 → 1 rad/s 까지 (J/a)·ln((a·19.9+τc)/(a·1.0+τc)) = **0.091 s** (a = kd+bv, 팀원 FL 값). 관측 0.092~0.102 s
- 도구가 출력하는 J ≈ kd·(정지 시간/3) 는 마찰을 빼서 약 30 % 작게 나온다(6.1~6.8e-4) — 쓰지 않는다
- kd 0.02 는 12 비트(KD 0~5, 한 칸 0.00122)로 0.0195 가 들어간다. 응답으로는 0.018~0.019
- **판단: kd 0.02 유지.** 바닥 직진 추정(m 4 kg 추정값, r 0.04 m, 바퀴당 등가 관성 J + m r²/4 = 0.00247): 시정수 0.12 s, 0.5 m/s 에서 0.28 s·4.5 cm 에 정지. 단 초기 감속 4.6 m/s² > μ 0.3·g = 2.9 m/s² → 미끄러운 바닥이면 정지 첫 순간 슬립 가능(5단계에서 확인)
- spin 단계(MIT kd 0.5, v_des 20)의 응답 |τ| 최대 0.28~0.40 N·m. spin 은 `--max_tau` 를 거치지 않는다(`cmd()` 가 t_ff 만 자른다) — 도구 docstring "모든 명령은 --max_tau 안에서" 와 다르다. 10 ms 표본 사이 순간값은 이 데이터로 모른다

## 7. 2단계 마찰 계단 — `steps` (`steps_N.csv` 오름, `stepsdown_N.csv` 내림)

### 정지 마찰 (오름 0.02 → 0.04 → 0.06 → 0.1, 각 3 s, 29 °C)

| 모터 | 처음 돈 토크 |
|---|---|
| FL | 0.1 (0.06 에서 안 돎. 단 4절 시험에서는 0.05 로 출발 — 멈춘 각도에 따라 다르다) |
| FR | 0.06 |
| RL | 0.1 |
| RR | 0.04 |

운동 마찰 τc(약 0.02)의 2~5 배. `torque_output` 의 보상 τc·tanh(ω/0.5) + bv·ω 는 ω = 0 에서 0 이라 정지 마찰은 보상하지 않는다 → 5단계 저속 출발에서 확인.

### 운동 마찰 (내림 0.1 → 0.06 → 0.04 → 0.03 → 0.02, 각 3 s, 30~32 °C)

| 모터 | 0.1 | 0.06 | 0.04 | 0.03 | 0.02 |
|---|---|---|---|---|---|
| FL ω [rad/s] | 36.48 | 20.03 | 9.00 | 3.18 | 0 |
| FR | 40.93 | 22.47 | 9.51 | 3.31 | 0 |
| RL | 41.36 | 21.33 | 9.86 | 3.97 | 0 |
| RR | 41.61 | 24.55 | 13.50 | 6.50 | 0 |

계단 마지막 1 s 평균의 (ω, 응답 τ) 4점으로 τ = τc + bv·ω:

| 모터 | τc [N·m] | bv [N·m·s/rad] | 잔차 최대 | 지금 `torque_plant.yaml` (팀원) |
|---|---|---|---|---|
| FL | 0.0214 | 0.00206 | 3.4 mN·m | 0.0280 / 0.00113 |
| FR | 0.0215 | 0.00180 | 3.4 mN·m | 0.0279 / 0.00235 |
| RL | 0.0217 | 0.00177 | 0.9 mN·m | 0.0239 / 0.00221 |
| RR | 0.0183 | 0.00168 | 1.4 mN·m | 0.0253 / 0.00080 |

- 잔차 ≤ 응답 τ 1.4 칸 → 직선 모델로 충분
- 지금 값으로 보상하면(바퀴에 실제로 가는 토크의 오차): 38.7 rad/s 에서 FL 0.072 vs 측정 0.101, RR 0.056 vs 0.083 (약 30 % 부족), FR 0.119 vs 0.091, RL 0.109 vs 0.090 (과보상). 12.5 rad/s 에서는 ±0.013 이내
- **마찰은 돌릴수록 줄었다(워밍업)**: 같은 토크에서 오름(29 °C)보다 내림(30~32 °C)이 빠르다. FR 0.06 N·m → 15.4 vs 22.5 rad/s. FL 4절 점(26 °C, 11.6 rad/s 에서 0.0525)은 내림 직선보다 8 mN·m 높다. 20 rad/s 에서 약 15 % 차이
- 팀원 값은 10 rad/s 이하 자료라 그 위는 외삽이었다
- **값 반영은 PC 쪽에서 결정**(이 커밋에서는 설정을 바꾸지 않았다). 과보상은 정지 근처 떨림을 부르므로 측정값보다 약간 작게 넣는 것이 관례

### 무부하 최고 속도 (따로 `torque` 시험 안 함)

0.1 N·m 에서 RR 41.6 rad/s, 응답 τ 0.0888(명령보다 11 mN·m 낮음). FR·RL 도 응답 τ 가 명령보다 3~5 mN·m 낮고 끝 1 s 에 아직 +2~4 rad/s² 가속.
→ 약 41~42 rad/s 에서 전압 한계로 전류가 더 안 들어간다(이날 배터리 전압, 무부하). 응답 범위 45.5 아래라 잘리지 않음.
운용 상한 38.7 rad/s(370 rpm, `max_wheel_rad_per_sec`)와 여유 약 7 % — 바닥 하중·배터리 저하에서 더 줄어든다.

## 8. 3단계 — CAN 노드(`ak40_can_node`), 네 바퀴 띄움, ±0.1 N·m 묶음

끊김 보호 설정 전이라 바퀴를 띄운 채 `hardware_checked: false`(±0.1 N·m) 로만 진행하기로 함(사용자 결정).

### 장착 부호

```bash
export ROS_DOMAIN_ID=81 && unset ROS_STATIC_PEERS && ros2 run amr_mpc ak40_can_node                       # 터미널 1
export ROS_DOMAIN_ID=81 && unset ROS_STATIC_PEERS && ros2 topic echo /wheel_states --field velocity      # 터미널 2
export ROS_DOMAIN_ID=81 && unset ROS_STATIC_PEERS && ros2 topic pub -r 50 /wheel_torque_cmd amr_interfaces/msg/WheelTorqueCommand "{mode: 0, torque: [0.08, 0.08, 0.08, 0.08]}"   # 터미널 3
```

- 처음에는 바퀴가 안 돌았다: 터미널 1·2 는 시뮬 때 export 한 도메인 81, 터미널 3 은 `.bashrc` 기본 13 이라 명령이 노드에 안 닿음. 모두 81 로 맞춰 해결
- 0.05 N·m 대신 0.08 N·m: 7절 정지 마찰(FL·RL 은 0.06 에서도 출발 못 함)
- 결과: **네 바퀴 모두 눈으로 전진 방향** → `direction: [-1, 1, -1, 1]` 맞음. velocity(정상 구간 평균 근처) FL 35.5 · FR 28.3 · RL 31.6 · RR 33.9 rad/s
- velocity 부호로는 판정할 수 없다: 노드가 명령에 direction 을 곱하고(τ_motor = d·τ_wheel) 응답에도 곱하므로(ω_wheel = d·ω_motor) ω_wheel 부호 = d²·sign(τ_wheel) = + 로 direction 과 무관
- 같은 0.08 N·m 에서 7절 내림 직선의 예측 ω = (0.08 − τc)/bv 는 FL 28.4 · FR 32.5 · RL 32.9 · RR 36.7 → 관측과 FL +25 %, FR −13 %. 마찰이 시간·온도로 계속 변한다는 7절 관찰과 같은 방향

### 100 Hz·응답 시간 (DAMPING, 토크 없음, 11 s)

| 항목 | 값 |
|---|---|
| `/wheel_states` 주기 (`ros2 topic hz -w 1000`) | 100.002 Hz, 간격 표준편차 0.05 ms |
| 틱 시간 (`/ak40_can/status` 5번째) | 평균 0.79 · 중앙 0.75 · 95 % 0.96 · 최대 1.34 ms |
| 응답 수 (4번째) | 1117 주기 모두 4 |
| 누적 누락 (6번째) | 2 → 2 (기동 때 2 개뿐) |
| 누적 늦은 응답 (7번째) | 175 → 2407, **주기마다 정확히 2** |

`candump -t A can0` 1 s (`candump_damping_1s.txt`, 노드는 4 프레임을 한꺼번에 보낸다)로 잰 명령 송신 → 응답 도착:

| 바퀴 | 평균 | 최대 |
|---|---|---|
| FL | 0.60 ms | 0.69 |
| FR | 0.65 ms | 0.71 |
| RL | 1.13 ms | 1.21 |
| RR | 1.17 ms | 1.22 |

→ 버스는 `reply_wait` 4 ms 안에 다 온다. 늦은 응답 2 개/주기는 노드 로직 때문이다: `on_tick` 이 먼저 `take_pending()` 으로 지난 주기 늦게 온 RL·RR 응답을 반영하면서 `fresh` 로 세고, 송신 뒤 `collect()` 는 `all_replied()` 가 참이 되는 순간(FL·FR 새 응답 0.6 ms) 끝난다(틱 0.79 ms 와 일치). RL·RR 의 새 응답(1.1 ms)은 수신 버퍼에 남아 다음 주기 pending 이 된다. 기동 때 한 번 생기면 계속 유지된다.
결과: **RL·RR 바퀴 상태는 늘 한 주기(10 ms) 늦은 값**. `reply_wait` 를 늘려도 collect 가 그 전에 끝나므로 고쳐지지 않는다.

**같은 날 NUC 에서 고침** (`stack_changelog.md` 2026-11-14 (4)): collect 의 끝 조건을 "이번 주기 송신 뒤 받은 응답 4 개"(`cycle_rx`)로. 고친 노드로 같은 측정(DAMPING 11 s, `candump_damping_1s_fixed.txt`):

| 항목 | 고치기 전 | 고친 뒤 |
|---|---|---|
| 누적 늦은 응답 | 175 → 2407 (주기마다 +2) | **4 → 4** |
| 틱 평균 / 최대 | 0.79 / 1.34 ms | 1.24 / 3.11 ms (네 응답을 다 기다림) |
| `/wheel_states` | 100.002 Hz, 표준편차 0.05 ms | 99.999 Hz, 표준편차 0.10 ms (간격 8~12 ms) |
| 응답 4/4 | 100 % | 100 % |
| 응답 도착 FL/FR/RL/RR | 0.60 / 0.65 / 1.13 / 1.17 ms | 0.49 / 0.61 / 0.99 / 1.12 ms |

### 종료 신호 (SIGTERM)

0.08 N·m 로 네 바퀴가 도는 중 `pkill -TERM -f ak40_can_node` → 바퀴가 "틱" 하고 제동된 뒤 섰다. 터미널 1:
```
[WARN] [1794638113.512803841] [ak40_can_node]: 응답 3/4 — 빠진 바퀴 [3]     ← 명령(ros2 topic pub)을 켠 순간
Terminated                                                              ← pkill -f 가 ros2 run 껍데기도 죽임
[ak40_can_node] DAMPING 뒤 모터 모드 해제                                 ← 노드 본체의 정지 절차 (print, 로그 파일엔 없음)
```
"CAN 송신 실패" 없음. `응답 3/4` 한 주기 누락은 다른 ROS 프로세스가 켜지고 꺼질 때마다 한 번씩 나왔다(이날 4 회, 모두 단발 — error 255 기준 50 ms 에 못 미침).

### 모터 하나 전원 껐다 켜기 (노드 켠 채, 바퀴 정지 상태)

기록: `powercycle_wheel_states.csv` (읽기 전용 기록기, `/wheel_states` error·fresh·velocity 와 `/ak40_can/status`).

| 시각 (epoch …) | 일 |
|---|---|
| 364.026 | RR 응답 끊김 (전원 뺌) |
| 364.067 | RR error 255 (41 ms, 기준 reply_timeout 50 ms) — 나머지 셋 정상 |
| 373.586 ~ 373.636 | RR 을 다시 꽂은 순간 FL·FR·RL 도 50 ms 안에 응답 끊김 → 네 바퀴 error 255 |
| 373.6 ~ 406 | 노드가 0.2 s 마다 ENTER 를 다시 보냈지만 33 s 동안 응답 없음 (Ctrl+C 로 끔) |

그 뒤 모터 LED 네 개 모두 켜짐, `ak40_bench.py status --id 1` 도 응답 없음 → `sudo ip link set can0 down; sudo pkill slcand; bash tools/setup_can.sh` 로 CANable2 채널을 닫았다 열자 네 모터 모두 응답(error 0, 38 °C).
→ 원인은 모터가 아니라 **CANable2 bus-off** (RR 쪽 CAN 트랜시버가 켜질 때의 잡음으로 추정). slcan 펌웨어는 bus-off 를 리눅스에 알리지 않고(`can0` UP·ERROR-ACTIVE, 오류 카운터 0) 스스로 풀지도 않는다. USB 재열거 흔적은 없었다.
이날 맨 처음 status 에서 네 모터가 모두 "응답 없음" 이었던 것(그 뒤 어떤 조작 후 회복)도 같은 상태였을 수 있다 — 확인 안 됨.
도는 중에 빼는 시험은 손 위험으로 하지 않았다.

## 9. 4단계 — 센서·측위·전체 스택 (책상 위, 바퀴 띄움, 무장 안 함)

세 터미널 모두 `export ROS_DOMAIN_ID=81 && unset ROS_STATIC_PEERS &&` 뒤에 `sensors.launch.py` / `cartographer.launch.py mode:=localization map:=smoc_lab lua_basename:=localization_torque_2d.lua` / `torque_experiment.launch.py plant:=real use_rviz:=false`. ak40_can_node 는 늦은 응답 고친 판.

| 항목 | 결과 |
|---|---|
| CAN 자동 준비 | `CAN setup: can0 already UP` (sudo 없이) |
| 노드 | 22 개, 모두 하나씩 (imu_to_base·robot_state_publisher 중복 없음) |
| `/imu/base_link` QoS | 발행 imu_to_base_node BEST_EFFORT → 구독 state_estimator_node·ekf_carto BEST_EFFORT |
| TF base_footprint → camera_imu_optical_frame | (0.098, 0.017, 0.140) m, RPY (−90°, 0, −90°) — 추정기 IMU 기준점과 같음 |
| 주기 (6 s, BEST_EFFORT 구독) | `/camera/imu` 200.6 Hz · `/imu/base_link` 200.6 Hz (두 배 아님, 수신−stamp 0.8 ms) · `/wheel_states` 100.2 Hz · `/scan_rear` 10.2 Hz |
| **`/scan_front`, `/scan`, `/tracked_pose`, `/state_estimator/odom`** | **0 개** |
| 추정기 | 정지 바이어스 2 s 안에 잡힘. IMU stamp 경고 없음(170 s). "SLAM 포즈 0 개" 오류 계속 → "준비 끝" 안 나옴 |

다시 띄워 측정 (같은 조건, 10 s):

| 항목 | 결과 |
|---|---|
| 노드·QoS·TF·CAN 준비 | 위와 같음 (22 개 중복 없음, BEST_EFFORT, (0.098, 0.017, 0.140), already UP) |
| `/camera/imu` 가속도 갱신 | 200.4 Hz 중 **가속도는 2 표본마다 바뀜(100.2 Hz)**, 자이로는 매 표본. 같은 가속도 값이 2 개 이어짐 970 회, 1·3·4 개 33 회 |
| `/imu/base_link` 정지 잡음 (n 2001) | ax σ 0.0156 · ay σ 0.0168 m/s², wz σ 0.0031 rad/s. 평균 ax −0.222 m/s²(받침 기울기 asin(0.222/9.81) ≈ 1.3°), wz −0.0017 rad/s |
| `/wheel_states` (1000 개) | error ≠ 0 행 0, fresh 4/4 100 % |
| `/ak40_can/status` (전체 스택 부하) | 응답 4/4 100 %, 틱 평균 1.35 · 95 % 1.44 · 최대 2.68 ms, **늦은 응답 9 → 9 (안 늘어남)**, 누락 9 → 9 |
| CPU (top 5 s, 12 코어) | 전체 user 6.4 % + sys 0.9 %. 노드별(한 코어 기준) ak40_can 9.0 · imu_to_base 8.0 · state_estimator 6.8 · fault_monitor 5.2 · rplidar 4.2 · realsense 4.2 · torque_mpc 4.2 · cartographer 4.0 % — 여유 큼. MPC 는 상태가 없어 아직 안 풂 |

**전면 라이다 미연결**: `rplidar_front` 가 `Error, unexpected error, code: 80008004` 로 시작하자마자 죽음. udev 규칙(`/etc/udev/rules.d`)은 시리얼 `f2d6a1d4…` → `/dev/lidar_front`, `fc55e23e…` → `/dev/lidar_rear` 인데, `lsusb` 의 CP210x 는 하나(후면)뿐이고 `/dev/lidar_front` 가 없다 — NUC 부팅(13:08) 이후 한 번도 USB 에 안 잡힘. 하드웨어(USB 선·어댑터·전원) 확인 뒤 4단계를 다시 한다.
측위 정확도는 책상 위(라이다 높이가 지도와 다름)라 이번엔 보지 않는다 — 바닥에서.

## 남은 것 (체크리스트에 반영)

- 1단계: 네 모터 CAN sudden interrupt 설정 → 4절 시험 다시
- 2단계: 모터 모드 밖 MIT 응답, 응답 시간, 에러 코드 종류 (미확인)
- 3단계: CAN bus-off 자동 복구 결정, 도는 중 전원 빼기(생략), ak40_can_node 늦은 응답 고착 수정(PC)
- 4단계: 전면 라이다 연결 뒤 측위·추정기 "준비 끝"·포즈 시각 경고·IMU 가속도 갱신 간격, `unset ROS_STATIC_PEERS` (또는 `.bashrc` 정리)
- 5단계: 정지 마찰로 저속 출발이 굼뜬지, DAMPING 정지 순간 슬립, 바닥 하중에서 마찰 보상
