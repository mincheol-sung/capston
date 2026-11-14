# 스택 변경 기록

`amr_ws/src/amr_basic_stack`의 코드·설정·launch를 바꿀 때마다 여기에 남긴다.
**2026-11-14부터 git(브랜치 `smc`)을 다시 쓴다. 스냅샷(`stack_status.sh --snapshot`)은 더 찍지 않는다** — 바뀐 파일·되돌리기는 git 으로 한다
(사용자 결정). 이 파일은 "왜 바꿨고 어떻게 확인했나" 를 남기는 용도로 계속 쓴다. 아래 "이 파일이 git의 무엇을 대신하나"·"기준 시점" 은
git 이 깨져 있던 2026-11-10 ~ 11-14 의 기록이다. 그 기간의 스냅샷은 `stack_baseline/` 에 남아 있다.

- 결정·과제·논의는 `torque_mpc_todo.md`, **실제로 무엇을 바꿨는지**는 이 파일이다.
- 최신 기록이 위로 온다.

## 이 파일이 git의 무엇을 대신하나 (2026-11-10 ~ 11-14, 지금은 git 을 쓴다)

| git | 여기서는 |
|---|---|
| 커밋 메시지 | 아래 "변경 기록"의 항목 하나 |
| `git status` / 바뀐 파일 목록 | `bash stack_baseline/stack_status.sh` (amr_ws에서 실행). 기준 시점 대비 수정·새 파일·삭제를 보여 준다 |
| 이전 커밋으로 되돌리기 | `stack_baseline/<기준>_code.tar.gz`에서 원본 파일을 꺼낸다 |
| 새 커밋 지점 | 큰 단계가 끝나면 `bash stack_baseline/stack_status.sh --snapshot`으로 새 기준을 만든다 |

원본 파일 하나 꺼내 보기:

```bash
mkdir -p ~/restore && tar -xzf ~/Documents/amr_ws/stack_baseline/2026-11-10_1924_code.tar.gz -C ~/restore amr_basic_stack/<경로>
```

- 백업 tar는 코드·설정 파일(py, cpp, hpp, yaml, lua, xml, msg, launch, CMakeLists 등)만 담는다. 디스크 여유가 작아서 지도·메시·데이터 파일은 뺐다. 그런 파일은 고치기 전에 따로 복사해 둔다.
- manifest(sha256)는 전체 파일을 담고 있어서 데이터 파일이 바뀐 것도 `stack_status.sh`가 잡는다.

## 기록 형식

```
### YYYY-MM-DD (n) — 한 줄 제목
- 왜: 어떤 결정·과제 때문인지 (todo.md 항목)
- 바꾼 파일:
  - `경로` — 무엇을
- 확인: 빌드/시뮬/실차 중 무엇으로 확인했나, 안 했으면 "미확인"
- 되돌리려면: (특별한 절차가 있을 때만)
```

## 기준 시점 (스냅샷 — 2026-11-14_1042 가 마지막. 이후는 git 커밋)

| 기준 | 내용 |
|---|---|
| `2026-11-10_1924` | 출발점. Linux로 복사해 온 상태 그대로(브랜치 `fix/semantic-map-jump-dynamic-gate`, 커밋 안 된 DRA_MPPI 변경 포함 — git이 깨져서 어디까지가 커밋된 것인지는 구분 불가). 파일 2964개, 코드 백업 1094개 |
| `2026-11-10_2051` | 1·2단계 끝(메시지 2개, 토크 모델·시뮬 플랜트·실험 launch). `stack_status.sh`는 이제 이 기준과 비교한다. 이전 기준과 비교하려면 `stack_status.sh 2026-11-10_1924` |
| `2026-11-11_1051` | 3단계 끝(torque_mpc_node, torque_output, 게이트 토크 모드, 시뮬 폐루프·통과시험 ①②③). `stack_status.sh 2026-11-11_1051`로 비교 |
| `2026-11-11_1551` | 4단계 끝(무장해야 기준 시간 진행, 8자 지도 고정, state_estimator·시뮬 센서) |
| `2026-11-11_2222` | 5단계 끝(슬립 모델, slip_logger, 랩별 속도, fault_monitor) |
| `2026-11-11_2257` | 6단계 끝(비교군 A: 퓨어 퍼슛 + 바퀴별 PI, `controller:=pure_pursuit`) |
| `2026-11-12_1111` | MPC 잠정 튜닝(w_delta 10, 속도 가중 ×0.1) |
| `2026-11-12_1144` | 7단계 끝(ak40_can_node·가짜 모터·벤치 도구, 실물 확인 전) |
| `2026-11-12_1300` | AK40-10 매뉴얼 대조 반영 |
| `2026-11-12_2048` | 리뷰 R1·R2(마찰 몫 경고, 시뮬 구동계 마찰 + 팀원 벤치 값) |
| `2026-11-13_1506` | 리뷰 R3~R5(SLAM 끊김 감시, 기준열 0.05 s, 좌표계 검사, run_compare.sh) |
| `2026-11-13_1654` | 리뷰 R6(게이트 Ctrl+C 마지막 DAMPING, NaN·inf → DAMPING, 발행 순서) |
| `2026-11-13_1720` | 리뷰 R7(퓨어 퍼슛 좌표계 검사) |
| `2026-11-13_1900` | 실차 IMU QoS(BEST_EFFORT)·추정기 입력 감시·시뮬 IMU BEST_EFFORT·imu_to_base 이중 보정 끔·시뮬 기동 로그 |
| `2026-11-14_1005` | 리뷰 R8(ak40 늦은 응답 반영·SIGTERM 정지 절차, 시뮬 가짜 모터 import 복구·응답 지연 흉내) |
| `2026-11-14_1023` | 리뷰 R9(공통 종료 처리 node_main.spin, fault_monitor·slip_logger 적용, 응답 없는 바퀴 슬립 제외) |
| `2026-11-14_1042` | 리뷰 R10 끝(실차 launch robot_state·즉시 무장 금지, 전 노드 공통 종료 처리, launch 모드 시험) — **리뷰 0~R10 완료.** 마지막 스냅샷 |

## 변경 기록

### 2026-11-14 (4) — 실차에서 찾은 ak40_can_node 늦은 응답 고착 고침, 벤치 spin 토크 상한 (NUC 에서 수정)
- 왜: 실차 3단계(`analysis/torque_mpc_real/2026-11-14_bench/RESULTS_KO.md` 8절). candump 로 응답은 1.2 ms 안에 다 오는데
  `/ak40_can/status` 누적 늦은 응답이 주기마다 2 씩 늘었다. 주기 시작에 `take_pending` 으로 반영한 지난 주기 RL·RR 응답이
  `fresh` 라서, `collect` 의 끝 조건(`all_replied` = fresh.all)이 FL·FR 새 응답(0.6 ms)만으로 차 0.7 ms 에 끝나고 RL·RR 새 응답
  (1.1 ms)은 다시 다음 주기로 — 자기 유지. RL·RR 바퀴 상태가 늘 10 ms 늦었다. R8 에서 늦은 응답을 버리지 않게 바꿀 때 생긴 것.
  벤치 도구는 todo "벤치 spin 토크 상한" (docstring 은 max_tau 안이라는데 spin 이 kd 0.5 × 20 rad/s = 10 N·m 를 요구).
  이날부터 코드를 NUC 에서도 고치기로 함(사용자 결정, smc 브랜치).
- 바꾼 파일:
  - `amr_mpc/ak40/bridge.py` — `cycle_rx`(begin_cycle 이후 받은 응답) 추가, `all_replied` 는 cycle_rx 기준
  - `amr_mpc/nodes/ak40_can.py` — 송신 직전 `begin_cycle()` (기동·주기), collect·take_pending 주석
  - `test/test_ak40.py` — `_SplitBus`(FL·FR 0.6 ms, RL·RR 1.1 ms, 주기 10 ms) + `test_late_replies_do_not_lock_in`
  - `tools/ak40_bench.py` — `spin()`: v_des 를 마지막 응답 속도 ± max_tau/kd 로 잘라 모터 토크 kd·|v_des − v| ≤ max_tau
- 확인: 새 시험이 옛 코드에서 실패(늦은 응답 2 → 42), 고친 코드에서 통과(2 → 2). 시험 183개 통과. `colcon build --packages-select amr_mpc`.
  **실차**(DAMPING 11 s): 늦은 응답 4 → 4(안 늘어남, 고치기 전 175 → 2407), 응답 4/4 100 %, 틱 평균 1.24 ms(네 응답을 다 기다림,
  전 0.79), `/wheel_states` 99.999 Hz(간격 8~12 ms, 표준편차 0.10 ms). candump 응답 도착 FL 0.49 · FR 0.61 · RL 0.99 · RR 1.12 ms.
  벤치 spin 은 무부하 FL 모델 계산으로 요구 토크 최대 0.500 N·m·1.5 s 끝 19.88 rad/s — 실물 미확인.

### 2026-11-14 (3) — 리뷰 R10 (launch 연결): 실차 경로에 robot_state, 실차 즉시 무장 금지, 모든 토크 스택 노드에 공통 종료 처리, 모드별 launch 시험
- 대조 결과 (문제없음): launch 가 노드에 넘기는 파라미터 이름 60여 개가 모두 노드에 선언돼 있다(rclpy·rclcpp 는 선언 안 된 이름을 말없이 버리므로 오타면 조용히 무시된다 — 스크립트로 전부 대조). 토픽 사슬 figure8 → MPC/퓨어 퍼슛 → (바퀴 PI) → torque_output → 게이트 → 시뮬/ak40, /wheel_states·/state_estimator/odom·/mpc/armed 구독자 모두 기본 이름이 맞다
- 바꾼 것
  - `launch/torque_experiment.launch.py` (plant:=real)
    - **robot_state_publisher 포함** (`robot_state:=true` 기본): 안내문은 "sensors.launch.py 가 robot_state_publisher 를 띄운다" 고 했지만 실제로는 안 띄운다(bringup.launch.py 가 띄우는데 그쪽은 RoboClaw 구동 노드까지 띄운다). 없으면 imu_to_base 가 TF 를 기다리다 IMU 를 하나도 못 내고 Cartographer 도 라이다 위치를 모른다. amr_description 이 없으면 알아듣는 오류
    - **armed:=true 거부**: 주석에는 "시뮬에서만" 이라 했지만 막혀 있지 않았다. 실차는 준비를 보고 'a'
    - 안내문 정정: 같이 띄우지 말 것(bringup.launch.py·localization.launch.py — 후자는 imu_to_base_node 를 하나 더 띄워 IMU 가 두 배), NUC 에서 use_rviz:=false 권장, 실차 확인 목록 경로
  - 공통 종료 처리 `node_main.spin` 을 나머지 노드에 적용: 게이트·추정기·figure8·퓨어 퍼슛·바퀴 PI·torque_output·휠 오돔·시뮬 플랜트·ak40_can (R8 에서 ak40 에 따로 넣은 SIGTERM 처리를 이것으로 합침). 옛 스택 노드(body_tracking, wheel_tracking, sim_plant_body/wheel, fake_reference)는 손대지 않음
  - `test/test_launch_modes.py` 11개: sim 기본·비교군 A·참값·sim_can·휠 오돔·플랜트만·실차(MPC·A)·robot_state 끔·실차 즉시 무장 거부·실차 참값 거부. `test_can_setup.py` 의 launch 시험에 armed 값 추가(실차 무장 검사가 CAN 준비 전에 읽는다)
- 실제 프로세스 확인: 게이트에 무장·토크 중계 중 SIGINT 2·SIGTERM 2·SIGHUP 1 → 다섯 번 모두 마지막 메시지 DAMPING, exit 0, Traceback 없음(예전 SIGTERM 은 DAMPING 없이 즉시 죽음). 시뮬 세 판(B·A·CAN 6 ms 지연) 끌 때 모든 노드 "finished cleanly", "process has died" 0
- 실차 확인으로 남김 (`real_robot_checklist.md` R10 절 + 4단계 명령 정정): TF 확인, 노드 중복, CAN 자동 준비 sudo, 끌 때 로그
- 확인: Python 시험 182개·C++ 12개 통과. 시뮬 B 15.4 mm(게이트 신호 시험과 겹친 판) → 다시 12.8 mm, A 5.1 mm, CAN(6 ms 지연) 13.0 mm

### 2026-11-14 (2) — 리뷰 R9 (fault_monitor·slip_logger): 종료 중 죽던 것 고침(공통 spin), 응답 없는 바퀴 슬립 제외
- 대조 결과 (문제없음): fault_monitor 검사 5개(포즈 튐·추종 오차·모터 에러·바퀴 끊김·상태 끊김)는 무장 중에만, 무장 순간에 포즈 튐 기준·추종 오차를 다시 잡는다. 한 번 세우면 같은 판에서 반복 발행 안 함. /mpc/arm(VOLATILE)·/mpc/armed(TRANSIENT_LOCAL) 짝 맞음. 실차 경로의 끊김(카메라 IMU·Cartographer·ak40)은 모두 추정기 출력 멈춤이나 /wheel_states 끊김·255 로 이어져 잡힌다. slip_logger 식은 구조도 정의와 같다(ω_nom = T⁻¹v̂, 분모 max(|ω|,|ω_nom|,ω_ε))
- 바꾼 것
  - **종료 중 죽던 것** (R7 에서 봄): Ctrl+C 가 rclpy 의 메시지 변환(pybind11) 한가운데 떨어지면 KeyboardInterrupt 가 `RuntimeError: Unable to convert call argument '0' to Python object` 로 바뀌어 `except KeyboardInterrupt` 를 빠져나갔다 → Traceback, exit code 1. 메시지를 1 kHz 로 쏟아붓고 SIGINT 를 보내는 재현에서 fault_monitor 20 판 중 4 판
  - 새 파일 `amr_mpc/core/node_main.py` `spin(node)`: SIGINT·SIGTERM·SIGHUP → 종료 기록 + KeyboardInterrupt, 신호 뒤의 RuntimeError 는 종료로 봄(신호 없이 난 RuntimeError 는 그대로 올림), 끝나면 세 신호 무시(정리 중 두 번째 신호로 안 끊기게)
  - `fault_monitor.py`·`slip_logger.py` main 이 이것을 씀. 재현 시험 fault_monitor 0/20, slip_logger 0/15, 15 판 모두 요약 출력. slip_logger 는 이제 SIGTERM 에도 요약·CSV 를 남긴다
  - `slip_logger.py`: 응답이 없거나(fresh false) 에러인 바퀴의 λ 는 NaN(그 바퀴 속도는 마지막 값이 남은 것 — 차체만 변하면 가짜 슬립), 요약은 그 행을 빼고 뺀 수를 적음. 시뮬 토픽 모드는 늘 fresh·에러 0 이라 안 보였다
  - 시험 6개 추가: `test/test_node_main.py` 5개(변환 중 신호 = 정상 종료, 신호 없는 RuntimeError 는 올림, 세 신호 각각), `test_slip_logger.py` 1개
- 남김: 다른 Python 노드(게이트·추정기·torque_output·figure8·퓨어 퍼슛·바퀴 PI·시뮬 플랜트)도 같은 spin 패턴이라 같은 종료 오류가 날 수 있다 → R10(launch·종료 연결)에서 한꺼번에. ak40_can 은 R8 에서 SIGTERM 처리를 따로 넣었다. 과속·바퀴 멈춤 검사는 여전히 "나중에, 필요하면"
- 관찰 (조사 안 함): 이번 B μ 0.3 한 판은 이탈 없이 완주(13.3 mm). 예전 기록(2026-11-12)은 튜닝 9가지 모두 이탈 — 바퀴별 마찰 보상 변경 뒤 달라졌을 수 있다. todo "B 의 μ 0.3 이탈" 과제 볼 때 다시 확인
- 확인: Python 시험 171개 통과. 시뮬 μ 0.3: B 13.3 mm·참 슬립률 RMS 0.10~0.17, A 5.4 mm·0.023~0.025. 두 판 모두 Traceback·"process has died" 0

### 2026-11-14 (1) — 리뷰 R8 (ak40 CAN): 늦은 응답 반영, SIGTERM 에도 정지 절차, 가짜 모터 모드 복구 + 실차 확인 목록
- 실차 확인 목록: `amr/notes_amr/real_robot_checklist.md` 새로 만듦 — todo 곳곳의 실차 항목을 전원 전 → 모터 설정 → 벤치 → 네 바퀴 → 센서 → 바닥 주행 순서로 모음 (사용자 요청). todo "과제" 머리에 링크
- 대조 결과 (문제없음): MIT 명령·응답 비트 배치, 범위 P ±12.5 V ±45.5 T ±5 KP 500 KD 5, 응답 CAN ID = 모터 ID, 온도 −40, 8 바이트 응답, ID 1~4 = FL FR RL RR, 1 Mbps(setup_can `-s8`) — 매뉴얼·팀원 스크립트(`CAN/can_torque_feedback.py`)와 같다. NaN·inf·길이 틀린 토크 → DAMPING 은 이미 `bridge.set_command` 에 있고 시험도 있음(R6 에서 넘긴 것 닫음)
- 참고 (고치지 않음): 팀원 스크립트는 정수 변환을 버림으로, 우리는 반올림으로 한다. 토크 0 이 팀원 −1.22 mN·m / 우리 +1.22 mN·m, 평균 반 칸 차이(쿨롱 마찰 0.025 N·m 의 약 5 %). 팀원 마찰 식별값은 버림 기준 명령 토크로 낸 것 — 식의 offset 항이 대부분 흡수한다
- 바꾼 것 (시뮬에서는 안 보이는 실차 문제)
  - `amr_mpc/nodes/ak40_can.py` **늦은 응답**: 주기 시작에 `bus.drain()` 으로 버리던 지난 주기의 늦은 응답(reply_wait 4 ms 뒤 도착)을 반영한다(`take_pending`). 실차 slcan 에서 네 모터 응답이 늘 4 ms 보다 늦으면 전부 버려져 0.05 s 뒤 네 바퀴 error 255 → fault_monitor 무장 해제 → 로봇이 아예 못 움직일 수 있었다. 팀원은 네 모터에 9 ms 를 기다렸다(모터 하나 최대 3.6 ms 만 잼). 가짜 모터는 바로 답해서 안 보였다. 늦은 응답도 이번 발행의 fresh 로 센다(거짓 "응답 0/4" 경고 방지). `/ak40_can/status` 에 7번째 값(누적 늦은 응답) 추가
  - 같은 파일 **SIGTERM·SIGHUP**: Ctrl+C 처럼 DAMPING 0.5 s → 모터 모드 나오기를 밟게 함. 전에는 `SignalHandlerOptions.NO` 라 kill·systemd·터미널 닫힘에 즉시 죽어 모터가 마지막 명령(토크일 수 있다)을 모터 쪽 CAN 끊김 설정에만 맡겼다. 정지 절차 중 두 번째 신호는 무시
  - 같은 파일: 잘못된 토크 명령(NaN·inf·길이)에 오류 로그
  - `amr_mpc/nodes/sim_plant_torque.py` **가짜 모터 모드가 켜지자마자 죽던 것**: `from amr_mpc.ak40 import protocol as AKP` 가 빠져 있었다(스냅샷으로 보면 2026-11-12_1300 ~ 2048 사이, R1·R2 무렵 빠짐) → `plant:=sim_can` 이 그 뒤로 내내 안 돌았다. import 복구
  - 같은 파일 + launch: 가짜 모터 **응답 지연** `can_reply_delay`(기본 0) — 늘 늦는 실차 버스를 시뮬에서 재현
  - `amr_mpc/ak40/bridge.py` 머리주석: NaN·inf → DAMPING 규칙 적음. `torque_output.py` 머리주석의 옛 "nan_to_num 에서 토크 0" 설명 고침
  - `test/test_ak40.py` 3개 추가: 늘 늦는 버스에서 error 255 가 안 생기고 상태가 갱신된다 / SIGTERM 으로 꺼도 마지막 프레임이 네 모터 모터 모드 나오기 / 가짜 모터 모드 시뮬 플랜트가 켜진다
- 실차 확인으로 남김 (`real_robot_checklist.md` R8 절): 네 모터 응답 시간과 늦은 응답 수(늘 늦으면 reply_wait 0.008), kill 로 껐을 때 제동 뒤 풀림, 끌 때 slcan 송신 버퍼 넘침(팀원 벤치에서 한 번), 모터 하나 전원 껐다 켰을 때 다시 붙는지
- 확인: Python 시험 165개 통과. CAN 전 구간 시뮬(`plant:=sim_can`, unshare 이름공간의 vcan0) B 지연 없음 13.3 mm / 응답 6 ms 지연 11.7·12.8 mm(두 판), 정지 없이 완주. "응답 x/4" 경고는 지연 판도 지연 없음 판과 같은 수준(기동 때 + 데스크톱 흔들림)

### 2026-11-13 (6) — 실차 IMU 연결 실패(QoS) 고침 + 시뮬·실차 차이 점검
- 왜: 실차에서 state_estimator 가 IMU 를 하나도 못 받았다 (다른 세션 "SMC 브랜치 연결" 이 원인 확인, 사용자가 이 작업공간에서 고치라고 함). 실차 `imu_to_base_node` 는 `/imu/base_link` 를 BEST_EFFORT 로 내는데 추정기는 숫자(50)만 넘겨 RELIABLE 로 구독 → 연결 막힘 → 바이어스 초기화가 안 끝나 출력 없음 → MPC 도 안 움직임. 시뮬은 RELIABLE 로 내서 안 드러났다
- 바꾼 것
  - `amr_mpc/nodes/state_estimator.py`: IMU 구독을 BEST_EFFORT(KEEP_LAST 50)로. BEST_EFFORT 구독은 RELIABLE 발행(시뮬)과도 붙는다. 발행 쪽은 안 바꿈(200 Hz 재전송, 다른 구독자 영향)
  - 같은 파일: **입력 감시** — 준비 전까지 2 s 마다 IMU·포즈 0 개면 오류 로그(확인 명령 포함). IMU stamp 가 시스템 시각과 0.1 s(`clock_tolerance`) 넘게 어긋나면 경고(MPC 가 기준열을 그만큼 밀린 시각에서 고른다). 포즈 stamp 가 IMU 시각보다 0.05 s 넘게 미래면 경고(그 시각 추정값을 못 찾아 최신 값과 비교하게 된다). "오래된 포즈" 경고에 나이 표시
  - `amr_mpc/nodes/sim_plant_torque.py`: 시뮬 IMU 를 실차처럼 BEST_EFFORT 로 낸다 → RELIABLE 구독 실수를 시뮬에서도 잡는다
  - `launch/torque_experiment.launch.py` (plant:=real): `imu_to_base_node` 에 `bias_calib_s: 0.0`. 이 노드가 기동 3 s 동안 자이로 바이어스를 잡아 빼는데, 추정기도 1 s 만에 같은 바이어스를 잡는다 → 3 s 째에 한 번 더 빠져 자이로가 바이어스만큼 계단처럼 어긋났다. 실차에만 있는 노드라 시뮬에서 안 보였다
  - `sim_plant_torque.py` 기동 로그: 마찰 계수가 바퀴별 4 개(오늘 17:23 `torque_plant.yaml`·`model.py` 변경, 이 기록에 없음)가 된 뒤 `{coulomb:.4f}` 형식에서 **시뮬 노드가 켜지자마자 죽었다** → 배열로 찍게 고침
  - `test/test_torque_estimator.py`: 3개 추가 — BEST_EFFORT 로 낸 IMU 를 추정기가 받는다 / RELIABLE 도 받는다 / 시뮬 IMU 발행이 BEST_EFFORT
- 점검했지만 문제없음: `/camera/imu`(RealSense) → imu_to_base 는 BEST_EFFORT 짝, Cartographer 는 `/camera/imu` 원본을 따로 받음, `/tracked_pose`·`/scan_matched_points2` 는 Cartographer 기본(RELIABLE)과 우리 기본 구독이 짝, `/mpc/armed` 는 발행·구독 모두 TRANSIENT_LOCAL, `/mpc/arm` 은 게이트가 기본(VOLATILE) 구독이라 `ros2 topic pub`·fault_monitor·잠금 발행 모두 붙음. URDF(xacro)에 base_footprint·camera_imu_optical_frame·camera_gyro_frame 있음. 카메라 장착 회전 0
- 실차 확인으로 남김: RealSense `global_time_enabled`(기본 켬에 기대고 있다, sensors.launch.py 에 명시 없음), 라이다 수신 시각과 IMU 시각이 맞는지 — 새 경고로 보인다. 목록은 todo "시뮬에서 안 보이는 실차 차이" 절
- 확인: Python 시험 162개 통과(다른 작업으로 늘어난 것 포함). 시뮬 B 12.8 mm·최대 35 mm, A 6.3 / 5.1 mm(두 판, 평소 5.1~5.5 범위 흔들림). 새 경고는 시뮬에서 한 번도 안 뜸, 추정기 "준비 끝" 정상

### 2026-11-13 (5) — 리뷰 R7 (비교군 A): 퓨어 퍼슛에도 좌표계 검사, 재무장 토크 측정
- 왜: R7 리뷰. 계산부는 문제 없음. R5 에서 MPC 에 넣은 기준·상태 좌표계 검사가 A 에는 없어 정지 규칙이 갈라져 있었다 (공정성 기준 "같은 정지 규칙")
- 바꾼 것
  - `amr_mpc/nodes/pure_pursuit.py` — 상태 frame_id 가 있고 기준열 frame_id 와 다르면 명령을 끊는다 (wheel_pi 가 cmd_timeout 뒤 DAMPING + 리셋). 머리주석에 규칙 추가
  - `config/torque_baseline_a.yaml` — 주석의 J_w 9.7e-4 → 8.9e-4 (R2 팀원 벤치)
  - 시험 1개: `test_baseline_a.py::test_pure_pursuit_node_refuses_frame_mismatch`
- 리뷰에서 확인한 것 (문제 없음): 퓨어 퍼슛 곡률 κ = 2y/L² 와 시간 기준 룩어헤드(기준 시각 점부터, "멀어지는 중" 조건), 진행 보정 v = v_ref + k_along·e_along(주행 중 후진 없음), 제자리 회전 구간, ω·v 상한. 바퀴 PI: 크기·변화율 상한이 B 와 같은 값(τ̄ 1.2, Δτ̄ 0.4/10 ms), |u_prev| ≤ τ̄ 라 lo ≤ hi 항상 성립, 조건부 적분 + 적분항 ±τ̄, dt 0~30 ms 자름, 리셋 시 적분·직전 출력 0. 노드: 미무장·무장 뒤 새 명령 대기·명령 끊김 50 ms·바퀴 상태 끊김 감시 → DAMPING + 리셋, 끌 때 DAMPING. 역기구학 T⁻¹ 는 플랜트와 같은 행렬. NaN 이 들어오면 PI 적분이 NaN 으로 남지만 R6 의 torque_output 검사가 DAMPING 으로 바꾸고 무장 해제 때 리셋된다 (안전 쪽으로 멈춤)
- **재무장 토크 (A, B 와 같은 시나리오)**: 3 × 1.5 m 8자 0.5 m/s, 12 s 에 정지 → 1.5 s 뒤 재무장, PI 출력 최대 |τ|
  | 상태 | 순항 중 최대 | 재무장 첫 틱 | 재무장 뒤 0.3 s 최대 | (B 같은 칸) |
  |---|---|---|---|---|
  | 추정기 잡음 1배 | 0.058 | 0.053 | 0.057 | 0.24 / 0.067 / 0.116 |
  | 참값 | 0.029 | 0.062 | 0.062 | 0.034 / 0.063 / 0.094 |
  - A 가 B 보다 부드럽다 (PI 는 바퀴 속도 오차만 보고, figure8 재개 램프가 속도 기준을 0 에서 올린다)
- 확인: Python 시험 134개 통과. 시뮬 A 5.07 mm·최대 17 mm (전과 같음)
- 측정 도구 메모: 재무장 측정은 /mpc/arm 구독자가 붙은 뒤 무장해야 한다 — 첫 판은 연결 전에 보낸 무장 신호가 사라져 재무장만 먹혔다. 그 판 종료 때 **fault_monitor 가 rclpy 종료 중 take_message RuntimeError 로 죽었다** → R9 에서 본다
- 남은 것 (R9): fault_monitor 종료 처리

### 2026-11-13 (4) — 리뷰 R6 (torque_output·게이트): Ctrl+C 때 마지막 DAMPING, NaN·inf 토크 → DAMPING, 발행 순서
- 왜: R6 리뷰에서 실제로 돌려 확인한 구멍 셋
  1. **게이트를 Ctrl+C 로 끄면 마지막 신호가 DAMPING 이 아니었다** (실측: 명령 100 Hz 흐르는 중 SIGINT → 마지막 메시지 mode 0, 토크 0.1). rclpy 기본 SIGINT 처리기가 먼저 context 를 닫아 `shutdown()` 의 `rclpy.ok()` 가 거짓 → 정지 신호를 건너뜀. CAN 노드의 cmd_timeout 0.2 s 뒤에야 DAMPING
  2. **토크에 NaN·inf 가 섞이면 정지가 아니었다**: inf 는 torque_output 클램프에서 ±1.3 N·m(최대 토크), NaN 은 CAN 다리 `nan_to_num` 에서 0(관성 주행). 스택 어디에도 isfinite 검사가 없었다
  3. 키보드 스레드의 정지('s')와 실행기 스레드의 즉시 중계가 lock 밖에서 발행해, 드물게 정지 신호 **뒤에** 토크가 한 번 더 나갈 수 있었다 (다음 타이머 20 ms 안에 다시 DAMPING)
- 바꾼 것
  - `amr_mpc/nodes/keyboard_command_gate.py` — `rclpy.init(..., signal_handler_options=SignalHandlerOptions.NO)` + 끌 때 두 번째 Ctrl+C 무시 (torque_output·ak40 과 같은 방식). 발행 셋(즉시 중계·정지·타이머)을 모두 lock 안으로. 머리주석에 이유
  - `amr_mpc/nodes/torque_output.py` — MODE_TORQUE 인데 토크가 유한하지 않으면 DAMPING 으로 바꿔 내고 오류 로그 (stamp 유지). 바퀴 속도의 NaN·inf 는 그 바퀴 마찰 보상만 0. 머리주석 구조도 v3 → v4
  - 시험 5개 추가: `test_torque_output.py` (바퀴 속도 NaN, 토크 NaN·+inf·−inf → DAMPING), `test_command_gate.py` (끌 때 마지막 DAMPING)
- 리뷰에서 확인한 것 (문제 없음): DAMPING 그대로 통과, 안전 클램프 ±1.3 + 경고, stamp 유지(torque_output·게이트 둘 다), forward_on_receive 에서 타이머 중복 없음, 무장 직후 묵은 명령 안 냄(`latest_command_time = 0`), 게이트 command_timeout 0.2 s = CAN cmd_timeout 0.2 s, 정지 신호에는 지금 시각, /mpc/armed latched
- 확인: Python 시험 133개 통과. 실제 프로세스 SIGINT 3회 모두 마지막 메시지 DAMPING. 시뮬 B 12.7 mm (변화 없음)
- 남은 것 (R8): CAN 다리 `bridge.py` 도 NaN·inf 를 `nan_to_num`(→ 0, +inf → 최대) 대신 DAMPING 으로 바꾸는 게 맞다 — 마지막 방어선

### 2026-11-13 (3) — 리뷰 R5 (torque_mpc C++): 기준·상태 좌표계 검사, 재무장 토크 측정
- 왜: R5 리뷰. 기준열 frame_id 와 상태 frame_id 를 비교하지 않아, 다른 좌표계(예: odom 기준 + map 상태)가 들어오면 두 좌표계 차이만큼 로봇을 끌고 간다
- 바꾼 것: `controller/mpc_cpp/src/torque_mpc_node.cpp` — 둘 다 frame_id 가 있고 다르면 풀지 않고 DAMPING + 리셋, 오류 로그. 머리주석의 기준열 꼴(0.2 × 20 → 0.05 × 80) 정정. amr_mpc_cpp 다시 빌드
- 리뷰에서 확인한 것 (문제 없음): 이산 모델(A, B(θ_r), E — ZOH 정확), 감도 Φ 누적 순서(P += Ts·V 다음 ½Ts²G), Nc 이후 입력 유지, 비용 H = ΦᵀWΦ + w_Δ DᵀD + q_null N 과 g 의 τ_prev 항, DAQP 의 ½xᵀHx + fᵀx 규약, 영공간 방향 n 이 (T⁻¹)ᵀ 의 영공간, 첫 스텝 상한 우선 교집합, QP 실패 대비, d̂ 식-9·11(기본 끔), θ 언랩(식-15), B·C 상한, 노드의 무장·기준·상태 끊김·무장 뒤 새 기준열 대기·hold 래치·워치독 DAMPING
- **재무장 토크 (열린 과제였던 것)**: 3 × 1.5 m 8자 0.5 m/s, 무장 → 12 s 에 정지 → 1.5 s 뒤 재무장, MPC 순토크 최대 |τ|
  | 상태 | 순항 중 최대 | 재무장 첫 틱 | 재무장 뒤 0.3 s 최대 |
  |---|---|---|---|
  | 추정기 잡음 1배 | 0.24 (잡음 탓) | 0.067 | 0.116 |
  | 참값 | 0.034 | 0.063 | 0.094 |
  - 역사: 1.3 N·m(2026-11-11 처음) → 0.3(무장 연동·재동기, 1판 튜닝) → **0.09~0.12(지금 튜닝)**. figure8 재동기가 거리 4.7 → 0.1 cm 로 맞춰 준다
  - 주의: μ 0.3 바닥의 물리 마찰 한계는 바퀴당 0.083 N·m 라 B 는 재무장 순간 잠깐 넘을 수 있다 (C 는 0.067 로 묶임)
- 확인: C++ 시험 12개 통과, 시뮬 B 12.6 mm(추정기) / 0.52 mm(참값) — 평소 주행에서 잘못 서지 않음

### 2026-11-13 (2) — 리뷰 R4 (figure8): 기준열 간격 0.2 s → 0.05 s, 통과시험 ④(두 바퀴) 통과
- 왜: R4 리뷰. MPC(`ReferenceHorizon`, C++)와 퓨어 퍼슛(Python)은 기준열 행 사이를 직선으로 보간한다 → 곡선 안쪽으로 줄이 생겨 위치 기준이 경로에서 v²·dt²/(8R) 만큼 안쪽, 속도 기준(접선)과 어긋난다. 오프라인 계산 (3 × 1.5 m 8자, R_min 0.31 m), 최대 보간 오차: 0.5 m/s 3.9 mm, 1.0 m/s 15.4 mm, **1.5 m/s 32.5 mm** (dt 0.2) → dt 0.05 면 0.25 / 1.0 / 2.2 mm. 허밋 보간이면 0.2 s 에서도 1.8 mm 이하지만 C++·Python 둘 다 바꿔야 해서, 설정만 바꾸는 쪽을 골랐다
- 바꾼 것
  - `config/torque_track.yaml` — `dt: 0.05`, `horizon_steps: 80` (길이 4 s 는 예전 0.2 × 20 과 같음), 근거 표
  - `launch/torque_experiment.launch.py` — 트랙 키에 `dt`·`horizon_steps` 추가 (launch 인자로도 덮어씀, `horizon_steps` 는 정수로)
- 시뮬 결과 (참값 기준 추종, B)
  | 조건 | dt 0.2 RMS / 최대 | dt 0.05 RMS / 최대 |
  |---|---|---|
  | 참값 상태, 0.5 m/s | 0.71 / 3.2 mm | 0.52 / 2.0 mm |
  | 참값 상태, 0.65 m/s (이 트랙 부하 예산 한계) | 1.13 / 5.4 mm | 0.89 / 2.6 mm |
  | 추정기 잡음 1배, 0.5 m/s | 12.6~12.7 mm (R2) | 12.6 / 31.8 mm |
  - 잡음이 있으면 추정기 잡음이 바닥이라 차이가 안 보인다. 실제 계획 속도(1.0·1.5 m/s)에서 효과가 커진다
- **통과시험 ④ (8자 두 바퀴, `laps:=2`)**: B 12.7 mm·yaw RMS 0.27°, A 5.5 mm, 정지 없음. 이 8자는 헤딩이 −225° 까지 가서 한 바퀴에도 ±180° 를 넘는다 → 각도 언랩(C++ `ReferenceHorizon`, figure8 `np.unwrap`) 정상
- 리뷰에서 확인한 것 (문제 없음): Gerono 곡선·호길이 재매개화·곡률(ω = κv), 랩별 속도 표, 정렬 회전, 무장 연동(무장 중에만 기준 시간, 배속 0→1 램프, 재무장 때 가장 가까운 경로 시각으로 재동기, 첫 행 = 지금), 지도 고정 출발 거부·진입 구간, abort_error, 완주 뒤 발행 중단 → MPC ref_timeout
- 알아 둘 것: 기준 부하 검사가 3 × 1.5 m 트랙에서 0.67 m/s 위를 거부한다 — 계획 속도 1.0·1.5 m/s 는 실제 트랙(더 큰 8자)을 정한 뒤
- 확인: Python 시험 128개 통과

### 2026-11-13 (1) — 리뷰 R3 (상태 추정기): SLAM 포즈가 끊기면 출력을 멈춘다 + 비교 실행 스크립트를 tools 로
- 왜: R3 리뷰에서 찾은 안전 구멍. SLAM(`/tracked_pose`)이 멈춰도 IMU 는 계속 오므로 추정기가 IMU 적분만으로 100 Hz 출력을 이어 간다. 위치가 t² 로 흘러가는데 MPC 는 그 추정값을 따라가므로 추종 오차 감시(fault_monitor, 추정값 기준)에도 안 걸리고, fault_monitor 의 끊김 검사는 `/wheel_states`·`/state_estimator/odom` 만 본다 → 실제 로봇은 아무도 모르게 벗어난다
- 바꾼 것
  - `amr_mpc/torque/estimator.py` — `last_fix_t`(마지막으로 포즈를 반영한 IMU 시각: 보정·초기화·재초기화), `dead_reckoning_age()`, `ready_to_publish()`. 설정 `max_dead_reckoning`
  - `amr_mpc/nodes/state_estimator.py` — `ready_to_publish()` 가 False 면 출력을 멈추고 오류 로그 → MPC 는 state_timeout(0.3 s) 뒤 DAMPING, fault_monitor 는 state_timeout 으로 무장 해제. 포즈가 다시 오면 바로 재개
  - `config/torque_estimator.yaml` — `max_dead_reckoning: 0.5` (평소 보정 간격 약 0.1 s, 0 이면 검사 안 함)
  - `test/test_torque_estimator.py` — 시험 1개 (0.4 s 는 냄, 0.6 s 면 멈춤, 포즈가 오면 재개)
  - `tools/run_compare.sh` — 새 파일. 시뮬 비교 한 판(launch → 5 s 뒤 무장 → `compare_run.py` JSON). 지금까지 임시 폴더에 두던 것이 날짜가 바뀔 때마다 지워져서 저장소로. 결과 폴더는 `OUT` 환경변수(기본 `/tmp/amr_compare_runs`)
- 리뷰에서 확인한 것 (문제 없음): 예측식(IMU 점, 중간 각 회전), 보정량을 늦은 포즈 시각에서 지금까지 전파하는 식(δp + Dδv − ½D²δb, δv − Dδb, ψ: −D·δb_g)과 이력 갱신, 바이어스의 map→몸체 변환, 로봇 중심으로 옮기는 식(v − ω×r), 정상상태 칼만 게인 설계(잡음 밀도 0.07 = 0.95 m/s² × √0.005 s), 정지 바이어스 초기화, 포즈 튐 처리, 같은 stamp 포즈 중복 제거, 시뮬 SLAM 모드 기본 `scan_time` = 실차 lua(외삽 끔)
- 고치지 않은 것 (작음): IMU 표본 간격을 0.05 s 로 자름(그보다 긴 공백은 시간이 빠짐), 포즈 튐이 5번 이어지는 0.5 s 동안 출력이 잠깐 멈출 수 있음(무장 중이면 fault_monitor 가 튐으로 이미 세움)
- 확인: Python 시험 128개 통과. 시뮬 B 1판 12.6 mm(이전 12.6~12.7) — 평소 주행에서 잘못 멈추지 않음(멈춤 로그 0번)

### 2026-11-12 (7) — 리뷰 R2: 시뮬 플랜트에 구동계 마찰 + 팀원 벤치 값 반영
- 왜: 사용자 결정 ("마찰 모델을 넣자"). 팀원 벤치(`amr/notes_amr/CAN/BENCH_RESULTS_KO.md`)에서 구동계 쿨롱 마찰 ≈ 0.026 N·m — C 상한 0.067 의 약 40 %. 시뮬에 마찰이 없어서 실차보다 낙관적이었다
- 바꾼 것
  - `amr_mpc/torque/model.py` — `DrivetrainFriction(coulomb, viscous, smooth)` 새 클래스 (τ_fric = coulomb·tanh(ω/ω_s) + viscous·ω, 안정 적분 간격 계산). `TorquePlant.step(..., fric=None)`, `SlipPlant(..., fric=None)` 가 바퀴에 τ − τ_fric(ω) 를 준다. MPC 예측 모델은 그대로(마찰 모름)
  - `amr_mpc/nodes/sim_plant_torque.py` — 파라미터 `friction_scale`(기본 1, 0 = 마찰 없음)·`friction_smooth`(0.05 rad/s, 보상 쪽 0.5 보다 작게 → 정지 근처 보상 오차를 흉내). 적분 간격을 마찰 강성으로도 제한. 시작 로그에 마찰 표시. 중복 import 한 줄 정리
  - `launch/torque_experiment.launch.py` — `friction_scale` 인자
  - `config/torque_plant.yaml` — `friction.coulomb_Nm 0 → 0.026`, `viscous 0 → 0.0016`, `tau_fric_max_Nm 0 → 0.1` (≥ 0.026 + 0.0016 × 45.5). `wheel.inertia_J_w_kgm2 9.7e-4 → 8.9e-4` (팀원 4개 평균). **B 상한 τ̄ 1.3 → 1.2 N·m, Δτ̄ 0.43 → 0.40.** C 는 τ_μ 가 상한이라 그대로
  - `test/test_drivetrain_friction.py` — 새 시험 6개 (모양, 작은 토크는 못 밂, 직진 가속 = 순토크, 보상이 고속에서 마찰을 지움, 정지 안정, 바퀴 띄운 감속)
- 시뮬 결과 (3 × 1.5 m 8자 0.5 m/s, 추정기 잡음 1배, 참값 기준 추종 RMS / 최대 [mm], `tools/compare_run.py`)
  | 조건 | RMS | 최대 | 최대 τ |
  |---|---|---|---|
  | B, 마찰 = 보상 (scale 1) — 2판 | 12.7 / 12.6 | 33 / 38 | 0.19 |
  | B, 실제 마찰이 보상보다 30 % 작음 (0.7) | 12.6 | 33 | 0.19 |
  | B, 실제 마찰이 보상보다 30 % 큼 (1.3) | 13.0 | 33 | 0.19 |
  | B, 실제 마찰 없음 + 보상만 (0) — 2판 | 16.3 / 14.8 | 39 / 37 | 0.19~0.20 |
  | A (PP+PI), scale 1 | 5.4 | 16 | 0.11 |
  | C μ 0.3, scale 1 (참 λ RMS 0.07) | 15.0 | 38 | 0.13 |
  | A μ 0.3, scale 1 (참 λ RMS 0.022) | 5.5 | 18 | 0.11 |
  - 보상이 맞으면 마찰이 있어도 추종은 마찰 넣기 전과 같다(B 12.4~13.3, A 5.2, C 16.7·λ 0.074). **마찰값 ±30 % 오차는 0.5 mm 안**(판마다 흔들림 1~2 mm 보다 작음). 크게 틀린 경우(없는 마찰을 보상)만 2~4 mm 나빠진다. A/B/C 순서·결론은 그대로
  - 7판 중 1판이 시작 직후 알려진 시뮬 멈춤(0.32 s)으로 서서 다시 돌림. 마찰이 요구하는 적분 간격(무슬립 2.3 ms)은 기존 간격(0.5 ms)보다 넓어 계산량은 늘지 않음
- 확인: Python 시험 127개·C++ 12개 통과
- 남은 R2: 센서 흉내(`sim_sensors.py`) 리뷰, 시뮬 멈춤

### 2026-11-12 (6) — 리뷰 R1 (설정·메시지): 마찰 보상 몫 경고 추가
- 왜: 코드 리뷰 R1. 팀원 벤치(`amr/notes_amr/CAN/`)로 구동계 쿨롱 마찰 ≈ 0.026 N·m 가 나와 곧 `friction.*` 을 채울 텐데, `coulomb_Nm` 만 넣고 `tau_fric_max_Nm` 을 0 으로 두면 MPC·PI 상한(τ_lim − tau_fric_max)이 보상 몫을 안 남겨 τ_lim 근처에서 보상이 잘린다. 실행 중에는 "안전 클램프에 걸림" 경고만 나와 원인을 알기 어렵다. 실제로 닿는 경우는 드물다(C 는 τ_μ 0.067 이 상한, 지금 튜닝의 B 는 최대 0.15~0.26 N·m) — 설정 실수를 시작할 때 알려 주는 정도의 안전장치
- 바꾼 것: `controller/mpc/amr_mpc/nodes/torque_output.py` — `friction_reserve_warning()` 추가, 노드 시작 때 경고. `controller/mpc/test/test_torque_output.py` — 시험 1개 추가
- 리뷰에서 확인한 것 (문제 없음): C++(`torque_mpc_node`)와 Python(`core/config.py`)이 같은 규칙(환경변수 → 설치본 share)으로 같은 파일을 읽고, 설치본은 소스로 링크됨 / 기하(r 0.04, k 0.19835)가 torque_plant·plant·base.yaml·URDF 에서 같음 / C++ 힘 행렬 = Python Tinvᵀ / τ_μ·τ_lim − τ_fric,max 식 같음 / A 의 상한·변화율이 B 와 같은 값에서 나옴 / 상태 주기 ≠ Ts 면 MPC 노드가 경고 / 메시지 바퀴 순서·부호 규약이 모든 노드에서 같음
- 고치지 않은 것: `PlanarReference.msg` 주석 "normally odom"(실제는 map) — 공용 메시지라 주석만 위해 재빌드하지 않음
- 확인: Python 시험 121개·C++ 12개 통과 (ROS 환경을 source 한 뒤 — 안 하면 rclpy 쓰는 4개 파일이 수집 오류)

### 2026-11-12 (5) — 디스크 정리: amr_ws 의 NUC 빌드 폴더 삭제 (코드 변경 없음)
- 왜: 디스크 여유 2.5 GB (사용자 결정, 디스크 정리)
- 지운 것 (amr_ws 안): `build/`, `install/`, `log/` — NUC 에서 복사된 빌드 결과(CMake 캐시·스크립트 경로가 `/home/smoc/2026amr_ws`). 이 PC 는 `build_pc/`·`install_pc/` 를 쓴다. 지우기 전 install 의 모든 패키지가 `src/` 에 소스가 있는지 확인함. NUC 에 다시 올릴 때는 NUC 에서 빌드한다
- amr_ws 밖에서 지운 것: `~/ros2_jazzy`(안 쓰는 소스 빌드 ROS), `~/unicorn_ws`·conda 환경 `unicorn`(레이싱 스택 — 커밋 안 한 수정은 `~/Documents/unicorn_backup/` 에 백업), conda·pip 캐시. 여유 2.5 → 16 GB
- 확인: `amr` 환경에서 `ros2 pkg prefix amr_mpc` → install_pc, Python 시험 120개·C++ 12개 통과, `stack_status.sh` 코드 변화 없음

### 2026-11-12 (4) — 벤치 도구 설명 정정 (토크 정확도)
- 왜: `steps` 의 "응답 토크 − 명령 토크 = 작은 토크 정확도" 설명이 틀렸다. 드라이버가 명령과 응답 모두 같은 K_T 로 환산하므로 그 차이는 양자화만 보여 준다(매뉴얼 5.1)
- 바꾼 파일: `controller/mpc/tools/ak40_bench.py` — 머리주석만
- 확인: 동작 변화 없음

### 2026-11-12 (3) — 7단계: AK40-10 매뉴얼 대조 반영 (범위·응답 ID·확인 단계)
- 왜: 사용자가 CubeMars 문서 2개를 넣음 — `amr/notes_amr/ak-series-driver-manual-v1-0-18-for-ak-2-0-robotic-actuator.pdf`(5.3 MIT 프로토콜 60~68쪽), `amr/notes_amr/ak40-2410-1a-a1-drive-installation-instructions.pdf`
- 대조 결과
  - 맞음: 명령·응답 바이트 배치, 특수 명령 FC/FD/FE, 응답 8 바이트(ID·위치 16·속도 12·토크 12·온도·에러), 온도 −40, 1 Mbps, 11 비트 표준 프레임
  - 틀렸던 가정: AK40-10 범위 **V ±45.5 rad/s, T ±5 N·m** (가정 ±50, ±10). **응답 CAN ID = "0X00 + Drive ID" = 모터 자기 ID** (가정 0)
  - 매뉴얼 예제 송신식 (x−min)·2^bits/span 버림은 최댓값에서 4096 으로 넘치고 반 칸 치우친다 → 우리는 수신식(÷(2^bits−1))의 역·반올림을 유지
  - 새 사실: 에러 코드 0~6(과열·과전류·과전압·저전압·엔코더·MOSFET 과열), 공장 CAN ID 1, 상위 프로그램에서 CAN ID·CAN 끊김(timeout) 설정·서보/MIT 모드 전환, 드라이버 16~28 V(6S 22.2 V 범위 안), 설치 지침 전원 핀 색이 "−: 빨강, +: 검정" 으로 적혀 있음(관례와 반대 — 멀티미터 확인 필요)
- 바꾼 파일 (`controller/mpc/`)
  - `config/ak40.yaml` — 범위 AK40-10 값, `reply_can_id: motor`, `verified` 를 **`manual_checked: true` / `hardware_checked: false`** 둘로 나눔, `first_run_tau_limit: 0.1`, 근거 쪽수 주석
  - `amr_mpc/ak40/protocol.py` — 근거 문서·쪽수, 기본 범위 AK40-10, 에러 코드 이름, 정수 변환 설명
  - `amr_mpc/ak40/bridge.py` — `is_reply()`(응답 CAN ID 규칙 + [0] 모터 ID), `reply_can_id`
  - `amr_mpc/ak40/fake.py` — 응답을 모터 자기 ID 로 (`reply_id`)
  - `amr_mpc/nodes/ak40_can.py` — manual_checked false 면 실제 버스 거부, **hardware_checked false 면 실제 버스 토크 ±0.1 N·m 로 묶고 경고**, 모터 에러 코드를 이름과 함께 ERROR 로그
  - `amr_mpc/nodes/sim_plant_torque.py` — 가짜 모터 응답 CAN ID
  - `tools/ak40_bench.py` — manual_checked 기준, 응답 ID 규칙, MIT 모드 전환·ID 안내
  - `test/test_ak40.py` — AK40-10 값으로, 새 3개(최댓값 넘침 없음, 응답 ID 규칙, 첫 시험 토크 묶음) → 24개
- 확인: Python 단위 시험 120개 통과. CAN 전 구간 시뮬(vcan, 이름공간): B 13.0 mm·최대 τ 0.20·Δτ 0.014, A 5.4 mm·0.13·0.013 — 완주, fault 없음. 벤치 도구(모터 1개 가짜): 명령 0.010 → 0.011 N·m(+10 %, 가정 범위에서는 +22 %), DAMPING 기어감 0.056 rad/s, J 1.01e-3 맞춤, 속도가 45.5 rad/s 에서 응답 상한에 붙음
- 확인 못 함: 실물 전부 (모터 ID·부호·MIT 모드 전환·CAN 끊김 설정·응답 시간·모터 모드 밖 동작)

### 2026-11-12 (2) — 7단계: ak40_can_node (AK40-10 × 4, CAN MIT 모드) + 가짜 모터 + 벤치 도구
- 왜: 전환 지도 7단계 — 시뮬 플랜트 자리에 실차 모터 노드를 꽂는다. 위쪽 노드(MPC·A·게이트·추정기)는 그대로 두고 `/wheel_torque_cmd` ↔ `/wheel_states` 반대편만 바꾼다
- 새 파일 (`controller/mpc/`)
  - `amr_mpc/ak40/protocol.py` — MIT 모드 메시지(Mini Cheetah 규약: p 16 | v 12 | kp 12 | kd 12 | t 12, 응답 6/8 바이트, 특수 명령 FC/FD/FE). 보내는 쪽은 반올림(원본 C 의 버림은 반 칸 치우침). 위치 펼치기
  - `amr_mpc/ak40/bus.py` — SocketCAN (파이썬 내장 `socket.AF_CAN`, 설치 불필요)
  - `amr_mpc/ak40/bridge.py` — 노드 계산부(ROS 무관): ID·부호 대응, TORQUE(kp 0, kd 0, t_ff) / DAMPING(kd = damping_kd), 명령 0.2 s 끊김 → DAMPING, ±tau_peak 자르기, 응답 0.05 s 없음 → error 255, **응답 없는 모터에 0.2 s 마다 모터 모드 명령 재송신**
  - `amr_mpc/ak40/fake.py` — 가짜 AK40 4개 (MIT 법칙, 마지막 명령 유지, 모터 모드 밖이면 무응답, 장착 부호, 양자화)
  - `amr_mpc/nodes/ak40_can.py` — `ak40_can_node`. 100 Hz: 명령 4개 송신 → 응답을 최대 4 ms 모아 `/wheel_states`(stamp = 마지막 응답 수신 시각, fresh) 발행, `/ak40_can/status`(종단 지연 등). **`config/ak40.yaml` verified: false 면 실제 버스에서 시작 거부**(vcan 은 허용). 끌 때 DAMPING 0.5 s → 모터 모드 해제
  - `config/ak40.yaml` — 인터페이스, master_id, can_ids, direction, MIT 범위, 응답 길이, 온도 오프셋, 주기·감시 시간. **범위·ID·부호·응답 형식은 전부 미확인(매뉴얼·벤치로)**
  - `tools/ak40_bench.py` — 모터 1개 벤치(ROS 없음): status / torque(J_w·끝속도) / steps(τ_fric 곡선·작은 토크 정확도) / damping(서는 시간·J). max_tau 0.5 N·m·10 s 제한, Ctrl+C 면 DAMPING 뒤 해제, CSV
  - `test/test_ak40.py` — 21개 (메시지 왕복·바이트 배치·자르기·반올림 치우침 없음·특수 명령·응답 6/8 바이트·위치 펼치기·부호·DAMPING 규칙·응답 없음 에러·설정 거부·가짜 모터 왕복·DAMPING 제동·마지막 명령 유지·**0 은 표현 불가**·**늦게 켜진 모터 재무장**·C 상한 근처 양자화)
- 바꾼 파일
  - `amr_mpc/nodes/sim_plant_torque.py` — 파라미터 `can_interface`: 주면 가짜 AK40 모드(토픽 대신 CAN, 수신 스레드가 즉시 응답, `/wheel_states` 는 ak40_can_node 가 냄). 비우면 예전 그대로
  - `launch/torque_experiment.launch.py` — `plant:=sim_can`(가짜 모터 + ak40_can_node, vcan0), `plant:=real`(ak40_can_node + `imu_to_base_node`, 센서·Cartographer 는 따로 띄우라는 안내), 인자 `can_interface`
  - `setup.py` — `ak40_can_node`
- 확인
  - Python 단위 시험 117개 통과(새 21)
  - **CAN 전 구간 시뮬** (sudo 없이 `unshare -rn` 사용자 네트워크 이름공간에 vcan0 을 만들고 그 안에서 ROS 까지 실행): MPC/A → 게이트 → ak40_can_node → CAN → 가짜 모터 → CAN → /wheel_states

    | 판 | 토픽 시뮬 (참고) | CAN 전 구간 |
    |---|---|---|
    | B 무슬립 | 12.4 mm, 최대 τ 0.15, Δτ 0.013 | 12.5 mm, 0.17, 0.014 |
    | A 무슬립 | 5.2 mm, 0.063, 0.005 | 5.5 mm, 0.087, **0.010** |
    | C μ 0.3 | 16.7 mm, λ 0.074 | 15.1 mm, λ 0.040 |
  - 처음엔 B 가 참값 상태에서도 이탈했다 → 원인: 시작 때 ENTER 를 가짜 모터 수신 스레드가 뜨기 전에 보내 일부 모터만 켜짐. 실차에서도 모터가 늦게 켜지거나 전압 강하로 다시 켜지면 같은 일이 생기므로 노드에 재송신을 넣었다
  - 토크 직접 시험: 바퀴 0.02 N·m → 실제 0.022, 0.01 → 0.0122 (+22 %). T ±10 N·m 가정에서 토크 한 칸 0.0049 N·m, **12 비트 ±범위에서 0 은 표현이 안 돼** "토크 0" 은 +0.0024 N·m, "속도 0" 은 +0.012 rad/s 로 나간다 → DAMPING 중 무부하 바퀴가 약 0.1 rad/s 로 기어감(벤치용 가짜 모터에서 0.086 rad/s 확인)
  - 벤치 도구: 모터 1개 가짜(J 1e-3, 마찰 있음)로 확인 — damping 법 J 1.01e-3, torque 법 1.13e-3(마찰 포함이라 약간 큼), 작은 토크 응답 차이 +0.002 N·m 표시, steps 는 hold 가 J/b 의 3배는 돼야 정상 속도
- 확인 못 함: **실제 모터·CANable 전부**(통신, 응답 시간, 100 Hz 유지, 범위·ID·부호·응답 형식, 모터 모드 밖 동작, 명령 끊김 때 모터 동작), `plant:=real` 센서 묶음(실차 필요). 종단 지연(`/ak40_can/status`)은 기록만 하고 값은 안 봤다
- 시뮬 환경: 약 1/4 판에서 플랜트가 0.2~0.3 s 멈춰 fault_monitor 가 세움(2026-11-12 (1) 과 같은 현상) — 그 판은 다시 돌렸다

### 2026-11-12 (1) — MPC 잠정 튜닝: w_delta 10, 속도 가중 ×0.1 (추정기 잡음 대응)
- 왜: 추정기 잡음만으로 B 토크가 최대 1.0 N·m·틱마다 0.13 N·m 떨렸다(필요한 것은 약 0.06). 실차 첫 주행 전 위험 제거 + B·C 비교 성립. torque_mpc_todo.md 2026-11-12 (4)
- 바꾼 파일: `controller/mpc/config/torque_mpc.yaml` — `tuning.W` 속도 [1.0, 1.0, 0.5] → [0.1, 0.1, 0.05], `tuning.w_delta` 0.01 → 10.0. 위치·방향 가중, Np·Nc·q_null·상한은 그대로. 근거 표를 주석에
- 확인 (헤드리스 시뮬, 3 × 1.5 m 8자 0.5 m/s, 잡음 1배, `tools/compare_run.py` 참값 기준, ROS_DOMAIN_ID 80~83)

  | 설정 (W 속도 배율 / w_delta) | 추종 RMS | 최대 τ | 틱간 Δτ RMS |
  |---|---|---|---|
  | 1 / 0.01 (1판) | 13.5 mm | 1.03 | 0.127 |
  | 1 / 0.3, 0.3 / 0.3, 0.1 / 0.3 | 12.7, 11.5, 13.7 | 0.31~0.46 | 0.043~0.047 |
  | 1 / 1, 0.3 / 1 | 13.3, 11.1 | 0.27~0.31 | 0.031~0.034 |
  | 1 / 3, 0.3 / 3, 0.1 / 3 | 13.4, 12.7, 12.8 | 0.21~0.26 | 0.023~0.025 |
  | **0.1 / 10 (채택)** | 13.3, 반영 뒤 기본값으로 12.4 | 0.15 | 0.013 |

  - 적재 5 kg 을 제어기가 모를 때(B): 1판 값 11.9 mm·0.70 N·m → 채택값 13.9 mm·0.24 N·m. 끝에서 hold 진입 정상
  - C μ 0.3: 채택값 16.7 mm, 참 λ RMS 0.074 (1판 값 15.0 mm, 0.077) — 상한이 지배해 거의 그대로
  - **B μ 0.3 은 어떤 값으로도 이탈**(0.3/1, 0.3/3, 0.1/10 각 2~3판). 잡음 토크가 마찰 한계 0.083 N·m 를 한 번 넘으면 바퀴가 헛돌고, B 는 그걸 멈출 장치가 없다(참값 상태면 이 트랙에서 B 도 안 미끄러진다)
  - C++ 단위 시험 12개, Python 96개 통과(설정을 읽는 시험 없음 — 수치는 시뮬로만 확인)
- 확인 못 함: 실차. 최종값은 AK40 장착 뒤 IMU 진동 잡음을 다시 재고 정한다
- 시뮬 환경 메모: 데스크톱이 바쁠 때(브라우저·카카오톡) 약 5판에 1판꼴로 Python 노드가 0.2~0.3 s 멈춰 fault_monitor 가 "끊김" 으로 세운다(남은 프로세스 없음 확인). 그 판은 버리고 다시 돌렸다

### 2026-11-11 (10) — 6단계: 비교군 A (퓨어 퍼슛 + 바퀴별 PI)
- 왜: torque_mpc_todo.md "비교군 A" — 리포에 퓨어 퍼슛 코드가 없어 새로 만든다. B·C 와 같은 기준열·상태·출력 경로·토크 상한·정지 규칙으로 공정 비교
- 새 파일 (`controller/mpc/`)
  - `amr_mpc/baseline/pure_pursuit.py` — 퓨어 퍼슛 계산부(ROS 무관). κ = 2y_l/L², L_d = clamp(0.5 s·|v_ref|, 0.15, 0.6 m). **시간 기준궤적용 변경 두 가지**: ① 룩어헤드 탐색을 "가장 가까운 점" 대신 기준 시각의 점에서 시작(멀어지는 중인 첫 원 밖 점 — 앞서 있을 때 뒤쪽 점을 겨누지 않게) ② 진행 속도 v = v_ref + k_along·e_along(시간 오차 P 보정, 앞서면 서서 기다림). 정렬 회전 구간(|v_ref| < 0.02)은 ω = ω_ref + k_heading·e_θ. 근거는 파일 머리주석
  - `amr_mpc/baseline/wheel_pi.py` — 바퀴별 PI. 역기구학 = `TorquePlant.Tinv`(드라이버와 같음), 크기·변화율 상한(B 와 같은 τ̄, Δτ̄), 조건부 적분 와인드업 방지 + 적분항 ±τ̄, 리셋
  - `amr_mpc/nodes/pure_pursuit.py` — `pure_pursuit_node`. 기준열 + `/state_estimator/odom` + `/mpc/armed` → `/pure_pursuit/cmd_vel`(TwistStamped, 상태 추정값마다 100 Hz), `/pure_pursuit/status`(앞 4개가 MPC status 꼴), `/pure_pursuit/lookahead`. 정지 조건·hold 문턱은 `torque_mpc.yaml` 을 그대로 읽는다. 정지하면 명령을 끊는다
  - `amr_mpc/nodes/wheel_pi.py` — `wheel_pi_node`. `/wheel_states` 마다(100 Hz) PI → `/wheel_pi/cmd_raw`(WheelTorqueCommand) → torque_output. 미무장·무장 뒤 새 명령 전·명령 끊김(0.05 s)·바퀴 상태 끊김(0.3 s)이면 DAMPING + 리셋
  - `config/torque_baseline_a.yaml` — 퓨어 퍼슛·PI 게인(K_p 0.03, K_i 0.15)과 근거. 상한·변화율은 -1 = B 와 같게
  - `test/test_baseline_a.py` — 16개 (원 위 κ = 1/R, 측면 오차 방향, 뒤처짐/앞섬, 제자리 회전, 기준열 끝, 8자 교차점에서 시간 가지 유지, 역기구학, PI 비례·적분·독립·상한·와인드업·리셋·dt 자르기, 무슬립·슬립 플랜트에서 계단 응답)
  - `tools/baseline_a_offline.py` — ROS 없이 A 폐루프(게인 격자 `--sweep`), `tools/compare_run.py` — 참값 기준 추종 오차·최종 토크 기록(A/B/C 같은 잣대, JSON)
- 바꾼 파일
  - `launch/torque_experiment.launch.py` — 인자 `controller:=mpc|pure_pursuit`(기본 mpc). A 면 torque_mpc_node 자리에 pure_pursuit_node + wheel_pi_node, torque_output 입력을 `/wheel_pi/cmd_raw` 로, fault_monitor 가 `/pure_pursuit/status` 를 본다. MPC 전용 인자를 주면 안 쓴다고 알림
  - `amr_mpc/nodes/fault_monitor.py` — 파라미터 `track_status_topic`(기본 `/torque_mpc/status`)
  - `setup.py` — `pure_pursuit_node`, `wheel_pi_node`
- 확인: Python 단위 시험 96개 통과(새 16). 빌드 `amr_mpc`. 헤드리스 시뮬(ROS_DOMAIN_ID 71~74, 3 × 1.5 m 8자 0.5 m/s, 추정기 잡음 1배, `compare_run.py` 로 참값 기준):

  | 판 | 참 추종 RMS / 최대 [mm] | 최대 τ [N·m] | 틱간 Δτ RMS | 참 λ RMS / 최대 | 결과 |
  |---|---|---|---|---|---|
  | A 무슬립 | 5.2 / 18.0 | 0.063 | 0.005 | 0.011 / 0.16 | 완주 |
  | B 무슬립 (w_delta 0.01) | 13.5 / 31.5 | 1.03 | 0.127 | 0.129 / 1.91 | 완주 |
  | C 무슬립 | 13.4 / 42.8 | 0.067 | 0.018 | 0.046 / 0.36 | 완주 |
  | A μ 0.3 (2판) | 5.2 / 13.7, 7.2 / 17.2 | 0.085, 0.072 | 0.005 | 0.022 / 0.13 | 완주 |
  | B μ 0.3 (2판) | — | 1.3 | — | 0.91 / 1.99 | 2판 모두 2 s 안에 30 cm 이탈 (출발 대기 중 헛돎) |
  | C μ 0.3 | 15.0 / 39.1 | 0.067 | 0.015 | 0.077 / 0.27 | 완주 |
  | (참고) B w_delta 1.0 무슬립 | 13.1 / 32.5 | 0.26 | 0.033 | 0.048 / 0.60 | 완주 |
  | (참고) B w_delta 1.0 μ 0.3 (2판) | — | 1.3 | — | 0.38 / 1.98 | 13 s·약 30 s 에 30 cm 이탈 (5단계 기록의 완주와 다름 — 잡음에 따라 갈린다) |

  - A 첫 μ 0.3 판은 무장 2.8 s 뒤 추정기 출력이 0.32 s 끊겨 fault_monitor 가 세웠다. 같은 조건 재실행 2판은 정상 — 일시적 CPU 지연으로 본다(재현 안 됨)
  - **A 가 이 시뮬에서 B·C 보다 추종·슬립 둘 다 좋다.** 이유로 보이는 것: ⓐ A 의 안쪽 루프는 바퀴 속도로 닫혀 있어(시뮬 /wheel_states 는 잡음 0) 추정기 잡음이 토크로 거의 안 간다(Δτ RMS 0.005 vs C 0.015, B 0.127) ⓑ 바퀴가 헛돌면 측정 ω 가 올라 PI 가 토크를 줄인다 — 암묵적 트랙션 제어 ⓒ 이 트랙·속도는 마찰 한계(0.083 N·m)까지 갈 일이 거의 없다. 해석은 torque_mpc_todo.md 2026-11-11 (16)
- 확인 못 함: 실차(모터 미장착), 엔코더 잡음이 있을 때의 A, 적재 5 kg, 빠른 속도·큰 트랙. 같은 조건 반복은 μ 0.3 의 A·B 만 2판
- 하네스 메모: `&` 로 띄운 launch 는 SIGINT 가 무시된 채 상속돼 Python 노드가 Ctrl+C 를 못 받는다. 헤드리스 실행은 SIGINT 를 기본값으로 되돌린 뒤 exec 해야 slip_logger 요약이 남는다. `armed:=true` 로 띄우면 센서가 오기 전에 fault_monitor 가 "한 번도 안 옴" 으로 세운다 — 띄우고 5 s 뒤 `/mpc/arm` 을 냈다

### 2026-11-11 (9) — 5단계 ③④: figure8 랩별 속도, fault_monitor (5단계 끝)
- 바꾼/새 파일 (`controller/mpc/`)
  - `amr_mpc/core/figure8.py` — `lap_speeds=[V₁, V₂, …]`: 랩마다 순항 속도, 랩 경계를 지나며 가감속 accel로 바꾸고 끝에서 정확히 선다(1 ms 격자 표). 느려지는 랩은 경계를 지나며 내려온다. 주지 않으면 기존 사다리꼴 그대로(시험으로 확인)
  - `amr_mpc/nodes/figure8_reference.py` — 파라미터 `lap_speeds`(기본 [0.0] = 안 씀), 기동 로그에 랩별 목표·최고 속도·시작 시각
  - `config/torque_track.yaml` — `lap_speeds: []`(잠정 계획 [0.5, 1.0, 1.5]는 실제 트랙 크기를 정한 뒤. 임시 3 × 1.5 m 트랙에서는 기준 부하 검사에 걸린다)
  - `amr_mpc/nodes/fault_monitor.py` — 새 노드. 무장 중 SLAM 포즈 튐 증가, 추종 오차 > `max_track_error`(0.30 m), 모터 에러, `/wheel_states`·추정기 0.3 s 끊김이면 `/mpc/arm` false(→ 게이트 DAMPING)와 `/fault_monitor/fault`(latched) 이유. 판정은 순수 함수 `check_faults`
  - `launch/torque_experiment.launch.py` — 트랙 키 `lap_speeds`(인자는 "[0.3, 0.6]" 꼴), fault_monitor(추정기 모드에서만), 인자 `max_track_error`
  - `setup.py`, `test/test_figure8.py`(랩 2개), `test/test_fault_monitor.py`(3개)
- 확인: Python 단위 시험 80개, C++ 12개 통과. 시뮬 `lap_speeds:=[0.3, 0.6, 0.45]` — 랩별 속도 구간이 나오고 추종 RMS 6~7 mm로 일정. fault_monitor 정상 주행에서 오작동 없음, 문턱을 3 mm로 낮추면 추종 오차로 걸려 게이트가 무장 해제됨
- 확인 못 함: 포즈 튐·모터 에러·끊김으로 거는 것은 단위 시험만(시뮬에서 일부러 만들지 않음). 실제 트랙 크기에서 0.5/1.0/1.5 m/s 프로파일

### 2026-11-11 (8) — 5단계 ①②: 시뮬 슬립 모델 + slip_logger
- 바꾼/새 파일 (`controller/mpc/`)
  - `amr_mpc/torque/model.py` — `SlipPlant` 추가. 바퀴 각속도를 따로 상태로 두고, 메카넘 롤러 때문에 정해진 방향 u_i = (1, s_i)/√2 로만 바닥 힘 F_i = μN·tanh(Δ_i/v_ε) 를 낸다(N = mg/4). 미끄러지지 않으면 기존 모델과 같아짐을 시험으로 확인. 메카넘 전진 최대 가속 = μg/√2
  - `amr_mpc/nodes/sim_plant_torque.py` — `surface_mu`(≤ 0이면 기존 무슬립 모델, 기본), `slip_v_eps`. 슬립 모드에서는 `/wheel_states.velocity`가 실제로 도는 바퀴 속도. 적분 간격을 마찰 시정수에 맞춰 자동으로 나눈다
  - `amr_mpc/nodes/slip_logger.py` — 새 노드. 구조도 정의 λ = (ω − ω_nom)·sgn(ω_nom)/max(|ω|, |ω_nom|, ω_ε), ω_nom = T⁻¹·(추정기 차체 속도). `/slip_logger/slip` 토픽, CSV(`log_prefix`_slip), 종료 때 바퀴별 요약. 시뮬이면 참값으로 구한 λ도 같이 남김. 주기 타이머를 둬서 종료 신호를 바로 받게 함(처음에는 메시지가 끊기면 안 깨어나 강제 종료되고 요약이 사라졌다)
  - `launch/torque_experiment.launch.py` — slip_logger 기본 실행, 인자 `surface_mu`
  - `setup.py`, `test/test_torque_model.py`(슬립 3개), `test/test_slip_logger.py`(3개)
- 확인 (시뮬 μ = 0.3, 3 × 1.5 m 8자 0.5 m/s)
  - 참값 상태로 제어하면 B·C 모두 토크 0.05 N·m로 마찰 한계(0.083) 아래 → 슬립 없음
  - 추정기 + B(w_delta 0.01): 잡음으로 튄 토크(최대 1.3 N·m)에 바퀴가 헛돌아 35 cm 이탈, 중단
  - 추정기 + B(w_delta 1.0): 참 λ RMS 0.23·최대 1.98, 추종 10.5 mm
  - 추정기 + C: 참 λ RMS 0.07·최대 0.27, 추종 14~15 mm, 토크가 상한 0.0666에 걸림 → **1판 통과시험 ③ "슬립이 상한 없을 때보다 작고, 추종은 느려지되 발산 안 함" 확인**
  - 추정기로 잰 λ RMS는 C에서 0.21(참값 0.07) — 추정 속도 잡음이 슬립 측정의 바닥

### 2026-11-11 (7) — 휠 오돔: 구조만 두고 기본 끔 (스위치로 켤 수 있게)
- 왜: 사용자 결정. 기준물을 둔 8자 트랙이면 휠 오돔 없이 측위가 될 것으로 보지만 아직 검증할 데이터가 없다(로봇 구동 불가). 필요해지면 모드만 바꿔 켤 수 있게 한다.
- 새 파일
  - `controller/mpc/amr_mpc/torque/wheel_odometry.py` — 계산부. 바퀴 각속도 → 몸체 속도(드라이버 역기구학의 역, 최소제곱) → SE(2) 정확 적분. 샘플 끊김(0.1 s 초과)은 적분하지 않음
  - `controller/mpc/amr_mpc/nodes/wheel_odometry.py` — `/wheel_states` → `/wheel_odom`(Odometry, odom → base_footprint). TF는 내지 않음
  - `controller/mpc/test/test_wheel_odometry.py` — 7개 (역기구학 일치, 영공간, 일정 속도 4가지, 원 닫힘, 끊김)
  - `perception/amr_perception_bringup/config/localization_torque_odom_2d.lua` — `localization_torque_2d.lua` + `use_odometry = true`. 슬립 중 결과가 휠 오돔 쪽으로 끌릴 수 있다는 주의 포함
- 바꾼 파일
  - `controller/mpc/config/torque_estimator.yaml` — `slam:` 절 추가: `use_wheel_odom: false`(스위치), 켰을 때/껐을 때 Cartographer lua 이름, 토픽 이름
  - `controller/mpc/launch/torque_experiment.launch.py` — 스위치를 읽어 켜져 있으면 `wheel_odometry_node`를 띄운다. launch 인자 `wheel_odom:=true/false`로 한 번만 덮어쓰기. `plant:=real`일 때 쓸 lua를 로그로 알린다(Cartographer 실제 연결은 7단계)
  - `controller/mpc/setup.py` — `wheel_odometry_node` 실행 파일
- 확인: Python 단위 시험 69개 통과. 시뮬 폐루프에서 `wheel_odom:=true`로 8자 주행 — `/wheel_odom` 이동이 참값과 최대 3 mm 차(총 9.6 m, 시뮬엔 슬립 없음). 스위치 기본값(끔)에서는 노드가 뜨지 않음을 확인
- 확인 못 함: 휠 오돔 켠 lua를 Cartographer와 함께 실제로 돌리는 것(같은 내용의 lua로 bag replay는 했다 — 2026-11-11 (6)의 "외삽 끔 + 휠 오돔 켬"). `amr_perception_bringup` 패키지는 이 PC에서 빌드되지 않는다(colcon이 파일을 못 찾음, 원래 이 PC에서 빌드한 적 없음) — 7단계에서 정리

### 2026-11-11 (6) — 토크 실험용 Cartographer 설정 (외삽 끔, 휠 오돔 끔)
- 왜: 사용자 결정 — Cartographer에서는 스캔 매칭 결과만 받고, 스캔 사이와 지연은 state_estimator의 IMU 외삽으로 채운다.
- 새 파일: `perception/amr_perception_bringup/config/localization_torque_2d.lua` — `localization_2d.lua`를 include하고 `use_pose_extrapolator = false`, `use_odometry = false`, `pose_publish_period_sec = 5e-3`만 바꿈. 기존 lua는 그대로
- 소스 확인 (cartographer_ros 2.0.9003, GitHub `ros2/cartographer_ros` `src/node.cpp` `PublishLocalTrajectoryData`): `use_pose_extrapolator = false`면 stamp = `local_slam_data->time`(스캔 시각), 포즈 = `local_pose`(스캔 매칭 결과), 같은 stamp는 다시 발행하지 않음. 이 옵션은 발행용 외삽기(`extrapolators_`)만 바꾸고 로컬 SLAM 내부 스캔 매칭에는 영향 없음. TF도 같은 stamp로 나간다
- 추정기 쪽은 이미 대응되어 있음: `config/torque_estimator.yaml`의 `pose_source: stamped`(기본)
- (같은 날 이어서) Cartographer 설치 후 bag replay로 확인 — 저장 지도 `smoc_lab.pbstream`, bag 135325(30 s), 초기 자세 = bag 첫 `/tracked_pose`, 1배속
  - lua 끝에 `return options`가 빠져 Cartographer가 읽지 못했다 → 추가. `pose_publish_period_sec` 5 ms는 0.5배속 replay에서 발행용 외삽기 IMU 시각 검사에 걸려 멈춘 적이 있어 원래 값 10 ms로 되돌림
  - | 설정 | `/tracked_pose` 발행 | 실차 기록과의 위치 차 (중앙값 / 최대) | 0.5 s 이동 − 휠 오돔 RMS |
    |---|---|---|---|
    | 지금 실차 그대로 (외삽 켬, 휠 오돔 켬) | 40 Hz(25 ms), stamp = 지금 | 25 / 50 mm | 20 mm |
    | 외삽 끔 + 휠 오돔 켬 | 스캔마다 1번, stamp = 스캔 시각, 지연 약 128 ms | 25 / 78 mm | 24 mm |
    | 외삽 끔 + 휠 오돔 끔 (`localization_torque_2d.lua`) | 스캔마다 1번, 지연 약 126 ms | **180 / 393 mm** | **94 mm** |
  - **외삽 끄기는 측위에 영향 없음, 휠 오돔 끄기가 측위를 무너뜨린다**: 휠 오돔이 없으면 오차가 시간에 따라 쌓여 30 s에 약 30 cm, 회복하지 않음(켜면 같은 구간 5 cm 이내). 실차 기록도 참값은 아니지만, 실차 설정 replay가 그것을 2.5 cm로 재현하므로 비교 기준으로 쓸 수 있다
  - 외삽 끔 + 휠 오돔 켬은 스캔 297장 중 167장만 결과가 나왔다(간격 200 ms 구간이 있음). 원인 미확인
- 아직 launch에 연결하지 않았다: 실차 연결은 7단계 (`cartographer.launch.py lua_basename:=localization_torque_2d.lua`로 쓸 수 있음)

### 2026-11-11 (5) — 4단계: state_estimator (SLAM 포즈 + IMU → 100 Hz 상태) + 시뮬 센서
- 왜: 전환 흐름 4단계. MPC가 시뮬 참값 대신 실차와 같은 꼴의 센서에서 추정한 상태를 쓰게 한다. 결정(2026-11-06): 스캔 매칭 때만 보정, 사이는 IMU 가속도 적분, 속도는 위치 잔차 보정, yaw는 자이로 상보 필터.
- 새 파일 (`controller/mpc/` 아래)
  - `config/torque_estimator.yaml` — 추정기 설정: 포즈 고르는 방식 `pose_source`(stamped / after_scan_match), IMU·포즈 기준점 위치, 잡음 → 게인, 포즈 튐 문턱
  - `amr_mpc/torque/estimator.py` — 계산부(ROS 무관). IMU 점의 [p, v]·yaw·바이어스 추정, 늦게 온 포즈를 이력으로 보정, 정상상태 칼만 게인(K_p 0.71, K_v 4.2 /s), 포즈 튐 처리, 출력은 로봇 중심으로 변환
  - `amr_mpc/torque/sim_sensors.py` — 시뮬 센서(ROS 무관). IMU 200 Hz(바이어스 + 움직일 때 커지는 진동 잡음), SLAM 10 Hz·지연 0.13 s, `slam_mode` scan_time / extrapolated
  - `amr_mpc/nodes/state_estimator.py` — 노드. `/imu/base_link` + `/tracked_pose`(+ `/scan_matched_points2`) → `/state_estimator/odom` 100 Hz, `/state_estimator/status`
  - `test/test_torque_estimator.py` — 12개
- 바꾼 파일
  - `amr_mpc/nodes/sim_plant_torque.py` — `sensors`(기본 true)면 물리 틱 200 Hz, 가짜 `/imu/base_link`·`/tracked_pose`·`/scan_matched_points2` 발행. `/wheel_states`·참값은 100 Hz 그대로
  - `launch/torque_experiment.launch.py` — `state_source`(기본 **estimator**, truth면 3단계 방식), `slam_mode`, `sensor_noise_scale` 인자. 추정기 노드 연결
  - `setup.py`(state_estimator_node), `package.xml`(sensor_msgs)
- 실제 bag 분석 (각 약 30 s — **bag 파일도 복사가 잘려 있다**, mcap 색인 없음)
  - `/tracked_pose`는 로봇 중심이 아니라 **tracking_frame(카메라 IMU 위치)의 포즈**. 중심으로 보면 제자리 회전에서 옆 속도가 45 mm/s 틀린다. 기준점 (0.098, +0.017) m로 두면 휠 오돔과의 차 50 → 16 mm/s. (지금 URDF로 계산하면 y = −0.030 — 어느 쪽이 맞는지 미확인)
  - 스캔 매칭 결과는 스캔 시각보다 **132 ms 늦게** 나온다(129~139). 스캔 매칭 직후 표본은 `/scan_matched_points2` 직후의 `/tracked_pose`로 고를 수 있다(To do 3절 항목 해결)
  - 정지 시 가속도계 바이어스 약 (−0.15, +0.13) m/s², 주행 중 진동 잡음 σ 0.95 m/s²·0.09 rad/s(200 Hz), SLAM 정지 잡음 0.7 mm·0.05°
  - 주행 bag에서 추정 속도와 휠 오돔의 차 86 mm/s RMS — 휠 오돔 자체의 약 90 ms 지연·슬립이 섞여 있어 추정 오차의 상한으로만 본다
- 시뮬 폐루프 (3 × 1.5 m 8자, 0.5 m/s, 비교군 B, d̂ 끔)
  | 상태 원천 | 참 추종 오차 RMS / 최대 | 추정 위치·속도 오차 RMS | 토크 틱간 변화 RMS / 최대 토크 |
  |---|---|---|---|
  | 참값 (3단계 방식) | 0.6 / 2.7 mm | — | 0.003 / 0.049 N·m |
  | 추정기, 잡음 없음 | 0.6 / 2.6 mm | 0.0 mm, 0.4 mm/s | 0.004 / 0.052 |
  | 추정기, 실측 수준 잡음 (w_delta 0.01) | 11.7 / 30.5 mm | 8.6 mm, 53 mm/s | 0.183 / 0.79 |
  | 〃 w_delta 0.1 | 12.6 / 32.6 mm | 8.7 mm, 53 mm/s | 0.094 / 0.48 |
  | 〃 w_delta 1.0 | 13.2 / 32.8 mm | 8.6 mm, 52 mm/s | 0.052 / 0.26 |
  | 추정기, slam_mode extrapolated (잡음 없음) | **발산** (35 cm 넘어 자동 중단) | — | 상한 1.3 |
  - 외삽 방식 발산의 원인: 추정기 자체는 안정(참값으로 제어하며 추정기만 돌리면 오차 최대 17.6 mm). 그 오차가 가속도에 비례하는 뒤처짐이라, 그 값으로 MPC 루프를 닫으면 오차 → 가속 → 더 큰 오차가 된다. w_delta 1.0으로도 발산
  - d̂를 켠 경우(추정기 사용): α = 1이면 잡음이 없어도 토크가 매 틱 변화율 상한(0.433 N·m)으로 왕복, 추종 14 mm. α = 0.05면 추종 11.1 mm·토크 변화 0.27(끈 것보다 나쁨). **추정기와 함께 쓸 때 d̂는 끄는 게 맞다**
- 확인: Python 단위 시험 62개, C++ 12개 통과
- 확인 못 한 것: 실차 Cartographer에서 `use_pose_extrapolator = false`가 실제로 "스캔 시각 stamp의 포즈를 한 번씩" 내는지(이 PC에 cartographer_ros가 없어 소스·실행 확인 못 함), SLAM 전역 최적화의 포즈 튐(시뮬 플랜트에 없음 — 단위 시험만), RViz 화면

### 2026-11-11 (4) — 8자를 지도에 고정 (출발점 검사 + 진입 구간)
- 왜: 사용자 결정. 노면을 바꿔 가며 비교하려면 8자가 매번 바닥의 같은 자리를 지나야 한다. 로봇 기준이면 로봇을 옮겨 놓은 만큼 8자 전체가 옮겨진다.
- 바꾼 파일
  - `controller/mpc/config/torque_track.yaml` — 새 파일. 트랙 단일 출처: `origin_mode`(fixed/robot), 교차점 지도 좌표 `origin_x/y/yaw_deg`(**임시 0, 0, 0°** — 실제 트랙을 지도 위에 정한 뒤 넣는다), 출발 허용 거리 30 cm·30°, 진입 속도 상한, 8자 모양·속도(3.0 × 1.5 m, 0.5 m/s — 임시)
  - `controller/mpc/amr_mpc/nodes/figure8_reference.py` — `origin_mode`(기본 robot = 예전 동작) 추가. fixed면
    - 기동 전부터 지도 고정 경로를 RViz에 보낸다
    - 무장 때 로봇이 출발점에서 허용 범위 밖이면 출발 거부(기준을 안 내므로 MPC는 DAMPING). 다시 무장하면 다시 검사
    - 범위 안이면 로봇 포즈 → 교차점의 진입 구간(스무스스텝, 평균 0.1 m/s·0.5 rad/s 이하, 최소 start_delay) 뒤 8자
    - 시간 → 기준 계산을 `ref_at()` 하나로 모음 (진입 구간 포함)
  - `controller/mpc/amr_mpc/core/figure8.py` — `lead_in()`, `lead_in_duration()` 추가
  - `controller/mpc/test/test_figure8.py` — 진입 구간 시험 3개 (양 끝 정지, 속도 = 포즈 미분, 시간 상한)
  - `controller/mpc/launch/torque_experiment.launch.py` — 트랙 인자 18개를 `torque_track.yaml`에서 읽고, launch 인자로 준 것만 덮어쓴다(인자 이름 = YAML 키)
- 확인 (시뮬, 무장 4 s 뒤)
  | 경우 | 결과 |
  |---|---|
  | 출발점 위 | 진입 3 s 뒤 8자, 오차 RMS 0.49 mm·최대 2.2 mm, 출발점으로 복귀 |
  | 18 cm·9.7° 어긋남 (x0 0.15, y0 −0.10, yaw0 10°) | 진입 구간 최대 토크 0.007 N·m·오차 0.3 mm, 8자 RMS 0.55 mm, **지도의 교차점 (0, 0)으로 복귀** |
  | 112 cm 떨어짐 | 출발 거부, MPC 한 번도 안 풂, 로봇 정지 |
  | `origin_mode:=robot`, (1.0, 0.5)에서 | 예전처럼 그 자리가 교차점, (1.0, 0.5)로 복귀 |
  - Python 단위 시험 50개 통과
  - 중간에 `ref_at()`이 자기 자신을 부르는 실수(일괄 치환)로 기준 노드가 죽은 것을 시뮬에서 잡아 고쳤다
- 참고: launch 종료(Ctrl-C) 때 torque_output·게이트가 rclpy 종료 경합 traceback을 남긴다. 종료 뒤의 일이라 동작과 무관.

### 2026-11-11 (3) — 무장해야 기준 시간이 흐르게 (게이트 막힘 중 토크 누적 문제)
- 왜: 3단계에서 발견한 안전 문제. 게이트가 막혀 있어도 기준궤적은 흐르고 MPC는 계속 풀어서, 서 있는 로봇과의 오차로 토크가 상한까지 쌓였다가 무장 순간 한꺼번에 나갔다. 사용자 결정: 무장해야 기준 시간이 흐르게.
- 바꾼 파일
  - `controller/mpc/amr_mpc/nodes/figure8_reference.py` — `wait_for_arm`(기본 false, 기존 사용처 그대로) 추가. 켜면
    - 기준 시간은 `/mpc/armed`가 true인 동안만 흐른다. 기동 포즈(8자 교차점)와 start_delay는 처음 무장한 순간부터
    - 정지하면 기준 시간이 멈추고 그 자리 정지 기준(속도 0)을 낸다
    - 무장할 때마다 배속을 0 → 1로 `resume_ramp_s`(기본 speed/accel) 동안 올린다. 발행 속도도 배속을 곱한다
    - 재무장 때는 멈춘 시각 ±`resync_window_s`(2 s) 안에서 로봇 위치에 가장 가까운 경로 시각으로 옮긴다
    - 기준열 첫 행 = 지금 (stamp = 지금 − dt, 형식 규약은 그대로). 받는 쪽이 첫 행 속도로 거꾸로 외삽하던 오차를 없앴다
  - `controller/mpc/amr_mpc/core/figure8.py` — `ramp_elapsed()`, `ramp_rate()` 추가 (배속 램프 적분)
  - `controller/mpc/test/test_figure8.py` — 램프 시험 3개 추가
  - `controller/mpc_cpp/src/torque_mpc_node.cpp` — `armed_topic`(기본 `/mpc/armed`, latched 구독). 미무장 동안 DAMPING + reset. 무장 뒤 새로 받은 기준열이 올 때까지 풀지 않음
  - `controller/mpc/launch/torque_experiment.launch.py` — figure8에 `wait_for_arm: true`, MPC에 `armed_topic` 전달. start_delay 설명 "무장 뒤"로
- 확인 (시뮬, 3 × 1.5 m 8자 0.5 m/s, launch 10 s 뒤 무장 → 8 s 주행 → 정지 4 s → 재무장)
  | 단계 | 재무장 직후 2 s 최대 토크 | 비고 |
  |---|---|---|
  | 수정 전 (배속 램프 없음) | 1.300 N·m (상한) | 순항 속도 0 → 0.5 m/s 계단 |
  | + 배속 램프 | 0.867 | 정지 중 미끄러짐 8.8 cm를 한 번에 메움 |
  | + 로봇 위치로 재동기 | 1.089 | MPC가 새 기준열 전 0.1 s를 묵은 기준으로 풂 |
  | + 무장 뒤 새 기준열 대기 | 0.406 | 첫 행 외삽이 "이미 0.1 m/s"를 요구 |
  | + 첫 행 = 지금 (최종) | **0.25~0.33** | 오차 2.5 mm·1.8°에 대한 MPC 자체 반응 (튜닝 문제) |
  - 처음 무장(10 s 늦게) 직후 최대 토크 0.008 N·m, 오차 0 — 늦게 눌러도 안전
  - 부수 효과: 평소 추종 오차 RMS 2.2 → 0.5 mm, 최대 9.6 → 2.3 mm (첫 행 외삽 오차가 평소에도 있었다)
  - w_delta 비교(재무장 토크 / 평소 RMS): 0.01 → 0.33 N·m / 0.5 mm, 0.1 → 0.19 / 0.6, 1.0 → 0.14 / 1.0. 기본값은 1판 값 0.01 유지
  - 단위 시험 Python 47, C++ 12 통과

### 2026-11-11 (2) — 3단계: torque_mpc_node + torque_output + 게이트 토크 모드 → 시뮬 폐루프
- 왜: 전환 흐름 3단계. 1판 설계(식-8~22)대로 토크 MPC를 만들고, 시뮬 플랜트와 폐루프를 닫아 1판 통과시험 ①②③을 돌린다. 2026-11-10 결정(노미널 MPC 기본, d̂·B/C는 YAML 스위치) 반영.
- 바꾼 파일 — `controller/mpc_cpp/` (C++)
  - `include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — 새 파일. ROS 무관 핵심부. 모델(G, w, 상한 식), LTV 예측(Φ를 점화식으로), 비용 H·g(식-18), DAQP 호출(변수 bound + 차분 일반 제약), 첫 스텝 상한 우선 규칙(`clamp(τ_prev ± Δτ̄, −τ̄, τ̄)`), 외란 추정치(식-9~11, 항상 계산·스위치로만 예측에 반영), 바퀴별 상한 override 자리(MTTE용)
  - `src/torque_mpc_node.cpp` — 새 파일. **상태 메시지가 올 때마다 한 번 푼다**(타이머 아님). 명령 stamp = 그 상태의 stamp. 추종 중이 아니면 항상 DAMPING(기준·상태 끊김, hold, 종료). status·기준 지평 Path·CSV 로그(d̂ 포함)
  - `test/test_torque_mpc.cpp` — 새 파일. 12개: 1판 손 계산 예(0.0275 N·m), 정지 시 T* = 0, 각도 언랩, 등속 선회 최소 노름 토크(바퀴 관성 켬·끔), 상한·변화율, 상한 급락 시 해 존재, d̂ 수치 예(7.8e-6, 0.00156), d̂ 오프셋 제거, 잘못된 설정 거부
  - `CMakeLists.txt` — `amr_torque_mpc` 라이브러리, `torque_mpc_node`, `test_torque_mpc` 추가 (기존 타깃은 그대로)
- 바꾼 파일 — `controller/mpc/` (Python)
  - `config/torque_mpc.yaml` — 새 파일. 제어기 설정: `torque_limit.mode`(motor_only = B / friction = C), `disturbance.enable`(기본 false)·alpha, 튜닝(Ts, Np, Nc, W, w_delta, q_null), safety, hold
  - `amr_mpc/nodes/torque_output.py` — 새 파일. τ_cmd = τ* + τ_fric(ω) + 안전 클램프(±정격). 받는 즉시 중계, stamp 유지, DAMPING 통과. 마찰 기본 0이라 지금은 클램프만 한다
  - `amr_mpc/nodes/keyboard_command_gate.py` — 수정. `command_type: torques` 추가(정지 신호 = MODE_DAMPING, 지금 시각 stamp). 정지 메시지 생성을 `_stop_message()`로 모음. twist·wheels 동작은 그대로
  - `amr_mpc/nodes/sim_plant_torque.py` — 수정. **실제로 흐른 시간만큼 적분**하도록 변경(고정 10 ms 적분 + 시계 stamp가 1 m/s에서 1 mm/틱 위치 튐을 만들었다)
  - `launch/torque_experiment.launch.py` — 수정. figure8 기준·MPC·torque_output·게이트 연결. 인자 추가: state_topic, use_mpc, use_figure8, armed, ctrl_payload_kg, limit_mode, disturbance, mu_hat, log_prefix, 8자 모양·속도, max_wheel_rad_per_sec(추정 42)
  - `rviz/torque_experiment.rviz` — 수정. 8자 경로·MPC 기준 지평 표시 추가
  - `test/test_command_gate.py` — 토크 모드 시험 3개 추가, `test/test_torque_output.py` — 새 파일 3개
  - `setup.py` — `torque_output_node` 실행 파일 추가
- 확인 (빌드: `--symlink-install`로 변경. YAML·Python은 재빌드 없이 반영)
  - 단위 시험: C++ 토크 12 + 기존 바퀴 11, Python 44 모두 통과
  - 시뮬 폐루프 8자(3 × 1.5 m, R_min 0.31 m, 0.5 m/s, 4 kg, B): 완주 후 hold. 위치 오차 RMS 2.2 mm·최대 9.6 mm, QP 평균 0.10 ms·최대 0.43 ms, fallback 0
  - 통과시험 ① 등속 선회(1 m/s, R 2.4 m): τ 평균 [−0.0167, +0.0166, +0.0166, −0.0167] N·m(계산 ±0.0167), 오차 평균 0.30 mm, |nᵀτ| ≈ 0
  - 통과시험 ② 적재 +5 kg: (a) 제어기도 9 kg → τ ±0.0375, 오차 0.44 mm / (b) 제어기는 4 kg·d̂ 끔 → 오차 0.51 mm, d̂ 로그에 질량 오차가 5.2e-3 m/s로 보임 / (b′) d̂ 켬 → 오차 0.32 mm이지만 토크 흔들림 8배(표준편차 0.0033 → 0.026)
  - 통과시험 ③ 마찰 상한(8자 0.6 m/s): B 최대 0.075 N·m, 오차 RMS 2.6 mm / C(μ̂ 0.3, τ̄ 0.0666) 상한에서 잘림, 2.98 mm / C(μ̂ 0.15, τ̄ 0.0333) 37 % 틱이 상한, RMS 27 mm·최대 74 mm, 발산 없이 완주
- 확인 못 한 것
  - 통과시험 ③의 "슬립 λ가 줄어든다"는 부분 — 시뮬 플랜트에 슬립(견인력 포화) 모델이 없다
  - 통과시험 ④(8자 두 바퀴 언랩)는 따로 돌리지 않았다(언랩 단위 시험만)
  - RViz 화면과 키보드 'a' 무장은 이 세션에서 직접 보지 않았다(헤드리스, armed:=true로 확인)
- 참고: launch를 Ctrl-C로 내릴 때 게이트가 KeyboardInterrupt traceback을 남긴다. 기존 게이트 종료 경로의 동작이고 토크 모드와 무관.

### 2026-11-11 (1) — 시뮬 시연 주행 스크립트 추가
- 왜: 2단계 시뮬 플랜트를 RViz로 눈으로 확인하기 위해. 토크를 계속 주면 끝없이 가속하므로, 정해진 변위만 움직이고 서는 개루프 시연이 필요했다.
- 바꾼 파일:
  - `controller/mpc/tools/torque_demo_drive.py` — 새 파일. 움직임 6개(앞 1 m, 왼쪽 게걸음 0.6 m, 반시계 90°, 대각선 0.8 m, 시계 90°, 뒤 1 m)를 "가속 T초 → 감속 T초"로 준다. 토크는 (T⁻¹)ᵀ 최소 노름 분배, 질량·관성은 `torque_plant.yaml`. 움직임 사이 DAMPING. `--loops`, `--payload_kg`, `--use_wheel_inertia`, `--pause`. 설치 대상 아님(`/usr/bin/python3 tools/...`로 직접 실행)
- 확인: 헤드리스 1회 실행 끝 위치 (0.546, 1.163, 1.9°), 계산값 (0.566, 1.166, 0°). 차이는 Python sleep 타이밍 흔들림(개루프라 보정 없음). 사용자 화면에 launch(RViz 포함)와 시연 2회를 터미널 탭으로 띄움.

### 2026-11-10 (4) — 2단계: 토크 플랜트 파라미터·모델·시뮬 플랜트·실험 launch
- 왜: 전환 흐름 2단계. 실차 ak40_can_node 자리에 같은 토픽으로 꽂히는 "가짜 로봇"을 먼저 만든다. 2026-11-10 결정(노미널 MPC 기본, 1판 추가 항은 스위치 기본 끔)을 파라미터에 반영.
- 바꾼 파일 (모두 `src/amr_basic_stack/controller/mpc/` 아래):
  - `config/torque_plant.yaml` — 새 파일. 토크 스택 파라미터 단일 출처: 기하, 질량(4 kg + 적재), I_z(추정 0.06), J_w(9.7e-4), 모터 정격·피크·damping kd(추정 0.02), 구동계 마찰(기본 0), μ̂·c_s, `model.use_wheel_inertia`(기본 false). 추정값마다 To do 측정 항목 표시
  - `amr_mpc/torque/model.py`, `amr_mpc/torque/__init__.py` — 새 파일. 1판 식-1~6 모델(Ẍ = M⁻¹(R(θ)(T⁻¹)ᵀτ + w)), 바퀴속도, 토크 상한 τ_μ(C)·τ_lim−τ_fric,max(B), RK4 적분. 시뮬 플랜트와 이후 MPC가 같이 쓴다
  - `test/test_torque_model.py` — 새 파일. 영공간, 전진·횡·회전 부호, 무슬립 바퀴속도, 바퀴 관성 스위치, 1판 수치 예(τ_μ = 0.067 N·m), 적분 정확도 — 14개
  - `amr_mpc/nodes/sim_plant_torque.py` — 새 파일. `/wheel_torque_cmd` 받아 `/wheel_states`(100 Hz), `/sim/ground_truth`(참값 Odometry), TF map→base_footprint 발행. TORQUE는 피크로 자름, DAMPING·명령 끊김(0.2 s)은 τ = −kd·ω
  - `launch/torque_experiment.launch.py` — 새 파일. 기존 autonomy.launch.py와 별개인 실험 launch. 인자 plant(sim/real — real은 7단계), payload_kg, use_wheel_inertia, x0·y0·yaw0, use_rviz
  - `rviz/torque_experiment.rviz` — 새 파일. map 고정, TF, 참값 궤적
  - `setup.py` — `sim_plant_torque_node` 실행 파일, rviz 폴더 설치 추가
  - `package.xml` — `tf2_ros` 의존 추가
- 확인
  - pytest 14개 통과. 모델 수치가 1판과 일치(μ = 2.425 kg, I_eff = I_z + 0.0954, τ_μ = 0.0666 N·m)
  - 빌드 후 launch로 띄워 손으로 토크를 줌: 전진 0.02 N·m × 2 s → x 1.000 m, vx 1.000 m/s(계산 1.0) / 명령 끊으면 timeout 후 감쇠 정지 / 반시계 토크 1.5 s → ω_z 0.992 rad/s(계산 0.99) / 영공간 토크 0.3 N·m → 몸체 정지. `/wheel_states` 99.6 Hz
  - RViz 화면은 이 세션에서 띄워 보지 않았다(헤드리스로만 확인)
- 참고: 1 m/s로 달리다 명령이 끊기면 timeout 0.2 s 동안 마지막 토크로 약 0.2 m 더 간다. 실차 ak40_can_node도 같은 규칙이라, cmd_timeout은 안전 파라미터로 다시 볼 것.
- 기준 스냅샷: 이 단계 끝에 새로 만듦(아래 "기준 시점" 표).

### 2026-11-10 (3) — 1단계: 토크 명령·바퀴 상태 메시지 추가
- 왜: 전환 흐름 1단계(인터페이스 정의). 기존 `/wheel_cmd`(Float32MultiArray)는 stamp가 없어 종단 지연을 잴 수 없고, 정지를 "토크 0"(관성 주행)과 구분할 수 없다.
- 바꾼 파일:
  - `perception/amr_interfaces/msg/WheelTorqueCommand.msg` — 새 파일. header + `mode`(TORQUE / DAMPING) + `float64[4] torque`[N·m]. 같은 타입을 `/torque_mpc/cmd_raw` → `/torque_output/cmd_raw` → `/wheel_torque_cmd` 세 구간에 쓴다. header.stamp는 상태 추정값 시각으로 끝까지 유지(지연 측정용)
  - `perception/amr_interfaces/msg/WheelStates.msg` — 새 파일. header + position / velocity / torque / temperature / error / fresh, 각 4개
  - `perception/amr_interfaces/CMakeLists.txt` — 위 두 메시지 등록
- 정한 규약: 바퀴 순서 [FL, FR, RL, RR], 부호는 바퀴 각속도와 같음(전진 시 모두 +), 모터 장착 반전은 ak40_can_node가 푼다. 상태 추정값은 새 메시지 없이 `nav_msgs/Odometry`를 쓴다.
- 확인: `ros-jazzy-vision-msgs` 설치 후 **colcon 빌드 성공**(26 s). `ros2 interface show`로 두 메시지 구조 확인, Python에서 import·상수(`MODE_DAMPING`, `RR`) 확인.
- 환경 메모 (이 PC에서 빌드하는 법)
  - `build/`·`install/`은 NUC에서 복사된 것이라(CMake 경로 `/home/smoc/2026amr_ws`) 이 PC에서 쓸 수 없다. 이 PC 빌드는 `build_pc/`·`install_pc/`로 따로 한다.
  - PATH 맨 앞에 miniforge(conda) Python이 있어서 그대로 빌드하면 `No module named 'em'`으로 실패한다. 시스템 Python을 지정한다.
  ```bash
  cd ~/Documents/amr_ws && source /opt/ros/jazzy/setup.bash
  colcon build --packages-select <패키지> --build-base build_pc --install-base install_pc --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
  source install_pc/setup.bash
  ```

### 2026-11-10 (2) — 깨진 `.git` 삭제, 코드 변경 없음
- 왜: git을 쓰지 않기로 했고 디스크가 99% 차 있었다(여유 864 MB).
- 삭제: `amr_ws/.git` (3.4 GB, pack 파일 잘림으로 손상). 사용자 확인 후 삭제. 되돌릴 수 없다.
- 결과: 디스크 여유 4.2 GB(93% 사용). `.gitignore`는 남겨 뒀다(쓰이지 않음).

### 2026-11-10 (1) — 기록 체계 시작, 코드 변경 없음
- 왜: git 손상. 사용자 결정으로 git 대신 이 파일 + 기준 스냅샷을 쓴다.
- 만든 것:
  - `amr_ws/stack_baseline/stack_status.sh` — 기준 대비 바뀐 파일 보기, `--snapshot`으로 새 기준 만들기
  - `amr_ws/stack_baseline/2026-11-10_1924_manifest.sha256`, `2026-11-10_1924_code.tar.gz` — 출발점 기준
  - 이 파일
- 코드: 바꾼 것 없음 (`stack_status.sh` 결과 수정·새 파일·삭제 모두 0 확인)
