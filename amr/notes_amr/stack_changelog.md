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

### 2026-11-28 (10) — `amrseq` 기본 회차 수: 세 경로 모두 5 → 6
- 왜: 사용자 (2026-11-28) "기본값 6 으로 바꿔줘" — μ̂ 0.60 · 배율 1.2 튜닝 뒤 세 경로 다시 돌기 전. zigzag 는 제어기마다 fwd·rev 3:3, 순서 끝에 A 로 돌아온다.
- 바꾼 파일: `mpc/tools/seq.py` `default_n` (늘 6), 도움말·`amr_aliases.sh` 표·`pitwall_usage.md`, `test_seq_plan.py`. `--n 5` 로 예전처럼.
- 확인: test_seq_plan 통과.

### 2026-11-28 (9) — MPC 기본값: μ̂ 0.33 → 0.60 (MPC 전용 키), 슬립 되먹임 배율 1.0 → 1.2
- 왜: 사용자 (2026-11-28 밤) "바꿔줘". 원 · zigzag · polyline 세 경로에서 μ̂ × 되먹임 배율을 실차로 비교했다 (`analysis/torque_mpc_real/2026-11-28_c_sfb_q4/RESULTS_KO.md` 13 절).
  (0.60, 1.2) 가 세 경로 모두 위치 RMS 에서 PP·ADPP 를 이기고 (원 3.4 · zigzag 2.2 · polyline 2.7 cm), PP·ADPP 에 p < 0.05 로 지는 지표가 없다.
- 바꾼 파일:
  - `mpc/config/torque_mpc.yaml`: 새 키 `torque_limit.mu_hat: 0.60` (MPC 전용). `slip_feedback.gain_scale` 1.0 → 1.2.
  - `mpc_cpp/src/torque_mpc_node.cpp`: `mu_hat` 파라미터 기본값을 `torque_limit.mu_hat` 에서 먼저 읽고, 없으면 예전처럼 `torque_plant.yaml surface.mu_hat`.
  - `mpc/config/pitwall_presets.yaml`: s1 · s2 · s3 의 mpc 묶음 `slip_fb_scale` 1.0 → 1.2, `mu_hat: 0.60` 추가. 그대로 두면 `pit sN` 이 배율을 1.0 으로 되돌린다.
- 안 바꾼 것: `torque_plant.yaml surface.mu_hat` 0.33. 기준 노드 부하 예산 (`fig8_limits` · zigzag 최고 속도), Python 모델, 시뮬이 읽는 값이다.
  실험 때도 `pit set mu_hat` 으로 MPC 만 바꿨으니 같은 조건이다. launch `mu_hat:=` 는 부하 예산까지 바꾸니 MPC 만 바꿀 때는 `pit set mu_hat=`.
- 확인:
  - `colcon build --packages-select amr_mpc_cpp` 성공.
  - 격리 도메인 97 에서 노드 기동 로그: μ̂ 0.60, 바닥 상한 [0.1450 0.1453 0.1584 0.1586] N·m (= 0.33 값 × 0.60/0.33), K_ω 배율 1.20, W [80, 80, 10, 2, 2, 1].
  - pytest test_seq_plan · test_launch_modes 224 통과.
  - 실차로 새 기본값 그대로 띄운 판은 아직 없다 (실험 판은 pit 로 같은 값을 넣은 것).
- 되돌리려면: `torque_limit.mu_hat` 줄을 지우고 (또는 0.33), gain_scale 1.0, 프리셋 slip_fb_scale 1.0 · mu_hat 줄 삭제. 노드는 키가 없으면 surface.mu_hat 을 쓴다.

### 2026-11-28 (8) — `amrseq`: zigzag_n 짝수 거부 (복귀 뒤 rev 회차가 다른 바닥)
- 왜: 직선 길이 후보 계산 중 발견 — 180° 복귀 뒤 rev 회차가 fwd 와 같은 바닥 선을 지나는 것은 zigzag_n 홀수일 때만 (5 · 2.5 m: 0.00 cm, 4 · 2.5 m: 최대 2.18 m). 짝수도 출발점에는 돌아온다.
- 바꾼 파일: `mpc/tools/seq.py` (짝수면 중단, `--allow-even`), `test_seq_plan.py` (n 3 · 4 · 5).
- 확인: test_seq_plan 통과. 지금까지 돈 s2 는 n 9 · 5 (홀수) 라 영향 없음.

### 2026-11-28 (7) — `pit s2` 프리셋 zigzag: 직선 9 개 × 1.0 m → 5 개 × 2.0 m
- 왜: 사용자 (2026-11-28) "zigzag_length = 2, zigzag_n = 5 로 기본값 수정". 나머지 (30° · square · pass 0.3 · accel 1.15 · 최고 = 부하 예산 · PP 0.1~0.2 · fault 0.5) 는 그대로.
- 바꾼 파일: `mpc/config/pitwall_presets.yaml` s2 (`zigzag_n` 5, `zigzag_length` 2.0, description), `test_seq_plan.py` (끝 거리 2.52 고정 단언 → 일반 단언).
  `torque_track.yaml` 기본 (zigzag_n 5 · zigzag_length 1.0) 은 안 바꿈.
- 계산 (core/zigzag.py, V_LIMIT 1.55): 기준 13.37 → 14.45 s, 최고 속력 0.765 → 1.317 m/s, 부하 55 → 85 %, 끝 (+1.536, +2.000) → (+2.536, +2.000) m (거리 3.23 m),
  경로 범위 x 0~2.54 · y 0~2.00 m (출발 자세 기준). 180° 복귀 뒤 rev 회차가 같은 바닥 선을 지나는 성질 그대로 (표본 계산 최대 0.09 cm).
- ★ 11-26 S2 (9 × 1 m) 와는 경로가 다르다 — 그 회차와 합치지 말 것. 이미 돈 s2 6 바퀴 (오늘) 도 9 × 1 m 이다.

### 2026-11-28 (6) — `amrseq` 기본 회차 수: 세 경로 모두 6 → 5
- 왜: 사용자 (2026-11-28) "그냥 s1, s2, s3 모두 5 바퀴를 기본값으로". (5) 를 되돌림 — 실제로 돈 것은 s2·s3 6 바퀴, s1 5 바퀴.
- 바꾼 파일: `mpc/tools/seq.py` `default_n` (늘 5), 도움말·`amr_aliases.sh` 표·`pitwall_usage.md`·`COMMANDS_KO.md` 문구, `test_seq_plan.py`.
- 주의: zigzag 5 바퀴 = data 15 (홀수) → 제어기마다 fwd·rev 3:2 / 2:3, 순서 끝 로봇은 A 반대편 B 에서 180° 돈 방향. 짝수는 `--n 6`.
- 확인: test_seq_plan 통과.

### 2026-11-28 (5) — `amrseq` 기본 회차 수: zigzag 6 · 그 밖 5 → 세 경로 모두 6
- 왜: 사용자 (2026-11-28) "s1, s3 도 6 회씩 실행하게 수정해줘".
- 바꾼 파일: `mpc/tools/seq.py` `default_n` (늘 6), 도움말·`amr_aliases.sh` 표·`pitwall_usage.md`·`COMMANDS_KO.md` 문구, `test_seq_plan.py`. `--n 5` 로 예전처럼.
- 확인: test_seq_plan 통과, `amrseq --dry` 는 실차 스택에서 안 돌림 (아래 메시지).

### 2026-11-28 (4) — 세 제어기 자동 순서: `controller:=all` + `controller_select_node` + `amrseq` (사람 없이 mpc → pp → adpp × N)
- 왜: 사용자 (2026-11-28) "rqt 로 파라미터 설정 후 경로 선택까지 하면 mpc → pp → adpp 순으로 5 번씩", "회차마다 사람이 안 누르고 가만히 두면 알아서",
  결정 "A launch 하나" (B 회차마다 재실행은 측위 다시 맞추기·RealSense 54 번 재기동이 순서를 끊어서), fault 면 "전체 순서 중단", zigzag "180° 회전, 제어기당 6 회차".
- 구조 (amr/notes_amr/pitwall_usage.md "세 제어기 자동 순서"): launch 가 세 제어기를 다 띄우고, 새 `controller_select_node` 가 `/mpc/armed` 를 고른 묶음에만
  (`/ctrl/mpc/armed` · `/ctrl/pp/armed`). 고르지 않은 제어기는 미무장과 같이 쉰다. `torque_output`·`fault_monitor` 는 `/ctrl/active` 가 고른 토픽만 받는다 (토크 중계 노드 없음).
- 바꾼 파일:
  - 새 `mpc/amr_mpc/core/ctrl_select.py` (family · route · parse_active · TopicSelect), 새 `mpc/amr_mpc/nodes/controller_select.py` (+ setup.py 진입점 `controller_select_node`)
  - `mpc/amr_mpc/nodes/torque_output.py` — `select_topic`·`select_inputs` (비우면 예전 그대로 `input_topic` 하나)
  - `mpc/amr_mpc/nodes/fault_monitor.py` — `select_topic`·`select_status`, `max_track_error` 실행 중 변경 (정지 중, 0.05~1.5 m). 그 밖 parameter set 은 거부 (예전에도 안 쓰였다)
  - `mpc/amr_mpc/nodes/pure_pursuit.py` — `variant_topic` (/ctrl/active → pp | adpp 룩어헤드 묶음), `adpp_lookahead_*`, 룩어헤드 여섯 값 실행 중 변경 (정지 중)
  - `mpc/amr_mpc/nodes/figure8_reference.py` — `/figure8/status` 에 `end` (done | abort — 완주와 abort_error 이탈을 가른다), `T_total`, `hold_s`
  - `mpc/launch/torque_experiment.launch.py` — `controller:=all`, `ctrl_start` (처음 고를 제어기). mpc·pp·adpp 단독 모드는 같은 노드·파라미터 (시험으로 확인)
  - `mpc/launch/real_all.launch.py` — `_PASS` 에 `ctrl_start`
  - 새 `mpc/tools/seq.py` (`amrseq`): 회차마다 제어기 고르기 → 출발점 (anchor, origin fixed) → 정지 확인 → `/mpc/arm true` → finished(done) → `/mpc/arm false`. zigzag 는 회차 뒤 180° 회전 다리. `/seq/status` 기록
  - `mpc/tools/pit.py` — PP·fault·제어기 노드 값도 (`GROUPS`), 프리셋 `pp:`·`fault:` 키. `mpc/config/pitwall_presets.yaml` 새 프리셋 s1·s2·s3 (11-26 기준 회차 params 덤프의 그 모양 값 전부 + PP 룩어헤드 + fault 한계 + 새 C MPC 값)
  - `mpc/tools/bag_runs.py` — 판마다 `controller`·`seq` (`/ctrl/active`·`/seq/status`)
  - `mpc/tools/amr_aliases.sh` — `amrall3`, `amrseq`, amrhelp 범위 6~39
  - 시험: 새 `test_ctrl_select.py`·`test_seq_plan.py`, `test_launch_modes.py` (all 6 개), `test_bag_runs.py` (2 개)
- 확인: 파이썬 시험 686 통과. `colcon build --symlink-install --packages-select amr_mpc` (새 진입점). 시뮬 (도메인 87) s2 zigzag 6 다리 · s1 circles8 3 회차 완주, 도중 `/mpc/arm false` → 중단·재무장 없음.
  bag 10 판 대조: `torque_output` 명령 100 % 고른 제어기 토픽 (다른 쪽 0), 고르지 않은 묶음 무장 0 번, L_d zigzag 0.10~0.20 · circles8 pp 0.30 · adpp 0.15~0.50 m. 프리셋 s1·s3 기준 값 = 11-26 덤프, s2 는 accel 1.15 vs 1.1500000000000001 (float 표기 차) 만.
  C++ 는 안 바꿈. 실차 미확인.
- 주의: 11-26 zigzag 의 PP·ADPP 는 둘 다 `pp_lookahead_min:=0.1 pp_lookahead_max:=0.2` 였다 → lookahead_time 0.5 가 같아 두 제어기 룩어헤드가 같았다 (s2 도 그대로 재현).
  all 모드의 PP 회차에는 MPC 노드가 떠 있다 (풀지는 않음) — 단독 PP 판과 CPU·DDS 조건이 다르다. run_info/mpstat.txt 로 볼 것.

### 2026-11-28 (4) — 기준 경로 `first_turn` (left | right): 원 8 자·8 자를 오른쪽부터
- 왜: 사용자 "원 우측 원부터 돌리게 가능해?" → "여기서 해줘". circles8·figure8 은 늘 출발 헤딩 기준 왼쪽 원(잎)부터 돌았고, 출발 방향을 돌려서는 못 바꾼다 (거울은 회전이 아님).
- 바꾼 파일:
  - `controller/mpc/amr_mpc/core/figure8.py` — `Figure8(first_turn="left")`. right 면 `at()` 에서 원점 변환 직전에 로컬 위치 y·헤딩·속도 방향·요레이트 부호를 바꾸고 (몸체 vy 도 따라서 부호가 바뀌고 vx 는 그대로), `path_xy()` 도 y 를 뒤집는다.
  - `controller/mpc/amr_mpc/core/ref_config.py` — LIVE (다음 판부터 바꿀 수 있음)·CHOICES·RELEVANT (figure8·circles8)·UI_NOTES·`make_path` 전달.
  - `controller/mpc/amr_mpc/nodes/figure8_reference.py` — 파라미터 선언 `first_turn` (기본 left) 한 줄.
  - `controller/mpc/config/torque_track.yaml` — `first_turn: left`.
  - `controller/mpc/launch/torque_experiment.launch.py` — `_TRACK_KEYS`·문자열 키·`_TRACK_CHOICES`. `real_all.launch.py` — `_PASS`.
  - `controller/mpc/tools/amr_aliases.sh` — circ 도움말 예. 새 시험 `controller/mpc/test/test_figure8_first_turn.py` (12 개).
- 확인: 수정 전 출력을 7 설정 (circles8 tangent·blend 0.6·fixed·속도 변조, gerono tangent·spin·fixed) × 원점 2 곳으로 저장해 대조 — 기본값·left 차이 0.0 (비트 같음).
  right 는 left 의 y·헤딩·vy·요레이트 부호를 바꾼 것과 최대 4.4e-16 (원점 0) · 3.1e-15 (원점 (1.3, −0.7, 0.9 rad)), 부하 (주행·정렬) 같음.
  pytest test_figure8_first_turn · test_figure8 · test_ref_config 60 통과. launch `_track` 에 넣어 보니 비움 → 'left', right → 'right', up → 기동 전 거부.
  기준 노드 단독 기동 (도메인 87, 170146 판 파라미터 + first_turn right): 파라미터 right, 실행 중 up 거부·left 받음. 실차·시뮬 주행 미확인.
- 쓰는 법: `circ width:=2.0 speed:=1.0 accel:=1.0 laps:=1 first_turn:=right` 또는 `amrall … shape:=circles8 first_turn:=right`, pit·rqt 에서 다음 판 값으로.

### 2026-11-28 (3) — 별칭 `circ`: 원 8 자 스트레스 판 한 줄
- 왜: 사용자 요청 "원도 alias 로 추가 바람 circ". 판 155544 조건.
- 바꾼 파일: `controller/mpc/tools/amr_aliases.sh` — `circ` = `amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=circles8 width:=3.0 speed:=1.232 accel:=1.5 laps:=3 bag:=true "$@"`,
  amrhelp 표 한 줄 (출력 범위 6~31 → 6~32 줄). 1.232 m/s = 반지름 0.75 m 에서 기준 부하 v (1 + 0.1932/0.75) ≤ 1.550 의 최고 (1.5 는 기준 노드가 기동 거부, 판 155528).
- 확인: `bash -n` 통과, amrall 을 echo 로 바꿔 펼친 명령 확인. 실차로 띄워 보진 않음 (155544 는 같은 값을 인자로 직접 준 판).

### 2026-11-28 (2) — 별칭 `poly`: 꺾은선 스트레스 판 한 줄
- 왜: 사용자 요청 "이거 alias 로 추가 바람 poly". 같은 날 `use_mpc:=true` 를 빠뜨린 명령 (real_all 기본 false → 기준·MPC·게이트가 안 떠 'a' 가 안 먹음, 판 154834_stress15) 을 다시 안 겪게.
- 바꾼 파일: `controller/mpc/tools/amr_aliases.sh` — `poly` = `amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=polyline accel:=1.5 square_v_max:=1.5 square_corner_mode:=pass bag:=true "$@"`,
  amrhelp 표에 한 줄 (출력 범위 6~30 → 6~31 줄).
- 확인: `bash -n` 통과, amrall 을 echo 로 바꿔 펼친 명령 확인. ros2 launch 는 같은 인자를 두 번 주면 뒤 값이 쓰인다 (작은 launch 로 `accel:=1.5 … accel:=1.0` → 1.0) — `poly accel:=1.0 bag:=pl10` 처럼 덮어쓸 수 있다.
  새 터미널 (또는 `source ~/.bashrc`) 부터 쓰인다. 실차로 띄워 보진 않음.

### 2026-11-28 — 슬립 되먹임 기본 켬: `slip_feedback.enable` false → true
- 왜: 사용자 (2026-11-28) "true 로 바꿔". 11-27 (6) 의 Q 튜닝 (W [80, 80, 10, 2, 2, 1]) 을 슬립 되먹임 (lqr, 배율 1.0) 을 켠 채로 했는데,
  yaml 기본이 꺼져 있어 인자 없이 띄우면 Q 만 새 값이고 되먹임은 꺼진 조합이 됐다. 기본값을 튜닝한 조합에 맞춘다.
- 바꾼 파일:
  - `controller/mpc/config/torque_mpc.yaml` — `slip_feedback.enable: true` (머리 주석 "기본 끔" → "2026-11-28 부터 기본 켬", ★ 경고에 friction 실차 판은 떨림·발산 없었다는 줄 추가)
  - `controller/mpc/launch/torque_experiment.launch.py` — `slip_fb` 인자 설명만 (동작 같음: 비우면 yaml, `slip_fb:=false` 면 이번 실행만 끔)
  - `controller/mpc_cpp/src/torque_mpc_node.cpp` — 머리 주석만 (빌드 안 함, 바이너리 동작 같음)
- 확인: 설치본 yaml 이 src 링크라 빌드 불필요. 다른 도메인 (87, STATIC_PEERS 비움) 에서 `torque_mpc_node` 단독 기동 → `slip_fb_enable` True · `slip_fb_scale` 1.0,
  기동 로그 "슬립 되먹임 켬", K_ω lqr 대각 0.0162, W [80, 80, 10, 2, 2, 1]. 시험 스크립트 중 이 기본값을 읽는 것은 없음 (grep). 파이썬 시험·gtest 는 안 돌림. 실차 미확인.
- 주의: 이제 인자 없이 띄운 MPC 판은 모두 되먹임이 켜진다. 옛 판 (11-27 이전, 기본 끔) 과 견줄 때는 run_info 의 `slip_fb_enable` 로 가른다.
  PP·ADPP 판은 torque_mpc_node 를 안 쓰므로 영향 없음.

### 2026-11-27 (6) — MPC 상태 무게 Q 위치 항 ×4: W [20, 20, 10, 2, 2, 1] → [80, 80, 10, 2, 2, 1]
- 왜: 사용자 (2026-11-27) "×4 로 바꿔". 슬립 되먹임 배율을 1.0 으로 정한 뒤 (11-27 (5)) R 은 고정하고 Q 만 튜닝.
  - 후보 거르기: `amr_mpc_cpp::unconstrained_gain` 을 공칭 모델에 닫아 9 후보 계산 — 위치 무게만 일정 외란의 정상 오프셋을 줄인다
    (0.1 m/s² 당 1.21 → 0.82 (×2) → 0.59 cm (×4)), 2 틱 지연 극 0.970 그대로. 속도 무게는 효과 작고, 요 속도 ×4 는 감쇠 0.33 → 0.18.
  - 실차 (friction · 슬립 되먹임 1.0 · R 고정, `analysis/torque_mpc_real/2026-11-27_q_tuning/RESULTS_KO.md`):

  | 경로 | 판 (기준 / ×4) | 위치 RMS | 큰 슬립 (> 50 %) | 헤딩 RMS |
  |---|---|---|---|---|
  | circles8 2.0 · 0.8 m/s · accel 1.0 · laps 3 | 4 / 5 | 3.41 → 2.61 cm (−23 %, Welch p 0.01) | 1.2 → 1.5 % | 0.94 → 0.99° |
  | polyline · accel 1.0 · square_v_max 0.8 · pass | 7 / 5 | 중앙값 2.64 → 2.04 (−23 %, Mann–Whitney p 0.11) · 평균 2.53 → 2.40 | 중앙값 19.8 → 17.2 % | 0.73 → 0.81° |

  circles8 은 원 안쪽 치우침 (옆 RMS 2.59 → 1.96 cm) 이 준다. polyline 은 ×4 5 판 중 4 판이 기준 7 판보다 모두 낮고 1 판 (213652) 만 슬립 26 % 로 4.44 cm.
  - 같은 날 버린 후보: 요 속도 ×0.25 `20,20,10,2,2,0.25` — 평균 개선 없음, 210605 첫 교차점에서 요 속도 −4.7 rad/s (기준 ±1.6) 넘침.
  - 왜 0.8 m/s: 1.0 m/s circles8 (R 0.5 m) 은 구심 + 롤러 저항이 메카넘 옆 힘 상한 μ̂ c_s m g / √2 = 8.35 N 의 120 % 라 Q 로 못 줄인다.
    polyline accel 1.0 = 직진 C 상한 가속 4 τ̄_min / (m_eff r) = 1.18 m/s² 의 0.85 배 (11-24 주 시나리오 정의 0.8~0.9 배).
- 바꾼 파일:
  - `controller/mpc/config/torque_mpc.yaml` — `tuning.W` (위 표를 주석으로. 예전 값: launch `mpc_W:="20,20,10,2,2,1"`)
- 확인: yaml 파싱, 파이썬 시험 308 · gtest 76 통과. 설치본 yaml 이 src 링크라 빌드 불필요. 실차에서 `mpc_W` 없이 띄운 판의 기동 로그 W 는 미확인.
- 남은 것: polyline 에서 213652 같은 슬립 큰 판이 ×4 에서 더 잦은지는 5 판으로 못 가림. 헤딩 RMS +0.05~0.08°.

### 2026-11-27 (5) — 슬립 되먹임 기본 배율 1.4 → 1.0 (0.8 m/s 에서 슬립 같고 추종 나음)
- 왜: 사용자 (2026-11-27) "0.8 속도에 1.0 배율로 돌렸음 … 슬립이 덜 발생하는 조합으로 가보자". Q 튜닝을 원이 마찰 상한 안에 드는 0.8 m/s 에서 하기로 해서
  (1.0 m/s 원은 옆 힘 필요량이 상한의 120 %, 0.8 m/s 는 80 %) 그 속도에서 1.4 와 1.0 을 비교했다.
  실차 `bags/2026-11-27/2006*~2009*_yarr_mpc` (1.4) · `2013*~2015*_yarr_mpc` (1.0) — friction · circles8 width 2.0 · speed 0.8 · accel 1.0 · blend 0.0, 각 3 판:

  | 지표 | 1.4 | 1.0 | Welch p |
  |---|---|---|---|
  | > 50 % 슬립 시간 (최대 바퀴) | 1.87 % | 0.63 % | 0.18 |
  | > 25 % | 28.1 % | 28.1 % | 0.99 |
  | 미끄럼 속도 r·e_ω RMS | 8.92 cm/s | 8.58 | 0.24 |
  | 위치 RMS | 4.70 cm | 3.29 | 0.055 |
  | 속도 RMS | 9.17 cm/s | 7.16 | 0.031 |
  | 보정 토크 Δτ RMS | 50.6 mN·m | 34.7 | 0.001 |

  슬립은 차이 없음 (1.0 쪽이 조금 적음), 추종·명령 떨림은 1.0 이 낫다. 0.8 m/s 에서는 큰 슬립이 1~2 % 라 되먹임 토크 대부분이 큰 슬립이 아닌 e_ω 에
  반응한 몫. 판 순서는 1.4 세 판 → 1.0 세 판 (섞지 않음, 모터 37 °C 같음).
- 바꾼 파일:
  - `controller/mpc/config/torque_mpc.yaml` — `slip_feedback.gain_scale` 1.4 → 1.0 (두 실험 근거 주석)
  - `controller/mpc/launch/torque_experiment.launch.py` — `slip_fb_scale` 설명의 기본값 1.0
- 확인: 실차 기동 로그 K_ω 대각 0.0162 (배율 1.0 판 세 개, 인자로 준 값). yaml 기본값으로 띄운 판은 미확인.

### 2026-11-27 (4) — 슬립 되먹임 기본 배율 1.0 → 1.4 (실차 배율 훑기 결과, 켬/끔 기본은 그대로 끔)
- 왜: 사용자 (2026-11-27) "1.4로 고정" — 2단계 Q·R 튜닝 전에 슬립 되먹임 배율을 정한다.
  실차 19:24~19:44 `bags/2026-11-27/19*_yarr_mpc` 21 판 (friction · circles8 width 2.0 · speed 1.0 · accel 1.0 · lqr · crossing_blend 0.0,
  매 바퀴 1.0→1.5 오름차순 3 바퀴). SLAM 튐 2 판 (193327 교차점 13 cm 튐 → 위치 오차 70 cm, 194141 39 cm 튐 → fault 정지) 빼고 19 판:

  | 배율 | > 50 % (최대 바퀴) | > 25 % | 미끄럼 속도 r·e_ω RMS | 위치 RMS | 틱간 명령 변화 RMS |
  |---|---|---|---|---|---|
  | 끔 (165139, 16:51 참고) | 60.0 % | 83.9 % | 31.6 cm/s | 6.05 cm | 6.8 mN·m |
  | 1.0 | 12.8 | 56.2 | 13.08 | 9.73 | 19.5 |
  | 1.4 | 13.0 | 46.5 | 11.21 | 12.84 | 26.0 |
  | 1.5 | 14.0 | 44.1 | 10.99 | 13.31 | 28.4 |

  배율 회귀 (19 판): 미끄럼 속도 기울기 p < 0.001, > 25 % p 0.001, 위치 RMS p < 0.001 (나빠짐), > 50 % p 0.32 (차이 없음 — 남은 큰 슬립은 거의 RR 헛돎).
  1.0 판 세 개 미끄럼 속도 13.06~13.10 (모터 35→38 °C) 이라 추세는 판 순서가 아닌 배율 몫. 1.4 는 1.5 와 슬립 같고 위치·떨림 덜함.
  1.4 는 2 틱 지연 발산 배율 3.40 의 41 %.
- 바꾼 파일:
  - `controller/mpc/config/torque_mpc.yaml` — `slip_feedback.gain_scale` 1.0 → 1.4 (근거 주석). `enable: false` 는 그대로 → 쓰려면 `slip_fb:=true`
  - `controller/mpc/launch/torque_experiment.launch.py` — `slip_fb_scale` 인자 설명의 기본값 1.4
- 확인: yaml 파싱, 파이썬 launch·슬립 시험 231 통과. 노드는 기동 때 yaml 을 읽으므로 (`torque_mpc_node.cpp` slip_fb_scale 기본값) 빌드 불필요 (설치본이 src 링크).
  실차에서 `slip_fb_scale` 없이 띄운 판의 기동 로그 K_ω 대각 0.0227 은 미확인.

### 2026-11-27 (3) — 슬립 되먹임을 바퀴 공간으로: Δτ = −K_ω (ω − ω_nom), K_ω 4×4 (LQR 기본 · 역기구학 선택)
- 왜: 사용자 (2026-11-27) "몸체 속도 차이가 아니라 바퀴 각속도 차이 −K_ω Ω, 4×4 K 가 필요. 역기구학이든 최적화 문제로든 K 를 다시 찾아 구현".
  (2) 의 몸체 오차 e = R(δ0)Tω − x_e 는 바퀴 오차 e_ω = ω − T⁻¹R(δ0)ᵀ x_e 를 R(δ0)T (3×4) 로 줄인 것이라, 역기구학으로 옮긴
  K_ω = K_v R(δ0) T 는 4×4 이지만 rank 3 — 앞·뒤 쌍 [1, 1, −1, −1] 방향 미끄럼에 0 을 내고, 바퀴 하나의 미끄럼이 이웃 바퀴 토크를 ∓0.0023 씩 바꾼다
  (대각 0.0043). 11-27 r2·r3 bag 재생: 이 식으로 다시 만든 Δτ 가 기록값과 상관 0.9996·0.9965 (중앙 차 0) — 지금 구현과 같은 것임을 확인.
- 식 (torque_mpc.hpp `slip_feedback_gain` 주석):
  - ω_nom = J x_e, J = T⁻¹R(δ0)ᵀ = force_map()ᵀ (kinematic–static duality), R(δ0)·T·J = I.
  - `gain: lqr` (기본) — 바퀴 미끄럼 모델 (single-corner model 을 네 바퀴로): J_w ω̇ = τ − τ_g, M_b v̇ = Jᵀ τ_g, τ_g = C e_ω
    → ė_ω = J_w⁻¹ Δτ − (J_w⁻¹ I + J M_b⁻¹ Jᵀ) C e_ω. ZOH (Ts 0.01) → DARE, 비용 Σ q_slip ‖r e_ω‖² + r_dtau ‖Δτ‖² (q_slip 2 = MPC q_ẋ, r_dtau 10 = MPC r_U).
    `grip_speed_mps` 0 (기본) = 바닥 힘이 포화된 미끄러지는 중 (C = 0): 바퀴끼리 안 엮여 대각 0.0162 N·m/(rad/s), 닫힌 고리 극 0.818 (시정수 50 ms).
    > 0 = 그 속도에서 붙어 구르는 모델, C_i = mu_slope·N_i·r²/(√2·v) (μ–λ 첫 기울기 1.26): 1 m/s 에서 대각 0.0049/0.0045, 이웃 ±0.0013 (엮인 4×4).
  - `gain: ik` — K_ω = K_v R(δ0) T. (2) 와 Δτ 가 똑같다 (gtest 로 1e-12).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — `SlipFbGain`·`SlipFbDesign`·`parse_slip_fb_gain`·`wheel_nominal_map`·`slip_feedback_gain`
    (8×8 행렬지수 ZOH 는 discretize_zoh 와 같은 방식, DARE 는 Riccati 반복).
  - `mpc_cpp/src/torque_mpc_node.cpp` — `/wheel_states` ω 를 저장, 매 주기 e_ω = ω − J x_e, Δτ = −K_ω e_ω. 기동 로그에 K_ω 4×4.
    파라미터 `slip_fb_gain`·`slip_fb_q_slip`·`slip_fb_r_dtau`·`slip_fb_grip_speed`·`slip_fb_mu_slope`.
    `/torque_mpc/slip_fb` 맨 뒤에 e_ω ×4 덧붙임 (앞 열 위치 그대로 — 몸체 e ×3 는 기록용으로 남김), CSV 맨 뒤 `sfb_ew_fl..rr`.
  - `mpc/config/torque_mpc.yaml` `slip_feedback`: gain lqr, q_slip 2.0, r_dtau 10.0, grip_speed_mps 0.0, mu_slope 1.26 (enable false·gain_scale 1.0 그대로).
  - `mpc/launch/torque_experiment.launch.py` 인자 `slip_fb_gain:=lqr|ik`, `real_all.launch.py` 전달 목록.
  - `mpc_cpp/test/test_torque_mpc.cpp` — `SlipFbIkEqualsBodyForm`, `SlipFbLqrSlidingIsScalarLqr` (스칼라 닫힌 해와 1e-10), `SlipFbLqrGripCoupled` (Python scipy 값).
- 확인:
  - gtest 76/76 통과. 노드 단독 기동 로그 K_ω: lqr 대각 0.0162 / ik FL [+0.0043 −0.0023 +0.0023 −0.0004] / grip 1 m/s FL [+0.0049 +0.0013 −0.0013 +0.0003].
  - 시뮬 circles8 (width 2.0 · speed 1.0 · accel 1.0 · motor_only, 실차 sfb 판과 같은 조건) 1 판씩 — 끔 / lqr 배율 0.25 / lqr 배율 1.0:
    낸 τ = clamp(τ_MPC + Δτ) 차 0, Δτ = −K_ω e_ω (로그 K_ω 4 자리 반올림 안). 배율 1.0 에서도 떨림 없음 (Δτ RMS 0.016~0.018, 최대 0.064 N·m).
    위치 RMS 1.39 / 1.82 / 1.58 cm. 시뮬은 끈 판도 큰 슬립이 없어 (> 50 % 네 바퀴 중 최대 0.0 %) 슬립을 줄이는지는 시뮬로 판단 못 한다.
    시뮬 e_ω 에는 앞·뒤 쌍 방향 성분이 0 % — 실차 r1~r3 은 e_ω 제곱합의 16~19 % (예전 rank 3 이득이 0 을 내던 몫).
  - 실차 미확인.
- 주의: lqr 배율 1 을 11-27 실차 circles8 r2·r3 (motor_only) bag 의 e_ω 에 대 보면 Δτ RMS 0.05~0.14 N·m, 최대 0.84 — 바닥 상한 τ_μ (0.080·0.087) 의 몇 배.
  e_ω 에는 추정기 지연 (바퀴보다 40 ms 안팎) 과 잡음 바닥 (~0.1 m/s = 2.5 rad/s → 배율 1 에서 0.04 N·m) 이 그대로 들어간다. 작은 배율 (0.25 안팎) 부터.
  (2) 의 rank 3 문제 (n 방향 0) 는 lqr 에서 없어지지만, 붙어 구르는 중에는 e_ω 가 대부분 잡음·지연이다.

### 2026-11-27 (2) — 슬립 되먹임: MPC 뒤 바깥 회로 τ = τ_MPC − K_v (x̂_바퀴 − x_추정기) (기본 끔)
- 왜: 사용자 결정 (2026-11-27) — tube MPC 의 ancillary feedback / 외부 슬립 제어기 꼴. MPC 최적화 안은 건드리지 않고, 풀이 뒤에 한 항을 더한다.
  K_v 는 무제약 MPC 이득 K_x (가장 nominal: θ_r = 0, 정지 기준) 의 속도 열 (ẋ, ẏ, θ̇) 4×3.
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — `unconstrained_gain(md, tu)`: K_x = [H⁻¹ΦᵀW̄Λ]₀:₄, K_τ = w_Δ[H⁻¹]₀:₄,₀:₄
    (TorqueMPC 와 같은 A, B(θ=0), move blocking, H_const). TorqueMPC 풀이에는 안 쓰인다.
  - `mpc_cpp/src/torque_mpc_node.cpp` — 기동 때 K_v = gain_scale × K_x[:, 3:6] 계산·로그. 매 주기 e = R(δ0)·T ω − (추정기 몸체 [v_x, v_y, ω_z]),
    Δτ = −K_v e, 켜면 τ = clamp(τ_MPC + Δτ, ± 모터 한계). 선회 미끄럼 보정 (slip_coef) 은 e 에 안 넣는다 (잡으려는 슬립이라).
    바퀴 값이 wheel_timeout 넘게 묵으면 Δτ = 0. MPC 의 τ_prev·d̂·제약은 τ_MPC 만 안다. `/wheel_states` 의 T ω 는 velocity_source 와 무관하게 늘 계산.
    새 토픽 `/torque_mpc/slip_fb` = [t, 켬, fresh, e×3, Δτ×4, τ_MPC×4, 낸 τ×4]. CSV 뒤에 sfb_on, sfb_fresh, sfb_evx/evy/ewz, dtau_*, tau_mpc_* 추가,
    **tau_* 와 /torque_mpc/status 의 τ 는 낸 명령** (끄면 τ_MPC 와 같음). 꺼져 있어도 e·Δτ 는 계산·기록.
  - `mpc/config/torque_mpc.yaml` — `slip_feedback: enable false, gain_scale 1.0`
  - `mpc/launch/torque_experiment.launch.py` — 인자 `slip_fb:=true|false`, `slip_fb_scale:=` (PP 판에서는 무시 알림)
  - `mpc/launch/real_all.launch.py` — 넘기는 인자 목록에 `slip_fb`, `slip_fb_scale` (안 넣으면 `amrall slip_fb:=true` 가 전달되지 않는다)
  - `mpc_cpp/test/test_torque_mpc.cpp` — `UnconstrainedGainMatchesSolver`: K_x 가 실제 풀이 (constrained=false) 의 finite difference 와 1e-8 안
- 확인:
  - gtest 73/73 통과. Python 배치 계산 (scratch) 과 C++ 풀이 finite difference 의 K_x·K_τ 차이 5×10⁻¹⁴.
  - 기동 로그 K_v (실차 δ0 3.1°): FL [+0.1031 −0.0925 −0.0450], FR [+0.0925 +0.1031 +0.0450], RL [+0.0925 +0.1031 −0.0450], RR [+0.1031 −0.0925 +0.0450]
    [N·m per m/s, m/s, rad/s]. 시뮬 (δ0 0): ±0.0980, ±0.0450. 몸체 wrench F·K_v = diag(9.795 N/(m/s), 9.795, 0.869 N·m/(rad/s)).
  - 시뮬 8자 (기본 인자) 1 판씩: 켬 위치 RMS 1.91 cm · 토크 RMS 41.1 mN·m · 최대 132 / 끔 2.07 cm · 42.5 · 146. e RMS (켬) v_x 0.033 · v_y 0.034 m/s · ω_z 0.060 rad/s,
    Δτ RMS 5.4 mN·m. 1 판씩이라 차이를 결론으로 쓰지 않는다. 명령 = τ_MPC + Δτ 를 토픽 값으로 확인.
  - test_launch_modes 194 통과, 5 실패는 plant:=real 경로의 CAN (can0) 설정 실패 — 로봇 CAN 이 내려가 있어서, 이번 수정과 무관.
  - 실차 미확인.
- 주의: 추정기 v_x 는 바퀴보다 90~120 ms 늦다 (11-18 bag) — 가속 중에는 슬립이 없어도 e > 0 이라 Δτ 가 가속을 깎는다.
  d̂ 는 τ_MPC 기준 잔차라 Δτ 가 오래 한쪽으로 쏠리면 그 몫을 외란으로 배워 되돌린다 (시정수 1 s).

### 2026-11-27 (1) — MPC 속도 출처 실차 A/B 스크립트 (`tools/vs_ab.sh`) · 결과
- `mpc/tools/vs_ab.sh w|e <판>`: 지금 기본값 꺾은선 통과에서 `velocity_source:=wheels` (w) / 기본 estimator (e). 코드·설정 기본값은 안 바꿨다.
- 실차 3 판씩 (`analysis/torque_mpc_real/2026-11-27_vs_ab/RESULTS_KO.md`): 바퀴 속도가 평균으로 위치 6.8 → 5.3 cm, 경로까지 3.9 → 3.1 cm, 슬립 > 50 % 27.7 → 23.8 %,
  회전 중 헤딩 최대 4.1 → 2.9° (이것만 판 사이 폭 안 겹침). 속도 출처 기본 (estimator) 은 그대로 — 사용자 결정 전.
- 판을 더 모아 (`summary_all.py`, 같은 설정 7 대 7): 토크 변화 10.4 대 11.0 (p 0.004), 경로까지 최대 7.6 대 9.4 cm (p 0.026), 위치 RMS 4.9 대 6.0 (p 0.053), 슬립·헤딩 차이 없음.
  3 판 때 "회전 중 헤딩 최대 안 겹침" 은 7 판에서 사라졌다 (2.9 대 3.3, p 0.81) — 정정. 옛 설정 (보정 없음) 에서는 바퀴·추정기 차이 없음.

### 2026-11-26 (5) — RViz 지나간 경로: SLAM 경로·점프 없는 경로 추가 (`torque_viz_node`)
- 왜: 사용자 "estimator 로 그린 경로가 실제로 없는 좌우 운동을 그린다, SLAM 으로 또는 점프 없게". bag 191922 에서 estimator 경로의 100 Hz
  표본 사이 속도 밖 이동 최대 46 mm, 경로 횡가속이 IMU 와 상관 −0.22 → SLAM 보정 계단 (analysis/torque_mpc_real/2026-11-26_trail/RESULTS_KO.md).
- 바꾼 파일:
  - 새 `mpc/amr_mpc/core/trail_fusion.py`: `slam_to_base` (tracked_pose 카메라 IMU 점 → base_footprint, r = torque_estimator.yaml imu_offset),
    `SmoothTrail` (바퀴 FK + 자이로 적분, SLAM 2차 complementary filter τ 0.5 s, 보정 연속 적용)
  - `mpc/amr_mpc/nodes/torque_viz.py`: `/torque_viz/trail_slam`·`/torque_viz/trail_smooth` 발행 (`/tracked_pose`·`/wheel_states`·`/imu/base_link` 구독), 무장 때 셋 다 비움
  - `mpc/rviz/torque_real.rviz`: SLAM(초록)·smooth(자홍) 켬, estimator 경로는 끔 (회색)
  - 시험: `test_trail_fusion.py` 4 개
- 확인: 시험 통과. bag 3 판 재생 — smooth 속도 밖 이동 최대 0.4~1.5 mm, SLAM 과 거리 RMS 0.9~2.9 cm. 다른 도메인에서 노드 + bag play 로 세 토픽 발행 확인.
  실차 RViz 에서는 미확인 (소스 링크라 다음 amrviz/amrall 부터 적용, 떠 있는 torque_viz 는 재시작 필요). 제어·추정기는 안 바뀜.

### 2026-11-26 (4) — 구동 방향 보정을 기본으로 (`torque_plant.yaml drive`: 3.1° · +0.8 °/(m/s²)), 시뮬 판은 0
- 왜: 사용자 결정 "B (3.1° · +0.8)" — (3) 끝의 후보 실차 비교에서 경로까지 RMS 2.6 · 최대 6.7 cm 로 가장 좋았다 (11-23 값 3.4 · 9.6).
- 바꾼 파일:
  - `mpc/config/torque_plant.yaml` `drive`: `yaw_offset_deg 3.1`, `slip_coef_deg_per_mps2 0.8` (근거 주석). MPC (force_map·엔코더 속도)·wheel_pi (PP·어댑티브 PP 바퀴 목표)·torque_output (δ0) 이 같이 읽는다 → 세 제어기 공정
  - `mpc/launch/torque_experiment.launch.py` `_drive_args`: plant 가 real 이 아니면 비운 인자를 0 으로 넘긴다 — 시뮬 플랜트에 이 물리가 없어서 (안 그러면 없는 어긋남을 보정해 시뮬이 밀린다). 준 값은 그대로
  - `mpc/tools/drive_ab.sh`: off 가 `drive_yaw_deg:=0 drive_slip_coef:=0` 을 직접 준다. `mpc/tools/roller_ab.sh`: 11-26 A/B 조건 유지하려고 같은 0 을 준다
  - 시험: `test_drive_offset.py` (yaml 3.1 · 0.8), `test_launch_modes.py` (실차는 안 넘김 · 시뮬은 0 · 준 값은 그대로)
- 확인: Python 628 통과. 실차 미확인 (설정은 소스 링크라 다음 amrall 부터 적용).
- ★ 예전 판과 견줄 때 (11-26 17:36 까지의 s1_* · 그 전 모든 판): `drive_yaw_deg:=0 drive_slip_coef:=0` (MPC 는 + `mpc_roller_rr:=0.0 mpc_inertia_z:=0.06`).
  같은 작업 트리를 쓰는 팀원 실차 판도 이제 세 제어기 모두 보정이 켜진 채 돈다.

### 2026-11-26 (3) — 구동 방향 보정 MPC 실차 A/B 스크립트 (`tools/drive_ab.sh`)
- 왜: 사용자 결정 "구동 방향 보정 비교" → "MPC만 비교". 롤러 항이 헤딩을 바로잡자 2 m 직선 끝 왼쪽 밀림이 5~8 cm = 2·sin(2.4° + 헤딩 오차) 로 드러났다
  (`analysis/torque_mpc_real/2026-11-25_roller_ab/RESULTS_KO.md` "그림과 남은 오차").
- `mpc/tools/drive_ab.sh off|on <판>`: 지금 MPC 기본 (롤러 항 켬) 꺾은선 통과, 켬 = `drive_yaw_deg:=2.4 drive_slip_coef:=-2.5` (11-23 측정, 기본값은 그대로 0).
- 분석 `analysis/torque_mpc_real/2026-11-26_drive_ab/` (roller_ab 스크립트 복사, 판 이름 `_dy_`).
- ★ PP·어댑티브 PP 는 이 보정을 안 받는다 — PP 와 견주는 비교에는 끔 판을 쓴다 (11-23 쓰는 법: 로봇 보정은 세 제어기 공통).
- 실차 결과 (같은 날 18:13~18:21, 정상 3 판씩, `analysis/torque_mpc_real/2026-11-26_drive_ab/RESULTS_KO.md`): 2 m 직선 끝 왼쪽 밀림 +6.8 · +6.2 → +2.1 · −0.6 cm (안 겹침),
  경로까지 RMS (SLAM) 4.0 → 2.9 cm (겹침), 슬립 > 50 % 26.0 → 23.5 % (겹침). 기본값은 그대로 0 (사용자 결정 전).
- 그 앞 18:06~18:11 세 판은 내가 띄운 시뮬 스택 3 개가 안 꺼져 NUC CPU 포화 (유휴 0.2 %) → IMU 시각 0.36 s 늦음 · SLAM 튐으로 버렸다.
- 보정값 다시 맞춤 (같은 날, `analysis/torque_mpc_real/2026-11-26_drive_ab/kin_calib.py`): 오늘 12 판 엔코더 vs SLAM 이동 방향 → 직선 +3.1°, 선회 계수 +0.8 ± 0.2 (11-23 의 2.4 · −2.5 와 다름).
  지금 설정은 왼쪽 회전 뒤 직선이 왼쪽, 오른쪽 회전 뒤 직선이 오른쪽으로 남는다 (`segment_errors.py`). `drive_ab.sh a` (2.4 · 0), `b` (3.1 · +0.8) 추가 — 실차 비교 전.
- 후보 실차 (18:30~18:37, 3 판씩): 경로까지 RMS 11-23 값 3.4 · A 2.9 · B 2.6 cm (겹침), 최대 9.6 · 7.5 · 6.7 cm (B 는 11-23 값과 안 겹침), 헤딩 A 0.81 · B 1.0°, 슬립 넷 다 겹침.
  기본값 (`torque_plant.yaml drive`) 은 아직 0 — 사용자 결정 전.

### 2026-11-26 (2) — 시뮬 플랜트 기본을 실차 식별값으로 (`torque_plant.yaml` 새 절 `sim_plant`: 롤러 0.88 N · I_z 0.03)
- 왜: 사용자 결정 "시뮬 맞추고". MPC 기본이 롤러 항을 켜서 (1) 시뮬 기본 플랜트 (롤러 0, I_z 0.06) 와 어긋났다. 실차 닮은 플랜트가 실차 MPC 회전 속도 비를 재현했다 (0.954 ↔ 0.958).
- 바꾼 파일:
  - `mpc/config/torque_plant.yaml`: 맨 끝 `sim_plant: {roller_rr_N: 0.88, inertia_z_kgm2: 0.03}` — sim_plant_torque.py 만 읽는다. `body.inertia_z_kgm2` 0.06 은 그대로
  - `mpc/amr_mpc/nodes/sim_plant_torque.py`: `roller_rr_N`·`inertia_z` 기본 −1 = yaml `sim_plant` (그 절이 없으면 예전 그대로 0 · body)
  - `mpc/launch/torque_experiment.launch.py`: `sim_roller_rr`·`sim_inertia_z` 비우면 −1 (yaml), 설명
  - `mpc/test/test_launch_modes.py`: 기본 −1 · yaml `sim_plant` 값 시험
- 확인: Python 628 통과. 시뮬 기본 launch 시작 로그 "I_z=0.030 … 롤러 구름 저항 0.880 N/롤러", `sim_roller_rr:=0 sim_inertia_z:=0.06` 이면 "I_z=0.060" · 롤러 없음.
- ★ 예전 시뮬 결과 (11-25 까지) 와 견줄 때는 `sim_roller_rr:=0 sim_inertia_z:=0.06`. 팀원 시뮬 판도 이제 실차 닮은 플랜트로 돈다.

### 2026-11-26 (1) — 롤러 구름 저항 MPC 항을 기본으로 켬 (0.88 N) · MPC 전용 회전 관성 0.03 (`model_terms.inertia_z_kgm2`)
- 왜: 사용자 결정 "기본으로 켜기" — 실차 A/B (2026-11-25 (5) 끝, `analysis/torque_mpc_real/2026-11-25_roller_ab/RESULTS_KO.md`).
- 값 점검 (`analysis/torque_mpc_real/2026-11-25_roller_ab/param_check.py`, 같은 6 판):
  회전 때 외란 추정이 배운 회전 가속 끔 −0.35 → 켬 −0.07 rad/s² (저항을 모를 때 다 배웠다면 −3.0~−3.8 — 관측기가 느려 1 초 회전 안에 12 % 만 따라감),
  회전 중 헤딩 오차 (+ = 더 돎) 끔 가속 끝 −1.64 · 감속 끝 −3.61 · 평균 −1.59° → 켬 +0.72 · −0.19 · −0.05°. 값은 맞다고 본다 (남은 오차가 판 사이 흔들림 크기).
- 바꾼 파일:
  - `mpc/config/torque_mpc.yaml`: `roller_rr_N: 0.88`, 새 키 `inertia_z_kgm2: 0.03` (근거·예전 동작 명령 주석)
  - `mpc_cpp/src/torque_mpc_node.cpp`: 파라미터 `inertia_z` 기본 = torque_mpc.yaml `model_terms.inertia_z_kgm2` (> 0) → 없으면 torque_plant.yaml
  - `mpc/launch/torque_experiment.launch.py`: `mpc_roller_rr`·`mpc_inertia_z` 설명 (기본값 바뀜)
  - `mpc/tools/roller_ab.sh`: off 가 `mpc_roller_rr:=0.0 mpc_inertia_z:=0.06` 을 직접 준다 (11-26 A/B 와 같은 두 설정)
  - `mpc/test/test_launch_modes.py`: yaml 기본값 시험 (0.88 · 0.03, torque_plant.yaml 0.06 그대로)
- 그대로 둔 것: `torque_plant.yaml` `inertia_z_kgm2` 0.06 (시뮬 플랜트·fig8_limits·단계 시험 도구가 같이 씀), 시뮬 플랜트 롤러 저항 기본 0.
  → 시뮬 기본 플랜트는 롤러 저항이 없어서 MPC (0.88) 와 어긋난다 — 시뮬에서 MPC 가 회전을 조금 더 돌 수 있다. 실차 닮은 시뮬: `sim_roller_rr:=0.88 sim_inertia_z:=0.03`.
- 확인: gtest 72 · Python 628 통과. 시뮬 기본 launch 의 MPC 시작 로그 "I_z 0.030 … 롤러 구름 저항 켬: 롤러당 0.880 N … 제자리 회전 1 rad/s 에서 -0.423 N·m".
- ★ 같은 작업 트리를 쓰는 팀원 MPC 판도 이제 이 항이 켜진 채 돈다. 예전과 같게 돌리려면 `mpc_roller_rr:=0.0 mpc_inertia_z:=0.06`.

### 2026-11-25 (5) — 롤러 구름 저항을 MPC·시뮬 모델에 (on/off) · MPC·시뮬 회전 관성 덮어쓰기 · 실차 A/B 스크립트
- 왜: (3)(4) 실차 식별에서 롤러 하나당 구름 저항 0.88 N 이 옆 저항 4.99 N 과 회전 저항 0.43 N·m 를 둘 다 설명. 모델에는 없었다.
  코너 제자리 회전 계산 (`analysis/torque_mpc_sim/2026-11-25_roller_plan/`): 지금 모델은 가속 국면 15.5, 감속 국면 29.1 mN·m/바퀴 만큼
  회전 방향 토크를 모자라게 계획 (관성 과대 + 저항 누락) — 실차 MPC 회전각 오차가 마지막 4 분의 1 에서 크게 느는 것과 같은 쪽.
- 사용자 결정 (같은 날): E 까지 on/off 로 구현하고 실차 비교. 값 0.88 N (측정 중앙), 켜는 판은 I_z 0.06 → 0.03 같이, 비교는 꺾은선 통과 3 판씩.
- 식: 롤러 상대 속도 앞 √2(v_y + l_x ω_z), 뒤 √2(v_y − l_x ω_z). 일률 P = F_rr Σ√(s² + s₀²), 몸체 힘 Q = −∂P/∂v. 앞 0, 옆 −4√2F, 회전 −4√2 l_x F,
  옆이 우세하면 회전 저항 0 (옆 시험에서 몸체가 돈 것과 맞다).
- 바꾼 파일:
  - `mpc_cpp` `torque_mpc.hpp/.cpp`: `TorqueModel::roller_force`, `TorqueTuning::roller_rr` (0~2 N, 기본 0)·`roller_s0` (0.02 m/s).
    a_rr = M⁻¹R(θ)Q(기준 몸체 속도) 를 a_ρ 와 같은 자리에서 U_s 에서 빼고 예측에 더함. 외란 추정 잔차의 한 스텝 예측에도 추정 상태 속도의 a_rr (두 번 보상 방지).
  - `mpc_cpp` `torque_mpc_node.cpp`: 파라미터 `roller_rr`·`roller_s0` (yaml `model_terms.roller_rr_N`·`roller_s0_mps`), 켜면 시작 로그. `inertia_z` 는 원래 있던 파라미터.
  - `mpc/config/torque_mpc.yaml`: `roller_rr_N: 0.0`, `roller_s0_mps: 0.02`
  - `mpc/amr_mpc/torque/model.py`: `RollerResistance` (바퀴 위치·T⁻¹ 행으로 롤러 상대 속도 벡터를 만드는 다른 길), `TorquePlant.step(roller=)`, `SlipPlant(roller=)`
  - `mpc/amr_mpc/nodes/sim_plant_torque.py`: 파라미터 `roller_rr_N`·`roller_s0_mps`·`inertia_z` (덮어쓰기, < 0 = yaml)
  - `mpc/launch/torque_experiment.launch.py`: `mpc_roller_rr`·`mpc_inertia_z` (제어기만), `sim_roller_rr`·`sim_inertia_z` (시뮬 플랜트). PP 에서는 무시 알림
  - `mpc/launch/real_all.launch.py`: `_PASS` 에 `mpc_roller_rr`·`mpc_inertia_z`
  - `mpc/tools/roller_ab.sh`: 실차 A/B (`off <판>` / `on <판>`, 지난 sq2p_mpcw 조건)
- 확인: gtest 72 개 (새 RR1~RR4: 식 = 일률 유한차분 (바퀴 위치로 따로 만든 롤러 속도), 0 이면 비트까지 같음, U_s 차이의 몸체 힘 = −Q, 저항 있는 플랜트에서 값이 맞으면 d̂ ≈ 0·헤딩 오차 < 1e-3 rad),
  Python 628 개 (새 롤러 3 개 — 시뮬 플랜트가 실차 회전 가속비 0.45·0.70·0.81 을 ±0.12 안에서 재현, 옆은 4.75 N (출발 직후 s₀ 몫), launch 1 개).
  끄면 (0) 지금과 같은 출력.
- 시뮬 (`analysis/torque_mpc_sim/2026-11-25_roller/RESULTS_KO.md`): 실차 닮은 플랜트 + 지금 MPC 가 실차 회전 속도 비 0.954 (실차 0.958) 를 재현, MPC 켬 한 판에서 위치 57 → 15 mm, 헤딩 2.6 → 0.41°.
- 실차 A/B 11-26 (`analysis/torque_mpc_real/2026-11-25_roller_ab/RESULTS_KO.md`, 꺾은선 통과 3 판씩): 헤딩 RMS 2.6 → 0.97°, 출발 회전 덜 돎 2.8 → 0.46°,
  회전 속도 비 0.97 → 1.00 (판 사이 폭 안 겹침). 위치 RMS 6.9 → 5.8 cm (겹침), 경로까지·슬립·토크 변화 그대로. 기본값은 아직 끔 (yaml 0) — 사용자 결정 전.

### 2026-11-25 (4) — 옆걸음 단계 토크 시험 (`floor_tests_w4_t7.sh lat`) · 분석 `floor_wheel_tests.py lat` — 롤러 구름 저항 가설 확인용
- 왜: 회전 시험 (3) 의 바닥 회전 저항 0.40 N·m 를 롤러 구름 저항으로 보면 옆 저항이 0.40 / l_x = 4.6 N (앞 0) 이어야 한다. 옆 단계 토크로 확인한다.
- 바꾼 파일 (`controller/mpc`):
  - `tools/floor_tests_w4_t7.sh lat <토크> <left|right> <판>`: 0.08·0.10·0.12 N·m, 0.3 m, 보상 끔 — 사용자 승인 (같은 날). 시간 한도는 옆 저항 5.5 N 가정 × 1.3 + 0.15 s
  - `tools/floor_wheel_tests.py lat`: Σc_y(τ − 띄운 마찰) = m_eff·a_y + F_c·sgn v_y. IMU 옆 가속을 몸체 중심으로 옮김 (− ψ̈·r_x + ψ̇²·r_y, imu_offset).
    가속 세 길 (IMU · 추정기 기울기 · 추정기 거리). `imu_rest_baseline(col=)` 인자 추가 (기본 1 = 앞, 기존 동작 그대로)
  - `launch/torque_step.launch.py`·`torque_step.py`·`torque_experiment.launch.py`: 방향 back·right 도 받음 (노드는 원래 받았고 launch 검사만 막혀 있었다)
- 확인: Python 시험 624 개 통과 (합성 옆걸음 2 개 — 저항 3 % 안, 몸체가 돌아도 IMU 위치 몫을 빼서 같은 값). 시뮬 right 0.10 → 0.31 m (0.74 s), left 0.08 → 0.30 m (0.88 s, 저항 없는 시뮬 예측 0.85 s).
- 옛 bag 1 차 (`analysis/torque_mpc_real/2026-11-25_lat_prior`, 11-15 lat1): 옆 저항 회마다 4.9~5.5 N (헛돈 0.20 N·m 빼고), 회들끼리 5.0~5.4 N — 예측 4.6 N 과 같은 크기.
- 실차 (같은 날 17:35~17:48, 0.08·0.10·0.12 × 왼쪽·오른쪽, 서 있다가 시작한 83 회, `analysis/torque_mpc_real/2026-11-25_lat/RESULTS_KO.md`):
  옆 저항 4.92~5.06 ± 0.1 N (가속 세 길), 같이 맞춘 질량 6.1~6.4 kg. 옆 저항 × l_x = 0.43 N·m = 회전 시험 범위 안 → 롤러 구름 저항 한 값 (롤러당 0.88 N) 이 둘 다 설명.
  왼쪽이 약 10 % 큼, 옆걸음 중 움직이는 쪽으로 돎 (원인 미확인). 모델 값은 안 바꿨다.

### 2026-11-25 (3) — 제자리 회전 단계 토크 시험 (`step_dir:=ccw|cw`, `step_angle_deg`) · 분석 `floor_wheel_tests.py rot`
- 왜: 실차 MPC 가 제자리 회전을 기준의 0.74~0.83 배로 덜 돈다. 회전 관성과 띄운 마찰 밖의 바닥 회전 저항을 따로 재서 모델과 견준다 (시스템 식별, Ljung 1999).
- 바꾼 파일 (`controller/mpc`):
  - `amr_mpc/nodes/torque_step.py`: 방향 `ccw`·`cw` (몸체 [0,0,±1] → 패턴 [−,+,−,+]). 회전 판은 `angle_deg` (추정기 yaw 누적) 에서 끊고,
    `distance` 는 벗어남 한도 (안 주면 0.3 m). 정지 마찰 방향용 기준에 yaw_rate 1.5 rad/s. CSV 끝에 `dyaw` 열. fwd·left 는 그대로.
  - `launch/torque_step.launch.py` (`angle:=`, 회전이면 angle 을 꼭 받음) · `launch/torque_experiment.launch.py` (`step_angle_deg`) · `launch/real_all.launch.py` (`_PASS`)
  - `tools/floor_tests_w4_t7.sh rot <토크> <ccw|cw> <판>`: 0.06·0.08·0.10 N·m, 90°, 벗어남 0.5 m, 보상 끔 — **후보 (사용자 승인 전)**
  - `tools/floor_wheel_tests.py rot`: Σc_i(τ_i − 띄운 마찰) = I_eff·ψ̈ + M_c·sgn ψ̇ + B·ψ̇ 를 IMU 자이로로 최소제곱 (표본 전부 · 회 평균 두 길, 바퀴 쪽 검산)
- 확인: Python 시험 622 개 통과 (합성 회전 3 개 — 참값 I·M_c·B 3 % 안, launch 1 개). 시뮬 ccw 0.1 → +92.5° 에서 끊김 (0.58 s), cw 0.06 → −90.8°, fwd 0.1 → 0.31 m (그대로).
- 고친 것: 처음 짠 분석은 Savitzky–Golay 미분 창이 토크를 끊은 뒤 DAMPING 감속을 걸쳐 관성을 30 % 낮게 냈다 (합성 시험이 잡음) → 켠·끊은 순간 ±0.04 s 표본 뺌.
- 옛 bag 으로 미리 본 값 (`analysis/torque_mpc_real/2026-11-25_rot_prior/floor_rot.md`, 11-15 rot1 · 11-16 rot_slow, 0.10~0.20 N·m × 0.14~0.24 s):
  바닥 회전 저항 M_c 0.43~0.45 N·m (바퀴당 22 mN·m — 띄운 쿨롱 17 보다 크다, 모델에는 0), I_eff 0.115~0.127 (모델 0.143), B 는 방법마다 0~0.21 로 안 정해짐.
  자이로/바퀴 회전 속도 0.86~0.95. 짧은 구간·높은 토크뿐이라 MPC 가 쓰는 낮은 토크·낮은 회전 속도 쪽은 새 시험으로 잰다. 모델 값은 안 바꿨다.
- 실차 (같은 날 17:16~17:23, 0.06·0.08·0.10 N·m × 반시계·시계, 54 회, `analysis/torque_mpc_real/2026-11-25_rot/RESULTS_KO.md`):
  바닥 회전 저항 0.37~0.44 N·m (바퀴당 19~23), 회전 관성 0.11~0.12 (모델 0.143), 속도 비례 몫 작음. 실측/모델 가속 0.45 · 0.70 · 0.81.
  분석 미분 창 기본 0.08 → 0.15 s (짧은 창은 미분 잡음이 관성을 깎음, errors-in-variables). 바퀴 쪽 검산은 미분 잡음으로 못 씀 → 보고서에 그렇게 적게 함.
  분석 회별 표에 "시작 회전 속도" 열 (앞 회가 덜 멈춘 채 'a').

### 2026-11-25 (2) — 지그재그 꼭짓점에서 서지 않기: `zigzag_corner:=pass` (cubic spiral 로 깎기 `zigzag_turn_m`, 꼭짓점 속도 `zigzag_corner_speed`)
- 왜: 사용자 — PP 와 MPC 를 같은 경로·속도·가속도 부하에서 비교 (PP 는 미끄러지고 MPC 는 제약을 지키며 따라가는지). 지그재그가 꼭짓점마다 서서 (쉼 0.1 s → 제자리 150° 1.10 s → 쉼 0.1 s) 주행 30.35 s 중 11.7 s (38.6 %) 를 병진 0 으로 보냈다.
  처음엔 꺾인 채 꼭짓점을 v_c 로 지나게만 했다 (`zigzag_turn_m:=0` 으로 남김) — 11-25 실차에서 PP·MPC 둘 다 못 따라감 (꼭짓점에서 속도 방향 150°·heading 계단, v_c 0.5 면 C 앞먹임 6.36 배). 그래서 꼭짓점을 곡률이 이어지게 깎는다.
- 바꾼 파일:
  - `controller/mpc/amr_mpc/core/zigzag.py` — `corner_mode` stop (기존) | pass. pass + `turn_m` > 0: 꼭짓점 앞뒤 turn_m 를 cubic spiral (Kanayama & Hartman 1989, θ(s) = θ_k + φ·(3x² − 2x³)) 로 깎음 → 양 끝 κ = 0 (ω 계단 없음), 몸체 v_y = 0·ω = vκ (퓨어 퍼슛도 낼 수 있음). turn 은 v_c 등속, 앞뒤 `dwell_s` 동안 v_c 등속 (acc_dt 창 분리). 직선은 v_c → 가속 A → 최고 → 감속 A → v_c 사다리꼴 (`_PassEdge`). path following 창 = turn 가운데 ~ turn 가운데. `turn_m` 0 이면 꺾인 채. profile square·tangent·늘 전진만.
  - `controller/mpc/amr_mpc/core/fig8_limits.py` — `zigzag_corner_speed_max` (경로 전체 C 앞먹임 ≤ max_ff_ratio·부하 예산 안 최대 v_c, 0.01 격자), `zigzag_make`·CLI 인자.
  - `controller/mpc/amr_mpc/nodes/figure8_reference.py` — 파라미터 `zigzag_corner` (stop)·`zigzag_corner_speed` (0.3)·`zigzag_turn_m` (0.5), 기동 로그.
  - `controller/mpc/launch/torque_experiment.launch.py` — 세 키, `zigzag_corner` 허용값, pass 면 `zigzag_rot_s:=max` 안 찾음, `zigzag_corner_speed:=max` 해석 (꺾인 채 + max 는 거부), max 로그에 turn κ_max·ω_max·횡가속.
  - `controller/mpc/launch/real_all.launch.py` — `_PASS` 에 세 키.
  - `controller/mpc/config/torque_track.yaml` — `zigzag_corner: stop`, `zigzag_turn_m: 0.5`, `zigzag_corner_speed: max`.
  - `controller/mpc/test/test_zigzag.py`·`test_launch_modes.py` — pass 시험 (꺾인 채·깎음 연속성·turn 끝점·유한차분·창·잘못된 값, launch → 노드·real_all·max).
- 확인: 단위 시험 605 통과. `python3 -m amr_mpc.core.zigzag` 깎은 pass 속도 vs 위치 유한차분 7e-10. 10 × 1 m, 30°, turn_m 0.5, accel max 1.15 → corner_speed max 0.30 m/s, κ_max 9.50 1/m (R 0.105 m), ω_max 2.85 rad/s, 횡가속 0.85 m/s², turn 1.38 s, 직선 최고 0.77 m/s, C 앞먹임 경로 최대 0.98, 기준 16.2 s (stop 31.9 s). 실차 미확인.

### 2026-11-25 (2) — 꺾은선 기준 `shape:=polyline` (`poly_path`, 기본 = 정사각형 두 개를 이은 8 자)
- 왜: 사용자 안정민 요청 "사각형 두개 합치는데 우(1m)-좌(1m)-좌(2m)-우(1m)-우(1m)-우(2m) 로 돌아오는 경로". "돌고 간다" 로 읽으면 닫힌다:
  출발 좌표 (0,0)→(0,−1)→(1,−1)→(1,1)→(2,1)→(2,0)→(0,0), 8 m. 정사각형 [0,1]×[−1,0] 을 왼쪽으로, [1,2]×[0,1] 을 오른쪽으로, 2 m 직선 둘이 맞닿은 점 (1,0) 을 지난다.
- 바꾼 파일:
  - `mpc/amr_mpc/core/polyline.py` (새) — `Polyline`: 명령 R/L/F<길이> (우·좌·앞 도 됨), 속도·꼭짓점·헤딩 규칙은 `core/square.py` 그대로 (`_Edge`·`_smooth` 를 가져다 씀).
    tangent 이고 첫 명령에 회전이 있으면 출발 때 제자리 회전 (쉼·rot_s·쉼). path_xy 는 2 cm 간격.
  - `mpc/amr_mpc/nodes/figure8_reference.py` — `poly_path` 파라미터, shape polyline 분기 (square_* 를 같이 씀), 부하 검사·기동 로그.
  - `mpc/launch/torque_experiment.launch.py` — `poly_path` 인자, shape 허용값에 polyline, polyline + spin 거부, 명령 줄을 launch 에서 미리 읽어 봄.
    `mpc/launch/real_all.launch.py` `_PASS` 에 `poly_path`. `mpc/config/torque_track.yaml` `poly_path: "R1,L1,L2,R1,R1,R2"`.
  - 시험: `mpc/test/test_polyline.py` (새, 12 개 — 꼭짓점·닫힘, 속도 vs 위치 유한차분, 출발 회전, stop 에서 옆 속도 0·병진과 회전 안 겹침,
    "F1.5,L1.5,L1.5,L1.5" = Square(1.5) 와 같은 기준) · `test_launch_modes.py` 1 개. pytest 618 개 통과.
- 시뮬 (`analysis/torque_mpc_sim/2026-11-25_polyline/`, accel 1.0, friction 상한): 꼭짓점 정지는 오차 RMS 21 · 최대 50 mm, 슬립 최대 0.16.
  꼭짓점 통과 (pass) 는 따라가지 못함 (최대 842 mm) — 정사각형 pass 와 같은 문제라 stop 을 쓴다.
- ★ 정정 (같은 날 오후): 아래 "pass 시뮬 최대 오차 842 mm" 는 다시 나오지 않는 한 판이었다 — 같은 조건 재실행 RMS 17 · 최대 41 mm, accel 1.5 · v_max 1.5 pass 도 MPC RMS 35 mm (`analysis/torque_mpc_sim/2026-11-25_corner_pass/`).
  기본 stop 은 그 틀린 근거로 정한 것이다 (사용자에게 알림).
  → 사용자 결정 "통과로 일단 해볼게": launch 의 "꺾은선은 안 주면 stop" 을 뺐다 — 꺾은선도 yaml square_corner_mode (pass) 그대로. pytest 618 개 통과.
- 같은 날 사용자 결정 "정지로 갈게" (퓨어 퍼슛도 이 경로를 돌림): launch 는 shape:=polyline 이고 square_corner_mode 를 안 주면 stop (정사각형은 yaml pass 그대로).
  근거: 뾰족한 꼭짓점을 서지 않고 지나면 목표 토크가 C 상한의 1.7~5 배 (꼭짓점 앞뒤 등속 구간을 넣어도 tangent 는 1.66 배 이상, 헤딩 고정은 0.1 m/s 이하에서만 0.98),
  시뮬 최대 오차 842 mm, 같은 날 지그재그 실차 (팀원) 에서 PP·MPC 둘 다 못 따라감. 최대 가속 (accel:=max 규칙) 1.15 m/s² → 직선 최고 1.07 · 1.52 m/s, 주행 19.9 s.
- 같은 시각 (14:03) 다른 사람이 지그재그 쪽으로 같은 파일 (`figure8_reference.py` · `torque_experiment.launch.py` · `real_all.launch.py` · `torque_track.yaml` · `test_launch_modes.py`) 을 고치고 있었다 —
  내 변경은 polyline 줄만 덧붙였다.

### 2026-11-25 (1) — 마찰계수 0.33 (바퀴별 값 끔) · 바닥 토크 기준 상한을 설정 기본값으로
- 왜: 사용자 안정민 결정 "기본으로 넣어". 11-24 에 정한 마찰계수 0.33 × 안전 계수 0.8 이 launch 인자 (`mu_hat_wheels:=off mu_hat:=0.33 mpc_cap_on:=ground`) 로만 쓰이고 있어,
  `limit_mode:=friction` 만 주면 예전 바퀴별 값 (상한 0.118~0.186 N·m) 이 쓰였다.
- 바꾼 파일:
  - `mpc/config/torque_plant.yaml` — `surface.mu_hat` 0.3 → 0.33, `surface.mu_hat_wheels` [0.598, 0.489, 0.547, 0.703] → [−1, −1, −1, −1] (끔 = 네 바퀴 모두 mu_hat × 하중 몫). `c_s` 0.8 그대로.
  - `mpc/config/torque_mpc.yaml` — `torque_limit.cap_on` total → ground.
  - `limit_mode` 기본은 그대로 `motor_only` — friction 상한은 `limit_mode:=friction` 을 줄 때만.
- 확인: pytest 581 개 · gtest 68 개 통과 (`test_mu_hat_ignored_warning_when_wheels_set` 을 새 기본에 맞춰 고침).
  시뮬 한 판 (`analysis/torque_mpc_sim/2026-11-24_base_config/` base_friction_1125, `limit_mode:=friction` 만 줌): 시작 로그 "τ_μ 0.0835 @ μ̂ 0.33", 바퀴별 상한 [0.0798 0.0799 0.0871 0.0872] N·m
  (하중 몫 [0.2388 0.2392 0.2608 0.2612] 때문에 앞바퀴가 작다), "마찰 상한을 바닥 토크에 건다 (cap_on ground)". fault 없이 완주 (이동 중 오차 RMS 25.5 mm, 슬립 10 % 넘는 몫 0.14 · 최대 0.24).
- 같은 날 점검 (코드 변경 없음): 기본 설정에서 켜진 기능 분류 — 추종 토크 계산은 모델 (몸체 + 바퀴 관성, w 항, 구동계 마찰, 일정 외란 상태) 과 표준 MPC 식뿐이고 슬립 쪽 덧붙임
  (η_s · 슬립 가드 · 목표 입력 사영 · 구심 보정 추정 · path following · 축 토크 보상) 은 모두 끔. 켜진 덧붙임은 외란 추정 갱신 보호 4 종 · hold 래치 · 기준 가속 창.
  11-24 실차 확인 주행 기록: 등속 구간 명령의 76~81 % 를 모델 마찰이 설명, 정지 10.1 s 동안 명령 평균 25 · 최대 105 mN·m (바퀴는 안 돎), 외란 추정이 크기 상한 0.5 m/s² 에 닿은 주기 23.3 % —
  정지 마찰이 모델에 없어서 생기는 일이고 지금은 크기 상한이 막는다.
- 남은 것: 실차 확인 없음.

### 2026-11-24 (12) — 토크 MPC 기본값: 마찰 상한 옮김 · 마찰 모델 · 외란 추정 켬 (`torque_mpc.yaml`)
- 왜: 사용자 안정민 결정 "둘다 키고 외란도 켜". (11) 로 보상 기본이 꺼지면서 인자 없이 띄운 MPC 는 마찰을 맡는 것이 하나도 없었다 (보상 · 마찰 모델 · 외란 추정 모두 끔),
  `limit_mode:=friction` 에 상한 옮김이 빠지면 제동 쪽 바닥 토크가 구동계 마찰만큼 (23~45 mN·m, 상한 83.5 의 28~54 %) 상한을 넘었다.
- ★ 외란 추정 기본 켬은 교수님 11-16 지시 ("구현하되 끈 채로") 와 다르다 — 보고할 것. 끄려면 `disturbance:=false`.
- 바꾼 파일:
  - `mpc/config/torque_mpc.yaml` — `torque_limit.friction_shift: true`, `model_terms.use_fric_model: true`, `disturbance.enable: true` (모두 false 였다). `fric_rolling_Nm` 은 0 그대로.
  - `mpc/launch/torque_experiment.launch.py` — 보상을 켠 판 (`friction_comp:=true`) 에서 상한 옮김·마찰 모델을 인자로 따로 켜지 않았으면 끈다 (yaml 기본이 켬이어도 마찰을 두 번 세지 않게, 안내 출력).
    인자로 둘을 같이 켜면 예전처럼 기동 거부. "limit_mode 가 friction 이 아니다" 안내는 인자로 켰을 때만. 인자 설명 고침.
  - `mpc_cpp/src/torque_mpc_node.cpp` — "cap_friction_shift 는 limit_mode friction 에서만" 을 경고에서 안내로 (이제 기본 상태).
  - C++ `TorqueTuning` 구조체 기본값은 그대로 (false) — gtest 는 영향 없음.
- 확인: gtest 68 개 · pytest 581 개 통과 (`test_mpc_yaml_defaults_2026_11_24` 새로, yaml 기본을 보던 시험 2 개와 구름 손실 인자 시험 고침).
  시뮬 3 판 (`analysis/torque_mpc_sim/2026-11-24_base_config/`): 마찰·외란 인자 없이 띄워 시작 로그에 "보상 없음(0) · 마찰 모델 켬 · 외란 추정치 켬", friction 상한이면 "마찰 상한 옮김 켬".
  `friction_comp:=true` 판은 둘이 꺼진다. 세 판 모두 fault 없이 완주 (한 판씩).
- 남은 것: 실차 확인 없음 (새 상한 확인 주행 — 직선 1.0 m · 1.0 m/s · 기준 가속 2.5 — 은 아직). 같은 작업 트리를 쓰는 사람의 MPC 판도 기본이 바뀐다.

### 2026-11-24 (11) — 주행 launch 의 torque_output 구동계 마찰 보상을 기본으로 끔 (`friction_comp` 기본 false)
- 왜: 사용자 안정민 결정 "기본 꺼". 11-23 부터 모든 판을 `friction_comp:=false` 로 돌리는데 기본값이 켜짐 (`torque_plant.yaml compensation_scale` 0.9) 이라 인자를 빼면 보상이 다시 켜졌다.
- 바꾼 파일:
  - `mpc/launch/torque_experiment.launch.py` — `friction_comp` 기본 `false`, 비워도 끔 (`compensation_scale` 0 을 넘긴다 — coulomb·viscous·정지 모두). `friction_comp:=true` 일 때만 yaml 0.9.
    마찰 상한 옮김·마찰 모델은 이제 `friction_comp:=false` 를 안 줘도 켜진다 (`true` 와 같이 주면 예전처럼 기동 거부).
    보상이 꺼져 있고 `limit_mode:=friction` 인데 상한 옮김이 꺼져 있으면 안내를 낸다 (바닥 토크가 구동 쪽은 구동계 마찰만큼 모자라고 제동 쪽은 그만큼 상한을 넘는다).
  - `mpc/launch/torque_step.launch.py` · `torque_ramp.launch.py` — `friction_comp` 인자 (기본 `true`) 를 real_all 로 넘긴다: amrstep · amrramp 는 "바닥 토크 τ_g 를 준다" 가 정의라
    지금 동작 (보상 켬) 그대로. 이제 `friction_comp:=false` 로 끌 수 있다.
  - `mpc/config/torque_plant.yaml` (값은 그대로 0.9, 주석만) · `mpc/amr_mpc/nodes/torque_output.py` · `mpc/launch/real_all.launch.py` (머리말만).
  - 안 바꾼 것: `teleop.launch.py` (자체 torque_output, 보상 켠 채), torque_output 노드의 파라미터 기본 (yaml 0.9).
- 확인: pytest 580 개 통과 (`test_friction_comp_default_is_off`, `test_step_and_ramp_keep_compensation_on` 새로, 기존 3 개는 기본 끔에 맞춰 고침). `ros2 launch … --show-args` 기본 `false`.
- ★ 주의: 기본 (보상 끔) 으로 `limit_mode:=friction` 을 쓰려면 `mpc_cap_friction_shift:=true` 가 같이 있어야 상한이 바닥 토크에 걸린다 — 상한 옮김·마찰 모델의 기본은 아직 끔 (사용자 결정 대기).
  비교군 PP (`controller:=pp|adpp`) 도 기본이 보상 끔이 됐다 (바퀴 PI 의 적분이 마찰을 메운다).

### 2026-11-24 (10) — 토크 MPC 마찰 모델에 구름 손실 항 (`model_terms.fric_rolling_Nm`, launch `mpc_fric_rolling`, 기본 0 = 예전 그대로)
- 왜: 바닥 등속 직진 시험 (11-24 (5)) 에서 주행 저항이 띄운 마찰보다 5 / 10 / 15 rad/s 에서 6.1 / 5.2 / 4.0 mN·m 컸다 (12 구간 3.1~7.0, 전진·후진 부호가 뒤집힘 —
  정석 바퀴 식의 rolling resistance 항 f_r·N·r·sgn(ω) 자리). 지금은 이 몫을 외란 추정이 받는다. 사용자 안정민 결정: "슬립 방지나 제어가 메인이니 보수적으로 구현".
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp` · `src/torque_mpc.cpp` — `TorqueTuning::fric_rolling` [N·m, 0~0.02]. 마찰 모델이 켜졌을 때 τ_b 에 네 바퀴 공통
    `fric_rolling·tanh(ω_r/ω_s)` 를 더해 예측·목표 입력·외란 추정 잔차에 넣는다 (`model_friction`). **상한 옮김의 f (`fs_steps_`) 에는 안 넣는다** — 상자가 이 값과 무관하다.
  - `mpc_cpp/src/torque_mpc_node.cpp` — 파라미터 `fric_rolling` (yaml `model_terms.fric_rolling_Nm`, 없으면 0), 시작 로그.
  - `mpc/config/torque_mpc.yaml` — `model_terms.fric_rolling_Nm: 0.0` (잰 값·후보 주석). `mpc/launch/torque_experiment.launch.py` — `mpc_fric_rolling` (마찰 모델 켰을 때만 넘김),
    `sim_rolling` (시뮬 플랜트). `mpc/launch/real_all.launch.py` — `_PASS` 에 `mpc_fric_rolling`.
  - `mpc/amr_mpc/torque/model.py` `DrivetrainFriction(rolling=)` · `mpc/amr_mpc/nodes/sim_plant_torque.py` `rolling_Nm` — 시뮬 플랜트 구름 손실 (기본 0, `friction_scale` 과 무관).
- 확인: gtest 68 개 통과 (새 `FrictionRolling` 4 개 — 모델 끄면 값을 줘도 비트까지 같은 출력, 지평 마찰·목표 입력에 ±값이 더해지고 상자·옮긴 양은 그대로,
  값 0.02 로 넘쳐도 낸 토크는 같은 상자 안, 잔차에 들어가 외란 추정이 0 근처), pytest 579 개 통과.
  시뮬 18 판 (`analysis/torque_mpc_sim/2026-11-24_fric_rolling/`, 플랜트 5.1 mN·m): 슬립은 값 8.0 (넘침) 에서도 안 늘었다. 상한 닿는 조건의 추종 RMS 35.8 → 28~33 mm (판 사이 폭 겹침),
  ★ 최대 오차는 2.7 · 5.1 에서 55 → 65 · 69 mm 로 커졌다 (끝점 넘어감, 원인 미확인). 앞서 플랜트 마찰 배율만 바꾼 시뮬은 `2026-11-24_fric_mismatch/`.
- ★ 찾은 것: 상한 옮김이 모르는 마찰 R 이 있으면 상한에 닿은 바퀴의 실제 바닥 토크는 구동 변 τ̄ − R, **제동 변 −(τ̄ + R)**. 이 항을 f 에 안 넣었으므로 제동 쪽은
  지금도, 이 항을 켜도 실제 구름 손실만큼 상한을 넘는다 (5.1 mN·m 면 안전 계수 0.8 → 0.85 해당). 양쪽을 다 지키려면 상자 두 변을 따로 옮겨야 한다 — 안 만들었다.
- 남은 것: 기본값은 0 그대로 (실차 A/B 뒤 사용자 결정, 후보 0.0027 = 12 구간 어디서도 안 넘침 · 0.0051 = 평균). 제동 변 조임을 만들지 사용자 결정. 실차 확인 없음.
- 2026-11-25 바로잡음 (사용자 지적 "롤러 구름손실은 횡이동에서만 작용하는 값 아닌가"): 이 항은 메카넘 **롤러가 자기 축으로 도는 손실 (롤러 구름 저항, 지시서 `use_rolling_res`) 이 아니다**.
  롤러 표면 속도 = √2 × 바퀴 중심의 옆 방향 속도 (리포 기구학으로 검산, 오차 1e-15) — 직진 0, 옆 0.2 m/s 에서 0.283 m/s, 회전 1 rad/s 에서 0.122 m/s.
  이 항은 롤러가 안 도는 직진 등속에서 잰 "주행 저항 − 띄운 마찰" 이고 원인은 가르지 못했다. 어제 hpp 머리주석에서 "롤러 구름 저항 … 넣지 않았다" 를 지운 것이 잘못 — 되살렸다 (주석만, 코드 동작 그대로).

### 2026-11-25 (10) — 지그재그 "늘 전진" `zigzag_reverse:=false` + 꼭짓점 회전 시간 `zigzag_rot_s:=max`
- 왜: 사용자 요청 — 꼭짓점에서 전진·후진을 바꾸지 말고 헤딩을 돌려 계속 전진.
- 바꾼 것:
  - `mpc/amr_mpc/core/zigzag.py` — `reverse` (기본 True = 지금까지). False 면 tangent 헤딩 θ_k = 0 | s·(π − α) (= 진행 방향), 직선마다 v_x > 0,
    꼭짓점 회전 ±(π − α) 를 짧은 쪽으로 번갈아 (헤딩이 쌓이지 않음). 진행 방향 d_k·(cos θ_k, sin θ_k) 는 같아 꼭짓점·모양은 같다. n 짝수면 끝 정렬 −(π − α).
  - `mpc/amr_mpc/core/fig8_limits.py` — `zigzag_rot_max`: 꼭짓점 회전이 경로 전체 앞먹임을 max(max_ff_ratio, 아주 느린 회전일 때 값) 위로 올리지 않고
    부하 예산 안인 가장 짧은 회전 시간 (0.05 s 격자, 이분 탐색). CLI `--zigzag_reverse`, `--zigzag_rot_s max`.
  - `mpc/launch/torque_experiment.launch.py`·`real_all.launch.py` — 인자 `zigzag_reverse`. `zigzag_rot_s:=max` 허용, 늘 전진에서 `zigzag_rot_s` 를 비우면 max
    (yaml 0.5 s 는 사이각 10° 용 — 170° 를 0.5 s 에 돌면 ω 최대 8.9 rad/s, 앞먹임 3.9 배, 부하 111 %). 기동 로그에 늘 전진·꼭짓점 회전 시간.
  - `mpc/amr_mpc/nodes/figure8_reference.py` — 파라미터 `zigzag_reverse`, 기동 로그. `mpc/config/torque_track.yaml` — `zigzag_reverse: true`, 주석.
  - `mpc/test/test_zigzag.py` (늘 전진 모양·전진·회전·끝 정렬, 유한차분 2), `test_launch_modes.py` (새 5).
- 확인: pytest 590 통과. 170° 회전 시간별 (accel 1.15, 경로 전체 앞먹임): 0.8 s 1.93 · 1.0 s 1.32 · 1.2 s 0.963 (직선 값과 같음) → `max` = 1.20 s.
  ROS 시뮬 (square 파형, fault 0): 늘 전진 C RMS 17.6 mm · 요 0.35°, A 14.6 mm · 1.88° / 전진·후진 C 16.6 mm · 0.30°, A 14.6 mm · 0.76°. 늘 전진 sine C 16.9 mm.
  주행 시간은 꼭짓점 회전이 길어 늘 전진이 3.2 s 길다 (14.9 vs 12.1 s, square).
- 함께 알게 된 것: 작업 트리의 `torque_plant.yaml` 이 다른 작업 (11-25, 커밋 안 됨) 에서 `mu_hat` 0.3 → 0.33, `mu_hat_wheels` 끔으로 바뀌어
  τ̄ = [0.0798, 0.0799, 0.0871, 0.0872] N·m → 지그재그 `accel:=max` 가 1.70 → 1.15 m/s² 다. 위 숫자는 모두 이 설정 기준.
- 쓰는 법: `amrall use_mpc:=true shape:=zigzag zigzag_reverse:=false …` (꼭짓점 회전 시간은 자동, 직접 주려면 `zigzag_rot_s:=1.5`).

### 2026-11-24 (9) — 지그재그에도 다른 경로의 기능: `yaw_mode` tangent | fixed | spin, 끝 정렬 `align_s`, `path_follow_lead`
- 왜: 사용자 요청 ("원래 다른 경로에서 쓰던 기능들 지그재그에서도 동일하게").
- 바꾼 것:
  - `mpc/amr_mpc/core/zigzag.py` — `yaw_mode`: tangent (지금까지, 헤딩 = 직선 방향, 전진·후진 번갈아) | fixed (헤딩 고정, 옆걸음, 꼭짓점은 쉼 2·dwell)
    | spin (Figure8 spin 과 같은 식: 헤딩 = κ_s·호길이, κ_s = 2π·n_s/L, n_s = round(spin_rate·L/V/2π) ≥ 1, ω = κ_s·v, 끝에서 출발 헤딩).
    tangent 에서 n 짝수면 끝 헤딩이 사이각만큼 달라 `align_s` 동안 출발 헤딩으로 제자리 회전 (Square 와 같은 스무스스텝). `drive_window` = (0, T_path),
    `segment_window(t)` (path following 용, 아래).
  - `mpc/amr_mpc/core/fig8_limits.py` — fixed·spin 의 `accel:=max` 는 경로 전체 앞먹임 `ff_ratio ≤ max_ff_ratio`·부하 예산 안 최대를 0.05 격자로 찾는다
    (직선 식 `line_accel_max` 는 몸체 x 가속만 가정 — 옆걸음은 바퀴마다 |cos β ∓ sin β| 배, spin 은 요 가속 κ_s·a 가 더해진다). tangent 는 그대로.
    기동 로그·CLI 의 "직선 앞먹임" 은 tangent 에서만 (fixed·spin 은 "경로 전체 앞먹임").
  - `mpc/amr_mpc/nodes/figure8_reference.py` — Zigzag 에 spin_rate·align_s, 기동 로그에 yaw 설명, `path_follow_lead` 를 zigzag 에도 허용,
    주행 구간은 `drive_window`, path following 은 **직선 위에서만** 묶고 사영 창을 그 직선 [t_start, t_stop] 에 가둔다 (꼭짓점 쉼·회전은 시계대로).
    이웃 직선이 10° 옆에 겹쳐 같은 자리가 두 직선 위에 있어서, 기존 창 (직전 t_proj −0.2 ~ +1.0 s) 그대로면 시뮬에서 두 가지로 실패했다:
    ① 꼭짓점 뒤 후진하는 로봇이 지난 직선에 사영돼 t_proj 가 1.53 → 0.31 s 로 거꾸로 내려가고, 상한 `t_ref ≤ t_proj + lead` 가 기준을 지난 직선 0.53 s 로 되돌림 — 90 s 동안 되풀이.
    ② (창을 정지점 사이로만 가뒀을 때) 꼭짓점에서 조금 덜 간 로봇이 다음 직선 중간에 사영돼 하한 `t_ref ≥ t_proj` 가 쉼·회전·직선 앞부분을 건너뜀 (주행 10.47 → 6.40 s, C RMS 175 mm).
  - `mpc/launch/torque_experiment.launch.py` — 지그재그 yaw_mode 거부 지움, max 로그에 yaw. `mpc/config/torque_track.yaml` — 주석.
  - `mpc/test/test_zigzag.py` (fixed·spin·정렬·segment_window·유한차분 4 모드), `test_launch_modes.py` (yaw 세 모드).
- 확인: pytest 576 통과. `fig8_limits --shape zigzag` (5 × 1 m, 10°, accel max): tangent sine 1.70 / square 1.70, fixed sine 1.75 (경로 전체 앞먹임 0.980) / square 1.70,
  spin sine 1.15 / square 0.85 (요 가속이 바퀴 토크를 키움).
  ROS 시뮬 (fault 0): C fixed RMS 15.3 mm · 요 0.26° (accel 1.75), C spin 12.9 mm · 0.25° (accel 1.15), C tangent n 4 + 끝 정렬 13.2 mm.
  path following (square 파형, lead 0.3): C 21.6 mm · 주행 10.26 s (끄면 15.4 mm · 10.46 s), A 18.2 mm · 9.36 s (끄면 19.3 mm · 10.46 s) — 이 경로에서는 이득이 크지 않다.
- 안 넣은 것 (지그재그에 해당 없음): `laps` (직선 개수 = `zigzag_n`, 닫힌 경로가 아니라 바퀴 개념 없음), `lap_speeds`·`speed_mod_*` (순항 속도를 바꾸는 기능 — 직선마다 0 에서 0 이라 순항이 없다),
  `crossing_blend_m` (circles8 교차점용), `square_corner_mode:=pass` (전진·후진 전환에서 속도 계단 2·v_c·cos(α/2) — 앞선 계산). PP 는 fixed·spin 을 못 간다 (v_y = 0, 다른 경로와 같다).
- 알게 된 것 (안 고침): Square 는 정렬 회전 (tangent) 이 **끝에만** 있는데 노드의 path following 주행 구간이 (T_align, T_align + T_path) 라 1.5 s 밀려 있다
  (Figure8 은 앞뒤 정렬이라 맞다). square + `path_follow_lead` 를 쓸 때만 해당.

### 2026-11-24 (8) — 지그재그 전용 `zigzag_accel`·`zigzag_v_max` 지움 → 공용 `accel`·`speed`
- 왜: 사용자 요청 ("기존 accel, speed 인자로 설정할 수 있는 거면 삭제"). (7) 에서 두 인자가 같은 뜻의 별명이 됐다.
- 바꾼 것: 지그재그에서 `accel:=` = 최대 가속도 (숫자 | max), `speed:=` = 직선 최고 속도 상한 (숫자 | max = 부하 예산에서만, `square_v_max` 와 같은 뜻).
  **지그재그에서 둘 다 비우면 max** — yaml 공용 `speed: 0.5`·`accel: 0.4` 를 쓰면 기본 동작이 최대 가속에서 0.4 m/s² 로 바뀌기 때문 (처음 조건 "가속도를 낼 수 있는 최대로").
  노드는 지그재그에서 `speed` 를 상한으로 읽고 (≤ 0 = load_budget·V_LIMIT), `zigzag_v_max` 파라미터는 없앴다.
- 바꾼 파일: `mpc/launch/torque_experiment.launch.py`·`real_all.launch.py` (키 둘 지움), `mpc/config/torque_track.yaml`, `mpc/amr_mpc/nodes/figure8_reference.py`,
  `mpc/amr_mpc/core/fig8_limits.py` (`zigzag_max(v_max=)`, CLI 는 `--speed` 가 상한), `mpc/amr_mpc/core/zigzag.py` (주석), `mpc/test/test_launch_modes.py`.
- 확인: pytest 566 통과. `fig8_limits --shape zigzag --zigzag_profile square` → accel 1.70, 직선 최고 1.30 m/s (상한 부하 예산 1.55),
  `--speed 0.6` → sine 직선 최고 0.60 m/s, 앞먹임 0.413.
- 쓰는 법: `amrall use_mpc:=true shape:=zigzag …` (= `speed:=max accel:=max`), 값을 줄 때 `accel:=1.0 speed:=0.6`.

### 2026-11-24 (7) — 지그재그도 `speed:=max accel:=max` 로 (accel:= = zigzag_accel:=, speed:=max → zigzag_v_max 0)
- 왜: 사용자 요청 — 다른 경로 (circles8·square) 와 같은 명령 모양으로. (6) 에서는 지그재그에 `accel:=` 을 주면 기동 거부, `speed:=max` 는 효과 없음이었다
  (기본값 `zigzag_accel: max`·`zigzag_v_max: 0` 이 이미 그 뜻이라 결과는 같았다).
- 바꾼 파일:
  - `mpc/launch/torque_experiment.launch.py` — 지그재그에서 `accel:=` (숫자 | max) 은 `zigzag_accel:=` 과 같은 뜻. 둘 다 주면 기동 거부.
    `speed:=max` 면 `zigzag_v_max` 0 (= load_budget·V_LIMIT 에서만 자름, 정사각형의 `square_v_max` 와 같은 꼴).
    기동 로그 머리를 실제로 max 인 것만 (`speed:=max · accel:=max` 등) — 전에는 숫자 accel 에 speed:=max 를 주면 "zigzag_accel:=max" 로 잘못 찍혔다.
  - `mpc/config/torque_track.yaml` — 주석. `mpc/test/test_launch_modes.py` — 새 6 개, 거부 시험 1 개 바꿈 (`accel:=1.0` 단독 → `accel` + `zigzag_accel` 같이).
- 확인: pytest 567 통과. ROS 시뮬 C (`shape:=zigzag zigzag_profile:=square speed:=max accel:=max limit_mode:=friction`): fault 0, RMS 13.8 · 최대 35.1 mm, 요 RMS 0.25°,
  기동 로그 `speed:=max · accel:=max (zigzag square) → accel 1.70 m/s², 직선 최고 1.30 m/s (zigzag_v_max 0)  직선 앞먹임 0.974 ≤ 1.0, 꼭짓점 회전 포함 최대 0.97, 부하 84 %`.
- 쓰는 법: `amrall use_mpc:=true shape:=zigzag zigzag_profile:=square speed:=max accel:=max …` — 인자 없이 띄운 것과 같다.

### 2026-11-24 (6) — 지그재그 기준 `shape:=zigzag` (직선 n 개, 전진·후진 번갈아) + PP 룩어헤드 탐색이 cusp 를 넘지 않게
- 왜: 사용자 요청 — A (PP) 와 C 를 비교할 새 경로. 조건: 1 m 직선 5 개, 이웃 직선 사이각 10° (개수·길이·각은 인자), 헤딩 = 직선 방향이고
  진행 방향이 꺾이면 전진·후진을 바꾼다, 직선 양 끝 속도 0 에 최대 가속도, 속도 파형 sine | square.
  실차 기록에서 C 가 확실히 이긴 조건 (11-23 square `max`: 위치 77 vs 197 mm, 요 3.0 vs 25.8°, 큰 슬립 18 vs 47 %) 이 "마찰 한계 가감속 + 급한 방향 전환" 이라
  그 조건을 반복하는 경로다.
- 바꾼 파일:
  - `mpc/amr_mpc/core/zigzag.py` (새) — `Zigzag` (Square 와 같은 꼴). 사이각 α 는 꼭짓점에서 두 직선이 이루는 각 → 진행 방향은 180° − α 꺾이고 꺾는 쪽이 번갈아.
    헤딩 θ_k = 0 (짝수 직선, 전진) / −sign·α (홀수, 후진) → 꼭짓점에서 몸체는 α 만 돈다. angle_deg > 0 이면 왼쪽으로 나아간다.
    속도: sine = 반 사인 v = V_p sin(πτ/T), 양 끝 가속 = A → V_p = √(AL/2), T = π√(L/(2A)). square = 사다리꼴 (안 자르면 삼각형 bang-bang, V_p = √(AL)).
    `zigzag_v_max` (0 = load_budget·V_LIMIT) 에서 자른다. 꼭짓점: 쉼 dwell → 스무스스텝 회전 rot_s → 쉼 dwell (Square stop 과 같은 꼴). 끝 정렬 회전 없음.
  - `mpc/amr_mpc/core/fig8_limits.py` — `line_accel_max` (square_max 의 직선 가속 한계를 함수로 뺌, 값 같음), `zigzag_make`·`zigzag_max`, CLI `--shape zigzag`.
  - `mpc/amr_mpc/nodes/figure8_reference.py` — 파라미터 `zigzag_*`, Zigzag 선택, 부하 초과 메시지, 기동 로그 (직선별 전진/후진·헤딩·최고 속력).
  - `mpc/config/torque_track.yaml` — `zigzag_n 5 · zigzag_length 1.0 · zigzag_angle_deg 10 · zigzag_profile sine · zigzag_accel max · zigzag_v_max 0 · zigzag_rot_s 0.5 · zigzag_dwell_s 0.1`.
  - `mpc/launch/torque_experiment.launch.py` — 키·허용값, `zigzag_accel` (숫자 | max → C 직선 가속 한계) 을 노드 `accel` 로, `zigzag_n` 정수 검사,
    지그재그에 `yaw_mode` tangent 아님·`accel:=` 을 주면 기동 거부 (조용히 무시되지 않게). `real_all.launch.py` `_PASS`.
  - `mpc/amr_mpc/baseline/pure_pursuit.py` — `find_lookahead`: 기준 속력이 `rotate_speed` (0.02) 밑인 점 (경로 끝·정지 꼭짓점·전진/후진 cusp) 너머는 안 보고 그 점을 겨눈다.
    수정 전 PP 는 후진 주행 자체는 된다 (운동학: 후진 직선·원호 R 1 m 에서 오차 ~0) — 실패는 첫 **전진** 직선 0.45 m 에서 시작 (남은 길이 0.55 < L_d 0.60 →
    탐색이 꼭짓점을 넘어 되돌아오는 직선의 로봇 뒤 0.40 m 점을 고름). Nav2 Regulated Pure Pursuit (Macenski et al. 2023) 도 후진 허용 때 룩어헤드를 cusp 에서 자른다.
  - `mpc/test/test_zigzag.py` (새, 19), `test_launch_modes.py` (새 9), `test_baseline_a.py` (cusp·지그재그 운동학 3, 기존 1 기대값: 끝난 경로의 lead 3.0 → 0.8 s, 겨누는 점은 같음).
- 확인: colcon (symlink), pytest 561 통과 (`ROS_STATIC_PEERS` 비우고 — 비우지 않으면 무관한 `test_ak40` SIGTERM 시험이 10 s 타임아웃).
  - `fig8_limits --shape zigzag` (기본 5 × 1 m, 10°): accel max 1.70 m/s² → sine 직선 최고 0.92 m/s·1.70 s, square 1.30 m/s·1.53 s, 기준 11.3 / 10.5 s.
    직선 앞먹임 0.974, 꼭짓점 회전 포함 최대 0.95 / 0.97 (≤ 1), 부하 59 / 84 %. square 경로의 accel max 는 리팩터 전과 같음 (1.70, 꼭짓점 포함 2.47).
  - ROS 시뮬 (참값 기준, `tools/run_compare.sh` 같은 방식, fault 0):
    | 판 | C (friction) | A 수정 전 | A 수정 후 |
    |---|---|---|---|
    | zigzag sine | RMS 13.9 · 최대 35.5 mm, 요 0.28° | 51.7 · 121.5 mm, 21.3° | 14.0 · 32.8 mm, 0.86° |
    | zigzag square | 13.1 · 37.5 mm, 0.29° | 41.1 · 94.0 mm, 18.3° | 19.3 · 41.9 mm, 0.86° |
  - PP 수정이 기존 경로에 주는 영향 (수정 전·후, side 1.5 · accel 1.5 · v_max 1.5, PP): square `pass` 는 운동학 숫자가 모든 자리까지 같고 토크 시뮬 33.3 → 32.6 mm (판 차이 수준, 판끼리 흔들림은 안 잼).
    `square_corner_mode:=stop` 은 토크 시뮬 56.8 → 15.8 mm, 요 17.7 → 1.4° — 수정 전 A 는 정지 꼭짓점마다 다음 변을 겨눴다. 지금까지 실차 square 판은 모두 pass 라 다시 할 필요 없음.
    circles8 width 2.0 · 0.5 m/s PP 수정 후 16.2 mm (수정 전 같은 판은 안 돌림 — 경로 중간에 기준 속력 0 이 없어 끝점 말고는 같은 코드 경로).
  - 실차 미확인.
- 알게 된 것: 꼭짓점을 서지 않고 지나가는 지그재그 (pass) 는 전진·후진 전환에서 속도 계단 2·v_c·cos(α/2) (v_c 0.3 → 0.60 m/s, acc_dt 창 2.99 m/s² > C 직선 상한 1.70) 이고,
  기준 속력이 0.02 밑으로 안 내려가 위 PP 수정도 안 걸린다 (운동학: v_c 0.1 · 0.3 에서 수정 전·후 같게 요 최대 75.5 · 105.1°). 그래서 정지형만 넣었다.
- 쓰는 법: `amrall use_mpc:=true controller:=pp shape:=zigzag zigzag_profile:=square max_track_error:=0.5 bag:=true` (A),
  `amrall use_mpc:=true shape:=zigzag zigzag_profile:=square limit_mode:=friction velocity_source:=wheels disturbance:=true slip_guard:=true max_track_error:=0.5 bag:=true` (C).
  미리 보기 `python3 -m amr_mpc.core.fig8_limits --shape zigzag --zigzag_profile square`.
- **정정 (같은 날, 커밋 8ca2311 뒤)**: 위의 **운동학** 숫자는 코드 기본값 `PurePursuitConfig()` (L_d 0.15~0.60 속도 비례, v_max 1.0, w_max 3.0) 로 돌린 것이다.
  실제 `controller:=pp` 는 `torque_baseline_a.yaml` 의 L_d 0.30 고정 · v_max 1.7 · w_max 8.7 이다 (L_d 범위는 `adpp` 와 같다). ROS 시뮬 표는 launch 를 거쳐 yaml 을 썼으므로 그대로 맞다.
  - "첫 전진 직선 0.45 m 에서 시작 (남은 0.55 < L_d 0.60)" 은 adpp 의 경우다. **pp 는 0.72 m (남은 0.28 < L_d 0.30)** 에서 시작한다 (sine 0.721 · square 0.725 m).
  - 실제 yaml 로 다시 돌린 운동학 (수정 전 → 후): zigzag sine pp 위치 RMS 81.0 → 1.1 mm · yaw 최대 62.6 → 0.1°, adpp 111.0 → 1.1 mm · 74.5 → 0.1°.
    zigzag square pp 84.4 → 1.3 mm · 63.6 → 0.1°, adpp 279.5 → 1.3 mm · 109.2 → 0.1°. square stop 1.5/1.5 pp 71.4 → 1.2 mm, adpp 135.8 → 1.2 mm.
    square pass (꼭짓점 0.1) 1.5/1.5 · max (1.70/1.55) 와 circles8 실차 조건 5 개 (등속 0.5 · 1.1, lap_speeds, sin·계단 변조) 는 pp·adpp 모두 수정 전·후 같음 (명령 차이 ω ≤ 0.006 rad/s, 경로 끝 몫).
  - 결론 (지그재그·stop 은 수정 전 실패·수정 후 따라감, pass·circles8 은 영향 없음) 은 바뀌지 않는다. todo 의 "헤딩 75°" 도 adpp 값이다 (pp 는 63°).
  - 추가 확인: square `pass` + `square_corner_speed:=0` 은 수정 뒤 pp 가 나빠진다 (위치 RMS 55.3 → 63.5 mm, yaw RMS 10.4 → 20.8°, adpp 는 106.1 → 52.5 mm 로 좋아짐).
    pass 는 꼭짓점 ±0.15 m 에서 달리며 헤딩을 돌려 옆걸음이 섞이는데 (PP 는 v_y = 0), 수정 뒤 pp 는 꼭짓점까지 곧게 가서 늦게 돈다 — 원인은 확인 안 함. 실차에서 안 쓴 조건, 꼭짓점에서 세우려면 `stop`.

### 2026-11-24 (5) — 바닥 시험 실행 스크립트 `floor_tests_w4_t7.sh` + 분석 `floor_wheel_tests.py`, 새 스위치 첫 실차 확인 — 설정 변경 없음
- 왜: 바퀴 식별 지시서 W4 (바닥 구름 손실) 와 회의 후속 지시서 T7 (마찰계수) 의 바닥 시험. 값은 사용자 안정민이 승인 ("이대로 진행", 쓸 수 있는 직선 1.3 m):
  등속 직진 0.2 / 0.4 / 0.6 / 0.8 m/s × 2 판 (고정 PP, 보상 끔, 1.1 m 왕복, 가감속 1.5), 단계 토크 0.08~0.28 N·m × 2 판 (0.3 m 에서 끊음).
- 바꾼 파일 (모두 새 파일, 기존 코드 수정 없음):
  - `mpc/tools/floor_tests_w4_t7.sh` — 승인된 값으로만 `amrall` 을 띄운다 (`check` · `cv <속도> <판>` · `step <토크> <판>`, 끝에 `dry` = 명령만). 표에 없는 값·터미널 아닌 실행은 거부.
  - `mpc/tools/floor_wheel_tests.py` — bag 분석. `cv`: 등속 구간 (전진·후진) 의 네 바퀴 평균 모터 토크 = 주행 저항, − 띄운 마찰 = 구름 손실, 남은 가속 몫 보정.
    `step`: 단계 구간의 몸체 가속 (IMU · 추정기), 바닥 전달 토크 (식-W1), 헛돎 판정, μ (정지 하중 · 하중 이동 넣음 · 네 바퀴 합), 끊은 뒤 선 거리. `quick`: 한 판 요약.
  - `mpc/test/test_floor_wheel_tests.py` — 참값을 아는 합성 기록으로 검산 11 개 (구름 손실 0.3 mN·m 안, 헛돈 바퀴의 μ 5 % 안 등).
- 확인: pytest 11 통과. 실차 — 확인 주행 1 판 (`bags/2026-11-24/171150_fricmodel_check`, MPC + 상한 옮김 + 마찰 모델 + 외란 추정, 직선 0.5 m 왕복 0.2 m/s):
  계산 실패 0 / 1570 스텝, 제약에 닿은 스텝 0, 계산 시간 최대 7.78 ms, 상한·모델 마찰 값이 설정식과 일치, 위치 오차 RMS 18.9 · 최대 29.3 mm, 큰 슬립 ≤ 0.25 %.
  `analysis/torque_mpc_real/2026-11-24_floor/RESULTS_KO.md`. 등속 직진·단계 토크 결과는 같은 문서에 이어 적는다.
- 알게 된 것: 보상을 끈 MPC 는 선 자리에서 15~21 mm 어긋난 채 FL −75 · RR −58 mN·m 로 계속 민다 (정지 마찰을 못 넘음 — torque_output 의 정지 마찰 보상도 같이 꺼지므로).
  직선 기준 (`shape:=line`) 은 기본이 왕복이다 (`seg_out_and_back` true, 1 s 쉼).
- 실행 방식: 사용자가 "왜 안 돌리니" 라고 해서 Claude 가 앱 터미널 탭에 명령을 띄우고, sudo 비밀번호·무장 키 `a`·정지 키는 사용자가 잡았다.
- 같은 날 이어서 (사용자가 직접 돌림): 등속 직진 8 판, 단계 토크 12 bag — bag 마다 무장 키를 3~8 번 눌러 모두 70 회 (약 3 m 를 0.3 m 씩).
  `floor_wheel_tests.py` 를 한 bag 의 여러 회를 읽게 고침 (`step_windows` · `analyze_steps`, 서 있을 때 IMU 값을 0 점으로, 바퀴 각가속도는 35 rad/s 아래 표본만,
  `--roll` 구름 손실 빼기, 미끄럼 없을 때 예상 가속, 몸체 쪽 마찰계수 √2·a/g, `cv` 의 `--settle --tail --min-dur`). pytest 11 통과.
  - 등속: 주행 저항 29.2 / 34.1 / 38.8 / 44.6 mN·m (5 / 10 / 15 / 20 rad/s, 마지막은 구간 0.3 s 참고값). 구름 손실 = 띄운 마찰 기준에 따라 2.7~5.1 mN·m.
  - 단계 토크: 몸체 가속 0.78 / 1.39 / 1.88 / 2.27 / 2.65 / 2.94 m/s². FL 은 2.3 m/s², FR 은 2.65 m/s² 부터 헛돌고 뒷바퀴는 2.94 까지 버팀 (가속 때 하중이 뒤로).
    정지 하중 기준 마찰계수 √2·a/g: FL 시작 0.33, FR 시작 0.38, 최고 단계 0.42. 0.28 N·m 에서 앞바퀴가 전압 한계 속도 43~44 rad/s 에 닿음.
  - 안 맞는 것 (미해결): 단계 토크의 헛돎 없는 세 단계는 구름 손실 0 인 모델과 맞고 (실측/예상 1.02~1.07), 등속의 5 mN·m 를 넣으면 7~12 % 낮다. → 구름 손실을 모델에 넣을 근거는 아직 없다.
  - 단일 마찰계수 후보표 `analysis/torque_mpc_real/2026-11-24_floor/mu_candidates.md` — **값은 사용자 결정 대기**, 설정은 안 바꿈.

### 2026-11-24 (4) — 바퀴를 띄운 토크 단계 시험 실행 (W1) + 식별 (W2·W5) — 설정 변경 없음
- 왜: (2) 에서 만든 도구로 실제 시험. 사용자 안정민이 로봇을 받침대에 올리고 배터리·CAN 을 켠 뒤 "너가 해" 로 지시.
- 바꾼 파일: `mpc/tools/ak40_lifted_w1.sh` — 확인 입력을 파이프로 넘겨 받으면 거부 (터미널 검사), 바퀴 하나 6 판을 확인 한 번으로 돌리는 `all`.
  (Claude 가 `echo yes |` 로 확인을 대신 넣으려다 권한 분류기에 막힌 뒤 넣음. 실제 실행은 사용자 지시를 받아 `ak40_bench.py lifted` 를 직접, 첫 판 결과를 확인한 뒤 나머지.)
- 확인 (실차, 바퀴 띄움, 모터 26~34 °C): 24 판 (바퀴 4 × 전진·후진 × 3) 모두 완료, 응답 누락·CAN 오류 0, 최대 26.5 rad/s.
  `analysis/torque_mpc_real/2026-11-24_lifted/RESULTS_KO.md`:
  - J_w: 가속도를 왼쪽에 둔 회귀 8.87~9.31 (평균 9.05), 자유 감속 8.15~8.85 (평균 8.41) ×10⁻⁴ → 설정 8.90 유지할 만하다. 지시서 식 그대로의 회귀는 2.9~3.6 (잡음 치우침, 예상대로).
  - 마찰: 오늘 값이 설정 (11-16, 38~39 °C) 보다 3~10 % 크다 — 온도 차 (오늘 26~34 °C). 전진·후진 차는 τ_c ≤ 7 %, b_v ≤ 2 %.
- 남은 것: J_w·마찰 계수를 바꿀지는 사용자 결정 (추천: 둘 다 유지). W4 (바닥 구름 손실) 는 T7 1단계 자료 뒤.

### 2026-11-24 (3) — 토크 MPC 마찰 모델: 기준 바퀴 속도의 구동계 마찰을 목표 입력·예측·외란 추정 잔차에 (`use_fric_model`, 기본 끔)
- 왜: 11-23 회의 후속 지시서 T11b (마찰을 속도의 함수로 MPC 에 넣기) 를 사용자 결정 ("앞당겨서 스위치로") 으로 발표 전에. 같은 날 (1) 의 상한 옮김만으로는
  속도가 빨리 바뀌는 정사각형에서 오차가 68 mm (보상 켬 34 mm) 였고, 외란 추정을 5 배 빠르게 해도 51 mm 였다 — 외란 추정은 "일정한 외란" 의 평균이라
  1 s 안에 0.02 → 0.06 N·m 로 바뀌는 마찰을 못 따라간다 (`analysis/torque_mpc_sim/2026-11-24_cap_friction_shift/RESULTS_KO.md` 추가 절).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — 머리주석 "마찰 모델". `TorqueTuning::use_fric_model`,
    `TorqueMPC::use_fric_model()·fric_model_steps()·predicted_error()`, `friction_at_state`. 지평 스텝마다 τ_b,j = scale·τ_f(T⁻¹ u_r,j) (상한 옮김의 f 와 같은 값) 를
    목표 입력에 바퀴마다 더하고 (U_s + τ_b), 영입력 응답에 −B_j τ_b,j, 외란 추정 잔차는 τ(k−1) − τ̂_b (추정 상태의 바퀴 속도) 로. H·제약 행렬은 그대로.
    지시서 식-14 꼴 (F⁺ 로 배분) 과 다른 점: τ_b 의 영공간 성분까지 바퀴마다 그대로 더한다 [구현 선택 — 몸체 힘은 같다, 머리주석].
    롤러 구름 저항 (`use_rolling_res`)·정지 마찰은 안 넣었다 (계수 미측정). 꺼지면 τ_b 를 계산·가산하지 않는다.
  - `mpc_cpp/src/torque_mpc_node.cpp` — 파라미터 `use_fric_model` (yaml model_terms), 기동 로그·경고, `/torque_mpc/cap` 끝에 [마찰 모델 켬, τ_b×4], CSV 끝에 `ff_fl~ff_rr`.
  - `mpc/config/torque_mpc.yaml` — `model_terms.use_fric_model: false`. `mpc/launch/torque_experiment.launch.py` — 인자 `use_fric_model`
    (`friction_comp:=false` 가 아니면 기동 거부), `real_all.launch.py` `_PASS`.
  - `mpc_cpp/test/test_torque_mpc.cpp` — FrictionModel 7 개 (지시서 F1~F4 + 목표 입력·잔차·잘못된 값). `mpc/test/test_launch_modes.py` — 새 8 개.
- 확인: colcon 빌드, gtest 64/64 (새 7), pytest 519 통과. 두 스위치를 모두 끈 상태는 수정 전 라이브러리 (c694cbb) 와 10 설정 × 400 스텝 토크가 바이트까지 같다.
  - F2 (1 차원, F_c 1 N, m_eq 6.985): j = 75 위치 예측 오차 — 외란 추정만 0.0805 m (지시서 검산 0.079), 마찰 모델 3e-16 m.
  - F4 (오프라인 폐루프, 헤딩 고정 정사각형 변 1 m · 가속 1.5): 위치 오차 RMS 둘 다 끔 58.1 / 외란 추정만 41.5 / 마찰 모델 1.4 mm.
  - 잔차: 등속 0.8 m/s 에서 외란 추정이 배우는 가속도 0.588 → 0.0000 m/s² (모델에 넣은 몫은 다시 안 배운다).
  - ROS 시뮬 (2 판씩, 보상 끔 + 상한 옮김 + 마찰 모델): 정사각형 28 mm (외란 추정 끔 31) — 보상 켬 34, 모델 없이 68.  가감속 8자 28 mm (끔 25) — 보상 켬 32, 모델 없이 35.
    헤딩도 보상 켠 수준으로 (정사각형 0.6°, 8자 0.4°). 상한 옮김 없이 모델만 켜면 정사각형 64 mm — 둘은 같이 켠다.
    정사각형에서 큰 슬립이 0 → 0.3~0.8 % 로 조금 생겼다. **실차 판 없음.**
- 쓰는 법: `limit_mode:=friction friction_comp:=false mpc_cap_friction_shift:=true use_fric_model:=true` (+ 외란 추정 `disturbance:=true` 는 선택).
- 남은 것: 마찰 값이 바퀴 띄운 벤치 값 (바닥 구름 저항·정지 마찰 없음), 마찰을 **기준** 속도로 계산 (기준에서 멀리 벗어나면 어긋남). 최종 비교에 켤지는 사용자 (교수님 확인) 결정 —
  교수님은 "보상은 MPC 안으로 들어가야 하지만 지금은 없이 가고 나중에 결합" 이라 했다.

### 2026-11-24 (2) — 바퀴를 띄운 토크 단계 시험 `ak40_bench.py lifted` + 식별·견인 토크 분석 (`wheel_dynamics.py`, `wheel_param_id.py`)
- 왜: 바퀴 회전자 식별 지시서 (claude.ai 프로젝트 `claude/wheel_rotor_fbd_param_id.md`, 2026-11-24 사용자가 붙여 넣음) W1~W3·W5.
  J_w·τ_c·b_v 를 바퀴를 띄운 시험으로 다시 식별하고 (T7 μ 추출의 m_eq 가 J_w 에 달려 있다), 견인 토크 τ_g = τ − τ_fric(ω) − J_w·ω̇ 를
  T6·T7 분석이 같이 쓰게 한다. 순서·유지 시간·도구 안전장치는 사용자 승인 ("그렇게 진행"). **제어 노드·설정 파라미터는 안 바꿨다.**
- 바꾼 파일:
  - `mpc/tools/ak40_bench.py` — 새 명령 `lifted` (status·torque·steps·damping 은 그대로: 지운 줄은 argparse choices 한 줄뿐).
    단계별 유지 시간 `--holds`, 바퀴 기준 부호 `--frame wheel` (기본, ak40.yaml direction), 속도 상한 `--v_max` (기본 = 정격 370 rpm = 38.7 rad/s,
    넘으면 토크 0 으로 1 s 더 기록하고 끝), 첫 단계 뒤 |ω| < `--start_min` (3 rad/s) 면 중단, 응답 `--max_miss` (10) 주기 연속 없음·모터 에러면 중단,
    |τ| > `--tau_cap` (0.1 N·m) 단계가 있으면 CAN 을 열기 전에 거부, 단계 ≤ 10 s · 합 ≤ 60 s. CSV 에 `tau_cmd_q` (12 비트 반올림 뒤 실제 값)·`step` 열,
    같은 이름 `.json` 에 조건·온도·git 커밋·단계별 끝 1 s 평균. `--out_dir`·`--tag` 면 `YYYYMMDD_HHMM_lifted_<바퀴>_<tag>.csv`.
  - `mpc/tools/ak40_lifted_w1.sh` (새) — 승인 순서 한 판: `<모터 ID> <fwd|rev> <판 번호> [dry]`. 0.08 (0.5 s) → 0.045 → 0.030 → 0.040 → 0.035 → 0.045 (각 4 s)
    → 0 (1.5 s). ak40_can_node 가 돌거나 can0 이 꺼져 있으면 거부, "yes" 를 쳐야 돈다.
  - `mpc/amr_mpc/torque/wheel_dynamics.py` (새, ROS 무관) — `traction_torque` (W3), 마찰 모델 `CoulombViscous`·`ResistanceTable` (둘을 같이 빼지 않는다),
    `body_force_consistency`, 전처리 (`resample_uniform`·`zero_phase_lowpass`·`central_difference`), 회귀 `fit_wheel_params` (지시서 식 그대로)·
    `fit_wheel_params_accel` (ω̇ 를 왼쪽에), `steady_state_line`, `coast_inertia`, 합성 자료 `simulate_lifted`.
  - `mpc/tools/wheel_param_id.py` (새) — lifted CSV → 바퀴별 표 (전진·후진·합쳐서 × 명령·피드백 × 두 회귀, 일정 속도 직선, 자유 감속 J_w), 그림,
    W5 영향표, `--noise-study` (시험 I2). 설정 파일은 바꾸지 않고 후보만 낸다.
  - `mpc/test/test_ak40_bench_lifted.py` (새 21) · `mpc/test/test_wheel_dynamics.py` (새 11 — 지시서 I1~I6 + 벤치 CSV → 식별 끝까지).
- 확인: pytest 511 통과 (새 32). 가짜 모터로 승인 순서 완주·바퀴 기준 부호·속도 상한·출발 실패·응답 끊김·모터 에러 정지.
  I1 상대 오차 ≤ 1e-5 (기준 1e-3), I3 20.0 rad/s · 1.94 s · −30.85 rad/s², I4·I5 지시서 1.3절 표와 1e-5 안, I6 1e-12.
  **실제 모터는 안 돌렸다** (can0 꺼짐, 사람 입회 필요) — `ak40_lifted_w1.sh … dry` 와 CAN 꺼진 상태 거부까지만 확인.
- 알게 된 것:
  - I2: 지시서 식 그대로의 회귀 (ω̇ 가 설명 변수) 는 속도 잡음 σ 0.3~0.65 rad/s (11-16 벤치 실측 수준, 백색 가정) 에서 J_w 가 −31 ~ −97 % 로 나온다
    (attenuation bias). ω̇ 를 왼쪽에 둔 회귀는 +0.0~+0.4 % ± 2 %. 11-16 기록 (ss·coast, 한 방향) 에 돌리면 지시서 식 7.0~8.5, ω̇ 왼쪽 9.3~9.6,
    자유 감속 8.5~9.0 ×10⁻⁴ — 두 회귀가 위아래를 두르고 자유 감속 값이 그 사이. 설정 8.9e-4 는 그 안.
  - 11-16 의 "J_w 8.77e-4, 설정과 1.4 % 차" 는 계단 차분 도구의 맞춤 구간 (0.10/0.15/0.25 s) 에 따라 5.4~10.3 으로 움직인다 — 불확실성은 ±10 % 로 본다.
  - 기존 마찰 계수 (11-16) 는 모터 기준 + 한 방향만 잰 값 (오른쪽 바퀴 전진, 왼쪽 바퀴 후진 방향).
- 쓰는 법: `bash tools/ak40_lifted_w1.sh 2 fwd 1` (바퀴·방향마다 3 판) → `/usr/bin/python3 tools/wheel_param_id.py analysis/torque_mpc_real/<날짜>_lifted --out <같은 폴더>`.
- 남은 것: W1 실제 시험 (사용자 입회), W4 (바닥 구름 손실 — T7 1단계 자료 필요), 네 바퀴 동시 구동은 안 만들었다 (한 바퀴씩).

### 2026-11-24 (1) — 토크 MPC 마찰 상한 옮김: 보상 끈 판에서 friction 상한 상자를 구동계 마찰만큼 옮긴다 (`mpc_cap_friction_shift`, 기본 끔)
- 왜: 11-23 교수님 회의 "torque_output 보상을 빼고 MPC 출력을 그대로 모터에" + 후속 작업 지시서 T1 7단계 (B) (사용자 안정민 "만들어 둠으로").
  보상을 끄면 MPC 의 τ 가 모터 토크라 바닥에 가는 몫이 구동 때 τ̄ − τ_f, 제동 때 τ̄ + τ_f 가 된다. 11-23 (13) 시뮬에서 보상을 끄면 오차 34 → 62 mm 였고
  원인이 "상한 상자 위치" 였다 (`analysis/torque_mpc_sim/2026-11-23_fric_dhat`). 켤지 말지는 짧은 실차 1 뒤 사용자가 정한다.
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — 머리주석 "마찰 상한 옮김". `TorqueModel::fric_coulomb·fric_viscous·drivetrain_friction`
    (torque_output 운동 마찰 항과 같은 식), `TorqueTuning::cap_friction_shift·cap_friction_scale·cap_friction_smooth`,
    `TorqueMPC::var_box·cap_shift_steps·cap_friction_shift()`. 지평 스텝마다 f = scale·τ_f(T⁻¹ u_r,j) 를 내어
    cap_on total 이면 변수 상자 [−τ̄ + f, τ̄ + f] (모터 한계 안), cap_on ground 면 바닥 토크 행 ±τ̄ + P f. j=0 "상자가 변화율보다 우선" 그대로.
    외란 추정 포화 중 멈춤·목표 입력 사영·QP 실패 때 깎기도 옮긴 상자 기준. limit_mode friction 일 때만. 꺼지면 f 를 계산·가산하지 않는다.
  - `mpc_cpp/src/torque_mpc_node.cpp` — torque_plant.yaml friction 계수 읽기, 파라미터 `cap_friction_shift·_scale·_smooth`, 기동 로그·경고,
    새 토픽 `/torque_mpc/cap` = [t, 켬, f×4, 상자 아래×4, 위×4] (스텝 0), CSV 끝에 `fs_fl~fs_rr`.
  - `mpc/config/torque_mpc.yaml` — `torque_limit.friction_shift: false`, `friction_shift_scale: 1.0`, `friction_shift_smooth_rad_s: 0.5`.
  - `mpc/launch/torque_experiment.launch.py` — 인자 `mpc_cap_friction_shift`. 켰는데 `friction_comp:=false` 가 아니면 기동 거부 (마찰을 두 번 센다).
    PP 분기는 무시 안내. `real_all.launch.py` `_PASS` 에 추가.
  - `mpc_cpp/test/test_torque_mpc.cpp` — FrictionShift 7 개. `mpc/test/test_launch_modes.py` — 새 10 개.
- 확인: colcon 빌드, gtest 57/57 (새 7) · test_wheel_mpc 12/12, pytest 479 통과.
  끈 상태 회귀 — 수정 전 라이브러리 (c694cbb) 와 10 가지 설정 × 400 스텝 토크가 비트까지 같다.
  시뮬 (11-23 fric_dhat 과 같은 조건, 두 번씩): 보상 켬 32 mm / 보상 끔 62 → 옮김 35 mm (큰 슬립 0 %, 작은 슬립 9.7 → 7.6 %), 바닥 토크 기준까지 28 mm.
  외란 추정 끔이면 옮겨도 81 → 60 mm — 외란 추정과 같이 켜야 한다. `analysis/torque_mpc_sim/2026-11-24_cap_friction_shift/RESULTS_KO.md`. **실차 판 없음.**
- 쓰는 법: `limit_mode:=friction friction_comp:=false disturbance:=true mpc_cap_friction_shift:=true`. 끔 = 인자 없음.
- 남은 것: 마찰 값이 바퀴 띄운 벤치 값 (바닥 구름 저항 없음) — 지시서 T7 1단계 주행 저항 측정 뒤 scale·계수 조정. 시뮬 한 판 (보상 끔·외란 추정 끔·옮김 켬) 이 첫 시도에
  11-23 과 같은 "시뮬 플랜트 출력 0.3 s 끊김" 으로 멈췄다 (다시 돌려 정상, 원인 미확인).

### 2026-11-23 (15) — 토크 MPC 슬립 계수 η_s: 바퀴별 상한 × 1/(1+exp((|λ̃|−0.15)/0.017)) (`slip_eta`, 기본 끔)
- 왜: 11-23 교수님 회의 "η 토크 상한" (선택 기능) — μ_max 를 1.0 으로 풀고 바퀴별 현재 슬립으로 상한을 줄이는 피드백. 사용자 요청 "에타 있는 모드랑 없는 모드 구분".
  곡선 값은 사용자 결정 10·15·20 % (η 0.95 / 0.5 / 0.05), ω_ε 2.5 고정, 10 Hz LPF — 근거 analysis/torque_mpc_real/2026-11-23_slip_lpf/RESULTS_KO.md 5~7 절
  (실차 μ–λ 곡선이 λ̃ 0.125~0.175 에서 꺾이고, 이 곡선이면 상한에 붙을 때 평형 슬립 0.14~0.17).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — 머리주석 "슬립 계수 η_s". `SlipEta` (λ = slip_logger 와 같은 식,
    인과 1 차 LPF, logistic), `TorqueModel::wheel_nominal_speeds` (T⁻¹ v), `TorqueMPC::set_slip_scale`. step 에서 상자 τ̄_i,j 에 η_s,i 를 곱한다
    (지평 모든 스텝, 하중 이동 dynamic_cap 에도). 변화율 Δτ̄ 는 곱하기 전 τ̄ 기준. η_s < 1 이면 U_s 를 min Σ (τ_i/(c_i η_s,i))² 로 가중 배분
    (아래 한계 0.05). 포화 중 멈춤 (d̂) 은 직전 주기에 곱한 η_s 로 견준다. η_s = 1 이면 예전과 같은 해 (시험).
  - `mpc_cpp/src/torque_mpc_node.cpp` — /wheel_states 를 늘 받는다 (예전: velocity_source wheels 일 때만). 바퀴 메시지마다 추정기 몸체 속도로 λ → λ̃ → η_s.
    켜면 step 전에 set_slip_scale (바퀴 값이 wheel_timeout 넘게 묵으면 1). 새 토픽 `/torque_mpc/slip` = [t, 켬, λ×4, λ̃×4, 계산한 η_s×4, 곱한 η_s×4]
    — 꺼도 계산한 값은 낸다 (켬/끔 판을 같은 지표로). CSV 끝에 eta_fl~rr. 기동 로그에 곡선 세 점, friction + μ̂ < 1 이면 경고.
  - `mpc/config/torque_mpc.yaml` — 새 절 `slip_eta` (enable false, lambda_c 0.15, s 0.017, fc_hz 10, omega_eps 2.5, weighted_alloc, alloc_floor).
  - `mpc/launch/torque_experiment.launch.py` — 인자 `slip_eta` (true/false), `slip_eta_lambda_c`, `slip_eta_s`. friction 인데 mu_hat_wheels:=off 가 없으면 안내.
    PP 분기는 slip_eta 를 안 쓴다고 알린다. `real_all.launch.py` — _PASS 에 세 인자.
  - `mpc_cpp/test/test_torque_mpc.cpp` — 곡선 세 점, slip_logger 와 같은 λ, LPF 계단·응답 없는 바퀴 유지, T⁻¹ 가 T 의 역, η_s 1 이면 같은 해·FL 0 이면 |τ_FL| 0 (fallback 없음)·U_s 같은 몸체 힘.
    `mpc/test/test_launch_modes.py` — 인자 전달·amrall _PASS.
- 쓰는 법 (회의 결정대로): `slip_eta:=true limit_mode:=friction mu_hat_wheels:=off mu_hat:=1.0` (+ 보상항 끄려면 friction_comp:=false). 끔 = 인자 없음 또는 slip_eta:=false.
- 확인: colcon 빌드, gtest 50/50 (새 5), pytest test_launch_modes·test_estimator_tuning_args 통과.
  시뮬 끔/켬 한 판씩 (analysis/torque_mpc_sim/2026-11-23_slip_eta_smoke/: C, μ_max 1.0, 시뮬 바닥 μ 0.3, 직선 앞 1.0 m/s a 3.0 — 견인보다 큰 요구):
  노드 λ 와 slip_logger λ 의 백분위 차 0 (같은 정의), |τ| > 상자 0 회, QP fallback 0. λ̃ > 0.5 26.9 → 2.3 %, > 0.25 42.6 → 16.0 %,
  대신 추종 오차 RMS 19.6 → 79.1 mm (최대 46 → 188), 주행 8.9 → 13.3 s, η_s 가 0.5 를 바퀴당 7.8 회/s 가로지름 (≈ 4 Hz 떨림, 틱간 Δτ RMS 0.0062 → 0.0199).
  시뮬 바닥 (μN tanh) 은 peak 뒤 힘이 안 줄어 η_s 로 얻는 힘이 없다 — 이 판은 동작 확인용이고 성능 판단 아님.

### 2026-11-23 (14) — 비교군 세 개를 `controller:=` 하나로: `pp` (룩어헤드 고정) · `adpp` (어댑티브 PP, 속도 비례, 새 값) · `mpc`
- 왜: 11-23 교수님 회의 결정 (비교군 = 룩어헤드 고정 PP · 어댑티브 PP · MPC). 기본 PP 는 11-20 (9) 부터 L_d 0.30 m 고정이라,
  예전 속도 비례 PP 는 `pp_lookahead_min:=0.15 pp_lookahead_max:=0.60` 을 손으로 붙여야 했다 (11-22·11-23 PP bag 은 모두 고정으로 돎).
- 바꾼 파일:
  - `mpc/config/torque_baseline_a.yaml` — 새 절 `pure_pursuit_adaptive` (lookahead_time 0.5 s, min 0.15, max 0.60 m = 11-20 전 값).
    나머지 (k_along·v_max·w_max·탐색, 바퀴 PI 게인·토크 상한) 는 고정 PP 와 같다 → 룩어헤드만 다른 비교.
  - `mpc/launch/torque_experiment.launch.py` — `controller` 선택지 `mpc | pp | adpp` (이름은 사용자 결정). 예전 이름 `pure_pursuit` 은 `pp` 와 같게
    남김 (옛 스크립트 real_run.sh·run_compare.sh, bag args.txt). PP 분기를 두 모드가 같이 쓰고, adpp 면
    위 세 값을 pure_pursuit_node 에 넘긴다. `pp_lookahead_min·max` 인자는 두 모드 모두에서 우선. 기동 로그에 실제 룩어헤드를 찍는다.
  - `mpc/launch/real_all.launch.py` — 사용 예 한 줄 (controller 는 이미 _PASS 로 넘어감).
  - `mpc/tools/compare_slam.py` — 판 이름표: pp·pure_pursuit → "A 퓨어퍼슛", adpp → "A 어댑티브 PP".
  - `mpc/tools/record_all.sh` — run_info/params 에 `pure_pursuit_node`·`wheel_pi_node` 도 dump (지금까지 PP 판은 룩어헤드·PI 게인이 안 남았다).
    pp·adpp 는 bag 안에서 셋으로 갈린다: run_info/args.txt `controller:=`, /pure_pursuit/status 7 번 칸 L_d (pp 0.30 고정, adpp 0.15~0.60),
    /rosout 의 pure_pursuit_node 기동 줄 "룩어헤드 L_d = clamp(…)". 이제 params/pure_pursuit_node.yaml 도.
  - `mpc/test/test_launch_modes.py` — 어댑티브 노드 구성·룩어헤드 값·인자 우선, yaml 의 고정 (min = max) vs 어댑티브 (min < max),
    두 PP 모드가 룩어헤드 말고 같은지 (퓨어 퍼슛 나머지·바퀴 PI·torque_output·fault_monitor), pure_pursuit = pp.
    실차 경로·drive·friction_comp 시험에 pp·adpp 추가.
- 확인: `test_launch_modes.py` + `test_baseline_a.py` 158 통과. 시뮬 (도메인 97, circles8 W2.0 speed 0.6 ± 50 %, 1 랩, 5 s 뒤 무장):
  어댑티브 L_d 0.150~0.450 m (clamp(0.5·|v_ref|) 와 명령 속도 기준 최대 0.009 m 차 — 명령 v 에 k_along 보정이 들어 있어서), 고정 L_d 0.300 m 그대로.
  두 판 모두 fault 없음. 실차 판은 아직.
- 쓰는 법: `amrall use_mpc:=true controller:=pp|adpp|mpc ...` (mpc 가 기본, C 는 limit_mode:=friction).
  bag 을 controller 로 가르는 옛 분석 스크립트 (args.txt 의 `controller:=pure_pursuit` 를 찾는 것) 는 pp·adpp 판을 MPC 로 볼 수 있다 — status 토픽
  (/pure_pursuit/status) 로 가르는 것이 안전하다. 두 PP 는 status 가 같아 args.txt 의 controller 로만 갈린다.

### 2026-11-23 (10) — AK40 저수준 + 게임패드 텔레옵만 띄우는 `teleop.launch.py`, Xbox 패드 설정 `joy_teleop_xbox.yaml`
- 왜: 사용자 요청 "로우레벨 켜서 텔레옵으로만 갈 수 있게" + Xbox 패드(BLE) 매핑 반영. 기존 텔레옵(`amr_bringup bringup.launch.py`)은
  RoboClaw `/cmd_vel` 경로라 AK40 하부에서는 못 쓴다 (AK40 은 `/wheel_torque_cmd` 토크만 받는다).
- 경로: `joy_node → joy_teleop (/teleop/cmd_vel, TwistStamped) → wheel_pi_node (ω_ref = T⁻¹v, 바퀴별 PI, 비교군 A 게인) → torque_output
  → torque_command_gate (무장해야 통과) → ak40_can_node`. 추정기·MPC·센서·Cartographer 없음. `plant:=sim` 이면 시뮬 플랜트.
- 바꾼 파일:
  - `mpc/launch/teleop.launch.py` (새) — 인자 `plant` real|sim, `gamepad` xbox|f710, `cmd_timeout` 0.25 (wheel_pi; joy_teleop 이 20 Hz 라
    비교군 A 기본 0.05 s 로는 주기마다 끊김 판정), CAN 자동 준비는 torque_experiment 와 같다. stiction_comp 끔 (기준열이 없어 효과 없음).
  - `amr_bringup/config/joy_teleop_xbox.yaml` (새) — 실측 인덱스 (버튼 A0 B1 X3 Y4 LB6 RB7 View10 Menu11, 축 0 왼X 1 왼Y 2 오X 3 오Y 4 RT 5 LT).
    배치는 F710 판과 같다: 전후 = 왼쪽 스틱 Y(axis 1), 횡 = 오른쪽 스틱 X(axis 2, F710 은 3), 회전 = LB/RB(6/7, F710 은 4/5).
    속도 한계는 사용자 요청("동시에 하면 부하 100")으로 vx, vy 0.60 m/s · wz 1.80 rad/s — 세 축을 동시에 끝까지 밀 때 최악 바퀴
    (0.60 + 0.60 + 0.1932·1.80)/0.04 = 38.69 rad/s = AK40 정격 38.75 의 99.9 % (Tinv 부호 8 가지 검산). 이 경로에는 바퀴 속도를 자르는 곳이 없어서
    yaml 에서 맞춘다. F710 판의 0.72 m/s · 0.20 rad/s 는 매핑 정합용이다. (F710 yaml 주석의 "wz = 1.0 x 0.6 = 0.600" 은 실제 값 0.2 와 다르다 — 기존 주석, 안 고침.)
    `device_name` 으로 연다 (F710 이 같이 꽂히면 SDL index 가 바뀌므로). Menu = 무장, B = 해제.
  - `amr_base/joy_teleop_node.py` — `stamped_output`·`frame_id`·`arm_button`·`disarm_button`·`arm_topic` 파라미터 (기본값 = 예전 동작).
    기동 로그를 실제 매핑으로 (예전 "right stick x rotate" 는 yaml 과 달랐다). launch Ctrl+C 의 `ExternalShutdownException` 을 받아 traceback 없이 끝남.
  - `mpc/amr_mpc/nodes/fault_monitor.py` — `check_state_timeout` (기본 true). false 면 추정기 끊김만 안 본다 (모터 에러·바퀴 끊김은 본다).
  - `mpc/test/test_fault_monitor.py` — 새 1 (추정기 검사 끔 + 바퀴 끊김·모터 에러는 걸림).
- 확인: `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select amr_base amr_bringup amr_mpc` 통과
  (`~/.local` 의 setuptools 83 은 `develop --editable` 이 없어 colcon 이 일반 설치로 넘어갔다가 `develop --uninstall` 에서 실패한다 — 시스템 68.1.2 로 빌드).
  pytest: test_fault_monitor 4, test_launch_modes·test_command_gate 포함 128 통과. 격리 도메인 97 `plant:=sim` 에 가짜 /joy 단계 입력:
  전진 → cmd vx 0.72, ω_ref [18 18 18 18], 시뮬 vx 0.72 / 왼쪽 횡 → vy 0.72, ω_ref [−18 18 18 −18], vy 0.72 /
  LB → wz +0.20, ω_ref [−0.966 0.966 −0.966 0.966] / RB → −0.20 / B → DISARMED, 스틱을 밀어도 시뮬 0, 모드 DAMPING.
  F710 (`gamepad:=f710`) 은 axis 3·버튼 4/5·무장 버튼 없음으로 뜸. RoboClaw 경로 기본 파라미터는 `Twist on /cmd_vel` 그대로.
  **실차(AK40) 판 없음.**

### 2026-11-23 (9) — 구동계 방향 보정 launch 인자 `drive_yaw_deg`·`drive_slip_coef` (기본 0 = 보정 없음)
- 왜: 사용자 "추천대로". 근거 `analysis/torque_mpc_real/2026-11-23_horizon_real/RESULTS_KO.md` 4~6 절 — 접선 헤딩 8자 19 판 중 18 판에서
  몸체 왼쪽 0.6~2.3 cm 정상상태 추종 오차 (PP·C 모두). 바퀴가 굴러가는 축이 SLAM 몸체 축보다 +2.4° 반시계 (옆걸음 바퀴 비율로
  "대각선 반지름 차" 배제, 원인 후보 바퀴 toe) + 선회 옆미끄럼 −2.5 °/(m/s²) × a_c (R 0.3·0.5·0.75 에서 −2.25~−2.79, 시간차 아님).
  식: v_실제,xy = R(δ)·(T ω)_xy,  δ = δ0 + c·a_c,  a_c = |v|·ω_z (|a_c| ≤ 2 m/s²).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `mpc_cpp/src/torque_mpc.cpp` — TorqueModel `drive_yaw`·`slip_coef`·`slip_ac_max`.
    δ0: `force_map` xy 행을 R(δ0) 로 (→ G·B(k)·U_s·current_state 야코비안, kinematic–static duality). `correct_wheel_velocity` (δ0 + c·a_c).
  - `mpc_cpp/src/torque_mpc_node.cpp` — 파라미터 `drive_yaw_offset_deg`·`drive_slip_coef_deg`·`drive_slip_ac_max` (기본 torque_plant.yaml drive),
    범위 검사 (|δ0| ≤ 10°, |c| ≤ 10, 0 ≤ a_c 상한 ≤ 5), velocity_source wheels 의 엔코더 몸체 속도에 보정 (ω_z = 추정기 = 자이로), 기동 로그.
  - `mpc/amr_mpc/torque/model.py` — `drive_params`·`drive_angle`·`drive_Tinv` (ω = T⁻¹·R(−δ)·v). `TorquePlant.Tinv` 는 안 바꿈 (시뮬 플랜트 물리).
  - `mpc/amr_mpc/nodes/wheel_pi.py` (비교군 A) — 바퀴 목표 ω_ref = T⁻¹·R(−δ)·v_cmd (a_c = |v_cmd|·ω_cmd), speed_source estimator 의 변환도 같게.
  - `mpc/amr_mpc/nodes/torque_output.py` — δ0 만: 정지 마찰 방향의 기준 바퀴속도·슬립 가드 T⁻¹.
  - `mpc/config/torque_plant.yaml` — `drive:` 절 (yaw_offset_deg 0, slip_coef_deg_per_mps2 0, slip_ac_max_mps2 2) + 잰 값·근거 주석.
  - `mpc/launch/torque_experiment.launch.py` — 인자 `drive_yaw_deg`·`drive_slip_coef` → MPC·바퀴 PI (둘 다), torque_output (δ0). 숫자 아니면 거부.
  - `mpc/launch/real_all.launch.py` — `_PASS` 에 두 인자.
  - 시험: `mpc_cpp/test/test_torque_mpc.cpp` (새 2: force_map xy 행만 회전·G(θ) = G(θ+δ0)·보정 F⁺ 정확, correct_wheel_velocity·a_c 자름·기본 0 그대로),
    `mpc/test/test_drive_offset.py` (새 8: 보정 명령 → 원하는 운동, 보정 없으면 왼쪽 drift, 옆걸음 비율 tan(45°−δ), 자름, yaml 기본 0),
    `mpc/test/test_launch_modes.py` (새 6: 기본 안 넘김·MPC/바퀴 PI 둘 다·torque_output δ0 만·숫자 거부·real_all 전달).
- 안 바꾼 것: 시뮬 플랜트 물리, slip_logger λ (지표 정의 그대로 — 보정 λ 는 오프라인으로), 토크 램프·스텝, d̂ 식 (고친 B·측정으로 잔차가 저절로 바뀜).
- 확인: `colcon build --symlink-install --packages-select amr_mpc amr_mpc_cpp` 통과. gtest 43 통과 (새 2 포함). pytest 410 통과 (test_can_setup 빼고)
  + test_can_setup 8 통과·1 실패 (기존 `est_gains` KeyError, 11-22 (4) — 그 시험의 손으로 만든 launch 문맥에는 새 drive 인자도 없다).
  격리 도메인 199 기동 로그: 세 노드 모두 `drive_yaw_offset_deg:=2.4 drive_slip_coef_deg:=-2.5` 의 보정 줄이 찍힘, 인자 없으면 없음, 20° 는 기동 거부.
  **실차 판 없음.**
- 쓰는 법: δ0 는 로봇 보정값이라 A·C 둘 다 `drive_yaw_deg:=2.4`, 선회 몫은 C 에 `drive_slip_coef:=-2.5`. 계획: C ↔ C+δ0 ↔ C+δ0+선회 번갈아, A+δ0 로 새 기준.
  δ0 를 켜면 이전 판들과 직접 비교 못 한다 (몸체 왼쪽 치우침 y_b 가 0 으로 가는지가 가설 확인).
- 참고 (이번 변경과 무관): torque_output_node 를 `timeout -s INT … ros2 run` 으로 띄우면 SIGINT 로 끝나지 않는다 — 보정 없이도 같음 (launch 종료 경로는 안 봄).

### 2026-11-23 (13) — launch `friction_comp:=false`: torque_output 구동계 마찰 보상 전부 끔 (기본은 그대로 0.9)
- 왜: 교수님 "구동계 마찰은 d̂ 에 포함된 외란 — 빼라". d̂ 가 마찰을 대신할 수 있는지 시험하려고.
- 바꾼 것: `launch/torque_experiment.launch.py` 인자 `friction_comp` ("" | true | false). false 면 torque_output 에 `compensation_scale: 0.0`
  (coulomb·viscous·정지 마찰 항이 모두 이 값을 곱한다). `real_all.launch.py` `_PASS` 에 추가.
- 확인: `test_launch_modes` 130 통과 (새 8: 두 제어기 × "" / true / false, real_all 전달). 시뮬 7 판 + 실차 bag 계산 —
  `analysis/torque_mpc_sim/2026-11-23_fric_dhat/RESULTS_KO.md`: 보상을 빼면 위치 오차 34 → 62 mm, d̂ 상한·대역폭을 바꿔도 61–64 mm.
  원인은 상한 상자 위치 (보상이 곧 τ = τ_m − τ_f 변수 바꿈) — τ̄ ×1.3 이면 37 mm 로 돌아오지만 큰 슬립 0 → 1.9 %. 실차 판 없음.

### 2026-11-23 (12) — path following: 기준 시각을 로봇 진행에 묶기 (`figure8_reference` `path_follow_lead_s`, 기본 끔)
- 왜: 실차 계단 변조 8자 17 판에서 위치 오차 제곱의 66~86 % 가 진행 방향 뒤처짐 (최대 −0.29 m). 슬립을 막으려 토크를 줄이면 늦고 그게 오차가 됐다.
  사용자 제안 (경로만 주고 오차·슬립·랩타임 비교) 중 A 안 — 기준 노드만 바꿔 C·PP 가 같은 조건.
- 바꾼 것: 새 `amr_mpc/core/path_follow.py` (`project` 정사영 시각 — 직전 −0.2~+1.0 s 창·0.01→0.001 s, `decel_ahead`, `govern`).
  `figure8_reference.py`: 파라미터 `path_follow_lead_s` (음수 = 끔, 예전 동작). 주행 구간에서 t_ref ← clamp(t_ref, t_proj, t_proj + lead),
  정사영 지점 기준 속력 < 0.1 m/s 또는 앞 lead + 0.25 s 안에 감속이면 위쪽 상한 안 검. `/figure8/progress` [t, t_ref, t_clock, t_proj, 묶음],
  주행 끝 로그 "주행 끝: 실제 X s (기준 Y s, ±Z s)" (끈 때도). launch `path_follow_lead:=` (0~1 s, 주면 기준 발행 50 Hz) — real_all `_PASS`.
  실차 분석 `analysis/torque_mpc_real/2026-11-23_load_real/sweep_compare.py` 에 주행 시간·contouring 열.
- 확인: pytest 439 통과 · 1 실패 (`test_can_setup` est_gains, 다른 세션 변경 — 앞서부터). 새 `test_path_follow` 8 (정사영 ±3 ms, 교차점에서 다른 바퀴로 안 튐,
  clamp, 감속 앞 해제, 0.7 배속 로봇 주행 ≈ 1/0.7 배), launch 인자 7. 시뮬 (`analysis/torque_mpc_sim/2026-11-23_path_follow/RESULTS_KO.md`, 플랜트 h 0.2,
  마찰 밖 기준): PP 주행 9.77 → 10.90 s, contouring 157 → 137 mm, 슬립 그대로. C 는 시뮬에서 안 늦어 같음. 첫 구현은 끝 감속 직전에 묶여
  C 가 140 s 멈췄다 (가속도 앞먹임이 브레이크로 고정) → 감속 앞 해제로 고침. 실차 판 없음.

### 2026-11-23 (11) — `speed:=max` 계산에 하중 이동 (`fig8_limits` 바퀴별 N 상한·가중 배분) + launch `max_com_height`
- 왜: 실차 계단 변조 8자 (speed 0.69 · accel 3.0) 는 목표 입력이 상한의 최대 3.6 배 (bag U_s/τ̄(t), 시간의 24 % 가 > 1) — 마찰 밖 기준이라
  C 도 미끄러지거나 놓친다. C·PP 를 마찰 안 기준으로 비교하려면 h 를 넣은 상한으로 최대 speed 를 정해야 한다.
- 바꾼 것: `amr_mpc/core/fig8_limits.py` `plant_constants(..., com_height, load_min_frac, load_weighted_alloc, load_dynamic_cap)` (비우면
  torque_mpc.yaml load_transfer), `_ratio` — com_height > 0 이면 torque_mpc.cpp step 과 같은 식 (N_i, cap = τ̄ N/N0, U = W⁻¹Fᵀ(FW⁻¹Fᵀ)⁻¹ Fb).
  CLI `--com_height --speed --accel`. `torque_experiment.launch.py` 인자 `max_com_height` (비우면 mpc_com_height → yaml) — PP 도 같은 기준을
  받게 기준 계산에만 쓴다. 기동 로그에 h. `real_all.launch.py` `_PASS` 에 추가. h 0 (기본) 이면 예전 계산 그대로.
- 확인: pytest `test_fig8_limits` 9 (새 2: 스위치 둘 다 끄면 h 0 과 같음, 행별 식·힘 보존 F U = Fb), `test_launch_modes` 포함 122 통과.
  bag 대조: 같은 기준 (0.69 · 3.0) 에서 계산 3.92 (h 0.2, 창 위치 넷 중 최악) vs 실차 d̂ 끔 판 U_s/τ̄(t) 최대 3.64.
  실차 `speed:=max accel:=0.6 max_com_height:=0.2` → 0.55 m/s (앞먹임 0.997) 로 C 163519 · PP 163644 각 1 판 (배터리 방전):
  C RMS 22 mm · yaw 1.6° · 큰 슬립 4.0 %, PP 34 mm · 5.2° · 3.2 % (`analysis/torque_mpc_real/2026-11-23_load_real/RESULTS_KO.md`).
- 주의: 계단 변조에서 speed·accel 둘 다 max 면 accel 0.3 으로 speed 를 먼저 올려 h 0.2 에서 accel 0.05 (사실상 등속) 가 나온다 — accel 을 정해 줄 것.

### 2026-11-23 (10) — 토크 MPC 하중 이동: 바퀴별 수직력으로 목표 입력 가중 배분 + friction 상한 스텝별 (`load_transfer`, 기본 끔)
- 왜: 실차 계단 변조 8자 (C 151038 vs PP 151249) 에서 C 가 더 미끄러졌다 (25 vs 17 %). 바퀴별로 나누면 선회 안쪽만 —
  원심 가속 2.2~3.5 m/s² 에서 C 안쪽 앞 끌림 40 %·안쪽 뒤 헛돎 66 % (PP 2.6·47 %), 바깥 0. C 의 최소 노름 U_s 가 [−,+,+,−] 꼴로
  가벼워진 안쪽 앞바퀴에 제동 토크 (−0.03 N·m) 를 줬다 (PP 는 바퀴 속도 PI 라 +0.03). FL 기계 이상 아님 (사용자 확인).
  근거 `analysis/torque_mpc_real/2026-11-23_step_slip/RESULTS_KO.md`.
- 바꾼 것 (amr_mpc_cpp): `TorqueModel::wheel_normal_forces(a_x, a_y, h, min_frac)` — N_i = 몫_i m g − s_x m a_x h/(2L) − s_y m a_y h/(2W)
  (강체 축별 정역학, 아래 min_frac × 정지). `TorqueTuning load_com_height / load_min_frac / load_weighted_alloc / load_dynamic_cap`.
  `TorqueMPC::step` 지평 스텝마다 기준 관성 가속도의 몸체 성분으로 N_i,j → (1) U_s = argmin Σ (τ_i / c_i)², c_i = τ_μ,i N_i,j / N_i,0
  (가중 유사역행렬 W⁻¹Fᵀ(FW⁻¹Fᵀ)⁻¹, 몸체 힘 F U_s 그대로) (2) friction 상한 상자 τ̄_i,j = c_i (∧ 모터 한계), 변화율은 정지 상한 기준 그대로.
  target_projection·fallback 도 스텝별 상자. 노드: 파라미터 `load_com_height` 등 + `/torque_mpc/load` [t, N×4, τ̄×4] (스텝 0).
  `torque_mpc.yaml load_transfer` (com_height_m 0 = 끔). launch `mpc_com_height`, `mpc_load_weighted_alloc`, `mpc_load_dynamic_cap` (real_all 도).
  시뮬: `model.py` TorquePlant `wheel_base`·`robot_width`, SlipPlant `com_height` (직전 적분 스텝 가속도로 N_i, ≥ 0) + 적분 간격 보수화, launch `sim_com_height`.
- 확인: gtest 59 (새 `WheelNormalForcesLoadTransfer`, `LoadTransferWeightsTargetInputAndCaps`), pytest 421 · 1 실패 (`est_gains`, 무관).
  시뮬 (`analysis/torque_mpc_sim/2026-11-23_load_transfer/`, 계단 변조 8자 accel 3.0, μ 0.55): 플랜트 h 0.2 (실차 꼴 재현) 에서
  C 끔 큰 슬립 29~30 % (안쪽 앞 29 %) → C 켬 h 0.2 0 %, 위치 RMS 55 → 33 mm, 헤딩 2.8 → 0.4°. h 0.105 로 켜도 4 %. PP 21~22 %·RMS 235 mm.
  플랜트 h 0.105 에 제어 h 0.2 (과대) 도 슬립 0·추종 비슷. 무게중심 높이 실측 없음 (URDF 차체 상자 중심 0.105).
  실차 h 스윕 (같은 날 15:56~16:19, 13 판, 모터 38~41 °C, `analysis/torque_mpc_real/2026-11-23_load_real/RESULTS_KO.md`):
  평균 위치 RMS h 0 · 0.1 · 0.15 · 0.2 · 0.25 = 94 · 67 · 63 · 73 · 89 mm, 큰 슬립 27 · 26 · 21 · 16 · 12 % (PP 79 mm · 17 %).
  추종은 0.15 가 바닥, 슬립은 h 에 단조 감소. h 0.2 상한 고정 (가중 배분만) 은 추종 68 mm 인데 슬립 25 % — 슬립을 줄이는 건 스텝별 상한.
  yaml 기본값은 그대로 (com_height_m 0 = 끔) — 0.15 / 0.2 중 고르는 건 사용자 결정.

### 2026-11-23 (9) — check_localization 대기: 보정 0 도 정상, 최대 30 s (오경보 고침)
- 왜: (8) 은 서브맵 위치가 0.1 mm 넘게 옮겨져야 최적화로 봤는데, 초기 포즈가 정확한 판 (151038·151249) 은 첫 최적화 (+12.5 s) 가
  서브맵 위치·방향을 전혀 안 바꿔 (쿼터니언 기울기 성분만 1e-3 바뀜) 30 s 뒤 코드 5 (빨간 배너) 를 냈을 것. 또 144542 는 첫 최적화 보정 0,
  두 번째 (+24.3 s) 에 47 mm·1.2° — 지도와의 제약을 늦게 찾으면 보정이 늦다.
- 바꾼 것: `optimization_moved` 가 (거리, 각) 을 내고 문턱 1e-6 (아주 작은 보정도 최적화). `wait_optimization` 은 보정이 보이면 그때,
  30 s 동안 없으면 "고칠 게 없었다" 로 보고 다시 확인 → 0/4. 코드 5 는 새 trajectory 가 아예 안 생긴 경우만. real_all 배너 문구도.
- 확인: test_check_localization 2 개 통과, bag 11 판 재생 — 보정 있는 판은 +10.7~12.6 s (144542 는 +24.3 s), 없는 판은 30 s 에 정상 종료.
- 남은 한계: 보정이 30 s 뒤에 오면 못 잡는다. 근본 해결은 check_localization 의 초기 포즈 정확도 (지금 탐색 오차 16~112 mm·0.3~1.7°).

### 2026-11-23 (8) — check_localization --apply 가 Cartographer 첫 최적화를 기다린 뒤 끝난다 (출발 직후 SLAM 점프 원인)
- 왜: 실차 speed:=max 판들의 출발 약 1 s 뒤 /tracked_pose 계단 (121319 81 mm → 추정기 K_v 넘침으로 MPC 가 17 cm 가짜 오차, C 첫 꼭짓점이
  안으로 파고든 것처럼 보임). 원인: /initialpose 로 새 trajectory 가 시작되면 초기 포즈 어긋남 (이 도구 탐색 오차 16~112 mm·0.3~1.7°) 은
  포즈 그래프 최적화에서만 고쳐지는데 (localization_2d.lua optimize_every_n_nodes 20, 서 있으면 노드 0.5 s 마다 → +10.7~12.5 s),
  무장이 +6 s · 출발 +9.5 s 라 보정이 주행 중에 왔다. 무장 때 기준 원점 (origin_mode robot) 이 안 고친 포즈에 묶여 보정 = 가짜 오차.
  늦게 무장한 120544 만 첫 최적화가 서 있는 동안 끝나 점프 없음 (대조). 근거 `analysis/torque_mpc_real/2026-11-23_max_compare/RESULTS_KO.md`.
- 바꾼 것: `tools/check_localization.py` — `optimization_moved()` (새 trajectory 의 이미 본 서브맵 지도 좌표가 바뀌면 최적화) 와
  --apply 에서 /initialpose 전 가장 큰 trajectory id 를 잡고, 보낸 뒤 최대 30 s 동안 그 다음 trajectory 의 첫 최적화를 기다린 다음 다시 확인.
  못 보면 종료 코드 5. cartographer_ros_msgs 가 없으면 예전처럼 5 s. `launch/real_all.launch.py` 배너에 코드 5 문구.
- 확인: bag 6 판 /submap_list 재생 — 기다림이 +10.7~12.5 s 에 끝나며 보정 81·58·71·36·112·16 mm 를 잡음 (다섯 판은 실제 무장보다 늦음).
  새 `test/test_check_localization.py` (옛 trajectory 이동 무시, 새 서브맵은 기록만, 옮겨진 거리 0.10 m). pytest 404 통과 · 1 실패 (위 `est_gains`, 무관).
  실제 Cartographer 와 띄워 본 시험은 아직 없음. 사용: 측위 맞추기 배너가 예전보다 약 6 s 늦게 (적용 약 11 s 뒤) 나온다 — 배너 뒤에 무장.

### 2026-11-23 (7) — 정사각형도 `speed:=max`·`accel:=max` (변 직선 가속만 C 상한 안, 꼭짓점 짧은 포화는 허용)
- 왜: 사용자 "사각형도 speed:=max 되게, 1.5 1.5 로 했더니 돌아갔잖아". 실차 114214 (C, pass, accel 1.5, v_max 1.5) 는 꼭짓점 앞먹임/τ̄ 2.45 ·
  꼭짓점 시간 49 % 포화에도 완주 (SLAM RMS 45 mm, 최대 99 mm, 슬립 0.086·0.069 m/s ≈ 바닥 잡음). 꼭짓점 포함 ≤ 1 기준이면 pass 는 accel 0.40 이라 너무 보수적.
- 바꾼 것: `core/fig8_limits.py` `square_max` — accel = 0.05 내림(max_ff_ratio·4·min τ̄/(m_eff·r)) (직선 등가속 U_s = m_eff·a·r/4, w = 0),
  speed:=max → square_v_max 0 (figure8_reference 가 load_budget·V_LIMIT 에서 자름). 꼭짓점 포함 앞먹임은 기동 로그에 알려만 준다. CLI `--shape square`.
  `torque_experiment.launch.py` `_resolve_max` 가 square 를 받는다 (line·rotate 는 여전히 거부). 8자는 등속 원에서 상한이 계속 걸려 전체 구간 기준 그대로.
- 확인: test_fig8_limits +2 (지금 τ̄ 로 1.70, 한 칸 위 1.75 는 넘음, 변 최고 = 부하 예산, stop 은 꼭짓점까지 ≤ 1, accel 을 주면 그대로),
  test_launch_modes square 1 (square_v_max 1.5 를 줘도 speed:=max 면 0). pytest 403 통과 · 1 실패 (위 (6) 과 같은 `est_gains`, 무관).
  시뮬 launch 기동: "accel 1.70, 변 최고 1.55 … 꼭짓점 포함 최대 2.47, 부하 100 %" → figure8 "0.00→1.55→0.10 m/s 1.8 s × 4, 부하 주행 100 %" (거부 안 됨). 실차 판 없음.

### 2026-11-23 (6) — `speed:=max`·`accel:=max`: 원 크기만 주면 C 마찰 상한·부하 예산 안의 최대 speed·accel (`core/fig8_limits.py`)
- 왜: 사용자 요청 "원 크기 입력받고 낼 수 있는 최대로 설정되는 인자". 같은 날 손 계산 (`analysis/torque_mpc_sim/2026-11-23_fig8_max/`) 을 인자로.
- 바꾼 것: 새 `amr_mpc/core/fig8_limits.py` — figure8·circles8 에서 ① figure8_reference 와 같은 peak_load(K_GEOM) ≤ load_budget·r·max_wheel_rad_per_sec,
  ② torque_mpc 목표 입력 U_s = F⁺Rᵀ M (p̈_r − w) (0.2 s 차분, 창 0~0.15 s 앞당김 중 최악) 가 max_i |U_s,i|/τ̄_i ≤ max_ff_ratio (기본 1) 인
  가장 큰 speed (0.01 격자) → 그 speed 에서 가장 큰 accel (0.05 격자). τ̄ 는 늘 C 식 (μ̂_i·N_i·c_s, 이번 launch 의 mu_hat·mu_hat_wheels·
  wheel_load_share·payload 인자 반영) — A·B·C 가 같은 기준을 받는다. cap_on total 기준 (ground 면 보수 쪽). 외란 추정 â_d 는 0.
  `torque_experiment.launch.py`: `_track` 이 speed·accel 의 "max" 를 `_resolve_max` 로 숫자로 바꿔 figure8_reference 에 넘기고 기동 로그 한 줄,
  square·line·rotate·lap_speeds 와는 거부. 새 인자 `max_ff_ratio` (real_all `_PASS` 에도). 미리 보기 `python3 -m amr_mpc.core.fig8_limits --width 1.6 …`.
- 확인: 새 `test/test_fig8_limits.py` 5 개 (그날 τ̄ 로그 값 재현, 0.65 · 1.30 · 앞먹임 0.996 · 부하 87 % 재현, 한 칸 올리면 넘음, 여유 0.9,
  μ̂ 크면 부하 예산이 묶음 0.74), `test_launch_modes` 3 개. pytest 400 통과 · 1 실패 (`test_can_setup` 의 가짜 인자 표에 다른 세션이 넣은
  `est_gains` 가 없음 — 이 변경과 무관). 시뮬 launch 기동 (width 1.6): "speed 0.57 m/s, accel 1.25 m/s² (앞먹임 0.970, 부하 82 %)" → figure8 "순항 0.57 · 가감속 1.25".
- circles8 · tangent · laps 2 · speed_mod 0.5 × 2 · 위상 −90°, 지금 yaml μ̂: width 1.2 → 0.49 · 1.20, 1.6 → 0.57 · 1.25, 2.0 → 0.65 · 1.30,
  2.4 → 0.71 · 1.35, 3.0 → 0.80 · 1.40 (전부 C 상한이 묶음, 부하 78~97 %). 실차 판 없음.

### 2026-11-23 (5) — 실차 launch 실패 때 고아 노드가 남지 않게 (`real_all.launch.py`)
- 왜: 11-23 11:36 CAN 어댑터가 안 잡혀 `amrall` 두 번 실패 ("CAN setup failed for can0") → cartographer 묶음 (앞·뒤 스캔 필터, dual_scan_merger,
  점유 격자, pose_to_odom, set_initial_pose) 이 판마다 6 개씩, 12 개가 부모 PID 1 로 남았다. 다음 판에서 스캔·초기 포즈가 두 번씩 나가
  cartographer "Dropped ~1725 earlier points" 35 회/s (평소 4 회/s · 4~5 점), 추정기 "SLAM 포즈 튐 358.9 cm / 다시 초기화" 반복. 손으로 kill -INT 해 정리.
- 원인: real_all `_nodes` 가 bag·센서·cartographer 를 먼저, torque_experiment 를 뒤에 넣었다. torque_experiment 의 OpaqueFunction 이
  CAN 준비 실패에서 예외를 던지면 launch 는 "Caught exception" 으로 끝나고, 그 전에 방문한 ExecuteProcess 는 종료 신호 없이 남는다.
  launch 동작 시험 (sleep 프로세스 + 예외 OpaqueFunction): 프로세스가 먼저면 고아 1, 예외가 먼저면 0.
- 바꾼 것 (`mpc/launch/real_all.launch.py`): ① torque_experiment include 를 맨 앞으로 (CAN·인자 검사 예외가 다른 프로세스보다 먼저).
  ② `_leftover_nodes()` — 부모가 PID 1 (또는 systemd) 이고 이 작업공간 install/ 이나 /opt/ros/ 실행 파일이며 ROS_DOMAIN_ID 가 같은 프로세스가
  있으면, 아무것도 띄우기 전에 멈추고 `kill -INT <pid…>` 를 알려 준다 (ros2 daemon·ros2 launch 는 뺌).
- 시험 (`mpc/test/test_launch_modes.py` 새 3): torque include 가 첫 프로세스 동작 (bag·센서·cartographer 켬), 고아가 있으면 예외·정리 명령,
  실제 이중 fork 고아 (같은 도메인 찾음 · 다른 도메인 안 찾음). pytest 392 통과 (1 실패 test_can_setup — 기존 `est_gains`).
- 남은 것: 실차 real_all 로 CAN 실패 재현 시험은 실행 중 판이 있어 안 함.

### 2026-11-23 (4) — 토크 MPC 지평 기본값 Np 50 → 75, Nc 25 → 40 (`torque_mpc.yaml tuning`)
- 왜: 사용자 결정 "75 / 40 으로". 근거 2026-11-23 (3) (analysis/torque_mpc_sim/2026-11-23_horizon/RESULTS_KO.md).
- 바꾼 파일: `mpc/config/torque_mpc.yaml` (값·근거 주석), `mpc/launch/torque_experiment.launch.py` (`mpc_Np` 설명의 기본값 표기). 코드·구조체 기본값 (gtest 용) 안 바꿈.
  Np 를 읽는 곳은 torque_mpc_node 뿐 (PP·바퀴 PI 는 Ts 만, 옛 속도 MPC 는 plant*.yaml).
- 확인: pytest 389 통과 (1 실패 test_can_setup — 기존 `est_gains` KeyError, 11-22 (4)), 격리 도메인 노드 기동 로그 `Np 75  Nc 40`. 기준열 4 s 라 0.75 s 지평은 안에 든다.
- 예전 값: `mpc_Np:=50 mpc_Nc:=25` (11-23 (2)) · `mpc_Np:=50 mpc_Nc:=5` (11-23 전). **실차 기록 비교 때 지평 값을 같이 볼 것.** 실차 판 없음
  (첫 판에서 status[12] 계산 ms 최악값 확인 — 시뮬 두 판 동시 최대 5 ms).

### 2026-11-23 (3) — 예측 지평 launch 인자 `mpc_Np` + 지평 (Np, Nc) 고르기 계산·시뮬 (yaml 기본값은 안 바꿈: Np 50 · Nc 25)
- 왜: 사용자 "시뮬상에서 최적의 N_p 랑 N_c 예측 가능? 아님 계산상으로라도".
- 바꾼 파일: `mpc/launch/torque_experiment.launch.py` (인자 `mpc_Np` 5~300 → 노드 파라미터 `Np`, `mpc_Nc` 범위를 Np 기준으로), `mpc/launch/real_all.launch.py` (_PASS),
  `mpc/test/test_launch_modes.py` (지평 시험을 Np·Nc 같이: 덮어쓰기 3 · 거부 3). test_launch_modes.py 102 통과.
- 결과 (analysis/torque_mpc_sim/2026-11-23_horizon/RESULTS_KO.md): 계산 (무한 지평 LQ 기준) Np ≈ 100 · Nc ≥ Np/2 (K 차 1.2 %, 지금 50/25 는 21 %),
  Nc < Np/2 이면 긴 Np 가 더 나쁨. 시뮬 스택 (8자 R 0.3, 두 판씩): 50/25 → 75/40 시간 추종 RMS 25 → 21.6 mm (d̂ 켬) · 26.4 → 21.4 (끔),
  100/50 은 더 안 나아지고 계산 99 % 6.5~7 ms. 75/40 계산 평균 1.5 ms · 최대 ~5 ms. 실차 판 없음.

### 2026-11-23 (2) — 토크 MPC 제어 지평 기본값 Nc 5 → 25 (`torque_mpc.yaml tuning.Nc`)
- 왜: 사용자 결정 "25 로". 근거 2026-11-23 (1) 시뮬·11-21 (13) 이상 플랜트 — Nc 5 의 move blocking 이 8자 곡률 전환에서 안쪽 치우침을 만든다.
- 바꾼 파일: `mpc/config/torque_mpc.yaml` (값·근거 주석). 코드·구조체 기본값 (TorqueTuning Nc 5, gtest 용) 은 안 바꿈. 옛 속도 MPC 는 plant*.yaml 을 읽어 무관.
- 확인: pytest 386 통과 (1 실패 test_can_setup — 11-22 (4) 에 적힌 기존 `est_gains` KeyError, 이번과 무관), 격리 도메인 노드 기동 로그 `Np 50  Nc 25`.
- 예전 값: `mpc_Nc:=5`. **이날 전 C 실차·시뮬 기록은 모두 Nc 5** — 비교할 때 같은 값인지 볼 것. 실차 판 없음 (첫 판에서 status[12] 계산 ms 최악값 확인).

### 2026-11-23 (1) — MPC 제어 지평 launch 인자 `mpc_Nc` (기본 비움 = yaml tuning.Nc 5) + Nc 25 시뮬
- 왜: 11-21 (13) 원인 가르기 — Nc 5 뒤 τ 를 고정하는 move blocking 이 8자 곡률 전환을 예측 못 해 안쪽 치우침 (이상 플랜트 +5~10 mm).
- 바꾼 파일: `mpc/launch/torque_experiment.launch.py` (인자 `mpc_Nc` → torque_mpc_node 파라미터 `Nc`, 1~50 밖이면 거부), `mpc/launch/real_all.launch.py` (_PASS),
  `mpc/test/test_launch_modes.py` (새 3: 덮어쓰기·범위 거부·real_all 전달). yaml 기본값은 안 바꿈.
- 확인: test_launch_modes.py 99 통과. 시뮬 8자 R 0.3 (analysis/torque_mpc_sim/2026-11-23_nc25/RESULTS_KO.md):
  move blocking 만 남긴 조건에서 s 랩 1/2/3 +4.8/+13.4/+24.1 → +2.4/+6.5/+12.4 mm, 시간 추종 RMS 17.3 → 8.0 mm.
  현실 조건 (추정기·μ 0.45·C·d̂ 켬, 두 판씩) 시간 추종 RMS 32 → 24 mm, Δτ RMS 0.0065~0.0071 → 0.0054~0.0058, 큰 슬립 0 %,
  s 평균 랩 1/2/3 +10.2/+10.6/+30.4 → +2.6/+3.5/+18.5. 계산 평균 0.6 ms, 한 판에서 스택 전체가 밀릴 때 최대 12.6 ms. 실차 판 없음.

### 2026-11-22 (4) — 바퀴별 μ̂ 를 N_i 기준으로: `mu_hat_wheels` [0.598, 0.489, 0.547, 0.703] + friction 상한에 하중 몫 곱함
- 왜: 사용자 결정 — 바퀴별 μ̂ 는 "바퀴별 다른 하중을 고려해서 구한 값". 11-19 (11) ~ 11-21 코드는 `mu_hat_wheels` 를 mg/4 기준으로 읽어
  하중 몫(`body.wheel_load_share`)을 안 곱했다 → 측정한 하중 몫이 C 상한에 안 들어갔다.
- 식: τ_μ,i = (r/√2)·μ̂_i·N_i·c_s, N_i = 몫_i·m g, μ̂_i = `mu_hat_wheels[i]` (> 0) 아니면 `mu_hat`. 하중 몫 1/4 이면 예전 mg/4 식과 같다.
- 값: [0.572, 0.468, 0.570, 0.734] (mg/4 기준) → [0.598, 0.489, 0.547, 0.703] (N_i 기준). τ̄_i = 1.0122·μ̂_i·몫_i
  → [0.1445, 0.1184, 0.1444, 0.1859] N·m. 11-21 상한 [0.1447, 0.1184, 0.1442, 0.1857] 과 ±0.1 % 안 — 같은 측정의 다른 표현이라 C 주행은 사실상 그대로.
  (코드만 바꾸고 값을 그대로 뒀다면 FL −4.5 %, FR −4.3 %, RL +4.3 %, RR +4.5 % 로 달라졌다.)
- 바꾼 파일:
  - `mpc_cpp/src/torque_mpc.cpp` — `tau_friction_limits()` 가 바퀴별 μ̂ 에도 `wheel_load_share` 를 곱한다
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `mpc_cpp/src/torque_mpc_node.cpp` — 주석, 시작 로그 "(N_i 기준, - = μ̂ 0.30)"
  - `mpc_cpp/test/test_torque_mpc.cpp` — `MuHatWheelsOverrideLoadShare` → `MuHatWheelsUseLoadShare` (N_i 식, 몫 1/4 = mg/4 식 확인)
  - `mpc/config/torque_plant.yaml` — `surface.mu_hat_wheels` 값·기준 주석, `body.wheel_load_share` 쓰는 곳 주석
  - `mpc/launch/torque_experiment.launch.py`, `mpc/test/test_launch_modes.py` — 인자 설명·docstring "mg/4 기준" → "N_i 기준"
- 확인: `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select amr_mpc_cpp amr_mpc` (이전 설치가 egg-link 라 `~/.local`
  setuptools 83 으로는 `--uninstall` 에서 실패), `test_torque_mpc` 41/41 통과, `test_launch_modes.py` 통과.
  `torque_mpc_node limit_mode:=friction` 시작 로그 τ̄ [0.1445 0.1184 0.1444 0.1859] = 손계산.
  `test_can_setup.py::test_real_launch_does_not_start_nodes_if_setup_fails` 는 이번 변경과 무관하게 실패 — 11-21 `est_gains` 인자를
  가짜 인자 목록에 안 넣어 `KeyError: 'est_gains'`.
- 남은 것: `tools/mu_ramp_test.py` 의 바퀴별 μ 출력은 아직 mg/4 기준 (√2|τ_g,i|/r / (mg/4)) — 그 값을 `mu_hat_wheels` 에 넣으려면 4·몫_i 로 나눈다.

### 2026-11-22 (3) — 추정기 정리: kf 게인 후보 채택 안 함 · IMU(추적점) 위치 확인 끝 (`torque_estimator.yaml` 주석만, 값 안 바뀜)
- kf 게인: 2026-11-21 실차 A/B (C, circles8 0.7 m/s, 지금 225012·225104·225157 vs `est_gains:=candidate_1121` 225302·225349·225435,
  analysis/torque_mpc_real/2026-11-21_kf_gain_ab/RESULTS_KO.md). SLAM 포즈 기준 추종 5.0 → 4.8 cm (판 사이 차이 안), 추정 속도 떨림 1.57 배,
  Δτ +10 %, 최대 |τ| +22 %, 큰 슬립 (SLAM 기준 λ) 5.1 → 8.6 %, innovation ρ₁ +0.3 → −0.08~−0.20 (과보정). 사용자 결정: 지금 값 유지.
  `estimator_presets.candidate_1121` 는 비교용으로 남기고 주석에 결과를 적었다.
- IMU 위치 (`imu_offset` 0.1093, 0.017 = 실행 TF base_footprint → camera_gyro_frame 0.1093, 0.0173): yaml To do "제자리 회전으로 다시 확인"
  을 끝냈다. 제자리 스핀 2 판 (bag 2026-11-22/193123 ω 1.0, 193243 ω 1.5, `shape:=rotate seg_angle_deg:=720` 왕복) 에서
  추정 중심 속도 − 바퀴 기구학 속도를 ω 에 회귀 → 실제 추적점 (0.1063, 0.0223) m (δ −3.0, +5.3 mm, 두 ω 에서 같음).
  영향 ω 1.3 rad/s 에서 중심 속도 8 mm/s — 사용자 결정: 값은 그대로, 기록만 (analysis/estimator_tuning/2026-11-22_lever_arm/RESULTS_KO.md).
  기존 bag 의 짧은 회전 구간 (코너·정렬) 으로는 고르는 방법에 따라 δ 가 수 cm 씩 바뀌어 못 정했다.
- 남은 것: 튐 문턱 jump_xy 15 cm·jump_yaw 15° (실제 innovation 중앙값 약 5 mm·0.05° 보다 훨씬 크다) — 급하지 않음.

### 2026-11-22 (2) — 속도 변조 모양 `speed_mod_shape: sin | square` (기본 sin = 예전)
- 왜: 사용자 "P1 은 PP 도 잘 따라가 보인다, 속도·가속을 키우거나 가감속 구간을 다르게". square = V(1±ε) 계단 + accel 가감속 (호길이 forward–backward pass).
- 바꾼 파일: `core/figure8.py` (`_build_square_profile`), `figure8_reference.py`, `torque_track.yaml`, `torque_experiment.launch.py` (_TRACK_KEYS · 문자열 · 허용값),
  `real_all.launch.py` (_PASS), `test_figure8.py` 2 개 (|a_t| ≤ accel, 교차점 느린 쪽, σ–v 유한차분 맞물림). pytest 134 통과.
- 계산 (`analysis/torque_mpc_sim/2026-11-22_fig8_speedmod/RESULTS_KO.md` 끝 절): P1 보다 세게 하면 C 상한 사용률이 1 을 넘는다 (sin 0.35–1.05: 1.22, square accel 1.5: 1.60).
  바닥 토크 상한이면 0.82 · 1.07. 실차 판 없음.

### 2026-11-22 (1) — 가감속 8자: 속도 변조 위상 `speed_mod_phase_deg` (기본 0 = 예전) + 후보 시뮬
- 왜: 사용자 "경로 곳곳에 가감속 구간이 있는 플래너, C 가 A 보다 우위인 가감속 양". 판단 기준 = 가감속 8자에서 C > A (사용자 지시, 이 날부터).
- 바꾼 것: `core/figure8.py` v_target = V(1 + ε sin(2πnσ/ℓ_lap + φ)), `figure8_reference.py` 파라미터·기동 로그, `torque_track.yaml`,
  `torque_experiment.launch.py` (_TRACK_KEYS), `real_all.launch.py` (_PASS), 시험 `test_figure8.py` 2 개 (φ −90° 교차점 최저, NaN 거부). pytest 132 통과.
- 후보 (`analysis/torque_mpc_sim/2026-11-22_fig8_speedmod/`, width 2.0 · 2 바퀴 · φ −90°, 시뮬 바닥 μ 0.45, 1 회차씩): P1 (V 0.6 · ε 0.5 · n 2, 0.30~0.90 m/s)
  에서 C (엔코더 속도 + 새 d̂ + 슬립 가드) 가 A 대비 경로 거리 13.7 → 9.0 mm, 교차점 18.4 → 7.1, 헤딩 3.35 → 0.69°. P2·P3 (구간 더 많음) 는 우위가 작다. 
- 실차 P1 (C 002707·002931·003146, A 002822·003041, `analysis/torque_mpc_real/2026-11-22_fig8_P1/RESULTS_KO.md`): 경로 거리 비슷 (C 1.9–2.7 vs A 2.3–2.4 cm),
  헤딩 RMS 1.2–1.4 vs 4.2° (최대 3.0 vs 16.6°), 시간 기준 최대 5.2–8.4 vs 10–12 cm, **요 흔들림 0.088–0.101 vs 0.095–0.115 rad/s (C 가 처음으로 A 이하)**,
  가속 구간 lag +0.2 vs +1.5 cm. 모두 완주.
  A 3 회차째 003451 추가 (3 대 3): 범위가 안 겹치는 것 — 헤딩 RMS·최대, 시간 기준 최대 (5.2–8.4 vs 9.0–12.0 cm), 가속 구간 lag, v_y 흔들림, 교차점 최대. 경로 거리·요 흔들림은 겹친다.

### 2026-11-21 (15) — 바퀴별 슬립 가드 (`slip_guard:=true`, torque_output_node, 기본 끔)
- 왜: 사용자 "바퀴별 슬립 감지". C 는 바퀴별 속도 되먹임이 없어 직선 급가속에서 FL 출발 헛돎·RR 제동 잠김 — 바퀴별 μ̂ 상수로는 미끄러지는 바퀴가 옮겨 다녔다
  (`analysis/torque_mpc_real/2026-11-21_line_accel/RESULTS_KO.md`). A 의 바퀴별 PI 는 헛도는 바퀴 토크가 스스로 줄었다.
- 바꾼 파일: `mpc/amr_mpc/torque/slip_guard.py` (새, leave-one-out 기구학 잔차 + 바퀴별 이득 g), `mpc/amr_mpc/nodes/torque_output.py`
  (파라미터 `slip_guard`·`imu_topic`·`slip_*`, /imu/base_link 구독, 순토크 × g, 상태 /torque_output/slip_guard), `mpc/launch/torque_experiment.launch.py`
  (인자 `slip_guard`), `mpc/launch/real_all.launch.py` (`_PASS`), 시험 `mpc/test/test_slip_guard.py` 10 · `test_launch_modes.py` 4.
- 감지: 자이로 ω_z 와 나머지 세 바퀴로 ω̂_i, σ_i = (ω_i − ω̂_i)·sign(τ_i)/max(|ω̂_i|, 5). 가장 큰 바퀴가 0.5 를 50 ms 넘으면 g_i 를 8 /s 로 줄임 (≥ 0.4), 풀리면 2 /s.
  대각 짝은 기구학으로 못 가르고 토크 부호로 가른다. 공통 슬립은 안 보인다.
- 확인: bag 미리보기 (`analysis/torque_mpc_real/2026-11-21_slip_guard/detect_rule.py`) — C 슬립 판 FL 16~27 % · RR 8~10 % 판정, A 정상 ≤ 1.5 %.
  시뮬 (직선 accel 4 · 8자, 바닥 μ 0.45 / 없음) 판정 0 % · g 1.0 — 오탐 없음. 시뮬 바닥 μ 가 네 바퀴 같아 한 바퀴 슬립 줄이는 효과는 실차 판 필요.
  pytest 두 파일 102 통과.
- 실차 1 차 (233612 · 233706, accel 5, `analysis/torque_mpc_real/2026-11-21_slip_guard/RESULTS_KO.md`): 233706 FL 헛돎을 문턱 0.5 로 +0.15 s 에야 잡았다
  (헛돎 시작 −0.10 s) → 기본 문턱 0.5 → **0.35** (`slip_guard.py`·`torque_output.py`). 233612 는 헛돎 자체가 안 났다 (가드 판정 0) — 판마다 다르다.
- 실차 2 차 (문턱 0.35, 켬 234313·234437·234559 / 끔 234356·234520·234636, accel 5, RESULTS 2 차 절): 제동 RR 이 6 판 중 5 판에서 거꾸로 돈다.
  가드는 RR 을 잡아 g 0.4 로 줄여 역회전 최대를 반쯤 (−0.21 vs −0.44 m/s, 1 쌍) 줄였지만 없애지 못했다. 마찰 보상 coulomb 항이 측정 ω 방향을 따라 미끄러지는 쪽을 민다.
  가속에서 앞 (FL)·제동에서 뒤 (RR) 가 미끄러지는 패턴은 앞뒤 하중 이동과 맞다.

### 2026-11-21 (13) — C 8자 안쪽 치우침 원인 가르기 (분석만, 코드 변경 없음)
- 왜: (12) 의 ρ 로는 치우침이 반쯤만 줄 것 (실차 잔차 ρ ≈ 0.9) → 사용자 "시뮬로 원인 먼저 가리기".
- 한 것: 시뮬 8판 (d̂ 끔, 추정기 → 참값 → 슬립 없음 → 상한 B → 마찰 = 보상량) + 이상 플랜트 C++ (Nc·지연·기준 가속도 창·외란 방향).
  analysis/torque_mpc_sim/2026-11-21_bias_source/ (RESULTS_KO.md, ideal/*.cpp), 실차 RESULTS 21 절.
- 결과: (A) 시간 매개 추종의 rotating frame 강성 (k_p − ω²) — 진행 방향 끌림이 안쪽 오차로 (δ_t −0.05 m/s² → +6.8 mm @ R 0.3 v 0.61, 이상 플랜트로 식 검증).
  실차 끌림 d_x −0.15 m/s² 가 d̂ 끈 C 치우침의 대부분, 몸체 d̂ 가 이를 메운다. (B) move blocking Nc 5 → 8자 +5~10 mm, Nc 25 면 +0.6~1.4 (계산 0.5 ms).
  원인 아님: 지연 0~40 ms, acc_dt 창, 측정 어긋남. 시뮬 스택에 이상 플랜트보다 8 mm 더 있는 몫은 못 가름.

### 2026-11-21 (14) — 바퀴별 μ̂ 기본값 [0.432, 0.44, 0.456, 0.417] → [0.572, 0.468, 0.570, 0.734] (`torque_plant.yaml surface.mu_hat_wheels`)
- 왜: 사용자 "mu_hat_wheels:="0.572,0.468,0.570,0.734" 으로 수정해줘 (최신 업데이트임)". 근거 측정은 사용자 쪽 (이 리포에 기록 없음).
- τ̄_i = 0.2530·μ̂_i → [0.1447, 0.1184, 0.1442, 0.1857] N·m (예전 [0.1093, 0.1113, 0.1154, 0.1055]). 옆·요 힘 0 으로 낼 수 있는 앞 (= 옆) 가속 최대
  (힘 분배 LP, m + 4J_w/r² = 6.79 kg) 1.58 → **1.94 m/s²** (사용자가 인자로 쓰던 [0.486 …] 1.78). 좌우 균형 때문에 짝 바퀴의 작은 상한에 묶여 RR 0.734 는 다 못 쓴다
  (네 상한 합으로 치면 2.19). 같은 날 직선 스윕의 바닥 견인 한계 ≈ 3.0 m/s² (`2026-11-21_line_accel/RESULTS_KO.md`).
- 설치본이 소스에 링크 — 다음 `amrall` 부터. `mu_hat_wheels:=` 인자를 주면 그 값이 우선. 예전 값: `mu_hat_wheels:="0.432,0.44,0.456,0.417"`.
- 확인: pytest test_launch_modes 88 통과. 실차 판 없음 (22:3x 에 떠 있던 C 직선 accel 4.0 판은 인자로 0.486… 을 줘서 영향 없음).

### 2026-11-21 (13) — kf 게인 튜닝 시험 인자: `est_gains`·`est_noise` (추정기 잡음 값), `sim_noise` (시뮬 센서 잡음·시간 상관 SLAM 오차). 기본 동작 안 바뀜
- 왜: 실차 bag 37 개 innovation 분석 (analysis/estimator_tuning/2026-11-21_kf_innovation/README_KO.md) 에서 지금 kf 게인이 작다고 나왔다
  (ρ₁ > 0 이 xy 35/35·yaw 35/35 bag). 사용자 "라이다를 더 믿는 방향으로 튜닝할 때 걸림돌 2 가지를 시험할 수 있는 런치". 걸림돌:
  ① 주행 중 SLAM 오차가 시간 상관이면 게인을 키울수록 추정이 그 오차를 따라간다 (정지 SLAM 떨림은 ρ(0.1 s) 0.4~0.76 — 주행은 못 배제)
  ② K_v 4.16 → 7.89 로 포즈 잡음이 MPC 속도 상태로 더 들어간다 (토크 떨림).
- 바꾼 파일:
  - `amr_mpc/torque/estimator.py`: `NOISE_KEYS` (게인을 정하는 잡음 값 6 개 이름).
  - `amr_mpc/nodes/state_estimator.py`: 잡음 값 6 개를 ROS 파라미터로도 받는다 (기본 = yaml). 기동 로그에 값과 "yaml 과 다름" 표시.
    ROS 파라미터라 실차 판 run_info/params 에 어느 값이었는지 남는다.
  - `config/torque_estimator.yaml`: `estimator_presets.candidate_1121` (σ_a 0.14, σ_b 0.04, σ_p 2 mm, σ_ψ 0.05°) — estimator 절은 그대로.
  - `amr_mpc/torque/sim_sensors.py`: 1차 Gauss-Markov SLAM 오차 `slam_gm_xy`·`slam_gm_yaw_deg`·`slam_gm_tau` (기본 0 = 없음, 따로 둔 난수열이라
    끄면 예전 판과 같은 잡음), `parse_overrides()`.
  - `amr_mpc/nodes/sim_plant_torque.py`: 파라미터 `sensor_override` "키=값,…" (sensor_noise_scale 을 곱한 뒤 그 값으로).
  - `launch/torque_experiment.launch.py`: 인자 `est_gains` (비움·yaml | candidate_1121), `est_noise` ("키=값,…", est_gains 뒤),
    `sim_noise` ("키=값,…"). 잘못된 값은 CAN 준비 전에 멈춘다. `launch/real_all.launch.py`: `_PASS` 에 est_gains·est_noise.
  - `test/test_estimator_tuning_args.py` (새): 기본은 yaml 그대로, 묶음·덮어쓰기 순서, 잘못된 값 거부, real_all 전달, 후보 게인 값,
    Gauss-Markov 오차의 σ·lag-1 상관 (= exp(−T/τ)), 끄면 white.
- 확인: pytest test_estimator_tuning_args·test_torque_estimator·test_launch_modes·test_figure8 166 통과. 시뮬 기동 (nice 19, MPC 없이):
  `est_gains:=candidate_1121 est_noise:=gyro_noise_density=0.0128` → K_xy [0.881, 7.89, −2.18], K_psi 0.958, "yaml 과 다름" 표시,
  `sim_noise:=slam_gm_xy=0.004,slam_gm_tau=0.5,slam_noise_xy=0.0015` → 플랜트 "센서 잡음 덮어쓰기" 로그.
  symlink 설치라 바로 들어간다 — 22:33 에 뜬 실차 판 (bag 223310) 추정기 로그가 예전과 같은 게인 (K_p 0.711, K_v 4.16, K_psi 0.440) 으로 정상 기동.
- 쓰는 법: 실차 A/B `amrall … est_gains:=candidate_1121` ↔ 인자 없이. 시뮬 시험 판 구성은 analysis/estimator_tuning/2026-11-21_gain_risk/.

### 2026-11-21 (12) — offset-free 정석화: 외란 관측기 Kalman 이득 · 목표 입력 사영 · 구심 feedforward 계수 ρ (d̂ 와 결합 Kalman 추정). 모두 기본 끔
- 왜: 사용자 "우리 구현을 정석으로" → 1·3·4 번 선택. 근거 Rawlings·Mayne·Diehl 2017 1장 (offset-free 는 외란이 일정하고 예측 모델에 들어가야),
  Pannocchia & Rawlings 2003 (증강 외란 모델 x⁺ = Ax + Bu + B_d d), Muske & Rawlings 1993 (target calculation).
- 바꾼 것 (`mpc_cpp` torque_mpc.hpp·cpp, torque_mpc_node.cpp):
  - 관측기 `dist_kalman`: 축마다 random walk 가속도 외란 + 잔차 [r_p; r_v] 측정의 정상상태 Kalman 이득 (Riccati 반복, `steady_kalman_gain`). 파라미터
    `disturbance_observer` (lpf | kalman)·`disturbance_kalman_q`·`_r_pos`·`_r_vel`. R 은 실차 잔차 표준편차 (RESULTS 19 절) → 등가 α 0.0175 · 0.0103 · 0.0111.
  - `target_projection`: U_s 를 바퀴별 [−τ̄, τ̄] 에 clamp, 잘린 주기는 d̂ 갱신 멈춤에도 쓴다.
  - 구심 feedforward 계수 ρ: 곡률 중심 쪽 구심 몫 c_n 의 η = 1 − ρ 를 가속도 외란 a_ρ = η c_n n̂_c / m_eq 로 U_s (빼기) **와 예측 (더하기)** 에 같이.
    `ff_normal_scale` 고정 또는 `ff_normal_estimate` = d̂ 와 한 Kalman 필터 (상태 [d_x, d_y, η] 몸체, 측정 몸체 잔차, η 열 = [½Ts²; Ts] ⊗ g) — observer kalman·frame body 필요.
    파라미터 `ff_normal_q`·`_min`·`_max`·`_P0`·`_x_min`. 새 토픽 `/torque_mpc/adapt` = [t, ρ, η 분산, η 측정 0/1, |g|, 잔차 ĝ 성분, d̂ 건너뜀 코드].
  - 처음 만든 판 (feedback 힘을 오차로 쓰는 RLS — feedback-error learning 꼴) 은 버렸다: d̂ 와 같이 켜면 d̂ 가 오프셋을 지운 뒤 학습이 멈춰
    8자 시험 ρ̂ 0.92 (참값 0.7), 반경 +6.4 / +6.6 mm (d̂ 만 −5.1 / −4.9 보다 나쁨). 또 ρ 를 U_s 에만 넣으면 평형이 r' = (ρ̂ + β) R 로 어긋났다 (−13.4 mm).
  - torque_mpc.yaml: disturbance.observer lpf·kalman_q·kalman_r_pos·kalman_r_vel, 새 `feedforward` 블록 (target_projection false, normal_scale 1, normal_estimate false, …).
    launch `mpc_dist_observer`·`mpc_target_projection`·`mpc_ff_normal_scale`·`mpc_ff_normal_estimate`·`mpc_ff_normal_q` (real_all `_PASS`).
- 확인: gtest 39 + 12 (새 5: Kalman 이득 = 스칼라 Riccati, Kalman 몸체 d̂ 원 위 +54.4 → +0.6 mm, 사영, 원 하나 ρ 추정 −26.2 → −0.0 mm·힘 없으면 ρ̂ 1.000,
  8자 d̂ + ρ 추정 −0.8 / −0.5 mm vs d̂ 만 −5.1 / −4.9 · ρ̂ 0.733 (참 0.7) · 몸체 â_d,y +0.112 / +0.072 (참 0.09)), pytest 347 (새 4), 노드 기동 (격리 도메인) 로그·잘못된 조합 거부.
  시뮬 8자 C (analysis/torque_mpc_sim/2026-11-21_offset_free/, 판 하나씩): e_c 랩 1/2/3 lpf +6.7/+10.4/+31.9 · kalman +2.9/+10.3/+25.1 · +ρ +7.8/+9.5/+39.6 ·
  +ρ+사영 +3.7/+9.1/+27.1 mm — 같은 설정 반복 차 8.5 mm 안. Δτ RMS 0.0063~0.0065 같음, 큰 슬립 0 %. 시뮬엔 구심 어긋남이 없어 ρ̂ 0.98~1.05 (1 근처 유지).
  실차 bag 잔차 (RESULTS 20 절): ρ 0.90~0.93 (batch LS, η ± 0.10), d_y +0.07~0.10, d_x −0.15~−0.16 m/s² — **구심 어긋남은 작다**. 17 절의 "U_s 가 1.4 배 → κ 0.7" 과
  "치우침 = (U_s − 실제)/k_p 검산" 은 정정 (검산은 제어 법칙의 항등식). 실차 판 없음.
- 예전 동작: 아무 인자도 안 주면 그대로 (observer lpf, 사영 끔, ρ 1 고정).

### 2026-11-21 (11) — 외란 추정치 d̂: 몸체 좌표 + 느린 필터 + 포화 중 갱신 멈춤 + 상한 (`torque_mpc.yaml disturbance`, 기본은 여전히 끔)
- 왜: 사용자 "d̂ 키면 너무 별론데 끄면 정상상태 오차가 안 잡힌다". 실차 8자 (analysis/torque_mpc_real/2026-11-21_square/ 14 절): 예전 d̂ (월드 α 0.1) 는
  느린 랩 치우침을 반으로 줄이지만 (+29/+35 → +14/+14 mm) 흔들림 (â_d 표준편차 0.18~0.51) 이 정상상태 몫 (0.09~0.26 m/s²) 의 2~5 배 → Δτ +47 %,
  랩 3 (구심 2.13 > C 상한 1.63 m/s²) 은 못 따라가는 몫을 배워 +44 → +62 mm. 월드 좌표에서 느리게 거르면 원 위의 일정한 외란이 ω 로 돌아 지워진다.
- 바꾼 것: `mpc_cpp` torque_mpc.hpp·cpp — `dist_body_frame` (잔차 x·y 를 R(θ̂)ᵀ 로 몸체 좌표에 저장, 지평 스텝 i 마다 R(θ_r,i) 로 돌려 예측·U_s 에),
  `dist_freeze_on_sat` (직전 τ ≥ 0.98 τ̄ 또는 U_s > τ̄ 면 갱신 멈춤, d_skipped 3), `dist_acc_max`·`dist_yaw_acc_max` (거른 뒤 |â_d|·|α_d| 상한), `d_hat_world()`.
  구조체 기본값은 예전 동작. torque_mpc_node: 파라미터 `disturbance_frame`·`disturbance_freeze_on_saturation`·`disturbance_acc_max`·`disturbance_yaw_acc_max`.
  torque_mpc.yaml disturbance: alpha 0.1 → 0.01, accel_only false → true, frame body, freeze_on_saturation true, acc_max 0.5, yaw_acc_max 2.0 (enable 은 false 그대로).
  launch `mpc_dist_frame`·`mpc_dist_freeze`·`mpc_dist_acc_max`·`mpc_dist_yaw_acc_max` (real_all `_PASS`). **status[14:20] 의 d̂ 는 frame body 면 몸체 좌표.**
- 확인: amr_mpc_cpp 재빌드, gtest 34 + 12 (새 3: 원 위 몸체 옆 힘 2 N → 반경 어긋남 끔 +54.4 · 월드 α0.01 +48.1 · 몸체 α0.01 +0.6 mm, 포화 중 멈춤, 상한), pytest 343.
  시뮬 8자 (analysis/torque_mpc_sim/2026-11-21_dhat_body/): Δτ 예전 +41 % → 새 끈 것과 같음, 랩 3 +45 → +28~+37 mm, 랩 1·2 는 판 사이 편차 안. 실차 판 없음.
- 예전 동작: `mpc_dist_frame:=world mpc_dist_alpha:=0.1 mpc_dist_accel_only:=false mpc_dist_freeze:=false mpc_dist_acc_max:=-1 mpc_dist_yaw_acc_max:=-1`.

### 2026-11-21 (10) — 실차 C 8자 외란 추정치 켬 (`disturbance:=true`, launch 인자만 — 설정값 안 바꿈)
- 왜: 사용자 "켜서 시험" ((9) 의 정상상태 어긋남). 205044 (circles8 width 1.2 · `lap_speeds:=[0.43,0.61,0.8]`, 나머지 203133 과 같음), 1 판.
- 결과 (RESULTS_KO.md 3·4 절, **주행 중만** — 3 절의 2.0 cm 는 완주 뒤 정지 시간에 묽혀진 값이라 정정): 경로 거리 RMS 4.5 → 4.0 cm (PP 200944 3.7), SLAM 기준 3.7 → 4.3 (PP 4.1), 헤딩 RMS 4.0 → 2.8° (PP 6.4) — 위치는 PP 와 비슷하거나 조금 나쁨
  [2026-11-23 정정: SLAM 값이 IMU 점 그대로였다. 로봇 중심 기준 4.7 → 4.3 (PP 3.0) — 위치는 PP 보다 나쁨. analysis/torque_mpc_real/2026-11-23_horizon_real],
  랩 1·2 원 안쪽 3.0·3.4 → 1.4·1.2 cm. 랩 3 은 기본 τ̄ 에 81 % 걸려 lag +7.9 · 안쪽 +5.1 cm 남음. 교수님 11-16 지시 (d̂ 끈 채로) 와 다르니 보고할 것.
- d̂ 켬 + μ̂ ×2 (205817) vs 바로 뒤 PP (205937), 주행 중만 (RESULTS 5 절): 경로 거리 RMS 3.5 vs 3.3 cm (SLAM 4.0 vs 4.1) 같음 [2026-11-23 정정: SLAM 중심 기준 3.7 vs 2.9 — PP 가 작음], 시간 기준 4.6 vs 5.5, 헤딩 2.8 vs 5.9°,
  랩 3 lag +1.4 vs +6.5 cm — 위치는 비김, 시간·헤딩은 C. 1 판씩.
- 3 판씩 반복 (C 205817·210248·210536 / PP 205937·210424·210648, 번갈아, RESULTS 6 절): 경로 거리 RMS C 3.5–4.1 vs PP 3.3–3.7 cm (겹침), 시간 기준 4.6–5.7 vs 5.5–6.4,
  헤딩 2.6–2.8 vs 5.9–6.7° · 랩 3 lag +1.2–2.4 vs +6.5–8.8 cm (안 겹침). 모두 완주.
- 사용자 "보기에는 A 가 훨씬 낫다" → RESULTS 7 절: 교차점 밖 경로 거리는 같고 (3.5–3.9 vs 3.5–3.8 cm), C 는 교차점 0.15 m 안 3.3–4.9 / 최대 8.9–14 cm (PP 2.6–2.7 / 6–6.9),
  요 흔들림 RMS 0.36–0.41 vs 0.18–0.20 rad/s. μ̂ ×2 와 d̂ 가 각각 흔들림을 키웠다.
- 속도 출처 엔코더 (213847, width 2.0 · 새 d̂ · τ̄ 기본) vs 추정기 (212502·213246), RESULTS 10 절: 경로 거리 (추정기) 2.4–2.7 → 2.0 cm (PP 212629 1.8), 시간 기준 3.2–3.4 → 2.6 (PP 2.9),
  교차점 2.4 → 1.7 (PP 2.0), SLAM 기준 경로 거리 그대로, 요 흔들림 그대로 (0.147 → 0.145, PP 0.089). 1 판.
- 같이 ((9) 정정): width 2.0 판 (204329·204536) 과 견주면 C 의 원 안쪽 치우침은 반지름이 아니라 속도에 따라간다 — "구심 1.4~1.6 배가 원인" 은 뒷받침 안 됨.

### 2026-11-21 (9) — 8자에서 PP 가 MPC 보다 나은 이유: 앞먹임 U_s 의 구심 1.4~1.6 배 · 요 저항 0 (분석만, 코드 변경 없음)
- 왜: 사용자 "pp 가 mpc 보다 좋게 나오는 이유 분석 + 해결 방안". `analysis/torque_mpc_real/2026-11-21_pp_vs_mpc/` (`decomp.py`·`ff.py`, RESULTS_KO.md).
- C 의 주된 오차는 원 안쪽 +3.0~3.4 cm (랩 1·2) 일정한 치우침. μ̂ ×2 (201947) 로도, Q 20·θ 10 (203133) 으로도 3.2 → 3.0 cm — 13 절 (8) 의 "Q 20 이면 1/4~1/6" 추정은 맞지 않았다.
- 원인: U_s 가 원 중심 방향 토크를 PP 가 쓴 양의 1.55 (0.43) · 1.41 (0.61) 배로 잡고, 정속 회전 요 토크 (PP 0.015 · 0.020 N·m/바퀴 ≈ 0.010 N·m/(rad/s)) 를 0 으로 본다.
  적분 동작이 없어 (disturbance 끔) 이 어긋남이 정상상태 오차로 남는다. PP + 바퀴 PI 는 힘 모델이 없어 영향 없음. 구심 1.5 배의 원인은 못 가름.

### 2026-11-21 (8) — 토크 MPC 비용함수 Q 기본값 diag(5, 5, 2.5, 2, 2, 1) → diag(20, 20, 10, 2, 2, 1) (`torque_mpc.yaml tuning.W`)
- 왜: 사용자 "여기서 확정" (C3). 정사각형 실차 contour 47~75 → 21 mm, lag −78~−89 → −34, yaw 5.9~6.7 → 3.4° (RESULTS_KO.md 10·12 절, C3 는 1 판).
- 바꾼 것: Q₁₁ = Q₂₂ (x·y 위치 오차) 5 → 20, Q₃₃ (요 오차) 2.5 → 10. Q₄₄~Q₆₆ · R_Δ (w_dudt 0.01) · R_U (r_u 10) 그대로. 설치본이 소스에 링크 — 다음 `amrall` 부터.
  예전 값: `mpc_W:="5,5,2.5,2,2,1"`. 확인: pytest 340 통과. 8자 실차 판은 아직 없다.
- 같이 (RESULTS_KO.md 13 절): 오늘 8자 C 판 (201047, 예전 Q) 은 옆 오차가 원 안쪽으로 +34~+53 mm 일정 — 정상상태 어긋남. PP 200944 e_c RMS 28 vs C 62 mm.

### 2026-11-21 (7) — 실차 C3: Q₃₃ (요 오차 가중) 2.5 → 10 시험 (launch 인자만, 설정값 안 바꿈)
- 왜: 사용자 "다음 가중치 튜닝". C2 (Q₁₁ = Q₂₂ = 20) 의 남은 요 오차가 직선 변 1~3 에서 −6~−8.5° 로 일정 (꼭짓점에서 덜 돈 몫이 안 돌아옴),
  무제약 θ 가장 느린 pole −0.98 → −2.70 (`analysis/torque_mpc_real/2026-11-21_square/q_grid.py`, RESULTS_KO.md 11 절).
- 실차 201411 (`mpc_W:="20,20,10,2,2,1"`, 1 판): contour 31~37 → 21 mm, lag −54 → −34, yaw RMS 5.7 → 3.4°, 직선 요 어긋남 −6~−8.5 → −2.3~−3.7°,
  큰 슬립 14 % 그대로, Δτ +10 %, 포화 57 % 그대로. 측위는 C2 판들보다 나빴다 (SLAM 튐 16 vs 5~6). RESULTS_KO.md 12 절. 반복 판 필요.

### 2026-11-21 (6) — 측위 맞추기 판정 기준 70 → 60 % (`tools/check_localization.py --min` 기본값)
- 왜: 사용자 "검사 기준 60 % 로 바꿔줘". 같은 자세 (−0.59, +1.49 m, −75°) 에서 11 번 중 10 번 실패 — 탐색 자세 겹침은 79~84 % 인데 `/initialpose` 뒤 Cartographer 가
  0.3~0.6° 더 돌아간 자리에 앉아 65~69 %. 큰 어긋남 (1.3 m · 64° → 21 %) 을 잡는 검사의 목적에 비해 빡빡했다.
  근거 `analysis/torque_mpc_real/2026-11-21_square/RESULTS_KO.md` 9 절 (`loc_probe.py`·`loc_only.sh` 로 무장 없이 측위만 재는 도구 추가).
- 바꾼 파일: `controller/mpc/tools/check_localization.py` (기본값·머리주석). real_all 은 인자 없이 `--apply` 로 부르므로 다음 `amrall` 부터 60 %.
- 같이 겪음: 반복 재기동 뒤 D455 `xioctl(VIDIOC_S_FMT) failed, errno=5` → 카메라 노드만 `initial_reset:=true` 로 띄워 리셋 (`depth_camera_initial_reset` 는 real_all 이 안 넘긴다).

### 2026-11-21 (5) — 실차 정사각형 조건별 반복 (Claude 원격 5 판, 분석만 — 코드 변경 없음)
- 왜: 사용자 "알아서 싹다 돌려서 비교". `analysis/torque_mpc_real/2026-11-21_square/run_all.sh`·`run_square.sh` (측위 "맞음" 아니면 무장 안 함, 끝난 뒤 출발점 0.4 m 넘게 벗어나면 멈춤).
  사용자 확인 "로봇 옆 사람·공간 준비됨" 뒤 실행. 계획 8 판 중 5 판 (P1a·C1a·P2a·C2a·P1b), C1b 는 측위 겹침 65~68 % (기준 70 %) 3 번 실패로 멈춤.
- 결과 (`compare_all.py`, RESULTS_KO.md 7·8 절, 앞선 184008·183827·190553 포함): contour RMS P1 (PP L 0.30) 58~109 mm, **P2 (PP L 0.15~0.60) 27** (1 판), C1 47~75, **C2 (Q₁₁ = Q₂₂ 5 → 20) 31·37**.
  P2 vs C2: yaw 21.1 vs 5.4~6.0°, 큰 슬립 32.3 vs 13.8~14.5 %, Δτ 0.036 vs 0.0095~0.0099, τ max 0.45 vs 0.18. 룩어헤드 상수 0.30 (2026-11-20 (9)) 은 실차에서 나빴다 — 설정값은 안 바꿈.
- 측위: SLAM 튐 104 개 중 71 % 가 |ω| > 0.5 rad/s (시간 몫 49 %). Cartographer 입력은 아직 LaserScan (`/scan_*_carto`). 출발 측위 맞추기는 70 % 경계에서 갈린다.
- 최종 (같은 날 19:42·19:56 P2b·C2b 추가, 모두 pass 꼭짓점): contour 평균 P1 77 · P2 41 (27·55) · C1 61 · **C2 33 (31·37·31)**. C2 vs P2: yaw 5.7 vs 20.5°,
  큰 슬립 14 vs 35 %, Δτ 0.0098 vs 0.036, τ max 0.18 vs 0.41~0.45. RESULTS_KO.md 10 절.

### 2026-11-21 (4) — PP 룩어헤드 launch 인자 (`pp_lookahead_min`·`pp_lookahead_max`) + C 가 경로에 안 붙는 이유 분석
- 왜: 사용자 "왜 경로에 붙지 않고 MPC 가 주행하는지" + 다음 실차 판에서 PP 룩어헤드 A/B (184008 contour 109 mm 가 11-20 의 39 mm 보다 나쁨 — 룩어헤드와 측위 변경이 같이 바뀜).
- 바꾼 파일: `mpc/launch/torque_experiment.launch.py` (pure_pursuit_node 파라미터 `lookahead_min`·`lookahead_max` 덮어쓰기, 비우면 torque_baseline_a.yaml),
  `mpc/launch/real_all.launch.py` (`_PASS`), `mpc/test/test_launch_modes.py` (덮어쓰기 2 + real_all 전달 1). 확인: pytest 340 통과. 실차·시뮬 기동 확인 전 —
  기동 로그 "룩어헤드 L_d = clamp(0.5 s × |v_ref|, 0.15, 0.6) m" 로 본다. 예전 속도 비례: `pp_lookahead_min:=0.15 pp_lookahead_max:=0.60`.
- 분석 (`analysis/torque_mpc_real/2026-11-21_square/` 4절, `decomp.py`·`pole.py`): C 183827 첫 변 옆 오차 +10 → +100 mm 가 0.1 s 에 생김 = SLAM 포즈 튐
  (68·95 mm, 상한으로 0.1 s 에 갈 수 있는 옆 거리 ≤ 9.2 mm). 되돌림은 x·y pole −0.89 (무제약, 시정수 1.12 s; 실차 회귀 −0.58 ± 1.01j) 로 느리고,
  되돌리는 동안 FL·RR 포화로 진행 feedback −0.10~−0.15 m/s² → lag. r_U 를 낮추면 pole 이 오히려 느려진다 (−0.50 @ 0.1). Q₁₁ = Q₂₂ = 20 이면 −1.50.
  `pole.py` 는 torque_mpc.yaml 2026-11-18 (4) 주석의 pole 4 개를 다시 낸다 (5.20·8.87·13.18 Hz, −1.03).
- 실차 C2 (190553, `mpc_W:="20,20,2.5,2,2,1"` = Q₁₁ = Q₂₂ 5 → 20, 설정값 안 바꿈): contour 75 → 31 mm, lag 평균 −89 → −54, yaw 6.7 → 5.4°, 옆 회귀 k_p 1.35 → 5.88 (pole −0.81 ± 2.28j).
  단 SLAM 튐 17 번·95 mm → 6 번·42 mm 로 측위도 좋았던 판이라 Q 몫은 못 가름. 포화 48 → 57 %, FL 큰 슬립 9 → 18 %. 같은 세션 C1·C2 반복 필요.

### 2026-11-21 (3) — 실차 `amrall` 에서 랩별 속도 사다리 `lap_speeds` 쓰기 (real_all `_PASS` 에 추가)
- 왜: 사용자 "8자 3 바퀴, 랩마다 속도 증가로 A·C 비교". 플래너는 이미 있다 (`core/figure8.py` `lap_speeds`, 2026-11-11) — 실차 launch 만 안 넘기고 있었다.
- 바꾼 것: `launch/real_all.launch.py` `_PASS` 에 `lap_speeds`, `test_launch_modes.py` 검사 1 줄. 쓰는 법 `lap_speeds:=[0.4,0.55,0.7]` (laps = 목록 길이, speed 는 안 씀).
- 시뮬 (circles8 width 1.2 · tangent · accel 1.0 · 바닥 μ 0.26, C 는 바퀴별 μ̂ 기본값, 3 회차씩): 위치 RMS / 최대 —
  A 52.5~77.8 / 210~332 mm, C 39.9~41.8 / 88~111 mm. 헤딩 RMS A 5.9~7.5°, C 2.6~2.9°. 시뮬 λ RMS 는 C 가 더 크다 (랩별 0.22~0.28 vs A 0.16~0.25, 1 회차). 실차 회차 없음.

### 2026-11-21 (2) — C lag 원인 정정 (분석만, 코드 변경 없음)
- 왜: 사용자 질문 "뒤처짐은 무슨 문제야". 2026-11-20 (12)·2026-11-21 (1) 의 "C 가 못 따라가는 주원인 = 꼭짓점에서 기준이 마찰 상한을 넘는 것" 을
  실차 bag 4 판 (171531·171735·190506·190642, 모두 acc_window 고치기 전) 의 arc length lag 로 다시 쟀다.
- 결과 (`analysis/torque_mpc_real/2026-11-20_why_c/lag_arclen.py`, RESULTS_KO.md 3절): lag 는 직선 (가속·감속·그 밖) 에서 쌓이고 꼭짓점에서 풀린다
  (회전 구간 진입 −84~−270 → 이탈 0 근처). 앞에 꼭짓점이 없는 변 0 에서도 −127·−138 mm (새 μ̂). 추정기와 SLAM lag 평균이 1~2 mm 안에서 같다.
  → 위 "주원인" 은 실차 lag 로는 맞지 않다. 꼭짓점 U_s 초과 (54 %) 는 사실이고 heading·contour 쪽 문제.
- 직선 원인 후보 (분리 안 됨): 새 μ̂ RR 여유 τ̄ − U_s = 0.1055 − 0.1018 = 0.0037 N·m 보다 이 판들의 feedforward 부족 0.0254 (고친 뒤 0.0081) 가 크다,
  모델 밖 손실 (2026-11-19 cmd_chain 기울기 0.96~0.995 등). 시뮬 (고친 뒤) 은 stop 에서 lag −11~−15 로 작아 실차와 어긋난다 — 고친 뒤 실차 판으로 가른다.
- 같이: `2026-11-20_square` 표의 e_t (시각 t 접선 투영) 는 꼭짓점에서 앞 변 lag 가 cross-track 으로 넘어가 작게 나온다 (190506 −76 vs arc length −101 mm).
- 실차 확인 (같은 날 18:38, `analysis/torque_mpc_real/2026-11-21_square/`, C 183827 = c_s 0.9 상당 + acc_window 고친 뒤): ρ 0.76 → 0.99 로 고쳐졌지만
  lag 평균 −93 → −89 mm, 변 최고 속도 1.22~1.25 m/s 로 그대로 → 후보 (a)·(b) 는 주원인 아님. 남은 몫: 포화 때 토크가 앞 방향 밖으로 (a_τ 1.35~1.40 < a_ff 1.50),
  실제 가속 < 모델 가속 (변 0: 1.30 vs 1.45). 같은 세션 PP 184008 대비 C 가 contour 75 vs 109 mm, yaw 6.7 vs 21.2°, 큰 슬립 (v_ref>0.3) 12.8 vs 47.6 %.

### 2026-11-21 (1) — 정사각형 꼭짓점 정지·제자리 회전 옵션 (`square_corner_mode:=pass|stop`, 기본 pass)
- 왜: accel 1.5 · v_max 1.5 정사각형 (tangent, yaw_blend 0.3) 은 꼭짓점 헤딩 회전 토크가 직선 가감속 토크 (RR C 상한의 96 %) 위에 겹쳐
  노드가 보는 앞먹임 U_s 가 C 상한의 2.55 배, 꼭짓점 구간 내내 초과 — C 가 못 따라가는 주원인 (2026-11-20 (12)).
  [정정 2026-11-21 (2): 실차 lag 는 직선에서 쌓인다 — "주원인" 은 실차 bag 으로 뒷받침되지 않는다. U_s 초과 자체는 사실.]
  yaw_blend 를 넓혀도 최대 1.99 · 초과 시간은 오히려 66 % 로 는다 (`analysis/torque_mpc_sim/2026-11-20_square_profile/`).
- 바꾼 것: `core/square.py` `corner_mode` (pass | stop), `rot_s` (1.0), `dwell_s` (0.1). stop = 꼭짓점마다 선다 → dwell → (tangent) 제자리 +90°
  rot_s (스무스스텝) → dwell → 다음 변. 몸체 v_y 가 어디서나 0 이라 퓨어 퍼슛도 원리상 낼 수 있다 (PP 는 v_ref = 0 에서 회전 모드).
  `figure8_reference.py` 파라미터 `square_corner_mode`·`square_rot_s`·`square_dwell_s` + 기동 로그, `torque_track.yaml`, `torque_experiment.launch.py`
  (`_TRACK_KEYS`, 허용값 검사), `real_all.launch.py` (`_PASS`). 플래너는 그대로 하나 — PP·MPC 가 같은 기준열을 받는다.
- 기본값 근거 (`2026-11-20_square_profile/sweep_stop.py`, 노드 0.2 s 차분 창, 창 위치 0~0.15 s 앞당김 중 최악): rot_s 0.6 / 0.8 / 1.0 / 1.2 →
  최대 |U_s|/τ̄ 1.22 / 0.96 / 0.96 / 0.96 (0.96 = 직선 가감속 그 자체), 순수 회전만 1.22 / 0.77 / 0.53 / 0.38. I_z 0.06 이 추정값이라 1.0 (여유).
  dwell 0 도 넘지 않았지만 0.1 s 를 둔다 (PP·MPC 가 선 위치를 잡을 틈). 주행 7.6 s → 11.6 s.
- 확인: `python3 amr_mpc/core/square.py` (stop 속도 vs 위치 유한차분 2.6e-8), pytest 337 통과 (test_square 꼭짓점 정지 8, test_launch_modes 3 추가).
- 시뮬 (`analysis/torque_mpc_sim/2026-11-21_square_stop/`, accel 1.5 · v_max 1.5 · tangent · 바닥 μ 0.45, 2 판씩) pass → stop:
  C 시간 추종 RMS 103~110 → 51~54 mm, 윤곽 RMS 53 → 20~21, 포화 49 → 7~8 %, 앞먹임 초과 32 → 0 % (B 와 같아짐).
  B 53~58 → 53~54 mm, 윤곽 21~27 → 15~21. A (PP) 시간 추종 48~49 → 54~59, 윤곽 RMS 27 → 18~26, 헤딩 RMS 13 → 25° (최대 85°) —
  감속 중 룩어헤드 0.30 m 가 다음 변을 겨눠 꼭짓점 전에 돈다. 주행 9.1 → 13.1 s. 실차 판 없음.
  슬립 (참값 λ, |λ| > 0.5 시간): A 18 → 13 % (전부 꼭짓점 0.3 m 안, 92~98 % 끌림 λ < 0), B 0.4~1.0 → 0, C 0 → 0.
  사용자 판단 (2026-11-21): 기본은 pass 유지 — stop 은 PP 도 잘 가 비교가 약해진다.

### 2026-11-20 (12) — 토크 MPC 기준 가속도 차분 창을 기준열 첫 행 뒤로 밀기 (`acc_window_shift`, 기본 켬)
- 왜: 실차 C 판 분석 중 앞먹임 U_s 가 직선 등가속에서도 10 Hz 톱니 (필요량의 a/2 ~ a). `torque_mpc_node` 는 p̈_r 을 기준 속도의
  [t − 0.1, t + 0.1] 중앙 차분으로 내는데, 기준열 첫 행이 "지금"(header.stamp + dt, figure8 2026-11-11 결정)이라 t − 0.1 이 늘 첫 행 앞 →
  첫 행 속도로 상수 외삽 → p̈ = a·(h + ref_age − dt)/(2h). 실차 bag 190506·171531 직선 가속 213 표본씩: 측정 ρ 0.76·0.74,
  이 식 0.77·0.75 (평균 차 0.02, ref_age 와 상관 0.83·0.85). `analysis/torque_mpc_real/2026-11-20_why_c/verify_ff.py`.
- 바꾼 파일: `mpc_cpp/include/amr_mpc_cpp/reference.hpp` (`acc_window_start(t, h, t_first) = max(t − h, t_first)`),
  `mpc_cpp/src/torque_mpc_node.cpp` (창 [t_lo, t_lo + acc_dt], 파라미터 `acc_window_shift`), `mpc_cpp/test/test_wheel_mpc.cpp`
  (`Reference.AccelWindowShiftedPastFirstRow`: 등가속 기준열에서 민 창 = a, 예전 창 = a(h + t − t_first)/(2h)),
  `mpc/config/torque_mpc.yaml` (`tuning.acc_window_shift: true`), `mpc/launch/torque_experiment.launch.py` (`mpc_acc_window_shift:=`).
- 확인: amr_mpc_cpp 재빌드, gtest 12 + 31, pytest 326 통과. 시뮬 (정사각형 가속 1.5 · 1.5 m/s, 바닥 μ 0.45, 2 판씩,
  `analysis/torque_mpc_sim/2026-11-20_acc_window/`): 직선 가속 ρ 0.70~0.73 → 0.92~0.93 (고쳐짐).
  **추종은 거의 그대로** — C 뒤처짐 −58~−60 → −54~−58 mm, 윤곽 56~57 → 57~59. B 뒤처짐 −6~−9 → −1~−5, 윤곽 24~25 → 26~29, 헛돎 0~0.3 → 0.9~1.8 %.
  B 는 피드백이 모자란 앞먹임을 메워 왔고, C 는 꼭짓점에서 앞먹임 자체가 상한을 넘어 (실차 꼭짓점 구간 54 %) 어차피 상한에 잘린다 →
  C 가 못 따라가는 주원인은 이 버그가 아니라 기준이 마찰 상한을 넘는 것. 실차 판 없음. 예전 동작: `mpc_acc_window_shift:=false`.
  [정정 2026-11-21 (2): 실차 4 판의 lag 는 직선에서 쌓이고 꼭짓점에서 풀린다 (변 0 도 −127·−138 mm). 이 버그 (직선 feedforward 0.0254 N·m 부족 >
  RR 여유 0.0037) 가 실차 직선 lag 의 후보로 남는다 — 고친 뒤 실차 판 없음. `analysis/torque_mpc_real/2026-11-20_why_c/RESULTS_KO.md` 3절.]

### 2026-11-20 (11) — 바퀴별 μ̂ FR 0.285 → 0.44, RR 0.350 → 0.417 (`torque_plant.yaml surface.mu_hat_wheels`)
- 왜: 사용자 "바꿔". C 가 FR 상한(τ̄ 0.072)에 묶여 정사각형 가속 1.5 를 못 따라 13 cm 뒤처졌다.
- 근거 (`analysis/torque_mpc_real/2026-11-20_fr_mu/RESULTS_KO.md`): 실차 토크 램프 2 판 (fwd 175004 = FR 1사분면, right 180622 = 3사분면,
  right 판은 측위 겹침 64 % 인 채로 — 사용자 결정) 모두 FR 이 먼저 헛돎, 그 순간 접지 토크 명령 τ_g 0.139 / 0.204 N·m → mg/4 기준 0.44 / 0.64 (작은 쪽).
  RR 은 정사각형 bag 4 판의 미끄럼 없는 접지 토크 하한 0.417. FL·RL 그대로 (bag 하한 0.318·0.478 과 모순 없음).
- 결과: C τ̄ [0.1093 0.0721 0.1154 0.0886] → [0.1093 0.1113 0.1154 0.1055] N·m, 앞 방향 최대 가속 (LP) 1.18 → 1.58 m/s².
- 확인: 시뮬 기동 로그 "바퀴별 μ̂ [0.432 0.440 0.456 0.417] … 쓰는 τ̄ [0.1093 0.1113 0.1154 0.1055]". pytest 326 통과. 실차 C 판 없음.
- 같이: `wheel_slip_onset.py` 의 피드백 τ_c 는 헛돎 시작 순간 값이라 (표 제목 "30 ms 전" 과 다름) J_w·ω̇ 로 크게 깎인다 — 이번에는 τ_g 기준을 썼다.
  `analysis/torque_mpc_real/2026-11-20_ramp/run_one.sh` (원격 램프 한 판) — 띄운 상태에서 측위를 반복 적용하면 Cartographer 가
  "Non-sorted data added to queue" 로 죽어서 그 루프는 뺐다.

### 2026-11-20 (10) — 토크 램프에 back·right 방향 + 실차 램프 1 판 (FR 재측정 시작, 사용자 지시로 1 판에서 멈춤)
- 왜: FR μ̂ 재측정. fwd↔back, left↔right 를 번갈아 돌면 판마다 되돌려 놓지 않아도 되고 바퀴마다 두 사분면을 잰다.
- 바꾼 파일: `mpc/amr_mpc/nodes/torque_ramp.py` (DIRS back [−1,0,0]·right [0,−1,0]), `mpc/launch/torque_ramp.launch.py` (허용값·사용법),
  `mpc/launch/torque_experiment.launch.py` (ramp_dir 설명), `analysis/torque_mpc_real/2026-11-17_ramp/wheel_slip_onset.py` (back·right 패턴).
- 확인: 시뮬 back 한 판 3.45 m 뒤로 (right 는 앞 판과 겹쳐 판정 못 함). pytest 317 통과.
- 실차 (Claude 원격, 사용자 "로봇 옆 사람 있음" 확인 뒤): `analysis/torque_mpc_real/2026-11-20_ramp/run_one.sh fwd 2.0` 1 판 —
  2.06 m, τ_g 1.0 N·m 도달. wheel_slip_onset: **FR 이 먼저** 헛돎 (t 0.22 s), 접지 τ_c [0.095 0.042 0.104 0.126] → μ [0.30 0.13 0.33 0.40].
  1 판뿐이라 값으로 쓰지 않는다. 사용자 "이제 그만" 으로 나머지 판은 안 돌렸다.

### 2026-11-20 (9) — 퓨어 퍼슛 룩어헤드 상수 0.30 m (`torque_baseline_a.yaml` lookahead_min = max = 0.30)
- 왜: 사용자 "PP 룩어헤드 상수로". 속도 비례(0.15~0.60)는 정사각형 꼭짓점(기준 0.1 m/s)에서 0.15 로 줄며 교점이 넘어가 κ 가 뒤집혔다.
- 시뮬 (정사각형 1.5 m, 최고 1.5 m/s, 바닥 μ 0.45, 참값, `analysis/torque_mpc_sim/2026-11-20_pp_lookahead_c_lag/`):
  가속 1.5 에서 비례 → 상수 0.3: 윤곽 RMS 50~63 → 32 mm, τ max 0.50~0.72 → 0.43, Δτ RMS 0.022 → 0.014, 꼭짓점 머뭇 0.73 → 0.53 s.
  0.2 는 윤곽은 작지만 토크 튐 (λ max 1.9), 0.4 는 윤곽 48 mm. 가속 1.0 에서 0.3: 윤곽 22 mm (비례 44), λ max 0.52 (1.80).
- 바꾼 파일: `mpc/config/torque_baseline_a.yaml` (값 + 근거 표). 확인: 설치본 적재 L_d 0.3 (v_ref 0.1·0.8·1.5 모두). pytest 317 통과. 실차 판 없음.
- 같은 날 C 뒤처짐 원인 분리 (코드 변경 없음, 같은 폴더): 측정 μ̂ 의 FR 0.285 × c_s 0.8 이 앞 방향 최대 가속을 1.18 m/s² 로 묶는다
  (힘 분배 LP) → 기준 1.5 를 못 따라 뒤처짐 −130 mm. μ̂·c_s = 바닥이면 윤곽 84 → 22 mm · 뒤처짐 −37, 여기에 r_u 1 이면 −6 (윤곽 38).
  기준 가속 1.0 만 낮추면 −46. 실차 bag (171640·171531·171735): 퓨어 퍼슛은 1.51~1.63 m/s 를 내지만 최고점이 기준보다 0.18~0.64 m 뒤,
  C 는 위치·시각은 맞지만 1.04~1.16 m/s (72~81 %).

### 2026-11-20 (8) — 측위 맞추기 종료 배너 (real_all) + check_localization 종료 코드
- 왜: 실차 17:12 판에서 측위 맞추기(--apply)가 끝나기 전에 무장 → 보정으로 SLAM 포즈 158 cm·−67° 튐 → fault_monitor pose_jump 정지,
  다시 무장하자 출발점이 보정 전 좌표라 figure8 abort (163.8 cm). 사용자 "측위 맞추기 끝나는 게 다른 로그에 묻혀 안 보인다".
- 바꾼 파일:
  - `mpc/tools/check_localization.py`: 종료 코드 0 맞음 · 1 측위 결과 없음 · 2 찾았지만 안 넣음 · 3 지도 어디에도 안 맞음 · 4 넣었는데 아직 안 맞음
    (예전에는 결과와 상관없이 0, 결과 없음만 1).
  - `mpc/launch/real_all.launch.py`: check_localization 종료 시 OnProcessExit → 72 칸 배너 (0 이면 초록 "이제 a", 그 밖 빨강 "무장하지 말 것" + 이유).
    시작 줄에도 "끝나면 큰 배너, 그 전에 a 누르지 말 것".
- 확인: launch API 로 가짜 프로세스(exit 0 / exit 3)에 같은 핸들러 → 두 배너 출력 확인. pytest 317 통과. 실차 판 없음 (다음 amrall 부터).
- 같이 본 것 (171406 PP 판 bag): 퓨어 퍼슛은 정사각형 한 바퀴 완주 (출발점 9 mm 로 hold). 꼭짓점마다 0.3~0.5 s 머뭇거림 —
  v 명령 0.00~0.08 m/s, κ −6 ~ −11 (룩어헤드 교점이 지금 변으로 되돌아가고, 기준점이 로봇 뒤 → v = v_ref + k_along·e_along ≈ 0, v ≥ 0 클램프).
  CAN 응답 빠짐: 기동 직후 "응답 1/4 — 빠진 바퀴 [1, 2, 3]" 1 회, 주행 중 [3]·[2, 3].

### 2026-11-20 (7) — w 항 기본 켬 (`torque_mpc.yaml model_terms.use_w: true`)
- 왜: 사용자 "켜" — (5)(6) 의 R 튜닝을 use_w 켬으로 했다. ※ 교수님 11-16 지시("구현하되 끈 채로 제어")와 다르다 → 보고 필요.
- 바꾼 파일: `mpc/config/torque_mpc.yaml` (값 + 주석), `mpc/launch/torque_experiment.launch.py` (주석·인자 설명). 끌 때: `use_w:=false`.
- 확인: 인자 없이 시뮬 기동 → torque_mpc_node "w 항 켬", w_dudt 0.01, r_U 10. pytest 317 통과. 실차 판 없음.

### 2026-11-20 (6) — MPC R 기본값 w_dudt 0.001 → 0.01, r_u 0.1 → 10 (`torque_mpc.yaml`, Q 그대로)
- 왜: 사용자 "넣어" — (5) 의 R 튜닝 결과 (정사각형 1.5 m/s · 가속 1.5 · μ 0.4 · use_w 시뮬: B 윤곽 22~26 mm · 헛돎 2~6 %,
  C μ̂ 0.4 윤곽 72~96 mm (예전 128~137) · 헛돎 0 %, Δτ RMS 0.005~0.006 = 11-16 실차 떨림 없음 수준의 절반).
- 바꾼 파일: `mpc/config/torque_mpc.yaml` tuning.w_dudt·r_u + 근거 표 주석. 되돌리기: launch 인자 `mpc_w_dudt:=0.001 mpc_r_u:=0.1`.
- 확인: 기본 인자만으로 시뮬 기동 → torque_mpc_node 로그 "w_dudt 0.010000 … r_U 10", W [5, 5, 2.5, 2, 2, 1]. pytest 317 통과
  (C++ gtest 는 자기 튜닝값을 써서 yaml 과 무관). 빌드 불필요 (설정 링크). 실차 판 없음.
- 주의: 튜닝은 use_w 켬으로 했다 — 같은 날 (7) 에서 기본 켬으로.

### 2026-11-20 (5) — MPC R 튜닝 시뮬 (Q 고정, use_w, 정사각형 1.5 m/s) + 마찰 초과 비교 (코드 변경 없음)
- 왜: 사용자 "Q 는 그대로 두고 R(QR 비율)로 헤딩 있을 때 최적 가중치", "MPC 는 잘 따라가고 PP 는 미끄러지는 조합", "사각형으로, 1.5 m/s".
  구조는 퓨어 퍼슛 = 엔코더, MPC = 추정기 (지금 기본값 그대로).
- 결과 (`analysis/torque_mpc_sim/2026-11-20_R_tuning/RESULTS_KO.md`, `2026-11-20_square_friction/`):
  - Q·R 같은 배율에 해 불변 (유계 QP 검산 3.6e-16) → R 만 훑어도 Q:R 비율은 다 본다 (Q 안의 비율은 아님).
  - circles8 W2.0 μ 0.4: 1.1 m/s 도 필요 μ 0.349 < 0.4 라 퓨어 퍼슛은 안 미끄러짐 (정상), MPC 가 스스로 미끄러짐 (B 5~47 %).
  - 정사각형 가속 2.5: 셋 다 경로 붕괴. 가속 1.5 · 최고 1.5 m/s · μ 0.4 에서 R 스윕 40 판 → R 을 키울수록 B 헛돎·C 코너 밀림이 준다.
  - 최종 (2~3 회): 퓨어 퍼슛 윤곽 71~74 mm · 헛돎(|λ|>0.5) 30~33 % · λ max 1.4, MPC B (w_dudt 0.01, r_u 10) 22~26 mm · 2~6 % · 0.7~1.0,
    MPC C μ̂ 0.5 (μ̂·c_s = 바닥) 39~49 mm · 0 % · 0.45, C μ̂ 0.4 72~96 mm · 0 %.
  - 퓨어 퍼슛 꼭짓점 토크 튐 원인 = 룩어헤드 교점이 다음 변 ↔ 지금 변으로 한 틱에 넘어감 (κ +1.2 → −12.3), 고치지 않음.
- 후보 가중치 w_dudt 0.01 · r_u 10 은 **torque_mpc.yaml 에 안 넣었다** (사용자 결정 대기). 실차 판 없음.
- 시뮬 병렬 실행: sim_plant_torque 가 numpy BLAS 스레드로 판마다 3.4 코어 → 실행 스크립트에서 OMP/OPENBLAS 스레드 1 (26 %).
  `tools/run_compare.sh` 에는 아직 안 넣었다.

### 2026-11-20 (4) — 정사각형 기준 `shape:=square` (변 가운데 최고속도, 꼭짓점 0.1 m/s) + 시뮬 비교
- 왜: 사용자 "정사각형 경로 중앙에서 낼 수 있는 최고속도, 꼭짓점에서는 0.1 m/s". 시나리오 문서의 "PP 는 코너커팅, MPC 는 따라감" 확인용.
- 무엇: `amr_mpc/core/square.py` (새) — 출발 포즈가 꼭짓점, 첫 변 = 출발 헤딩, 왼쪽으로. 변마다 accel 로 가속·감속 →
  가운데 최고 √(v_c² + a·side) (대칭 변), 첫 변 0 출발·마지막 변 0 도착. square_v_max > 0 이면 사다리꼴로 자름, 0 이면 노드가
  load_budget·V_LIMIT 로 자름. 꼭짓점은 속도 크기 v_c 로 지나며 방향만 90° (속도 계단 √2 v_c → 노드 중앙 차분 p̈_r 0.71 m/s² × 0.2 s).
  yaw_mode tangent | fixed (spin 거부). tangent 는 꼭짓점 ±square_yaw_blend/2 에서 헤딩 스무스스텝 (위치는 그대로 정사각형),
  마지막에 서서 align_s 동안 출발 헤딩으로. Figure8·Segment 와 같은 꼴 (at, peak_load, path_xy, T_total, V, A).
- 바꾼 파일: `mpc/amr_mpc/core/square.py` (새), `mpc/amr_mpc/nodes/figure8_reference.py` (파라미터 square_side 1.5 ·
  square_corner_speed 0.1 · square_v_max 0 · square_yaw_blend 0.3, shape 분기, 부하 거부 문구, 기동 로그 변별 속도),
  `mpc/config/torque_track.yaml` (square_* 4 개 + 설명), `mpc/launch/torque_experiment.launch.py` (_TRACK_KEYS, shape 허용값 square,
  square + spin 은 launch 에서 거부), `mpc/launch/real_all.launch.py` (_PASS), 시험 `mpc/test/test_square.py` (새, 21 개),
  `mpc/test/test_launch_modes.py` (3 개).
- 확인: `python3 -m amr_mpc.core.square` 속도 vs 위치 유한차분 ≤ 7e-7, 가운데 최고 = √(v_c² + aL) (0.781 / 1.229), 끝 포즈 오차 0.
  pytest 317 통과. 시뮬 (도메인 93, 추정기 상태, 슬립 없음) — `analysis/torque_mpc_sim/2026-11-20_square/RESULTS_KO.md`:
  | 판 (tangent) | 꼭짓점 통과 [mm] | 윤곽 RMS [mm] | 추종 RMS [mm] |
  | MPC B a 0.4 / 1.0 | 4~23 / 8~43 | 14.0 / 17.6 | 17.9 / 22.2 |
  | MPC C a 0.4 / 1.0 | 10~24 / 1~18 (바깥으로 밀림) | 12.7 / 39.5 | 17.1 / 52.9 |
  | 퓨어 퍼슛 a 0.4 / 1.0 | 72~79 / 152~162 | 18.2 / 47.8 | 29.9 / 91.9 |
  참값 상태: MPC B a 1.0 10~12 mm (추종 8.4), 퓨어 퍼슛 158~161 (구조적). 퓨어 퍼슛 w_max 3.0 판도 같음 (155~168).
  퓨어 퍼슛 토크가 꼭짓점 0.2 s 앞에서 한 틱 0.5 N·m 로 튐 (원인 미확인, w_max 와 무관). 실차 판 없음.

### 2026-11-20 (3) — 퓨어 퍼슛 `w_max` 3.0 → 8.7 rad/s (요각속도 소프트웨어 상한 사실상 없앰)
- 왜: 사용자 "요각속도 제한도 없애". 3.0 은 gerono 3×1.5 에서 0.94 m/s 위 ω 를 잘랐다 ((2) 의 남은 상한).
- 무엇: 8.7 = 제자리 회전에서 바퀴가 실측 한계 42 rad/s 에 닿는 ω (42 × 0.04 / 0.1932 = 8.70, 적재값으로 42.02 rad/s 확인).
  주행 중에는 바퀴 부하로 모터가 먼저 막는다 — 모터 한계 속도에서 ω: circles8 W2.0 1.21 m/s → 2.42, gerono 1.04 m/s → 3.32 rad/s.
  무한대가 아니라 8.7 인 이유: 그 위 명령은 못 내고, 바퀴 PI 기준만 42 rad/s 위로 올라가 적분이 쌓인다.
  참고: 곡률 κ = 2y/L² 는 L_min 0.15 m 에서 최대 13.3 1/m → 큰 측면 오차·포즈 잡음 때 ω 명령이 예전보다 크게 나갈 수 있다 (바퀴 PI 토크 상한 τ̄ 은 그대로).
- 바꾼 파일: `mpc/config/torque_baseline_a.yaml` (값 + 주석). 코드 기본값(`baseline/pure_pursuit.py` 3.0)은 YAML 이 없을 때만.
- 확인: 노드와 같은 방식 적재 v_max 1.7 · w_max 8.7. pytest 293 통과. 실차·시뮬 판 없음 (빌드 불필요).

### 2026-11-20 (2) — 퓨어 퍼슛 `v_max` 1.0 → 1.7 m/s (`torque_baseline_a.yaml`)
- 왜: 사용자 결정 (1) 과 같이. 1.0 이면 circles8 W2.0 1.0 m/s 이상·line 에서 퓨어 퍼슛이 기준 + k_along 보정을 못 낸다.
  1.7 ≈ 실측 바퀴속도 한계 42 rad/s × r 0.04 = 1.68 m/s — 소프트웨어 상한이 모터 한계보다 먼저 걸리지 않게.
- 바꾼 파일: `mpc/config/torque_baseline_a.yaml` (값 + 주석). 코드 기본값(`baseline/pure_pursuit.py` 1.0)은 YAML 이 없을 때만 쓰여 그대로.
- 남는 퓨어 퍼슛 상한: `w_max` 3.0 rad/s (그대로) — ω = v/R 이 닿는 속도 circles8 W2.0 1.50 · W3.0 2.25 (부하 예산 상한 1.11·1.23 보다 위),
  gerono 3×1.5 0.94 m/s (예산 상한 0.96 보다 아래 → 이 트랙 0.94~0.96 에서는 ω 가 잘린다).
- 확인: 설치본 경로(share → build 링크 → src)로 노드와 같은 방식 적재 v_max 1.7. pytest 293 통과. 실차·시뮬 판 없음 (빌드 불필요).

### 2026-11-20 (1) — 기준 부하 예산 기본값 0.65 → 1.0 (`plant.yaml limits.load_budget`)
- 왜: 사용자 "어차피 각속도 MPC 이제 안 씀" — 11-19 (12) 에서 기본값을 안 바꾼 이유(옛 몸체 MPC·`run_circle.sh` 와 공유)가 없어졌다.
  토크 실험에서 이 값을 읽는 곳은 `figure8_reference_node` 기동 검사뿐 (토크 MPC·torque_output·ak40 에는 바퀴속도 상한이 없다 — grep 확인).
  DRA-MPPI 는 자기 `dra_mppi.yaml` 의 0.65 를 따로 읽어 영향 없음. `plant_wheel.yaml` (구조 B) 0.65 도 그대로.
- 바꾼 파일: `mpc/config/plant.yaml` (값 + 근거 주석), `mpc/launch/torque_experiment.launch.py` (주석·인자 설명), `mpc/test/test_launch_modes.py` (docstring).
- 의미 (V_LIMIT = 0.04 × 정격 38.75 = 1.55 m/s): 기준 바퀴속도 최고 34.9 (0.9) → 38.75 rad/s. 보정 몫은 정격 위 여유뿐 —
  실측 전압 한계 42 rad/s(저토크)까지 8.4 %. W2.0 1.11 m/s 에서는 바퀴 38.5 rad/s, 42 까지 몸체 0.10 m/s
  = 퓨어 퍼슛 k_along 1.0 으로 e_along 10 cm.
  35 rad/s 위는 부하 상태로 안 쟀다 (11-19 (12)).
- 이 검사가 안 보는 것: 마찰 (W2.0 1.11 m/s 원심 → 필요 μ √2v²/(Rg) 0.355, MPC C 상한 μ̂_i·c_s 최소 FR 0.228),
  퓨어 퍼슛 `v_max` 1.0 (`torque_baseline_a.yaml`) — circles8 W2.0 이상·line 에서 기준이 이 값을 넘을 수 있다 (→ (2) 에서 1.7).
- 확인: 설치본 경로(ament share → build 링크 → src)로 적재 LOAD_BUDGET 1.0, 노드와 같은 판정으로 circles8 tangent W2.0 1.11 통과 · 1.12 거부,
  W3.0 1.23 통과 · 1.24 거부. pytest 293 통과. 실차·시뮬 판 없음 (빌드 불필요 — 설정 링크).

### 2026-11-19 (10) — 추정기 모드 `est_mode:=kf|slam` (기본 kf) + 시뮬 포즈 튐 흉내 `sim_slam_jump` + `trip_on_pose_jump` 인자
- 왜: 사용자 "원래 설계 의도(휠 오돔 없이 IMU + 라이다 SLAM, SLAM 값을 튀어도 그대로 믿기)를 모드로 고를 수 있게. 지금 kf 버전은 논의로 채택 여부를 정한다".
  속도는 사용자 선택 (a) 두 포즈 차분 + 저역통과 (구조도 v3 "포즈 미분 + LPF"). pose_jump 정지는 "시뮬로 판단".
- 바꾼 파일 (`controller/mpc`):
  - `amr_mpc/torque/estimator.py`: `mode` kf | slam. slam = 포즈 시각의 위치·방향을 SLAM 으로 덮어쓰고(게인 1) 기존 이력 전파식으로 지금까지 옮김
    (늦은 포즈를 지금 상태에 그대로 쓰면 0.7 m/s 에서 9.1 cm, 1.4 rad/s 에서 10.4° 뒤처짐), 속도 = (z_k − z_{k−1})/Δt 를 1 차 저역통과 `slam_v_lpf_hz`,
    바이어스는 기동 값 고정, 튐(> jump_xy·jump_yaw)도 따르되 개수는 셈(`jump_used`), 튄 쌍의 차분은 속도로 안 씀. kf 보정 경로는 `_apply()` 로 묶었을 뿐 식은 그대로.
  - `nodes/state_estimator.py`: ROS 파라미터 `est_mode` (param dump 에 남음), 기동 로그에 모드, `jump_used` 로그.
  - `config/torque_estimator.yaml`: `mode: kf`, `slam_v_lpf_hz: 2.0`, `slam_v_max_dt: 0.5`.
  - `launch/torque_experiment.launch.py`·`real_all.launch.py`: `est_mode`, `trip_on_pose_jump`(fault_monitor), `sim_slam_jump`(시뮬 플랜트) 인자.
  - `amr_mpc/torque/sim_sensors.py`·`nodes/sim_plant_torque.py`: 포즈 튐 흉내 `slam_jump "t,dx,dy[,dyaw_deg[,once]]"` (처음 움직인 뒤 t 초부터, once 1 = 스캔 하나만).
  - 시험: `test_torque_estimator.py` slam 7 개 (덮어쓰기·튐 따름·튄 쌍 속도 제외·지연 보상·잡음·시뮬 튐 kf vs slam·튐 흉내), `test_launch_modes.py` 인자 전달 7 개.
- 속도 저역통과 2 Hz 를 고른 근거 (`test_torque_estimator` 원 주행, 실측 수준 잡음 3 판 평균 속도 RMS): 없음 77 · 5 Hz 74 · 3 Hz 70 · **2 Hz 69** · 1 Hz 87 mm/s (kf 54).
- 시뮬 (도메인 93, circles8 W 2.0 tangent 0.7 m/s, C μ̂ 0.6, 잡음 1 배, 각 1 판, 참값 기준):

  | 조건 | 모드 | 참값 추종 \|e\| RMS / 최대 | 추정기−참값 위치 RMS / 최대, 속도 RMS | 토크 떨림 | 튐 |
  |---|---|---|---|---|---|
  | 튐 없음 | kf | 3.3 / 6.9 cm | 0.8 / 2.7 cm, 4.5 cm/s | 0.0090 | 0 |
  | 튐 없음 | slam | 5.2 / 7.9 | 1.8 / 4.2, 9.6 | 0.0138 | 0 |
  | 0.3 m 스캔 하나 | kf | 3.1 / 5.6 | 0.9 / 2.3, 5.0 | 0.0090 | 1 (버림) |
  | 0.3 m 스캔 하나 | slam | 5.0 / 7.8 | 3.2 / **29.7**, 9.5 | 0.0143 | 2 (갔다 옴) |
  | 0.2 m 계속 옮김 | kf | 13.0 / 22.0 | 15.2 / 29.9, 7.4 | 0.0107 | 5 (0.5 s 뒤 따라감) |
  | 0.2 m 계속 옮김 | slam | 12.8 / 24.3 | 15.5 / 23.7, 9.3 | 0.0135 | 1 (바로 따라감) |
  | 0.8 m 계속 옮김 | kf | 49.8 / 81.6 | 60.1 / 89.4, 6.9 | 0.0099 | 5 |
  | 0.8 m 계속 옮김 | slam | 51.8 / 82.9 | 62.4 / 83.4, 9.8 | 0.0138 | 1 |

  "계속 옮김" 은 SLAM 자체가 다른 자리로 간 것이라 두 모드 다 결국 그 자리를 따르고 참값 오차가 그만큼 남는다 (정상). 위 판은 모두 `trip_on_pose_jump:=false`.
  `trip_on_pose_jump` 기본(true) 으로는 0.2 m 계속 옮김에서 kf·slam 둘 다 첫 튐에 `pose_jump` 로 정지 (kf 는 그 포즈를 버렸는데도 선다).
- 결정 (같은 날, 사용자): slam 모드에서도 `trip_on_pose_jump` 는 지금처럼 켜 둔다 (기본 true 그대로, 코드 변경 없음). 끄는 것은 시험용 인자로만.
- 미확인: 실차. `sim_slam_jump` 의 "처음 움직인 뒤" 는 무장 전 작은 움직임에도 잡혀 튐 시각이 판마다 다를 수 있다 (한 판에서 무장 4.8 s 뒤, 다른 판 8.3 s 뒤).
  `test_torque_model.py` 슬립 플랜트 시험 2 개 실패는 같은 날 `torque_plant.yaml`(7) 변경 쪽 — 이 변경과 무관 (추정기·launch 시험 78 개 통과).

### 2026-11-19 (13) — 추정기 정지 초기화가 차체 진동에 막히던 것: 흔들림 판정을 50 ms 평균으로 (`init_block_s`)
- 왜: 17:46 이후 4 판 (174603·174739·174835·175042) 이 무장하자마자 fault_monitor "state_timeout: 한 번도 안 옴" 으로 정지 — 안 움직였다.
  /state_estimator/odom 0 개, 추정기는 내내 "정지 바이어스 아직". 정지 판정(1 s 창 원신호 std ≤ 0.10 m/s²)이 차체 진동에 걸렸다:
  무장 전 원신호 std a_x 0.150~0.168, a_y 0.090~0.109 (1 s 창 통과 0 개), 성분은 30~200 Hz (a_x 77 Hz · a_y 100 Hz · a_z 160 Hz 정점, 0~5 Hz 0 %).
  173910 (17:39) 은 a_x 0.116 이라 창 위치가 맞은 16 s 에야 통과했다 (bag 재생). 진동이 왜 커졌는지는 모른다 (기구 풀림·마운트 확인).
- 무엇: `amr_mpc/torque/estimator.py` `_static_init` 이 std 를 init_block_s (기본 0.05 s) 칸 평균으로 잰다 (마지막 덜 찬 칸 버림, ≤ 0 이면 예전처럼 원신호).
  바이어스는 원신호 평균 그대로, 문턱 0.10·0.02 그대로. 50 ms 평균 std: 정지 0.006~0.018, 173910 출발 1 s 0.20, 주행 0.33~1.5 → 저주파 움직임은 예전처럼 거부.
- 바꾼 파일: `mpc/amr_mpc/torque/estimator.py`, `mpc/config/torque_estimator.yaml` (init_block_s), 시험 `mpc/test/test_torque_estimator.py` 2 개
  (77·100 Hz 진동 std 0.17 이면 통과·바이어스 ±0.01, 예전 판정은 3 s 동안 못 잡음 재현 / 1 Hz ±0.3 m/s² 움직임은 진동이 섞여도 거부).
- 확인: pytest 293 통과. bag 재생 (/imu/base_link → PlanarStateEstimator, torque_estimator.yaml 설정): 새 판정은 5 판 모두 1.00 s 에 바이어스
  (−0.147~−0.162, +0.148~+0.155), 예전 판정은 174603~175042 36~52 s 동안 못 잡음, 173910 16.05 s. 실차 판 없음 — amrall 을 다시 띄우면 들어간다 (빌드 불필요).
- 같이 본 것: RR 모터(ID 4) CAN 응답 빠짐 "응답 3/4 — 빠진 바퀴 [3]" 판마다 6~18 번 (173910 은 [2, 3] 도 6 번). 이번 정지 원인은 아니다.

### 2026-11-19 (12) — 기준 부하 예산을 launch 인자로: `load_budget` (기본 그대로 0.65)
- 왜: 174102 (PP, circles8 width 2.0, speed 1.0) 에서 figure8_reference_node 가 시작하자마자 exit 1 — 같은 인자로 노드만 띄워 재현:
  "기준 부하 1.386 > 예산 1.007" (부하 = |vx|+|vy|+k|ω| = v(1 + k/R), 예산 = 0.65 × V_LIMIT, V_LIMIT = r × 정격 38.75 rad/s = 1.55 m/s).
  예산 0.65 로는 width 2.0 최고 0.72 · 3.0 0.80 · 4.0 0.84 m/s (노드로 확인, `analysis/torque_mpc_real/2026-11-19_speed`). 사용자 "예산 풀기".
- 무엇: torque_experiment 인자 `load_budget` (0 < 값 ≤ 1, CAN 준비 전 검사) → figure8_reference_node 파라미터. 비우면 넘기지 않아 노드 기본
  0.65 (`config/plant.yaml limits.load_budget` — 옛 몸체 MPC·DRA-MPPI 와 같이 쓰는 값이라 안 바꿨다). real_all _PASS.
  노드로 확인: width 2.0 · 1.0 m/s 는 0.9 에서 통과 (부하 89 %, 보정 여유 11 %), 0.85 에서 거부. width 3.0 · 1.1 m/s 도 0.9 에서 통과.
- 예산을 풀어도 남는 한계 (계산): width 2.0 · 1.0 m/s 는 바퀴 최대 34.7 rad/s (정격 89 %, 11-19 cmd_chain 은 35 rad/s 까지 토크 부족 없음, 그 위는 안 쟀다),
  원심 2.0 m/s² → 필요 μ √2·a/g = 0.288, 바퀴당 옆힘 토크 0.0912 (use_w 켬) = 바퀴별 μ̂ (11) 의 FR τ̄ 0.0721 의 126 % → MPC 는 원을 못 돈다
  (mu_hat_wheels:=off, μ̂ 0.4 면 FR 0.0968 의 94 %). PP 는 torque_baseline_a.yaml v_max 1.0 이라 1.0 m/s 에서 따라잡을 여유가 없다 (0.7 판에서 0.92 까지 씀).
- 바꾼 파일: `mpc/launch/torque_experiment.launch.py` (`_budget`, 인자, 전달), `mpc/launch/real_all.launch.py`, 시험 `mpc/test/test_launch_modes.py` 8 개,
  `mpc/test/test_can_setup.py` 가짜 인자. 확인: pytest 291 통과.

### 2026-11-19 (11) — 바퀴별 μ̂ `surface.mu_hat_wheels: [0.432, 0.285, 0.456, 0.350]` (사용자 값, 기본 켬, 하중 몫보다 우선)
- 왜: 사용자 "이거 일단 넣어서 ㄱㄱ" — 바퀴별 값과 잰 사분면 (FL 3, FR 1, RL 3, RR 3). 출처 bag·계산은 이 리포에 없다.
  부호 패턴(−+−−)과 사분면이 11-19 앞서 받은 화면 값(−0.138 0.182 −0.184 −0.148 → 0.436 0.575 0.582 0.468)과 같아
  같은 방식(|τ_i| / ((r/√2)(mg/4)))으로 낸 값으로 보고 mg/4 기준으로 넣었다 → 하중 몫(10)을 다시 곱하지 않는다.
- 무엇: 값이 있는 바퀴 τ_μ,i = (r/√2)·μ̂_i·(m g/4)·c_s, 0 이하인 바퀴는 (10) 대로 mu_hat × 하중 몫. friction 모드에서 mu_hat:= 를 줘도 상한은 안 바뀐다
  (launch 로그 경고). 끄기 `mu_hat_wheels:=off`. 시뮬 플랜트 물리(surface_mu·하중 몫)에는 안 쓰인다.
  τ̄ = [0.1093 0.0721 0.1154 0.0886] N·m (c_s 0.8). FR 이 가장 낮다 — circles8 R 0.5 m 정상 원의 바퀴당 옆힘 토크는
  0.7 m/s 에서 use_w 켬 0.0447 (FR 의 62 %), 끔 0.0665 (92 %). FR 70 % 예산 최고속도: use_w 켬 0.74 m/s, 끔 0.61 m/s.
- 근거와 다른 점 (확인 안 함): 11-18 171247 bag 에서 FR 은 미끄럼 < 0.05 m/s 로 접지 토크 0.157 N·m 를 냈다 (μ ≥ 0.50, 추정기 속도라 약한 근거,
  `analysis/torque_mpc_real/2026-11-16_jw/traction_lower_bound.py`). 11-17 램프에서 먼저 헛돈 바퀴는 FL 5 · FR 2.
- 바꾼 파일: `mpc_cpp` hpp·cpp (TorqueModel `mu_hat_wheels`, `tau_friction_limits()` 바퀴마다 고름), node (파라미터·검사 ≤ 2·시작 로그),
  `mpc/config/torque_plant.yaml` surface.mu_hat_wheels, `mpc/launch/torque_experiment.launch.py` (인자 `mu_hat_wheels` 값|off, CAN 전 검사,
  mu_hat 무시 경고, A 무시 목록), `real_all.launch.py` _PASS. 시험: gtest `MuHatWheelsOverrideLoadShare`, pytest launch 9 개, `test_can_setup.py` 가짜 인자.
- 확인: gtest 31 + 11, pytest 283 통과. 노드 직접 기동 (도메인 97): friction μ̂ 0.4 → τ̄ [0.1093 0.0721 0.1154 0.0886] (손계산과 같음),
  `mu_hat_wheels:=[-1…]` (off) → (10) 의 [0.0967 0.0968 0.1056 0.1058]. 실차 판 없음.

### 2026-11-19 (10) — 바퀴별 상한을 μ̂ 대신 수직력 몫으로: `body.wheel_load_share` (2026-11-18 저울 값, 기본 켬)
- 왜: 사용자가 정지 하중 측정(`analysis/torque_mpc_real/2026-11-18_load`)을 주고 "바꿔줘". 하중 몫은 로봇 성질, μ̂ 는 바닥 성질이라
  (9) 의 `mu_hat_wheels`(바퀴별 μ̂ 절댓값 — μ̂ 를 바꿀 때마다 네 값을 다시 계산해야 했다)를 지우고 몫으로 바꿨다. (9) 는 커밋 전이었다.
- 무엇: N_i = 몫_i·m g. MPC friction 상한 τ_μ,i = (r/√2)·μ̂·N_i·c_s, 시뮬 SlipPlant F_i = μ N_i tanh(Δ_i/v_ε). 몫 1/4 이면 예전 mg/4 와 같다.
  측정 몫 [0.2388, 0.2392, 0.2608, 0.2612] → μ̂ 0.4 에서 τ̄ [0.0967 0.0968 0.1056 0.1058] (같게 0.1012). 앞 4.4 % 가벼움.
  안 바꾼 것: MPC 동역학(무게중심 3.8 mm 뒤 — 무시), mass_kg 4.56 (저울 합 +0.9 %·+0.05 %), PP (μ 안 씀),
  `tools/mu_ramp_test.py` (네 바퀴가 다 미끄러지는 구간의 가속도로 μ 를 낸다 — 앞 방향 F_x = μ g Σ m_i/√2 라 몫과 무관).
- 바꾼 파일:
  - `mpc_cpp` hpp·cpp: TorqueModel `mu_hat_wheels` → `wheel_load_share` (TVec4, 기본 0.25), `tau_friction_limits()` = (r/√2)·μ̂·m g·c_s·몫.
    node: 파라미터 `wheel_load_share` (YAML body.wheel_load_share; 4 개·값 0.1~0.4·합 1 ± 0.05 아니면 기동 실패, 합 1 로 맞춤), 시작 로그.
  - `mpc/amr_mpc/torque/model.py`: `load_share()` (같은 검사), TorquePlant `wheel_load_share`·`wheel_normal_force`·`tau_friction_limits()`,
    SlipPlant `N`·`Fmax` 바퀴별 (4,). `nodes/sim_plant_torque.py`: 파라미터 `wheel_load_share`, 시작 로그 바퀴별 최대 마찰력.
  - `mpc/config/torque_plant.yaml`: body.wheel_load_share (+ 측정 근거), surface.mu_hat_wheels 지움.
  - `mpc/launch/torque_experiment.launch.py`: 인자 `mu_hat_wheels` → `wheel_load_share` (MPC + 시뮬 플랜트, CAN 준비 전 검사, friction 아니면 경고).
    `real_all.launch.py` _PASS.
  - 시험: gtest `TorqueMPC.PerWheelMuHat` → `WheelLoadShare`; pytest launch 9 개 교체 (MPC·시뮬 플랜트 전달, 거부, 경고, real_all),
    `test_torque_model.py` 2 개 추가 (몫·한계, 가벼운 앞바퀴 미끄럼이 큼) + 기존 2 개 손봄 (슬립 포화: Fmax 가 (4,) — 합으로;
    게걸음·회전 부호: 하중 같게 — 몫이 다르면 tanh 미끄럼이 바퀴마다 달라 회전이 8e-5 rad/s 섞인다), `test_can_setup.py` 가짜 인자.
- 확인: gtest 30 + 11, pytest 274 통과. 노드 직접 기동 (도메인 97): MPC friction μ̂ 0.4 → τ̄ [0.0967 0.0968 0.1056 0.1058],
  `wheel_load_share:=[0.25…]` → 0.1012, 합 1.2 → 기동 실패. 시뮬 플랜트 surface_mu 0.4 → 바퀴별 최대 [4.27 4.28 4.67 4.67] N. 실차 판 없음.
- 주의: friction 모드의 기본 상한이 바뀐다 (앞 −4.5 %, 뒤 +4.4 %). 예전과 같게 비교하려면 `wheel_load_share:="0.25,0.25,0.25,0.25"`.

### 2026-11-19 (9) — 바퀴별 μ̂ 토크 상한 `mu_hat_wheels` (기본 끔, 값은 아직 없음)
- **(10) 에서 `wheel_load_share` 로 바뀌었다 — 아래 `mu_hat_wheels` 는 지금 코드에 없다.**
- 왜: 사용자 "각 바퀴 토크 제약에 이 값들 반영" — 다른 도구가 8자 bag 의 토크 −0.138 0.182 −0.184 −0.148 N·m 를 μ = |τ|/((r/√2)(mg/4))
  로 바꾼 0.436 0.575 0.582 0.468. 환산식(분모 0.3163 N·m)은 식-21 과 같지만 값 자체는 쓰지 않았다:
  ① 출처를 못 찾았다 — 8자 bag 97 개의 /wheel_states·/wheel_torque_cmd·/torque_mpc/cmd_raw·/wheel_pi/cmd_raw 어디에도 그 순간·바퀴별 최대가 없다.
  ② 측정 토크에는 구동계 마찰(쿨롱 0.016~0.021, μ +0.05~0.065)·J_w ω̇·정지 마찰(최대 0.08)이 섞인다.
  ③ 각 값이 그 바퀴의 슬립 시작이어야 바퀴 한계다 — 부호 −+−− 는 한 방향 시험 패턴이 아니다 (영공간 몫 0.188 N·m).
  실측 근거: 11-17 램프 7 판에서 먼저 헛돈 바퀴 FL 5 · FR 2 · RL·RR 0 (`analysis/torque_mpc_real/2026-11-17_ramp/RESULTS_KO.md`) — FL 이 약한 것은
  맞지만 μ 크기는 판정 기준에 따라 0.15~0.51 로 갈린다. 그래서 장치만 넣고 값은 지금 하부에서 다시 잰다.
- 무엇: τ̄_i = min(τ_lim − τ_fric,max, (r/√2)·μ̂_i·(mg/4)·c_s), limit_mode friction 에서만. 비우면 mu_hat 하나 (예전과 같음).
  μ̂_i 에 바퀴별 수직력을 같이 담는다: 저울로 잰 N_i 가 있으면 μ̂_i = μ̂·N_i/(mg/4).
  `amrall use_mpc:=true limit_mode:=friction mu_hat_wheels:="0.40,0.45,0.45,0.42" …` (순서 FL, FR, RL, RR)
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — TorqueModel `mu_hat_wheels` (TVec4, 음수 = mu_hat), `tau_friction_limits()`;
    `TorqueMPC::tau_bar()` 가 friction 모드에서 바퀴별로 (QP 상한·변화율 Δτ̄_i = τ̄_i/3 은 원래 바퀴별이었다).
  - `mpc_cpp/src/torque_mpc_node.cpp` — 파라미터 `mu_hat_wheels` (YAML surface.mu_hat_wheels, 4 개·0 < μ̂ ≤ 2 아니면 기동 실패), 시작 로그에 바퀴별 τ_μ,i·τ̄.
  - `mpc/config/torque_plant.yaml` — surface.mu_hat_wheels: [] (+ 근거 주석).
  - `mpc/launch/torque_experiment.launch.py` — 인자 `mu_hat_wheels` ("a,b,c,d"), CAN 준비 전 형식 검사, friction 이 아니면 launch 로그 경고, A 에서는 무시 목록.
    `mpc/launch/real_all.launch.py` — _PASS.
  - 시험: `mpc_cpp/test/test_torque_mpc.cpp` `TorqueMPC.PerWheelMuHat` (바퀴별 τ̄, 음수 → mu_hat, 100 스텝 동안 바퀴마다 자기 상한·변화율 안, FL 상한에 붙음,
    motor_only 무시), `mpc/test/test_launch_modes.py` 9 개 (전달·형식 거부(CAN 전)·경고·real_all), `mpc/test/test_can_setup.py` 가짜 인자 `mu_hat_wheels`.
- 확인: gtest 30 + 11, pytest 258 통과. 노드 직접 기동(도메인 97): friction μ̂ 0.5 + [0.44, 0.58, 0.58, 0.47] → τ̄ [0.1113 0.1468 0.1468 0.1189],
  motor_only → 1.2 네 개 + "안 쓰인다" 로그, 3 개 → "기동 실패: mu_hat_wheels 는 4 개". 실차 판 없음.
- 빌드 메모: `colcon build --symlink-install --packages-select amr_mpc` 가 "option --uninstall not recognized" 로 실패한다 (setuptools 쪽, 이번 변경과 무관).
  build/amr_mpc 의 launch·config 는 src 로 걸린 링크라 이번 변경은 다시 빌드하지 않아도 들어간다. amr_mpc_cpp 는 빌드했다.

### 2026-11-19 (8) — 직선 급가속 슬립 비교 시뮬 (코드 변경 없음)
- 왜: 사용자 질문 "직진 급가속에서 슬립 제어 차이가 나나". 조건: shape:=line 2.5 m 왕복, 1.0 m/s, accel 3.0 m/s², 슬립 모델 바닥 μ 0.45
  (바퀴 슬립 토크 0.142 N·m), 새 기하 k 0.1932. 참 슬립률 λ = (ω − ω_nom)·sgn(ω_nom)/max(|ω|, |ω_nom|, 1), ω_nom = T⁻¹ v_참값 (slip_logger 와 같은 정의).
  | 판                         | λ 최대 | |λ|>0.1 시간 | 슬립 속도 최대 | 실제 가속 최대 | 뒤처짐 최대 | 위치 RMS | τ 최대 |
  | A 엔코더 (표준)              | 0.69  | 17.5 %      | 0.211 m/s    | 3.13 m/s²    | 73 mm     | 36.8 mm | 0.284 |
  | A 추정기 (pp_speed_source)   | 1.99  | 99 %        | 발산          | 11.4         | 1.43 m (fault_monitor 정지) | 673 | 1.30 |
  | MPC μ̂ 0.4                  | 0.10  | 0.2 %       | 0.017        | 1.53         | 110 mm    | 52.0    | 0.146 |
  | MPC μ̂ 0.55                 | 0.25  | 2.6 %       | 0.025        | 2.08         | 54 mm     | 29.3    | 0.180 |
  - A 추정기: 바퀴가 미끄러져도 추정기 속도가 안 올라가 PI 적분이 상한(1.3 N·m)까지 → 헛돎이 커지는 와인드업. 시뮬 플랜트에 모터 속도 한계가 없어
    바퀴 속도가 비현실적으로 커졌다 (실차는 무부하 45.6 rad/s 에서 멈추지만 헛돎·이탈은 같은 방향).
  - MPC μ̂ 0.4 는 τ̄ 0.101 로 가속이 1.5 m/s² 에 묶여 슬립은 없지만 더 뒤처진다. μ̂ 0.55 (× c_s 0.8 = 0.44 ≈ 바닥 0.45) 면 슬립·추종 모두 A 엔코더보다 낫다.
- 실차 의미: 직선은 앞 방향 μ (11-17 램프 0.44~0.51, 하한) 기준 μ̂ 0.55 근처, 원은 옆 방향 μ (0.31~0.41) 기준 μ̂ 0.4~0.5.

### 2026-11-19 (7) — 새 하부(frame + AK40-10 + 커플러) 기하·센서 위치 반영 (회전자 관성은 실측 유지)
- 왜: 문서 "AMR 하부 변경 전달 사항 — frame + AK40-10 + 커플러" (2026-11-19, CAD). 사용자 결정: 회전자 관성은 벤치 실측
  (inertia_J_w_kgm2 8.9e-4, 바퀴별 값) 그대로 — 문서의 "540 g·cm² × 100 = 5.4e-3 kg·m²" 해석은 실측의 6 배라 쓰지 않는다. 나머지는 반영.
- 바꾼 값:
  | 파일 | 항목 | 예전 → 새 |
  | mpc/config/torque_plant.yaml, plant.yaml, plant_wheel.yaml | wheel_base_m / robot_width_m | 0.1897 / 0.207 → 0.1724 / 0.2319 (k 0.19835 → 0.20215) |
  | mpc/config/torque_estimator.yaml | imu_offset_x | 0.098 → 0.1093 (base_link 가 몸체 기준 −11.3 mm 앞뒤 이동, 센서는 그대로) |
  | sensors/amr_description/urdf/mech/amr_basic_wheel.urdf.xacro | 바퀴 조인트 | (±0.09491, ±0.10352) → (±0.0862, ±0.11595) |
  | sensors/amr_description/urdf/robots/amr_basic.urdf.xacro | laser_front / laser_rear / base_scan z / camera | 예전 실측값 + (+0.0113, 0, −0.0075):
    (0.097591, −0.020494, 0.170297) / (−0.080092, 0.02052, 0.170297) / 0.170297 / (0.12528, 0.0475, 0.085272) |
  센서는 문서의 반올림값(0.099·0.169 등) 대신 예전 실측값에 문서의 base_link 이동을 더했다. xacro 전개로 확인: URDF 의 base_link → camera_gyro
  = (0.12528 − 0.01602, 0.0475 − 0.03022) = (0.1093, 0.0173) — torque_estimator imu_offset 과 일치.
- 그대로 둔 것: 질량 4.56 kg·I_z 0.06 (새 frame 재질에 따라 바뀜 — 재측정 필요), 바퀴 visual 메시 위치, C++ TorqueModel 구조체 기본값
  (1판 수치 시험용, 노드는 yaml 을 읽는다). 이 스택 밖의 같은 값 (팀원 영역, 안 바꿈): DRA_MPPI (dra_mppi.yaml·mppi_controller.py·dra_mppi_node.py),
  amcl.yaml·amcl_wiggle_node.py, amr_bringup base.yaml·joy_teleop.yaml·roboclaw_mecanum_node.py (JGB37 시절), stack_master check_localization.py,
  sys_id, mpc/tools/diag_fig8.py, amr_basic.urdf (xacro 아닌 옛 파일).
- 이후 `mpc/config/plant.yaml` robot_width_m 이 0.2140 으로 바뀌어 있었다 (Claude 가 바꾼 것 아님). 사용자 확인: 의도한 값.
  plant.yaml 은 figure8 바퀴 속도 예산 검사의 K_GEOM 출처라 K_GEOM = (0.1724 + 0.2140)/2 = 0.1932.
  → 사용자 결정 "전부 0.2140": torque_plant.yaml·plant_wheel.yaml robot_width_m, amcl.yaml robot_width 0.2319 → 0.2140,
    URDF 바퀴 조인트 y ±0.11595 → ±0.1070 (트랙 214.0 mm, 문서 CAD 231.9 와 다름). k = 0.1932 (TorquePlant·K_GEOM 같음), xacro 전개 확인.
    pytest 249·gtest 29 통과.
- 센서·측위 쪽 추가 (같은 날, 사용자 질문 "센서쪽도"):
  - `perception/amr_perception_bringup/config/amcl.yaml` amcl 보조(wiggle) 노드 wheel_base / robot_width → 0.1724 / 0.2319 ("plant_wheel.yaml 과 반드시 같게").
  - `perception/sensor_fusion/sensor_fusion/camera_lidar_fusion_node.py` laser_origin_x 기본값 0.086291 → 0.097591 (URDF laser_front 와 같게).
    sensor_fusion 은 설치본이 복사본이라 PYTHONNOUSERSITE=1 --symlink-install 로 다시 빌드 → 이제 소스 링크.
  - 바꿀 필요 없음 (확인): Cartographer·지도 — 라이다와 IMU(tracking_frame)가 같은 만큼 옮겨져 상대 위치가 같다
    (laser_front − camera_gyro = (−0.011669, −0.037774, 0.077625) 예전·새 같음). scan_merge.yaml 섹터 (base_scan 기준 각도, 5° 겹침),
    imu_to_base_node·pose_to_odom (TF 로 읽음), sim_plant_torque 의 IMU 위치 (torque_estimator.yaml 을 읽음).
  - 남긴 것: traversability.yaml·depth_preprocess.yaml 의 높이 주석 (카메라 높이 0.1328 → 0.1253 m, 라이다 0.2178 → 0.2103 m — 값은 지면 기준이라
    동작은 그대로, z_max 는 예전 값이 7.5 mm 보수적), traversability·amr_perception_bringup 의 가짜 센서 도구 (fake_robot·fake_scan·fake_depth_cloud),
    scan_merge.yaml 주석의 라이다 높이 0.1778.
- 확인: pytest 249, gtest 29 통과. TorquePlant k 0.20215, K_GEOM 0.20215. 시뮬 (circles8 2.0, 0.7 m/s, μ̂ 0.6, use_w) 주행 RMS 21.2 mm, 요 1.05°
  (예전 기하 21.4 mm, 1.07° 와 같은 수준). yaml·xacro 는 설치본이 링크라 다음 실행부터 바로 적용 — 예전 하부로 돌리면 기하가 틀린다.

### 2026-11-19 (6) — 비교군 A 바퀴 PI 의 속도 되먹임을 추정기로 고를 수 있게 (`pp_speed_source:=estimator`)
- 왜: 사용자 결정 — MPC 에 엔코더를 안 쓰므로 비교군 A 도 위치·속도 정보 출처를 추정기로 같게 해서 비교.
- 바꾼 파일:
  - `mpc/amr_mpc/nodes/wheel_pi.py`: 파라미터 `speed_source` (wheels 기본 | estimator), `speed_topic` (기본 /state_estimator/odom).
    estimator 면 추정기 몸체 속도 [v_x, v_y, ω_z] 를 T⁻¹ 로 바퀴 속도로 바꿔 PI 되먹임에 (실행 시각은 /wheel_states 마다 그대로),
    추정기 값이 wheel_timeout 보다 묵으면 DAMPING. 기동 로그에 출처.
  - `mpc/launch/torque_experiment.launch.py`: 인자 `pp_speed_source`, `pp_kp`, `pp_ki` (speed_topic 은 MPC 와 같은 state_topic). `real_all.launch.py`: 넘기기.
  - `mpc/test/test_launch_modes.py`: 시험 2 개.
- 시뮬 (circles8 3.0 tangent 1.0 m/s, 슬립 μ 0.4, max_wheel 49, 참값 기준, 판마다 1 판):
  | 판                                   | 위치 RMS / 최대 | 요 RMS / 최대   | τ max | Δτ RMS |
  | A 엔코더 (표준)                        | 17.4 / 54.2 mm | 1.85° / 12.2°  | 0.149 | 0.0033 |
  | A 추정기 (게인 그대로 kp 0.03 ki 0.15)  | 23.4 / 46.1    | 3.09° / 16.2°  | 0.254 | 0.0164 |
  | A 추정기 (게인 절반)                    | 24.3 / 50.6    | 3.66° / 15.9°  | 0.201 | 0.0078 |
  | MPC 추정기 (friction μ̂ 0.4 use_w)      | 21.5 / 44.7    | 1.01° / 7.5°   | 0.155 | 0.0123 |
  같은 정보에서 MPC 가 위치 8 % (판 차이 ±1 mm 수준이라 작음)·요 3 배·토크 최대 40 % 낮음. ※ A 의 토크 상한은 motor_only τ̄ (1.2 N·m),
  MPC 는 friction τ̄ (μ̂ 0.4 → 0.101 N·m + 마찰 보상) — A 가 토크를 더 쓸 수 있는 조건. 실차 판 없음. pytest 249 통과.

### 2026-11-19 (5) — Cartographer 스캔 하나씩 매칭 lua (`localization_torque_acc1_2d.lua`) + 추정기 `est_pose_period` 인자
- 왜: SLAM 포즈 지연 131 ms 중 앞·뒤 두 스캔 누적 몫(약 25 ms)을 빼고 포즈를 20 Hz 로 (2026-11-19 (4) 분해).
- 바꾼 파일:
  - `perception/amr_perception_bringup/config/localization_torque_acc1_2d.lua` (새): localization_torque_2d.lua + num_accumulated_range_data 1.
    cartographer_print_configuration 로 해석 확인 (num_accumulated_range_data = 1). 설치하려고 amr_perception_bringup 을 다시 빌드
    (PYTHONNOUSERSITE=1 --symlink-install; 첫 번째는 복사→링크 전환에서 실패, 두 번째 성공. 새 파일만 링크, 나머지는 예전 복사본).
  - `mpc/launch/torque_experiment.launch.py`: 인자 `est_pose_period` → state_estimator_node pose_period (비우면 노드 기본 0.1).
  - `mpc/launch/real_all.launch.py`: `est_pose_period` 넘기기. lua 이름에 acc1 이 있고 직접 안 주면 0.05 를 자동으로 (로그).
  - `mpc/test/test_launch_modes.py`: 시험 5 개 (est_pose_period 전달, acc1 자동·직접 지정).
- 게인 (torque_estimator.yaml 잡음): 포즈 주기 0.1 s → K_p 0.711, K_v 4.162, K_b −1.133 | 0.05 s → K_p 0.521, K_v 3.756, K_b −1.032.
- 기대 효과 (175240 재생, 10 Hz 포즈만 있어 20 Hz 효과는 못 봄): 포즈 도착을 stamp + 131 → 106 ms 로만 바꾸면 v_x 지연 +85 → +80 ms (작다).
  10 Hz 포즈에 0.05 s 게인을 쓰면 +110 ms 로 나빠진다 → 실차에서 /tracked_pose 가 실제로 20 Hz 인지 확인하고, 아니면 est_pose_period:=0.1.
- 확인: pytest 247 통과. 실차 판 없음.

### 2026-11-19 (4) — D455 IMU 기본 400/400 Hz + 추정기 속도 지연 분해 (MPC 엔코더 속도는 안 쓰기로)
- 사용자 결정: 슬립 때문에 MPC 에 엔코더 속도(velocity_source:=wheels)는 쓰지 않는다 (옵션은 기본 estimator 로 남김). IMU 주기만 최대로.
- 바꾼 파일: `sensors/amr_bringup/launch/sensors.launch.py` `depth_camera_accel_fps`·`depth_camera_gyro_fps` 기본 0 → 400 (unite 는 2 그대로).
  예전 동작은 0 (장치 기본, IMU 메시지 200 Hz). 실차 판 없음 — amrlag 로 가속도계·추정기 지연, pidstat 로 추정기 CPU 확인할 것.
- 추정기 안쪽 지연 분해 (코드 변경 없음): 11-18 175240 의 IMU·/tracked_pose 를 PlanarStateEstimator 에 수신 순서대로 재생 →
  기록된 /state_estimator/odom 과 v_x 차 RMS 0.0000 m/s (그대로 재현). 바퀴 기구학 v_x 기준 상호상관 (격자 오차 ±10 ms):
  | 조건                                  | v_x 지연 |
  | 재생 (실차와 같게)                      | +85 ms (기록 +95) |
  | SLAM 포즈 처리 지연 0 (stamp 에 바로 도착) | +35 ms |
  | 가속도만 40 ms 앞당김                    | +75 ms |
  | 둘 다                                   | +20 ms |
  | accel_noise_density ÷3 (IMU 더 믿음, K_v 4.16 → 2.08) | +85 ms |
  | accel_noise_density ×3 (SLAM 더 믿음, K_v 7.80)        | +65 ms (r 0.98 → 0.96, 잡음 증가) |
  → 가장 큰 몫(≈50 ms)은 /tracked_pose 가 stamp 보다 131 ms 늦게 도착하는 것. 이력 보정 전파(δv(D) = δv − D δb)로 시각은 맞지만
    최근 131 ms 의 속도 변화는 IMU 적분에만 의존하고, 그 가속도계가 늦고(+50~55 ms) 잡음이 크다 (바퀴 가속 대비 잔차 RMS 0.33~0.41 m/s²,
    0.5~3 Hz 진폭비 0.70~0.91·위상 −10~−38°). 추정기 코드 결함이 아니라 설계대로의 결과.
  SLAM 지연 131 ms 의 구성 (175240, 같은 날 추가 분석): 라이다 /scan_front·/scan_rear 는 10 Hz, header.stamp = 첫 점, scan_time 99 ms,
  메시지 수신 = 마지막 점 + 0.5 ms (스캔이 다 돌아야 나온다 → 첫 점은 이미 100 ms 지난 것). *_carto 필터 +0.8 ms.
  Cartographer 처리: tracked_pose 수신 − 직전 필터 스캔 수신 = 5.9 ms (중앙). tracked_pose stamp ≈ 최신 스캔 stamp − 25 ms
  (num_accumulated_range_data 2 — 앞·뒤 두 스캔을 모아 한 번). → 131 ms ≈ 스캔 한 바퀴 100 ms + 두 스캔 누적 25 ms + 처리 6 ms.
  Cartographer 계산 자체는 원인이 아니다.

### 2026-11-19 (3) — D455 IMU 표본 주기·결합 방식 launch 인자 + 센서 지연 측정 도구 `tools/sensor_lag.py` (`amrlag`)
- 왜: 추정기 v_x 가 바퀴보다 +90~95 ms 늦은 원인을 나눠 보니 가속도계 a_x 가 이미 +55 ms (자이로 +15 ms) — 11-18 bag 175240·170819.
  D455 는 unite_imu_method 2 (linear_interpolation), accel·gyro 주기 장치 기본 (IMU 메시지 200 Hz). 나머지 35~40 ms 는 추정기 안 (원인 미확인).
  D455 fw 5.17.0.10 지원값 (rs-enumerate-devices): accel 400/200/100 Hz, gyro 400/200 Hz.
- 바꾼 파일:
  - `sensors/amr_bringup/launch/sensors.launch.py`: 인자 `depth_camera_accel_fps`, `depth_camera_gyro_fps` (기본 0 = 장치 기본 → 동작 그대로)
    → realsense2_camera 4.58.4 파라미터 accel_fps·gyro_fps.
  - `mpc/launch/real_all.launch.py`: `imu_accel_fps`, `imu_gyro_fps`, `imu_unite` → sensors.launch.py 의 depth_camera_* (비우면 안 넘김).
  - `mpc/tools/sensor_lag.py` (새): 바퀴 엔코더 기준 상호상관으로 자이로·가속도계·추정기 v_x·ω_z·SLAM v_x 지연, IMU 메시지 주기, bag 의 imu_ 인자.
  - `mpc/test/test_sensor_lag.py` (새, 2 개), `mpc/test/test_launch_modes.py` (amrall imu_* 가 sensors 로 가는지 2 개), `mpc/tools/amr_aliases.sh` (`amrlag`).
- 확인: pytest 242 통과. 175240 에 amrlag: 자이로 +15, 가속도계 +55, 추정기 v_x +95, 추정기 ω_z +15, SLAM v_x −25 ms. 설정을 바꾼 실차 판 없음.
  ※ 표본 주기를 올리면 IMU 메시지가 400 Hz 가 될 수 있다 (추정기 CPU·DDS 부하 확인).

### 2026-11-19 (2) — 8자 `yaw_mode:=spin` (경로와 따로 헤딩 회전, MPC 만 가능) + 1.0·1.5 m/s·spin 시뮬 비교
- 왜: 발표에서 A(퓨어 퍼슛)보다 나은 점을 보여 줄 세팅 (사용자 제안: "8자를 요 회전하면서 진행 — MPC 만 가능").
- 바꾼 파일:
  - `mpc/amr_mpc/core/figure8.py`: yaw_mode `spin`. θ = κ_s·σ (호길이), κ_s = 2πn/L, n = round(spin_rate·L/V/2π) ≥ 1 → 끝에서 출발 헤딩.
    ω = κ_s·v 라 속도 프로파일을 따른다 (출발·정지 0). 위치 기준은 fixed 와 같다. 정렬 구간 없음.
  - `mpc/amr_mpc/nodes/figure8_reference.py`: 파라미터 `spin_rate` (기본 1.0 rad/s), 기동 로그에 "헤딩 n 바퀴 회전, 순항 ω".
  - `mpc/launch/torque_experiment.launch.py`: 트랙 키 `spin_rate`, yaw_mode 허용값에 spin. `real_all.launch.py`: `spin_rate` 넘기기.
  - `mpc/config/torque_track.yaml`: `spin_rate: 1.0`.
  - `mpc/test/test_figure8.py`: MODES 에 spin (경계상자·닫힘·유한차분 검산), 새 시험 2 개 (정수 바퀴·위치 = fixed·속력 같음, spin_rate ≤ 0 거부).
    기존 "잘못된 yaw_mode" 예시 "spin" → "spiral".
- 부하 (circles8 width 3.0, spin 1 rad/s): v 0.5 → 0.905 (기본 검사 1.007 통과), 0.7 → 1.175 (max_wheel_rad_per_sec:=46, 바퀴 76 % 정격), 1.0 → 바퀴 정격 초과.
- 시뮬 (슬립 모델 바닥 μ 0.4, MPC: 지금 가중치 friction μ̂ 0.4 use_w, 참값 기준 주행 RMS / 최대, 요 RMS):
  | 조건                                  | A 퓨어 퍼슛                  | MPC 추정기 속도        | MPC 바퀴 속도            |
  | circles8 3.0 tangent 1.0 m/s (mw 49)  | 17.1 / 54.7 mm, 2.34°        | 22.9 / 47.5, 1.01°   | **9.3 / 19.5, 1.01°**   |
  | 직선 3 m 1.5 m/s (가속 1.0, mw 58)     | 277.7 / 502 mm ※ v_max 1.0   | 17.6 / 35.1, 0.40°   | **5.6 / 11.2, 0.39°**   |
  | circles8 3.0 spin 1 rad/s 0.5 m/s     | 발산 (track_error 150 cm 정지, 요 최대 180°) | 22.4 / 49.8, 0.41° | **5.2 / 9.5, 0.37°** |
  MPC 바퀴 속도 μ̂ 0.35 (1.0 m/s): 11.1 / 30.0 mm. ※ A 의 1.5 m/s 는 torque_baseline_a.yaml `v_max: 1.0` (명령 상한) 때문에 평균 20 cm 뒤처진 것 —
  공정 비교하려면 v_max 를 올리고 다시. 시뮬 플랜트에는 모터 토크-속도 곡선이 없다 (고속에서 낙관적). 실차 판 없음.
- 확인: pytest 238 통과.

### 2026-11-19 (1) — A·MPC 공통 비교 도구 `tools/compare_slam.py` (`amrcmp`)
- 왜: 발표용 A(퓨어 퍼슛) vs MPC 비교. analyze_run.py 는 /torque_mpc/status(추정기 기준)라 A bag 을 못 읽는다.
- 무엇: 제어기와 상관없는 토픽만 쓴다 — 기준열(/dra_mppi/reference_trajectory)을 SLAM 포즈 시각에 보간, /tracked_pose 를
  IMU 점 → 중심 (torque_estimator.yaml imu_offset 0.098, 0.017 — 2026-11-18 에 보정 안 해서 진행 방향 +98 mm 로 보였던 것) 으로 옮겨 비교.
  지표: 위치 RMS·최대, e_t·e_c (RMS·평균), 교차점 창·나머지, 요 RMS·최대, |τ| 최대·Δτ RMS (/wheel_torque_cmd, DAMPING 제외),
  슬립 = 바퀴 기구학 − SLAM 중심 속도 (SLAM 차분 잡음으로 0.1 m/s 근처가 바닥 — 상대 비교만). `--csv` 로 표 저장. 추종 판이 아니면 이유를 적고 건너뜀.
- 바꾼 파일: `mpc/tools/compare_slam.py` (새), `mpc/test/test_compare_slam.py` (새, 5 개: IMU 점 보정, 안쪽 offset = +e_c, 0.1 s 지연 = e_t −69.8 mm,
  시계 방향 부호, 교차 창·무장 구간), `mpc/tools/amr_aliases.sh` (`amrcmp`, amrhelp 범위).
- 확인 (실차 bag): MPC μ̂ 0.9 w (11-18 170819) RMS 40.8 / 최대 82.3 mm, 요 3.99° | A (11-17 161611, 0.7 m/s) 40.4 / 71.1 mm, 요 5.77° (최대 20.7°) |
  MPC μ̂ 0.3 w (175240) 55.4 / 111.5 mm. A 와 MPC 는 다른 날 판이라 직접 비교는 같은 날 다시. pytest 232 통과.
- 같이 계산 (최고속도, 코드 변경 없음): figure8 의 바퀴 속도 예산 검사 v(1 + k/R) ≤ 0.65·r·max_wheel (정격 38.75 rad/s) 가 width 2.0 에서
  0.72 m/s 로 막는다 (11-18 170149 speed 1.0 이 기준 없이 끝난 이유). 검사를 풀면 원 위 실용 최고 (μ 0.35~0.41, c_s 0.8, τ̄ 70 % 사용):
  width 2.0 0.82~0.89 · 3.0 1.01~1.09 · 4.0 1.17~1.26 m/s (모터 정격 한계 1.11 · 1.23 · 1.29). 11-17 램프 CSV 의 μ: 앞 0.44~0.51, 옆 0.31~0.41
  (추정기 속도로 슬립을 판정해 낮게 나왔을 수 있다).

### 2026-11-18 (6) — MPC 속도 상태를 바퀴 엔코더로 고를 수 있게 (`velocity_source:=wheels`, 기본 estimator)
- 왜: 오늘 실차 bag(지금 가중치, circles8 0.7 m/s) 의 SLAM 기준 오차 41~65 mm RMS (시뮬 21 mm 의 2~3 배), 요 3~4.5°.
  상호상관 (175240·175112·170819): 추정기 v_x 가 바퀴 기구학보다 +90~120 ms 늦음 (r 0.98). 추정기 ω_z 는 자이로와 0 ms,
  SLAM 포즈는 자이로·바퀴와 −50~+50 ms. 지금 Q 는 등가 k_d/k_p 1.18 s — 등가 PD 근사에 속도 지연을 넣으면 0.10 s 에 ζ 0.08,
  0.12 s 에 불안정 (11-18 (1) 가중치는 0.15 s 까지 ζ ≥ 0.5). bag 의 토크 전력 22~58 % 가 1~4 Hz. 퓨어 퍼슛(A)은 속도 루프를 엔코더로 닫는다.
  ※ 분석 중 실수: /tracked_pose 는 D455 IMU 점 (중심에서 0.098, 0.017 m) — 보정 안 하면 진행 방향 +98 mm 로 보인다.
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`: `TorqueModel::wheel_body_velocity(ω)` = T ω (force_map 의 T⁻¹ 짝).
  - `mpc_cpp/src/torque_mpc_node.cpp`: 파라미터 `velocity_source` (estimator | wheels), `wheel_timeout` (0.05 s), `wheel_states_topic`.
    wheels 면 /wheel_states 로 몸체 v_x·v_y 를 바꾸고 ω_z 는 추정기 값. 바퀴가 묵으면 그 주기는 추정기 속도 (경고, 누적 횟수). 기동 로그에 출처.
  - `mpc/config/torque_mpc.yaml`: `state_input:` 절 (velocity_source estimator, wheel_timeout 0.05).
  - `mpc/launch/torque_experiment.launch.py`·`real_all.launch.py`: 인자 `velocity_source`.
  - `mpc_cpp/test/test_torque_mpc.cpp`: `WheelKinematicsInvertsForceMap`.
- 시뮬 (circles8 2.0, tangent, 0.7 m/s, C μ̂ 0.6, use_w, 지금 가중치, 참값 기준 주행 RMS):
  | 플랜트 슬립      | 추정기 속도        | 바퀴 속도       |
  | 없음 (두 판씩)   | 28.7 · 26.2 mm   | 5.3 · 5.4 mm  |
  | 바닥 μ 0.6       | 21.9 mm          | 6.2 mm        |
  | 바닥 μ 0.45      | 21.7 mm          | 7.0 mm        |
  요 RMS 는 어느 판이나 1.05~1.5° (ω_z 는 그대로 추정기라 바뀌지 않음, Q 가 정함). τ max·Δτ 비슷. (참고: 시뮬 PP 12.4 mm, 슬립 없음)
- 확인: gtest 29·pytest 227 통과. 실차 판 없음 — 실차 슬립(바퀴 − 추정기 RMS 0.08~0.19 m/s)은 시뮬보다 크다.

### 2026-11-18 (5) — 입력 변화 벌점을 시간 미분으로 (`tuning.w_dudt`, r_τ̇ [(N·m/s)⁻²])
- 왜: 교수님 지시 (사용자 전달) "Δu 를 시간에 대한 미분으로 고려하도록". 비용의 둘째 항을 r_τ̇ ‖Δτ/Ts‖² 로 본다.
  r_τ̇ ‖Δτ/Ts‖² = (r_τ̇/Ts²) ‖Δτ‖² 이므로 코어(식-16·18, `TorqueMPC`)는 그대로 두고 노드가 스텝 단위 r_Δ = r_τ̇/Ts² 로 바꿔 넘긴다.
  가중치 숫자가 Ts 에 묶이지 않는다 (Ts 를 바꿔도 r_τ̇ 은 같은 의미).
- 바꾼 파일:
  - `mpc_cpp/src/torque_mpc_node.cpp`: 파라미터 `w_dudt` (yaml tuning.w_dudt). `w_delta`(스텝 단위)를 주면 그것이 우선 (예전 launch·bag 재현용).
    둘 다 없으면 시작 거부. 기동 로그에 "가중: W […] r_Δ …/스텝 (= w_dudt … / Ts²) r_U … q_null …" 한 줄.
  - `mpc/config/torque_mpc.yaml`: `w_delta: 10.0` → `w_dudt: 0.001` (= 10 × 0.01²). 주석에 환산표 (지시서 0.01 → 1e-6, 400 → 0.04, 1e6 → 100).
  - `mpc/launch/torque_experiment.launch.py`·`real_all.launch.py`: 인자 `mpc_w_dudt`. `mpc_w_delta` 는 스텝 단위로 남김.
  - `mpc/tools/analyze_run.py`: params 출력에 w_dudt.
- 확인: gtest 28·pytest 227 통과. 시뮬(위 (4) 조건) yaml 기본값 판 = 스텝 10 직접 판: 위치 RMS 21.4 / 21.4 mm, Δτ RMS 0.0126 / 0.0126,
  기동 로그 "r_Δ 10 /스텝 (= w_dudt 0.001000 (N·m/s)⁻² / Ts²)". 동작은 바뀌지 않았다 (단위만).

### 2026-11-18 (4) — Q 를 교수님 지시값 [5, 5, 2.5, 2, 2, 1] 로 되돌리고 입력만 새로 (w_delta 10, r_u 0.1)
- 왜: 사용자 확인 — 2026-11-17 저녁 yaml 의 Q [5, 5, 2.5, 2, 2, 1] 은 교수님이 정한 값. (1) 에서 "입력쪽만" 을 W 원래값 고정으로 잘못 읽어
  덮어썼다 (Claude 실수). (1)·(3) 의 W 변경(θ̇ 0.125 포함)은 무효, 이 Q 고정으로 입력만 다시 고름.
- 계산 (무제약, θ_r = 0 직선, m_w 포함): 이 Q 에서는 입력을 아무리 싸게 해도 가장 느린 pole ≈ −1.03 — A 의 y (ω_n 2.95) 에 못 닿는다.
  어제 18:45 값 wΔ 1e6·rU 1e4 는 −0.01 ± 0.11j, 사용자 제안 1e4·100 은 −0.56. k_d/k_p 는 입력과 거의 상관없이 1.0~1.2 s (W0 0.21~0.38) —
  추정기 속도 잡음에 약 3 배 민감 (2026-11-18 (2) 의 추정기 몫 분석).
- 시뮬 (circles8 2.0, tangent, 0.7 m/s, C μ̂ 0.6, use_w, 추정기 상태):
  | wΔ  | rU  | 위치 RMS 주행 | 요 RMS 주행 | τ max | Δτ RMS |
  | 100 | 1   | 27.6 mm | 1.35° | 0.154 | 0.0042 |
  | 30  | 1   | 24.7 mm | 1.49° | 0.164 | 0.0086 |
  | 10  | 1   | 22.6 mm | 1.43° | 0.186 | 0.0121 |
  | **10** | **0.1** | **21.4 mm** | **1.45°** | 0.179 | 0.0126 |
  | 3   | 0.1 | 20.4 mm | 1.41° | 0.194 | 0.0180 |
  | 1   | 0.1 | 20.4 mm | 1.45° | 0.197 | 0.0249 |
  | (참고) W0 θ̇ 0.125, 400, 25 | | 19.9 mm | 0.61° | 0.147 | 0.0028 |
  wΔ 10 = 11-16 실차에서 떨림 없음이 확인된 값 (1 은 떨림). 가장 빠른 pole 8.9 Hz ζ 0.76 — 11-16 실차 판의 8.7 Hz ζ 0.78 과 같은 자리.
  wΔ 10·rU 0.1 + crossing_blend_m 0.3: 위치 23.4 mm, 요 1.45° → 0.83° (교차 창 최대 7.7° → 3.9°), 교차 창 밖 요는 0.37° 그대로.
- 바꾼 파일: `mpc/config/torque_mpc.yaml` tuning.W [5, 5, 2.5, 2, 2, 1], w_delta 10, r_u 0.1 (주석에 표).
- 확인: pytest 227 통과, 설치본 링크로 반영 확인. 실차 판 없음.

### 2026-11-18 (3) — MPC θ̇ 가중 0.5 → 0.125 (요 오차)
- 왜: 사용자 지시 "요 오차 줄이자". 같은 시뮬 조건(2026-11-18 (2))에서 요 RMS(주행, 참값 − 기준)를 W θ·θ̇ 와 crossing_blend_m 으로 비교.
  기본 0.82° → θ̇ 0.125 만 0.61° → crossing_blend_m 0.3 만 0.48° → 둘 다 0.37° (교차 창 최대 5.2° → 1.5°). 위치 RMS 19.4~20.1 mm 로 그대로,
  Δτ RMS 0.0033 → 0.0026. θ 가중 200 은 0.46° 에 Δτ 2 배라 뺐다. 교차 창 밖 요 RMS 는 어느 판이나 0.26~0.29° (추정기 요 오차 0.19°).
- 바꾼 파일: `mpc/config/torque_mpc.yaml` tuning.W 의 θ̇ 0.5 → 0.125 (주석에 표). x·y pole 그대로, θ pole −10.65·−17.0 ± 1.95j → −7.17 ± 6.82j (ζ 0.72).
  crossing_blend_m 은 기본(torque_track.yaml 0.0) 그대로 — 퓨어 퍼슛(A)은 v_y = 0 이라 헤딩 ≠ 진행 방향인 완화 기준을 따라갈 수 없어서, MPC 판에서만 인자로.
- 확인: pytest 227 통과. 실차 판 없음.

### 2026-11-18 (2) — d̂ 가속도 외란 모델 옵션 (`disturbance.accel_only`, 기본 끔) + 오차 원인 시뮬 (적분 작용·교차점·d̂ 필터·추정기)
- 왜: 사용자 질문 "적분 작용이 없어서 오차가 남나", "d̂+w 가 나빴던 건 저역통과 튜닝으로 해결되나", "PP 도 추정기 값을 받지 않나".
- 시뮬 조건: circles8 width 2.0, tangent, 0.7 m/s, C μ̂ 0.6, wΔ 400·rU 25, 참값 − 기준 주행 RMS. 판마다 차이 약 1 mm (같은 설정 19.0~20.0).
  기록 도구는 세션 scratchpad 의 compare_run 변형 (참값·기준·추정기 시계열 저장) — 리포에는 넣지 않았다.
  - 적분 작용: 무제약 이득으로 옆 외란 0.283 m/s² (원 위 w 항) 의 offset = DC 32.4 mm, ω 1.29 rad/s 에서 34.5 mm (등가 k_p 8.74 1/s²).
    w 끔 시뮬의 경로 좌표 bias 31.0 mm (안쪽) — 이론과 맞음. 11-16 값(wΔ 10, rU 0.01)은 3.4 mm: pole 맞춤으로 강성이 1/10 이 되어 w 항이 필요해졌다.
    rU = 0 이어도 offset 23.9 mm (Δu 꼴이 적분을 주지 않는다). w 켬이면 bias 는 원 위 평균제곱의 16 % 뿐.
  - crossing_blend_m 0 / 0.1 / 0.2 / 0.3: 위치 RMS 변화 없음 (19.0~20.0), 요 RMS 0.58° → 0.44° (0.2) → 0.37° (0.3).
    교차 창(−0.5~+1.5 s) 위치 오차는 같은 설정에서 19.8~27.6 mm 로 판마다 흔들린다. 요 가속 한계가 원인이라던 설명은 틀렸다.
  - d̂ α (w 켬, 추정기 상태): 끔 19.0 · α 0.02 26.0 · 0.05 29.3 · 0.1 29.4 · 0.3 29.6 mm. 참값 상태: w 만 8.5 → d̂+w 5.2 mm.
    → d̂ 구조(offset-free)는 맞고, 추정기 x̂ 로 만든 잔차가 문제. EWMA 는 DC 이득 1 이라 속도 보정 계단의 면적(Δv)을 α 와 상관없이 통과시킨다 (해석).
  - 가속도 외란 모델 (아래 옵션): α 0.1 두 판 29.2·29.4, α 0.02 26.8, α 0.3 29.4, 참값 5.2 mm — 1판 식-9 와 같다. 위치 잔차는 원인이 아니었다.
  - PP (A): 추정기 12.4 · 참값 11.0 mm (추정기 몫 +1.4). MPC w 켬: 추정기 19.0 · 참값 8.5 mm (+10.5). 추정기 오차(참값 − x̂) 는 둘 다 8.0 mm.
    PP 는 추정기 pose 만 쓰고 속도 루프는 엔코더(wheel PI). MPC 는 추정기 속도에 등가 k_d/k_p = 0.38 s (0.05 m/s ≈ 19 mm) 로 반응 —
    W 속도 가중을 0 으로 해도 0.34 s (예측 지평 위치 오차가 초기 속도를 본다).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`·`src/torque_mpc.cpp`: `TorqueTuning::dist_accel_only` — 건너뛰기 판단 뒤 잔차 위치 성분을
    ½Ts·(속도 잔차) 로 바꾼다 (일정 가속도의 ZOH 한 스텝, input disturbance model). 기본 false = 1판 식-9 그대로.
  - `mpc_cpp/src/torque_mpc_node.cpp`: 파라미터 `disturbance_accel_only` (yaml `disturbance.accel_only`), 기동 로그에 표시.
  - `mpc/config/torque_mpc.yaml`: `disturbance.accel_only: false`.
  - `mpc/launch/torque_experiment.launch.py`·`real_all.launch.py`: 인자 `mpc_dist_accel_only` (넘기기).
  - `mpc_cpp/test/test_torque_mpc.cpp`: `DisturbanceAccelOnly` — 1 N 은 식-9 수치 예와 같은 d̂, 위치만 0.5 mm 튄 잔차는 0, 일정한 힘 offset 제거.
- 확인: `amrbuild` 성공, gtest 28 개 통과, pytest 227 통과. 시뮬은 위 표. 실차 판 없음.

### 2026-11-18 (1) — MPC 입력 가중치만으로 비교군 A pole 에 맞춤 (wΔ 400, rU 25), 설치본을 소스 링크로 (`amrbuild`)
- 왜: 사용자 질문 "같은 오차에 PP 는 덜, MPC 는 더 공격적인 이유" → 11-17 의 pole 맞춤(W [5,…], rU 1)은 소스 yaml 에만 있고 빌드가 안 돼
  11-17 모든 실차 판이 예전 W [100,100,50,1,1,0.5]·wΔ 10·rU 0.01 로 돌았다 (bag run_info params 로 확인). 그 값의 x·y pole −6.96·−15.7 ± 14.7j 는
  A(x −1.11 / y −1.58 ± 2.49j) 의 4~6 배 빠르다. 소스에 남아 있던 wΔ 1e6·rU 1e4 (11-17 18:45) 는 pole −0.01 ± 0.11j 로 되먹임이 거의 없다.
  사용자 지시: "입력쪽만 건드려서" 맞춤.
- 바꾼 파일:
  - `mpc/config/torque_mpc.yaml`: W 원래값으로 되돌림, w_delta 400, r_u 25 (주석에 pole 표). 같은 방법(무제약, θ_r = 0 직선, m_w 포함 —
    K_x = −[H⁻¹ΦᵀQ̄F]₀:₄, K_τ = wΔ[H⁻¹]₀:₄,₀:₄ 의 증강 closed loop, 11-17 주석의 '예전'·'지금' 행을 재현해 검산)으로 wΔ 10⁰~10⁶ × rU 10⁻²~10⁴ 격자 탐색:
    x·y −1.66 ± 2.64j (ζ 0.53, A y 와 같음), θ −10.65·−17.0 ± 1.95j (A −15.2 ± 3.1j), 영공간 −21.1. 사용자 제안 wΔ 1e4·rU 100 은
    x·y −1.46 ± 2.68j 로 맞지만 θ −2.76 ± 3.21j (5 배 느림). 입력만으로는 θ ω_n 이 x·y ω_n 2~4 에서 ≈ 11.8 이 한계 (A 15.5).
  - `mpc/tools/amr_aliases.sh`: `amrbuild` — `PYTHONNOUSERSITE=1 colcon build --symlink-install`. ~/.local 의 setuptools 83 에
    'develop --editable' 이 없어 colcon 이 symlink-install 을 말없이 복사 설치로 바꾸던 것 (colcon_core/task/python/build.py 105–117).
    시스템 setuptools 68 로 빌드하면 share/config·launch·rviz 와 Python 패키지가 소스 링크 → yaml·launch·Python 은 고치면 다음 실행부터 반영,
    C++·새 파일만 빌드. 링크 없이 빌드해도 복사가 링크를 먼저 지워 소스는 안 깨진다 (setuptools _distutils file_util).
  - `mpc/test/test_can_setup.py`: 가짜 launch 인자에 `controller` — 11-17 `79ce6b1` (ramp·step) 부터 CAN 준비 전에 읽어 KeyError 로 실패하던 것.
- 확인: Python 시험 227 개 통과. `ros2 run amr_mpc torque_output_node` 기동 (import 경로 = 소스). 시뮬 (도메인 95, circles8 폭 2 m, tangent, 0.7 m/s,
  C μ̂ 0.6, 한 판씩): PP RMS 9.4 / 최대 39.2 mm, τ RMS 0.047, Δτ RMS 0.0038 | **MPC 새 가중치 27.4 / 68.7 mm**, τ RMS 0.047, Δτ RMS 0.0033 |
  MPC 예전 11.4 / 32.2 mm, τ RMS 0.070, Δτ RMS 0.0179. → 토크 크기·떨림은 A 와 같아졌지만 **추종은 A 의 3 배로 나빠졌다** (외란 추정치 d̂ 끔 —
  A 는 바퀴 PI 의 적분과 엔코더 안쪽 루프가 모델 오차를 지운다. pole 은 추종 동역학만 맞추고 외란 억제는 안 맞춘다). 실차 미확인.

### 2026-11-16 (18) — 잘못된 yaw_mode·shape 을 launch 에서 거부, circles8 tangent 교차점 헤딩 완화 `crossing_blend_m`
- 왜: 접선 판 분석 (`analysis/torque_mpc_real/2026-11-16_circles8/tools/crossing.py`·`halves.py`).
  ① `yaw_mode:=true` 로 띄운 판 6 개(142227, 185214, 185603, 185707, 185814, 185918)에서 figure8_reference_node 가 시작하자마자
  ValueError 로 죽었는데 오류가 화면에만 나오고 /rosout·bag 에는 안 남아, 무장해도 기준 없이 서 있었다.
  ② circles8 tangent 는 교차점에서 곡률 +1/R → −1/R 계단 → 기준 ω 가 +v/R → −v/R 로 한 번에 뒤집힌다 (요 가속 충격).
  실차 0.7 m/s (μ̂ 0.4): 교차점 ±0.6 s 위치 RMS 10~12 cm (나머지 5.6~6.3), 어느 바퀴든 상한 닿음 95~100 %, 헤딩 최대 3.4~5.7°.
- 바꾼 파일 (`controller/mpc`):
  - `launch/torque_experiment.launch.py`: `_track()` 이 yaw_mode(tangent|fixed)·shape(figure8|circles8|line|rotate)·origin_mode(robot|fixed)
    허용값을 검사해 `RuntimeError` — `_nodes` 맨 앞(실차 CAN 준비 전)에서 먼저 부른다. 트랙 키에 `crossing_blend_m`.
  - `amr_mpc/core/figure8.py`: `Figure8(crossing_blend=Lb)` (circles + tangent 만, 기본 0 = 예전 그대로). 경로(위치)는 그대로 두고 교차점
    앞뒤 Lb/2 에서 헤딩 곡률을 σ/R → −σ/R 직선으로 → 요 가속 최대 2v²/(R·Lb), 헤딩이 접선보다 최대 Lb/(4R) 덜 돎(그 구간 메카넘 게걸음).
    출발·끝 교차점(속도 0)은 건드리지 않음. 여러 바퀴면 안쪽 교차점 모두.
  - `amr_mpc/nodes/figure8_reference.py`: 파라미터 `crossing_blend_m`, 기동 요약에 완화(요 가속·헤딩) 또는 "교차점 ω 계단" 한 줄.
  - `config/torque_track.yaml`: `crossing_blend_m: 0.0`. `launch/real_all.launch.py`: `_PASS` 에 `crossing_blend_m`.
  - 시험: `test_figure8.py` 3 개 (0 = 예전과 같음, Lb 0.2 에서 요 가속 9.8·헤딩 5.73°·경로 같음·유한차분, circles+tangent 만·안쪽 교차점),
    `test_launch_modes.py` 7 개 (잘못된 값 거부, 맞는 값 통과, 실차에서 CAN 준비 전 거부 — prepare_can 은 가로챔),
    `test_can_setup.py` 가짜 launch 인자에 use_mpc·use_figure8.
- 확인: Python 시험 227 개 통과. 수치 검산 (R 0.5, v 0.7, Lb 0.2): ω 의 1 ms 변화 2.800 → 0.0098 rad/s, 요 가속 최대 9.80 (해석 9.80),
  헤딩 차 최대 5.73° (해석 5.73°), 위치 경로 차 0, 유한차분 |dyaw/dt − ω| 1.8e-3 rad/s·|dp/dt − R(yaw)u| 4.8e-4 m/s.
  시뮬 (도메인 96, circles8 tangent 0.7 μ̂ 0.4 use_w, 2 판씩): 완화 0 RMS 11.2·11.4 / 최대 27.1·28.0 mm / yaw RMS 0.68·0.69°,
  완화 0.2 m 13.7·11.6 / 27.7·34.8 mm / 0.51·0.56° → 헤딩 오차 −22 %, 위치는 흔들림 안. **실차 미확인** (교차점 포화가 실차만큼 안 나는 시뮬).
  `python3 amr_mpc/core/figure8.py` 자기검사는 HEAD 부터 `from mpc import K_GEOM` 로 깨져 있다 (이번 변경과 무관, 안 고침).

### 2026-11-16 (17) — 구동계 마찰 보상표를 오늘 실측으로 (11-14 값은 네 바퀴 모두 과보상)
- 왜: 접선 판에서 "FR 마찰 2 배" 로 보였던 것을 다시 잼 (`analysis/torque_mpc_real/2026-11-16_jw/RESULTS_KO.md` 끝 절). 바퀴 띄움·38~39 °C 에서
  코스트다운(토크 0 감속)과 전압 한계 아래 정상 상태(0.045/0.04/0.035/0.03 N·m) — FR 은 같은 속도에 0.003~0.005 N·m 더 들 뿐(2 배 아님, 앞선 결론 틀림).
  대신 11-14 표가 20 rad/s 에서 네 바퀴 모두 0.007~0.018 N·m 과보상 (τ̄ 0.1012 의 7~18 %, 점성 항 1.5~1.9 배) — 바닥 직선 판 순항에서
  MPC 순토크가 음수(제동)였던 것과 같은 방향. 사용자 결정: 오늘 값으로.
- 바꾼 파일: `mpc/config/torque_plant.yaml` friction `coulomb_Nm` [0.02142, 0.02151, 0.02170, 0.01830] → [0.0174, 0.0206, 0.0155, 0.0157],
  `viscous_Nms_per_rad` [0.002062, 0.001799, 0.001767, 0.001679] → [0.001066, 0.001158, 0.001112, 0.001177] (compensation_scale 0.9 그대로),
  근거·모르는 것(온도, 26 rad/s 위 외삽, 바닥 하중) 주석. 이 표는 시뮬 플랜트의 실제 마찰로도 쓰인다.
  `mpc/test/test_torque_output.py` 시험 2 개를 새 값으로 (정격 속도 최대 마찰 0.10132 → 0.06547, 여유 0.1 이면 경고 없음·0.05 면 경고).
- 확인: Python 시험 통과 (위 227 개에 포함). 시뮬 B 기본 8자 13.64 mm·최대 39.0 (예전 11.7~13.9). **실차 미확인** —
  `real_robot_checklist.md` "마찰 보상표 갱신 뒤 다시 보기".
- 되돌리려면: 위 괄호 안 11-14 값.

### 2026-11-16 (16) — 바퀴 축관성 J_w 실측 (계단 차분): 8.77e-4 → 설정 8.9e-4 그대로, 측정 도구 순서·요약 고침
- 왜: (13) 의 도구로 실차(장착 그대로, 바퀴 띄움) J_w 를 잼. 결과는 `analysis/torque_mpc_real/2026-11-16_jw/RESULTS_KO.md`.
- 결과: 네 바퀴 121 계단 중앙 **8.77e-4 kg·m²** (사분위 6.6~10.5, 평균 8.58 ± 0.35 표준오차) — 설정 8.9e-4 와 −1.4 % → **설정 안 바꿈**.
  바퀴별 중앙 FL 9.76 / FR 9.35 / RL 6.81 (저속 다시 7.79) / RR 9.12 ×10⁻⁴. RL 만 낮은 이유는 모름 (전압 한계·기록 누락 아님).
- 바꾼 파일:
  - `mpc/tools/jw_step_analysis.py`: CLI `--w_max` (기본 30 rad/s — 0.08 N·m 에서 약 40 rad/s 로 전압 한계에 닿아 응답 토크가 0.010 낮았다),
    요약을 중앙값·사분위 대표로 (실측 속도 잡음 0.3~0.65 rad/s 라 계단 하나가 ±30 % 흩어짐), 설명의 측정 순서를 실측 순서로
    (워밍업 `steps --taus 0.09,0.05 --hold 5`, 계단 `0.08,(0.035,0.055)×4,0`), 검증 숫자를 지수 꼴 맞춤 값으로.
  - `mpc/config/torque_plant.yaml`: `inertia_J_w_kgm2` 주석에 실측 확인 (값 그대로).
  - `amr/notes_amr/real_robot_checklist.md`: J_w 항목 완료, RL 차이는 하위 항목으로.
- 확인: `test/test_jw_step_analysis.py` 5 개 통과. 첫 순서(0.1, 0.05/0.08)로 잰 FL 판은 버림 (`hi_jw_1_*`, 결과 문서에 이유).

### 2026-11-16 (15) — 시뮬 플랜트 기본도 바퀴 관성 켬 (`model.use_wheel_inertia: true`)
- 왜: (14) 로 제어기는 M 에 m_w 를 늘 넣는데 시뮬 플랜트 기본은 M = m 이라 둘이 달랐다. 사용자 결정: 플랜트도 실차처럼 바퀴 관성이 있는 물리로
  (지시서 7.3 "플랜트는 w 를 포함한 식-5"). 이제 기본 시뮬은 플랜트 M = m_eq + w, 제어기 M = m_eq·w 끔 — w 항만 다르다.
- 바꾼 파일: `mpc/config/torque_plant.yaml` `model.use_wheel_inertia: false → true`, 주석.
  영향은 슬립 없는 시뮬 플랜트(`sim_plant_torque`, `tools/baseline_a_offline.py`, `tools/torque_demo_drive.py`) 뿐 — 다른 Python 노드는
  Tinv·토크 한계·몸체 질량만 쓴다. 슬립 모델(`surface_mu > 0`)은 몸체에 m·I_z, 바퀴에 J_w 를 따로 두므로 이 값과 상관없다 (이중 반영 없음).
- 확인: Python 시험 216 개 통과. 시뮬 기본 8자 (도메인 95, 한 판씩): **B 11.67 mm·최대 31.5** (예전 12.0~13.9), **A 4.72 mm·최대 14.7** (예전 5.1~6.3). 둘 다 완주.
- 되돌리려면: launch 인자 `use_wheel_inertia:=false` (플랜트만).

### 2026-11-16 (14) — w 항·외란 추정치 지시서 1·2 번만: 제어기 M 에 m_w 늘, `use_w` 로 w 항만, d̂ 기본 끔
- 왜: 교수님 지시서(2026-11-16) "w 항과 외란 추정치를 구현하되 끈 채로 제어". 지시서와 지금 코드의 차이 6 개 중 **사용자 결정: 1·2 번만 구현,
  3(α 0.1 유지)·4(실행 중 전환 없음)·5(status 기록 그대로)·6(수치는 실측값 유지) 은 현행 유지.**
  1 번 = 지시서는 M = diag(m_eq, m_eq, I_eq) 고정에 w 만 스위치, 코드는 `use_wheel_inertia` 하나가 M(m ↔ m_eq) 과 w 를 같이 껐다 켰다.
  2 번 = 지시서는 d̂ 기본 끔, 코드는 같은 날 앞선 결정으로 켬(`enable: true`) — **오늘 실차 판 135118~144233 은 d̂ 켬(α 0.1)**.
  지시서의 회귀 기준("두 스위치 끈 출력 = 지금 출력")은 1 번 때문에 성립하지 않는다: 끈 상태 모델이 M = m → m_eq 로 바뀐다
  (선회 목표 입력 4 kg 예 ±0.01667 → ±0.02677 N·m, 지금 설정 4.56 kg 이면 ±0.0190 → ±0.0283).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`·`src/torque_mpc.cpp`: `TorqueModel::use_wheel_inertia` → `use_w`. `mu()` = 4J_w/r² 늘
    (M_eff·I_eff 에 늘 들어감). `w()` 는 `use_w` 가 꺼지면 계산 없이 0 (0 을 곱하지 않음), `w_value()` 는 스위치와 상관없는 값.
    current_state 선형화의 ∂w/∂Ẋ 도 `use_w` 일 때만. 예측(식-13)·목표 입력(식-14)·d̂ 잔차(식-9) 모두 같은 `w()` 를 써서 s_w 가 세 곳에 같이 걸린다.
  - `mpc_cpp/src/torque_mpc_node.cpp`: 파라미터 `use_wheel_inertia` → `use_w` (기본 `torque_mpc.yaml model_terms.use_w`). 기동 로그
    "바퀴 등가 질량 m_w 2.225 kg (M 에 늘)  w 항 끔/켬".
  - `mpc/config/torque_mpc.yaml`: `model_terms.use_w: false` (새 절), `disturbance.enable: true → false`.
  - `mpc/config/torque_plant.yaml`: `model.use_wheel_inertia` 주석 — 이제 시뮬 플랜트 물리만. **기본 false 라 시뮬 플랜트 M = m, 제어기 M = m_eq 로 다르다.**
  - `mpc/launch/torque_experiment.launch.py`: `use_wheel_inertia` 는 플랜트에만, `ctrl_wheel_inertia` 를 없애고 `use_w` (제어기 w 항).
  - `mpc/launch/real_all.launch.py`: `_PASS` 의 `use_wheel_inertia` → `use_w` (실차에는 플랜트가 없다), docstring.
  - `mpc_cpp/test/test_torque_mpc.cpp`: 시험 W1(`TorqueModel.WTermSwitch`: w = [−0.1574, 0, 0], 끄면 0, M 은 늘 6.425) 추가,
    U10·통과시험 1·current_state 선회의 기대 토크를 use_w 켬 m·a_c / 끔 m_eq·a_c (W3: 0.01667 / 0.02677) 로, 나머지 관성 스위치를 `use_w` 로.
    시험 안의 수치는 지시서 값(4 kg, J_w 9.7e-4)을 구조체 기본값으로 그대로 쓴다.
  - `mpc/test/test_launch_modes.py`: 스위치 시험을 새 의미로 (`use_wheel_inertia` 는 플랜트만, 제어기에는 `use_w`), real_all 전달 목록.
- 확인: `amr_mpc_cpp`·`amr_mpc` 빌드, C++ 시험 38 개(토크 27·바퀴 11)·Python 216 개 통과.
  시뮬 B 기본 8자 (도메인 94, 한 판씩, 판 사이 흔들림 약 ±0.9 mm — 2026-11-16 같은 조건 반복 14.53/15.47):
  | 플랜트 | 제어기 | RMS / 최대 mm | Δτ RMS |
  |---|---|---|---|
  | M = m (기본) | m_eq, w 끔, d̂ 끔 (새 기본) | 13.06 / 36.2 | 0.021 |
  | m_eq + w (`use_wheel_inertia:=true`) | m_eq, w 끔 | 12.94 / 37.0 | 0.019 |
  | m_eq + w | m_eq, w 켬 (플랜트 = 모델) | 12.80 / 35.7 | 0.019 |
  | M = m | m_eq, w 끔, **d̂ 켬** | 15.26 / 39.6 | 0.029 |
  처음 세 줄은 흔들림 안. 시뮬에는 일정한 외란이 없어 d̂ 를 켜면 잡음만 더한다 (같은 날 옛 코드 d̂ 켬 B 13.35~13.89). 실차 미확인.
- 되돌리려면: `disturbance:=true` (d̂). 옛 "M = m" 모델은 스위치가 없다 — J_w 를 0 으로 두면 같다.

### 2026-11-16 (13) — 바퀴 축관성 J_w 계단 차분 측정 도구 `tools/jw_step_analysis.py`, 벤치 CSV 에 명령 토크 열
- 왜: J_w 8.9e-4(팀원 11-12 벤치)가 맞는지 물음. 있는 자료로 구하면 방법마다 다르다 (모두 명령 토크 기준):
  | 방법 | 자료 | J_w [kg·m²] |
  |---|---|---|
  | 팀원 적분 맞춤 τ = J·ω̇ + τc·sign + bv·ω | 11-12 띄움 ≤ 10 rad/s | 8.65~9.18e-4 |
  | 같은 꼴 회귀 (ω > 2 rad/s) | 11-14 내림 계단 | 6.05~6.94e-4 |
  | 계단 뒤 시정수 J = b_loc·T (두 속도 사이 마찰 직선) | 11-14 내림 계단 | 1.04e-3 ± 0.12e-3 |
  | 바닥 Σ(τ−τ_f)/r = m·a_IMU + μ·a_바퀴 (±0.10~0.20 N·m) | 11-15 basic2 | 6.0e-4 (μ 1.51 ± 0.14 kg) |
  흩어짐(1.7 배)의 원인은 마찰 모델이 설명 못 하는 토크가 J 로 잡히는 것 → 마찰 모양을 가정하지 않는 측정이 필요.
- 바꾼 파일 (`controller/mpc`):
  - `tools/jw_step_analysis.py` (새): 돌고 있는 중 토크 계단 순간 t_s 의 앞뒤 ω̇ 차 → J = Δτ/Δω̇ (같은 ω 라 τ_f 소거).
    ω̇ 는 앞뒤 window 0.15 s 를 ω̇ = α − β·ω 꼴(지수, β = b_loc/J 앞뒤 공유, 절편·기울기 따로)로 맞춰 t_s 에서 구한다.
    |ω| < 3 rad/s(정지 마찰·0 지남)·> 39 rad/s(전압 한계) 계단은 뺀다. 옛 CSV 는 `--taus --hold` 로 계단 순서를 되살린다.
  - `tools/ak40_bench.py`: `--csv` 끝에 `tau_cmd` 열 (MIT 명령 토크 항). 기존 열 순서는 그대로 (`analyze_ak40_logs.py` 는 열 이름으로 읽음). 사용 예 한 줄.
  - `test/test_jw_step_analysis.py` (새, 시험 5 개): 해석해 합성 자료로 J 1.0e-3·8.9e-4·6.0e-4 를 4 % 안에서, 정지·전압 한계 계단 빼기, 옛 CSV.
- 확인:
  - 합성 벤치 자료 (Stribeck 정지 마찰 0.08·워밍업 15 %·명령 지연 5 ms·속도 양자화 0.022 rad/s·100 Hz ± 1 ms, 3 판):
    제안 순서 `0.1,(0.05,0.08)×4,0` hold 1.0 에서 정답 1.0e-3 → +0.4 %, 8.9e-4 → +0.4 %, 6.0e-4 → +0.3 % (쓴 계단 27·27·24/27).
    감속기 부하 의존 마찰 10 % 를 넣으면 +11 % (1/(1 − 0.1) 예상과 같음) — 이 방법이 못 지우는 오차.
    처음 쓴 2 차식 맞춤은 +1.7~+3.4 % 편향이라 지수 꼴로 바꿨다.
  - 11-14 내림 계단에 적용: 3.7e-4 ~ 2.3e-3 로 흩어짐 — 쓸 수 있는 계단 10 개, Δτ 0.01~0.04, 계단 시각을 일정표로 되살린 탓. 새 측정이 필요하다.
  - Python 시험 216 개 통과. **실차 측정 미실시** — `real_robot_checklist.md` 2절 "바퀴 축관성 J_w 다시 재기".

### 2026-11-16 (12) — 바닥 μ̂ 토크 램프 시험 도구 `tools/mu_ramp_test.py` (`amrmu`)
- 왜: C 의 상한 τ̄ = (r/√2)·μ̂·(mg/4)·c_s 와 곡선 최고 속도(v_max ∝ √μ̂)가 μ̂ 하나에 걸려 있는데 0.3 은 추정값 (To do.md "노면마다 토크 램프 시험").
  기존 데이터로 먼저 본 값 (bag `2026-11-15_lidar/basic2`, 오늘 세션 계산): 가속 0.2 N·m 에서 몸체 2.29 m/s² 를 미끄럼 없이 → μ̂ ≥ √2·a/g = 0.33,
  DAMPING 제동 중 바퀴가 몸체보다 느린 구간의 감속 → 미끄럼 마찰 0.37~0.45 (0.15·0.2 판).
- 도구: 방향(fwd·back·left·right·fl·fr·bl·br) 최소 노름 토크를 2 N·m/s 로 올리다가, 바퀴 기구학 속도 − IMU 적분 몸체 속도가
  0.12 + 0.15·|v| m/s 를 50 ms 넘으면 그 토크를 0.3 s 유지 → 유지 중 차가 0.05 m/s 넘게 더 벌어지면 슬립 확인, 그 구간 IMU 몸체 가속 a 로
  μ̂ = max_i √2·|τ_g,i|/r / (mg/4), τ_g = pinv((T⁻¹)ᵀ)·[m a; 0] (식-21·시뮬 SlipPlant 의 45° 롤러 모델. 앞·옆 √2a/g, 대각 2a/g).
  미끄러지는 동안 값이라 정지 마찰 최대값보다 작거나 같다 (보수 쪽). 슬립 없이 최대 토크·안전 속도(1 m/s)면 하한. `--no-prompt` 는 시뮬 전용.
- 바꾼 파일 (`controller/mpc`): `tools/mu_ramp_test.py` 새 파일, `tools/amr_aliases.sh` `amrmu` (+ amrhelp 범위), `test/test_mu_ramp.py` 시험 7 개
  (μ̂ 규약을 SlipPlant 기하로 검산 — 앞·옆·대각, 판정·확인·하한 분기).
- 확인: Python 시험 211 개 통과. 시뮬 (도메인 93, `plant:=sim use_mpc:=false surface_mu:=…`, 방향마다 3 판):
  - 같은 유지 구간을 **참값 속도**로 계산하면 설정 μ 0.25 → 0.244~0.247 — 판정·유지 구간·롤러 규약은 맞다.
  - IMU 로는 판마다 흩어진다: μ 0.25 앞 0.206~0.257 (중앙 0.226), 옆 0.117~0.216 / μ 0.4 앞·옆·대각 0.343~0.459 (중앙 0.41~0.43).
    시뮬 IMU 잡음(흰잡음 σ 0.95 m/s² + 진동)이 실차(움직일 때 고주파 σ 0.3~0.7)보다 크다. 고속 거짓 판정은 확인 단계가 모두 걸렀다.
  - → 방향마다 5 판 이상 중앙값, 앞·뒤를 주 값으로. **실차 미확인.**

### 2026-11-16 (11) — 추종 오차 정지 문턱 0.30 → 1.5 m, figure8 abort 0.35 → 1.6 m
- 왜: 사용자 결정. C(마찰 상한)가 물리 한계를 넘는 기준(예: circles8 0.7 m/s, μ̂ 0.3 에서 C 한계 ≈ 0.63 m/s)에서 "미끄러지는 대신 늦어지는"
  모습을 끝까지 기록하려고. 30 cm 에서 서면 그 전에 끊긴다.
- 바꾼 파일 (`controller/mpc`):
  - `launch/torque_experiment.launch.py`: 인자 `max_track_error` 기본 "0.30" → "1.5" (실제로 쓰는 값. `amrall` 도 이 기본값)
  - `amr_mpc/nodes/fault_monitor.py`: 노드 기본값 1.5, 머리 주석
  - `config/torque_track.yaml`·`amr_mpc/nodes/figure8_reference.py`: `abort_error` 1.6 — 경성 중단이 연성 정지(fault_monitor)보다 뒤에 걸리는 순서 유지.
    그대로 0.35 였으면 1.5 m 로 올린 효과가 없다 (figure8 이 먼저 기준을 끊는다)
  - `launch/real_all.launch.py`: 주석의 기본값 표기
  - `test/test_launch_modes.py`: 1.5 < 1.6 순서 시험 1 개
- 확인: Python 시험 204 개 통과, 빌드. 실차 미확인.
- 주의: 로봇이 경로에서 1.5 m 까지 벗어나도 안 선다 — 두 사람 정지 절차. 예전처럼: `amrall max_track_error:=0.30` (abort 는 torque_track.yaml).
  옛 속도 MPC launch(`launch/figure8.launch.py`)의 abort_error 0.35 는 그대로.

### 2026-11-16 (10) — 바퀴 등가 질량 스위치를 실차에서 켤 수 있게 (`amrall use_wheel_inertia:=true`), 제어기 전용 `ctrl_wheel_inertia`
- 왜: 발표(1판 식-3·5)는 바퀴 등가 질량 m_w = 4J_w/r² 를 모델에 넣는데 구현은 10/29 "휠 관성 무시" 결정으로 끔(`model.use_wheel_inertia: false`).
  실측 J_w 8.9e-4 로 m_w = 2.225 kg (m 4.56 의 49 %) → 모델 관성은 실제의 병진 67 %, 요 41 % (I_eq = 0.06 + 2.225·0.19835² = 0.1475).
  5-2 개루프 가속을 실측 구동계 마찰과 함께 적분하면 (오늘 세션 계산, 맞춘 값 없음): 앞 0.15 N·m × 0.5 s 실차 1.50~1.59 m/s² ↔
  관성 켬 1.62 (지금 모델 2.23, 지금 모델 × 맞춤 η_x 0.65 = 1.57); 회전 0.15 N·m × 0.25 s 실차 14.5 rad/s² ↔ 관성 켬 15.0 (지금 30.1, × η_ψ 0.40 = 14.8).
  → 2026-11-16 (2) 의 "바닥 효율" 앞 0.65·회전 0.40 은 관성 비 0.672·0.407 과 같다 (회전 쪽은 I_z 추정값 0.06 이라 약한 증거).
  옆 η_y 0.40 은 0.40/0.672 = 0.60 이 관성으로 설명 안 되는 옆걸음 몫으로 남는다.
- 바꾼 파일 (`controller/mpc`):
  - `launch/real_all.launch.py`: `_PASS` 에 `use_wheel_inertia`, docstring 한 줄
  - `launch/torque_experiment.launch.py`: 인자 `ctrl_wheel_inertia` (제어기만 덮어씀, 비우면 `use_wheel_inertia` 와 같게 — `ctrl_payload_kg` 와 같은 꼴).
    시뮬에서 지금 실차 흉내: `use_wheel_inertia:=true ctrl_wheel_inertia:=false`
  - `test/test_launch_modes.py`: 시험 5 개 (스위치 4 경우, real_all 전달)
- 확인: Python 시험 203 개 통과. 시뮬 (도메인 92, `run_compare.sh` NUC 사본, 한 판씩):

  | 조건 | 플랜트 관성 | 제어기 관성 | RMS / 최대 mm | 결과 |
  |---|---|---|---|---|
  | C μ̂ 0.3, circles8 yaw fixed 0.3 m/s, `floor_eff:=1.0,0.6,1.0` | 끔 | 끔 | 14.4 / 46.1 | 완주 |
  | 〃 (지금 실차 흉내) | 켬 | 끔 | 16.1 / 52.1 | 완주 |
  | 〃 (제안) | 켬 | 켬 | 17.5 / 56.9 | 완주 |
  | 같은 조건 0.7 m/s, 세 경우 모두 | | | 105~111 / 296~318 | 30 cm track_error 정지 (옛 문턱) |
  | B 기본 8자 | 켬 | 켬 | 15.3 / 41.1 | 오늘 끔 13.89 / 35.3 (2026-11-16 (9)) |

  **시뮬에서는 제어기 관성을 켜도 추종이 좋아지지 않았다** (0.3 m/s 16.1 → 17.5 mm, 같은 조건 두 판 흔들림 0.1 mm 수준이라 우연은 아님).
  33 % 관성 오차는 d̂·되먹임이 흡수한다. 지금 튜닝(W, w_delta 10)이 관성 끈 모델로 맞춘 것이라는 점은 미확인. → **기본값은 false 유지**,
  실차 A/B(`amrall use_wheel_inertia:=true`)로 정한다.
- 실차 미확인.

### 2026-11-16 (9) — 정지 마찰 보상 (기준 속도 부호 + Stribeck)
- 왜: 오늘 실차 circles8(`yaw_mode:=fixed`, `limit_mode:=friction`, μ̂ 0.3 → τ̄ 0.0759 N·m) 7 판 중 3 판이 `fault_monitor` track_error(30 cm)로 섰다.
  0.3 m/s 판 142534: 이동 방향이 45° 를 지날 때 FL·RR 이 역회전해야 하는데(ω = (vx − vy ∓ kω_z)/r), 바퀴가 0 에 먼저 서서
  1.78 s 동안 ω = 0.0 ± 0.1 rad/s 로 붙어 있었다 (필요 −0.9 → −9.2 rad/s). 그동안 모터 명령은 평균 1.00 τ̄ — 마찰 보상
  coulomb·tanh(ω/ω_s) 가 ω = 0 에서 0 이라 τ̄ 만 나갔고 출발 토크(벤치 0.04~0.1, 바닥 ≈ 0.09)를 못 넘었다. 같은 조건 142932 는
  같은 자리에서 0.32 s 붙었다 풀려 완주, 142743 은 관성으로 0 을 지나쳐 완주. 추정기−SLAM 위치 차 RMS 0.6 cm·CAN fresh 정상·
  QP fallback 0 이라 다른 원인은 아니다. 0.7 m/s 판(142402·143319)은 여기에 네 바퀴 토크 상한(1.3~1.6 τ̄, 보상 포함)이 겹쳤다 — 이 변경으로는 안 풀린다.
- 바꾼 파일:
  - `mpc/amr_mpc/nodes/torque_output.py`: 보상에 정지 항 τ_static = F_s·exp(−(ω/ω_st)²)·tanh(ω_ref/ω_s) 추가.
    ω_ref = T⁻¹·[vx, vy, ω_z]_ref (기준열 `/dra_mppi/reference_trajectory` 의 지금 시각 값, 몸체프레임, 행 사이 선형 보간 —
    `reference_wheel_speed()`). 기준열이 0.5 s 넘게 묵었거나 지평 끝이면 ω_ref = 0, `/wheel_states` 가 끊기면 정지 항을 끈다.
    파라미터 `stiction_comp`·`static_Nm`·`stribeck_rad_s`·`reference_topic`·`reference_timeout`. 기준 속도 기반 friction
    feedforward (Armstrong-Hélouvry·Dupont·Canudas de Wit 1994, Automatica 30(7)). 바퀴가 돌면(|ω| > 1 rad/s) 정지 항 ≤ 0.072·e⁻⁴ = 1.3 mN·m.
  - `mpc/config/torque_plant.yaml`: `friction.static_Nm: [0.08]×4` (× compensation_scale 0.9 = 0.072), `stribeck_rad_s: 0.5`. 근거 주석.
    0.072 는 출발 토크(≈ 0.09)보다 작아 보상만으로는 바퀴를 안 돌리고, −τ̄ 와 더하면 0.148 로 넘는다.
  - `mpc/launch/torque_experiment.launch.py`: 인자 `stiction_comp` (auto | true | false). auto = `plant:=real` 에서만 켬 — 시뮬 플랜트에는 정지 마찰이 없다.
  - `mpc/launch/real_all.launch.py`: `_PASS` 에 `stiction_comp` (→ `amrall` 기본으로 켜짐, `amrall stiction_comp:=false` 로 끔). docstring 한 줄.
  - `mpc/test/test_torque_output.py` 시험 7 개, `mpc/test/test_launch_modes.py` 시험 5 개.
- 확인:
  - Python 시험 198 개 통과. `colcon build --symlink-install --packages-select amr_mpc`.
  - bag 재적용 (142534 의 실제 τ_net·ω·기준열로 새 식 계산): 붙은 구간(기준 출발 후 1.81~3.61 s) FL·RR 모터 명령 −0.069~−0.081 → **−0.132~−0.149 N·m**.
    같은 입력에서 정지 항 크기: |ω| > 1 rad/s 최대 1.0 mN·m, |ω| ≤ 1 최대 72.0 mN·m. 재생 충실도(예전 식 재계산 − bag 출력) 중앙 0, 99 % 3.6 mN·m.
  - 시뮬 (도메인 91, `run_compare.sh` 의 NUC 사본, 정지 마찰 없는 플랜트라 "해가 없는지" 만 본다):
    B 끔 RMS 13.89·최대 35.3 mm / 켬 13.35·35.9 mm, C 끔 14.48·36.4 mm / 켬 14.43·40.7, 14.50·36.5 mm (C 켬 첫 판은 무장 약 2 s 뒤 시뮬 노드
    0.33 s 멈춤 timeout — run_compare.sh 주석의 알려진 경우, 다시 돌린 두 판은 정상). 판 사이 흔들림 범위 안.
  - **실차 미확인.** 확인 항목은 `real_robot_checklist.md` "정지 마찰 보상 실차 확인".
- 알려진 성질: 서 있는 바퀴는 기준 방향으로 기운다 — ω_ref > 0 이면 MPC 가 −τ̄ 를 내도 명령은 −0.0759 + 0.072 = −0.004 라 반대로는 못 돌린다.
- 되돌리려면: `stiction_comp:=false` (amrall 도 같음), 또는 `friction.static_Nm` 을 0.

### 2026-11-16 (8) — 녹화에 CAN·CPU·온도·로그 추가, Cartographer 표시용 토픽 제외, MPC status 에 hold·stopped·U_s
- 왜: 오후 디버깅에서 bag 만으로 못 본 것 — Cartographer 최적화 시각(SLAM 약 10 cm 튐, Cartographer 로그는 /rosout 에 안 나온다),
  RL·RR CAN 응답 누락 증가(RR 4 → 35 회/판), "상태 주기 ≠ Ts" 가 부하 탓인지, hold 들어간·풀린 시각(로그 문자열에만 있었다).
  그리고 bag 의 약 60 % 가 Cartographer 표시용 `/trajectory_node_list` 였다 (142932: 396 MiB 중 240, 135202: 1.3 GiB 중 821).
- 바꾼 파일:
  - `mpc/tools/record_all.sh`: 녹화기를 exec 하지 않고 자식으로 돌리고 끝날 때 처리. `run_info/` 에 `can_link_start/end.txt`
    (`ip -s -d link show can0`), `candump.log` (`candump -L`), `mpstat.txt`·`pidstat.txt`·`thermal.csv` (1 s), `ros_log/`
    (녹화 중 바뀐 `~/.ros/log` 노드 로그·launch 로그). 녹화 중 파일은 `<폴더>.run_info.tmp` 에 쓰고 끝에 옮긴다.
    제외 정규식에 `/trajectory_node_list`·`/landmark_poses_list`·`/constraint_list`.
    녹화기는 `setsid` + SIGINT 를 SIG_DFL 로 되돌려 실행, 끝낼 때 SIGINT 한 번 → 5 s 안에 안 끝나면 SIGTERM.
  - `mpc_cpp/src/torque_mpc_node.cpp`: `/torque_mpc/status` 뒤에 6 칸 [hold, stopped, U_s FL, FR, RL, RR] (앞 20 칸 그대로 —
    fault_monitor 는 [1]·[2] 만 본다). hold 주기에는 τ·U_s = 0.
  - `mpc/tools/analyze_run.py`: 새 칸이 있으면 hold 구간, 추종 중 stopped 비율, τ − U_s RMS. 없는 옛 bag 은 NaN 으로 건너뛴다.
  - `mpc/tools/amr_aliases.sh`: amrrec 도움말.
- 확인: `amr_mpc_cpp` 빌드, C++ 시험 39 개 통과. 시뮬(도메인 88) 녹화 — status 1140 개 모두 26 칸, hold=1 230 개, analyze_run 의
  hold 구간 9.1~11.4 s 와 MPC 로그 "hold 진입" 일치. 가짜 `/trajectory_node_list`·`/constraint_list` 는 빠지고 다른 토픽은 남음.
  launch 와 같은 신호 조건(SIGINT 기본)에서 SIGINT 뒤 0.4 s 에 bag·run_info 정리 끝.
  `&` 로 띄운 셸 밑(SIGINT 무시 상속)에서는 녹화기가 SIGINT 를 못 받아 안 끝났다 → SIG_DFL 실행 + SIGTERM 단계로 고침.
  can0 이 없을 때는 CAN 파일만 건너뛴다. **실차 CAN 이 있는 상태로는 아직 안 돌려 봤다.**
- 본 것: 시뮬 시험 중 NUC 패키지 온도 100 °C (`thermal.csv`), 같은 시각 다른 작업(`2026amr_ws` 의 DRA_MPPI 벤치마크)이 CPU 한 코어 100 %.
  실험 중 NUC 에서 다른 무거운 작업을 같이 돌리면 시간 지터의 원인이 될 수 있다.

### 2026-11-16 (7) — 두 원 8자 기준 `shape:=circles8`
- 왜: 사용자 요청 "로봇 기준 좌우 8자, 원 지름 1 m, 가로 2 m, 원 두 개 합친 경로, 헤딩 앞 고정". 기존 8자는 Gerono 렘니스케이트라
  잎 곡률이 변하고 잎이 앞뒤(로컬 x)로 나간다.
- 바꾼 파일 (`controller/mpc`):
  - `amr_mpc/core/figure8.py`: `Figure8(curve="gerono"|"circles")`. circles 는 반지름 R = width/4 원 두 개가 기동 포즈에서 맞닿고
    중심 (0, ±R) — 잎이 로봇 왼쪽·오른쪽, 교차점 접선이 앞. 왼쪽 원 반시계 → 오른쪽 원 시계. |dp/ds| = 2R 일정, L = 4πR.
    곡률이 교차점에서 +1/R → −1/R 로 계단 (구심 가속 방향이 한 번에 뒤집힌다). 기존 Gerono 동작 그대로.
  - `amr_mpc/nodes/figure8_reference.py`: `shape:=circles8` (width = 좌우 전체 폭, height 안 씀, yaw_mode·speed·accel·laps 공통).
  - `launch/real_all.launch.py`: `width`·`height`·`yaw_mode`·`align_s` 를 torque_experiment 로 넘긴다 (예전엔 안 넘어갔다).
  - `test/test_figure8.py`: 두 원 위·좌우 폭 2R·왼쪽 먼저·닫힘·유한차분·고정 헤딩 ω ≡ 0 (tangent·fixed). **20 개 통과**.
- 확인: 닫힘 0 mm·0°, 원에서 벗어남 1e-13 mm, 유한차분 |R·u − dp/dt| 1.1e-9 m/s. 부하(L1, V_LIMIT 1.55) 0.3 m/s 27 %, 0.5 m/s 46 %.
  시뮬(도메인 88, `width:=2.0 yaw_mode:=fixed speed:=0.3 limit_mode:=friction mu_hat:=0.3`, 바닥 효율 없음): 참값 기준 RMS 14.9 mm,
  최대 40.8 mm, 헤딩 RMS 0.23°, 상한 닿음 26 %, 완주 뒤 hold. **실차 미확인.**

### 2026-11-16 (6) — 모든 토픽 bag 녹화 `record_all.sh`(`amrrec`, `amrall bag:=`) + 한 판 분석 `analyze_run.py`
- 왜: 오후 대각 세 판이 bag 없이 MPC CSV 만 남아 SLAM·IMU 교차 확인을 못 했고, MPC 가중치가 실제로 몇이었는지도 로그에 없었다.
- 바꾼 파일 (`controller/mpc`):
  - `tools/record_all.sh` (새): `ros2 bag record --all-topics --storage mcap --exclude-regex '(.*/(image_raw|image_rect_raw|image|points)(/.*)?)'`
    — 카메라 영상·점군만 뺀다. 녹화 20 s 뒤 같은 폴더 `run_info/` 에 args.txt(실행 인자), params/(실험 노드 10 개 `ros2 param dump`),
    git.txt·git.diff(커밋 해시·커밋 안 된 변경). 같은 이름 폴더가 있으면 `_시분초` 를 붙인다.
  - `launch/real_all.launch.py`: 인자 `bag:=<이름>` (상대 경로는 작업공간 `bags/` 아래) → record_all.sh 를 같이 띄운다. Ctrl+C 로 같이 닫힌다.
  - `tools/amr_aliases.sh`: `amrrec <이름>` 을 record_all.sh 로 (예전엔 토픽 11 개 고정).
  - `tools/analyze_run.py` (새): 추종 구간마다 대기/주행 오차·상한·Δτ·자이로 ω RMS·주파수, 추정기−SLAM 위치·속도 지연·진폭비, 보정 잔차,
    정지 판정 지연, 바퀴별 CAN fresh=False 수, 슬립, /rosout WARN 집계, `--plot`.
- 확인: 시뮬(도메인 88)에서 record_all.sh 로 53 s 녹화 → SIGINT 로 정상 종료, 토픽 23 개, 가짜 `/camera/color/image_raw`·`…/compressed`·
  `/camera/depth/color/points` 는 빠지고 `camera_info` 는 남음, run_info params 의 `w_delta: 10.0`. real_all 은 실차 노드를 띄우지 않고
  `_nodes()` 가 만드는 action 만 확인 (bag:= 있으면 녹화 ExecuteProcess 1 개, 없으면 0 개). analyze_run.py 를 오전 `slow2` bag 에 돌려
  앞서 손으로 낸 값과 같음: 대각 앞 오차 최대 15.1 cm·상한 60 %·Δτ 0.0148, 추정기 vx 지연 +60 ms·진폭비 0.79, 정지 판정 0.45 s.
  **실차로 녹화해 보지는 않았다** (실차 CPU 부하는 다음 판 /rosout 의 "상태 주기 ≠ Ts" 경고로 본다).
- 추가(같은 날, 사용자 요청 "날짜 시간 자동"): 폴더 이름을 record_all.sh 가 정한다 — `bag:=true`·`amrrec`(이름 생략) → `bags/<YYYY-MM-DD>/<HHMMSS>`,
  `/` 없는 이름 → `bags/<YYYY-MM-DD>/<HHMMSS>_<이름>`, `/` 있는 이름 → `bags/<그대로>`, 절대 경로 → 그대로. `bag:=false`·빈 값이면 녹화 안 함.
  `RECORD_ALL_DRYRUN=1` 로 이름만 확인 (6 가지 경우 확인).

### 2026-11-16 (5) — MPC r_Δ(`tuning.w_delta`) 기본값 0.01 → 10
- 왜: (3)의 지시서 가중(r_Δ 0.01)으로 실차 대각을 돌리니 1 Hz 앞뒤 진동은 사라졌지만, 기준이 서 있는 동안에도 토크가 ±τ̄ 를
  5.7~6.5 Hz 로 오가며 로봇이 회전 방향으로 떨었다(대기 3 s 동안 상한 68 %, 자이로 ω RMS 9.9 °/s). r_Δ 만 바꿔 같은 명령을 다시 돌렸다.
- 실차 (5-3 대각 0.4 m·45°·0.10 m/s, C μ̂ 0.55, W·r_U·d̂ 는 (3) 그대로, MPC CSV 추정기 기준, 각 1 판, bag 없음):
  | r_Δ | 앞 오차 최대 / RMS | 상한 닿음 대기 / 주행 | 틱간 Δτ RMS | ω RMS 대기 / 주행 | hold |
  |---|---|---|---|---|---|
  | 0.01 (`spec_diag_*125406`) | 2.7 / 0.88 cm | 68 / 82 % | 0.0317 | 9.9 / 11.4 °/s | 12.8 s, 1.5 mm |
  | 1 (`spec_diag_wd1_*`) | 3.1 / 1.19 | 43 / 86 | 0.0274 | 6.6 / 11.7 | 15.1 s, 5.3 mm |
  | **10** (`spec_diag_wd10_*`) | **2.1 / 0.50** | **0 / 17** | **0.0129** | **1.7 / 4.3** | 12.9 s, 3.4 mm |
  | (참고) 오전 예전 가중 W_v 0.1·r_Δ 10 (`diag_slow_*`) | 9.9 / 3.12 | 0 / 62 | 0.0141 | 0.3 / 8.4 | 못 들어감 (1 Hz 진동) |
- 바꾼 파일: `mpc/config/torque_mpc.yaml` `tuning.w_delta` 10.0 (근거 표를 주석으로). C++ 코드 기본값(`torque_mpc.hpp` `TorqueTuning::w_delta` 0.01)은
  그대로 — 노드는 YAML 값을 쓴다. 한 번만 바꿀 때 launch 인자 `mpc_w_delta:=…` 그대로.
- 확인: `amr_mpc` 재빌드 뒤 설치본 YAML 10.0, MPC 노드를 단독으로 띄워 `ros2 param get /torque_mpc_node w_delta` → 10.0.
- 남은 것: 각 값 1 판씩이라 반복 확인 전. 13:06 판도 끝 구간(10~14 s)에 상한 40~56 % 로 올라갔다. 옆 0.3 m·제자리 90° 미확인.
  1 Hz 진동이 사라진 이유가 W_v(0.1 → 1)인지 다른 (3) 변경(r_U·U_s·q_null 0·d̂)인지는 가르지 않았다.

### 2026-11-16 (4) — 실차를 터미널 하나로 `real_all.launch.py` + `amrall`
- 왜: 실차마다 SSH 터미널 다섯 개(amrcan·amrsens·amrloc·amrbase/amrmpc·amrviz) + 측위 맞추기 (사용자 요청 "터미널 하나에서").
- 바꾼 파일 (`controller/mpc`): `launch/real_all.launch.py` (새) — sensors · cartographer(localization) · torque_experiment(`plant:=real use_rviz:=false`,
  `use_mpc` 기본 false, 트랙·상한·MPC 튜닝 인자 전달) · torque_viz(`rviz:=false`) 를 각각 `GroupAction(scoped=True)` 로 include,
  `auto_localize` 면 15 s 뒤 `tools/check_localization.py --apply` 한 번. `tools/amr_aliases.sh` 에 `amrall` (환경·`AMR_PEERS`·`sudo -v`).
  CAN 은 torque_experiment 의 자동 준비(`auto_setup_can`)가 맡는다.
- 확인: 빌드, `--show-args`, include 인자 조합(dry-run). **실차로 띄워 보지는 않았다.**

### 2026-11-16 (3) — MPC 비용을 구현 지시서대로: 목표 입력 U_s·입력 편차 항, 가중치 되돌림, 외란 추정치 건너뛰기
- 왜: 실차 5-3 옆·대각·회전 진동. 구현 지시서(`amr/자료/torque_mpc_v1_implementation.md`)와 대조하니 비용이 달랐다 —
  Q 속도 가중 1/10 (11-12 잠정 튜닝), r_Δ 1000 배, 입력 편차 항 r_U‖U − U_s‖² 과 목표 입력 U_s(식-14) 없음(대신 영공간 벌점).
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp`: `TorqueTuning::r_u`(기본 0.01), `q_null` 기본 0,
    U_s,j = ((T⁻¹)ᵀ)⁺ R(θ_r,j)ᵀ M (p̈_r,j − w_j − â_d) (j < N_c), H += R̄_U, g −= R̄_U 𝐔_s (식-16·18).
    `step(x̂, ref, acc_ref, dt_state)` — 기준 가속도를 밖에서 받고(없으면 지평 점에서 R(θ)(u̇ + ω S u)), 상태 간격·잔차 계단이면 d̂ 갱신 건너뜀
    (`dist_dt_tol` 0.002 s, `dist_skip_xy` 1 mm, 지시서 4절·7.4절), `target_input()`, `stats().d_skipped`
  - `mpc_cpp/src/torque_mpc_node.cpp`: 파라미터 `r_u`·`acc_dt`·`disturbance_dt_tol`·`disturbance_skip_xy`, 기준 가속도 = 기준 속도(관성)의 중앙 차분 ±0.1 s (지시서 3.9), 상태 간격 전달
  - `mpc/config/torque_mpc.yaml`: W [100, 100, 50, 1, 1, 0.5], w_delta 0.01, r_u 0.01, q_null 0, acc_dt 0.2, disturbance dt_tol·skip_xy.
    **외란 추정치 enable 은 false 그대로** (2026-11-10 결정. 지시서는 켬 — 켤지 따로 결정)
  - `mpc/launch/torque_experiment.launch.py`: `mpc_W`, `mpc_w_delta`, `mpc_r_u`, `mpc_q_null` 인자 (실차 튜닝용)
  - `mpc_cpp/test/test_torque_mpc.cpp`: U8(손 계산 0.0241 — 입력 편차 항 포함으로 0.0275 → 0.0241), U10(U_s = ±0.01667, 바퀴 관성 켬·끔),
    U11(기준 위에서 U* ≈ U_s), 외란 추정치 건너뛰기. **26 개 모두 통과**
- 시뮬(도메인 88, 잡음 1 배): 8자 B 0.5 m/s — 예전 가중 RMS 10.1 mm·|τ|max 0.159·틱간 Δτ RMS 0.013 / 지시서 6.7 mm·0.501·0.093 / 지시서 + r_Δ 1: 9.4·0.242·0.034.
  대각 0.4 m C(μ̂ 0.55)+바닥 효율 — 예전 오차 최대 2.0 cm·상한 1 %·Δτ 0.011 / 지시서 1.7 cm·22 %·0.032 / r_Δ 1: 2.4 cm·14 %·0.022.
  → 추종은 좋아지고 토크 떨림이 3~7 배 (11-12 에 잠정 튜닝을 한 이유). 시뮬 잡음(σ 0.95)은 실차(앞 0.3, 옆 0.6~0.7 m/s²)보다 크다. **실차 미확인.**
- 추가(같은 날, 사용자 결정 "켜기"): **외란 추정치 기본 켬**, α 1 → **0.1** (지시서 7.4절 3안). launch 인자 `mpc_dist_alpha` 추가.
  시뮬 8자 B: d̂ 끔 6.7 mm·0.501·0.093 / α 1 7.7 mm·0.998·**0.322** (â_d 최대 7.7 m/s² — 속도 잔차 잡음 ÷ Ts) / α 0.1 7.2 mm·0.508·0.101 / α 0.02 6.8·0.634·0.094.
  대각 C: 끔 1.7 cm·상한 22 % / α 1 2.6 cm·66 % / α 0.1 2.2 cm·42 %.

### 2026-11-16 (2) — 시뮬 플랜트 바닥 효율·바퀴별 효율 (`floor_eff`, `wheel_eff`)
- 왜: 실차 5-3 옆 0.3 m 에서 앞뒤 진동(±11 cm). 실차와 시뮬 차이를 하나씩 넣어 재현하려고.
- 바꾼 파일 (`controller/mpc`): `amr_mpc/torque/model.py` `TorquePlant.step(…, eff=None, wheel_eff=None)` — Ẍ = M⁻¹ R(θ) diag(η) Tinv^T diag(η_w) τ
  (MPC 예측 모델은 그대로 1). `nodes/sim_plant_torque.py` 파라미터 `floor_efficiency`·`wheel_efficiency`(슬립 모델과 같이 못 씀),
  `launch/torque_experiment.launch.py` 인자 `floor_eff:=ηx,ηy,ηψ`, `wheel_eff:=FL,FR,RL,RR`. 기본 1 → 기존 동작 그대로.
- 맞춤값: 5-2 의 0.15 N·m 가속 → `floor_eff:=0.65,0.40,0.40`. 결과: 진동 재현 안 됨 (`analysis/torque_mpc_real/2026-11-16_5-3/RESULTS_KO.md`).

### 2026-11-16 (1) — 5-3 단일 축 기준 `shape:=line|rotate`
- 왜: 체크리스트 5-3(한 축 폐루프)에 직선·제자리 회전 기준이 필요했다 (기준 노드는 8자만 냈다).
- 바꾼 파일 (`controller/mpc`):
  - `amr_mpc/core/segment.py` (새): `Segment` — line(출발 몸체 방향 `dir_deg` 로 `distance`, 사다리꼴 speed·accel, 헤딩 고정) /
    rotate(제자리 `angle_deg`, 사다리꼴 yaw_rate·yaw_accel), `out_and_back` 이면 `pause_s` 쉬고 돌아옴. Figure8 과 같은 꼴(at·peak_load·path_xy·T_total).
    자체 검산: 속도 vs 위치 유한차분 최대 차 ≤ 5e-12, 끝 위치 오차 0, 사다리꼴 시간 손계산과 같음(0.5 m·0.2 m/s·0.4 m/s² → 3.00 s)
  - `nodes/figure8_reference.py`: 파라미터 `shape`, `seg_distance`, `seg_dir_deg`, `seg_angle_deg`, `seg_yaw_rate`, `seg_yaw_accel`, `seg_out_and_back`, `seg_pause_s`
  - `launch/torque_experiment.launch.py` `_TRACK_KEYS`, `config/torque_track.yaml` 기본값(shape figure8 — 기존 동작 그대로)
- 확인(시뮬, 도메인 88, `origin_mode:=robot`): B 로 앞 0.5·옆 0.3·대각 0.4 m (0.2 m/s)·회전 90° 완주, 위치 오차 최대 2.2~2.6 cm·끝 0.4~0.7 cm·헤딩 ≤ 0.6°.
  C `mu_hat:=0.55`(τ_μ 0.1392) 로 앞 0.5 m (0.15 m/s)·회전 90° 완주, 상한 0.139 에 닿음. **실차 미확인.**

### 2026-11-15 (4) — 측위 `translation_weight` 10 → 1 (빠를 때 SLAM 넘침)
- 왜: 실차 5-2 에서 토크 0.15·0.2 로 빠르게 움직이면 SLAM·추정기가 수십 cm~1 m 틀렸다(줄자 50 cm 에 SLAM 101.6). Cartographer 가 직전 움직임으로
  예측한 위치를 스캔-지도 맞춤보다 10 배 믿어서(Ceres 비용의 예측 이탈 가중 10), 급정지 뒤에도 1 s 넘게 "계속 달렸다".
  bag 재생으로 설정을 하나씩 바꿔 원인을 이 값으로 좁혔다(IMU 예측 끔·스캔 누적 1 은 아님) — `analysis/torque_mpc_real/2026-11-15_basic/RESULTS_KO.md`.
- 바꾼 파일:
  - `perception/amr_perception_bringup/config/localization_torque_2d.lua` — `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.` (사용자가 직접 넣음).
    `localization_torque_odom_2d.lua` 도 include 로 같이 바뀐다
  - `controller/mpc/tools/slam_replay/variants/v_tw10.lua` (예전 값 비교용) 추가, `v_tw1.lua` 주석
- 확인(재생, `tools/slam_replay`): 빠를 때 — fwd02 끝 49.9·넘침 50.2 cm (줄자 50, 이전 64.2·71.0), basic2 앞·뒤 8 번 IMU 와 1~3 cm·넘침 없음
  (이전 최대 +185 cm). 느릴 때 — day1 손 밀기 16 표시가 이전 설정과 0.4 cm·0.2° 안에서 같음, 사람 옆 표시 0.7 → 0.2 cm.
  **고침(같은 날 밤)**: 처음 넣을 때 `include "localization_2d.lua"` 와 `options.use_…` 두 줄이 두 번 들어가 Cartographer 가 시작하자마자 abort
  (`Tried to include … twice`, af48c1b 에 들어간 상태) — 중복을 지웠다. 앞서 "설치본 43 행 확인"은 이 중복을 못 본 확인이었다.
  **실차 확인**: 5-2 4-1 앞·뒤 × 0.15·0.2 에서 SLAM 넘침 0.5~1.6 cm, 앞 0.2 줄자 47.0 vs SLAM 48.4 cm (이전 설정 101.6) — `RESULTS_KO.md` "실차 확인".

### 2026-11-15 (3) — bag 재생 도구 `tools/slam_replay/` (로봇 없이 측위 설정 비교)
- 왜: 빠르게 움직일 때 SLAM 이 틀린 원인을 bag 재생으로 찾았는데(translation_weight), 그 스크립트가 세션 임시 폴더에 있다가 NUC 재부팅으로
  지워졌다. 측위 설정을 바꿀 때마다 같은 방법으로 확인하려고 리포에 넣음(사용자 결정).
- 바꾼 파일 (모두 새 파일, `controller/mpc/tools/slam_replay/`):
  - `replay.sh` — bag 의 `/scan_front`·`/scan_rear` 로 `scan_sector_filter` 를 다시 돌리고, `/imu/base_link` → `/camera/imu` 로 축을 되돌리고,
    `cartographer_node`(smoc_lab, frozen)를 준 설정으로 띄워 시작 위치를 넣은 뒤 재생. 도메인 89, 결과 `~/slam_replay_out/<라벨>/pose.csv`
  - `imu_relay.py`, `set_init.py`(finish/start_trajectory), `pose_rec.py`, `compare.py`(토크 시험: 바퀴·IMU·기록 SLAM·재생 비교 /
    손 밀기: `slam_mark` 표시마다 줄자와 비교)
  - `variants/v_tw1.lua`(translation_weight 1), `v_noimu.lua`, `v_acc1.lua`
- 확인: fwd02(줄자 50 cm) 재생 — 지금 설정 끝 64.2·넘침 71.0, v_tw1 끝 49.9·넘침 50.2 (어제 임시 스크립트 결과와 같음). 바퀴 48.4, IMU 47.9.

### 2026-11-15 (2) — 기본 동작·SLAM 표시 도구: 메시지 시각 사용, 끝낼 때 core dump 고침
- 왜: 5-2 4-1 첫 판에서 도구가 SLAM·추정기 값을 "받은 시각"으로 기록해 SLAM 지연(0.1~0.2 s)만큼 밀렸다 → 토크·제동 구간 해석이 틀어짐
  (그 판의 "제동 중 몸체가 미끄러져 바퀴의 두 배를 갔다"는 해석은 틀렸고 실제로는 SLAM 오류였다 — `analysis/torque_mpc_real/2026-11-15_basic/RESULTS_KO.md`).
  `q`·끝날 때 rclpy spin 스레드가 남아 "terminate called without an active exception" 으로 죽었다.
- 바꾼 파일:
  - `tools/basic_motion_test.py` — `/tracked_pose`·`/state_estimator/odom` 기록 시각을 header stamp 로, `--dirs` 경로는 executor 를 따로 두고 끝낼 때 shutdown·join
  - `tools/slam_mark.py` — 같은 종료 처리
- 확인: 문법, `slam_mark` 를 `q` 로 끝내 종료 코드 0. 4-1 두 번째 판(`fwdback2`)이 고친 도구로 기록됨.

### 2026-11-15 (1) — 로봇 질량 4.0 (추정) → 4.56 kg (저울)
- 왜: 체크리스트 7단계 m. 사용자가 저울로 잼(AK40 장착, 배터리 포함, 적재 없음). 4.0 은 1판 추정값이라 14 % 가벼웠다.
  바닥 마찰 한계·C 상한·MPC 모델 질량이 모두 이 값에 기대므로 μ 측정 전에 바꿈(사용자 결정).
- 바꾼 파일:
  - `config/torque_plant.yaml` — body.mass_kg 4.56
- 영향(계산): 바퀴당 수직력 N = m g/4 = 11.18 N (전 9.81). C 상한 τ_μ = (r/√2)·μ̂·N·c_s = 0.0759 N·m @ μ̂ 0.3 (전 0.0666).
  μ 별 바퀴 미끄럼 시작 토크 μ·N·r: 0.1 → 0.045, 0.15 → 0.067, 0.2 → 0.089, 0.3 → 0.134 N·m.
  시험 안의 0.0666 은 `mass=4.0` 을 직접 주거나(C++ TorqueModel 기본값) 1판 수치 예라 그대로.
- 확인: 시험 Python 184·C++ 33 통과. 시뮬(NUC, 도메인 88) 플랜트 로그 "m=4.56 kg", MPC "τ_μ 0.0759 @ μ̂ 0.30".
  B 12.73 mm·최대 36.2 (4.0 kg 일 때 같은 날 12.0~12.9), A 6.01 mm·최대 18.4 (5.1~6.3), B 0.3 m/s 12.63 mm·36.9 — 모두 완주.

### 2026-11-14 (8) — 측위 확인 도구 `check_localization.py`, 기본 동작 도구에 방향·크기·CSV
- 왜: 2026-11-14 밤 첫 바닥 주행 전 측위가 가정값 (0, 0, 0°)에 머물러 있었다(원본 지도 겹침 20.8 %, 실제 1.3 m·64° 어긋남). 손 계산으로 찾아
  `/initialpose` 로 넣었는데 매번 할 일이라 도구로. 그리고 8자 run1 이 옆걸음 슬립으로 발산 → 사용자 제안으로 5단계를 기본 동작부터(체크리스트 5-0~5-4).
- 바꾼 파일:
  - `tools/check_localization.py` (새) — `/scan_matched_points2` 를 원본 지도 png 벽과 겹쳐 비율(±5 cm), 낮으면 지도 전체 탐색(10 cm·5° → 2 cm·0.5°)으로
    위치를 찾아 `/initialpose` 명령 출력, `--apply` 면 넣고 다시 확인. `/map_carto` 는 새 스캔이 섞여 자기 비교가 되므로 원본 png 사용
  - `tools/basic_motion_test.py` (jjm 도구, 기존 동작 그대로) — `--dirs`(fwd back left right fl fr bl br ccw cw), `--amps` 목록, `--csv`
    (`_samples.csv` 추정기·SLAM·바퀴 시계열, `_summary.csv` 동작별: 추정기·SLAM·바퀴 계산 이동, 효율 = SLAM/바퀴, 방향 오차, 바퀴 평균 속도)
  - `tools/slam_mark.py` (새) — 5-1 손으로 밀기 시험용: Enter 마다 SLAM 위치(바퀴 축 중심, 0.5 s 평균)와 앞 표시·기준에서의 이동, 줄자 값 차이, CSV
  - `amr/notes_amr/real_robot_checklist.md` — 5단계를 5-0 준비 / 5-1 라이다 오도메트리 / 5-2 개루프 기본 동작 / 5-3 한 축 폐루프 / 5-4 8자로
- 확인: 시험 184개 통과, 두 도구 문법. `check_localization` 위치 찾기 — 지도 벽에서 만든 가짜 스캔(참 (−0.33, +1.25) m, −64°, 잘못 놓인 겹침 15.8 %)을
  (−0.331, +1.248) m, −64.5° 로 찾음(5 s). `basic_motion_test` 방향 10 개의 토크 → 몸체 힘 Tinv^T τ 가 의도한 축(대각은 Fx = Fy), pinv(Tinv)·Tinv = I,
  SLAM 추적점 → 바퀴 축 환산 검산. 실차 미확인.

### 2026-11-14 (6) — 토크 MPC 에 "현재 상태 기준 선형화"(current_state) 추가, 기본은 reference 유지 (jjm)
- 왜: 2026-11-13 교수님 지시 "현재 스텝 기준 linearized model". 전준민이 다른 폴더(`2026amr_ws` `feat/torque-mpc`)에서 만든
  패치를 이 저장소로 옮겼다. todo "선형화: 기준궤적을 따라 한다" 결정은 그대로 두고 스위치로 비교할 수 있게 했다.
- 방식: 매 주기 x̂(k)·τ(k−1) 에서 ẋ = f(x, τ) 를 야코비안으로 선형화(상수항 포함) → 증강행렬 지수로 ZOH 이산화 →
  그 한 모델을 지평 50 스텝 전체에 쓴다. 외란 추정치도 이 모델의 한 스텝 잔차. 비용·제약·결정변수는 그대로.
- 바꾼 파일:
  - `mpc_cpp/include/amr_mpc_cpp/torque_mpc.hpp`, `src/torque_mpc.cpp` — `TorqueLinearization`, `parse_linearization`,
    `torque_dynamics`, `linearize_continuous`, `discretize_zoh`, `step()` 예측부 분기, d̂ 분기
  - `mpc_cpp/src/torque_mpc_node.cpp` — 파라미터 `linearization` (YAML `linearization.mode`, 없으면 reference), 기동 로그
  - `mpc/config/torque_mpc.yaml` — `linearization: {mode: reference}`
  - `mpc/launch/torque_experiment.launch.py` — 인자 `linearization:=current_state|reference`
  - `mpc_cpp/test/test_torque_mpc.cpp` — current_state 시험 10 개 (문자열, 야코비안 = 유한차분, ZOH = 1판 식-8,
    한 스텝 = RK4 플랜트, 정지·yaw 언랩, 등속 선회 2 종, 직선 출발에서 두 방식 같은 토크, 큰 초기 방향 오차, d̂ 오프셋 제거)
- 확인 (PC WSL, ROS 2 Jazzy): gtest torque 22·wheel 11 통과, pytest 184 중 실패 4 는 Windows 체크아웃의 CRLF `setup_can.sh` 뿐(기존).
  시뮬 8자 0.5 m/s (PC, 40 s, 한 번씩 — 반복 실행 사이 RMS 가 ±0.1 cm 정도 흔들림):

  | 조건 | reference | current_state |
  |---|---|---|
  | 기본 (상한 B) | RMS 0.90 cm · 최대 2.71 | RMS 0.95~1.10 · 최대 2.30~2.75 |
  | payload 5 kg, 제어기는 0 kg | 1.17 · 3.11 | 1.17 · 3.01 |
  | surface_mu 0.3 + 상한 C (friction) | 1.35 · 4.33, 토크 최대 0.127 | 1.29 · 4.62, 0.128 |
  | **surface_mu 0.3 + 상한 B (motor_only)** | **1.05 · 3.00, 토크 최대 0.25, 완주** | **3/3 회 실패: 약 5 s 에 track_error 30 cm, 토크 1.3 포화, 슬립 λ RMS 0.84** |
  | 단위시험 초기 방향 오차 30°/40° (R 1 m) | RMS 1.69 / 1.92 mm | 1.48 / 1.78 mm |

- 결정 (전준민): **기본은 reference 유지.** 실차 기본이 상한 B 이고 바닥이 미끄러울 수 있어서. current_state 는 `linearization:=current_state` 로만.
- 남은 과제: B·저마찰 실패 원인 확인 — 추정: 미끄러져 τ(k−1) 이 커지면 그 τ₀ 로 만든 ∂(Gτ₀)/∂θ 항이 커져 다음 토크가 더 커지는 되먹임.
  보완 후보: 선형화에 쓰는 τ₀ 를 상한 C 수준으로 제한, 또는 지난 해의 예측 궤적을 따라 다시 선형화(RTI). 보완 뒤 같은 표로 다시 비교. 실차 미확인.

### 2026-11-14 (7) — `hardware_checked: true` (토크 ±0.1 N·m 묶음 해제)
- 왜: 묶음의 목적은 ID·장착 부호가 틀렸을 때 엉뚱한 바퀴가 세게 도는 것을 막는 첫 시험용이었다(2026-11-12 (3), 0.1 은 벤치 입력 상한과 같게 잡은 값,
  주행 한계 측정값 아님). 실차 3단계에서 ID 1~4·장착 부호(+0.08 N·m 네 바퀴 눈으로 전진)를 확인해 목적은 끝났다.
  바닥에서는 0.1 이 안전에 거의 도움이 안 되고 주행을 막는다: 최종 토크(보상 포함) 상한이라 0.5 m/s 에서 MPC 몫은 약 0.057 N·m 인데
  시뮬 8자 0.5 m/s 의 최대 토크는 0.165~0.27 N·m. 폭주 시 0.1 을 계속 내도 FL 실측 마찰로 바퀴 38 rad/s = 1.53 m/s 까지 약 0.8 s
  (상한 1.2 면 전압 한계 1.66 m/s 까지 약 0.6 s) — 상한은 가속만 줄이고 최고 속도는 거의 못 줄인다.
  모터 쪽 CAN 끊김 보호가 없어 사용자·팀이 수동 정지 절차(한 명 로봇 들기, 한 명 모터 전원 뽑기)를 조건으로 결정.
- 바꾼 파일:
  - `config/ak40.yaml` — `hardware_checked: true`, 근거·조건 주석, `first_run_tau_limit` 주석(false 일 때만, 폭주 방지 효과 없음)
- 확인: Python 시험 184개, C++ `test_torque_mpc` 22·`test_wheel_mpc` 11 통과 (jjm 의 current_state 선형화 커밋 bb69f8d 포함 빌드).
  시뮬 B(기본 linearization reference) 12.95 mm·최대 33.1, fault 없음. 실차 미확인 — 노드 기동 로그에 "토크 ±4.1 N·m", hardware_checked 경고가 없는지 볼 것.

### 2026-11-14 (5) — 구동계 마찰을 실측값으로, 보상은 90 % (compensation_scale)
- 왜: 실차 2단계 내림 계단(`analysis/torque_mpc_real/2026-11-14_bench/RESULTS_KO.md` 7절)에서 바퀴별 마찰이 기존 값(팀원 식별 후보,
  10 rad/s 이하 자료)과 38.7 rad/s 에서 ±30 % 달랐다(FL·RR 보상 부족, FR·RL 과보상). todo 7단계 "마찰 값 갱신".
  보상을 측정값보다 약간 작게 넣는 것은 과보상(정지 근처 떨림)을 피하려는 관례이고, 같은 날 워밍업으로 마찰이 약 15 % 줄었다.
  사용자 결정 90 %. `friction` 절은 실측 그대로 두고(시뮬 플랜트 = 실제 마찰), 보상 노드만 배율을 곱하게 나눴다 —
  절 값에 90 % 를 넣으면 시뮬 플랜트도 90 % 가 되어 "덜 보상" 효과를 시뮬에서 못 본다.
- 바꾼 파일:
  - `config/torque_plant.yaml` — friction coulomb [0.02142, 0.02151, 0.02170, 0.01830], viscous [0.002062, 0.001799, 0.001767, 0.001679]
    (이전 [0.0280, 0.0279, 0.0239, 0.0253] / [0.00113, 0.00235, 0.00221, 0.00080]), `compensation_scale: 0.9` 추가, tau_fric_max 주석
  - `amr_mpc/nodes/torque_output.py` — 파라미터 `compensation_scale`(기본 = yaml), 보상 계수 = 실측 × 배율, 0~1.5 밖이면 시작 거부
  - `test/test_torque_output.py` — 새 값 확인, `test_compensation_scale_applies_to_output_only`
- 확인: 시험 184개 통과. 정격 38.75 rad/s 보상 최대 0.9 × 0.1013 = 0.091 N·m < tau_fric_max 0.1 → "마찰 보상 여유" 경고 사라짐(시뮬 로그 0 회).
  시뮬(NUC, 도메인 88) B 12.62 mm·최대 32.5 (이전 마찰 같은 날 12.16 / 35.9, PC 기준 12.8), A 5.09 mm·최대 15.9 (이전 5.34 / 15.5).
  Traceback·"process has died" 0. 실차 미확인(5단계, 바닥 하중에서 다시 본다).

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
