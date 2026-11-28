# pitwall — 스택을 켠 채 다음 판 조건 바꾸기 (2026-11-28)

예전에는 실험 조건을 바꿀 때마다 `amrall`(`poly`·`circ`) 을 껐다 켰다. 이제는 한 번 띄운 채로 기준 경로·MPC 값을 바꾸고 's' → 'a' 로 다음 판을 돈다.
설계·unicorn-racing-stack pitwall 조사: [pitwall_design.md](pitwall_design.md).

## 구성

| 도구 | 어디서 | 하는 일 |
|---|---|---|
| `pit` (`tools/pit.py`) | NUC 다른 터미널 (SSH 가능) | 상태 보기, 프리셋 `pit poly`·`pit circ`, `pit set k=v`, `pit new` |
| `amrpit` (`rqt_reconfigure`) | 화면 있는 컴퓨터 (랩 PC) | `figure8_reference_node`·`torque_mpc_node` 값을 GUI 로 — 슬라이더, `preset` 칸, `new_run` 체크박스. 실행 중 못 바꾸는 기준 값은 회색 (read_only) |
| RViz (`torque_real.rviz`) | 랩 PC | **Next run preview** (자홍색) = 다음 판 경로, **Pitwall status text** = 판 상태·다음 판 요약·거부 이유 |
| `amrruns` (`tools/bag_runs.py`) | 어디서나 | bag 하나에 든 여러 판을 나눠 판마다 조건·MPC 값 표. `amrcmp ... --runs` 는 판마다 한 줄 |

이 도구들은 **무장하지 않는다**. 'a' · RT 는 사람이 직접 누른다.

## 실차 순서 (예: 꺾은선 → 원)

```bash
poly                       # 터미널 1: 한 번만 띄운다 (bag:=true 포함 — 녹화는 launch 하나에 bag 하나)
```
1. 측위 맞추기 배너("맞음") 뒤 터미널 1 에서 'a' → 판 #1 (꺾은선).
2. 판이 끝나면 (완주 + hold) 터미널 1 에서 's'. **다음 'a' 가 새 판 #2** — 무장한 자리가 출발점 (origin robot).
3. 터미널 2 에서 조건을 바꾼다:
```bash
pit circ                   # 원 8 자 스트레스 판 기준으로 (amr_aliases.sh circ 와 같은 값)
pit circ speed=1.0         # 프리셋 + 덮어쓰기
pit set accel=1.2 square_v_max=1.3      # 값 몇 개만
pit set W=[40,40,10,2,2,1] slip_fb_scale=0.5   # MPC 값
pit                        # 지금 상태 (판 번호, 다음 판 요약, 받아 둔 변경, 마지막 거부)
```
4. RViz 의 자홍색 경로 (다음 판, 지금 로봇 자리 기준) 가 벽과 겹치지 않는지 본 뒤 'a'.

판 도중에 's' 로 멈췄을 때 값을 바꾸면 그 판은 버리고 다음 'a' 가 새 판이다. 값을 바꾸지 않고 판만 새로 하려면 `pit new`.

### rqt_reconfigure 로 (unicorn pitwall 과 같은 방식)

```bash
amrpit          # 화면 있는 컴퓨터에서 (= ros2 run rqt_reconfigure rqt_reconfigure, 도메인 81)
```
왼쪽에서 `figure8_reference_node` (기준) 와 `torque_mpc_node` (MPC) 를 고른다.

| 칸 | 하는 일 |
|---|---|
| `preset` (맨 위) | `circ` · `poly` 를 쓰고 Enter → 그 프리셋 기준으로 (`pit circ` 와 같다). 적용되면 칸이 다시 빈다 |
| `new_run` (맨 위 체크박스) | 켜면 멈춘 판을 버리고 다음 'a' 가 새 판 (`pit new`). 정지 중에만, 노드가 도로 끈다 |
| 슬라이더 (speed·accel·width·laps·square_* …) | 끌거나 오른쪽 칸에 숫자 → 다음 판 값. 범위는 `core/ref_config.py` `UI_RANGES` |
| 회색 칸 (dt·rate_hz·load_budget·abort_error …) | launch 때만 — 못 바꾼다 |
| `torque_mpc_node` 의 W·w_dudt·slip_fb_scale … | MPC 12 개 값. 그 밖의 칸은 넣어도 거부된다 |

거부되면 (부하 예산 초과·범위 밖·launch 전용) 칸이 원래 값으로 돌아가고, 이유는 RViz 상태 글자·`pit`·노드 로그에 나온다.
값을 하나씩 보내니 예산에 걸리면 안전한 쪽부터 (속도를 먼저 낮추고 반지름을 줄인다). 여러 값을 한 번에는 `pit set`.

**IP 없이 도메인만 맞추기 (unicorn 스택처럼, 2026-11-28 `AMR_DISCOVERY`)** — 우리 별칭은 기본이 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (NUC 혼자) 라
다른 PC 가 안 보였다. unicorn 은 Jazzy 기본 `SUBNET` (multicast 로 같은 공유기 안을 찾음) 이라 도메인만 같으면 보였다. 같게 하려면:
```bash
export AMR_DISCOVERY=SUBNET      # NUC, 스택 띄우기 전에 (AMR_PEERS 는 필요 없다)
poly
```
보는 PC (Linux + Jazzy, 작업공간 없어도 됨):
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS
ros2 run rqt_reconfigure rqt_reconfigure      # 또는 rviz2 -d ~/amr_viz/torque_real.rviz
```
- NUC 방화벽 (ufw, 들어오는 것 기본 차단) 이 그 PC 를 허용해야 한다: `sudo ufw allow from <그 PC IP>` (사람이 직접).
- 같은 공유기에 도메인 81 을 쓰는 다른 컴퓨터가 있으면 그 노드도 섞인다.
- 보는 PC 를 rqt·RViz 를 켠 채 끄거나 와이파이가 끊기면, NUC 가 lease (약 10 s) 동안 그 PC 로 계속 보낸다 — 11-23 응답 없는 STATIC_PEERS 멈춤과 같은 꼴이 날 수 있으니 **창을 먼저 닫고** 끈다.

랩 PC 에서 (작업공간 없이, RViz 와 같은 환경, IP 를 직접 넣는 방식 — 지금까지):
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=192.168.0.132
ros2 run rqt_reconfigure rqt_reconfigure
```
(NUC 쪽은 `export AMR_PEERS=192.168.0.209` 로 스택을 띄워야 서로 보인다 — [rviz_lab_pc.md](rviz_lab_pc.md))
`fault_monitor` 가 세운 뒤에도 같다: LT 로 옮기고 → 's' → (값 바꾸거나 `pit new`) → 'a'.

## 언제 반영되나

| 대상 | 무장 중 (주행·완주 뒤 hold) | 판 도중 멈춤 ('s'·B) | 판 끝난 뒤 정지 · 판 없음 |
|---|---|---|---|
| 기준 경로 | 받아 두기만. 지금 판은 그대로 → 그 판이 끝나고 's' 할 때 다음 판으로 | 멈춘 판을 버리고 다음 'a' 가 새 판 | 다음 'a' 부터 |
| MPC 값 | 받아 두기만 → 다음 's' 때 `TorqueMPC` 를 새로 만든다 | 바로 새로 만든다 | 바로 새로 만든다 |

## 바꿀 수 있는 값

| 노드 | 실행 중 바꿀 수 있음 | launch 때만 |
|---|---|---|
| `figure8_reference_node` | 모양·크기·속도·가속·랩·꼭짓점·지그재그·출발점 방식 등 `core/ref_config.py` `LIVE` (shape, width, height, speed, accel, laps, yaw_mode, align_s, square_*, poly_path, zigzag_*, seg_*, speed_mod_*, origin_*, start_delay, hold_s …) | `dt`·`horizon_steps`·`rate_hz` (MPC 기준열 격자), `load_budget`·`max_wheel_rad_per_sec`·`abort_error` (안전 문턱), `path_follow_lead_s`, topic·frame |
| `torque_mpc_node` | `W`, `w_dudt`, `r_u`, `q_null`, `mu_hat`, `slip_fb_enable`, `slip_fb_scale`, `slip_eta_enable`, `disturbance_enable`, `disturbance_alpha`, `roller_rr`, `inertia_z` | `Ts`·`Np`·`Nc` (추정기 100 Hz·계산 시간), `limit_mode`·`cap_on`·`linearization` (비교군), 그 밖 전부 |

`speed:=max`·`accel:=max` 는 launch 가 숫자로 바꿔 넘기는 것이라 실행 중에는 숫자로 준다.

## 안전장치

| 장치 | 내용 |
|---|---|
| 부하 예산 검사 | 기동 때와 같은 `peak_load ≤ load_budget·V_LIMIT` (지금 1.0 × 1.550 m/s). 넘으면 **거부** — 아무 값도 안 바뀐다. 노드 값은 늘 "검사 통과한 다음 판" |
| MPC 시험 생성 | 바꾼 값으로 `TorqueMPC`·슬립 되먹임 이득을 먼저 만들어 보고 생성자 검사에 걸리면 거부. 값마다 범위 (예: `slip_fb_scale` 0~5, `disturbance_alpha` (0, 1]) |
| 진행 판 보호 | 무장 중에는 경로·MPC 를 바꾸지 않는다 (받아 두기만) |
| 미리 보기 | 다음 판 경로가 RViz 에 (정지 중 1 Hz 로 로봇 자리를 따라감) |

rqt_reconfigure 는 값을 하나씩 보낸다. 중간 값이 예산에 걸리면 거부되니 **안전한 쪽부터** 바꾼다 (속도를 먼저 낮추고 반지름을 줄인다). 여러 값을 한 번에 바꿀 때는 `pit set` (한 번에 검사) 이 편하다.

## bag 기록과 판 나누기

launch 하나 = bag 하나 그대로. 판마다 조건이 bag 에 남는다 (`record_all.sh --all-topics` 가 다 녹화):

| 토픽 | 언제 | 내용 |
|---|---|---|
| `/figure8/run_config` | 새 판 출발 | 판 번호, 그 판의 기준 값 전체, 출발 포즈, 부하 %, 기준 시간, launch 때 값 |
| `/figure8/status` | 상태·값이 바뀔 때마다 | 상태 (idle·running·paused·finished), 다음 판 값, 받아 둔 변경, 마지막 거부 |
| `/torque_mpc/config` | 기동·적용·받아 둠·거부·**무장** | MPC 12 개 값 (지금·받아 둔), launch 때 값 (limit_mode, Ts, Np, Nc …) |
| `/parameter_events` | parameter 가 바뀔 때마다 | ROS 기본 (바뀐 값만) |

```bash
amrruns bags/2026-11-28/170000_poly              # 판 표
amrruns bags/2026-11-28/170000_poly --changes    # 판 사이 변경·거부 기록도
amrruns bags/... --json runs.json                # 분석 스크립트용
amrcmp bags/2026-11-28/170000_poly --runs        # SLAM 기준 오차를 판마다 한 줄
```
분석 스크립트에서: `sys.path.insert(0, ".../controller/mpc/tools"); from bag_runs import runs` → `for r in runs(bag)` 의 `r["t0"]`·`r["t1"]` (bag 수신 시각 [s], `/mpc/armed` 와 같은 축), `r["ref"]`, `r["mpc"]`, `r["armed"]`.
**bag 하나 = 판 하나 (첫 무장만) 로 짠 스크립트는 판이 여럿인 bag 에서 첫 판만 본다** — `runs()` 창으로 자를 것.

## 랩 PC 준비 (한 번)

- `rqt_reconfigure` 가 없으면 랩 PC 에서 `sudo apt install ros-jazzy-rqt-reconfigure` (NUC 에는 이미 있다).
- RViz 설정을 다시 가져온다 ([rviz_lab_pc.md](rviz_lab_pc.md) 0-3 의 scp) — 다음 판 경로·상태 글자 표시가 들어 있다.
- 랩 PC 에서 `rqt_reconfigure` 는 RViz 와 같은 환경 (도메인 81, `ROS_STATIC_PEERS=192.168.0.132`) 으로 띄운다.

## 세 제어기 자동 순서 (2026-11-28, `controller:=all` + `amrseq`)

사용자 결정 (2026-11-28): "회차마다 사람이 안 누르고 가만히 두면 알아서", "A launch 하나", fault 면 "전체 순서 중단", zigzag 는 "180° 회전, 제어기당 6회차".

```
 /mpc/armed (게이트) ─▶ controller_select_node ─▶ /ctrl/mpc/armed ─▶ torque_mpc_node ──▶ /torque_mpc/cmd_raw ─┐
                          (active = mpc|pp|adpp)  └▶ /ctrl/pp/armed ──▶ pure_pursuit_node → wheel_pi_node ─▶ /wheel_pi/cmd_raw ─┤
                          └▶ /ctrl/active (JSON, latched) ──▶ torque_output_node: 고른 제어기 토픽만 ◀─────────────────────┘
                                                          ──▶ fault_monitor_node: 고른 제어기 status 의 추종 오차만
                                                          ──▶ pure_pursuit_node: pp | adpp 룩어헤드 묶음
```
- 고르지 않은 제어기는 무장 신호가 false 라 미무장 때와 똑같이 쉰다 (MPC 는 풀지 않고 DAMPING + reset — `torque_mpc_node.cpp` `on_state`). 그 DAMPING 은 `torque_output` 이 버린다.
- 토크 경로에 노드를 더 끼우지 않았다 (중계 없음). 각 제어기 노드 파라미터는 단독 launch 와 같다 — 다른 것은 무장 토픽·룩어헤드 선택 배선뿐 (`test_launch_modes.py` `test_all_controller_params_match_single_mode`).
- 실행 중 바꿀 수 있는 값이 늘었다 (정지 중만): `pure_pursuit_node` `lookahead_*`·`adpp_lookahead_*`, `fault_monitor_node` `max_track_error`, `controller_select_node` `active`. `pit set` 이 이름으로 나눠 보낸다.

### 실차 순서

```bash
amrall3 bag:=seq_s1        # = amrall use_mpc:=true controller:=all limit_mode:=friction bag:=true … (뒤 인자가 이긴다)
```
1. 초록 배너 ("측위 맞추기 끝: 맞음") 를 본다. 로봇을 출발점에 둔다 (그 자리 = 출발점 A).
2. 다른 터미널에서 경로 조건 → 순서:
```bash
pit s1                     # 11-26 S1 circles8 (기준·PP 룩어헤드·fault 한계·MPC 값). s2 = zigzag, s3 = polyline
amrseq                     # mpc → pp → adpp × 6 (세 경로 모두, zigzag 는 회차마다 180° 복귀 회전 — fwd·rev 3:3). --n 5 로 줄일 수 있다. --dry 로 할 일만
```
3. 끝나면 다음 경로: `pit s2` → `amrseq` (launch 를 다시 켜지 않는다. bag 하나에 다 들어간다).
4. 분석: `amrruns bags/…_seq_s1` — 판마다 `제어기 mpc|pp|adpp`, `amrseq 다리 i data|return 회차 r fwd|rev`. 스크립트는 `r["controller"]`·`r["seq"]` (복귀 다리 `kind == "return"` 는 뺀다).

| 무엇 | 동작 |
|---|---|
| 무장 | `amrseq` 가 `/mpc/arm true` (패드 RT 와 같은 토픽). 무장 전 확인: 게이트 정지·기준 idle·로봇 속력 < 0.03 m/s · \|ω\| < 0.05 rad/s 가 2 s |
| 회차 끝 | 기준 `finished` + `end: done` (완주 + hold 5 s) → `/mpc/arm false` |
| 중단 (다시 무장 안 함) | 회차 도중 무장 꺼짐 (fault_monitor · 's' · 패드 B), 기준 `end: abort` (abort_error), 출발 거부, 제한 시간 (기준 시간 + hold + 30 s), 값 거부, Ctrl+C. 출력의 `--skip`·`--anchor-pose` 로 이어서 |
| 출발점 (anchor, 기본 켬) | 시작 때 로봇 포즈 A 를 `origin_mode fixed` 로 넣는다 → 회차 끝 오차가 다음 출발점에 쌓이지 않는다. 진입은 `start_delay` 3 s 동안 스무스스텝 (lead_speed 0.10 m/s 이하). 30 cm · 30° 밖이면 기준 노드가 출발 거부 → 중단. `--no-anchor` = 예전 origin robot |
| zigzag 복귀 | 끝 자리에서 `shape rotate` 180° (`--return-ctrl`, 기본 mpc). 180° 뒤 zigzag 는 같은 바닥 선을 거꾸로 지나 A 로 돌아온다 (`test_seq_plan.py` 가 `path.at()` 으로 검산, 오차 1e-9 m). fwd·rev 가 번갈아 — 제어기 셋 × 6 이면 제어기마다 3 · 3 |
| 기록 | `/ctrl/active` (선택·무장마다), `/seq/status` (다리마다 start·arm·done·abort), `/figure8/status` 에 `end`·`T_total`·`hold_s` 추가 |

시뮬 확인 (2026-11-28, 도메인 87): s2 zigzag 6 다리 (data 3 + 복귀 3) · s1 circles8 3 회차 모두 완주, 회차 도중 `/mpc/arm false` → 순서 중단·재무장 없음.
bag 대조 10 판: `torque_output` 명령 전부 고른 제어기 토픽에서 (다른 쪽 0 개), 고르지 않은 묶음 무장 true 0 번, PP L_d — zigzag pp·adpp 0.10~0.20 m, circles8 pp 0.30 고정 · adpp 0.15~0.50 m.
실차는 미확인.
