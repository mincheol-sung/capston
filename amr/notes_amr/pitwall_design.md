# pitwall 조사 + AMR 실시간 튜닝 도구 설계 (2026-11-28)

> 사용자 결정 (2026-11-28, 시작 세션 경유): 1~3 추천안 그대로. 4 는 "launch 하나 = bag 하나로, 파라미터가 바뀔 때마다 기록해 bag 하나로 판별" →
> `/figure8/run_config` (판 출발마다 값 전체) · `/figure8/status` (변경마다) · `/torque_mpc/config` (변경·적용·무장마다) + `tools/bag_runs.py`·`compare_slam.py --runs`.
> 쓰는 법: [pitwall_usage.md](pitwall_usage.md).

## 1. unicorn-racing-stack 의 pitwall 이 하는 일

리포: https://github.com/HMCL-UNIST/unicorn-racing-stack (기본 브랜치 jazzy). 파일 경로는 리포 기준.

| 부분 | 파일 | 하는 일 | 적용 방식 |
|---|---|---|---|
| telemetry 라이브러리 | `race_utils/pitwall/include/pitwall/pitwall.hpp`, `src/pitwall.cpp`, `pitwall/__init__.py` | `pitwall::log(key, v)` → `/pitwall/<key>` (`std_msgs/Float64`), `pitwall::event(name)` → `/pitwall/events` (`std_msgs/String`). 구독자가 없으면 publish 안 함 (`get_subscription_count() == 0` 이면 return) | topic |
| 기록 | `launch/record.launch.py`, `scripts/monitor_node.py` | `ros2 bag record -s mcap` 로 `/pitwall/.*` + 센서를 MCAP 하나에. 기록 프로세스 CPU·RSS 를 `/pitwall/monitor/*` 로 | — |
| 라이브 보기 | `launch/live.launch.py` | `foxglove_bridge` (ws://host:8765) → Foxglove/Lichtblick | — |
| RViz 패널 | `include/pitwall/pitwall_panel.hpp`, `src/pitwall_panel.cpp`, `plugin_description.xml` | `rviz_common::Panel` plugin. 위: state banner (`/state_machine` 값으로 색). 가운데: `/pitwall/events` 를 시각과 함께 쌓는 창. 아래: 시뮬 버튼 — `/sim/remove_opponent` (Empty), `/sim/clear_obstacles` (Empty), `/sim/opp_speed_delta` (Float32 ±0.5), `/sim/opp_mode` (String manual·path·ftg), `/sim/ego_lidar_enable`·`/sim/opp_lidar_enable` (Bool), `/vp/inject_mode` (String), `/ego/use_keyboard` (Bool) | topic (버튼 = publish 한 번) |
| 띄우기 | `stack_master/launch/pitwall.launch.xml` | `rviz2 -d pitwall.rviz` + **`rqt_reconfigure`** | — |

pitwall 밖이지만 그 launch 의 `rqt_reconfigure` 가 고치는 대상:

| 노드 | 파일 | 실시간으로 바뀌는 것 | 방식 |
|---|---|---|---|
| controller_manager | `controller/controller/controller_manager.py:152, 215` | L1 게인·FTG 값. `add_on_set_parameters_callback(dyn_param_cb)` 가 `setattr(self.controller, name, value)` | ROS 2 parameter (주석: "ROS 1 dynamic_reconfigure 대체") |
| speed_sector_tuner | `race_utils/f110_utils/nodes/sector_tuner/sector_tuner/sector_tuner.py` | 구간별 속도 배율 `Sector{i}.scaling` (`FloatingPointRange` 0~2, step 0.01). `ParameterEventHandler` 로 받아 `/global_waypoints` 의 `vx_mps` 에 곱해 `/global_waypoints_scaled` 로 다시 발행. `save_params` (bool) 를 켜면 yaml 로 저장 | ROS 2 parameter + topic 재발행 |

정리
- pitwall 자체는 경로·파라미터를 바꾸지 않는다. **보기** (telemetry·event·state banner) 와 **시뮬 조작 버튼** 이다.
- 실시간 튜닝은 pitwall launch 가 같이 띄우는 `rqt_reconfigure` 가 각 노드의 ROS 2 parameter 를 고치고, 노드가 `on_set_parameters` callback 으로 받아 쓰는 구조다.
- 속도 프로파일은 sector_tuner 가 구간 배율로 waypoint 속도를 다시 발행한다. **경로 모양을 실시간으로 바꾸는 기능은 없다.**
- dynamic_reconfigure (ROS 1) 는 쓰지 않는다. ROS 2 parameter 로 옮겨져 있다.

## 2. 우리 스택 지금 (코드에서 확인)

| 노드 | 지금 | 근거 |
|---|---|---|
| `figure8_reference_node` | 파라미터를 기동 때 한 번 읽는다. 부하 예산 검사에 걸리면 `SystemExit` | `figure8_reference.py:207-224` |
| | 완주·중단 뒤 `finished = True` 로 발행을 끝낸다. 다시 무장해도 아무것도 안 한다 → 다음 판 = launch 재시작 | `on_tick` 첫 줄 |
| `torque_mpc_node` | parameter callback 없음. 모든 값을 `declare_parameter` 로 기동 때 읽어 `TorqueMPC` 를 만든다 | `torque_mpc_node.cpp` 생성자 |
| | `TorqueMPC` 생성자는 `H_const`·제약 행렬 크기만 잡는다. 무장할 때마다 이미 `reset()` 한다 → 정지 중에 새로 만들어도 기동과 같다 (다른 점: η̂ 가 시작값으로 돌아감) | `torque_mpc.cpp:201-326` |
| 게이트·fault_monitor | 고장이 나면 `/mpc/arm false`. 다시 'a'·RT 로 무장 가능 (launch 재시작 불필요) | `fault_monitor.py` 머리주석 |
| bag | launch 하나 = bag 하나 (`real_all.launch.py` ⑥). `compare_slam.py` 는 bag 의 모든 무장 구간을 한 판으로 합친다 | `compare_slam.py:69, 149` |

## 3. 설계안

```
 랩 PC                                   NUC (amrall 스택)
 ┌──────────────────┐  parameter set     ┌───────────────────────────────┐
 │ rqt_reconfigure  │ ─────────────────▶ │ figure8_reference_node        │
 └──────────────────┘                    │  on_set: 부하 예산 검사 → 거부/받음 │
 ┌──────────────────┐                    │  받으면 "다음 판" 경로를 새로 만듦  │──▶ /figure8/next_path (미리보기)
 │ RViz             │ ◀───────────────── │  판 끝·중단 뒤 다음 무장 = 새 판   │──▶ /figure8/status (JSON, latched)
 │ (next_path 표시) │                    └───────────────────────────────┘
 └──────────────────┘                    ┌───────────────────────────────┐
 NUC 다른 터미널                          │ torque_mpc_node               │
 ┌──────────────────┐  set_atomically    │  on_set: 허용 목록·TorqueMPC   │
 │ pit (CLI)        │ ─────────────────▶ │  시험 생성 → 거부/받음          │──▶ /torque_mpc/config (JSON, latched)
 │ poly·circ 프리셋 │                    │  정지 중이면 바로 새로 만듦      │
 └──────────────────┘                    └───────────────────────────────┘
```

도구는 **무장하지 않는다** ('a'·RT 는 사람이 직접).

### 적용 시점 (추천안)

| 대상 | 무장 중 (주행) | 판 도중 멈춤 ('s'·B) | 판 끝·중단 뒤 |
|---|---|---|---|
| 기준 경로 (모양·속도·가속·꼭짓점·랩) | 받아 두기만, 지금 판에는 안 들어감 | 받으면 멈춘 판을 버리고 다음 'a' 가 새 판 | 다음 'a' 가 새 판 (무장한 자리 = 출발점) |
| MPC 튜닝 값 | 받아 두기만 | 바로 TorqueMPC 를 새로 만듦 | 바로 새로 만듦 |

### 안전장치

| 장치 | 내용 |
|---|---|
| 부하 예산 검사 | 기동 때와 같은 `peak_load ≤ load_budget·V_LIMIT` (지금 1.0 × 1.550 m/s). 넘으면 parameter set 을 **거부** — 노드 값은 늘 "통과한 다음 판" |
| 실시간 변경 금지 값 | 기준: `dt`·`horizon_steps`·`rate_hz`·`load_budget`·`max_wheel_rad_per_sec`·`abort_error`·`path_follow_lead_s`·topic 들 (read_only). MPC: `Ts`·`Np`·`Nc` (추정기 100 Hz·계산 시간), `limit_mode`·`cap_on`·`linearization` (비교군 정체), safety·hold·topic 값 |
| MPC 시험 생성 | 바꾼 값으로 `TorqueMPC`·슬립 되먹임 이득을 먼저 만들어 보고, 생성자 검사 (예: `w_delta + r_u > 0`) 에 걸리면 거부 |
| 미리보기 | 다음 판 경로를 RViz 에 `/figure8/next_path` 로 (origin robot 이면 지금 로봇 자리 기준, 1 Hz 로 따라감). 'a' 전에 벽과 겹치는지 본다 |
| 기록 | 새 판마다 `/figure8/status` 에 판 번호·값을 남기고, MPC 를 새로 만들 때마다 `/torque_mpc/config` 에 값 → bag 에 남음 |

## 4. 사용자 결정 (구현 전)

1. **UI**: (추천) `rqt_reconfigure` + RViz 미리보기 + NUC 용 `pit` CLI — 새 GUI 코드 없음, 랩 PC 는 `rqt_reconfigure` 만 있으면 됨 / 위 + unicorn 식 RViz 패널 (C++ Qt plugin, 랩 PC 에서 작업공간 빌드 필요 — 지금 랩 PC 는 rviz 설정 파일만 scp 로 씀)
2. **적용 시점**: (추천) 위 표 / 무장 중에는 둘 다 거부 (정지 중에만 받음)
3. **MPC 실시간 대상**: (추천) `W`·`w_dudt`·`r_u`·`q_null`·`mu_hat`·`slip_fb_enable`·`slip_fb_scale`·`slip_eta_enable`·`disturbance_enable`·`disturbance_alpha`·`roller_rr`·`inertia_z` / `W`·`w_dudt`·`slip_fb_*` 만
4. **bag**: (추천) launch 하나에 여러 판 — 판 표식을 bag 에 남기고, `compare_slam.py` 판 나누기는 후속 작업 / 판마다 bag 새로
