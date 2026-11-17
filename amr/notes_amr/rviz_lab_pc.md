# 랩 PC(192.168.0.209)에서 실차 RViz 보기

작성 2026-11-16. 실차 토크 실험을 랩 리눅스 PC 의 RViz 로 본다.
화면 구성: 연구실 지도, 로봇, 라이다 스캔, 8자 경로(로봇 기준), 지나간 경로, 2D Pose Estimate.

> **아직 시험 안 한 것**
> - 두 컴퓨터 사이 연결은 이 문서 작성 때 시험하지 못했다.
> - RViz 창도 실제로 띄워 보지 못했다. NUC 에 SSH 로 접속한 채 NUC 화면에 rviz2 를 띄우면, 설정 파일 없이도 바로 죽었다(segfault).
> - 시뮬에서 확인한 것: 지나간 경로 발행, TF `map → base_footprint` 25 Hz, RViz 설정 파일 문법, 표시 플러그인 이름 16 개.
> - 처음 하는 판에서 안 되면 맨 아래 "안 될 때" 표를 본다.

## 구성

| 컴퓨터 | 띄우는 것 |
|---|---|
| NUC (192.168.0.132, 로봇) | A 센서 · B 측위 · C CAN + 상태 추정기(+MPC) · E 표시 노드 `torque_viz_node` |
| 랩 PC (192.168.0.209) | `rviz2` 만 |

- 두 컴퓨터는 ROS 2 의 통신 계층 DDS(Data Distribution Service)로 토픽을 주고받는다.
  - 구현은 CycloneDDS 다. NUC `~/.bashrc` 의 RMW(ROS middleware) 설정이 `rmw_cyclonedds_cpp` 이다.
  - 지금까지 실험은 NUC 혼자만 보이게 해 왔다: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS` 없음.
  - 그래서 양쪽 모두 상대 IP 를 `ROS_STATIC_PEERS` 에 넣어야 서로 보인다.
- 표시 노드 `torque_viz_node` 를 NUC 에서 돌리는 이유: 이 노드는 커밋 전이라 NUC 에만 있다. NUC 에서 돌리면 랩 PC 에는 RViz 설정 파일 하나만 있으면 된다.
- 표시 노드가 하는 일:
  - 상태 추정기 출력 `/state_estimator/odom` 으로 TF(transform) `map → base_footprint` 를 낸다.
  - 실험 스택에는 원래 지도 기준 로봇 위치 TF 가 없어서(2026-11-16 bag 의 `/tf` 0 개) 로봇 모델·스캔을 지도에 놓을 수 없었다.
  - 지나간 경로를 `/torque_viz/trail` 로 낸다. 무장할 때마다 비운다.

---

## 0. 한 번만 할 것

### 0-1. NUC 방화벽 (NUC 터미널)
NUC 는 방화벽 ufw(Uncomplicated Firewall)가 켜져 있고, 들어오는 연결을 기본으로 막는다(`DEFAULT_INPUT_POLICY="DROP"`). 랩 PC 연결이 허용돼 있는지 본다.
```bash
sudo ufw status
```
192.168.0.209 를 허용하는 줄이 없으면 그 PC 만 연다.
```bash
sudo ufw allow from 192.168.0.209
```

### 0-2. 랩 PC 프로그램 (랩 PC 터미널)
ROS 2 Jazzy 가 깔려 있다는 전제다.
```bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-rmw-cyclonedds-cpp
```

### 0-3. RViz 설정 파일 가져오기 (랩 PC 터미널)
```bash
mkdir -p ~/amr_viz
scp smoc@192.168.0.132:/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/rviz/torque_real.rviz ~/amr_viz/
```

### 0-4. 랩 PC 짧은 명령 (선택)
랩 PC `~/.bashrc` 끝에 넣으면, 다음부터 `amrrviz` 한 줄로 2-1 의 환경 설정과 RViz 실행을 한 번에 한다.
```bash
amrrviz() (
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=192.168.0.132
  export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
  exec rviz2 -d ~/amr_viz/torque_real.rviz
)
```

---

## 1. NUC 쪽 (랩 PC 에서 SSH 로 연 터미널들)

**모든 터미널에서 맨 먼저** 이 줄을 친다. 이게 있어야 NUC 의 노드가 랩 PC 를 찾는다.
```bash
export AMR_PEERS=192.168.0.209
```

그다음 터미널마다 하나씩 띄운다. 짧은 명령 목록은 `amrhelp` 로 볼 수 있다. 정의는 `src/amr_basic_stack/controller/mpc/tools/amr_aliases.sh` 에 있다.

| 터미널 | 명령 | 하는 일 |
|---|---|---|
| T1 | `amrcan` | CAN 준비 (재부팅 뒤 꼭) |
| A | `amrsens` | 라이다 2 개 + 카메라 IMU |
| B | `amrloc` | 측위 (smoc_lab 지도) |
| C | `amr8 speed:=0.2` | CAN 노드 + 추정기 + MPC, 8자를 무장한 자리 기준으로. `a` 무장, `s`·스페이스 정지 |
| E | `amrviz rviz:=false` | 표시 노드만 (RViz 는 랩 PC 에서) |
| D (선택) | `amrrec <이름>` | bag 녹화 |

- MPC 없이 기본 동작 시험만 할 때는 C 를 `amrbase` 로 띄운다.
- **이미 떠 있던 노드는 `AMR_PEERS` 없이 뜬 것이라 랩 PC 에서 안 보인다. 다시 띄운다.**
- 랩 PC 를 안 쓸 때는 `AMR_PEERS` 를 넣지 않는다(지금까지처럼 NUC 혼자). 꺼진 컴퓨터 IP 가 peer 에 들어 있으면 노드가 뜨는 데 18 s 걸린 적이 있다(NUC `~/.bashrc` 주석).
- 8자 속도:
  - 기본값은 3.0 × 1.5 m, 0.5 m/s (`config/torque_track.yaml`)이다. 최대 곡률이 3.19 1/m 라서 최대 회전 속도 ω = v·κ = 0.5 × 3.19 ≈ 1.6 rad/s 다.
  - 2026-11-16 회전 시험에서 ω 1.2~1.6 rad/s 뒤 추정기가 거짓 속도 0.06~0.10 m/s 를 냈다(`analysis/torque_mpc_real/2026-11-16_rot`).
  - 0.2 m/s 면 ω ≈ 0.64 rad/s 로 그 아래다. 첫 8자는 0.2 m/s 로 시작한다.
- 두 사람 정지 절차(한 명은 로봇을 들 준비, 한 명은 모터 전원 옆)는 그대로 지킨다.

---

## 2. 랩 PC 쪽 (랩 PC 자기 화면의 터미널 — SSH 창 아님)

### 2-1. 환경 설정
0-4 의 `amrrviz` 를 넣었으면 이 단계 없이 `amrrviz` 만 치면 된다.
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=192.168.0.132
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

### 2-2. 연결 확인
세 줄 이상 나와야 한다.
```bash
ros2 topic list | grep -E "map_carto|scan_front|state_estimator"
```

### 2-3. RViz
```bash
rviz2 -d ~/amr_viz/torque_real.rviz
```
랩 PC 에 리포가 빌드돼 있으면(`install_pc`), `rviz2` 전에 `source <리포 경로>/install_pc/setup.bash` 를 해야 로봇 모델 메시가 보인다(`amr_description` 패키지). 없어도 로봇 위치는 `base_footprint` 축으로 보인다.

### RViz 화면 구성 (`torque_real.rviz`)
| 표시 | 토픽 | 색 |
|---|---|---|
| 지도 | `/map_carto` (Cartographer 측위가 냄) | 회색·검정 |
| Cartographer 가 맞춘 라이다 점 (map 좌표) | `/scan_matched_points2` | 흰색 — 지도 벽에 겹치면 측위가 맞은 것 |
| 로봇 모델 | `/robot_description` | — |
| 로봇 위치 축 | TF `base_footprint` | 빨강 x(앞), 초록 y(왼쪽) |
| 앞 라이다 | `/scan_front` | 빨강 |
| 뒤 라이다 | `/scan_rear` | 파랑 |
| 8자 기준 경로 전체 | `/figure8/path` | 보라 |
| MPC 가 보는 앞쪽 0.5 s 기준 | `/torque_mpc/reference_horizon` | 주황 |
| 지나간 경로 (무장 뒤부터) | `/torque_viz/trail` | 초록 |
| SLAM 포즈 (IMU 위치) | `/tracked_pose` | 노랑 화살표, 기본 꺼짐 |

---

## 3. 쓰는 순서

1. NUC: T1 `amrcan` → A `amrsens` → B `amrloc` → C `amr8 speed:=0.2` → E `amrviz rviz:=false` (모두 `AMR_PEERS` 먼저).
2. 랩 PC: `amrrviz` (또는 2-1 → 2-3).
3. RViz 위쪽 도구 **2D Pose Estimate** 를 고르고, 지도에서 로봇이 있는 자리를 클릭한 채 로봇이 향한 방향으로 끌었다 놓는다.
   - `/initialpose` 로 가고, 측위 launch 의 `set_initial_pose_node` 가 Cartographer 궤적을 그 자리에서 다시 시작한다.
   - 흰 점이 지도 벽에 겹치는지 본다.
4. NUC T1: `amrcheck` → **"맞음"** 이 나와야 한다. 안 맞으면 이 도구가 지도에서 위치를 찾아 다시 넣는다.
5. C 터미널에서 `a` 로 무장한다. 지나간 경로(초록)가 비워지고 새로 그려진다. 8자 기준(보라)은 무장한 자리 기준으로 다시 만들어진다.
6. 한 바퀴 돈 뒤 5 s 유지(hold)하고 기준 발행이 끝난다. 지나간 경로는 화면에 남는다.
7. 끝낼 때: C 에서 `s`(또는 스페이스)로 정지 → 각 터미널 Ctrl+C.

---

## 안 될 때

| 증상 | 확인할 것 |
|---|---|
| 2-2 에서 NUC 토픽이 안 나옴 | ① NUC `sudo ufw status` 에 192.168.0.209 허용 ② NUC 노드를 `export AMR_PEERS=192.168.0.209` 뒤에 띄웠는지 ③ 랩 PC 환경 변수 4 개(도메인 81, `rmw_cyclonedds_cpp`, `LOCALHOST`, peers 192.168.0.132) ④ NUC IP 가 192.168.0.132 그대로인지 — Wi-Fi(DHCP)라 바뀔 수 있다. NUC 에서 `hostname -I` |
| 지도가 안 보임 | B(`amrloc`)가 떠 있는지, 랩 PC 에서 `ros2 topic hz /map_carto` |
| 로봇·라이다 스캔이 안 보임 (Fixed Frame 오류) | E(`amrviz rviz:=false`)가 떠 있는지. TF `map → base_footprint` 가 없으면 스캔을 놓을 곳이 없다. 랩 PC 에서 `ros2 run tf2_ros tf2_echo map base_footprint` |
| 로봇 모델에 빨간 오류 | 랩 PC 에 `amr_description` 이 없다. 위치는 축으로 보인다 |
| 지나간 경로가 안 그려짐 | C 의 추정기가 내는지: `ros2 topic hz /state_estimator/odom` (100 Hz) |
| 8자 경로가 안 보임 | C 를 MPC 판(`amr8`/`amrmpc`)으로 띄웠는지. `amrbase` 에는 8자 기준이 없다 |

## 하지 말 것
- `localization.launch.py` 를 같이 띄우지 않는다. `ekf_filter_node` 의 `odom → base_footprint` 와 표시 노드의 `map → base_footprint` 가 겹쳐 TF 가 깨진다.
- `ssh -X` 로 NUC 의 RViz 창을 끌어오지 않는다. NUC 에 SSH 로 접속한 채 NUC 화면에 띄워도 rviz2 가 죽었다. 원격 화면 전송(X11 forwarding)은 그보다 더 불리하다.
