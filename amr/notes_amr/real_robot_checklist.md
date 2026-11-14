# 실차 확인 목록 — 한 번에 진행용 (2026-11-14)

시뮬로는 확인할 수 없고 로봇 앞에서만 확인되는 항목을 `torque_mpc_todo.md`·`To do.md`·리뷰 R8~R10 에서 모아, 진행 순서대로 한 목록으로 정리했다.
**위에서부터 순서대로** 진행한다. 앞 단계가 끝나야 다음 단계가 안전하다. 바퀴가 바닥에 닿는 것은 5단계부터다.
끝난 항목은 여기와 todo 양쪽에 체크하고, 값이 바뀌면 `stack_changelog.md` 에 적는다. 새 실차 항목은 맞는 단계 안에 넣는다.

새 터미널마다 먼저 `amr`. 벤치 도구는 `src/amr_basic_stack/controller/mpc` 에서 돌린다.
**ROS 를 쓰는 터미널(시뮬·4단계 이후)은 `amr` 다음에 `unset ROS_STATIC_PEERS`** — NUC `~/.bashrc` 의 무응답 peer 때문에 로컬 토픽이 수 초씩 밀린다(2026-11-14, 시뮬 IMU 200 → 11 Hz). `.bashrc` 정리는 팀 결정 대기.
**한 실험의 터미널은 모두 같은 `ROS_DOMAIN_ID`** — `.bashrc` 기본 13, 시뮬 때 `export ROS_DOMAIN_ID=81` 한 창을 다시 쓰면 그 창만 81 이라 서로 토픽이 안 보인다(2026-11-14 3단계에서 명령이 노드에 안 닿음). 명령 앞에 `export ROS_DOMAIN_ID=81 &&` 를 붙여 통일하는 게 안전
실차 데이터는 `analysis/torque_mpc_real/<날짜_무엇>/` 한 곳에 모은다 (bag 원본은 `bags/`, 리포 밖).

---

## 0. 전원 넣기 전 (멀티미터)

- [x] 전원 XT30 극성. 설치 지침 표가 "−: 빨강, +: 검정" 으로 관례와 반대라 문서 오류일 수 있다 → 실물로 확인 — 2026-11-14 팀원 벤치(네 모터 동작)로 간접 확인
- [x] 드라이버 입력 16~28 V 안인가 (6S 만충 25.2 V) — 2026-11-14 같은 근거로 간접 확인
- [x] CAN 종단: 전원 끈 상태에서 CAN_H–CAN_L 사이 약 60 Ω (양 끝 120 Ω 두 개) — 2026-11-14 직접 측정은 안 함. 팀원 벤치 FL·RR 111 s 동안 11100/11100 응답으로 간접 확인. 3단계에서 늦은 응답이 계속 늘면 그때 잰다

## 1. 모터 설정 — CubeMars 상위 프로그램 (시리얼 CP2102), 모터마다

- [x] MIT 모드인가 (서보 모드면 전환, 매뉴얼 4.1.1.10) — 2026-11-14 MIT 명령에 네 모터 응답
- [x] CAN ID 1~4 = FL FR RL RR (2026-11-11 벤치 로그로 확인됨 — 바뀌지 않았는지만) — 2026-11-14 status 로 다시 확인
- [ ] **CAN 끊김(sudden interrupt) 설정** — 명령이 끊기면 모터가 스스로 서게. `kill -9`·정전·노드 멈춤 때는 코드가 아무것도 못 하므로 이것이 유일한 안전장치다
  - 2026-11-14 시험: **공장값에는 보호가 없다** — FL 0.05 N·m 중 `kill -9`, 5 s 뒤 응답 v 11.6 rad/s·t 0.0525 N·m (계속 냄). 매뉴얼 V1.0.17 4.1.1.4 Application Function 에 항목이 있다. 상위 프로그램은 Windows 로 보이고, 값을 쓰기 전에 Read Parameters 먼저(4.1.1.5). 설정 뒤 2단계의 끊김 시험을 다시 해서 t 가 ±0.0012 로 떨어지는지 본다. **설정 전에는 3단계를 바퀴 띄운 채 ±0.1 N·m 묶음(`hardware_checked: false`)으로만 하고, `hardware_checked: true`·4단계 바닥·5단계는 설정 뒤에** (2026-11-14 사용자 결정 — 노드가 죽어도 띄운 바퀴가 공중에서 도는 것으로 끝나 벤치와 같은 위험). 결과: `analysis/torque_mpc_real/2026-11-14_bench/RESULTS_KO.md`
  - 2026-11-14 저녁 설정 시도 (전준민, Windows PC + CubeMars Tool V1.32, CP2102 → 모터 3핀 UART, FL·FR 각각) — **설정 못 함, 막힘**. 모터 설정은 아무것도 바뀌지 않았다 (Write·Restore·Mode Switch 안 누름)
    - 매뉴얼 V1.0.18 (공식 사이트, `Downloads\cubemars_manual\`): 4.1.1.4 Application Function = "CAN ID, CAN 통신 속도, CAN sudden interrupt 설정" 한 줄뿐. **MIT 모드에서 적용되는지, 끊기면 어떻게 서는지는 안 적혀 있다.** 4.1.1.5: 쓰기 전에 Read Parameters 필수(안 하면 다른 설정이 망가짐 → 공식 기본 파일을 Import Settings 로 복구). 연결은 전용 **R-link** 기준(19쪽)
    - 연결은 됨(펌웨어 5.1, UUID 읽힘. FL `24 00 46 00…`, FR `1E 00 2B 00…`). 그러나 Read Parameters 가 **기본값만 보인다**: 모터 종류 AK80_3, T_MAX 18 N·m, FR 인데 CAN_ID 1. Application Function 화면의 Timeout 1000 ms·Brake current 0 A 도 기본값으로 보임(1000 ms 가 실제 값이면 오전 kill 시험에서 1 s 뒤 섰어야 한다)
    - MIT Control → DEBUG: `run / calibrate / setup / encoder / origin / exit` 메뉴(오픈소스 mini-cheetah 펌웨어와 같은 구조로 보임). `setup` 입력 → **응답 없음.** COM3 를 직접 열어 921600·115200 bps 로 듣기 + `setup` 보내기 → 0 글자. 같은 시각 CAN status(FL) 는 정상(ID 1, error 0)
    - 추정 원인 (확인 안 됨): ① CP2102 대신 R-link(또는 CH340 — 다운로드 폴더에 AK 2.0 용 CH340 드라이버가 있음)가 필요 / UART 배선·속도 불일치 ② MIT 펌웨어는 시리얼 설정을 지원 안 함
    - 다음: R-link 가 연구실에 있는지 확인 → 있으면 다시 시도. 없으면 CubeMars 에 문의 (MIT 모드 timeout 설정 방법, UART 어댑터·속도). **절대 `calibrate`·`origin` 입력 금지**
  - 대안 (팀 결정 대기, 2026-11-14 전준민 제안): 모터 전원을 직접 끊는 **물리 비상정지 스위치**를 달면 이 항목 대신 바닥 주행 조건으로 쓸 수 있다. 그 전 임시 운용안 = 저속(≤ 0.3 m/s) + 로봇 옆 1 명(바퀴에서 먼 프레임을 잡고 들어 올림) + 전원 차단 담당 1 명 + 넓은 빈 공간. 들어 올려도 모터는 공중에서 계속 돌고 내려놓으면 다시 달리므로 **들어 올린 뒤 전원 차단까지가 한 동작**이다
- [x] 필요하면 MIT 캘리브레이션 (4.2.2) — 2026-11-14 네 모터 위치·속도·토크 응답이 정상이라 지금은 필요 없음. 드라이버 보드를 다시 달거나 위치가 튀면 그때

## 2. 벤치 — 바퀴를 띄우고 모터 하나씩 (ROS 없이)

```
bash tools/setup_can.sh                       # can0 준비 (모터 명령은 안 냄)
/usr/bin/python3 tools/ak40_bench.py status --id 1
```

- [x] status: 4개 모두 응답 (위치·속도·토크·온도·에러 0) — 2026-11-14 error 0, 21~22 °C
- [ ] 가정 확인: 모터 모드 밖에서 MIT 명령에 응답하나 / **명령이 끊기면 마지막 토크를 계속 내나** (1단계 끊김 설정 전후) / 응답 시간 4 ms 안인가 / 에러 코드 종류
  - 2026-11-14: 끊김 → **계속 낸다(설정 전)** 확인. 시험 명령 `(/usr/bin/python3 tools/ak40_bench.py torque --id 1 --tau 0.05 --time 10 & P=$!; sleep 3; kill -9 $P; sleep 5; /usr/bin/python3 tools/ak40_bench.py status --id 1)`. 나머지 셋은 미확인 — 응답 시간은 3단계 "네 모터 응답 시간" 항목으로 넘김(노드가 늦은 응답을 센다). 에러 코드는 운용 중 0 이 아닌 값이 나오면 그 값을 기록. 모터 모드 밖 MIT 응답은 안전과 무관해 보류
- [x] damping: `damping --id N --spin 20` → 서는 시간으로 DAMPING kd(지금 0.02 추정)가 적당한지. 너무 약하면 정지 거리가 길고, 너무 세면 덜컥 선다 — 2026-11-14 네 모터 0.092~0.102 s (예측 0.091), 실제 kd 0.018~0.019. **kd 0.02 유지**
- [x] steps: `steps --id N --taus 0.02,0.04,0.06,0.1 --hold 3` → τ_fric(ω) 곡선. **FR·RL 재시험** (팀원 데이터가 중단된 판) — 2026-11-14 오름 계단은 정지 마찰(출발 0.04~0.1 N·m)에 걸려 점이 모자라 **내림 계단 `--taus 0.1,0.06,0.04,0.03,0.02`** 로 네 바퀴 τc 0.018~0.022, bv 0.0017~0.0021. 지금 `torque_plant.yaml` 값과 고속에서 ±30 % 차이 → 반영은 PC 쪽 결정
- [x] 작은 토크 정확도, 12 비트 양자화(한 칸 2.44 mN·m, "0" 이 +1.22 mN·m) — DAMPING 중 무부하 바퀴가 기어가는지(가짜 모터 0.056 rad/s) — 2026-11-14 응답 t·v 모두 반 칸 오프셋 양자화 확인. DAMPING 3 s 동안 위치 변화 ≤ 0.0004 rad → 안 기어감(쿨롱 마찰 약 20 mN·m ≫ 1.22)
- [x] torque: 무부하 최고 속도. V 범위 45.5 rad/s 에 가까운지 (넘으면 응답 속도가 잘린다) — 2026-11-14 내림 계단 0.1 N·m 에서 약 41~42 rad/s 로 전압 한계(응답 τ 가 명령보다 최대 11 mN·m 낮음). 범위 안. 운용 상한 38.7 rad/s 와 여유 약 7 %
- [x] **측정 마찰 반영 (5단계 전)** — 2026-11-14 NUC 에서 반영: friction = 실측, 보상 × 0.9 (`compensation_scale`), 시뮬 B 12.6·A 5.1 mm. 원래 항목: 2026-11-14 내림 계단 바퀴별 τc·bv 를 `config/torque_plant.yaml` friction 에 넣을지·얼마나(워밍업 15 %, 약간 작게) 정하고 시뮬 재확인. 지금 값이면 38.7 rad/s 에서 보상이 FL·RR 약 30 % 부족, FR·RL 과보상
- [ ] (메모) 드라이버의 K_T·기어 효율 가정은 응답 토크로는 못 잰다 — 관성을 아는 가속 시험 `τ ≈ J·α + τ_fric` 로 비교

## 3. 네 바퀴 장착, 바퀴 띄운 채 — CAN 노드만

```
bash tools/setup_can.sh                       # 2단계에서 했으면 생략
ros2 run amr_mpc ak40_can_node                # 터미널 1. 켜자마자 DAMPING
ros2 topic pub -r 50 /wheel_torque_cmd amr_interfaces/msg/WheelTorqueCommand "{mode: 0, torque: [0.05, 0.05, 0.05, 0.05]}"   # 터미널 2. Ctrl+C 로 멈추면 0.2 s 뒤 DAMPING
```

- [x] **장착 부호**: +0.05 N·m 에서 네 바퀴가 모두 **물리적으로 전진 방향**으로 도는지 눈으로, 그리고 `/wheel_states.velocity` 4개가 모두 + 인지. 아니면 `config/ak40.yaml` `direction` 고침 — 2026-11-14 **+0.08 N·m**(정지 마찰 때문에 0.05 로는 FL·RL 이 안 돎)에서 네 바퀴 모두 눈으로 전진 방향, velocity FL 35.5·FR 28.3·RL 31.6·RR 33.9 rad/s 모두 + → `direction: [-1, 1, -1, 1]` 맞음. 주의: 같은 direction 을 명령과 응답에 곱하므로 velocity 부호는 direction 이 틀려도 늘 + — **눈 확인이 판정 기준**
- [ ] 위가 끝나면 `config/ak40.yaml` `hardware_checked: true` — 그 전에는 노드가 토크를 ±0.1 N·m 로 묶는다. **1단계 CAN 끊김 설정이 끝난 뒤에만**
- [x] 100 Hz 가 흔들리지 않는지 (`ros2 topic hz /wheel_states`, `/ak40_can/status` 5번째 값 = 틱 ms). 흔들리면 송수신을 C++ 로 옮기는 것 검토 — 2026-11-14 DAMPING 중 100.002 Hz, 간격 표준편차 0.05 ms, 틱 평균 0.79 ms·최대 1.34 ms → C++ 불필요
- [x] **네 모터 응답 시간**: `ros2 topic echo /ak40_can/status` 의 4번째 값(응답 수)이 늘 4 인지, 7번째 값(누적 늦은 응답)이 계속 늘어나는지. 늘 늦으면 `config/ak40.yaml` `reply_wait` 를 0.004 → 0.008 정도로 (10 ms 주기 안). 늦은 응답은 다음 주기에 반영돼 멈추지는 않지만 바퀴 상태가 10 ms 늦어진다
  - 2026-11-14 `candump` 로 잰 응답 도착(명령 송신 뒤): FL 0.60·FR 0.65·RL 1.13·RR 1.17 ms(최대 1.22) — **버스는 4 ms 안에 다 온다.** 그런데 늦은 응답 수는 주기마다 정확히 2 씩 늘었다(11 s 동안 175 → 2407). 원인은 노드 쪽: 지난 주기에 늦게 온 RL·RR 응답을 `take_pending` 이 이번 주기 fresh 로 세서, FL·FR 새 응답(0.6 ms)만으로 4 개가 차 `collect` 가 0.7 ms 에 끝나고, RL·RR 새 응답(1.1 ms)은 또 다음 주기로 밀린다 — 한 번 생기면 계속 유지된다. **RL·RR 바퀴 상태가 늘 한 주기(10 ms) 늦다.** `reply_wait` 를 늘려도 안 고쳐진다(기다리기 전에 끝나므로). → 같은 날 NUC 에서 고침(collect 는 이번 주기 응답만 셈). 고친 뒤 실차 11 s: 늦은 응답 4 → 4(안 늘어남), 틱 평균 1.24 ms, 99.999 Hz
- [x] **kill 로 꺼도 서는지**: 토크를 주는 중 다른 터미널에서 `pkill -TERM -f ak40_can_node` → 바퀴가 먼저 제동(DAMPING 0.5 s)된 뒤 풀리는지, 로그 "DAMPING 뒤 모터 모드 해제" — 2026-11-14 0.08 N·m 로 도는 중 SIGTERM → "틱" 하고 제동 후 정지, 화면에 `DAMPING 뒤 모터 모드 해제` (이 줄은 print 라 로그 파일엔 안 남는다). `pkill -f` 가 `ros2 run` 껍데기도 같이 죽여서 "Terminated" 가 먼저 찍히고 프롬프트 뒤에 해제 줄이 나온다 — 정상
- [x] **끌 때 송신 실패**: 팀원 벤치에서 끌 때 "No buffer space available"(slcan 송신 버퍼 넘침)가 한 번 나왔다. 종료 로그에 "CAN 송신 실패" 가 있는지. 모터 모드 나오기가 빠지면 모터는 DAMPING 상태로 남는다(안전한 쪽) — 2026-11-14 SIGTERM 1 회, "CAN 송신 실패" 없음 (한 번뿐이라 가끔 나는지는 계속 본다)
- [x] **모터 하나 전원을 껐다 켜기**(노드 켠 채): 그 바퀴 error 255 → 전원이 돌아오면 노드가 다시 모터 모드에 넣어 응답이 돌아오는지. 돌고 있는 모터가 모터 모드 명령을 다시 받았을 때 튀지 않는지
  - 2026-11-14 (정지 상태만, 도는 중 빼기는 손 위험으로 생략): RR 전원 뺌 → 40 ms 뒤 RR error 255, 나머지 셋은 정상 ✅. **다시 꽂는 순간 FL·FR·RL 도 50 ms 안에 모두 응답 끊김, 노드 재진입 명령에도 33 s 동안 안 돌아옴** ❌. 모터 LED 는 모두 켜져 있었고, CANable2 를 다시 열자(아래 복구) 네 모터 모두 응답 → **CANable2 가 bus-off 로 멈춘 것**(RR CAN 칩이 켜질 때 잡음으로 추정). slcan 펌웨어는 bus-off 를 스스로 풀지도, 리눅스에 알리지도 않아 `can0` 은 UP·ERROR-ACTIVE 로 보인다
  - **복구**: `sudo ip link set can0 down` → `sudo pkill slcand` → `bash tools/setup_can.sh` (slcand `-c` 가 꺼질 때 채널 닫기, `-o` 가 켤 때 열기). 안 되면 CANable2 USB 를 뽑았다 꽂고 같은 절차
  - **운용 규칙**: 모터 전원 커넥터는 로봇 전원을 끈 상태에서만 꽂는다. `status` 가 네 모터 모두 "응답 없음" 인데 LED 가 켜져 있으면 먼저 위 복구
- [ ] (도는 중 모터 하나 전원 빼기 — 손 위험으로 생략. 필요하면 원격으로 끊을 수 있는 스위치를 단 뒤)

## 4. 센서·측위·전체 스택 — 로봇 정지 상태 (바퀴는 띄운 채 또는 바닥에 두고 무장하지 않음)

터미널 셋에 차례로 (각각 `amr` 먼저):
```
ros2 launch amr_bringup sensors.launch.py
ros2 launch amr_perception_bringup cartographer.launch.py mode:=localization map:=<지도> lua_basename:=localization_torque_2d.lua
ros2 launch amr_mpc torque_experiment.launch.py plant:=real use_rviz:=false
```
마지막 launch 가 robot_state_publisher(로봇 모델 TF)도 같이 띄운다. **`bringup.launch.py`(RoboClaw 구동 노드)·`localization.launch.py`(IMU 회전 노드가 하나 더 뜬다)는 띄우지 않는다.** `armed:=true` 는 실차에서 거부된다.

- [x] **세 터미널 모두 `unset ROS_STATIC_PEERS`** (또는 `.bashrc` 정리 뒤). 안 하면 IMU·포즈가 수 초씩 늦게 와서 추정기가 출력을 멈춘다(2026-11-14 시뮬에서 재현) — 2026-11-14 19:17 세 launch 모두 `unset` + 같은 `ROS_DOMAIN_ID` 로 실행, 아래 주기 정상. 참고: 비대화형 ssh 로 띄우면 `.bashrc` 를 안 읽어 RMW 가 FastDDS 로 바뀐다 → `RMW_IMPLEMENTATION`·`CYCLONEDDS_URI` 를 `.bashrc` 에서 가져와 맞춤
- [x] **전면 라이다 연결** — 2026-11-14 19:17 해결 (아래 기록). 두 라이다 health 0, 각 10 Hz — 2026-11-14 4단계(책상 위, 바퀴 띄움, 무장 안 함)에서 전면 RPLIDAR 가 USB 에 아예 안 잡힘: CP210x 장치가 후면(`fc55e23e…` → `/dev/lidar_rear`) 하나뿐, `/dev/lidar_front`(시리얼 `f2d6a1d4…`) 없음, `rplidar_front` 가 `code: 80008004` 로 바로 죽음 → `/scan_front`·`/scan` 0, `/tracked_pose`·추정기 출력 0. USB 선·어댑터·전원 확인 뒤(`lsusb | grep -c CP210` 가 2, `ls /dev/lidar_*` 에 front) 4단계 다시. 참고: `/dev/ttyUSB0`(CP2102N)은 후면 라이다다 — CubeMars R-Link 가 아니다
  - 2026-11-14 저녁 (전준민, `~/capston_jjm` = `b38dea1` 빌드, `ROS_DOMAIN_ID=42`, `.bashrc` 의 CycloneDDS 설정, `unset ROS_STATIC_PEERS`, 무장 안 함): 전면 USB 를 **본체 포트 3-9** 로 옮기자 인식됨(`/dev/lidar_front`, S/N 99A3…). 단 **CAN 어댑터가 쓰던 포트**라 CANable2 는 3-5 로 옮김 → can0 재준비(`sudo pkill slcand` → `setup_can.sh`). 모터 전원이 꺼진 동안 보낸 프레임 때문인지 can0 가 응답 0 이었고, **배터리를 다시 연결하자** 네 모터 응답(error 0, 23 °C)
    - 1회차 18:52: 노드 22 개 중복 없음, `/imu/base_link` 200 Hz, `/wheel_states` 100 Hz·error 0·fresh 4, `/scan_rear` 10 Hz, CAN setup "already UP", Ctrl+C 종료에 "DAMPING 뒤 모터 모드 해제"·남은 프로세스 없음. **전면은 `health status : 2`(internal error, "reboot the device") 로 죽음** → `/scan`·`/tracked_pose`·추정기 출력 없음
    - 2회차 18:56 (전면 USB 뽑았다 꽂은 뒤): **전면·후면 둘 다 `health status : 2`** (1회차엔 후면 정상). S/N·펌웨어 1.02 는 둘 다 읽힘 → USB 통신은 되고 라이다 자체가 오류 상태. 원인 미확인 — 라이다 전원 공급(USB 5 V 인지 배터리 쪽 레귤레이터인지, 배터리를 뺐다 꽂은 것과 관계있는지) 확인 필요
    - 원인 후보: 그 직전에 **모터 배터리를 NUC 용 배터리 단자에 잘못 꽂았다**(사용자, 바로 원래 자리로). 라이다는 NUC 쪽 전원에서 레귤레이터를 거쳐 받는다(사용자 확인). RPLIDAR 는 USB 를 다시 꽂아도 전원이 안 끊기므로 health 오류가 안 풀렸다
    - **해결 19:03: NUC 를 정상 종료(`sudo poweroff`) 후 다시 켬** → 라이다 전원이 리셋되어 두 라이다 health 0. NUC 재부팅 뒤 can0 는 다시 `setup_can.sh`(sudo) 필요
    - **3회차 19:17 (4단계 전체 정상)**: 노드 23 개 중복 없음, `/scan_front` 10.0·`/scan_rear` 10.0·`/scan` 10.0·`/tracked_pose` 10.0·`/imu/base_link` 200.0·`/state_estimator/odom` 100.0·`/wheel_states` 100.0 Hz, 바퀴 error 0·fresh 4. 정지 상태 `/tracked_pose` (−0.08, 0.05) m, 추정 속도 (−0.006, 0.001) m/s, 포즈 튐 0. Ctrl+C 종료 "DAMPING 뒤 모터 모드 해제", 남은 프로세스 없음
    - 남은 관찰: `dual_scan_merger` "라이다 stamp 차이 0.09 s > 0.05 s — 오래된 scan 제외" 가끔(40 s 에 2 회). `ekf_carto` 가 "Could not obtain transform odom->base_footprint" 를 5 s 마다 냄 — 토크 스택(추정기)은 이 EKF 를 쓰지 않아 4단계 판정엔 영향 없음, 꺼도 되는지 팀 확인
    - 주의: 같은 시각 다른 PC(192.168.0.213)가 NUC 에 ssh 로 Cartographer 빌드(`~/amr_timing_validation_20261114`)를 돌렸다. 한 번에 한 사람만 노드를 띄우기
- [x] CAN 자동 준비: launch 첫 줄 로그 "CAN setup: …". sudo 암호를 묻는 설정이면 실패한다 → 먼저 `sudo -v`, 또는 손으로 준비하고 `auto_setup_can:=false` — 2026-11-14 `CAN setup: can0 already UP` (sudo 없이 통과)
- [x] TF 가 있는지: `ros2 run tf2_ros tf2_echo base_footprint camera_imu_optical_frame` 가 값을 낸다 (없으면 imu_to_base 가 "TF 대기 중" 에서 멈추고 추정기가 "IMU 0 개") — 2026-11-14 (0.098, 0.017, 0.140) m, RPY (−90°, 0, −90°). 추정기 IMU 기준점 (0.098, 0.017) 과 같다
- [x] 노드가 겹치지 않는지: `ros2 node list` 에 imu_to_base_node·robot_state_publisher 가 하나씩만 — 2026-11-14 22 개 노드 모두 하나씩
- [x] `ros2 topic info -v /imu/base_link` — 발행 `imu_to_base_node` BEST_EFFORT, 구독 `state_estimator_node` BEST_EFFORT — 2026-11-14 맞음 (구독에 `ekf_carto` 도 BEST_EFFORT 로 붙어 있음)
- [x] IMU 주기: `ros2 topic hz /imu/base_link` 가 D455 주기 그대로(두 배가 아님), 가속도 값이 몇 표본마다 바뀌는지 (D455 는 가속도·자이로 주기가 다르다) — 2026-11-14 `/camera/imu` 200.6 Hz → `/imu/base_link` 200.6 Hz (두 배 아님), 수신−stamp 0.8 ms. 다시 띄워 10 s: `/camera/imu` 200.4 Hz 중 **가속도는 2 표본마다 바뀐다(100.2 Hz 갱신, 같은 값 2 개 이어짐 970 회 / 1·3·4 개 33 회)**, 자이로는 매 표본. 즉 가속도는 5 ms 마다 같은 값을 한 번 더 낸다(zero-order hold)
- [x] 추정기 로그 "준비 끝. 바이어스 …" 가 나온다. "IMU 0 개", "SLAM 포즈 0 개" 오류가 없다 — 2026-11-14 정지 바이어스는 2 s 안에 잡힘("바이어스 아직" 은 첫 검사 한 번), "IMU 0 개" 없음. "SLAM 포즈 0 개" 는 전면 라이다 때문 → 라이다 연결 뒤 다시 — 19:17 "준비 끝. 바이어스: 가속도 (−0.102, +0.063) m/s², 자이로 −0.0012 rad/s", "SLAM 포즈 0 개" 없음
- [x] 추정기 로그에 시각 경고가 없다: "IMU stamp 가 시스템 시각과 … 어긋남", "SLAM 포즈 stamp 가 IMU 시각보다 … 미래", "너무 오래된 stamp 의 포즈" (RealSense `global_time_enabled` 기본 켬에 기대고 있다) — 2026-11-14 170 s 동안 IMU stamp 경고 없음. 포즈 쪽 경고는 포즈가 없어 미확인 — 19:17 포즈 포함 약 90 s 동안 세 경고 모두 없음
- [x] `/wheel_states` 의 error 가 네 바퀴 모두 0 — 2026-11-14 100.2 Hz, 에러 로그 없음 (다른 ROS 프로세스가 켜지고 꺼질 때 `응답 3/4`·`2/4` 단발 2 회)

## 5. 바닥 주행 — 낮은 속도부터, 무장은 키보드 `a`, 정지 `s`·스페이스

- [ ] **무장 순서**: `a` 는 4단계의 "준비 끝" 과 error 0 을 본 뒤에. 그 전에 무장하면 fault_monitor 가 바로 세운다(정상 동작 — 모터 응답 없음 255, 상태 안 옴)
- [ ] 종단 지연: `/ak40_can/status` 3번째 값 [ms] (토크 모드에서만 나온다)
- [ ] **IMU 장착 y 좌표**: 제자리 회전에서 추정 속도가 0 근처인가 (설정 +0.017 m 는 9/23 bag 값, 지금 URDF 로는 −0.030 m) — 2026-11-14 실차 TF `base_footprint → camera_imu_optical_frame` 는 y = +0.017 → 지금 URDF 와 설정이 같다. 제자리 회전 확인은 그대로
- [ ] AK40 교체 뒤 IMU 진동 잡음 재측정 (지금 0.95 m/s² 는 RoboClaw 시절 bag) — 추정기 잡음 설정과 MPC 튜닝이 이 값에 기대고 있다 — 정지 기준값(2026-11-14 책상 위, 모터 DAMPING): `/imu/base_link` ax σ 0.0156 · ay σ 0.0168 m/s², wz σ 0.0031 rad/s, 평균 ax −0.222 m/s²(받침 기울기 약 1.3°)·wz −0.0017 rad/s. 주행 중 값과 비교
- [ ] **SLAM 포즈 튐으로 서는 횟수**: 실차 Cartographer 가 스캔 매칭을 고칠 때 0.15 m·15° 넘게 튀면 fault_monitor 가 세운다(`pose_jump`). 시뮬에는 튐이 없다. `/fault_monitor/fault` 에 pose_jump 가 자주 뜨면 문턱(`torque_estimator.yaml` jump_xy·jump_yaw_deg)이나 `trip_on_pose_jump` 를 정한다
- [ ] 주행 중 갑자기 서면 `/fault_monitor/fault` 의 이유부터. 배터리가 낮으면 드라이버가 에러 4(저전압)를 낸다
- [ ] 마찰 보상이 하중 실린 바닥에서도 맞는지: 일정 속도 직진에서 d̂ x ≈ 0 (또는 PI 적분이 0 근처). 약 10 rad/s 이상은 외삽 값이다 — 2026-11-14 무부하 내림 계단으로 41 rad/s 까지 쟀다(바퀴별 값은 `analysis/torque_mpc_real/2026-11-14_bench/`). 바닥 하중에서는 롤러 마찰이 더해진다
- [ ] 슬립률 잡음 바닥: 슬립이 없을 속도(0.2 m/s 직진)에서 slip_logger 요약의 |λ| RMS. 시뮬 추정기 잡음에서는 0.13~0.27 이었다. 실차 IMU 진동이 다르면 noise_floor(0.1)를 다시 정한다
- [ ] **저속 출발**: 무부하 정지 마찰이 바퀴마다 0.04~0.1 N·m 로 운동 마찰(약 0.02)의 2~5 배이고, 마찰 보상은 ω = 0 에서 0 이다. 출발이 굼뜨거나 바퀴마다 출발 시각이 달라 틀어지는지
- [ ] **DAMPING 정지 순간 슬립**: kd 0.02 의 첫 감속 추정 4.6 m/s²(4 kg 추정) > μ 0.3·g 2.9 m/s². 미끄러운 바닥에서 정지할 때 바퀴가 미끄러지는지(slip_logger)
- [ ] 비교군 A 의 PI 게인: 저속 정지 마찰 근처에서 적분 떨림(limit cycle) 없는지
- [ ] **최고 속도 여유**: 무부하 최고 약 41~42 rad/s(2026-11-14, 전압 한계) vs 운용 상한 38.7 rad/s(1.55 m/s) — 여유 약 7 %. 바닥 하중·배터리 저하에서 1.5 m/s 랩이 바퀴 속도 한계에 걸리는지(응답 τ 가 명령보다 낮아지는지)
- [ ] 실제 트랙 좌표: smoc_lab 지도 위 8자 자리 → `config/torque_track.yaml` `origin_x/y/yaw_deg`, 바닥 테이프

## 6. 끝낼 때

- [ ] 세 launch 를 Ctrl+C 로 끄고 로그에 Traceback·"process has died" 가 없는지, ak40 "DAMPING 뒤 모터 모드 해제"
- [ ] slip_logger 요약이 찍혔는지 (`log_prefix` 를 줬으면 CSV 도)

## 7. 측정값 (발표용, 시간 나면)

- [ ] m: 저울, 적재 조건마다
- [ ] I_z: CAD 또는 제자리 회전 가속 시험
- [ ] μ̂: 노면마다 토크 램프 시험
- [ ] 8자 트랙 공간 약 10 × 5 m + 여유
