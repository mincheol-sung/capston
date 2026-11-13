# Claude Code 세션 인수인계 — AMR 토크 MPC 전환 검토 (2026-10-31 ~ 2026-11-10)

Windows의 Claude Code에서 나눈 대화 전체를 Linux 쪽 Claude Code가 이어받을 수 있게 정리한 문서다. 이 문서 하나만 읽어도 맥락·결정·남은 일을 알 수 있게 썼다.

- 같이 볼 파일: `notes/torque_mpc_todo.md`(누적 결정·과제), `notes/To do.md`(실측·검증 항목만)
- 코드는 이 세션에서 **한 줄도 바꾸지 않았다.** 만든 것은 문서뿐이다(아래 8절).

---

## 0. Linux 쪽 Claude에게 먼저 알릴 것

1. **이 리포(`amr_ws`)는 로봇 NUC에서 돌고 있는 AMR 스택의 복사본이다.**
   - NUC 경로: `~/2026amr_ws`, 사용자 `smoc`.
   - 여기서 고치는 것은 실제 로봇 스택을 고치는 것으로 다룬다.
2. **사용자 요청: 토크 MPC 관련 대화를 할 때마다 `notes/torque_mpc_todo.md`에 변경사항·결정·새 과제를 정리하고, 변경 이력에 날짜와 함께 한 줄 남긴다.** 질문에만 답한 턴이라도 바뀐 게 있으면 기록한다.
3. 이미 정한 사항(아래 4절)은 다시 논쟁하지 않는다. 미정·보류 항목만 다룬다.
4. 브랜치: `fix/semantic-map-jump-dynamic-gate`. 이 브랜치에는 DRA_MPPI 쪽에 커밋 안 된 변경이 많다. 이번 세션과는 무관하다.
   - 노트 폴더는 리포 밖 `제품개발설계/제어기/notes/`로 옮겨져 있다(이 문서의 `notes/`는 모두 이 폴더). git에 들어 있지 않으므로 Linux로는 폴더째 직접 복사해야 한다.
5. 아래 외부 문서는 Windows PC에만 있다. Linux에서 필요하면 따로 복사해야 한다.
   - `제품개발설계/제어기/notes/AMR 제어기 구조도 v3.html`: 최신 구조도
   - `OneDrive/Documents/카카오톡 받은 파일/메카넘 토크 MPC 1판 설계.html`: MPC 세부 설계
   - `제품개발설계/발표/2차 발표/2차 발표_정다솔_AMR슬립제어.pdf`: 2차 발표, 2026-11-03
   - `제품개발설계/회의/1028_교수님 미팅/2026-10-28_교수님 회의_종합본_notion.md`
   - `제품개발설계/회의/1029_5차 회의/제어기_방향_정리.md`
   - 구조도 v2: claude.ai/artifact/X4fkdPvuHnDosLSQctFrQo (v3로 대체됨)

---

## 1. 프로젝트 배경

- 과목: 제품개발설계(3학년 2학기). 지도교수 정다솔. 팀: 성민철(사용자), 이석원, 전준민, 전준서.
- 주제: **메카넘 휠 AMR의 휠 슬립을 바퀴별 토크 제어로 줄이기.**
  - 모터를 RoboClaw 속도제어 모터(30:1, 1320 cpr)에서 **CubeMars AK40-10 KV170 토크 모터**(10:1, 내장 드라이버, CAN, MIT 모드)로 바꾼다.
  - 제어기를 **토크 입력 MPC**로 새로 만든다.
- 일정: 2차 발표 11/03(끝남), 3차 발표 11월 말(포스터), 최종 12/03.
- 하드웨어 BOM(발표 13쪽): AK40-10 × 4, 6S 22.2 V LiPo × 2, CANable 2.0(USB-CAN), CP2102 × 4, 모터-휠 육각축 일체형 허브(3D 프린팅). 센서·NUC·프레임은 기존 것을 쓴다.

---

## 2. 현재 스택 구성 (인지 → 판단 → 제어)

진입점은 `ros2 launch stack_master autonomy.launch.py map:=<맵>`이다. 아래는 기본값 기준이다.

```
[센서]  RPLiDAR S2 ×2 ─ scan_merger ─> /scan         RealSense D455 (RGB-D, IMU → /camera/imu)
        RoboClaw 엔코더 ─> /odom, /measured_wheel_speeds
[인지]  /odom + /imu/base_link ─ EKF(ekf.yaml) ─> /odometry/filtered (odom→base_footprint)
        /scan ─ Cartographer(localization) ─> map→odom, /tracked_pose
        map_server ─> /map ; /map + /scan ─ traversability ─> /traversable_map
        YOLO(/detections) + LiDAR 클러스터(/lidar_clusters) ─ camera_lidar_fusion
          ─> object_tracker(IMM CV/CT) ─> /tracked_object_modes ─ motion_predictor ─> /predictions
[판단]  /goal_pose ─ Theta*(rviz_theta_star_node) ─> /global_path
        DRA-MPPI ← /global_path, /traversable_map, /predictions, /odometry/filtered
                 ─> /dra_mppi/reference_trajectory (PlanarReferenceArray, dt 0.2 s × 20, 10 Hz)
[제어]  바퀴 MPC ─> /wheel_mpc/cmd_raw ─ keyboard_command_gate('a' 무장, 's'/SPACE 정지) ─> /wheel_cmd
[구동]  roboclaw_mecanum_node
```

- **패키지 위치**: `src/amr_basic_stack/` 아래 `sensors/`, `perception/`, `planner/`, `controller/`, `stack_master/`.
- **MPC 구조**는 `mpc_structure` 인자로 하나만 고른다.
  - `body`: 몸체 twist를 내는 구조 A
  - `wheel`(기본): 바퀴 속도를 내는 구조 B, Python
  - `wheel_cpp`: 구조 B의 C++ 판 `amr_mpc_cpp`(100 Hz, DAQP, d 단 dead time). 게이트가 받는 즉시 중계한다.
- **로봇 기하**(`controller/mpc/config/plant_wheel.yaml`): r = 0.04 m, wheel_base 0.1897, width 0.207, k = (wb + w)/2 ≈ 0.198 m.
- **역기구학**(`roboclaw_mecanum_node.py:861` `twist_to_wheels`): FL = (vx − vy − kω)/r, FR = (vx + vy + kω)/r, RL = (vx + vy − kω)/r, RR = (vx − vy + kω)/r.
- **8자 기준궤적**이 이미 있다.
  - 궤적: `amr_mpc/core/figure8.py`(Gerono 렘니스케이트, 시간 재매개화)
  - 노드: `figure8_reference_node`(시작 포즈에 고정하고 `/dra_mppi/reference_trajectory`로 발행)
- **Cartographer 측위 설정**(`perception/amr_perception_bringup/config/localization_2d.lua`, `mapping_2d.lua` 상속):
  - tracking_frame = camera_gyro_frame, pose_publish_period 1e-2(100 Hz), use_pose_extrapolator = true
  - use_odometry = true(지금은 휠 오돔 사용), use_imu_data = true, num_accumulated_range_data = 2(앞·뒤 라이다를 합쳐 360°)
  - provide_odom_frame = false, publish_tracked_pose = true, rotation_weight 10, optimize_every_n_nodes 20, motion_filter 각도 3°
  - 주석에 기록된 실측: 전역 최적화 때 /tracked_pose가 계단처럼 튐(약 2 s 간격), 회전 중 yaw RMS 2.5~3.4°. 이 RMS가 무엇과 비교한 값인지는 주석에 없다.
- **EKF**: `ekf.yaml`은 휠 속도 + 자이로를 융합해 odom 레벨 추정을 한다. `ekf_carto_localization.yaml`은 `/odom` + `/tracked_pose/odom` + IMU를 map 레벨로 융합한다.
- 저장 지도 `smoc_lab`: 326 × 347 px @ 0.05 m(경계 약 16 × 17 m). 자유 영역은 약 78 m²이고 모양이 불규칙하다.

---

## 3. 대화 흐름 (시간순)

### 3.1 리포 소개와 스택 구성 설명 (10/31)
- 사용자가 이 폴더가 NUC 스택의 복사본이라고 알렸다. 메모리에 저장했다.
- 인지·판단·제어 틀로 스택 구성을 설명했다(2절 내용).

### 3.2 회의록(10/28) + 구조도 v2의 타당성 검토 (11/01)

**구조도 v2 요지**
- 토크 입력 MPC 단독, 슬립 제어기 제거, 토크 상한은 MPC 제약
- SLAM 10 Hz + IMU 적분으로 100 Hz 상태
- 슬립률은 로깅 전용, 8자 트랙에서 주행마다 노면 μ 변경
- 새 노드: ak40_can_node(CAN ↔ AK40 × 4, /wheel_states, /wheel_torque_cmd), state_estimator, torque_mpc_node, torque_output(전류 환산 i = (τ + τ_fric)/(K_T·N)), fault_monitor, slip_logger

**10/28 회의 결정**
- 토크 제어로 간다. 슬립은 예상 각속도와 실제 각속도의 차이로 정의한다.
- 뮤·질량은 안다고 가정한다(추정 안 함). 슬립 제어는 토크 상한만 둔다.
- 8자 시간 기준맵을 쓴다. A·B·C 비교 평가를 한다.
- MPC 비용은 XY 오차, 현재 스텝 기준으로 선형화한다.

**검토 결과**
- 토크→힘 행렬 H가 드라이버 역기구학의 전치와 일치함을 확인했다. null space [1,1,−1,−1]과 IMU 적분식 부호도 맞다.
- 그 외 지적: 속도 1·2·3 m/s는 모터 한계상 불가능, μ 식의 √2 누락, 휠 오돔을 빼면 기존 노드 TF가 깨짐, MTTE는 회의에서 제외된 것과 충돌, 속도 리셋 지연, 포즈 튐, 마찰·적분 작용, 선형화 기준점, 노드 지연, 게이트 토크 0 문제, 기준궤적 뒤처짐, 비교군 A의 속도 루프, m·I_z·J_w 측정.
- AK40-10 속도 추정: 10/29 메모 기준 24 V 무부하 435 rpm, 정격 370 rpm. 22.2 V면 약 402 rpm.

### 3.3 사용자 1차 회신 (11/05)

1. **속도 프로파일**: 8자 1바퀴마다 0.5 → 1.0 → 1.5 m/s. 30 m라면 60 + 30 + 20 = 110 s.
   - 검산 결과(30 m 8자, 헤딩 = 접선, 바퀴 속도 = (v + k·v·κ)/r)
     - Gerono 2:1(9.8 × 4.9 m, 최소 곡률반경 1.03 m): 1.0 m/s에서 285 rpm, **1.5 m/s에서 427 rpm**
     - 두 원(R 2.39 m): 1.0 m/s에서 259 rpm, 1.5 m/s에서 388 rpm
   - 1.5 m/s는 22.2 V 무부하 추정치(약 402 rpm)에 너무 가깝다. 두 원 사이 교차점에는 완화곡선이 필요하다.
2. **μ 기반 토크 상한**: 나중에 논의.
3. **휠 오돔을 라이다 오돔으로 대체**: 나중에 논의.
   - 답변: 가능하고, 슬립을 보려면 오히려 필요하다. 휠 오돔은 무슬립을 가정하므로 이걸로 차체 속도를 추정하면 슬립률이 0으로 나온다.
   - 문제는 기존 노드가 `/odom`·odom TF에 의존한다는 것뿐이다. 절충안은 `ekf_carto_localization.yaml`에서 odom0을 뺀 라이다+IMU EKF.
4. **MTTE를 넣고, 토크 상한은 MPC 제약으로 넣는다.**
5. "속도 리셋 지연"이 무슨 뜻인지 질문.
   - 답변: 포즈 차분 속도는 0.05 s 전 값이다. 가속 1 m/s²면 0.05 m/s 오차가 생기고, 리셋할 때마다 이 오차가 다시 들어간다.
   - 정지 중 위치 잡음 1.2 mm를 미분하면 약 1.7 cm/s가 되어 0.5 m/s 구간에서 슬립률 약 3 % 오차다.
6. 2차 발표 PDF 기준으로 다시 검토. 발표와 구조도 사이의 불일치(비교군 C 정의, 적재 조건, 슬립률 정의 부호)를 지적했다.
- 이때 `notes/torque_mpc_todo.md`를 만들었다.

### 3.4 1판 설계 문서 + 사용자 2차 회신 (11/06)

**"메카넘 토크 MPC 1판 설계" 요지** (선배 작성, 2026-11-05)
- 관성 좌표 6상태 x = [x, y, θ, ẋ, ẏ, θ̇], 입력은 절대 순토크 τ(4개)
- 모델: M_eff·Ẍ = R(θ)(T⁻¹)ᵀτ + [μω_z·S·Ẋ_xy; 0]
  - μ = 4J_w/r² = 2.425 kg, m_eff = m + μ, I_eff = I_z + μk²
  - 코리올리 항이 없고, A는 상수 이중적분기다.
- 이산화: ZOH 정확해. B(k)만 R(θ_r(k))로 회전한다(기준궤적 따라 선형화).
- 적분 작용: 잔차 외란 d̂(k) = x̂(k) − A·x̂(k−1) − B·τ(k−1) − E·w. 선택적으로 저역통과 α.
- 비용: 기준 지평 Y_r 추종 + ‖Δτ‖ + q_null(nᵀτ)². ‖τ‖_R은 없다(선회 정상상태 오차 방지).
- 제약: |τ| ≤ τ̄ (상자), |Δτ| ≤ Δτ̄ (띠)
  - τ̄ = min(τ_lim − τ_fric,max, (r/√2)·μ̂·(mg/4)·c_s), c_s = 0.8
  - μ̂ 0.3이면 0.067 / 0.166 / 0.333 N·m (4 / 10 / 20 kg)
- Ts 10 ms, Np 50, Nc 5. 결정변수 20개, 제약 80행. 시작 가중치 W = diag(100, 100, 50, 1, 1, 0.5).
- 측정 목록: m, I_z, J_w(9.7×10⁻⁴), μ̂, τ_fric,max
- 통과 시험: ① 정상상태(등속 선회) ② 하중 변화 ③ 마찰 상한 활성 ④ 플래너 연동

**사용자 지시**
- 비교군과 적재 조건은 발표를 따른다. 슬립률은 구조도 정의를 따른다.

**질문 8개와 답**
1. **포즈 튐·yaw·tracked_pose**
   - 포즈 튐 = 전역 최적화로 map 기준 위치가 계단처럼 바뀌는 것.
   - yaw 2.5°는 추종에는 d̂가 흡수하지만, 1 m/s에서 횡속도 4.4 cm/s, 슬립률 약 4 % 오차를 만든다.
   - tracked_pose = Cartographer가 내는 map 기준 포즈. 100 Hz 중 9/10은 외삽값이다.
2. **마찰**: 바닥 마찰이 아니라 구동계 마찰·구름 저항을 말한 것이었다. 1판의 τ_fric(ω)와 d̂로 해소되었다.
3. **선형화**: 기준궤적을 따라 한다. 1판이 이미 그렇다.
4. **지연 보상**: 실측 → 노드 통합 또는 forward_on_receive → 남는 d 스텝은 τ_prev로 상태를 앞당겨 예측 → 크면 입력 버퍼 증강.
5. **게이트**: 토크 0은 관성 주행이다. MIT 모드 kp = 0, v = 0, kd > 0의 damping으로 정지시킨다.
6. **속도**: 모터 실측 후 결정.
7. **비교군 A**: 퓨어 퍼슛은 속도 명령을 내므로 토크 모터에서는 속도→토크 루프가 필요하다.
8. **측정**: 대부분 측정이지만 일부는 토크 제어가 동작해야 할 수 있는 시험이다.

### 3.5 구조도 v3 검토 (11/06)

**v3 요지**
- v2와 노드·토픽은 같다. torque_mpc_node 내부만 1판 설계로 바뀌었다.
  - 관성 좌표 모델, d̂, ‖τ‖_R 제거 + 기준 미리보기, 마찰 토크 상한 식 확정
- 변화율 제한을 torque_output에서 MPC 제약으로 옮겼다. torque_output에는 안전 클램프만 남는다.
- 안전정지 damping 표기. slip_logger에 d̂·활성 제약 수·QP 반복을 추가 기록.
- 시뮬 통과 시험 3개.

**v3에 남은 불일치**
- 토크 상한을 μ 식으로 확정하고 "MTTE 후보 폐기"라고 적었다.
- 평가 블록이 예전 설계다(저·중속 A vs B, 고속 B vs C 슬립 제어기).
- 질량이 10 / 20 kg이고, 속도가 1·2·3 m/s다.
- MPC 상태가 map 좌표다(1판은 odom {O}).

### 3.6 사용자 3차 회신 (11/06)

- **`/tracked_pose`는 지금처럼 쓴다.** 그래서 상태 좌표는 map이다.
- **속도 리셋 지연의 일반 해법**(답변): 속도를 포즈 미분으로 덮어쓰지 않고, 위치 잔차로 보정하는 선형 칼만필터 / 상보 필터를 쓴다.
  - 예측: a = R(ψ)·a_imu − b, p ← p + vΔt + ½aΔt², v ← v + aΔt
  - 보정: e = p_slam − p̂, p += K₁e, v += K₂e, (b −= K₃e)
  - K₁ = 1, K₂ = 0이면 v3의 "게인 1 리셋"과 같다. 선택적으로 스탬프 지연은 IMU 버퍼로 다시 적분한다.
- **yaw**: 자이로 주도 상보 필터.
  - 예측: ψ̂ += (ω_gyro − b_g)Δt
  - 보정: ψ̂ += K_ψ·wrap(ψ_slam − ψ̂), b_g −= K_b·e. K_ψ 0.1~0.3.
  - ω_z는 자이로 값을 그대로 쓴다. 주행 전 정지 상태에서 바이어스를 평균한다.
- **바퀴별 PI**: (v_x, ω_z) → 역기구학 → ωᵢ,ref. 바퀴마다 τᵢ = K_p·eᵢ + K_i∫eᵢ. RoboClaw 내부 PID를 우리 쪽으로 옮긴 것이다. 비교군 A 전용.
- **토크 상한**: 사용자가 "마찰 한계와 MTTE를 둘 다 고려하면 안 되나"라고 물었다. 가능하다고 답했다.
  - τ̄ᵢ(t_k) = min(τ_lim − τ_fric,max, τ_μ, τ_MTTE,i(t_k))
  - τ_μ는 사전 상한, MTTE는 온라인 보정이다.
  - 주의점 두 가지(나중에 정하기로 함):
    - MTTE 상한이 한 스텝에 Δτ̄보다 크게 떨어지면 변화율 제약과 충돌해 QP 해가 없다.
    - ω̇ 잡음을 걸러야 한다.

### 3.7 사용자 4차 회신 (11/06)
- **리셋 시점**: 라이다 스캔으로 SLAM 값이 들어오면 그 값을 믿는다.
- 속도 보정 필터, yaw 상보 필터, 바퀴별 PI, 토크 상한 병용을 모두 채택했다. 병용 주의점은 나중에 정한다.
- "리셋 지연"을 쉬운 비유로 다시 설명했다: 0.1 s마다 찍는 사진 두 장으로 구한 속도는 0.05 s 전 속도다.

### 3.8 Cartographer 외삽 vs 우리 추정기 (11/06)
- 질문: Cartographer가 100 Hz로 외삽하는 것과 같은 방식인가?
- 답: 아니다.
  - Cartographer: yaw는 자이로, **위치는 일정 속도 가정**(휠 오돔 속도 또는 포즈 차분 속도). 가속도는 쓰지 않고, 속도도 출력하지 않는다.
  - 우리 추정기: IMU 가속도를 적분하고 속도를 출력한다.
- **결정**: state_estimator는 `/tracked_pose` 중 스캔 매칭 직후 샘플만 측정값으로 쓴다. 사이 구간은 IMU 가속도로 직접 채운다. Cartographer 외삽은 RViz·다른 노드용으로 둔다.
- **소스 확인**
  - NUC 버전: ros-jazzy-cartographer 2.0.9004, cartographer-ros 2.0.9003
  - GitHub `ros2/cartographer` 태그 2.0.9004에서 확인
    - `pose_extrapolator.cc` `ExtrapolateTranslation` = 경과시간 × (휠 오돔 속도, 없으면 포즈 차분 속도)
    - `ExtrapolateRotation` = IMU 트래커(자이로)
    - `imu_tracker.cc`: 선가속도는 중력 방향 추정(자세 보정)에만 쓴다.
  - 결정이 맞다는 것을 확인했다.

### 3.9 외부 검토 의견 평가 (11/06)

사용자가 다른 AI의 1판 설계 검토를 붙여 넣었다. 평가 결과:

- **이미 정리된 지적**
  - 기준 출처: v3에서 8자 시간 기준궤적으로 정해짐. figure8 노드가 같은 인터페이스다.
  - 선형화 기준점: 기준궤적으로 정해짐.
  - B vs C: 발표 기준 C는 MPC + 토크 상한이므로, **B는 모터 한계만** 둔다.
  - √2: v3·1판에 반영되어 있다.
  - m: 4 / 9 kg으로 정해짐.
- **새로 유효한 지적**
  - **d̂와 SLAM 보정 스텝의 충돌**: 보정 스텝의 상태 계단을 d̂가 외란으로 읽어 100 ms마다 토크 스파이크가 날 수 있다.
    - 위치 2 mm 계단이 지평 끝에서 최대 약 10 cm(2 mm × 50 스텝).
    - 권장 대응: 보정 스텝에서는 d̂ 갱신을 건너뛰고 직전 값 유지. 필요하면 α ≈ 0.1.
    - IMU 스텝의 d̂는 깨끗한 외란 신호다.
  - **d̂ = 관측기라는 형식 문제**: 킥오프의 "관측기는 캡스톤 2차" 결정과 겹친다. 교수님께 확인한다.
  - **J_w 데이터시트 해석**: 9.7×10⁻⁴는 사이트 값 97.35 g·cm²를 로터 기준 × 100으로 본 값이다. 출력축 기준이면 1/100이 된다.
- **그 검토에 빠진 것**: 토크 상한 병용, 상태 추정 방식 변경.

### 3.10 검증 항목 문서 (11/08)
- 노션 업로드용으로 실측·검증이 필요한 항목만 뽑아 `notes/To do.md`를 만들었다(16개, 5그룹).
- d̂ 계단 오차 표현을 "수 cm"에서 "최대 약 10 cm"로 정정했다.

---

## 4. 확정된 결정 (2026-11-10 기준)

| 항목 | 결정 |
|---|---|
| 최신 구조도 | **AMR 제어기 구조도 v3**. MPC 세부는 1판 설계 문서 |
| 제어 구조 | 토크 입력 MPC 단독. 슬립 제어기(PID)는 범위 밖(종설 후보) |
| MPC 모델 | 1판대로: 관성(map) 좌표 6상태, ZOH, d̂ 적분 작용, ‖τ‖_R 없음, q_null, Np 50 / Nc 5 / Ts 10 ms |
| 선형화 | 기준궤적 θ_r을 따라(B 안의 R(θ_r)) |
| 비교군 | 발표대로. A = 퓨어 퍼슛 + 바퀴별 PI, B = 토크 MPC(상한은 **모터 한계만**), C = MPC + 토크 상한 |
| 적재 | 발표대로 없음 / +5 kg → m = 4 / 9 kg(1판의 4 kg 기준) |
| 슬립률 | 구조도 정의: λᵢ = (ωᵢ − ωᵢ,nom)·sgn(ωᵢ,nom) / max(\|ωᵢ\|, \|ωᵢ,nom\|, ω_ε). 로깅 전용 |
| 토크 상한 위치 | MPC 입력 상자 제약. **식은 마찰 한계 + MTTE 병용(min) 방향**, 세부는 미정 |
| 마찰 | 구동계는 torque_output τ_fric(ω), 나머지는 d̂ |
| 포즈 입력 | `/tracked_pose` 현행 유지(map 프레임). 포즈 튐은 fault_monitor로 감시 |
| 측정값 시점 | 라이다 스캔 매칭 결과가 들어올 때만 측정값으로 보정. 사이 9스텝은 IMU 가속도 적분 |
| 속도 추정 | 위치 잔차 보정 필터(축마다 [p, v](+b), 고정 게인) |
| yaw | 자이로 주도 상보 필터 + 자이로 바이어스 추정 |
| 기준궤적 | 8자 시간 기준궤적. `figure8_reference_node` 재사용, 랩마다 속도를 바꾸는 기능 추가 |

## 5. 보류·나중에 논의

- **속도 프로파일**: 잠정 0.5 / 1.0 / 1.5 m/s. 모터 무부하 속도를 실측한 뒤 확정한다. 발표 목표값(3 m/s 완주 등)도 함께 바꾼다.
- **토크 상한 병용의 주의점**
  - 변화율 제약과의 충돌: 첫 스텝은 상한 우선, l₀ = max(τ_prev − Δτ̄, −τ̄)
  - MTTE ω̇ 필터
  - C를 τ_μ만 / τ_μ + MTTE로 나눌지
- **μ 기반 상한식 세부**와 **휠 오돔 → 라이다 오돔 대체 방식**
  - 후보: odom0을 뺀 EKF, state_estimator가 직접 발행, 실험 전용 launch
  - 어느 쪽이든 Cartographer `use_odometry = false` 측위를 재검증해야 한다.

## 6. 남은 작업

### 문서 수정
- v3: "MTTE 폐기" 문구, 평가 블록(A/B/C), 질량 4 / 9 kg, 속도 문구
- 1판: 좌표계 문구 odom → map, 질량 예시값

### 구현
- ak40_can_node
- state_estimator(보정 필터, yaw 상보 필터, 스캔 매칭 샘플 선택)
- torque_mpc_node(`amr_mpc_cpp` 구조 재사용: DAQP, LTV)
- torque_output
- 게이트 `command_type: torques` + damping 정지
- fault_monitor(포즈 튐, 기준궤적 오차 상한, 모터 에러, timeout)
- slip_logger
- figure8 랩별 속도 프로파일
- 비교군 A 바퀴별 PI
- B / C 상한 설정 분리
- d̂ 보정 스텝 처리

### 실측·검증
`notes/To do.md` 참고.
- 무부하 속도, τ_fric, 작은 토크 정확도(0.067~0.150 N·m, 약 0.1~0.25 A), J_w, damping 정지
- m, I_z, μ̂, 트랙 공간(약 10 × 5 m)
- `use_odometry = false` 측위, 스캔 샘플 구분, yaw RMS의 기준
- d̂ 스파이크, 종단 지연, ω̇ 잡음
- 교수님 확인(d̂의 범위)

## 7. 자주 쓴 수치 (출처: 계산 또는 문서)

- AK40-10: 정격 1.3 N·m, 피크 4.1 N·m, 24 V 무부하 435 rpm, 정격 370 rpm
  - 22.2 V 무부하 약 402 rpm, 25.2 V 약 457 rpm(전압 비례 추정)
  - 토크상수 약 0.59 N·m/A(피크 4.1 ÷ 7 A로 역산, 데이터시트 미확인)
- 직선 1.5 m/s = 358 rpm. 정격 토크로 4 × 1.3 / 0.04 ≈ 130 N.
- τ_μ(μ̂ 0.3, c_s 0.8): 4 kg 0.067, 9 kg 0.150, 10 kg 0.166, 20 kg 0.333 N·m
- μ = 4J_w/r² = 2.425 kg(J_w = 9.7×10⁻⁴일 때), 10 kg 기준 가속도 24 % 차이
- 1판 시험 1의 선회 토크(4 kg, 1 m/s, R 2.4 m): τ = [−0.0167, 0.0167, 0.0167, −0.0167] N·m

## 8. 이 세션이 남긴 파일

| 파일 | 내용 |
|---|---|
| `notes/torque_mpc_todo.md` | 결정·보류·과제·검산 메모·변경 이력. 매 대화마다 갱신 |
| `notes/To do.md` | 실측·검증 항목만(노션 업로드용) |
| `notes/claude_session_handoff_2026-11-10.md` | 이 문서 |

Windows 쪽 Claude 메모리에 저장한 내용(Linux에는 없으므로 여기 옮겨 둔다)
- 이 리포는 NUC 스택의 복사본이다. 수정은 실제 로봇 스택 수정으로 다룬다. Windows에서는 ROS를 못 돌리므로 정적 검토만 했음을 밝힌다.
- 토크 MPC 결정사항(4절)과 보류 항목(5절).
- 토크 MPC 대화마다 `notes/torque_mpc_todo.md`를 갱신하라는 사용자 요청.
