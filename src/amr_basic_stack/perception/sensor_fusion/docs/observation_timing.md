# 원본 관측·시각 계약과 자기 운동 보정

## 2026-11-06 현재 소스 재검토

기존 공통 시각 변환과 원본 스캔/점 대응은 유지했다. 아래의 이전 구현 기록과
충돌하는 운용 설명은 이 절이 우선한다. 대기 큐·IMM·연결 게이트는 변경하지 않았다.

| 경계 | 좌표계 및 좌표 기준 시각 | 소비 방식 |
|---|---|---|
| 원본 LiDAR | `laser_front/rear`, 수신한 scan header | 실제 드라이버는 host grab 시작 시각. 빔 측정 시각 보장 없음 |
| LiDAR 대표점 | `base_link`, 검증된 sweep의 끝 / unknown snapshot의 scan header | `base_link(t_ref) ← odom ← sensor(t_beam)`로 변환 후 대표점 계산 |
| RGB-D 대표점 | `base_link`, depth header | color/depth stamp·optical frame 일치 필수. 원본 두 header 별도 보존 |
| 융합 결과 | `base_link`, batch에서 가장 늦은 좌표 기준 시각 | 입력별 과거 TF로 정렬 후 비교·융합. 배열/객체 header 불일치 거부 |
| 객체 트랙 | `odom`, 융합 배열 header 시각 | 배열 시각 TF 사용; 객체 header 일치 검사 후 기존 IMM 갱신 |
| 추적 관측 연결 | 배열은 추적 갱신 시각, 원본 관측 header는 불변 | 센서·프레임·원본 scan stamp와 원본 인덱스로만 마스킹 |

보강한 처리:

- 추적기의 빈 frame을 `odom`으로 간주하던 identity fallback 제거. 0 시각 TF도 거부한다.
- fusion/tracker의 원본 나이 검사는 배열뿐 아니라 각 객체의 provenance도 확인한다.
  새 배열 header가 오래된 개별 관측을 감추지 못한다. 추적 배열의 미래 시각도 별도로 거부한다.
- RGB-D는 ATS 허용폭 안이어도 서로 다른 stamp/frame이면 `rgbd_unaligned`로 폐기한다.
  시간 재투영 없이 과거 color ROI를 다른 depth 영상의 동일 픽셀에 적용하지 않는다.
  `rgbd_max_delta`는 시간 보정이 아닌 추가 거부 한계다.
- 카메라 `timing_mode=unknown`이 기본이다. 동일 header만으로 노출 시각 검증을 주장하지
  않는다. `unknown_timing_policy=snapshot`은 header 시각의 근사 관측을 유지하고
  `TIMING_UNKNOWN`을 발행한다. 검증된 노출만 허용하려면 `drop`을 사용한다.
  `timing_mode=capture`는 실제 노출 시각이 검증된 입력에만 명시적으로 설정한다.
- 반경/재클러스터링 필터는 `scan_timing_mode=unknown`이면 원본 scan을 통과시키고
  `scan_timing_unknown`을 보고한다. 검증된 `ros`/`reversed`에서만 빔 시각 TF와 기존
  제한된 CV 외삽을 사용한다. 원본 클러스터 재사용은 빔 시각 추정이 필요 없으므로 유지한다.

실제 드라이버 확인:

- RPLIDAR 설치 실행 파일은 workspace build를 가리키며 CMake source 경로도 현재
  `sensors/rplidar_ros`다. 현재 `rplidar_node.cpp:230`의 발행 코드와 `:467` 이후 수집
  코드에서 host timestamp, 각도 정렬/복제, reverse 및 flip-X에 따른 순환 이동을 확인했다.
  양의 `time_increment`만으로 실제 빔 시각을 복원할 수 없어 unknown을 유지했다.
- 설치된 RealSense ROS 패키지는 **4.58.4**, librealsense는 **2.58.4**다.
  같은 버전의 [공식 frame_callback 소스](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/src/base_realsense_node.cpp#L511)는
  frameset에서 계산한 공통 `t`를 color와 aligned depth의 `publishFrame`에 전달한다.
  [필터 입력 경로](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/src/rs_node_setup.cpp#L128)는
  depth alignment 같은 필터가 활성화되면 frameset 동기화 경로를 사용한다.
  따라서 image header가 같아도 독립 노출 시각이 같다는 증거가 되지 않는다.

남은 한계: RPLIDAR의 실제 빔별 acquisition epoch, RealSense frameset 안의 독립
color/depth 노출 시각 및 센서 간 clock offset은 현재 메시지만으로 복원할 수 없다.
snapshot은 이러한 오차를 보상했다는 의미가 아니며, strict 운용은 각 upstream의
`unknown_timing_policy=drop`을 사용해야 한다. 객체 자체 운동의 신뢰할 속도 prior는
사용하지 않는다. 기존 시간차 공분산은 불확실성을 나타낼 뿐 물체를 이동시키는 보정이
아니다. TF가 없으면 기존 bounded wait 후 폐기하며 최신 TF로 대체하지 않는다.

합성 검증에는 서로 50ms 다른 LiDAR/카메라 관측, 병진·회전하는 로봇과 고정 타깃을
실제 fusion→tracker 코드로 통과시키는 검사를 추가했다. 공통 시각 변환 뒤 odom 위치와
속도가 고정되고 원본 provenance가 유지되는지 확인한다. 검증된 역순 빔의 TF 시각,
잘못된 header 거부, unknown 기하 마스킹 통과도 기능 테스트로 검사한다.
현재 소스를 직접 불러 관련 13개 테스트 파일을 실행한 결과 **176 passed**였다.
이 결과에는 이전 단계의 센서/scan/원본 인덱스 대응과 메타데이터 보존 회귀 검사도
포함된다. 실차 노출·빔 시각이나 주행 성능을 측정한 결과는 아니다.

## 이전 구현·검증 기록 (2026-11-01)

구현·검증일: 2026-11-01, ROS 2 Jazzy / Python 3.12.

**현재 실차 RPLIDAR의 빔 시각은 검증되지 않았다.** 기본 설정은 이를
`TIMING_UNKNOWN`으로 발행하는 snapshot 모드이다. 빔별 deskew 구현과 합성
수치 검증은 완료했지만, 기본 실차 입력의 intra-scan 왜곡이 해결됐다고
주장하지 않는다. 원본 scan을 보존하고, 잘못된 시각을 만들어 넣지 않는다.

1. 변경 파일과 역할

| 파일(저장소 `amr_basic_stack/` 기준) | 목적 |
|---|---|
| `perception/amr_interfaces/msg/ObservationProvenance.msg` | 원본 topic/sensor ID, frame, header stamp, 관측 ID, beam index, 빔 offset, timing 상태 |
| `perception/amr_interfaces/msg/DetectedObject.msg` | 원본 관측 목록과 실제 LiDAR deskew 여부 추가; header는 좌표 기준 시각 |
| `perception/amr_interfaces/msg/DetectedObjectArray.msg` | 빈 입력에서도 원본 레코드 식별 가능; 배열 header와 객체 header 일치 |
| `perception/amr_interfaces/CMakeLists.txt` | 새 메시지 생성 |
| `perception/sensor_fusion/sensor_fusion/observation_time.py` | 검증된 timing 계약, SE(3) 배치 보간, 물체 운동 불확실성, 유한 fusion 큐 |
| `perception/sensor_fusion/sensor_fusion/lidar_cluster_node.py` | 전후방 원본 scan, native ABD membership, 점별 변환 후 대표점 계산 |
| `perception/sensor_fusion/sensor_fusion/rgbd_detection_node.py` | RGB/depth capture provenance, RGB-D skew 제한, TF 유한 대기 |
| `perception/sensor_fusion/sensor_fusion/camera_lidar_fusion_node.py` | 공통 epoch 정렬, 전방 한정 보정, 공분산 전파, timeout 기반 단독 출력 |
| `perception/object_tracker/object_tracker/object_tracker_node.py` | 배열/객체 epoch 검사, 원본 나이 제한, provenance 중복 방지 |
| `perception/amr_perception_bringup/config/observation_timing.yaml` | 시간·큐·불확실성 설정 |
| `perception/amr_perception_bringup/launch/perception_stack.launch.py`, `stack_master/launch/amr_system.launch.py` | `observation_timing_config` 전달 |
| `perception/sensor_fusion/package.xml`, `setup.py` | scipy, laser_tools, 감사 도구 의존성 및 문서·도구 설치 |
| `sensors/amr_laser_tools/amr_laser_tools/scan_merger.py`, `scan_sector_filter.py` | 보존/미확인 timing 의미에 대한 오래된 주석 정정; 기존 동작 유지 |
| `perception/sensor_fusion/test/test_observation_time.py`, `perception/object_tracker/test/test_observation_contract.py` | 수치·TF·중단/복구·중복·시각 계약 테스트 |
| bringup `test/test_semantic_launch.py`, stack_master `test/test_autonomy_semantic_launch.py` | 기존 build/install/log 복사본을 패키지로 중복 탐색하던 fixture 수정 |
| `perception/sensor_fusion/tools/audit_observation_bag.py` | 토픽 발행 없는 읽기 전용 bag 감사 |

기존 DRA-MPPI 및 사용자가 수정한 파일은 변경하지 않았다. IMM, segmentation,
SLAM 동적 필터, 지도 재통합, 제어기는 이번 변경 범위에 포함하지 않았다.

2. 실행 경로

```text
/scan_front ─┬─ 기존 merger ─ /scan ─ 기존 navigation 소비자
/scan_rear  ─┘
      └─ lidar_cluster (센서별 native ABD → 구성점 보정)
                   └─ /lidar_clusters [DetectedObjectArray + provenance]
RGB + aligned depth ─ rgbd_detection ─ /detections [capture provenance]
                         ↓
camera_lidar_fusion (유한 partner/TF 대기 → odom 경유 공통 epoch)
  ├─ /detections_fused   : camera-only + fused 의미 관측
  ├─ /lidar_obstacles    : unmatched LiDAR 기하 관측
  └─ /detections_tracking: 위 관측을 중복 없이 한 배열로 묶음
                         ↓
object_tracker ─ /tracks + /tracked_object_modes (odom)
```

`/lidar_clusters`는 센서별로 발행되므로 전후방 합산 발행률이 기존 merged 경로보다
높다. 빈 scan도 array provenance로 센서를 식별한다. 인지 섹터는 전방 [-90,90],
후방 [90,-90]으로 분담한다. 기존 merger/Cartographer의 5도 overlap은 변경하지
않았다. 인지는 overlap 중복을 줄이기 위해 반원씩 사용하며, 경계 물체가 나뉘는
현상이나 서로 다른 센서의 같은 물체를 통합하는 고급 segmentation은 남아 있다.
보정한 점은 내부 XYZ 배열이며 LaserScan.ranges에 다시 넣지 않는다.

3. 측정 시각과 reference epoch

`ObservationProvenance.header`는 수신한 원본 sensor header 그대로이다.
LiDAR에서는 원본 LaserScan header, RGB-D에서는 위치 계산에 사용한 depth와
검출에 사용한 color 각각의 capture header이다. 관측 ID는
`topic|frame|stamp_ns|원본 beam 집합 또는 detection index`이다.
원본 scan을 저장하면 이 ID와 beam_indices로 정확한 배열 원소를 찾을 수 있다.
beam index는 무효값, 섹터 masking, ABD 분리, wraparound 뒤에도 원래 인덱스이다.

검증된 `timing_modes=ros`는 `stamp + i*time_increment`, `reversed`는
`stamp + (N-1-i)*time_increment`이다. 후자는 **stamp가 가장 이른 측정이고
배열 마지막 원소가 가장 이른 측정인 것으로 검증된 드라이버에만** 사용한다.
0/음수/비유한 increment, 과도한 sweep duration은 unknown으로 처리한다.
`scan_time/N`으로 시간을 만들어 넣지 않는다. unknown의 offsets는 빈 배열이다.

확인한 bundled `sensors/rplidar_ros/src/rplidar_node.cpp`에서는:

- `grabScanDataHq()` 호출 전 host `now()`를 시작 stamp로 저장한다.
- `ascendScanData()`로 각도 정렬하고 angle compensation에서 노드를 복제한다.
- `publish_scan()`이 inverted/reversed 및 flip-X에 따라 배열 인덱스를 바꾼다.
- 그 뒤에도 `scan_duration/(node_count-1)`을 양의 increment로 넣는다.
- 센서 launch는 양쪽 inverted=false, angle_compensate=true, DenseBoost이다.

따라서 반전 인덱스만 바꿔도 실제 acquisition epoch/복제 노드의 시간을 보증할
수 없다. 기본 `unknown`을 사용하며, strict 운용은 `unknown_timing_policy=drop`이다.
snapshot은 header 시각에 한 번 변환한 근사 기하만 제공하고 deskew=false를 유지한다.

객체/배열 `header`는 **좌표가 표현된 base_link의 시각**이다. 검증된 LiDAR는
scan의 마지막 빔 측정 시각, unknown은 원본 header, 카메라는 depth capture를
최초 기준으로 쓴다. fusion은 batch의 가장 늦은 입력 reference epoch로 모두
변환한다. 원본 관측 header는 바꾸지 않는다. 처리 완료 시각은 관측 stamp로
사용하지 않으며 age/latency 진단에만 사용한다.

4. 운동·공분산·보정 순서

ABD membership은 원본 각도 배열에서 정한다. 각 구성점을
`base_link(t_ref) <- odom <- sensor(t_i)`로 변환한 뒤 평균/median/nearest 대표점과
폭을 계산한다. full TF에는 장착 위치와 회전이 포함된다. 10ms 간격 knot의
회전 SLERP와 이동 선형 보간으로 조회 횟수를 제한하며, 모든 조회는 nonblocking이다.
TF 실패는 기존 `PendingMeasurements`를 재사용해 기다리고 timeout/drop한다.
과거 관측에 최신 TF를 적용하는 fallback은 없다.

카메라·LiDAR 대표점은 다시 동일한 `base_link(t_ref)`로 정렬한 후 association한다.
기존 전방 C2 yaw=-0.043670 rad, 대표점 offset, residual 값은 그대로 유지했다.
실제 URDF의 전방 yaw는 0, 후방 yaw는 pi였다. 이 보정은 `/scan_front`에만,
native base_link에서 epoch 이동 전에 적용한다. URDF 장착 보정으로 이관할 때
`lidar_yaw_correction=0`으로 해야 중복 적용되지 않는다. 후방에는 전방 offset을
적용하지 않고 미실측 sigma를 명시한다. raw-per-sensor clustering으로 추정량이
달라졌으므로 C2 precision의 운용 적합성은 실차에서 다시 확인해야 한다.

신뢰할 속도 prior를 사용하지 않는다. 물체 운동은 평균 0, 표준편차 sigma_v로
모델링하고, 각 구성점/카메라 관측에서 reference epoch까지 가장 큰 시간차 h에
대해 `Q_time = (sigma_v*h)^2 I` [m²]를 더한다. intra-scan 시간차도 포함된다.
unknown timing에는 별도 `unknown_timing_sigma² I`를 더한다.
대표점 offset은 수치 Jacobian으로, 좌표 변환은 회전의 xy block으로 공분산을
전파한다. C2 residual과 Q_time은 **association과 최종 융합 공분산 모두**에
들어간다. chi-square gate=9.21은 바꾸지 않았다. 허용 시간차 밖의 쌍은 배정하지
않는다. 실제 물체를 속도로 이동시켜 보정했다고 주장하지 않는다.

제한사항: TF에는 pose covariance가 없다. odom drift/jump, 센서 clock offset,
장착 calibration 오차 및 shared ego error의 상관은 추정하지 않는다. 2×2 모델은
높이 uncertainty와 roll/pitch에 의한 z-noise→xy 전파도 생략한다. 고가속 운동에서
10ms knot 보간 오차가 커질 수 있다. RGB-D 내부 skew도 제한·보존하지만 rolling
shutter나 서로 다른 capture의 segmentation/depth reprojection은 해결하지 않는다.

5. 큐·설정·진단

| 설정 | 기본값 | 근거/의미 |
|---|---:|---|
| timing_modes | unknown, unknown | 확인한 bundled 드라이버 계약을 과신하지 않음 |
| unknown_timing_policy | snapshot | 기하 추적 지속; strict 선택지는 drop |
| tf_knot_interval | 0.01 s | 100ms sweep에 11 조회; 수치 테스트로 오차 확인 |
| max_scan_duration | 0.5 s | 잘못된 increment의 비정상 보간 비용 제한 |
| partner_wait | 0.10 s | nominal LiDAR 한 주기 후 camera 없이도 처리 |
| tf_wait_timeout | 0.25 s | TF 수신 지연의 유한 대기 |
| sync_slop | 0.067 s | 기존 값 유지; reference epoch 차이 제한 |
| max_association_time_delta | 0.167 s | 100ms sweep + 67ms epoch 차이 상한 |
| max_observation_age | 0.75 s | 센서·fusion·tracker에서 원본 관측 age 제한 |
| pending_capacity | LiDAR 16/센서, RGB-D 8 | 메모리 제한 |
| sync_queue | 32 | fusion 전체 입력 개수 제한 |
| object_velocity_sigma | 1.0 m/s | 최초 설계 prior; 실측값이 아님 |
| unknown_timing_sigma | 0.15 m | unknown snapshot 추가 uncertainty; 실측값이 아님 |
| uncalibrated_lidar_sigma | 0.05 m | 후방 C2 미실측을 명시하는 최초 설계값 |
| rgbd_max_delta | 0.025 s | 기존 RGB-D sync 허용폭 내에서 capture skew 제한 |

이미 처리한 reference epoch 이하의 입력은 폐기한다. 대기 중 역순 입력은
timestamp로 정렬하고 동일 stream/stamp는 중복으로 제거한다. 한 batch에는
camera/front/rear stream별 최대 한 배열만 포함하며 다음 입력 시각을 건너뛰지
않는다. timeout된 batch도 watermark를 전진시켜 복구 후 재사용하지 않는다.
queue overflow는 가장 오래된 대기 항목을 제거한다. clock이 뒤로 점프하는 bag
loop는 노드를 재시작해야 한다. 자동 rewind로 track을 중복 갱신하지 않는다.

tracker는 배열 header로 TF를 조회하므로 모든 객체 header가 배열과 같은지
검사한다. 원본 observation ID가 같은 객체의 새 epoch replay도 막는다. fused와
그 fused에 사용한 LiDAR가 같은 update에 중복으로 들어가지 않는다.
카메라 중단시 partner_wait 뒤 LiDAR-only 출력을 계속 내며, TF도 없으면
추가 TF timeout 이후 폐기한다. timer/executor 실행 지연은 설정 시간에 더해진다.

5초 집계 로그: 원본 header 시각차와 구성 빔 시각차, reference_ns, age,
처리 대기시간, TF wait/timeout/drop, timing_unknown/deskewed, matched/camera-only/
LiDAR-only, duplicate/reordered/late/overflow/stale 수. Fusion 로그는 최근 input
레코드 ID 일부만 표시하고, 전체 beam ID는 메시지로 조회한다. 점별 대량 로그는 없다.

6. 검증 결과와 재현

관련 6패키지를 기존 workspace를 덮어쓰지 않고 `/tmp/amr_observation_build`,
`/tmp/amr_observation_install`에 빌드했다. `amr_interfaces`, `amr_laser_tools`,
`sensor_fusion`, `object_tracker`, `amr_perception_bringup`, `stack_master` 빌드 성공.

기능 pytest: **201 passed, 9 deselected** (2.06s). 9개는 별도 colcon 전체 검사로
실행한 lint/copyright 항목이다. native beam membership/sector/wraparound,
timing 정상·반전·무효, 정지/병진/회전+lever arm/intra-scan, 물체 자체 이동,
실제 tf2 지연·누락, duplicate/out-of-order/age/overflow, 카메라 중단·복구 및
실제 tracker 동일 ID 유지, covariance 회전·PSD·m², launch 연결을 검증했다.

2,000빔 / 100ms / 10ms knot 합성 시험은 각각 약 1.03~1.21ms였다. 정지·등속
병진 최대 수치 오차는 부동소수점 오차 수준, yaw 1.4rad/s+0.4m lever는
9.8µm, yaw 2rad/s+0.4m lever는 20µm였다. TF knot 조회는 11회이다.
이는 합성 궤적의 계산 비용이며 실차 ROS callback deadline 보장은 아니다.

기존 full colcon lint도 실행했다. HEAD 비교본에서도 flake8와 pep257 실패가
확인되므로 전체 colcon을 통과했다고 보고하지 않는다. 기능 실패를 유발하던
launch fixture의 install 중복 탐색은 수정했다. 기존 문서/스크립트/다른 함수의
스타일 정리는 이 작업에서 전면 수행하지 않았다. 상세 최종 결과는 옆의
`observation_timing_validation.txt`와 `/tmp/amr_observation_colcon_test.txt` 참조.

실차 주행 및 모터 launch는 실행하지 않았다.

7. rosbag 감사와 실차 비교 절차

기존 `mpc_chain_20260923_135325`의 첫 10초를 읽기 전용 감사했다. front 100,
rear 100, positive increment 200, **timing unknown 200**, TF 가능 198,
TF 부족 2였다. Image/CompressedImage 토픽이 없어 RGB-D 재추론은 못 했다.
같은 점을 원래 epoch에 그대로 두는 경우와 odom을 통해 50ms 뒤로 옮긴 경우의
좌표 차이는 중앙값 0.000314m, p95 0.196644m, 최대 1.942203m였다.
이 값은 **변환 이동량**이며 정확도 개선량이 아니다. 큰 값은 원본 odom/회전의
연속성과 TF 기록을 추가 점검해야 한다. beam ground truth와 RGB-D가 없어 실차
deskew/fusion 정확도의 보정 전후 검증은 수행하지 못했다.

읽기 전용 감사 재현:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/amr_observation_install/setup.bash
python3 amr_basic_stack/perception/sensor_fusion/tools/audit_observation_bag.py \
  /home/smoc/2026amr_ws/bags/mpc_chain_20260923_135325 --seconds 10
```

실차 비교는 다음 순서로 진행한다.

- 먼저 드라이버의 capture epoch·각도 정렬·복제·반전 매핑을 하드웨어 시간 또는
  독립 기준으로 검증한다. 확인 전 `timing_modes=ros/reversed`로 켜지 않는다.
- `/scan_front`, `/scan_rear`, `/tf`, `/tf_static`, RGB, aligned depth, CameraInfo,
  `/clock`(sim이면), 입력/출력 detections와 진단을 새 bag에 기록한다.
- 로봇 정지/병진/회전에서 고정 타깃, 이어 별도 이동 타깃을 기록한다. 정답
  표면/위치를 독립적으로 확보하고 두 실행에 동일 bag·calibration·gate를 쓴다.
- 분리된 ROS domain에서 **센서·모터·주행 launch 없이** perception 노드만 실행하고
  원본 센서/TF 토픽만 재생한다. bag의 과거 fused/tracks는 재생하지 않는다.
- 두 실행 간 노드를 재시작하고 출력은 별도 새 bag에 저장한다. 원본 bag은 유지한다.
  centroid residual, NIS/PSD, 매칭률, ID 중복, 최대 input age/drop을 비교한다.
- 카메라 재생만 중단·재개하여 LiDAR-only 출력과 ID 유지, 늦은 입력 폐기를 확인한다.

메시지 필드 추가는 wire schema 변경이다. 소비자 소스는 기존 필드를 계속 쓰지만
같은 interface 빌드로 재시작해야 한다. **옛 DetectedObject bag을 새 타입으로 직접
deserialize하면 안 된다.** native LaserScan/TF/Image에서 재생성하거나 별도 타입
변환이 필요하다. 기존 bag에서 이를 임의 변환하지 않았다.

8. 다음 SLAM 동적 필터가 재사용할 정보

각 객체의 sensor/topic ID, 원본 frame/stamp, ranges beam_indices, 검증된 경우의
beam_time_offsets, observation_id, timing_status, ego_motion_compensated,
좌표 reference epoch를 그대로 사용할 수 있다. 이 정보로 원본 scan의 특정 빔을
정확히 마스킹하고, unknown timing을 보정 성공 사례에서 구분할 수 있다.
현재 `TrackedObject`에는 전체 빔 집합을 복제하지 않는다. 후속 필터는
`/detections_tracking` provenance와 track association의 연결을 추가해야 하며,
이번 작업에서 지도 제거/재통합이나 동적 SLAM 판정까지 수행하지 않았다.
