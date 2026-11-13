# dynamic_filter: 반경·재클러스터링·추적 관측 빔 비교

Mapping While Following의 사람 주변 고정 반경 마스킹을 참고한 SLAM 입력 필터다.
기존 고정 반경 동작은 `mask_method: radius`로 유지한다. `mask_method: cluster`는
원본 scan의 점 묶음을 트랙에 기하적으로 연결해 해당 빔만 제거하는 비교용 옵션이다.
두 방식 모두 지도에 이미 남은 흔적을 소급 제거하지 않는다.

**2026-11-06 시각 계약 보강:** `radius`/`cluster`의 기본 `scan_timing_mode`는
`unknown`이다. 검증되지 않은 빔 시각으로 TF 조회·제거를 수행하지 않고 원본을 통과시키며
`scan_timing_unknown`을 보고한다. 현재 RPLIDAR의 정렬·복제·reverse·flip-X 및 host
stamp로는 정확한 빔 시각을 확인할 수 없다. 검증된 입력에서만 `ros`(stamp가 0번 빔 시각,
시간순 배열) 또는 `reversed`(stamp가 마지막 인덱스의 가장 이른 측정 시각)를 설정한다.
0 increment도 순간 스캔으로 검증된 계약에서만 허용한다. `tracked_cluster`의 정확한
원본 인덱스 마스킹에는 이 설정이 영향을 주지 않는다. 아래 기존 비교 수치는 당시 설정의
기록이며, unknown 기본값으로 같은 기하 제거가 수행된다는 의미가 아니다.

## 세 번째 방식: tracked_cluster

`mask_method: tracked_cluster`는 기존 클러스터 노드가 만든 **원본 빔 묶음 중 추적기가
실제로 채택한 관측**을 제거한다. `radius`/`cluster`와 기존 IMM 메시지 정의는 유지한다.
새 메시지 `TrackedObservationArray`를 `/tracked_observations`에 별도로 발행한다.

```text
native /scan_front, /scan_rear
  -> 기존 lidar_cluster_node: ObservationProvenance(sensor/frame/stamp/beam_indices)
  -> camera_lidar_fusion: 원본 관측 정보 보존
  -> object_tracker: 이번 update에서 실제로 소비한 measurement index ↔ track_id
  -> /tracked_observations
  -> dynamic_filter (tracked_cluster): 정확히 대응하는 원본 빔만 NaN
```

`MultiObjectTracker.last_measurement_matches`는 매 step 시작 시 비운다. 최종 association에서
update에 사용한 관측과 새 트랙 생성에 사용한 관측만 기록한다. 반복/역행 시각, coast,
관측 없이 예측만 수행하는 트랙에는 과거 빔을 재발행하지 않는다. 지도 필터/수명 관리 후
실제로 발행되는 트랙만 연결 이벤트를 발행하며, 필터가 CONFIRMED와 사람/동적 정책을
확인한다. 카메라-only 관측처럼 빔 인덱스가 없으면 연결 이벤트에 넣지 않는다.

이벤트는 `track_id`, `lifecycle_state`, `motion_state`, `class_name`, `semantic_age`와
원본 `ObservationProvenance`를 가진다. 배열 header는 추적 갱신 시각이며 원본 관측의
header를 덮어쓰지 않는다. 클러스터 관측 식별자는 `observation_id`, 시간에 걸친 객체
식별자는 `track_id`다. 현재 scan과 실제로 연결된 빔이 없으면 제거하지 않는다.

| 방식 | 빔 선택 | 입력 |
|---|---|---|
| `radius` (기본) | 예측 위치 주변 0.35m | IMM + scan + TF |
| `cluster` | 필터 내부 재분할·위치 재연결 | IMM + scan + TF |
| `tracked_cluster` | 추적기가 채택한 관측의 원본 인덱스 | 새 연결 이벤트 + 정확히 대응하는 scan |

`tracked_cluster`는 반경/재클러스터링/TF 위치 검색으로 fallback하지 않는다. 본래 추적기의
오연결이나 사람+벽이 합쳐진 클러스터를 바로잡는 방법은 아니다. 추적기가 관측 하나만
사용했다면 연결되지 않은 다른 다리 묶음까지 확대해서 제거하지 않는다.

### 정확한 스캔 대응과 지연

* `observation_sensor_id` + 원본 `header.frame_id` + nanosecond stamp가 모두 일치해야
  한다. 클러스터의 빔은 **원본 LaserScan 인덱스**다. 섹터 밖 range만 Inf로 바꾸고
  인덱스·header·각도/시간 정보를 유지하는 현재 `scan_sector_filter` 출력에는 적용할 수 있다.
  병합 `/scan`, 재표본화·정렬·crop·stamp 변경된 scan에 인덱스를 재사용하면 안 된다.
* native sensor ID는 현재 `/scan_front`, `/scan_rear`다. 입력은 `/scan_front_carto`,
  `/scan_rear_carto`이므로 토픽 이름을 같은 것으로 추측하지 않고 명시적으로 설정한다.
  다른 센서는 launch/YAML에서 native sensor ID도 함께 맞춰야 한다.
* 연결 이벤트가 점진적으로 들어오므로, 원본 scan 수신 후 steady/wall time 기준
  `observation_wait_timeout=0.30s`를 **항상** 기다린다. 대기 중 수신한 일치 이벤트를
  합쳐 판단한다. 타이머가 늦게 실행돼도 기한 이후 도착한 이벤트는 적용하지 않는다.
  원본 scan보다 먼저 이벤트가 도착해도 이력 내에서 정확히 일치하면 사용할 수 있다.
* `max_observation_lag=0.75s`는 원본 scan stamp부터 tracker update stamp까지의 상한이다.
  처리 도착 기한과 별개다. 배열 시각이 scan보다 이전이면 거부한다. 사람 라벨의 수명은
  관측 연결을 판단한 update 시점에서 검사한다. 이전 방식의 CV 외삽을 사용하지 않는다.
* 알려진 빔 시간은 유한성·길이·스캔 기간·update 시각 이내 여부를 검사한다. 역순 시간도
  허용하고, timing unknown의 빈 offsets를 임의로 복원하지 않는다. 이 방식은 점 위치를
  재계산하지 않고 인덱스만 제거하므로 unknown timing도 원본 멤버십 자체는 사용할 수 있다.
* 다른 track ID가 같은 빔을 주장하거나 부적격 결정과 겹치면 해당 묶음 전체를 보존한다.
  범위 밖/중복 인덱스, 비어 있는 observation ID, 잘못된 시간 정보는 거부한다.
* 연결 없음, 기한 경과, 큐 초과 시 원본 통과한다. 기본 pending queue 5개,
  연결 캐시 최대 2,000개/2초이며 clock 역행 시 모두 비운다. 실제 출력 지연에는
  executor 지연이 더해지므로 0.30초를 실시간 처리 완료 보장으로 해석하지 않는다.

### 실행 및 기록

새 메시지를 포함한 패키지를 빌드하고 source해야 한다. 이미 실행 중인 구버전 추적기는
새 토픽을 발행하지 않으므로 새 버전으로 재시작한다. 기존 install을 보존하는 예:

```bash
cd /home/smoc/2026amr_ws/src
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
colcon --log-base /tmp/amr-tracked-cluster/log build --base-paths amr_basic_stack/perception/amr_interfaces amr_basic_stack/perception/object_tracker amr_basic_stack/perception/dynamic_filter --build-base /tmp/amr-tracked-cluster/build --install-base /tmp/amr-tracked-cluster/install --packages-select amr_interfaces object_tracker dynamic_filter --symlink-install --allow-overriding amr_interfaces object_tracker dynamic_filter --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source /tmp/amr-tracked-cluster/install/setup.bash

# 기존 인지/퓨전도 ObservationProvenance를 보존하는 현재 소스로 실행해야 한다.
# 추적기와 인지 스택은 시스템 launch에서 한 번만 실행한다.
# 세 방식의 scan 출력만 동시 비교 (6개 필터, Cartographer 실행 안 함):
ros2 launch dynamic_filter compare_filters.launch.py include_tracked_cluster:=true

# 별도 실행: Cartographer에는 세 번째 방식만 적용
ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping map:=ab_tracked use_dynamic_filter:=true dynamic_filter_config:=/home/smoc/2026amr_ws/src/amr_basic_stack/perception/dynamic_filter/config/dynamic_filter_tracked_cluster.yaml
```

기존 비교 launch는 기본 4개 필터를 그대로 유지한다. `include_tracked_cluster:=true`로
추가하면 `/dynamic_filter_compare/tracked_cluster/scan_front`, `.../scan_rear`에 발행한다.
`front_sensor_id`, `rear_sensor_id`, `tracked_observations_topic` 인자로 대응 정보를 바꿀 수
있다. Cartographer용 YAML은 `dynamic_filter_front/rear` 노드별 native sensor ID를 지정한다.
단일 노드는 `-p mask_method:=tracked_cluster -p observation_sensor_id:=/scan_front`가 필요하다.

```bash
# 세 방식 재생용 최소 입력: 연결 이벤트와 기존 IMM 둘 다 기록
ros2 bag record -o dynamic_filter_three_methods /scan_front_carto /scan_rear_carto /tracked_object_modes /tracked_observations /tf /tf_static /odom /camera/imu

# 새 bag에 저장된 실제 연결로 재생. 같은 도구에서 radius/cluster도 선택 가능.
python3 amr_basic_stack/perception/dynamic_filter/tools/compare_bag.py --bag /path/to/new_bag --output /tmp/filter_tracked_eval --mask-method tracked_cluster

# 실제 tracker -> DDS -> front/rear membership filter의 합성 통합 검사
python3 amr_basic_stack/perception/dynamic_filter/tools/check_tracked_pipeline.py --output /tmp/tracked_pipeline.json
```

기존 2026-09-23 bag 3개에는 `/tracked_observations`가 없다. 기존 IMM 위치나 현재
재클러스터링으로 이 관계를 만들어서 '실제 tracker 연결'로 간주하지 않는다.
오프라인 도구는 새 토픽이 없으면 명시적으로 중단한다. 저장된 원본 LiDAR와 카메라 검출로
현재 upstream을 재실행해 **새 연결 이벤트**를 만들거나, 새 데이터를 수집해야 한다.
아래 실험에서는 첫 번째 경로를 사용했다. 과거 추적기의 연결을 복원한 것은 아니다.

### 동일 135606 bag의 세 방식 비교 (2026-11-05)

`mpc_chain_20260923_135606`의 약 351초, 전·후방 7,018 scan, 유효 반사점
12,179,628개를 비교했다. 원본 LiDAR와 당시 `/detections`를 현재 클러스터→퓨전→추적기에
공급하고, **그렇게 재생성한 동일 추적 결과**를 세 필터에 넣었다. 모두 기본
`confirmed_person` 정책이므로 정지한 사람도 제거 대상이다.

| 방식 | 누적 제거 점 | 유효 점 대비 | 제거가 발생한 scan |
|---|---:|---:|---:|
| `radius` (0.35m) | 120,157 | 0.987% | 1,164 |
| `cluster` (필터 내부 재연결) | 64,440 | 0.529% | 915 |
| `tracked_cluster` (실제 관측 연결) | 124,530 | 1.022% | 1,238 |

* 세 번째 방식은 사람 트랙에 연결된 1,908개 LiDAR 관측의 빔을 제거했다. 이는 반복 관측
  수이며 사람 수가 아니다. 이전 두 방식처럼 위치를 보고 다시 연결하지 않는다.
* 반경 방식과 세 번째 방식이 공통 제거한 점은 80,765개다. 두 방식 사이에서 반경만
  제거한 점은 39,392개, 세 번째만 제거한 점은 43,765개다. 세 번째 방식은 단순히
  반경 결과를 더 작게 자르는 방식이 아니다.
* 약 135.3초 예시의 제거량은 반경 314점, 재클러스터링 202점, 실제 관측 연결 205점이다.
  세 번째 방식은 연결된 묶음만 선택하고 다른 묶음을 남긴다. 남긴 점이 벽인지 사람인지
  정답 라벨이 없으므로 과잉 제거 감소나 사람 제거 재현율 개선으로 단정할 수 없다.
* 사람에 연결된 관측 중 159개는 원본 점들의 최대 상호 거리가 1m를 넘었다
  (최대 1.372m). 사람+벽이 같은 클러스터가 되거나 잘못 연결되면 세 번째 방식도
  함께 제거할 수 있다. 이 통계 자체가 벽 포함을 증명하는 것은 아니다.
* 세 번째 방식의 가상 출력 대기는 중앙값 304.5ms, p95 309.6ms였다. 설정한 300ms
  관측 대기와 10ms 가상 타이머의 결과이며 실제 CPU/DDS 지연을 측정한 값은 아니다.

입력 보존 검사: 구형 카메라 메시지 5,222개를 bag에 내장된 정의로 읽고 CDR 왕복 일치를
확인했다. 7,018개 scan 모두 원본/Cartographer 입력의 header·빔 구조와 유효한 range가
대응했다. intensity는 전처리 과정에서 달라져 있으며, 필터 출력에서는 **Cartographer
입력의 intensity**를 그대로 보존했다. 세 방식의 원본 SLAM 입력 hash가 동일하고,
각 출력에서 제거되지 않은 range와 전체 scan 메타데이터가 유지됨을 검사했다.

예전 보고서의 반경 62,792점·클러스터 15,694점은 **bag에 저장된 과거 IMM**으로 계산했다.
이번 수치는 현재 인지/추적기를 다시 실행했으므로 그 수치와 직접 비교해 필터 개선량을
계산하면 안 된다. 원본 카메라 영상이 없어 검출 추론 및 영상 동기화는 다시 검증하지
못했다. Cartographer는 이번 비교에서 재실행하지 않았으며 측위/submap 정확도 결과가 아니다.

결과 파일:

* [전체·시간대별 그래프](../../../artifacts/dynamic_filter_three_methods_135606/three_methods.png)
* [동일 시각의 점 제거 비교](../../../artifacts/dynamic_filter_three_methods_135606/three_methods_example.png)
* [집계 CSV](../../../artifacts/dynamic_filter_three_methods_135606/three_methods.csv),
  [상세 JSON](../../../artifacts/dynamic_filter_three_methods_135606/three_methods.json)
* [재생성 조건·진단](../../../artifacts/dynamic_filter_three_methods_135606/rebuilt/rebuild_summary.json),
  [관측 크기 통계](../../../artifacts/dynamic_filter_three_methods_135606/person_membership_geometry.json)
* [재현 스크립트](../../../artifacts/dynamic_filter_three_methods_135606/reproduction/replay.sh):
  위 격리 overlay를 빌드한 동일 코드 상태에서 `bash .../replay.sh /tmp/new_comparison` 실행.
  옛 메시지 디코더 소스, 실행 소스 snapshot/hash도 `reproduction/`에 보관했다.

### 첨부 그림과 동일한 113151 bag 비교 (2026-11-05)

전방 130.009초·후방 129.999초, 반경 90점·클러스터 59점이 표시된 기존 그림은
`mpc_chain_20260923_113151`의 결과다. 이 bag의 약 288초, 5,672 scan 전체를 현재
클러스터→퓨전→추적기로 재처리하고, 동일한 새 추적 결과로 세 필터를 비교했다.
카메라 검출 4,220개 CDR 왕복 일치, 원본/Cartographer scan 5,672쌍의 빔 구조·유한
거리값 대응, 세 필터의 동일 SLAM 입력 hash를 확인했다. 예시 그림은 기존 그림과
**스캔 시각·원본 점 좌표·확대 범위가 모두 동일**하다.

| 방식 | 첨부 장면 제거 점 | 전체 bag 누적 제거 점 | 전체 유효 점 대비 |
|---|---:|---:|---:|
| `radius` | 128 | 44,148 | 0.447% |
| `cluster` | 59 | 25,350 | 0.257% |
| `tracked_cluster` | 92 | 60,576 | 0.613% |

예시 왼쪽 묶음은 세 방식 모두 59점을 제거했다. 오른쪽에서는 반경 69점,
재클러스터링 0점, 실제 관측 연결 33점을 제거했다. 이 장면에서 세 번째 방식은
반경보다 36점을 보존하고 재클러스터링보다 33점을 더 제거했다. 남긴 점이 벽인지,
추가 제거한 점이 사람인지는 정답 라벨이 없어 확정할 수 없다. 전체 bag에서는
세 번째 방식이 가장 많이 제거하므로 이 장면의 제거량 관계를 전체로 일반화하면 안 된다.

기존 그림의 90점과 이번 반경 128점의 차이는 새 인지·추적 결과를 사용했기 때문이다.
센서 입력은 같지만 과거 IMM과 현재 IMM은 다르다. 세 번째 방식은 1,126개의 사람
관측에 연결된 빔을 사용했으며, 기본 `confirmed_person` 정책은 정지한 사람도 포함한다.
가상 출력 대기 중앙값은 세 번째 방식 약 305ms다. 실제 DDS/CPU 지연이나 측위/submap
정확도는 이번 실험에서 측정하지 않았다.

* [첨부 장면의 원본+세 방식 비교](../../../artifacts/dynamic_filter_three_methods_113151/three_methods_example.png)
* [전체·시간대별 그래프](../../../artifacts/dynamic_filter_three_methods_113151/three_methods.png)
* [전체 집계](../../../artifacts/dynamic_filter_three_methods_113151/three_methods.json),
  [동일 장면 검증](../../../artifacts/dynamic_filter_three_methods_113151/example_comparison.json),
  [장면별 마스크 차이](../../../artifacts/dynamic_filter_three_methods_113151/example_mask_analysis.json)
* [재현 스크립트](../../../artifacts/dynamic_filter_three_methods_113151/reproduction/replay.sh)

카메라 판정 근거 추가 확인: 위 130초 장면에 가까운 `/detections`의 시각은
130.001953초(전방 scan보다 약 6.945ms 이전)이며 `person` 검출 2개,
신뢰도 점수 약 0.499·0.449가 기록돼 있다. 새 추적 연결의 59빔·33빔 묶음도 모두
CONFIRMED person이고 `semantic_age=0`이다. 따라서 **카메라 person 판정을 근거로
추적기에 연결된 관측의 제거 여부**는 확인할 수 있다. 분류/연결 결과를 같은 필터의
독립 정답으로 삼거나 해당 묶음의 모든 점이 사람임을 보장할 수는 없다. 이 확인만으로
카메라 detection index와 각 track의 일대일 대응을 주장하지 않는다.
[실제 카메라 검출·연결 기록](../../../artifacts/dynamic_filter_three_methods_113151/camera_person_evidence.json)

진단의 `observation_links`, `observations_masked`, `observation_conflicts`,
`observation_membership_rejected`, `observation_policy_or_age_rejected`,
`observation_timing_rejected`와 `observations_missing` 사유를 확인한다.

2026-11-05 검증: 관련 3개 패키지 격리 빌드 성공. 기존 두 방식 회귀를 포함한 기능 검사
**270개 통과**. 센서/프레임/ns 시각 불일치, 기한 이후 도착, coast/반복 update,
잘못된 인덱스, 경쟁 소유권, 두 센서 원본 빔, 역순 빔 시각을 검사했다.
실제 tracker와 두 필터를 격리 ROS 도메인 194에서 DDS로 연결해 5회 관측을 공급했고,
확정 전 두 회는 보존, 확정 후 세 회는 전방 지정 2빔·후방 지정 1빔만 제거됨을 확인했다.
새 연결 메시지를 포함한 합성 MCAP의 오프라인 재생도 같은 빔만 제거했다.

새/변경 필터·연결 helper 코드의 명시적 flake8/pep257 검사는 통과했다. 전체 디렉터리에서
실행된 기존 tracker lint 템플릿 2개는 이번 변경 밖의 저장소 오류를 보고했으므로,
저장소 전체 lint 통과를 주장하지 않는다. 상세 로그와 소스 hash는
[검증 기록](../../../artifacts/dynamic_filter_tracked_cluster_validation/validation.json)에 있다.
실제 센서 데이터의 정확도 개선이나 실부하 처리 지연을 검증한 결과는 아니다.

## 반경 / 클러스터 선택과 동시 비교

`filter_policy`는 **어떤 트랙**을 제거할지, `mask_method`는 **어떤 빔**을 제거할지 정한다.
기본은 기존과 같은 `confirmed_person` + `radius`다. 필터 자체도 launch에서 기본 OFF다.

| 구성 | 설정 | 동작 |
|---|---|---|
| OFF | `use_dynamic_filter:=false` | 원본 두 scan을 Cartographer로 전달 |
| 기존 반경 | `dynamic_filter.yaml` | 적격 트랙 위치 주변 0.35m 반환 제거 |
| 새 클러스터 | `dynamic_filter_cluster.yaml` | 적격 트랙과 연결된 점 묶음의 원본 빔 제거 |

현재 구현의 연결은 **같은 scan 시각의 예측 위치에 대한 기하적 재연결**이다.
카메라는 기존 트래커의 사람 라벨을 통해 간접적으로 사용한다. 필터가 영상을 처리하거나
upstream tracker의 관측→track ID 대응을 전달받아 동일 객체를 확정하는 것은 아니다.
기존 `/lidar_clusters` 대신 필터 입력 scan 자체에서 묶음을 만들어, 예전 bag에도 적용하고
전·후방 센서 및 다른 가공 scan 사이의 인덱스를 잘못 재사용하지 않도록 했다.

클러스터 모드의 처리 순서:

1. 원본 빔 순서에서 인접 점 간 거리가 `min(max_gap, base_gap + gap_scale ×
   min(range_i, range_j) × abs(angle_increment))`를 넘으면 분리한다.
   무효 빔은 항상 경계이며, 360° scan의 양 끝이 실제 이웃일 때만 연결한다.
2. 점이 너무 적거나 odom XY bounding box의 대각선 길이가 `cluster_max_span`을
   넘는 묶음은 보존한다. 나이 제한으로 묶음 일부를 먼저 잘라내지 않는다.
3. 각 빔 시각의 TF·트랙 CV 외삽 위치로 상대 점들을 구한다. 평균 상대 위치의 거리가
   `cluster_match_distance` 이내인 CONFIRMED 트랙을 후보로 삼는다.
   사람 이외/정적 트랙도 경쟁시켜 그 점을 사람에게 빼앗기지 않도록 한다.
4. 가장 가까운 두 후보의 거리 차이가 `cluster_ambiguity_margin`보다 작으면 보존한다.
   최선 후보가 정책상 제거 대상이 아니거나 묶음의 어느 빔에서든 관측/semantic 수명이
   만료되면 그 묶음을 보존한다. 모든 점은 후보 트랙에서 `cluster_max_point_distance`
   미만이어야 한다. 이 거리는 **묶음 전체의 승인 조건**이며 빔별 반경 제거가 아니다.
5. 양다리를 고려해 트랙당 최대 두 묶음을 허용한다. 후보 묶음이 더 많으면 임의로
   두 개를 고르지 않고 전부 보존한다. 합친 상대 점들의 크기도 `cluster_max_span`
   이내여야 한다. 승인된 원본 인덱스의 range만 NaN으로 바꾼다.

연결 실패 시 반경 방식으로 fallback하지 않는다. TF 실패·트랙 미수신·큐 상한은 기존과
같이 원본 통과한다. 동작 중 파라미터 변경은 거부하므로 변경 시 노드를 재시작한다.

| 클러스터 파라미터 | 기본값 | 단위/의미 |
|---|---:|---|
| `cluster_min_points` | 3 | 최소 점 수 |
| `cluster_base_gap` | 0.05 | m, 분리 거리의 기본값 |
| `cluster_gap_scale` | 2.0 | 거리·각분해능 항의 계수 |
| `cluster_max_gap` | 0.15 | m, 분리 거리 상한 |
| `cluster_max_span` | 0.80 | m, 개별/결합 묶음의 XY 범위 대각선 상한 |
| `cluster_match_distance` | 0.45 | m, 평균 상대 위치의 연결 게이트 |
| `cluster_max_point_distance` | 0.60 | m, 연결된 트랙에서 개별 점까지의 상한 |
| `cluster_ambiguity_margin` | 0.10 | m, 최선/차선 후보의 최소 거리 차이 |
| `cluster_max_per_track` | 2 | 트랙당 최대 묶음 수 |

이 값은 초기 공학 설정이며 성능 최적값이 아니다. `filter_radius`는 cluster에서 사용하지
않는다. 위치 공분산 기반 연결, 시간에 따른 형상 추적, 벽 지도 대조는 아직 하지 않는다.
작은 벽 조각과 사람이 하나로 합쳐지면 오제거가 가능하고, 보수적인 보류 조건 때문에
사람 점이 남을 수 있다. `dynamic_only`로 큰 객체를 제거하려면 크기 조건도 별도 검토한다.

수정 패키지 빌드 후, **Cartographer 없이 두 필터 출력만 동시에 비교**:

```bash
ros2 launch dynamic_filter compare_filters.launch.py use_sim_time:=true
# 다른 터미널에서 원본 입력 bag 재생. bag에 들어 있는 tracker를 중복 실행하지 않는다.
ros2 bag play /home/smoc/2026amr_ws/bags/mpc_chain_20260923_135606 --clock
```

출력은 `/dynamic_filter_compare/radius/scan_front`, `/dynamic_filter_compare/radius/scan_rear`,
`/dynamic_filter_compare/cluster/scan_front`, `/dynamic_filter_compare/cluster/scan_rear`다.
RViz에서 같은 frame/시각으로 비교하거나 이 네 토픽과 원본을 기록한다.
진단은 `/dynamic_filter_{radius|cluster}_{front|rear}/diagnostics`에 발행된다.
`mask_method`, `clusters_matched`, `clusters_ambiguous`, `clusters_oversized`,
`clusters_non_target_or_expired`, `clusters_track_limit_rejected` 등을 확인한다.
제거량은 정확도가 아니며 사람/정적 점 정답 라벨 없이는 precision/recall을 주장하지 않는다.

Cartographer에 적용할 때는 각 실행을 종료한 뒤 같은 bag을 다시 재생한다:

```bash
# 기본: 기존 반경 방식
ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping map:=ab_radius use_sim_time:=true use_dynamic_filter:=true

# 별도 실행: 새 클러스터 방식 (src에서 실행할 때의 설정 경로)
ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping map:=ab_cluster use_sim_time:=true use_dynamic_filter:=true dynamic_filter_config:=/home/smoc/2026amr_ws/src/amr_basic_stack/perception/dynamic_filter/config/dynamic_filter_cluster.yaml
```

SLAM 비교에서는 아래 기존 A/B 절차대로 bag의 기존 map→odom TF를 제외하고,
같은 Lua/odom/IMU/초기화 조건을 사용한다. 위 동시 비교 launch는 Cartographer를 실행하지 않는다.

기존 오프라인 도구도 두 방식 모두 지원한다. 출력 디렉터리는 새 경로여야 한다:

```bash
python3 amr_basic_stack/perception/dynamic_filter/tools/compare_bag.py --bag /home/smoc/2026amr_ws/bags/mpc_chain_20260923_135606 --output /tmp/filter_radius_eval --filter-config amr_basic_stack/perception/dynamic_filter/config/dynamic_filter.yaml
python3 amr_basic_stack/perception/dynamic_filter/tools/compare_bag.py --bag /home/smoc/2026amr_ws/bags/mpc_chain_20260923_135606 --output /tmp/filter_cluster_eval --filter-config amr_basic_stack/perception/dynamic_filter/config/dynamic_filter_cluster.yaml
```

각 폴더에 원본 `off/`, 선택 방식 `on/` SLAM 입력 bag, `filter_summary.json`,
빔 제거 수·보류 사유를 담은 `scan_metrics.csv`와 120–140초 frame 자료를 생성한다.
`--mask-method radius|cluster`로 YAML의 방식만 덮어쓸 수도 있다.
기록 수신 순서와 가상 timer를 사용하는 알고리즘 재생이며 실제 DDS 지연 검사는 아니다.

두 결과의 원본 hash·시각·기본 파라미터 일치를 검사하고 비교 그림/집계를 생성:

```bash
python3 amr_basic_stack/perception/dynamic_filter/tools/render_method_comparison.py --radius /tmp/filter_radius_eval --cluster /tmp/filter_cluster_eval --output /tmp/filter_method_comparison
```

2026-11-04 구현 검증: 기능·ROS 메시지/TF·launch 테스트 **85 passed**,
클러스터 분할 벡터화 후 관련 core/cluster **66 passed**, 변경 Python 스타일 검사 통과.
`/tmp/amr-cluster-filter/install`에 패키지를 격리 빌드했고 설치된 비교 launch의 인자
해석도 확인했다. 실제 네 노드의 동시 DDS 부하/실기 지연 검사는 아직 수행하지 않았다.

`135606` 전체 7,018 scan을 두 방식으로 재생했다. 반경 방식은 수정 전 구현과
재시도를 포함한 **7,029회 mask 호출**의 출력 LaserScan 직렬화 바이트와 전체 진단이
모두 일치했다. 원본 입력/메타데이터 및 제거하지 않은 range 보존도 검사했다.

| 실제 bag 결과 | 반경 | 클러스터 |
|---|---:|---:|
| 입력 유효 점 수 | 12,179,628 | 12,179,628 |
| 제거된 점 수 | 62,792 | 15,694 |
| 점이 제거된 scan 수 | 656 | 257 |

클러스터의 제거량 감소는 보수적 연결/보류의 결과이며, 정적 벽 보존율이나 사람 제거
재현율의 개선을 증명하지 않는다. 해당 bag에는 점 단위 정답 라벨이 없다.
이번 추가 비교에서는 Cartographer를 다시 실행하거나 측위 정확도를 측정하지 않았다.

* [동일 장면 원본·반경·클러스터 그림](../../../artifacts/dynamic_filter_methods_135606/methods_example.png)
* [전체 시간별 제거율](../../../artifacts/dynamic_filter_methods_135606/methods_timeline.png)
* [수치·보류 사유·공통 입력 검증](../../../artifacts/dynamic_filter_methods_135606/method_comparison.json)
* [수정 전 반경 구현과의 일치 검사](../../../artifacts/dynamic_filter_methods_135606/radius/radius_regression.json)

## 기존 반경 방식의 실험 기록

실제 bag 비교 결과: [135606 OFF/ON 보고서와 그림](../../../artifacts/dynamic_filter_ab_135606/REPORT.md).
전체 350.75초를 같은 Cartographer 설정으로 매핑했고, 62,792개 반환(0.516%)이 제거됐다.
사람 트랙 주변 입력 정제는 확인했지만 전체 지도 품질 개선은 작고 지표별로 혼재했다.

변경 소스(이 README 기준 상대 경로):

| 파일 | 변경 내용 |
|---|---|
| `dynamic_filter/core.py` | ROS 독립 이력 선택, 정책, 외삽, TF 적용과 고정 반경 마스킹 |
| `dynamic_filter/dynamic_filter_node.py` | IMM 구독, 제한 큐/대기, 시계 초기화, 노드별 진단 |
| `config/dynamic_filter.yaml` | 조정 가능한 baseline 기본값 |
| `setup.py`, `package.xml` | NumPy/TF/진단 의존성과 config/문서 설치 |
| `../amr_perception_bringup/launch/cartographer.launch.py` | 두 필터 소유 및 ON/OFF 라우팅 |
| `../amr_perception_bringup/launch/perception_stack.launch.py` | 기존 stub 실행만 제거 |
| `../../stack_master/launch/autonomy.launch.py` | 실제 Cartographer include로 인자 전달 |
| `test/test_core.py`, `test/test_node.py` | 기하·시간·정책 및 ROS 통합 검사 |
| `../amr_perception_bringup/test/test_dynamic_filter_launch.py` | 실제 launch 해석 검사 |
| `tools/cartographer_nan_check/{CMakeLists.txt,main.cpp}` | 설치 Cartographer 변환 함수 검사 |
| `tools/check_live_routes.py` | 별도 도메인의 실제 구독/DDS 전달 검사 |
| `README.md` | 실행, 파라미터, A/B, 검증 결과와 한계 |

## 연결과 실행 소유권

```text
/scan_front_carto -> dynamic_filter_front -> /scan_front_static -> Cartographer scan_1
/scan_rear_carto  -> dynamic_filter_rear  -> /scan_rear_static  -> Cartographer scan_2
/tracked_object_modes --------------------> 두 필터
/tf + /tf_static -------------------------> 두 필터 (odom <- 실제 scan frame)
```

`cartographer.launch.py`가 필터 두 개를 소유한다. `use_dynamic_filter:=false`가 기본이며
이때 필터는 실행되지 않고 원래 두 입력을 Cartographer에 직접 연결한다.
`scan_front_topic`, `scan_rear_topic`은 ON/OFF 모두 **원본 입력 토픽**을 뜻한다.
기존 두 scan 구성과 Lua는 변경하지 않는다. 병합 `/scan`과 `/detections_tracking`,
`/tracks`, `/predictions`, `/lidar_obstacles` 등 인지·주행 경로도 변경하지 않는다.

`perception_stack.launch.py`에서 기존 `/scan -> /scan_static` stub 실행을 제거했다.
현재 활성 소스에서 `/scan_static` 구독자는 발견되지 않았다. 저장소 밖 소비자가 있다면
더 이상 이 토픽이 자동 발행되지 않는 점에 유의한다. 두 개의 센서별 static 토픽을 사용한다.
`autonomy.launch.py`는 필터 ON/OFF, 설정 파일, 원본 토픽 인자를 Cartographer로 전달한다.
`amr_system.launch.py`는 인지/추적을 실행하지만 Cartographer를 포함하지 않는다.
그 파일의 `use_slam`과 `sensors/.../slam_mapping.launch.py`는 기존 slam_toolbox 경로이므로
필터 인자를 추가하지 않았다. Cartographer와 함께 사용할 때 `use_slam:=false`를 유지한다.

## 정책과 시간 계약

* 기본 `confirmed_person`: `ImmTrackedObject.LIFECYCLE_CONFIRMED`이며 확정된
  `class_name == person`인 트랙. 정지한 사람도 제거한다. detector class ID를 가정하지 않는다.
  현재 tracker는 camera/fused 사람 관측 3회로 라벨을 확정하고 LiDAR-only 관측으로
  semantic 수명을 갱신하지 않는다. `person_label_timeout=1.0`초가 지나면 라벨을 지운다.
  필터도 `semantic_age + 빔까지 경과시간`으로 같은 만료를 적용한다.
* `dynamic_only`: CONFIRMED이며 `TrackedObject.MOTION_DYNAMIC`인 트랙.
  클래스 미상도 포함한다. 현재 tracker의 IMM 확률·시간 hysteresis 판정을 그대로 쓴다.
* TENTATIVE/LOST/DELETED는 항상 제외한다. 실제 관측 경과시간은
  `measurement_age + (beam_time - posterior_time)`이며 빔마다 검사한다.
* 배열 header는 posterior 기준 시각이다. 크기·기간이 제한된 이력에서 scan 시작 시각
  이하의 가장 최근 **배열 전체**를 선택한다. 빈 배열도 이력에 저장하므로 그 시각 이후에는
  이전 트랙이 되살아나지 않는다. 수신 순서가 바뀐 배열은 timestamp 순으로 정렬한다.
  오래된/부적격 최신 상태 대신 더 옛날의 적격 상태로 되돌아가지 않는다.
* IMM 공통 상태 `[px, py, vx, vy, omega]`의 모델 확률을 정규화하여 위치·속도의 평균을
  계산하고, 빔 시각까지 `p + v*dt`로 전파한다. **전체 IMM 예측이 아니다.** 회전율,
  모델 전이, 공분산, 메시지의 radius는 사용하지 않는다.
* `beam_time = scan.header.stamp + i*time_increment`. 정확한 빔 시각의
  `odom <- scan.header.frame_id` TF를 timeout 0으로 조회한다. 3D 센서 변환 후 XY 거리로
  판정한다. 현재 트랙 프레임은 odom만 지원하며 다른 프레임은 원본 통과한다.
  최신 TF fallback은 없다. tf2에서 latest를 의미하는 epoch 0도 사용하지 않는다.
* `time_increment=0`이면 모든 빔에 scan stamp를 사용하고 진단 WARN을 낸다.
  임의의 scan_time 기반 시간을 만들지 않는다. ROS clock 역행/clock source 변경 시
  트랙 이력·대기 scan·TF 버퍼를 초기화한다. 전방/후방의 센서 시간차는 보존한다.

## 파라미터

`config/dynamic_filter.yaml`을 복사한 뒤 `dynamic_filter_config:=/절대경로/설정.yaml`로
양쪽 필터에 적용한다. 설정은 시작 시 검증하며 `use_sim_time` 이외에는 실행 중 변경을
거부한다. 변경하려면 노드를 재시작한다. 단일 노드는 `ros2 run dynamic_filter
dynamic_filter_node --ros-args --params-file ... -r scan_in:=... -r scan_out:=...`로도 실행한다.
Cartographer launch를 사용할 때 scan 입력/출력은 launch 인자가 소유하며, YAML의
`scan_input_topic`/`scan_output_topic` 값은 덮어쓴다. 나머지 설정은 YAML을 따른다.

| 파라미터 | 기본값 | 의미/허용 범위 |
|---|---:|---|
| `scan_input_topic`, `scan_output_topic` | `scan_in`, `scan_out` | 파라미터 또는 remapping; 빈 이름/동일 입력·출력 금지 |
| `tracks_topic` | `/tracked_object_modes` | IMM posterior 입력 |
| `filter_radius` | 0.35 m | 유한 양수; 거리 **미만**만 제거, 경계는 유지 |
| `filter_policy` | `confirmed_person` | `confirmed_person` / `dynamic_only` |
| `max_extrapolation` | 0.30 s | 유한 양수; 상한 초과 빔에는 트랙 사용 금지 |
| `max_measurement_age` | 0.30 s | 마지막 실제 관측부터 해당 빔까지 허용 시간 |
| `person_label_timeout` | 1.0 s | tracker 설정과 함께 변경할 semantic 만료 |
| `history_duration`, `history_size` | 2.0 s, 100 | 시간·개수 이중 제한; 개수 1–10000 |
| `pending_queue_size` | 5 | 대기 scan 수, 1–1000; 초과 시 가장 오래된 scan 원본 발행 |
| `tf_wait_timeout` | 0.15 s | TF 또는 causal track 미수신 재시도 대기 상한(steady/wall time) |
| `retry_period` | 0.01 s | 콜백당 scan 하나 처리, 양수이며 wait 이하 |
| `tf_cache_duration` | 10.0 s | TF 이력; history_duration + tf_wait_timeout 이상 |
| `diagnostic_period` | 1.0 s | 노드별 진단 발행 주기 |
| `max_scan_beams`, `max_tracks` | 10000, 200 | 작업량 제한; 각각 1–100000, 1–10000 |
| `use_sim_time` | false | bag 재생 시 true |

모든 시간 값은 유한 양수, `history_duration >= max_extrapolation`이어야 한다.
기본 시간값은 **조정이 필요한 초기 설정**이다. 기존 tracker.yaml의 기록에는 입력 간격
중앙값 약 0.133초, p99 약 0.20초가 있고, 카메라 설정은 15Hz다. LiDAR launch는 S2
scan_mode를 선택하지만 회전 주기를 직접 보장하지 않는다. 여기서는 이 자료를 근거로
짧은 상태 전파 0.30초, TF 대기 0.15초를 선택했으며, 이번 작업에서 실제 지연을 측정한
것은 아니다. 실제 bag의 scan 기간, tracker 도착 지연, TF 지연을 확인해 조정해야 한다.
기하 계산량은 빔 수 × 적격 트랙 수에 비례하므로 작업량 상한을 크게 올리지 않는다.

## 출력과 실패 동작

범위 내 finite 점에 대해 어떤 트랙이든 거리 `< filter_radius`이면 그 range만 NaN으로
바꾼다. 원본 메시지를 변경하지 않고 header/frame/stamp, 모든 angle/time/range 메타데이터,
range 길이/순서, intensity 값/길이/인덱스는 보존한다. 기존 NaN/Inf/범위 밖 값도 유지한다.

설치된 ROS Jazzy Cartographer ROS **2.0.9003**의 `ToPointCloudWithIntensities()`를 직접
링크하는 `tools/cartographer_nan_check`가 있다. NaN/Inf/범위 밖 값은 point로 변환되지
않고, 남은 빔의 원래 시간 간격·intensity 대응이 유지되는 것을 검증한다. 전체 NaN/빈 scan도
빈 point cloud가 된다. 확인한 포트의 `SensorBridge::HandleLaserScan()`은 빈 cloud를
반환 처리한다. 유효한 range_max 반환점을 만들지 않으므로 가려진 뒤쪽을 관측된 자유
공간으로 바꾸지 않는다. 마지막 빔이 제거되면 Cartographer 내부 기준 시각은 마지막
**남은** 빔 기준으로 바뀌지만, 출력 LaserScan stamp와 각 남은 빔의 절대 시각은 유지된다.

트랙 미수신/만료, 프레임 불일치, TF 실패, 작업량 초과 시 원본을 통과시킨다.
TF가 일부 빔에서만 실패해도 해당 scan 전체를 원본으로 되돌린다. TF/causal track을
기다리는 동안 FIFO를 유지하고 wall-time 상한 후 통과시킨다. 실행기 스케줄링 지연과
계산시간은 이 대기 상한 외에 더해질 수 있다. 일부 트랙/빔만 age 상한을 넘으면 유효한
나머지만 마스킹한다. **원본 통과 구간의 동적 객체 제거는 보장하지 않는다.**

`/dynamic_filter_front/diagnostics`, `/dynamic_filter_rear/diagnostics`에 최근 scan의
valid/removed 점 수, used_tracks, zero_time_increment, 외삽·관측·semantic 만료 빔 수,
TF 실패, 원본 통과 reason, 대기열 크기와 누적 카운터를 낸다. 만료 빔 수는 트랙별 합계일
수 있다. 프레임마다 로그를 출력하지 않는다. `no_points_in_radius`는 계산은 성공했으나
제거 대상 점이 없다는 뜻이다. `empty_tracks`는 최근 배열이 명시적으로 비어 있다는 뜻이다.

## OFF / ON 실행

기존 워크스페이스 빌드 후 같은 환경을 각 터미널에서 source한다. 아래 검증 절차의
격리 설치를 쓴다면 기존 install 다음에 `/tmp/amr-dynamic-filter/install/setup.bash`도 source한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
# 터미널 1: 센서, odom TF, 인지/추적을 한 번만 실행
ros2 launch stack_master amr_system.launch.py use_slam:=false use_perception:=true use_object_tracker:=true

# 터미널 2: OFF (ON과 동시에 실행하지 않음)
ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping map:=baseline_off use_dynamic_filter:=false
# 터미널 2: 기존 Cartographer 종료 후 ON
ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping map:=baseline_on use_dynamic_filter:=true

# 이미 저장된 지도에서 autonomy가 전체 스택을 소유할 때는 위 두 실행과 중복하지 않음
ros2 launch stack_master autonomy.launch.py map:=YOUR_MAP use_dynamic_filter:=true
# 원본 토픽을 변경해도 동일한 필터 경로
ros2 launch amr_perception_bringup cartographer.launch.py mode:=mapping use_dynamic_filter:=true scan_front_topic:=/bag/front scan_rear_topic:=/bag/rear
```

amr_system 기본 YOLO 모델 경로를 사용한다. `perception_stack.launch.py`를 단독 실행하면
`yolo_model_path` 기본이 빈 값이므로 실제 모델 경로를 반드시 전달한다. 다른 인지 스택이나
bag이 이미 `/tracked_object_modes`를 발행하면 트래커를 중복 실행하지 않는다.

```bash
ros2 node info /dynamic_filter_front
ros2 node info /dynamic_filter_rear
ros2 node info /cartographer_node
ros2 topic info -v /scan_front_static
ros2 topic info -v /scan_rear_static
ros2 topic info -v /tracked_object_modes
ros2 topic echo /dynamic_filter_front/diagnostics --once
ros2 topic echo /dynamic_filter_rear/diagnostics --once
```

ON에서 static 토픽 publisher는 해당 필터 하나, subscriber는 Cartographer여야 한다.
OFF에서는 Cartographer가 원본 두 토픽을 구독하고 필터 노드는 없어야 한다.

## 같은 bag으로 A/B

필터 자체 비교에는 이미 계산한 동일 posterior를 재생해 인지 결과 차이를 제거한다.
입력 bag의 최소 토픽은 `/scan_front_carto`, `/scan_rear_carto`, `/tracked_object_modes`,
`/tf`, `/tf_static`, `/odom`, `/camera/imu`다. 변경된 odom/imu 인자를 쓰면 해당 토픽도
기록한다. TF에는 odom→로봇 및 센서 정적 변환이 필요하다. 수집 예:

```bash
ros2 bag record -o dynamic_filter_input /scan_front_carto /scan_rear_carto /tracked_object_modes /tf /tf_static /odom /camera/imu
```

1. 입력 bag의 TF에서 기존 map→odom 발행은 제외한다. Cartographer launch의 ekf_carto가
   이 변환을 소유한다. 섞인 `/tf`는 재생 전 odom 트리만 남긴 별도 bag을 준비한다.
2. 새 프로세스에서 `cartographer.launch.py mode:=mapping use_sim_time:=true
   use_dynamic_filter:=false map:=ab_off` 실행. 센서/인지/트래커는 실행하지 않는다.
3. `ros2 bag play dynamic_filter_input --clock`로 재생한다. `/clock`이 bag에 이미 있으면
   중복 clock publisher가 생기지 않게 한 공급원만 선택한다.
4. 출력 `/scan_front_static`, `/scan_rear_static`, 양쪽 diagnostics, `/tracked_pose`,
   `/tracked_pose/odom`, `/map`, `/submap_list`를 별도 bag으로 기록한다. OFF에는 static
   토픽이 없으므로 원본 scan 두 개를 대신 기록한다. 지도는 동일 절차로 저장한다.
5. 모든 SLAM 프로세스를 종료한 뒤 새 map 이름과 `use_dynamic_filter:=true`로 반복한다.
   같은 bag/속도/Lua/odom prior를 사용하고, 설정 파일과 실행 명령도 보관한다.
6. 제거율, 원본 통과율, 만료/TF 실패율, 정적 벽 손실, 사람 흔적, 궤적 안정성을 비교한다.
   실험 중간 loop/seek 대신 각 실험을 새 프로세스로 시작해 Cartographer 자체 시간 이력도
   초기화한다. 필터의 clock reset만으로 Cartographer까지 reset되는 것은 아니다.

인지까지 다시 평가할 때는 병합 전 전·후방 scan, `/scan`, RGB/정렬 depth/CameraInfo,
센서 시간 정보를 포함한 검출 입력을 추가 기록한다. `/detections_tracking`,
`/tracked_object_modes`를 진단용으로 함께 기록하되 재생 posterior와 새 tracker 출력을
동시에 같은 토픽에 발행하지 않는다.

## 재현 가능한 검증

기존 build/install/log를 덮어쓰지 않는 격리 빌드:

```bash
cd /home/smoc/2026amr_ws/src
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
colcon --log-base /tmp/amr-dynamic-filter/log build --base-paths amr_basic_stack --build-base /tmp/amr-dynamic-filter/build --install-base /tmp/amr-dynamic-filter/install --packages-select amr_interfaces dynamic_filter amr_perception_bringup stack_master
source /tmp/amr-dynamic-filter/install/setup.bash
ROS_DOMAIN_ID=198 ROS_LOG_DIR=/tmp/dynamic-filter-ros-logs colcon --log-base /tmp/amr-dynamic-filter/log test --build-base /tmp/amr-dynamic-filter/build --install-base /tmp/amr-dynamic-filter/install --packages-select dynamic_filter
colcon test-result --test-result-base /tmp/amr-dynamic-filter/build --verbose

cmake -S amr_basic_stack/perception/dynamic_filter/tools/cartographer_nan_check -B /tmp/amr-dynamic-filter-nan-check -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/amr-dynamic-filter-nan-check -j2
ctest --test-dir /tmp/amr-dynamic-filter-nan-check --output-on-failure

# 실제 Cartographer OFF/ON 프로세스 및 두 필터의 DDS 전달 검사
# 도메인 198을 검증 전용으로 사용하고 하드웨어/주행 노드는 실행하지 않는다.
python3 amr_basic_stack/perception/dynamic_filter/tools/check_live_routes.py

# launch를 실제 해석하되 프로세스 실행 없이 연결 및 기존 동작 회귀 검사
python3 -m pytest -q amr_basic_stack/perception/amr_perception_bringup/test/test_dynamic_filter_launch.py amr_basic_stack/perception/amr_perception_bringup/test/test_semantic_launch.py amr_basic_stack/stack_master/test/test_autonomy_semantic_launch.py
```

Python 테스트는 core(ROS 없이 실행 가능), 실제 ROS 메시지/TF/clock/queue, 실제 launch
action 확장으로 구분된다. launch 검사는 프로세스 실행을 막고 실제 해석된 remapping과
autonomy 인자 전달을 검증한다. 실기 및 장시간 rosbag 지도 품질 비교는 별도 실험이다.

2026-11-02 검증 환경은 ROS 2 Jazzy, Python 3.12, 설치된 Cartographer ROS 2.0.9003이다.
`amr_interfaces`, `dynamic_filter`, `amr_perception_bringup`, `stack_master`를 `/tmp`에
격리 빌드했다. 인지/추적/주행 의존 패키지는 기존 설치를 사용했으며 전체 스택 재빌드는 아니다.
기존 워크스페이스 build/install/log와 사용자 변경사항은 보존했다.
실제 OFF/ON 노드 구독, static 토픽 publisher, 회전된 후방 센서의 DDS 전달·마스크·메타데이터
보존까지 검증했다. 초기 검색에서 bag을 놓쳤으나 후속 확인에서
`/home/smoc/2026amr_ws/bags`의 원본 MCAP 3개를 확인했다(아래 표).
초기 구현 검증 당시에는 실제 로봇 주행, 사람 제거율, 지도 품질 비교와 실부하 지연
측정을 수행하지 않았다. 후속 작업에서 위 보고서의 실제 bag OFF/ON 오프라인 지도
비교를 완료했다. 정답 기반 사람 제거율 및 실부하 지연 측정은 여전히 수행하지 않았다.

후속 bag 확인: 세 bag의 IMM/전·후방 scan/TF를 현재 ROS 메시지 정의로 직접 읽었다.
모두 `/scan_front_carto`, `/scan_rear_carto`, `/tracked_object_modes`, `/tf`,
`/tf_static`, `/odom`, `/camera/imu`를 포함한다. 트랙 프레임은 odom, 센서 프레임은
laser_front/laser_rear이며 두 scan의 time_increment는 양수다.

| bag (`/home/smoc/2026amr_ws/bags/` 아래) | 길이 | CONFIRMED person 표본 | 그중 DYNAMIC |
|---|---:|---:|---:|
| `mpc_chain_20260923_135606` | 350.75초 | 1502 | 109 |
| `mpc_chain_20260923_135325` | 128.13초 | 495 | 7 |
| `mpc_chain_20260923_113151` | 287.99초 | 556 | 1 |

숫자는 객체×프레임 표본 수이며 고유 사람 수/정답 수가 아니다. 위 CONFIRMED person
표본은 posterior 시점의 measurement_age≤0.30초, semantic_age≤1.0초도 만족했다.
scan 빔 시점의 추가 경과시간 및 실제 마스킹 성공 여부까지 검증한 수치는 아니다.
두 정책 비교에는 `135606`의 시작 후 약 120–140초 구간이 적합한 후보이며, 짧은 전체
재생에는 `135325`를 사용할 수 있다. 세 bag 모두 기존 map→odom TF를 포함하므로
Cartographer A/B 재생 전 그 변환을 제외한다. `/tmp/amr_slam_video_20261022`의 가공
bag에는 IMM 트랙과 동적 `/tf`가 없어 이 필터의 단독 검증 입력으로는 부족하다.

검증 결과:

* 관련 4개 패키지 colcon build 성공.
* dynamic_filter colcon test: **47 passed, 1 skipped**, 오류/실패 0.
  skip은 기존 copyright 검사 템플릿이다. Python 3.12에서 ament_flake8의 fork 경고
  12건이 있었으며 기능/스타일 검사 결과는 모두 통과했다.
* 새 launch 검사 8개 + 기존 semantic/autonomy launch 회귀 검사 19개: **27 passed**.
* 설치 Cartographer 라이브러리 NaN 변환 CTest: **1 passed**.
* 실제 프로세스 OFF/ON 구독 검사와 두 필터 DDS 메시지 검사 통과.
* 결과 파일: `/tmp/amr-dynamic-filter/build/dynamic_filter/pytest.xml`.

실제 프로세스 검사는 연결/필터 전달 범위다. 전체 SLAM 입력(IMU/odom/clock)은 공급하지
않았고, clock 대기 중인 기존 ekf_carto를 종료할 때 비정상 종료가 관측됐다. 필터와
Cartographer 입력 연결 검증 이후의 종료 로그이며, 전체 SLAM 정상 동작/종료를 검증한
것으로 해석하지 않는다. 관련 없는 EKF/지도 저장 노드 코드는 변경하지 않았다.

## 남은 한계

대표점은 사람의 몸 중심/외곽과 다를 수 있다. 0.35m 원이 사람 전체를 제거하지 못하거나
뒤의 정적 구조물을 지울 수 있다. 짧은 CV 평균 외삽은 회전·급가속·다중 모드 분기를
표현하지 못한다. 트랙/사람 라벨 확정 전에는 지도가 오염될 수 있고, 기존 지도 흔적은
제거하지 않는다. 기존 RGB-D–LiDAR 퓨전의 시간 오차·association 오류, 센서 stamp 정확도,
TF 보간 정확도가 그대로 영향을 준다. time_increment가 없는 scan은 deskew할 수 없다.
Python의 빔별 TF 조회 비용은 실제 센서 부하에서 확인해야 한다. 이 baseline은 추적기와
회피 입력을 변경하지 않지만, SLAM에 사용되는 정적 점 감소는 위치 추정 품질에 영향을 준다.
