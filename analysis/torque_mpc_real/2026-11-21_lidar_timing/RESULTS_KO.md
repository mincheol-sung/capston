# 실차 Cartographer의 LiDAR 점별 시각 검증

2026년 11월 21일, smoc-NUC의 `/home/smoc/capston/capston`에서 검증했다. 11월 20일 마지막 세 bag 모두 기존 LaserScan 인덱스로 계산한 점별 시각과 드라이버 재배치 전 순서로 계산한 시각이 달랐다. 두 주행 bag을 같은 조건에서 각각 다시 처리했을 때 빠른 회전 구간에서 저장 지도와의 정합이 개선됐다.

## 원본 bag에서 확인한 시각 차이

대상은 `bags/2026-11-20`에 있는 실차 기록이다. 실차 launch의 앞뒤 기본값은 `inverted=false`, `flip_x_axis=true`, 각도 보상 사용이다. 이 설정과 양의 원본 각도 진행 방향을 적용했다. 세 bag 모두 앞뒤 스캔은 3,240칸이었고 점 간격의 중앙값은 약 30.69 µs였다. 각 bag에 드라이버 설정을 확정할 시작 parameter event와 하드웨어 점별 시각은 없었다.

아래 통계는 기록된 `/scan_front_carto`와 `/scan_rear_carto`에서 거리 한계 안에 있는 모든 유한점에 대해 구했다. 표시한 값은 `abs(j - i) * time_increment`의 밀리초 값이다.

| bag | 앞 스캔 수 | 뒤 스캔 수 | 앞 중앙값 / P95 / 최대 (ms) | 뒤 중앙값 / P95 / 최대 (ms) |
|---|---:|---:|---:|---:|
| 190149 | 1,086 | 1,077 | 22.89 / 47.03 / 73.87 | 23.14 / 47.05 / 66.34 |
| 190506 | 862 | 853 | 22.48 / 47.00 / 58.55 | 22.61 / 47.03 / 76.36 |
| 190642 | 419 | 409 | 23.02 / 47.04 / 58.82 | 23.09 / 47.01 / 55.89 |

전방 `190642`의 첫 스캔에서 원래 칸 0은 LaserScan 칸 1619에 해당했다. 이 점의 기존 시각은 스캔 시작 후 약 50.6 ms였고 수정 후에는 0 s였다. 센서 자체의 발사 시각은 SDK의 각도 정렬·보상 이전에 있다. 이번 수정이 복원하는 시각은 보상 배열의 칸 번호와 기존 호스트 측정 간격으로 추정한 값이다.

## 같은 주행의 지도 정합 비교

Jazzy의 설치된 `cartographer_ros 2.0.9003`을 로컬호스트 전용 ROS 도메인 83과 84에서 실행했다. 두 경로 모두 같은 원본 앞뒤 스캔·IMU·정적 TF만 재생했다. 각 bag에서 이미 지도에 붙어 있고 움직이기 전인 같은 한 자세를 고르고, 동일한 `smoc_lab.pbstream`과 `localization_torque_2d.lua` 설정으로 시작했다. 비교군은 입력 수만 두 LaserScan으로 되돌렸다. `190642`는 전체 43.7초, `190506`은 첫 정사각형 주행을 포함하는 처음 44초를 사용했다. 양쪽 모두 새로 계산한 `/scan_matched_points2`와 `/tracked_pose`를 기록했다.

`map` 좌표계의 매칭 출력점마다 저장된 `smoc_lab.png`의 가장 가까운 점유 셀까지 거리를 구했다. YAML의 점유 문턱 0.65, 해상도 0.05 m, 거리장 이중선형 보간을 사용했다. 지도 밖 또는 1 m 이상은 1 m로 제한했다. 아래 표는 스캔별 평균 거리의 중앙값이다. 원래 시각과 수정 시각이 서로 가장 가까운 출력끼리 45 ms 이내에서 일대일로 짝지었다. 두 출력 시각의 중간에서 `/imu/base_link`의 yaw rate를 보간해 동일한 쌍에 회전 조건을 적용했다.

| bag | 비교 구간 | 스캔 쌍 | 기존 LaserScan | 점별 시각 PointCloud2 |
|---|---|---:|---:|---:|
| 190506 | 전체 | 407 | 78.1 mm | 77.6 mm |
| 190506 | 빠른 회전, 절대 각속도 > 0.5 rad/s | 39 | 93.7 mm | 76.7 mm |
| 190506 | 절대 각속도 < 0.05 rad/s | 331 | 77.7 mm | 77.6 mm |
| 190642 | 전체 | 407 | 80.3 mm | 79.5 mm |
| 190642 | 빠른 회전, 절대 각속도 > 0.5 rad/s | 39 | 105.9 mm | 76.7 mm |
| 190642 | 절대 각속도 < 0.05 rad/s | 331 | 79.9 mm | 80.2 mm |

빠른 회전에서 지도 점유 셀의 10 cm 이내에 들어온 점의 비율도 증가했다. 스캔별 중앙값은 `190506`에서 72.2% → 79.5%, `190642`에서 71.9% → 79.6%였다. 외부 기준 궤적이 없어 절대 측위 정확도의 향상량은 산출할 수 없다. [스캔 쌍별 시각·각속도·거리·비율](paired_wall_metrics.csv)

## 메시지 및 Cartographer 변환 검증

`190642`의 원본 LaserScan을 새 드라이버와 같은 C++ 함수로 변환했다. 하드웨어 노드와 모터는 실행하지 않았다. 설치된 실차 센서 launch의 remap을 전개해서 앞뒤 스캔·점군 연결도 확인했다.

| 원본 LaserScan | 원본에서 복원한 PointCloud2 | 섹터 출력 PointCloud2 | 메시지 수 | 유효점 → 섹터 통과점 |
|---|---|---|---:|---:|
| `/scan_front` | `/points_front_timed` | `/points_front_carto` | 각각 420 | 1,328,556 → 727,937 |
| `/scan_rear` | `/points_rear_timed` | `/points_rear_carto` | 각각 410 | 1,304,080 → 706,919 |

- 모든 2,632,636점의 XY와 원본 LaserScan 칸의 거리·각도를 대조했다. XY 거리 차이의 최대는 0.52 µm 이내였다. Z는 0이고 intensity는 같은 LaserScan 칸의 값과 일치했다.
- 모든 점의 `time`은 `float32(i * time_increment)`와 정확히 일치했고 각 메시지 안에서 증가했다. 레코드당 24바이트이며 `x, y, z, intensity, time`은 FLOAT32, `index`는 UINT32다. `time`의 offset은 16바이트이고 단위는 헤더 시각 기준 초다. `index`는 LaserScan 칸 j다.
- 정적 TF로 독립 계산한 통과점 집합과 일치했다. 필터 입력·출력의 헤더와 선택한 모든 레코드는 바이트 단위로 일치했다. 원본 bag에 기존 섹터 LaserScan이 있는 전방 419개, 후방 409개 모두 마스크가 일치했다. 원본 bag은 각 센서의 원시 스캔보다 섹터 출력을 한 개씩 덜 기록했다.
- 첫 실제 필터 출력에서 전방 1,736점의 `time`은 0~0.10122181 s, 후방 1,729점은 0~0.10214932 s였다. 이 두 메시지의 CDR을 NUC에 설치된 `cartographer_ros::ToPointCloudWithIntensities`에 직접 전달했다. 내부 마지막 점의 상대 시각은 0이고 첫 점은 각각 약 −0.10122 s, −0.10215 s였다. 마지막 점의 기준 시각과 내부 상대 시각으로 복원한 결과는 메시지의 모든 `time`과 최대 19 ns 이내에서 일치했다.
- ROS 그래프에서 `/cartographer_node`의 `/points_front_carto`, `/points_rear_carto` 구독 타입이 모두 `sensor_msgs/msg/PointCloud2`였다. 최신 bag의 새 매칭 출력과 추적 자세는 각각 407개였다.

추가로 선택형 동적 필터를 켜고 같은 bag의 처음 8초를 재생했다. Cartographer가 구독한 `/points_front_static`과 `/points_rear_static`에 각각 62개와 52개가 들어왔고 추적 자세 50개를 받았다. 저장소의 `replay.sh`로도 기존 LaserScan-only bag을 7초 재생해 추적 자세 53개를 받았다.

## 코드와 테스트

드라이버는 기존 스캔과 같은 헤더·거리·각도로 점군을 발행한다. 반 바퀴 이동은 모듈러 연산을 써서 홀수 길이에서도 모든 칸을 한 번씩 대응시킨다. 섹터 필터는 로봇 좌표계에서 방위를 검사하고, 통과한 점은 원래 LiDAR 좌표계의 레코드 전체를 복사한다. 선택형 동적 필터는 스캔 헤더와 원본 칸 번호로 마스크를 점군에 적용한다.

다음 파일은 `src/amr_basic_stack` 기준이다.

| 위치 | 수정·추가한 파일 |
|---|---|
| `sensors/rplidar_ros` | `src/rplidar_node.cpp`, `src/timed_cloud.hpp`, `src/scan_to_timed_cloud.cpp`, `test/test_timed_cloud.cpp`, `CMakeLists.txt`, `package.xml` |
| `sensors/amr_bringup` | `launch/sensors.launch.py` |
| `sensors/amr_laser_tools` | `amr_laser_tools/scan_sector_filter.py`, `amr_laser_tools/timed_cloud.py`, `amr_laser_tools/masked_timed_cloud.py`, `launch/scan_merge.launch.py`, `test/test_timed_cloud.py`, `setup.py`, `package.xml` |
| `perception/amr_perception_bringup` | `config/localization_2d.lua`, `config/localization_sim_2d.lua`, `launch/cartographer.launch.py`, `test/test_dynamic_filter_launch.py`, `package.xml` |
| `controller/mpc` | `tools/slam_replay/replay.sh` |

Cartographer의 설치된 `LoadOptions`로 작업 트리에서 빌드·설치한 Lua 10개를 읽었다. 실차 측위 4종(기본, torque, torque_acc1, torque_odom)은 스캔 0개와 점군 2개를 사용한다. 매핑 4종(기본, HQ, IMU+scan, 시뮬레이터), SLAM, 시뮬레이터 측위는 스캔 2개와 점군 0개를 사용한다.

네 패키지가 `colcon build --symlink-install`로 빌드됐다. 시스템 setuptools를 사용하기 위해 `PYTHONNOUSERSITE=1`을 지정했다. C++ 테스트 2개, Python 테스트 99개가 통과했고 3개가 건너뛰어졌다. Python 두 패키지에서 각각 flake8·pep257가 실패했다. 같은 네 린트 테스트를 수정 전 HEAD에서도 실행해 실패를 재현했다.
