# AMR 센서퓨전 코드 맵 (기능 → 파일 경로)

두 리포로 구성. **로봇 스택**=`amr_jazzy_basic_stack`(구동·URDF·센서 드라이버·라이다 병합), **인지·측위**=`amr_ws`(퓨전·EKF·SLAM·통합 launch).
NUC 실배포 경로: `~/test/amr_jazzy_basic_stack`, `~/test/amr_ws`.

---

## 1. 통합 실행 (여기부터 봄)

| 기능 | 파일 |
|---|---|
| **전체 시스템 한 번에** (로봇+센서+인지) | `amr_ws/src/amr_perception_bringup/launch/full_system.launch.py` |
| 인자: `mock_mode` / `use_sensors` / `use_perception` / `use_joystick` / `yolo_model_path` | 위 파일 `DeclareLaunchArgument` |
| CycloneDDS 큰 메시지 설정 자동 주입 | 위 파일 `set_cyclonedds_uri` + `config/cyclonedds.xml` |
| RViz 시각화 | `amr_perception_bringup/launch/rviz.launch.py` + `rviz/amr_perception.rviz` |
| SLAM(지도작성) | `amr_perception_bringup/launch/slam.launch.py` + `config/slam_toolbox.yaml` |
| 시스템 사전설정(rmem_max·USB autosuspend) | `amr_ws/scripts/setup_nuc_system.sh` |

---

## 2. 인지·측위 (amr_ws)

| 기능 | 파일 / 조절 위치 |
|---|---|
| **RGB-D 카메라 검출**(color+depth 동기화→YOLO→3D 역투영→TF→`/detections`) | `sensor_fusion/sensor_fusion/rgbd_detection_node.py` |
| ├ YOLO(OpenVINO) 검출 | 위 파일 `YoloOpenVINODetector` 클래스 |
| ├ 객체 대표 위치(ROI depth median) | 위 파일 `RgbdDetectionNode` 클래스 |
| ├ 파라미터(topic·slop·depth_scale·camera_position·yolo_*) | 위 파일 `__init__`의 `declare_parameter` |
| **카메라-LiDAR late fusion**(`/detections`+`/lidar_clusters`→`/detections_fused`) | `sensor_fusion/sensor_fusion/camera_lidar_fusion_node.py` |
| YOLO OpenVINO 변환 스크립트 | `sensor_fusion/scripts/export_yolo_openvino.py` |
| 검출 마커 시각화(`/detections`→MarkerArray) | `sensor_fusion/sensor_fusion/detection_markers_node.py` |
| 노드 등록(entry_points) | `sensor_fusion/setup.py` |
| **EKF 측위**(odom+IMU 융합→odom→base_link TF) | `amr_perception_bringup/launch/localization.launch.py` + `config/ekf.yaml` |
| 인지 스택 묶음(sensor_fusion+markers+stub들) | `amr_perception_bringup/launch/perception_stack.launch.py` |
| 커스텀 메시지 정의 | `amr_interfaces/msg/*.msg` (DetectedObject, TrackedObject, ObjectPrediction, RiskLayeredModel 등) |
| 추적/예측/위험/동적필터/시맨틱SLAM (**현재 stub**) | `object_tracker/`, `motion_predictor/`, `risk_model/`, `dynamic_filter/`, `semantic_slam/` 각 `*_node.py` |

**파이프라인:** `sensor_fusion→/detections→object_tracker→/tracks→{dynamic_filter→/scan_static→semantic_slam→/map, motion_predictor→/predictions}→risk_model→/risk`

---

## 3. 로봇 스택 (amr_jazzy_basic_stack)

| 기능 | 파일 / 조절 위치 |
|---|---|
| **센서 드라이버 실행**(라이다2+카메라, 병합 포함) | `amr_bringup/launch/sensors.launch.py` |
| ├ 라이다 프레임/inverted/포트 | 위 파일 `DeclareLaunchArgument`(`front_lidar_frame`, `*_inverted`, `*_serial_port`) |
| ├ 카메라 namespace/해상도/publish_tf/initial_reset/IMU | 위 파일 `depth_camera_*` 인자 |
| **라이다 병합**(앞+뒤→단일 `/scan`) | `amr_laser_tools/scan_merger` 노드 + `amr_laser_tools/config/scan_merge.yaml` |
| ├ 섹터 분담·target_frame·해상도 조절 | `scan_merge.yaml`(`input_sectors_deg`, `target_frame`, `angle_increment_deg`) |
| **모터 구동**(RoboClaw 메카넘, /odom) | `amr_bringup/launch/base.launch.py` + `config/base.yaml`(wheel_radius 등) |
| 조이스틱 텔레옵 | `amr_bringup/config/joy_teleop.yaml` |
| **URDF 로봇 조립**(링크·센서 장착위치) | `amr_description/urdf/robots/amr_basic.urdf.xacro` |
| ├ 라이다 TF/오프셋 | 위 파일 `xacro:lidar_sensor`(front/rear) + `urdf/sensors/lidar.urdf.xacro` |
| ├ 카메라 TF/오프셋(optical 프레임 포함) | 위 파일 `xacro:rgbd_camera` + `urdf/sensors/rgbd_camera.urdf.xacro` |
| ├ 바퀴/베이스 | `urdf/mech/amr_basic_*.urdf.xacro` |
| robot_state_publisher(URDF→TF) | `amr_description/launch/robot_state.launch.py` |

---

## 4. TF 소유권 (충돌 방지 규칙)

- **URDF(robot_state)**: `base_link → 센서 프레임`(laser_front/rear, camera_link→...→camera_color_optical_frame)
- **EKF(robot_localization)**: `odom → base_link`
- **SLAM(slam_toolbox)**: `map → odom`
- roboclaw는 `publish_tf:=false`, realsense 드라이버는 `publish_tf:=false` (URDF가 소유) → 이중발행 충돌 방지

---

## 5. 알려진 이슈 / 캘리브레이션 대상

- **라이다 스캔 방향 보정 프레임 미사용(추정 버그):** URDF에 `laser_front_scan`/`laser_rear_scan`(부모 대비 180° yaw)이 정의돼 있으나, rplidar 노드는 `frame_id=laser_front/laser_rear`로 발행 → 보정 프레임이 **dead**. RViz에서 스캔이 180° 어긋나 보이면 이것. → `sensors.launch.py`의 `front_lidar_frame`/`rear_lidar_frame`을 `*_scan`으로 바꾸거나, URDF 보정 프레임을 제거하고 rpy로 직접 보정.
- **캘리브레이션:** 카메라 내부(K)는 RealSense 공장값 사용. 외부(라이다↔카메라, IMU↔base) 정합은 URDF/CAD 오프셋이 nominal → 실측 미세보정 필요(아래).

---

## 6. 핵심 트러블슈팅 (재발 방지)

- **`/detections` 안 나옴**: 십중팔구 CycloneDDS 큰 메시지 드롭. `ros2 topic hz`로 `camera_info`(작음)는 뜨는데 `image_raw`(큼)만 0이면 확정 → `setup_nuc_system.sh` + `CYCLONEDDS_URI`.
- **카메라 Device busy**: 좀비 프로세스 → `pkill -9 -f realsense` + USB 재연결.
- **디버깅 원칙**: 카메라 `hz` 실제 확인 후에만 sensor_fusion 붙일 것.

---

## 7. 캘리브레이션 정의 (현재 = URDF/CAD 기반, 데이터 보정 없음)

정합은 전부 CAD URDF 고정 오프셋 = nominal 값. 라이다·카메라가 같은 `base_link`로 정렬됨.

| 항목 | 정의 위치 | 값 |
|---|---|---|
| 카메라 내부 K | RealSense `camera_info`(공장값) | fx/fy/cx/cy |
| 카메라→base_link | `rgbd_camera.urdf.xacro` 호출(로봇 xacro), rpy=**0 0 0 수평** | xyz `0.114,0.048,0.093` → color_optical `0.114,-0.011,0.093` |
| 라이다 앞→base_link | `amr_basic.urdf.xacro` `laser_front` | xyz `0.086,-0.020,0.178` |
| 라이다 뒤→base_link | `laser_rear`, rpy `0 0 π` | xyz `-0.091,0.021,0.178` |
| 병합 `/scan` 기준계 | `scan_merge.yaml` `target_frame` | **base_scan** (= base_link +z 0.177797, 회전 없음) |

→ #5 미세보정 대상: 라이다↔카메라 외부(URDF xyz/rpy 조정), IMU↔base yaw drift. 카메라 K는 재캘리브 불필요.

---

## 8. 융합 로직 상세 (`camera_lidar_fusion_node.py`)

`/detections`와 `/lidar_clusters`를 시간 동기화한 뒤 다음 순서로 처리한다.

1. LiDAR 대표점을 카메라 대표점 규약으로 보정한다.
2. 결합 공분산으로 chi-square 게이팅하고 Hungarian 배정한다.
3. 매칭된 관측은 공분산 가중 평균으로 융합한다.
4. 매칭되지 않은 관측은 CAMERA 또는 LIDAR source로 그대로 전달한다.

결과는 `/detections_fused`로 발행하며, 거리별 잡음·게이트·대표점 보정 파라미터는 `CameraLidarFusionNode.__init__`에서 선언한다.

---

## 9. YOLO 모델 현황

- 현재 `yolo11n` = Ultralytics **COCO 사전학습(80클래스)** → OpenVINO 변환. person·car·truck·bus 등은 있으나 **지게차·팔레트·작업자(안전모)·산업장비 없음.**
- 산업 객체 검출 → 커스텀 fine-tune 모델을 `yolo_model_path`로 교체(구조는 교체 가능). 시각화는 RViz 3D 마커(`/detection_markers`), 영상 위 2D박스 오버레이는 미구현.
