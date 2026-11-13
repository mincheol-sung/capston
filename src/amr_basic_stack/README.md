# 2026 AMR Basic Stack Layout

This workspace merges the deployed robot stack and perception workspace into one
colcon source tree.

```text
2026amr_ws/
  src/
    amr_basic_stack/
      sensors/       # robot base, drivers, joystick, URDF, sensor bringup
      perception/    # localization, mapping, fusion, tracking, risk modules
      planner/       # reserved for future planning packages
      controller/    # reserved for future control packages
      stack_master/  # top-level launch/config package
```

Build from the workspace root:

```bash
cd ~/2026amr_ws
colcon build --symlink-install
```

## 열린 항목

작업 전에 확인. 결론이 나면 **해당 위치의 알림과 이 줄을 같이 지운다.**

| 항목 | 어디 | 어떻게 닫나 |
|---|---|---|
| cartographer 의 오도메트리 prior 를 `/odom`(휠 원본)으로 둘지 `/odometry/filtered`로 둘지 — 실측 검증 안 됨 | `perception/amr_perception_bringup/launch/cartographer.launch.py` 의 `NOTICE_ODOM_PRIOR`. 기본값으로 띄우면 콘솔에 전문이 뜨고, `odom_topic` 을 바꿔 띄우면 자동으로 조용해진다 | 두 토픽으로 같은 경로를 주행해 `map->odom` 안정성을 비교하고, 이긴 쪽을 `default_value` 로 박은 뒤 알림 블록 삭제 |

배경: 2026-09-03 에 `ekf.yaml` 이 휠 오도메트리의 절대 위치를 융합하던 것을 고쳤다
(속도만 받는다 — `ekf_carto_*.yaml` 세 개가 이미 쓰던 패턴). 그래서 `/odometry/filtered`
가 이제 자기일관적이고 회전 정확도도 자이로 기준이라, 위 선택을 다시 볼 값어치가 생겼다.

Top-level launch entry:

```bash
ros2 launch stack_master amr_system.launch.py
```

The default YOLO model is stored under `perception/models`:

```text
perception/models/yolo11n_openvino_model
```

The default launch starts the robot stack and sensors, matching the previous
`amr_perception_bringup full_system.launch.py` behavior. It uses real RoboClaw
output and starts joystick teleop by default:

```bash
ros2 launch stack_master amr_system.launch.py
```

For ROS-only checks without LiDAR or camera hardware, disable the sensor layer:

```bash
ros2 launch stack_master amr_system.launch.py mock_mode:=true use_sensors:=false use_joystick:=false
```
