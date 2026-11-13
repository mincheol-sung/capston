# traversable map 디버깅 가이드

실차에서 traversable map 을 눈으로 보고, bag 으로 남기고, 숫자로 판정하는 절차.
2026-09-09 실제로 돌려서 확인한 값만 적었다.

## 도구 목록

| 파일 | 하는 일 |
|---|---|
| `stack_master/scripts/capture.sh` | 돌고 있는 토픽을 bag 으로. 타입 기준 필터, 파일 분할 |
| `stack_master/scripts/map_server.sh` | 저장된 PGM 을 `/map` 으로 발행 (라이프사이클 activate 포함) |
| `stack_master/scripts/tf_jump.py` | bag 의 `/tf` 에서 `map→odom` 불연속을 재서 map 프레임 누적 가능 여부를 판정 |
| `perception/traversability/rviz/traversable_view.rviz` | traversable map 을 바닥층으로, 복셀을 그 위에 |
| `perception/traversability/rviz/voxel_check.rviz` | 복셀만 (Fixed Frame `odom`, 측위 불필요) |

---

## 1. 기동 순서

**순서가 중요하다.** traversability_node 는 관측을 map 프레임 격자에 누적하는데
`free_memory_seconds: 0.0` (무한) 이라, 측위가 붙기 전에 쌓은 FREE 셀은 잘못된
자세로 찍힌 채 **영구히 남는다**. 그래서 그 노드만 맨 마지막에 붙인다.

```bash
# 공통 소싱 (octomap_server, STVL 은 2026-09-15 부터 apt 정식 설치다)
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/install/setup.bash
```

### 1) 센서 · 전처리 · octomap  (traversability 는 빼고)

```bash
ros2 launch traversability voxel_check.launch.py use_rviz:=false use_traversability:=false
```

뜨는 것: robot_state_publisher, roboclaw, imu_to_base, EKF(odom), RPLidar 앞/뒤,
scan_merger, RealSense D455(점군 on, decimation 3), depth_preprocess_node, octomap_server.

확인 (실측 정상값):

```
/scan                       10.0 Hz
/camera/imu                200.0 Hz
/odom                       50.0 Hz
/camera/depth/color/points  15.0 Hz
/occupied_cells_vis_array   15.0 Hz
```

RealSense 의 `HID set_power 1 failed` WARN 은 무해하다 — `/camera/imu` 가 200 Hz 로
나오면 IMU 는 정상이다.

### 2) 기준 맵

```bash
~/2026amr_ws/src/amr_basic_stack/stack_master/scripts/map_server.sh test_imu_scan
```

`lifecycle_manager_map: Managed nodes are active` 가 나오고 `/map` 발행자가 1 이 되면 된다.

### 3) 측위 (cartographer localization)

```bash
ros2 launch amr_perception_bringup cartographer.launch.py mode:=localization map:=test_imu_scan
```

`set_initial_pose_node` 가 "RViz 에서 '2D Pose Estimate' 로 찍으세요" 를 띄우고 기다린다.

### 4) RViz — 노트북에서

X11 forwarding 말고 **DDS 로 직접** 받는 게 낫다. 같은 서브넷이면 된다.

```bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
scp smoc@<NUC_IP>:/home/smoc/2026amr_ws/install/traversability/share/traversability/rviz/traversable_view.rviz /tmp/
rviz2 -d /tmp/traversable_view.rviz
```

노트북에 `octomap_rviz_plugins` 도 `amr_interfaces` 도 **필요 없다**. 복셀은
`visualization_msgs/MarkerArray` 로 오고, 이 설정이 쓰는 디스플레이는 전부 기본
플러그인이다. 순정 ROS 2 Jazzy 면 된다.

RViz 에서 **2D Pose Estimate** 로 로봇의 실제 위치와 방향을 찍는다. 붙었는지는
이렇게 확인한다:

```bash
ros2 run tf2_ros tf2_echo map base_footprint     # 도면 안의 실제 좌표가 나와야 한다
# cartographer 로그에 "N computations resulted in M additional constraints" 의 M > 0
```

찍기 전에는 `map→odom` 이 거의 항등이라 로봇이 map 원점 (0,0) 에 있다고 나온다.
`test_imu_scan.yaml` 의 origin 이 `[-6.786, -7.478]` 이므로 그 자리는 도면 바깥처럼
보인다 — 정상이다.

### 5) traversability_node

```bash
ros2 run traversability traversability_node --ros-args \
  --params-file ~/2026amr_ws/src/amr_basic_stack/perception/traversability/config/traversability.yaml
```

**순서를 신경 쓰지 않아도 된다(2026-09-10 부터).** 이 노드는 `/initialpose` 를
구독해서, 초기위치가 다시 선언되면 누적 기억을 통째로 비우고
`relocalize_settle_seconds`(1.5 s) 동안 누적을 멈춘다. RViz 2D Pose Estimate 든
`amcl_bootstrap_node` 든 같은 토픽이라 양쪽 다 처리된다. 예전처럼 초기위치를
찍은 뒤 노드를 재시작할 필요가 없다.

실측 (합성 입력):

```
530.24  memory blocked=150 free=1020                    누적 중
530.68  observation memory reset — initial pose on /initialpose
531.74  scans=0 clears=0 | memory 0/0 | out blocked=0    정착 구간
533.24  scans=10 clears=10 | memory blocked=150 ...      재개
```

`diagnostic_period: 2.0` 이라 2 초마다 이렇게 찍힌다:

```
scans=10 clears=10 depth=15 | memory blocked=2010 free=9491
                            | out blocked=4979 free=31115 unknown=27970
```

- `scans/clears/depth` = `scan_window_seconds`(1.0 s) 안에 들어온 관측 수
- `memory` = 누적된 관측. `free` 는 무한 기억이라 계속 는다
- `out` = 발행되는 격자의 3-state 셀 수. 기준 PGM 의 blocked 3651 에 관측이 얹힌 결과

기동 직후 0.6 초 동안 `Cannot transform ... "map" does not exist` 가 몇 줄 나오는 건
TF listener 가 차기 전이라 정상이다. 계속 나오면 3) 이 안 떠 있는 것이다.

---

## 1-B. autonomy.launch.py 로 한 번에 (Planning 까지 같이)

위 1~5 단계를 손으로 밟는 대신 통합 launch 를 쓸 수 있다. `autonomy.launch.py` 는
amr_system(센서·base·EKF) + cartographer localization + map_server +
traversability_node + Theta* + DRA-MPPI + MPC + RViz 를 한 번에 띄운다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/install/setup.bash
ros2 launch stack_master autonomy.launch.py map:=test_imu_scan use_voxel_3d:=true
```

**`use_voxel_3d` 는 기본 false 다.** 켜지 않으면 복셀이 한 개도 안 나온다. 이 인자
하나가 세 가지를 동시에 한다 (2026-09-09 추가):

| | 안 켜면 |
|---|---|
| `depth_camera_pointcloud_enable` → true | `/camera/depth/color/points` 자체가 없다 (sensors.launch.py 기본 off) |
| `depth_camera_decimation_enable` → true | 점군이 6.06 MB/s 가 아니라 92 MB/s 가 된다 |
| `voxel_3d.launch.py` include | depth_preprocess_node·octomap_server 가 안 뜬다 |

`stack_master/rviz/autonomy.rviz` 에 `Octomap Voxels (z 색)`,
`Cloud z-band`(켬), `Cloud filtered`(끔) 를 추가해 뒀다.

실측 (use_voxel_3d:=true, 22개 노드):

```
/camera/depth/color/points  14.99 Hz    /occupied_cells_vis_array  13.99 Hz
/traversability/cloud_band  14.99 Hz    /traversable_map_grid       2.00 Hz
```

### 초기위치까지 자동으로 — 손이 갈 일이 없다

`use_amcl_bootstrap:=true` 를 더하면 RViz 2D Pose Estimate 도 안 찍어도 된다.

```bash
ros2 launch stack_master autonomy.launch.py map:=test_imu_scan \
    use_voxel_3d:=true use_amcl_bootstrap:=true \
    use_joystick:=true use_command_gate:=false
```

```
map_server /map ─┐
                 ├─► amcl (tf_broadcast: false) ─► /amcl_pose
/scan ───────────┘                                    │
                                                      ▼
/odometry/filtered ──► amcl_bootstrap_node (수렴+검증) ──► /initialpose
                                       ┌──────────────────────┴───────┐
                                       ▼                              ▼
                        set_initial_pose_node              traversability_node
                        cartographer 궤적 재시작            기억 비우고 1.5 s 정지
```

`map→odom` 의 주인은 계속 `ekf_carto` 다 — `amcl.yaml` 의 `tf_broadcast: false`
때문에 AMCL 은 `/amcl_pose` 만 낸다. lifecycle manager 도 이름과 관리 대상이
`map_server` 쪽과 안 겹친다.

**조건**: AMCL 은 `update_min_d`/`update_min_a` 를 넘는 이동이 있어야 파티클을
갱신한다. 띄운 뒤 조이스틱으로 몇 미터 몰아야 수렴한다. 제자리 회전만으로는
대칭 구간에서 엉뚱하게 붙는다(그래서 스캔-지도 정합 검증이 붙어 있다).

`use_traversability` 가 기본 true 라 그 노드가 초기위치보다 먼저 뜨지만,
`/initialpose` 를 받으면 스스로 기억을 비우므로 문제되지 않는다.

### 조이스틱을 같이 쓸 때

`autonomy.launch.py` 는 `use_joystick` 을 인자로 갖고 있지만 **기본 false** 다.
MPC 의 `keyboard_command_gate` 가 `/cmd_vel` 을 독점하기 때문이다. 손으로 몰려면
게이트도 같이 꺼야 한다:

```bash
ros2 launch stack_master autonomy.launch.py map:=test_imu_scan use_voxel_3d:=true \
    use_joystick:=true use_command_gate:=false
```

둘 다 켜면 `/cmd_vel` 발행자가 둘이 되고, MPC 노드가 ERROR 를 낸다
(`/cmd_vel 발행자 1개 — 드라이버가 두 명령을 받는다`).

---

## 2. 조이스틱으로 주행

### 무엇이 필요한가

`voxel_check.launch.py` 는 조이스틱을 **안 띄운다**. `base.launch.py` 는
`roboclaw_mecanum_node` 만 올린다. 조이스틱 노드는 `bringup.launch.py` 에만 들어 있는데
그걸 쓰면 센서가 두 번 뜨므로, 위 구성에는 두 노드만 따로 붙인다.

```bash
JOY=~/2026amr_ws/src/amr_basic_stack/sensors/amr_bringup/config/joy_teleop.yaml
ros2 run joy joy_node        --ros-args --params-file "$JOY" &
ros2 run amr_base joy_teleop_node --ros-args --params-file "$JOY" &
```

### 연결 확인

```bash
ros2 run joy joy_enumerate_devices  # 인식된 패드의 ID·이름. 표가 비면 안 붙은 것이다
ls /dev/input/js*                   # device_id: 0 이므로 js0 가 있어야 한다
ros2 topic hz /joy                  # autorepeat_rate 20 Hz 라 가만히 둬도 20 Hz 로 온다
ros2 topic echo /cmd_vel            # 스틱을 밀 때만 나온다
```

`joy_enumerate_devices` 의 표가 머리글만 나오고 비어 있으면 패드가 안 붙은 것이다
(2026-09-09 확인). 붙은 뒤 ID 가 0 이 아니면 `joy_teleop.yaml` 의 `device_id` 를 맞춘다.

### 조작 (`joy_teleop.yaml`, Logitech F310/F710 XInput 기준)

| 입력 | 동작 | 파라미터 |
|---|---|---|
| 왼쪽 스틱 상하 (axis 1) | 전 / 후 `linear.x` | `axis_linear_x: 1` |
| 오른쪽 스틱 좌우 (axis 3) | 좌 / 우 평행이동 `linear.y` | `axis_linear_y: 3` |
| LB (button 4) | 반시계 회전 | `button_rotate_ccw: 4` |
| RB (button 5) | 시계 회전 | `button_rotate_cw: 5` |
| — | **deadman 없음** | `enable_button: -1` |

회전은 스틱이 아니라 **버튼으로만** 한다 (`axis_angular_z: -1` 로 꺼져 있다).
deadman 버튼이 없으므로 스틱을 건드리면 바로 나간다.

### 안전 — 두 겹의 타임아웃

| 무엇 | 값 | 동작 |
|---|---|---|
| `joy_timeout_s` | 0.5 s | `/joy` 가 끊기면 joy_teleop 이 0 을 한 번 보낸다 |
| `cmd_timeout_s` (base.yaml) | 0.5 s | `/cmd_vel` 이 끊기면 드라이버가 스스로 멈춘다 |

joy_teleop 은 **할 말이 있을 때만** 발행한다. 스틱을 놓으면 정지를 한 번 보내고
버스를 놓는다. 0 을 계속 쏘지 않는 이유는 노드 docstring 에 있다 — 20 Hz 중립 0 이
25 Hz MPC 와 섞여 `/odom` wz 이득이 0.97 → 0.83 으로 떨어졌던 실측 사례다.

### 속도 상한이 실제보다 크게 잡혀 있다

`joy_teleop.yaml` 의 `max_linear_vel: 3.0` 은 노드 기본값 0.3 을 덮어쓴 값인데,
드라이버가 낼 수 있는 최대는 그보다 작다:

```
max_wheel_rad_per_sec 38.0 × wheel_radius_m 0.04 = 1.52 m/s
```

즉 스틱을 절반(1.5 m/s) 넘게 밀면 그 위는 전부 포화라 반응이 같다.
디버깅 주행은 **0.3~0.5 m/s** 로 낮춰 쓰는 편이 낫다. 단
`ros2 param set` 은 **효과가 없다** — 노드가 `__init__` 에서 한 번만 읽고
`add_on_set_parameters_callback` 이 없다(joy_teleop_node.py:55). 띄울 때 덮어써야 한다:

```bash
ros2 run amr_base joy_teleop_node --ros-args --params-file "$JOY" -p max_linear_vel:=0.4
```

느리게 몰아야 하는 이유가 하나 더 있다 — 빨리 돌수록 스캔 정합이 어긋나
`map→odom` 보정이 커지고, 그게 그대로 격자 오염이 된다 (4절).

### /cmd_vel 소유권

`/cmd_vel` 에 발행자가 둘 이상이면 안 된다. MPC 를 같이 돌릴 때는 조이스틱을 끄고
(`use_joystick:=false`) `amr_mpc` 의 `keyboard_command_gate` 로 소유권을 강제한다.
`mpc_tracking_node` 는 발행자가 둘 이상이면 ERROR 를 낸다.

---

## 3. bag 으로 남기기

```bash
./capture.sh -o tmap_clean --split 120
```

인자 없이 **살아있는 토픽을 전부** 담고 아래만 뺀다 (2026-09-10 결정). `--split` 은
녹화를 끊지 않고 파일만 나눈다 (bag 은 하나, `ros2 bag play` 가 이어서 재생).

| 기본으로 빼는 것 | MB/s | 되살리는 인자 |
|---|---|---|
| 카메라 이미지 전부 (color·depth·정렬 depth·압축본) | 32.3 | `--heavy` |
| `/camera/depth/color/points` (raw 점군) | 6.04 | `--raw` |
| `/occupied_cells_vis_array` (복셀 마커) | 14.14 | `--heavy` |
| `/octomap_full` | 8.83 | `--heavy` |
| `/octomap_point_cloud_centers` | ~3.9 (계산) | `--heavy` |

기본으로 **담는** 카메라 파생 토픽은 `cloud_filtered` 2.27 · `cloud_band` 0.10 ·
`octomap_binary` 0.63 MB/s 다. `/parameter_events` 와 `/rosout` 도 담는다.
점군을 전부 빼려면 `--no-clouds`. 예전 `--clouds` 는 기본값이 되어 무시된다.

이미지를 빼는 이유: 압축본만(color JPEG 1.10 + 정렬 depth PNG 0.85) 담아도 되지만
image_transport 는 **구독자가 있을 때만** 인코딩하므로 녹화가 NUC 에 JPEG/PNG 인코딩
CPU 를 새로 얹는다. 필요해지면 `--heavy` 로 한 번 따면 된다.

D455 실측 전체 (640x480x15, `depth_camera_decimation_enable:=true`, 12 s 동시 구독):

| 토픽 | MB/s |
|---|---|
| `/camera/color/image_raw` | 13.83 |
| `/camera/aligned_depth_to_color/image_raw` | 9.22 |
| `/camera/depth/image_rect_raw` | 9.22 |
| `/camera/depth/color/points` | 6.04 |
| `/camera/color/image_raw/zstd` | 5.81 |
| `/camera/depth/image_rect_raw/compressedDepth` (PNG) | 2.16 |
| `/camera/depth/image_rect_raw/zstd` | 2.05 |
| `/camera/color/image_raw/compressed` (JPEG) | 1.10 |
| `/camera/aligned_depth_to_color/image_raw/compressedDepth` | 0.85 |
| IMU·metadata·camera_info 합 | ~0.3 |
| 합계 | 51.4 |

**decimation 은 발행되는 `depth/image_rect_raw` 크기를 안 줄인다.** 점군은 25k pt
(6.04 MB/s) 로 줄지만 `depth/image_rect_raw` 는 9.22 MB/s = 640 x 480 x 2 B x 15 Hz
원해상도 그대로다 (realsense-ros 4.58.1). 그러니 이미지 녹화 용량은 decimation 과 무관하다.

`aligned_depth_to_color/image_raw` 도 9.22 MB/s 지만 이건 **판별이 안 된다** — 정렬
결과는 정의상 color 해상도(640x480)로 나오므로, decimation 된 depth 로 만들어져 유효
픽셀이 성겨졌어도 바이트 크기는 같다. `sensors.launch.py` 주석의 "켜면 정렬 깊이가
바뀐다(roi_median_3d 영향)" 는 이 측정으로 반박되지 않는다. 확인하려면 두 설정에서
정렬 depth 의 0 이 아닌 픽셀 수를 세어 비교해야 한다.

traversability 계열 실측 대역폭 (`ros2 topic bw`, voxel_check 구성):

| 토픽 | 대역폭 | 비고 |
|---|---|---|
| `/occupied_cells_vis_array` | 14.14 MB/s | RViz 표시 전용. **기본 제외** (`_vis_array$`) |
| `/octomap_full` | 8.83 MB/s | octree + log-odds |
| `/camera/depth/color/points` | 6.06 MB/s | decimation 3. 끄면 92 MB/s |
| `/traversability/cloud_filtered` | 2.27 MB/s | 12.5k pt/frame |
| `/octomap_binary` | 627 KB/s | 같은 octree 를 22 배 싸게 |
| `/traversability/cloud_band` | 101 KB/s | 0.5k pt/frame |

`cloud_filtered` 는 z 를 안 자른 상태라 여기서 z-band, octomap/STVL, traversability
파라미터를 전부 다시 맞출 수 있다. raw 점군이 필요한 건 `self_filter_box`,
`min_range`/`max_range`, `voxel_leaf` 를 바꿀 때뿐이고 그때만 `--raw` 를 쓴다.

재생은 반드시 `--clock` 과 `use_sim_time:=true` 를 짝으로:

```bash
ros2 bag play <bag> --clock
ros2 launch traversability traversability.launch.py use_sim_time:=true
```

한쪽만 켜면 `memory_seconds`(5.0)·`free_memory_seconds`·`clear_hit_threshold` 의
누적창이 재생 속도에 따라 늘었다 줄었다 한다. `-r 0.5` 면 창이 절반만 덮인다.
재생 전에 실물 노드를 반드시 내릴 것 — 같은 토픽을 두 곳이 발행하면 격자가 섞인다.

QoS override 파일은 **필요 없다**. `/map` 과 `/traversable_map_grid` 는
`TRANSIENT_LOCAL` 로 발행되는데, rosbag2 가 metadata 의 `offered_qos_profiles` 에
그대로 기록하고 `rosbag2_player` 가 복원하는 것을 확인했다.

---

## 4. 판정 — map 프레임에 누적해도 되는가

traversability_node 는 관측을 **map 프레임 격자**에 누적한다. `map→odom` 이 움직이면
과거에 찍은 셀은 그대로 있고 새 관측만 새 좌표로 찍혀 격자가 어긋난다.
`memory_seconds`(5.0 s) 창 안의 배회폭이 1 셀(0.05 m)을 넘으면 이 구조는 성립하지 않는다.

```bash
python3 stack_master/scripts/tf_jump.py ~/2026amr_ws/bags/tmap_clean_*
```

| 5 s 창 배회폭 90 백분위 | 결론 |
|---|---|
| < 1 셀 | 구조 유지. `scan_use_latest_tf: false` + `tf_timeout: 0.5` 만 고친다 |
| > 1 셀 | 2D 격자를 odom 로컬 윈도우에 누적하고 발행할 때만 map 으로 변환한다 |

두 번째는 `depth_preprocess` 가 이미 하는 구조다 (`output_frame: odom`).
3D 만 그렇게 되어 있고 2D 격자가 map 에 남아 있는 것이 지금의 비대칭이다.

측정 시 주의: **초기위치를 여러 번 찍은 런은 못 쓴다.** `/initialpose` 마다 1 m 이상
점프가 나고, `tf_jump.py` 가 ±3 s 를 빼도 잔여 오염이 남는다. 한 번만 찍고 주행할 것.

"점프가 나면 기억을 리셋" 하는 방식은 **기각됐다**. 1 셀 임계면 초당 1.6 회,
2 셀로 올려도 초당 0.85 회 발동해서 누적이 남지 않는다.

---

## 5. 밟은 함정

1. **측위 전에 쌓은 관측은 영구히 남았다.** FREE 기억이 무한이라 잘못된 자세로
   뚫은 셀이 안 지워진다. 한 번 그렇게 32042 셀을 만들었다. → 2026-09-10 에
   `/initialpose` 구독으로 해결. 재시작 절차는 이제 필요 없다.
   `/initialpose` 구독은 반드시 **BEST_EFFORT** 여야 한다 —
   `amcl_bootstrap_node` 가 그 QoS 로 내므로 RELIABLE 구독자는 한 개도 못 받는다
   (`test/test_relocalize_reset.py` 가 이걸 지킨다).
2. **`/map` 발행자가 둘일 수 있었다.** `semantic_slam_node` STUB 가 빈 격자를 2 초마다
   `/map` 에 쐈다. `use_semantic_slam` 기본값을 `false` 로 바꿔 막았다.
3. **복셀 마커가 타입 필터에 안 걸린다.** `MarkerArray` 라 `--exclude-topic-types` 를
   빠져나가면서 14 MB/s 다. 이름 regex(`_vis_array$`)로 뺐다.
4. **`pkill -f` 를 툴 셸에서 쓰면 자기 자신을 죽인다.** PID 나 `pkill -x` 를 쓸 것.
5. **`/tmp` 는 재부팅에 사라진다.** 그래서 `map_server.sh` 를 리포로 옮겼다.
6. **`voxel_check.rviz` 에는 2D Pose Estimate 툴이 없다** — 측위가 필요 없는 용도라서다.
   `traversable_view.rviz` 에는 넣어 뒀다. 실행 중인 RViz 라면 툴바 `+` 로 추가해도 된다.
7. **`ros2 bag record` 의 `-d` 는 시간제한이 아니라 분할 주기다.** 총 시간 제한은
   `timeout --signal=INT` 로 건다 (capture.sh 가 그렇게 한다).

## 6. 내리기

```bash
# PID 로 내린다. pkill -f 는 쓰지 말 것
ps -eo pid,args | grep -E "cartographer|map_server|rplidar|realsense|traversability" | grep -v grep
```

`ros2 daemon` 은 CLI 캐시라 남아 있어도 하드웨어와 무관하다.
