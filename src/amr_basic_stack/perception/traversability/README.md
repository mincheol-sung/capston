# traversability

기준 2D 맵과 현재 관측을 합쳐 **3-state traversable map** 을 Planning 에 준다.
설계문서 §4/§7.1 의 "주행 가능성 판단 → Planning" 자리다.

실차에서 띄우고 보고 bag 으로 남기는 절차는 [docs/DEBUGGING.md](docs/DEBUGGING.md)
에 있다 — 기동 순서(측위가 붙은 뒤에 이 노드를 올려야 하는 이유 포함), 조이스틱
주행, `capture.sh` 대역폭 표, `tf_jump.py` 판정 기준.

## 계약

토픽 `/traversable_map`, 타입 `amr_interfaces/TraversableMap`, frame `map`, 2 Hz.
RViz 확인용으로 같은 격자를 `/traversable_map_grid` (`nav_msgs/OccupancyGrid`)
에도 낸다.

### 값 규약 — 이 세 값만 쓴다

| 값 | 의미 |
|----|------|
| `0` | 통과 가능 |
| `100` | 통과 불가 |
| `-1` | 미확인 (uint8 바이트로는 `255`) |

`nav_msgs/OccupancyGrid.data` 는 `int8[]` 이다. `255` 는 저장할 수 없고
바이트 `0xFF` 로 나가 `-1` 로 읽히는데, 그게 마침 ROS 의 unknown 규약이라
그대로 맞는다. **`1` 을 통과 불가로 쓰면 안 된다** — 소비자들이
`occupied_thresh 0.65` / `occupied_threshold 65` 로 판정하므로 `1` 은
"1% 점유" 즉 free 로 읽힌다. 1~99 는 cost 를 넣을 2단계를 위해 비워 둔다.

### 배열 순서

`OccupancyGrid` 규약을 따른다 — row-major, **row 0 = 최소 y**, `origin` 은
셀 (0,0) 의 좌하단. PGM 이미지 배열(row 0 = 최대 y)과 상하가 반대다.

이미지 순서를 쓰는 소비자는 경계에서 `np.flipud` 를 한 번 걸어야 한다.
Theta\* 쪽은 `global_planner/common/traversable_map.py` 가 처리한다.
DRA-MPPI 의 `DistanceField` 는 ② 규약 그대로라 변환이 필요 없다.

### 미확인 처리는 소비자마다 다르다

설계문서 §4 에 따라 갈린다.

| 소비자 | 정책 | 근거 |
|--------|------|------|
| Theta\* (global) | 통과 **허용** | "글로벌 경로는 기준 지도상 미리 생성할 수 있지만" |
| DRA-MPPI (local) | 통과 **불가** | "실제 주행은 확인된 범위와 정지 가능한 거리 안에서" |

Theta\* 쪽은 `theta_star.yaml` 의 `traversable_unknown_is_obstacle`
(기본 `false`) 로 바꾼다.

### 로봇 형상 팽창

인지 측은 팽창하지 **않는다** (`inflation_radius: 0.0`). Planning 이 자기
footprint 로 한다. 이 필드가 0 이 아니면 소비자는 그만큼 덜 팽창해야 한다
(설계문서 §7.1 "어느 단계에서 적용했는지 명시").

## 병합 규칙

셀마다 다음 순서로 정한다.

```
LiDAR 광선이 관통           -> 0     (해제. 아래 "관측 기반 해제" 참조)
빔/점군이 명중              -> 100   (해제를 덮는다 - 있는 것을 없다고 하면 위험)
관측 없음:
    관측 기억이 유효시간 내   -> 그 값 유지
    기준 맵이 점유            -> 100
    기준 맵이 free            -> 0    (trust_base_map_free 일 때만)
    그 외                     -> -1
```

만료는 **셀별** 최종 관측 시각으로 판정한다. 격자 전체에 타임스탬프를 하나만
두면 매 주기 기억을 다시 실어 나르면서 시각이 갱신되어 만료가 영원히 오지
않는다.

BLOCKED 과 FREE 는 만료 시간이 다르다.

| 기억 | 파라미터 | 기본값 | 이유 |
|---|---|---|---|
| BLOCKED | `memory_seconds` | 5 s | 동적 장애물이 시야에서 사라졌을 때 영원히 남지 않게. 만료되면 UNKNOWN 이지 빈 공간이 아니다(§6) |
| FREE | `free_memory_seconds` | 0 = **무한** | 광선이 관통했다는 증거는 시간이 지나도 사라지지 않는다. 다시 막히면 hit 이 덮으므로 영구 보존이 위험하지 않다 |

## 관측 기반 해제 (clearing)

설계문서 §6 — "삭제에는 해당 공간을 관측한 증거가 필요하다".

LiDAR 광선이 관통한 셀을 통과 가능으로 세운다. **이것이 없으면 traversable
map 은 장애물을 더할 수만 있고 뺄 수 없다** — 초기 매핑 때 있던 팔레트를
치워도 격자에서 영원히 사라지지 않는다(§6 의 "재관측 후 계획용 지도에 별도
해제 규칙").

```
raycast_free_cells(sensor_origin, endpoints, ...)
  끝점 셀은 항상 남긴다 (장애물 자신)
  그 앞 clear_stop_margin 셀도 남긴다 (측정 오차 여유)
  서로 다른 스캔 clear_hit_threshold 개가 관통해야 해제 (노이즈 방어)
```

안전 장치 세 가지:
1. 같은 주기에 관통과 명중이 겹치면 **명중이 이긴다**.
2. 끝점과 그 앞 한 셀은 절대 해제하지 않는다.
3. 표본을 성기게 잡으면(`clear_step_ratio` ↑) 해제가 줄어들 뿐 늘지 않는다 —
   틀리는 방향이 안전한 쪽이다. 테스트로 박제했다.

### 센서별 근거를 따로 기억한다

`_source` 배열(uint8 비트마스크)이 셀별로 **누가 이 셀을 막았는지**를 들고
있다. `SOURCE_LIDAR = 1`, `SOURCE_DEPTH = 2`.

이걸 기록하지 않으면 `use_voxel_3d` 와 `enable_clearing` 을 같이 켤 수 없다.
LiDAR 평면은 **0.2178 m** 이고 z-band 는 `[0.033, 0.31]` 이므로, 높이가
**0.033 ~ 0.2178 m** 인 물체(팔레트 약 0.14 m, 문턱, 낮은 박스)는 LiDAR 빔이
그 위를 지나간다. 값 `100` 하나만 보면 그 관통이 해제 근거가 되어 D455 가
본 낮은 장애물을 지운다. `free_memory_seconds: 0` 이라 그 소거는 영구다.
설계문서 §6 이 정확히 이걸 금지한다 — "LiDAR 빔이 장애물 위를 지나갔다고
해서 낮은 장애물이 사라졌다고 판단하면 안 된다".

`encoding.update_memory_sourced` 의 규칙:

```
hit    -> 해당 비트를 세운다
clear  -> 해당 비트만 내린다        (다른 센서의 비트는 건드리지 않는다)
같은 주기에 hit 과 clear 가 겹치면 hit 이 이긴다
근거가 하나도 남지 않은 셀만 FREE. 하나라도 남으면 BLOCKED
증거가 없는 셀은 기억을 그대로 들고 간다
```

`memory_seconds <= 0` 이면 BLOCKED 을 만료시키지 않는다(`free_memory_seconds`
와 같은 규약). 그러면 해제가 오직 관측 증거로만 일어나므로 **장애물을
기억하고 있다가 그 자리를 다시 봤을 때 없으면 지운다** 가 된다.
`memory_seconds: 0.0` 이 현재 기본값이다.

### D455 도 자기 해제 증거를 만든다

`_depth_cleared_cells` 가 **카메라 원점**에서 밴드 끝점까지 LiDAR 와 같은
`raycast_free_cells` 를 쏜다. 원점은 `camera_frame`(기본 `camera_link`) 이며
URDF 상 `base_link -> camera_link xyz 0.11398 0 0.092772` 라 `base_footprint`
원점과 x 로 0.114 m(약 2.3 셀) 떨어져 있다. 이 오프셋을 무시하면 근거리 셀의
해제가 어긋난다.

끝점이 밴드 반사이므로 그 광선은 해당 방위에서 밴드를 실제로 통과했다 — 더
가까운 곳에 밴드 물체가 있었다면 끝점이 더 가까웠을 것이다. **반사가 없는
방위에는 광선이 생기지 않아 해제도 없다**: 밴드보다 높은 구조물에 가려
밴드를 못 본 방위가 여기서 자동으로 제외된다.

3D 해제(밴드 전체가 비었다는 증거)는 여전히 OctoMap/STVL 담당이다(§3.3).
여기서 하는 것은 2D 밴드 평면에 대한 해제이고, LiDAR 평면 해제와 같은
강도의 증거다.

비용 실측 (1600 빔, 0.05 m/cell):

```
step 1.0×res, stride 1 :  6.5 ms
step 0.5×res, stride 1 : 13.0 ms
step 0.5×res, stride 2 :  6.5 ms   <- 기본값
```

`trust_base_map_free` 는 1단계에서만 `true` 다. 기준 맵의 free 는 0.218 m
높이 LiDAR 평면에서 관측된 것이고 1단계의 traversability 정의가 바로 그
평면이기 때문이다. 2단계에서 D455 z-band 투영이 들어와 정의가 넓어지면
`false` 로 뒤집는다 — 그때부터 기준 맵 free 는 통과 가능의 근거가 못 된다.

## Planning 연결 상태

**2026-09-09 현재 Planning 과 끊어져 있다.** `theta_star.yaml` 의
`use_traversable_map: false` 다. 인지 쪽에서 격자를 다듬는 동안 글로벌 경로가
그 변화에 흔들리지 않게 한 것이다. 노드는 계속 돌고 `/traversable_map`,
`/traversable_map_grid` 로 발행하므로 RViz 로 확인할 수 있다.

켜기 전에 반드시 함께 고칠 것 — Theta\* 의 `_layered_costmap` 은 기준 PGM 을
항상 깔고 OR 만 하므로 traversable map 이 장애물을 **더할 수만 있고 뺄 수
없다**. 실측(test_imu_scan): 미관측 33,446 셀 중 96.5 % 가 기준 PGM 단계에서
이미 장애물이라 `traversable_unknown_is_obstacle: false` 가 무력화된다.
traversable map 은 이미 `/map` 을 자기 안에 합쳐 두므로, 신선할 때는 base 를
OR 하지 말고 **대체**해야 한다.

## 정보원

기준 PGM + 2D LiDAR(명중·해제) + **D455 z-band**(명중만). `/map` 은 `nav2_map_server` 가 내는
전체 맵을 쓴다. Cartographer localization 모드의 `/map_carto` 를 쓰면 안 된다 —
`pure_localization_trimmer` 때문에 새로 만든 서브맵 3개뿐이라 그 밖은 전부
미확인이 된다.

`z_min`/`z_max` 와 D455 z-band 투영은 아래 3D 절을 볼 것.

---

# 3D — D455 z-band 투영과 OctoMap / STVL

## 신호 흐름

```
D455 /camera/depth/color/points  (camera_depth_optical_frame)
  |
  v  depth_preprocess_node          설계문서 §3.2-3 "공통 D455 전처리"
  |    TF -> base_footprint
  |    자기관측 상자 제거 -> 수평거리 0.30~6.0 m -> 0.03 m voxel 다운샘플
  |
  +--> /traversability/cloud_filtered   전체 높이, odom      -+
  |                                                            |
  +--> /traversability/cloud_band       z-band, base_footprint |
         |                                                     |
         v traversability_node                                 |
           map 으로 변환 -> 2D 셀 -> LiDAR 관측과 합집합       |
           /traversable_map                                    |
                                                               |
       +-------------------------------------------------------+
       |  같은 점군을 두 3D 표현이 나눠 받는다 (§3.3 비교 조건)
       v
    octomap_server 2.3.1        /occupied_cells_vis_array  (RViz voxel, 높이색)
                                /octomap_full  /projected_map
    STVL 2.5.5                  /voxel_grid    (RViz voxel)
    (nav2_costmap_2d 독립 노드)  /costmap  /stvl_layer
```

**2D z-band 투영은 우리가 한다.** octomap 의 `/projected_map` 이 아니라
`traversability_node` 가 `/traversability/cloud_band` 를 받아 자기 격자에
직접 투영한다. 격자 기하가 `/map` 과 정확히 같아야 하기 때문이다.

## z-band 값과 그 근거 (URDF 실측)

```
base_footprint -> base_link      z = 0.04       (amr_basic.urdf:42)
base_link -> camera_link         z = 0.092772   -> 카메라 높이 H = 0.132772 m
base_link -> laser_*_joint       z = 0.177797   -> LiDAR 마운트 0.2178 m
base_link 충돌박스               0.28584 x 0.18841 x 0.16701, 지면 위 0.0219~0.1889
```

설계문서 §4.1 의 초기 후보 `z_min = -0.15 m` (카메라 기준) 는 이 로봇에서
`H + z_min = -0.017 m` 로 **지면보다 1.7 cm 아래**다. 문서의 워크드 예시는
`H = 0.50 m` 를 가정했으므로 실제 H 에서는 부호가 뒤집혀 바닥면이 통째로
밴드에 들어온다. 그래서 카메라 기준 `-0.10 m`, **지면 기준 `z_min = 0.033 m`**
를 쓴다. `z_max = 0.31 m` 는 로봇 최고점(LiDAR 상단 약 0.258 m) + 여유 0.05 m.

이 값은 세 곳에 있고 **반드시 같아야 한다**:
`depth_preprocess.yaml` · `traversability.yaml` · `octomap.yaml`.

### 근거리 바닥 사각지대

`camera_joint` 의 rpy 가 `0 0 0` 이라 optical axis 는 수평이다. VFOV 58도
기준 바닥이 시야에 들어오는 거리는 `H / tan(29도) = 0.24 m` 이고, D455 의
depth Z-min(약 0.4 m) 때문에 **로봇 앞 0.40 m 안쪽 바닥은 관측 불가**다
(설계문서 §9, §3.3 "사각지대 유지율").

## OctoMap / STVL — 공식 바이너리를 그대로 쓴다

소스 포크가 아니다. packages.ros.org 의 공식 바이너리를 apt 로 설치해 쓴다.
버전은 설계문서 §3.3 이 ROS 2 Jazzy 용으로 지목한 그대로다:

| | 버전 | 비고 |
|---|---|---|
| `octomap_server` | 2.3.1 | 독립 노드. `/cloud_in` 구독 |
| `spatio_temporal_voxel_layer` | 2.5.5 | `nav2_costmap_2d` **독립 노드** 안의 Layer 플러그인. 경로 계획기는 안 띄운다 |
| `octomap_rviz_plugins` | 2.1.0 | RViz 표시 |
| `pcl_ros` · `octomap_msgs` · `octomap_ros` · `openvdb_vendor` | — | 의존성 |

환경에 얹으려면 평소 소싱이면 된다:

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/install/setup.bash
```

설치는 한 번만:

```bash
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins \
                 ros-jazzy-spatio-temporal-voxel-layer
```

2026-09-15 이전에는 sudo 를 못 써서 같은 `.deb` 를 `dpkg -x` 로 `vendor/` 에 풀고
`setup_vendor.bash` 가 AMENT_PREFIX_PATH·LD_LIBRARY_PATH 에 그 위치를 끼워 넣었다.
지금은 apt 정식 설치라 그 디렉터리와 스크립트가 없다.

### 실측으로 알게 된 함정 두 가지

1. **`occupancy_min_z`/`occupancy_max_z` 는 2D 투영만 거르지 않는다.**
   octomap_server 는 이 구간을 **발행되는 voxel 마커에도** 적용한다.
   0.033~0.31 로 두었더니 입력에 1.08 m 짜리 상부 돌출이 있는데도
   `/occupied_cells_vis_array` 의 voxel z 최고가 0.275 m 였다. RViz 에서 상부
   구조가 통째로 사라진다. 그래서 `-0.30 ~ 2.00` 으로 넓게 둔다.
2. **STVL 토픽은 `/costmap/...` 이 아니다.** `nav2_costmap_2d` 독립 노드는
   이름이 `costmap`, 네임스페이스가 `/` 라서 3D voxel 이 `/voxel_grid`,
   2D 가 `/costmap` 으로 나온다.

### 설계문서 6절과 STVL 의 충돌 (미해결)

STVL 의 시간 감쇠는 시간 경과만으로 voxel 을 제거한다. 문서 6절은 그것을
금지한다. `track_unknown_space: true` 로 **관측된 적 없는** 셀은 255 로
남지만, **감쇠로 사라진** voxel 은 0(FREE)이 되지 255 가 되지 않는다 —
costmap_2d 자료구조에 "만료됨" 상태가 없기 때문이다. STVL 을 채택하려면
만료 voxel 을 미확인으로 강등하는 어댑터가 추가로 필요하다.
`stvl_costmap.yaml` 의 `decay_model: -1` 로 두면 감쇠를 끄고 OctoMap 과 같은
조건에서 비교할 수 있다.

## 실행

2D 만 (기존):

```
ros2 launch traversability traversability.launch.py
```
또는 `autonomy.launch.py` 의 `use_traversability:=true` (기본값).
`/map` 이 필요하므로 `use_map_server:=true` 여야 한다.

3D voxel 확인 (실차):

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/install/setup.bash
ros2 launch traversability voxel_check.launch.py
```

RViz2 가 `rviz/voxel_check.rviz` 로 뜬다. **무엇을 보면 되나:**

1. `Cloud z-band [0.033, 0.31]` 초록 점이 **바닥에 깔리지 않아야** 한다.
   깔린다면 z_min 이 낮거나 실제 카메라 장착 높이가 URDF 와 다른 것이다.
2. `Octomap Voxels (z 색)` 이 물체 높이에 따라 다른 색으로 쌓여야 한다.
   **옆에서 봐야 한다** — 위에서 내려다보면 z 분포가 안 보인다.
3. 상자를 앞에 놓으면 그 높이만큼 voxel 기둥이 서고, 치우면 사라진다.
   사라지는 데 시간이 걸린다 — §3.3 의 log-odds 표대로 관통 miss 관측이
   약 9회 필요하다.

STVL 로 바꿔 비교하려면 (§3.3 은 한 방식씩 실행해 CPU·메모리를 재라고 정했다):

```
ros2 launch traversability voxel_check.launch.py use_octomap:=false use_stvl:=true
```
RViz 에서 `STVL Voxels` / `STVL Costmap` 체크박스를 켠다.

하드웨어 없이 배관만 보려면:

```
ros2 launch traversability voxel_check.launch.py use_robot:=false
python3 $(ros2 pkg prefix traversability)/share/traversability/tools/fake_depth_cloud.py
```

## 검증 (하드웨어 없이 실행함, 2026-09-09)

`tools/fake_depth_cloud.py` 가 만드는 장면 — 바닥(z=0), 낮은 물체(z 0.04~0.09,
**LiDAR 평면 0.218 m 아래**), 상자(z 0.05~0.25), 상부 돌출(z 0.9~1.1) — 으로
전 구간을 확인했다.

```
z-band 점군       816 점, z 0.040~0.250      바닥 안 섞임, 상부 돌출 제외
전체 높이 점군   7543 점, z 0.000~1.080      상부 돌출 3D 에 남음
OctoMap voxel    3806 개, z 0.025~1.075      11 층, 높이별 색 있음
OctoMap 2D 투영  82x51 @0.05, 점유 3650 셀
STVL voxel       3956 개, z 0.025~1.075      7 층
STVL costmap     240x240 @0.05, 치명 3600 셀, 미확인 53958 셀
2D 격자 반영     낮은 물체 12/20 셀, 상자 30/33 셀, 상부 돌출 0 셀
```

**낮은 물체가 2D 격자에 찍힌다는 것이 핵심이다.** 지면 위 0.04~0.09 m 라
LiDAR 마운트(0.2178 m)로는 절대 안 보인다. 설계문서 §3.3 의 "추가 검출"이
실제로 동작한다는 뜻이다.

단위 테스트 32개 (`test/test_depth_projection.py` 14 + 기존 18).

## 아직 없는 것

- 바닥 지지·단차·낙차 판정 (설계문서 §4)
- Planning 재연결 (위 "Planning 연결 상태" 의 base 대체 포함)
- D455 기반 3D 해제 → 2D 반영 (지금은 OctoMap/STVL 안에만 있다)
- STVL 만료 voxel 을 미확인으로 강등하는 어댑터 (위 6절 충돌)
- §3.3 비교 실험의 실측 (동일 rosbag 재생, CPU·메모리·지연)
- 폴리곤 장애물, 통과 금지 구역, 검증 공간, 장애물 기억 (설계문서 §7)
- DRA-MPPI 배선 (`dra_mppi/grid_costmap.py` 의 `DistanceField` 가 미사용 상태)
