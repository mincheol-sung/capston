# Traversable map 구현 기록 · 2026-09-08 ~ 09

설계문서 「Traversable map」(2026-09-08)의 구현 결과다. 문서의 절 번호(§)는
그 설계문서를 가리킨다. 실차 성능은 아직 측정하지 않았고, 아래 숫자는 전부
합성 입력과 리포 실제 값으로 낸 것이다.

---

## 0. 지금 돌아가는 것

```
nav2_map_server ──/map──────────────────┐
  (저장된 PGM 전체)                      │
                                         ▼
앞뒤 RPLIDAR ──/scan──────────► traversability_node ──/traversable_map──► rviz_theta_star_node
                                    2 Hz · map · 0/100/-1                  세 번째 장애물 레이어
                                         ▲
                                         │ /traversability/cloud_band
D455 ──/camera/depth/color/points──► depth_preprocess_node
                                         │ /traversability/cloud_filtered
                                         ├──────────► octomap_server 2.3.1
                                         └──────────► STVL 2.5.5 (nav2_costmap_2d 독립 노드)
```

| 구성요소 | 상태 |
| --- | --- |
| 2D 3-state 격자 → Planning | 구현·검증 완료 |
| Theta* 소비 | 구현·검증 완료 |
| D455 z-band 투영 | 구현·검증 완료 (합성 입력) |
| OctoMap 3D | 구현·검증 완료 (합성 입력) |
| STVL 3D | 구현·검증 완료 (합성 입력) |
| RViz2 z축 voxel 확인 | 준비 완료 (`voxel_check.launch.py`) |
| DRA-MPPI 배선 | **미구현** |
| 바닥 지지·단차·낙차 | **미구현** |
| §3.3 비교 실험 실측 | **미실시** |

---

## 1. 1단계 — Planning 전달 계약

### 1.1 확정한 결정

| # | 항목 | 결정 |
| --- | --- | --- |
| D0 | 1단계 정보원 | 기준 PGM + 2D LiDAR (D455 는 2단계) |
| D1 | wire 값 | `0` 통과 가능 / `100` 통과 불가 / `-1` 미확인 |
| D2 | 맵 범위 | 전역 고정 (`/map` 기하 승계) |
| D3 | frame | `map` |
| D4 | 갱신 | 전체 격자, 2 Hz |
| D5 | z_max | 0.31 m |
| D6 | 관측 기억 | 5 s |

### 1.2 값 규약이 0/1/255 가 아닌 이유

`nav_msgs/OccupancyGrid.data` 는 `int8[]` 이다. `255` 는 저장할 수 없고 바이트
`0xFF` 로 나가 `-1` 로 읽히는데, 그게 마침 ROS 의 unknown 규약이라 그대로 맞다.

문제는 `1` 이다. 소비자에게 `[0, 1, 255, 100]` 을 실제로 넣어 봤다:

```
보낸 바이트(uint8): [0, 1, 255, 100]
수신 int8        : [0, 1,  -1, 100]
clearance [m]    : [0.1, 0.05, 0.0, 0.0]      # 0.0 = 점유로 잡힘
                        ↑
                   1 은 free 로 읽혔다
```

`DistanceField` 가 `grid >= 65` 로 판정하기 때문이다 (nav2 의 `occupied_thresh
0.65` 와 같은 경계). `1` 은 "1% 점유" = free 다. 그래서 통과 불가는 `100`.
`1~99` 는 cost 를 넣을 다음 단계를 위해 비워 둔다.

부수 발견: 현재 맵 yaml 의 `free_thresh: 0.196` 은 표준 unknown 회색(gray=205)의
occupancy `50/255 = 0.196078` 과 **7.8e-5** 차이로 걸쳐 있다. 지금은 unknown 이
obstacle 로 분류되지만, 누가 `0.1961` 로 고치면 조용히 free 로 뒤집힌다.

### 1.3 배열 순서 — 인덱스 체계가 5개다

혼동의 원인은 "픽셀"이 서로 다른 다섯 가지를 가리키는 것이었다.

| # | 이름 | 원점·축 | 단위 |
| --- | --- | --- | --- |
| ① | image pixel (PGM/PNG) | 좌상단, y 아래쪽 | px |
| ② | grid cell (OccupancyGrid) | 좌하단, y 위쪽 | cell |
| ③ | map frame | origin = 셀(0,0) 좌하단 | m |
| ④ | depth pixel (D455) | 좌상단, perspective | px |
| ⑤ | `BoundingBox2D` | 출처에 따라 다름 | px 또는 m |

**①↔② 는 상하 반전이다.** OccupancyGrid 는 row 0 = 최소 y, PGM 이미지는
row 0 = 최대 y. Theta* 내부 배열은 이미지 순서라 경계에서 `np.flipud` 를 한 번
걸어야 한다. 대칭적인 실내 맵에서는 눈으로 안 걸리므로 **비대칭 단일 셀
테스트**로 잡는다.

④ 는 metric 이 아니다. 0.05 m 셀 하나를 덮는 depth 픽셀 수:

```
거리 0.5 m : 33.7 px 폭 → 셀당 약 1137 px
거리 1.0 m : 16.9 px 폭 → 셀당 약  284 px
거리 2.0 m :  8.4 px 폭 → 셀당 약   71 px
거리 4.0 m :  4.2 px 폭 → 셀당 약   18 px
거리 6.0 m :  2.8 px 폭 → 셀당 약    8 px
```

거리에 따라 140배 차이난다. "픽셀 N개 이상이면 점유" 같은 규칙을 거리 무관하게
쓰면 안 되고, §4.1 대로 voxel 점유 판정을 거친 뒤 투영해야 한다.

⑤ 는 이미 리포에 심어진 단위 지뢰다. 같은 `DetectedObject.bbox` 필드가
`source=CAMERA` 면 픽셀, `source=LIDAR` 면 미터다.

### 1.4 미확인 처리는 소비자마다 다르다

§4 가 두 정책을 요구한다.

| 소비자 | 미확인 | 근거 |
| --- | --- | --- |
| Theta* (global) | 통과 **허용** | "글로벌 경로는 기준 지도상 미리 생성할 수 있지만" |
| DRA-MPPI (local) | 통과 **불가** | "실제 주행은 확인된 범위와 정지 가능한 거리 안에서" |

### 1.5 병합 규칙

```
현재 관측이 통과 불가        → 100
현재 관측이 통과 가능        → 0
관측 없음:
    관측 기억이 유효시간 내   → 그 값 유지   (memory_seconds, 5 s)
    기준 맵이 점유            → 100
    기준 맵이 free            → 0            (trust_base_map_free 일 때만)
    그 외                     → -1
```

만료는 **셀별** 최종 관측 시각으로 판정한다. 격자 전체에 타임스탬프를 하나만
두면 매 주기 기억을 다시 실어 나르면서 시각이 갱신되어 **만료가 영원히 오지
않는다**. 테스트로 박제했다.

### 1.6 설계문서와 어긋난 지점 — `trust_base_map_free`

§7.1 은 "미확인을 빈 공간으로 변환하지 않는다"를 규칙으로 정했고, 그에 따라
"기준 맵이 free → 0" 분기를 죽이기로 했었다. 그런데 1단계의 정보원은 그
2D LiDAR 평면 하나뿐이라, 죽이면 격자에 `0` 이 한 칸도 없어 local planner 가
아예 못 움직인다.

§7.1 의 규칙은 **관측한 적 없는** 공간에 대한 것이다. 기준 맵의 free 셀은
0.218 m 높이 LiDAR 평면에서 관측된 셀이고, 1단계의 traversability 정의가 바로
그 평면이므로 위반이 아니다. **정의가 넓어지는 2단계에서 위반이 된다.**

→ `trust_base_map_free` 파라미터로 분리. 1단계 `true`, **D455 z-band 투영이
운용에 들어가는 순간 `false` 로 뒤집어야 한다.**

---

## 2. 2단계 — D455 z-band 투영

### 2.1 전처리 (§3.2-3)

```
raw (camera_depth_optical_frame)
  → NaN/Inf 제거
  → TF → base_footprint            ★ optical 의 z 는 전방 깊이다. 변환 필수
  → 자기관측 상자 제거              [-0.20,-0.15,-0.05] ~ [0.20,0.15,0.28]
  → 수평거리 0.30 ~ 6.0 m
  → 0.03 m voxel 다운샘플 (결정적)
  ├→ /traversability/cloud_filtered  전체 높이, odom      → OctoMap · STVL
  └→ /traversability/cloud_band      z-band, base_footprint → 2D 격자
```

octomap 과 STVL 이 **같은 점군**을 받는다. §3.3 의 비교 실험 조건
"공통 D455 전처리를 유지하고 3D 갱신 모듈만 교체한다"를 이 노드 하나로 보장한다.

3D 누적은 §3.2 대로 연속적인 `odom` 로컬 윈도우에서 한다. `map` 은 위치추정
보정이 과거 자세를 소급 변경해 누적이 어긋난다.

### 2.2 z-band 값 — 설계문서 후보값을 그대로 쓰면 안 된다

URDF 실측:

```
base_footprint → base_link      z = 0.04       (amr_basic.urdf:42)
base_link → camera_link         z = 0.092772   → 카메라 높이 H = 0.132772 m
base_link → laser_*_joint       z = 0.177797   → LiDAR 마운트 0.2178 m
base_link 충돌박스              0.28584 × 0.18841 × 0.16701, 지면 위 0.0219~0.1889
```

§4.1 의 초기 후보 `z_min = -0.15 m`(카메라 기준)를 이 로봇에 넣으면:

```
지면 기준 하한 = H + z_min = 0.1328 + (-0.15) = -0.0172 m
```

**지면보다 1.7 cm 아래.** 바닥면 전체가 투영 밴드에 들어와 모든 바닥 점이
장애물로 찍힌다. 문서의 워크드 예시는 `H = 0.50 m` 를 가정해 "바닥 위 0.35 m
아래 물체가 빠질 수 있다"를 걱정했는데, 실제 H 에서는 부호가 뒤집힌다.

**확정값**: 카메라 기준 `-0.10 m` = **지면 기준 `z_min = 0.033 m`**,
`z_max = 0.31 m` (로봇 최고점 LiDAR 상단 약 0.258 m + 여유 0.05 m).

이 값은 세 파일에 있고 반드시 같아야 한다:
`depth_preprocess.yaml` · `traversability.yaml` · `octomap.yaml`.

RPLIDAR S2 본체 높이를 실측하면 `z_max` 를 확정할 수 있다.

### 2.3 근거리 바닥 사각지대

`camera_joint` 의 rpy 가 `0 0 0` 이라 optical axis 는 수평이다. VFOV 58°:

```
바닥이 시야에 들어오는 거리 = H / tan(29°) = 0.1328 / 0.5543 = 0.240 m
D455 depth Z-min(약 0.4 m) 고려 → 실제 바닥 관측 시작 0.40 m
```

**로봇 앞 0.40 m 안쪽 바닥은 관측 불가.** §9 의 "근거리 바닥 사각지대",
§3.3 측정지표의 "사각지대 유지율(미달 시 탈락)"과 직결된다.

### 2.4 대역폭

640×480×15 점군을 그대로 내보내면 약 147 MB/s 다. CycloneDDS 가 못 버틴다
(RGB 2.7 MB 프레임 때 이미 `rmem_max` 를 올려야 했다). RealSense 드라이버의
`decimation_filter` 를 켜 DDS 를 타기 전에 줄인다. magnitude 3 이면
640×480 → 약 213×160.

기본값은 **꺼짐**이다. 켜면 `align_depth` 결과 해상도도 줄어들어
`sensor_fusion` 의 `roi_median_3d` 가 쓰는 정렬 깊이가 바뀐다. 점군을 쓰는
launch 에서만 켠다.

---

## 3. OctoMap / STVL 도입

### 3.1 소스 포크가 아니라 공식 바이너리

packages.ros.org 의 .deb 를 그대로 쓴다. 버전이 §3.3 이 ROS 2 Jazzy 용으로
지목한 것과 정확히 일치한다.

| 패키지 | 버전 | 역할 |
| --- | --- | --- |
| `octomap_server` | **2.3.1** | 독립 노드. `/cloud_in` 구독 |
| `spatio_temporal_voxel_layer` | **2.5.5** | nav2_costmap_2d 독립 노드 안의 Layer 플러그인 |
| `octomap_rviz_plugins` | 2.1.0 | RViz 표시 |
| `pcl_ros` 2.6.4 · `octomap_msgs` 2.0.1 · `octomap_ros` 0.4.4 · `openvdb_vendor` 2.5.5 · `libopenvdb` 10.0.1 | — | 의존성 |

시스템에 이미 있던 것: `liboctomap.so.1.9` + 헤더, PCL 1.14 + 헤더, TBB, blosc.
없던 것: **OpenVDB** (STVL 필수).

### 3.2 sudo 없이 설치한 구조 — **2026-09-15 에 apt 정식 설치로 대체됨**

> 아래는 2026-09-09 ~ 09-15 의 임시 구조 기록이다. 지금은 해당 없다.
> `sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins`
> `ros-jazzy-spatio-temporal-voxel-layer` 로 설치했고, `vendor/`(잠시
> `perception/voxel/`) 디렉터리와 `setup_vendor.bash` 는 삭제했다.
> 소싱은 `/opt/ros/jazzy` + `install` 두 줄이면 된다.
> 설치된 버전은 당시 .deb 와 같다 (octomap_server 2.3.1, STVL 2.5.5,
> octomap_rviz_plugins 2.1.0, pcl_ros 만 2.6.4 -> 2.6.5).

당시 이 NUC 은 `sudo` 에 비밀번호가 필요해 `apt install` 이 안 됐다. 그래서
`apt-get download` + `dpkg -x` 로 `~/2026amr_ws/vendor/` 에 풀고 환경변수로 얹었다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/vendor/setup_vendor.bash      # ← AMENT_PREFIX_PATH, LD_LIBRARY_PATH
source ~/2026amr_ws/install/setup.bash
```

- `vendor/COLCON_IGNORE` 가 없으면 colcon 이 그 안의 package.xml 을 워크스페이스
  소스로 오인해 빌드가 깨진다.
- `.gitignore` 에 `vendor/` 추가.
- `.deb` 를 푼 것이라 dpkg 데이터베이스에 등록되지 않는다. apt 가 업그레이드·제거를
  관리하지 못하고, 의존성(`pcl_ros`, `octomap_msgs`, `octomap_ros`,
  `openvdb_vendor`, `liblog4cplus`)을 손으로 받아야 했다.
- `.so` 에 RPATH/RUNPATH 가 없어 `LD_LIBRARY_PATH` 가 유일한 단서였다.

### 3.3 STVL 을 nav2 없이 돌리기

STVL 은 독립 노드가 아니라 `nav2_costmap_2d` 의 Layer 플러그인이다. nav2
네비게이션 스택 전체가 아니라 **`nav2_costmap_2d` 독립 실행 파일 하나**만
띄우고 그 안에 STVL 을 얹는다. 경로 계획기는 관여하지 않는다. §3.3 의
"자체 프레임워크용 입출력 어댑터와 의존성 분리 범위"가 이것이다.

lifecycle 노드이므로 `nav2_lifecycle_manager` 로 configure/activate 해야 한다.

### 3.4 §6 원칙과 STVL 의 충돌 (미해결)

§6 은 "시간 경과만으로 빈 공간 확정 금지"를 규칙으로 정했는데, STVL 의 기본
동작인 시간 감쇠는 정확히 시간 경과로 voxel 을 제거한다.

`track_unknown_space: true` 로 **관측된 적 없는** 셀은 `NO_INFORMATION(255)` 로
남는다. 그러나 **감쇠로 사라진** voxel 은 `FREE_SPACE(0)` 가 되지 255 가 되지
않는다 — `costmap_2d` 자료구조 자체에 "만료됨" 상태가 없기 때문이다.

→ STVL 을 채택하려면 만료 voxel 을 미확인으로 강등하는 어댑터가 추가로 필요하다.
→ `decay_model: -1` 로 감쇠를 끄면 OctoMap 의 "증거 없으면 해제 안 함" 성질과
같은 조건이 되어 §3.3 의 사각지대 유지율을 분리 측정할 수 있다.

---

## 4. 스택에 있던 제약과 처리

| # | 제약 | 처리 |
| --- | --- | --- |
| C1 | 미확인을 표현할 자리가 planner 양쪽에 없었다 | 소비자별 정책 파라미터화. global 통과 허용 / local 통과 불가 |
| C2 | Theta* 가 맵을 디스크에서 1회 로드, 토픽 입력 없음 | `/traversable_map` 구독 신설, `_combined_costmap` 에 세 번째 레이어 |
| C3 | Theta* 에 cell cost 항 없음 (Euclidean 만) | 1단계는 binary 로 진행. cost 는 다음 단계 |
| C4 | DRA-MPPI 에 static 입력 없음 (`DistanceField` 미배선) | **미해결.** local planner 는 지금도 벽을 모른다 |
| C5 | 로봇 형상이 등방(원/정사각) | 1단계는 로봇팔·적재물 없으므로 허용 |
| C6 | 폴리곤·높이 범위 담을 msg 없음 | `TraversableMap.msg` 에 z-band 계약 필드 신설 |
| C7 | 3D 파이프라인 전무 | OctoMap · STVL 도입 완료 |
| C8 | 픽셀 단위 혼동 | 인덱스 체계 5개 정리, 경계 변환 1회 규칙, flip 테스트 |
| C9 | 좌표계 | 문제 없음. TF 배선 이미 존재 |

---

## 5. 실측으로 발견한 함정

1. **`occupancy_min_z`/`occupancy_max_z` 는 2D 투영만 거르지 않는다.**
   octomap_server 는 이 구간을 **발행되는 voxel 마커에도** 적용한다.
   0.033~0.31 로 두었더니 입력에 1.08 m 상부 돌출이 있는데도
   `/occupied_cells_vis_array` 의 voxel z 최고가 0.275 m 였다. RViz 에서 상부
   구조가 통째로 사라진다. → `-0.30 ~ 2.00` 으로 넓히고, 2D 밴드 투영은
   `traversability_node` 가 따로 한다.

2. **STVL 토픽은 `/costmap/...` 이 아니다.** `nav2_costmap_2d` 독립 노드는
   이름이 `costmap`, 네임스페이스가 `/` 라서 3D voxel 이 `/voxel_grid`,
   2D 가 `/costmap` 으로 나온다.

3. **`robot_size` 는 폭, `inflate_obstacle_mask` 는 반지름.** 혼동하면 두 배로
   팽창해 로봇 출발점이 자기 안전여유 안에 갇히고 Theta* 가
   `start is inside an obstacle or inflated safety margin` 으로 모든 목표를
   거부한다. 게다가 `Costmap` 은 짝수 변 정사각 커널을 쓰는데 scipy 가 이를
   비대칭으로 놓아(음의 방향 3칸, 양의 방향 2칸) **원형 커널로는 절대 일치하지
   않는다.** 같은 커널을 써야 한다.

4. **`Costmap.is_collision` 은 `>= 1` 이 아니라 `== 1` 이다.** `robot_size <= 1`
   경로에서는 배열이 그대로 통과하므로, wire 값 100 을 담은 레이어를 넣으면
   장애물이 통째로 사라진다. bool 로 접어서 넘겨야 한다.

5. **rclpy 의 uint8 필드는 numpy 스칼라를 거부한다.** Python `int` 여야 한다.

6. **Cartographer localization 의 `/map_carto` 를 기준 맵으로 쓰면 안 된다.**
   `pure_localization_trimmer` 때문에 새로 만든 서브맵 3개뿐이라 그 밖은 전부
   미확인이 된다. `nav2_map_server` 의 `/map` 을 써야 한다.

---

## 6. 검증

### 6.1 성능 실측

Theta* (순수 Python, expansion 마다 line-of-sight):

```
test_imu_scan  224×286 =  64,064 cells : median 285 ms, max 1483 ms
test1          348×284 =  98,832 cells : median 151 ms, max 1103 ms
                                         corner-to-corner 최악 2701 ms
```

§7.1 의 예시 크기 500×500 = 250,000 cells 는 현재 최대 맵의 2.53배다.

`DistanceField` 재생성(`distance_transform_edt`):

```
200×200  =  40,000 cells :  1.6 ms
500×500  = 250,000 cells : 11.5 ms
1000×1000= 1,000,000 cells: 44.0 ms
```

local 쪽(5 Hz)은 여유, global 쪽은 재계획 주기 설계가 별건으로 필요하다.

### 6.2 traversable map 이 Theta* 경로를 실제로 바꾸는가

기준 PGM 은 건드리지 않고 `/traversable_map` 에만 반경 0.6 m 원판을 세웠다.

```
경로 A: 137 poses,  그 원판을 12칸 통과
경로 B: 136 poses,  그 원판을  0칸 통과      경로 바뀜: True
```

원인이 `/traversable_map` 뿐임이 확정된다.

### 6.3 3D voxel 체인 (합성 D455 입력)

`tools/fake_depth_cloud.py` 로 바닥(z=0) · 낮은물체(z 0.04~0.09) ·
상자(z 0.05~0.25) · 상부돌출(z 0.9~1.1) 장면을 만들어 전 구간 확인.

```
z-band 점군       816 점, z 0.040~0.250   바닥 안 섞임, 상부돌출 제외
전체 높이 점군   7543 점, z 0.000~1.080   상부돌출 3D 에 남음
OctoMap voxel    3806 개, z 0.025~1.075   11층, 높이별 색 있음
OctoMap 2D 투영  82×51 @0.05, 점유 3650 셀
STVL voxel       3956 개, z 0.025~1.075   7층
STVL costmap     240×240 @0.05, 치명 3600 셀, 미확인 53958 셀
2D 격자 반영     낮은물체 12/20 셀, 상자 30/33 셀, 상부돌출 0 셀
```

**낮은 물체가 2D 격자에 찍히는 것이 핵심이다.** 지면 위 0.04~0.09 m 라
LiDAR 마운트(0.2178 m)로는 절대 안 보인다. §3.3 의 "추가 검출"이 실제로
동작한다는 뜻이다.

### 6.4 테스트

단위 테스트 traversability 32 · global_planner 15 · DRA-MPPI 37(회귀) 통과.
신규 파일 flake8 0건.

---

## 7. 실행

2D 만 (기존 autonomy 스택):

```bash
ros2 launch stack_master autonomy.launch.py map:=<맵이름>
# use_traversability 기본 true, use_map_server 필요
```

3D voxel 확인:

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/install/setup.bash
ros2 launch traversability voxel_check.launch.py
```

RViz2 가 자동으로 뜬다. **옆에서 봐야** z 분포가 보인다.

1. `Cloud z-band [0.033, 0.31]` 초록 점이 바닥에 깔리지 **않아야** 한다.
   깔린다면 실제 카메라 장착 높이가 URDF 와 다른 것이다.
2. `Octomap Voxels (z 색)` 이 물체 높이에 따라 다른 색으로 쌓여야 한다.
3. 상자를 앞에 놓으면 그 높이만큼 voxel 기둥이 서고, 치우면 사라진다.
   사라지는 데 시간이 걸린다 — §3.3 의 log-odds 표대로 관통 miss 관측이
   약 9회 필요하다.

STVL 로 비교 (§3.3 은 한 방식씩 실행해 CPU·메모리를 재라고 정했다):

```bash
ros2 launch traversability voxel_check.launch.py use_octomap:=false use_stvl:=true
```

하드웨어 없이 배관 검증:

```bash
bash $(ros2 pkg prefix traversability)/share/traversability/tools/verify_voxel_chain.sh
```

---

## 8. 아직 없는 것

- **DRA-MPPI 배선** — `dra_mppi/grid_costmap.py` 의 `DistanceField` 는 완성돼
  있는데 어디서도 import 되지 않는다. local planner 는 지금도 static 장애물을
  전혀 모르고 벽을 통과하는 rollout 을 만든다. 가장 큰 구멍이다.
- **`trust_base_map_free` 뒤집기** — D455 투영이 운용에 들어가면 `false` 로.
- **바닥 지지·단차·낙차 판정** (§4)
- **STVL 만료 voxel 강등 어댑터** (§6 충돌)
- **§3.3 비교 실험 실측** — 동일 rosbag 재생, CPU·메모리·지연, 측정지표 9종
- **폴리곤 장애물 / 통과 금지 구역 / 검증 공간 / 장애물 기억 / 실행 상태**
  (§7 의 나머지 5개 출력)
- **RPLIDAR S2 본체 높이 실측** → `z_max` 확정
- **Theta* 재계획 주기 설계** (§7.1 의 500×500 로 키울 경우)

---

## 9. 파일

```
perception/amr_interfaces/msg/TraversableMap.msg      Planning 계약
perception/traversability/
  traversability/encoding.py                          값 규약·병합 규칙
  traversability/scan_projection.py                   LaserScan → 셀
  traversability/depth_projection.py                  D455 z-band → 셀
  traversability/traversability_node.py               2D 3-state 격자 생산
  traversability/depth_preprocess_node.py             D455 공통 전처리
  config/{traversability,depth_preprocess,octomap,stvl_costmap}.yaml
  launch/{traversability,voxel_3d,voxel_check}.launch.py
  rviz/voxel_check.rviz
  tools/{fake_depth_cloud,check_*}.py, verify_voxel_chain.sh
  README.md
planner/global_planner/global_planner/common/traversable_map.py   Theta* 어댑터
planner/global_planner/global_planner/rviz_theta_star_node.py     레이어 배선
sensors/amr_bringup/launch/sensors.launch.py          decimation 인자 (기본 off)
vendor/  (2026-09-15 삭제)                             공식 .deb (gitignore 였음)
```

Planning 경로 생성 핵심 코드(`theta_star/`, DRA-MPPI 컨트롤러)는 변경하지 않았다.
