#!/usr/bin/env bash
# 돌고 있는 토픽을 통째로 bag 에 담는다 — 카메라 이미지와 무거운 점군·octomap 만 빼고 전부.
# ---------------------------------------------------------------
# 기본 동작(--all): rosbag2 의 --all-topics 로 살아있는 토픽을 전부 받고, 아래만 뺀다.
#
#   빼는 것 (2026-09-10 D455 실측, 640x480x15)            MB/s   되살리는 인자
#   카메라 이미지 전부 (타입 필터)                           32.3   --heavy
#     color 13.83 · depth 9.22 · 정렬 depth 9.22 · 압축본들
#   /camera/depth/color/points (raw 점군)                    6.04   --raw
#   /occupied_cells_vis_array (복셀 마커)                   14.14   --heavy
#   /octomap_full                                            8.83   --heavy
#   /octomap_point_cloud_centers                           ~3.9    --heavy  (계산)
#
#   기본으로 담는 파생 점군 (traversable map·octomap 재계산용)
#   /traversability/cloud_filtered 2.27 · cloud_band 0.10 · /octomap_binary 0.63
#
# 이미지는 사용자 결정으로 기본 제외다(2026-09-10). 압축본만 담아도 1.95 MB/s 지만
# NUC 가 JPEG/PNG 인코딩 CPU 를 쓴다 — image_transport 는 구독자가 있을 때만 인코딩한다.
# 토픽 이름이 아니라 메시지 타입으로 이미지를 거르므로 카메라 네임스페이스가 바뀌어도 된다.
#
# --chain 을 주면 예전처럼 제어 체인 whitelist 만 담는다 (아래 목록).
# 어느 쪽이든 사전점검에서 체인 토픽의 발행 여부는 항상 O/X 로 찍어준다.
#
# 체인과 각 토픽이 답해주는 질문:
#
#   /goal_pose /initialpose        RViz 가 준 목표
#        ↓  rviz_theta_star_node
#   /global_path                   전역경로가 제대로 나왔나
#        ↓  dra_mppi_node  (10 Hz)
#   /dra_mppi/reference_trajectory 플래너가 MPC 에 준 기준열 (dt 0.2 × 20)
#   /dra_mppi/selected_path        MPPI 가 고른 롤아웃
#        ↓  mpc_tracking_node (25 Hz)
#   /mpc/reference_horizon         MPC 가 Ts=0.04 로 보간한 지평 — 기준열 해석이 맞나
#   /mpc/status                    [t, xe, ye, the, load, qp_iters, fallback, step_ms, ref_age]
#   /mpc/cmd_vel_raw               MPC 가 낸 명령
#        ↓  keyboard_command_gate (50 Hz)
#   /mpc/armed  /cmd_vel           게이트가 실제로 내보낸 명령 — 여기서 잘리는지
#
#   구조 B(mpc_structure:=wheel) 를 쓰면 위 세 줄이 아래로 바뀐다. 둘 다 목록에 넣어두므로
#   어느 쪽을 띄웠든 같은 명령으로 담긴다 (안 뜬 쪽은 발행자 없음으로 표시된다):
#   /wheel_mpc/reference_horizon /wheel_mpc/status /wheel_mpc/cmd_raw
#   /wheel_cmd                     게이트가 내보낸 바퀴 각속도 [FL,FR,RL,RR] rad/s
#   /measured_wheel_speeds         드라이버 encoder — 구조 B 의 속도 피드백 원천
#        ↓  roboclaw_mecanum_node (50 Hz)
#   /roboclaw/command_qpps         드라이버가 바퀴에 준 값 (L1 스케일링이 먹었는지)
#   /roboclaw/measured_qpps        바퀴가 실제로 돈 값
#   /joint_states  /odom           휠 오도메트리
#        ↓  ekf_filter_node (30 Hz)
#   /odometry/filtered             MPC·MPPI 가 공통으로 쓰는 상태 — 루프가 닫힌다
#   /imu/base_link  /tf  /tf_static
#
# 사용:
#   ./capture.sh                     전부 (이미지·raw 점군·octomap 무거운 것 제외), Ctrl+C 까지
#   ./capture.sh -d 60               60 초 뒤 자동 종료
#   ./capture.sh -o corner_test      이름 지정
#   ./capture.sh --chain             제어 체인 whitelist 만 (예전 기본 동작)
#   ./capture.sh --no-clouds         점군을 전부 뺀다 (파생 점군도)
#   ./capture.sh --raw               + raw 카메라 점군 6.04 MB/s (전처리 파라미터 재조정용)
#   ./capture.sh --heavy             위 표의 빼는 것까지 전부 (~65 MB/s)
#   ./capture.sh --split 60          끊김 없이 60 초마다 파일만 분할 (bag 은 하나)
#   ./capture.sh --split-mb 2048     2 GB 마다 파일 분할
#   ./capture.sh -z                  zstd_fast 로 압축하며 기록 (디스크 대신 CPU 를 쓴다)
#   ./capture.sh --check             녹화 안 하고 무엇이 담길지·빠질지만 본다
#   AMR_BAG_EXCLUDE='^/camera/' ./capture.sh    추가 제외 regex
#
# traversable map 디버깅:
#   ./capture.sh -o tmap --split 120
#   재생은 반드시 --clock 과 use_sim_time:=true 로 (아래 '재생' 참고).
#
# 실측 대역폭 (2026-09-09 실차, ros2 topic bw. voxel_check.launch.py 구성):
#   /occupied_cells_vis_array      14.14 MB/s  octomap 복셀 마커 — 가장 무겁다
#   /octomap_full                   8.83 MB/s  octree + log-odds
#   /camera/depth/color/points      6.06 MB/s  decimation 3 (24.9k pt/frame)
#   /traversability/cloud_filtered  2.27 MB/s  12.5k pt/frame
#   /octomap_binary                  627 KB/s  octree (free/occupied 만)
#   /traversability/cloud_band       101 KB/s  0.5k pt/frame
#   /scan                             30 KB/s
#   decimation 을 끄면 raw 점군은 307200 pt x 20 B x 15 Hz = 92 MB/s 가 된다.
#
# 두 가지가 함정이다:
#   1. /occupied_cells_vis_array 는 MarkerArray 라 --exclude-topic-types 에 안 걸린다.
#      순수 RViz 표시용이고 octree 에서 다시 만들 수 있으므로 기본 regex 로 뺀다.
#      같은 정보를 /octomap_binary 가 22 배 싸게 담는다.
#   2. 전처리 파라미터(self_filter_box, min/max_range, voxel_leaf)를 다시 만질 게
#      아니라면 raw 점군을 담을 이유가 없다 — cloud_filtered 와 2.7 배 차이고,
#      decimation 을 끄면 40 배가 된다.
#
# 보기:
#   ros2 bag info <bag>
#   ros2 topic echo /mpc/status --once           # 배열 순서는 위 주석 참고
#   ros2 bag play <bag> --topics /mpc/reference_horizon /global_path   # RViz 로 재생
# ---------------------------------------------------------------
set -uo pipefail

BAG_DIR="${AMR_BAG_DIR:-$HOME/2026amr_ws/bags}"
NAME=""; DUR=""; HEAVY=0; CHECK=0; COMPRESS=0; MODE=all
CLOUDS=1; RAW=0; IMAGES=0; SPLIT_SEC=""; SPLIT_MB=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--out)      NAME="$2"; shift 2 ;;
    -d|--dur)      DUR="$2";  shift 2 ;;
    -a|--all)      MODE=all;   shift ;;
    --chain)       MODE=chain; shift ;;
    --clouds)      CLOUDS=1;   shift ;;   # 이제 기본값. 예전 명령 호환용
    --no-clouds)   CLOUDS=0; RAW=0; shift ;;
    --raw)         CLOUDS=1; RAW=1; shift ;;
    --heavy)       HEAVY=1; CLOUDS=1; RAW=1; IMAGES=1; shift ;;
    --split)       SPLIT_SEC="$2"; shift 2 ;;
    --split-mb)    SPLIT_MB="$2";  shift 2 ;;
    -z|--compress) COMPRESS=1; shift ;;
    --check)       CHECK=1;    shift ;;
    -h|--help)     awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
    *) echo "모르는 인자: $1  (--help)"; exit 2 ;;
  esac
done

# ── 제어 체인. --chain 일 때 녹화 목록이고, --all 일 때도 사전점검 대상이다.
#    주석의 흐름과 같은 순서를 유지한다 ──
TOPICS=(
  /goal_pose /initialpose                                   # 입력
  /global_path                                              # Theta*
  /dra_mppi/reference_trajectory /dra_mppi/reference /dra_mppi/selected_path
  /mpc/reference_horizon /mpc/status /mpc/cmd_vel_raw        # MPC 구조 A (몸체 twist)
  /wheel_mpc/reference_horizon /wheel_mpc/status /wheel_mpc/cmd_raw   # MPC 구조 B (바퀴 각속도)
  /mpc/armed /cmd_vel /wheel_cmd /measured_wheel_speeds     # 게이트 → 로봇 (A 는 /cmd_vel, B 는 /wheel_cmd)
  /roboclaw/command_qpps /roboclaw/measured_qpps             # 드라이버
  /joint_states /odom                                       # 휠 오도메트리
  /odometry/filtered /imu/base_link                          # EKF (루프 닫힘)
  /tf /tf_static
)
HEAVY_TOPICS=( /dra_mppi/rollouts /scan /map /map_carto )

# ── 측위·traversability. --all 에서 발행 여부만 점검한다(녹화는 --all 이 이미 한다).
#    2026-09-09~10 에 디버깅한 경로들이다: 기준 맵 -> cartographer -> pose_to_odom
#    -> ekf_carto(map->odom), AMCL 자동 초기위치, traversable map, 3D voxel.
LOC_TOPICS=(
  /map /tracked_pose /tracked_pose/odom /odometry/map        # 측위 (map->odom 원천)
  /amcl_pose /particle_cloud /mpc/arm                         # AMCL 부트스트랩 · wiggle
  /scan /traversable_map /traversable_map_grid                # 2D traversable map
  /traversability/cloud_filtered /traversability/cloud_band   # D455 전처리
  /octomap_binary /projected_map                              # 3D voxel
  /rosout /parameter_events
)
[[ $MODE == chain && $HEAVY -eq 1 ]] && TOPICS+=( "${HEAVY_TOPICS[@]}" )

# --all 에서 기본으로 빼는 메시지 타입. 이름이 아니라 타입으로 거르므로
# 카메라 네임스페이스가 바뀌거나 노드가 늘어도 그대로 동작한다.
IMAGE_TYPES=(
  sensor_msgs/msg/Image
  sensor_msgs/msg/CompressedImage
  sensor_msgs/msg/CompressedDepth
  theora_image_transport/msg/Packet
)
CLOUD_TYPES=( sensor_msgs/msg/PointCloud2 )
EXCLUDE_TYPES=()
[[ $IMAGES -eq 0 ]] && EXCLUDE_TYPES+=( "${IMAGE_TYPES[@]}" )
[[ $CLOUDS -eq 0 ]] && EXCLUDE_TYPES+=( "${CLOUD_TYPES[@]}" )
# /parameter_events 는 담는다 — 주행 중 누가 파라미터를 바꿨는지(ros2 param set 포함)가
# 여기만 남는다. 기동 직후 선언 폭주 말고는 이벤트성이라 작다.
# /rosout 도 담는다 — 어느 노드가 무슨 경고를 뱉었는지가 사후 분석에 제일 자주 필요하다.
# 서비스 호출(--all-services)은 안 담는다. 리포의 어느 노드도 service introspection 을
# 켜지 않아 _service_event 토픽이 없다(2026-09-10 확인). FinishTrajectory/StartTrajectory
# 같은 호출은 set_initial_pose_node 가 /rosout 에 단계별로 찍으므로 거기서 본다.
# _vis_array 는 octomap_server 의 RViz 표시 전용 MarkerArray 다. 실측 14 MB/s 로
# 이 스택에서 가장 무거운데 MarkerArray 라 타입 필터에 안 걸린다. 같은 내용이
# /octomap_binary(627 KB/s) 에 있고 거기서 다시 그릴 수 있으므로 기본으로 뺀다.
RE_PARTS=()
# --heavy 가 아니면 octomap 의 무거운 표현 셋을 뺀다. 같은 octree 를 /octomap_binary 가
# 22 배 싸게 담고, 복셀은 cloud_filtered 로 octomap 을 다시 돌리면 그대로 나온다.
[[ $HEAVY -eq 0 ]] && RE_PARTS+=( "_vis_array$" "^/octomap_full$" "^/octomap_point_cloud_centers$" )
# 점군은 기본으로 담되(CLOUDS=1) raw 카메라 점군은 뺀다. decimation 3 에서도 6.04 MB/s 로
# cloud_filtered(2.27) 의 2.7 배고, decimation 을 끄면 92 MB/s 가 된다. cloud_filtered 는
# z 를 안 자른 상태라 z-band·octomap·traversability 파라미터는 전부 여기서 다시 맞출 수 있다.
# 전처리 자체(self_filter_box · min/max_range · voxel_leaf)를 다시 맞출 때만 --raw.
[[ $CLOUDS -eq 1 && $RAW -eq 0 ]] && RE_PARTS+=( "^/camera/.*points$" )
[[ -n "${AMR_BAG_EXCLUDE:-}" ]] && RE_PARTS+=( "${AMR_BAG_EXCLUDE}" )
# 빈 regex 는 모든 이름에 매칭되어 전부 빠진다. 그래서 비어 있으면 인자 자체를 안 넘긴다.
EXCLUDE_RE="$(IFS='|'; echo "${RE_PARTS[*]}")"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 가 소싱되지 않았습니다:  source /opt/ros/jazzy/setup.bash && source ~/2026amr_ws/install/setup.bash"
  exit 1
fi

# workspace overlay 가 걸렸는지 확인한다. ROS_DISTRO 만 보면 /opt/ros 만 소싱한 셸을 못 걸러낸다.
#
# ★ 2026-09-09 mpc_chain_20260909_151007 에서 /dra_mppi/reference_trajectory 가 통째로
#   안 담겼다. /mpc/status 의 ref_age 가 0.09~0.18 s 로 살아 있었으니 발행은 되고 있었다.
#   같은 bag 에서 /goal_pose(geometry_msgs)·/initialpose 는 녹화 도중 생겼는데도 잡혔다 —
#   즉 rosbag2 의 늦은 토픽 발견 자체는 동작했고, 표준 타입만 잡히고 custom 타입
#   (amr_interfaces/PlanarReferenceArray) 만 빠졌다. overlay 미소싱이 이 증상과 맞는다.
#   기준열이 없으면 MPC 가 무엇을 받았는지 사후에 확인할 방법이 없다 — 녹화 전에 막는다.
if ! python3 -c "from rosidl_runtime_py.utilities import get_message
get_message('amr_interfaces/msg/PlanarReferenceArray')" 2>/dev/null; then
  echo "amr_interfaces 타입을 못 읽습니다 — workspace overlay 가 안 걸린 셸입니다."
  echo "  source ~/2026amr_ws/install/setup.bash"
  echo "이대로 녹화하면 /dra_mppi/reference_trajectory 가 조용히 빠집니다 (2026-09-09 사례)."
  exit 1
fi

# ── 사전점검. `ros2 topic info` 를 토픽마다 부르면 CLI 기동만 토픽당 ~2 s 라
#    수십 개에 1 분이 넘고, 그동안 녹화가 안 돌아 정작 잡으려던 순간을 놓친다.
#    `ros2 topic list -v` 한 번으로 이름·타입·발행자수를 다 얻는다.
#    출력 형식:  " * /odom [nav_msgs/msg/Odometry] 1 publisher"
PUBLISHED="$(ros2 topic list -v 2>/dev/null | sed -n '/^Published topics:/,/^Subscribed topics:/p')"

echo "제어 체인 점검 (발행자 유무):"
NLIVE=0
for t in "${TOPICS[@]}"; do
  n=$(printf '%s\n' "$PUBLISHED" | awk -v t="$t" '$2==t {print $(NF-1); exit}')
  if [[ "${n:-0}" -gt 0 ]]; then
    NLIVE=$((NLIVE+1)); printf '  \033[32m O \033[0m %s\n' "$t"
  else
    printf '  \033[31m X \033[0m %s\n' "$t"
  fi
done
echo "발행 중 ${NLIVE} / ${#TOPICS[@]}"

if [[ $NLIVE -lt ${#TOPICS[@]} ]]; then
  echo "발행자 없는 토픽도 녹화 목록에는 넣습니다 (도중에 생기면 잡힙니다). 흔한 원인:"
  echo "  · /goal_pose /initialpose  → RViz 를 아직 안 띄웠거나 목표를 안 찍음 (녹화 중 찍으면 잡힘)"
  echo "  · /mpc/* 또는 /wheel_mpc/* → 그 구조를 안 띄운 것이다 (mpc_structure 확인). 한쪽은 항상 비어 있다"
  echo "  · /mpc/*                   → start_mpc:=false 이거나 mpc_publish_debug:=false"
  echo "  · /dra_mppi/*              → use_dra_mppi:=false 이거나 전역경로를 아직 못 받음"
  echo "  · /roboclaw/* /odom        → use_base:=false 이거나 드라이버 포트 문제"
  echo "  · /camera/depth/color/points → sensors.launch.py 의 depth_camera_pointcloud_enable"
  echo "                                기본값이 false 다. true 로 띄우지 않으면 D455 점군이"
  echo "                                아예 없고 /traversability/cloud_* 도 따라서 빈다."
fi

# ── 측위·traversability 발행 여부
if [[ $MODE == all ]]; then
  echo
  echo "측위·traversability 점검:"
  # 발행 여부만 보면 부족하다 — 이 목록의 토픽은 아래 "체인 밖" 목록에서 숨겨지므로,
  # 발행 중인데 필터로 빠지는 경우를 여기서 같이 알려야 한다(예: --no-clouds 의 cloud_filtered).
  for t in "${LOC_TOPICS[@]}"; do
    line=$(printf '%s\n' "$PUBLISHED" | awk -v t="$t" '$2==t {print; exit}')
    if [[ -z "$line" ]]; then
      printf '  \033[31m X \033[0m %s\n' "$t"
      continue
    fi
    type=$(awk '{gsub(/[][]/, "", $3); print $3}' <<<"$line")
    dropped=""
    [[ -n "$EXCLUDE_RE" && "$t" =~ $EXCLUDE_RE ]] && dropped="regex"
    for et in "${EXCLUDE_TYPES[@]}"; do [[ "$type" == "$et" ]] && dropped="타입 필터"; done
    if [[ -n "$dropped" ]]; then
      printf '  \033[33m O \033[0m %s  \033[33m(발행 중이지만 제외됨: %s)\033[0m\n' "$t" "$dropped"
    else
      printf '  \033[32m O \033[0m %s\n' "$t"
    fi
  done
  echo "  · /amcl_pose /particle_cloud     → use_amcl_bootstrap:=false 이거나 AMCL 이 아직 안 떴다"
  echo "  · /traversability/* /octomap_*   → use_voxel_3d:=false"
  echo "  · /tracked_pose /odometry/map    → cartographer localization 이 안 떴다"
fi

# ── --all 이면 체인 밖 토픽까지 무엇이 담기고 무엇이 빠지는지 보여준다.
if [[ $MODE == all ]]; then
  echo
  echo "체인 밖 토픽 (--all 로 함께 담김):"
  # 체인 토픽과 무거운 타입을 뺀 나머지를 나열한다. 이 목록이 곧 추가로 담기는 것.
  printf '%s\n' "$PUBLISHED" | awk -v chain="${TOPICS[*]} ${LOC_TOPICS[*]}" -v heavy="${EXCLUDE_TYPES[*]}" -v ex="$EXCLUDE_RE" '
    BEGIN { split(chain, c, " "); for (i in c) inchain[c[i]]=1
            split(heavy, h, " "); for (i in h) isheavy[h[i]]=1 }
    /^ \* \// {
      topic=$2; type=$3; gsub(/[\[\]]/, "", type)
      if (topic in inchain) next
      if (ex != "" && topic ~ ex) { printf "  \033[90m -  %s  [%s]  (제외: regex)\033[0m\n", topic, type; next }
      if (type in isheavy) {
        printf "  \033[33m -  %s  [%s]  (제외: 타입 필터 — --heavy)\033[0m\n", topic, type; nskip++; next }
      printf "  \033[32m +  \033[0m%s  [%s]\n", topic, type; nrec++
    }
    END { printf "추가 녹화 %d 개, 제외 %d 개\n", nrec, nskip }'
fi

echo
echo "디스크 여유: $(df -h "$(dirname "$BAG_DIR")" 2>/dev/null | awk 'NR==2 {print $4" / "$2}')"

if [[ $CHECK -eq 1 ]]; then exit 0; fi

# 죽은 토픽도 그대로 넘긴다 — rosbag2 는 없는 토픽을 에러 없이 받고, 녹화 도중
# 생기면 그때부터 잡는다 (확인함). /goal_pose 처럼 RViz 에서 클릭해야 생기는
# 토픽을 여기서 걸러내면 정작 목표를 못 담는다.
mkdir -p "$BAG_DIR"
OUT="$BAG_DIR/${NAME:-mpc_chain}_$(date +%Y%m%d_%H%M%S)"
# ros2 bag record 에는 '몇 초 뒤 정지' 플래그가 없다. -d 는 --max-bag-duration,
# 즉 파일 분할 주기라 시간제한으로 쓰면 영원히 녹화된다 (Jazzy 에서 확인함).
# 총 시간 제한은 timeout 으로 건다 — SIGINT 를 받으면 캐시를 비우고 정상 종료한다.
CMD=()
[[ -n "$DUR" ]] && CMD+=(timeout --signal=INT "$DUR")
CMD+=(ros2 bag record -o "$OUT")
# mcap 기본 저장소에서는 storage preset 이 rosbag2 압축보다 싸고 빠르다.
[[ $COMPRESS -eq 1 ]] && CMD+=(--storage-preset-profile zstd_fast)
# 파일 분할. 녹화를 끊었다 잇는 것과 다르다 — recorder 는 계속 돌므로 경계에
# 빈 구간이 없고, 결과는 bag 하나에 파일 여러 개다 (ros2 bag play 가 이어서 재생).
# 주의: rosbag2 의 -d 는 이 분할 주기다. 이 스크립트의 -d 는 총 녹화 시간(timeout).
[[ -n "$SPLIT_SEC" ]] && CMD+=(--max-bag-duration "$SPLIT_SEC")
[[ -n "$SPLIT_MB" ]]  && CMD+=(--max-bag-size $(( SPLIT_MB * 1024 * 1024 )))
if [[ $MODE == all ]]; then
  [[ -n "$EXCLUDE_RE" ]] && CMD+=(--exclude-regex "$EXCLUDE_RE")
  # nargs='+' 인 플래그라 뒤에 반드시 다른 플래그가 와야 목록이 닫힌다.
  [[ ${#EXCLUDE_TYPES[@]} -gt 0 ]] && CMD+=(--exclude-topic-types "${EXCLUDE_TYPES[@]}")
  CMD+=(--all-topics)
else
  # 토픽은 --topics 로 넘긴다 (위치 인자는 deprecated 경고가 뜬다).
  CMD+=(--topics "${TOPICS[@]}")
fi

echo
echo "녹화 → $OUT"
if [[ $MODE == all ]]; then
  what="기본(파생 점군 포함)"
  [[ $CLOUDS -eq 0 ]] && what="점군 없음"
  [[ $RAW    -eq 1 ]] && what="$what +raw카메라점군(6.04 MB/s)"
  [[ $HEAVY  -eq 1 ]] && what="전부 +이미지 +octomap 전체(~65 MB/s)"
  echo "     모드: --all ($what)"
  [[ -n "$SPLIT_SEC" ]] && echo "     ${SPLIT_SEC} 초마다 파일 분할 (bag 은 하나)"
  [[ -n "$SPLIT_MB"  ]] && echo "     ${SPLIT_MB} MB 마다 파일 분할 (bag 은 하나)"
else
  echo "     모드: --chain (${#TOPICS[@]} 토픽)"
fi
[[ -n "$DUR" ]] && echo "     ${DUR} 초 후 자동 종료" || echo "     Ctrl+C 로 종료"
echo
"${CMD[@]}"

echo
if [[ -d "$OUT" ]]; then
  ros2 bag info "$OUT"
  echo
  echo "다음:"
  # traversable map 계열이 담겼으면 재생 레시피를 먼저 띄운다.
  if ros2 bag info "$OUT" 2>/dev/null | grep -qE "/traversability/cloud|/traversable_map"; then
    echo "  ── traversable map 재계산 (파라미터를 바꿔 다시 돌린다) ──"
    echo "  ros2 bag play $OUT --clock"
    echo "  ros2 launch traversability traversability.launch.py use_sim_time:=true"
    echo "      --clock 과 use_sim_time:=true 는 짝이다. 한쪽만 켜면 memory_seconds(5.0),"
    echo "      free_memory_seconds, clear_hit_threshold 누적창이 재생 속도에 따라 늘었다 줄었다 한다."
    echo "      -r 0.5 로 느리게 재생하면 wall clock 에서는 창이 절반만 덮인다."
    echo "  ※ 재생 전에 실물 노드를 반드시 내릴 것 — 같은 토픽을 두 곳이 발행하면"
    echo "     격자가 섞이고 원인 추적이 불가능해진다 (ps -eo pid,args 로 확인)."
    echo
  fi
  echo "  ros2 bag play $OUT --loop"
  echo "      RViz 에 /global_path, /dra_mppi/reference_trajectory, /mpc/reference_horizon 을 띄우면"
  echo "      플래너가 준 것과 MPC 가 해석한 것이 겹쳐 보인다. 어긋나면 보간·프레임 문제."
  echo "  ros2 topic echo /mpc/status --once"
  echo "      [t, xe, ye, the, load, qp_iters, fallback, step_ms, ref_age]"
  echo "      fallback 이 1 이면 QP 가 못 풀어 clip 으로 떨어진 것, ref_age 가 크면 기준열이 늦은 것."
fi
