#!/bin/bash
# 하드웨어 없이 전 구간을 검증한다.
#   D455 -> z-band -> OctoMap/STVL -> 2D 격자
#   LiDAR -> 관측 기반 해제 -> 장애물 추가/제거
#
#   bash $(ros2 pkg prefix traversability)/share/traversability/tools/verify_voxel_chain.sh
#
# 실차 없이 배관이 살아 있는지 확인하는 용도다. 실차 확인은
# voxel_check.launch.py 로 RViz 를 보면서 한다.
set +u
SP="$(cd "$(dirname "$0")" && pwd)"
LOG="$(mktemp -d)"
source /opt/ros/jazzy/setup.bash
source "$HOME/2026amr_ws/install/setup.bash"
CFG="$(ros2 pkg prefix traversability)/share/traversability/config"
PIDS=()
FAILED=0

run_case() {
  local name="$1"; shift
  echo "################ $name"
  "$@"
  local rc=$?
  [ $rc -ne 0 ] && FAILED=1
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
  PIDS=()
  sleep 2
}

start_common() {
  python3 "$SP/fake_depth_cloud.py" --rate 5 > "$LOG/fake.log" 2>&1 & PIDS+=($!)
  ros2 run traversability depth_preprocess_node --ros-args \
    --params-file "$CFG/depth_preprocess.yaml" > "$LOG/prep.log" 2>&1 & PIDS+=($!)
}

case_octomap() {
  start_common
  ros2 run octomap_server octomap_server_node --ros-args \
    --params-file "$CFG/octomap.yaml" -r cloud_in:=/traversability/cloud_filtered \
    > "$LOG/octo.log" 2>&1 & PIDS+=($!)
  sleep 12
  python3 "$SP/check_voxels.py"
}

case_stvl() {
  start_common
  ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args \
    --params-file "$CFG/stvl_costmap.yaml" > "$LOG/stvl.log" 2>&1 & PIDS+=($!)
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p autostart:=true -p "node_names:=['costmap']" -p bond_timeout:=0.0 \
    -r __node:=costmap_lifecycle_manager > "$LOG/lm.log" 2>&1 & PIDS+=($!)
  sleep 22
  python3 "$SP/check_stvl.py"
}

case_grid() {
  start_common
  ros2 run traversability traversability_node --ros-args \
    --params-file "$CFG/traversability.yaml" > "$LOG/trav.log" 2>&1 & PIDS+=($!)
  sleep 6
  python3 "$SP/check_depth_to_grid.py"
}

case_clearing() {
  # 기준 맵을 전부 점유로 주고, 광선 관통이 그것을 뚫는지 본다.
  python3 "$SP/fake_scan.py" --obstacle 2.0 --rate 10 > "$LOG/scan.log" 2>&1 & PIDS+=($!)
  ros2 run traversability traversability_node --ros-args \
    --params-file "$CFG/traversability.yaml" -p use_depth:=false \
    > "$LOG/trav_clear.log" 2>&1 & PIDS+=($!)
  sleep 6
  python3 "$SP/check_clearing.py"
}

case_add_remove() {
  # 장애물이 놓였다가 치워지는 동안 격자가 따라오는지 본다. 느리다(약 35 s).
  python3 "$SP/fake_scan.py" --obstacle 2.0 --rate 10 --remove-after 22 \
    > "$LOG/scan2.log" 2>&1 & PIDS+=($!)
  ros2 run traversability traversability_node --ros-args \
    --params-file "$CFG/traversability.yaml" -p use_depth:=false \
    > "$LOG/trav_ar.log" 2>&1 & PIDS+=($!)
  sleep 4
  python3 "$SP/check_add_remove.py"
}

run_case "OctoMap 3D voxel"        case_octomap
run_case "STVL 3D voxel"           case_stvl
run_case "D455 z-band -> 2D 격자"  case_grid
run_case "관측 기반 해제"          case_clearing
run_case "장애물 추가 -> 제거"     case_add_remove

echo
if [ $FAILED -eq 0 ]; then
  echo "ALL VOXEL CHECKS PASSED"
else
  echo "SOME CHECKS FAILED — 로그: $LOG"
fi
exit $FAILED
