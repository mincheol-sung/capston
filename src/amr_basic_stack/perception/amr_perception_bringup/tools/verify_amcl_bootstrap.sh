#!/bin/bash
# 하드웨어 없이 AMCL 전역 측위 부트스트랩 전 구간을 검증한다.
#
#   bash $(ros2 pkg prefix amr_perception_bringup)/share/amr_perception_bringup/tools/verify_amcl_bootstrap.sh
#
# fake_robot.py 가 지도 위를 실제로 주행하며 스캔을 만들고, AMCL 이 전역
# 측위로 그 위치를 찾아내는지 본다. /ground_truth 와 비교해 오차를 잰다.
set +u
SP="$(cd "$(dirname "$0")" && pwd)"
LOG="$(mktemp -d)"
source /opt/ros/jazzy/setup.bash
source "$HOME/2026amr_ws/install/setup.bash"
MAPS="$(ros2 pkg prefix global_planner)/share/global_planner/maps"
CFG="$(ros2 pkg prefix amr_perception_bringup)/share/amr_perception_bringup/config/amcl.yaml"
MAP_NAME="${MAP_NAME:-test_imu_scan}"
PIDS=()

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$MAPS/$MAP_NAME/$MAP_NAME.yaml" \
  -p topic_name:=map -p frame_id:=map > "$LOG/map.log" 2>&1 & PIDS+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p "node_names:=['map_server']" -p bond_timeout:=0.0 \
  -r __node:=lm_map > "$LOG/lm.log" 2>&1 & PIDS+=($!)
sleep 7

python3 "$SP/fake_robot.py" --x "${GT_X:--1.0}" --y "${GT_Y:--1.0}" \
  --yaw "${GT_YAW:-30}" --speed 0.4 > "$LOG/robot.log" 2>&1 & PIDS+=($!)
sleep 3

ros2 run nav2_amcl amcl --ros-args --params-file "$CFG" > "$LOG/amcl.log" 2>&1 & PIDS+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p "node_names:=['amcl']" -p bond_timeout:=0.0 \
  -r __node:=amcl_lifecycle_manager > "$LOG/amcl_lm.log" 2>&1 & PIDS+=($!)
sleep 8

ros2 run amr_perception_bringup amcl_bootstrap_node --ros-args \
  --params-file "$CFG" > "$LOG/boot.log" 2>&1 & PIDS+=($!)

echo "로봇이 주행 중. 수렴을 기다린다 (최대 60 s)..."
timeout 80 python3 "$SP/check_amcl.py"
STATUS=$?

echo
echo "=== 수렴 과정 ==="
grep -a "std x=\|검증:\|수렴\|전역 측위" "$LOG/boot.log" | tail -10
for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
sleep 1
[ $STATUS -eq 0 ] && echo && echo "AMCL BOOTSTRAP PASSED" || echo "FAILED — 로그: $LOG"
exit $STATUS
