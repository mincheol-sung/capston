#!/usr/bin/env bash
# 저장된 PGM 을 /map 으로 발행한다. autonomy.launch.py:183-209 와 같은 구성을
# 떼어낸 것 — traversability_node 는 base_map_topic: /map 을 구독하므로 이게
# 없으면 "Waiting for the base map before publishing." 에서 멈춘다.
#
# map_server 는 라이프사이클 노드다. configure/activate 를 해주지 않으면
# 프로세스는 떠 있는데 /map 은 한 번도 안 나간다. 그래서 lifecycle_manager 를
# autostart 로 같이 띄운다.
#
#   ./map_server.sh                # test_imu_scan
#   ./map_server.sh test1
set -uo pipefail

MAP="${1:-test_imu_scan}"
MAPS_DIR="${AMR_MAPS_DIR:-$HOME/2026amr_ws/src/amr_basic_stack/planner/global_planner/maps}"
YAML="$MAPS_DIR/$MAP/$MAP.yaml"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 가 소싱되지 않았습니다:  source /opt/ros/jazzy/setup.bash && source ~/2026amr_ws/install/setup.bash"
  exit 1
fi
if [[ ! -f "$YAML" ]]; then
  echo "맵 yaml 이 없습니다: $YAML"
  echo "있는 맵:"; ls -1 "$MAPS_DIR" 2>/dev/null | sed 's/^/  /'
  exit 1
fi

echo "map_server → $YAML"
ros2 run nav2_map_server map_server --ros-args -r __node:=map_server \
  -p use_sim_time:=false -p yaml_filename:="$YAML" \
  -p topic_name:=map -p frame_id:=map &
MS=$!
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map \
  -p use_sim_time:=false -p autostart:=true -p node_names:="['map_server']" &
LM=$!
trap 'kill $MS $LM 2>/dev/null' TERM INT
wait
