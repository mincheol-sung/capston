#!/bin/bash
# bag 재생으로 Cartographer 측위만 다시 돌린다 — 로봇 없이 측위 설정을 바꿔 비교 (2026-11-15).
#
# 왜: 실차 토크 시험에서 빠르게 움직이면 SLAM 이 수십 cm~1 m 틀렸다. 같은 bag 을 설정만 바꿔 다시 돌려
#     원인(ceres_scan_matcher.translation_weight)을 찾았다 — analysis/torque_mpc_real/2026-11-15_basic/RESULTS_KO.md.
#
# 사용 (amr 먼저, 실차 노드와 섞이지 않게 도메인을 따로 — 기본 89):
#   tools/slam_replay/replay.sh --bag <bag 폴더> --label <이름> --init "x y qz qw" [--lua v_tw1.lua] [--odom] [--play 초]
#     --init  : 재생 시작 때 로봇의 /tracked_pose (추적점, 지도 좌표). 측위가 맞던 때의 값을 bag 에서 읽어 준다
#     --lua   : localization_torque_2d.lua(기본) 또는 variants/ 의 파일 이름. variants 의 lua 는 설치된 설정을 include 한다
#     --odom  : 바퀴 오돔을 예측에 쓰는 판 (localization_torque_odom_2d.lua 와 함께, wheel_odometry_node 를 같이 띄움)
#     --play  : 재생 시간 [s] (기본 = bag 길이). bag 은 처음부터 튼다 (/tf_static 이 맨 앞에 있어서)
#   결과: <out>/<label>/pose.csv (재생 중 /tracked_pose) 와 노드 로그. out 기본 ~/slam_replay_out
#   비교: tools/slam_replay/compare.py
#
# bag 에 있어야 하는 토픽: /scan_front /scan_rear /imu/base_link /tf_static (/wheel_states — --odom 일 때)
# 재생 때 다시 만드는 것: /scan_front_carto·/scan_rear_carto (scan_sector_filter), /camera/imu (imu_relay.py)
# (set -u 는 ROS setup.bash 와 충돌해서 쓰지 않는다)
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../../../.." && pwd)
BAG=""; LABEL=""; INIT=""; LUA=localization_torque_2d.lua; ODOM=0; PLAY=""; OUT=$HOME/slam_replay_out; DOM=89
while (( $# )); do
  case "$1" in
    --bag) BAG=$2; shift 2;; --label) LABEL=$2; shift 2;; --init) INIT=$2; shift 2;; --lua) LUA=$2; shift 2;;
    --odom) ODOM=1; shift;; --play) PLAY=$2; shift 2;; --out) OUT=$2; shift 2;; --domain) DOM=$2; shift 2;;
    *) echo "모르는 인자 $1"; exit 1;;
  esac
done
[[ -n "$BAG" && -n "$LABEL" && -n "$INIT" ]] || { echo "필요: --bag --label --init"; exit 1; }
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$DOM; unset ROS_STATIC_PEERS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SHARE=$(ros2 pkg prefix amr_perception_bringup)/share/amr_perception_bringup/config
PB=$WS/src/amr_basic_stack/planner/global_planner/maps/smoc_lab/smoc_lab.pbstream
YAML=$(ros2 pkg prefix amr_laser_tools)/share/amr_laser_tools/config/scan_merge.yaml
L=$OUT/$LABEL; CFG=$L/cfg; mkdir -p "$CFG"; cp "$SHARE"/*.lua "$CFG"/; cp "$HERE"/variants/*.lua "$CFG"/
[[ -f "$CFG/$LUA" ]] || { echo "설정 $LUA 없음"; exit 1; }
if [[ -z "$PLAY" ]]; then PLAY=$(ros2 bag info "$BAG" 2>/dev/null | awk '/Duration/{print int($2)+1}'); fi
echo "재생: $BAG → $L  (설정 $LUA, $PLAY s, 도메인 $DOM)"
P=()
ros2 run amr_laser_tools scan_sector_filter --ros-args -r __node:=front_cartographer_scan_filter --params-file "$YAML" \
  -p use_sim_time:=true -r scan_in:=/scan_front -r scan_out:=/scan_front_carto > "$L/filter_front.log" 2>&1 & P+=($!)
ros2 run amr_laser_tools scan_sector_filter --ros-args -r __node:=rear_cartographer_scan_filter --params-file "$YAML" \
  -p use_sim_time:=true -r scan_in:=/scan_rear -r scan_out:=/scan_rear_carto > "$L/filter_rear.log" 2>&1 & P+=($!)
/usr/bin/python3 "$HERE/imu_relay.py" --ros-args -p use_sim_time:=true > "$L/imu_relay.log" 2>&1 & P+=($!)
if (( ODOM )); then ros2 run amr_mpc wheel_odometry_node --ros-args -p use_sim_time:=true > "$L/wheel_odom.log" 2>&1 & P+=($!); fi
ros2 run cartographer_ros cartographer_node -configuration_directory "$CFG" -configuration_basename "$LUA" \
  -load_state_filename "$PB" -load_frozen_state true --ros-args -p use_sim_time:=true \
  -r scan_1:=/scan_front_carto -r scan_2:=/scan_rear_carto -r imu:=/camera/imu -r odom:=/wheel_odom > "$L/carto.log" 2>&1 & P+=($!)
/usr/bin/python3 "$HERE/pose_rec.py" "$L/pose.csv" --ros-args -p use_sim_time:=true > "$L/pose_rec.log" 2>&1 & P+=($!)
TOPICS="/scan_front /scan_rear /imu/base_link /tf_static"; (( ODOM )) && TOPICS="$TOPICS /wheel_states"
ros2 bag play "$BAG" --clock 100 --start-paused --topics $TOPICS > "$L/play.log" 2>&1 & BP=$!
sleep 12                                   # 지도(pbstream) 불러오기
/usr/bin/python3 "$HERE/set_init.py" "$CFG" "$LUA" $INIT | tee "$L/init.log"
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume > /dev/null 2>&1
sleep "$PLAY"
kill -INT $BP 2>/dev/null; sleep 1
for p in "${P[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 2
for p in "${P[@]}" $BP; do kill -KILL "$p" 2>/dev/null; done
pkill -KILL -f "configuration_directory $CFG" 2>/dev/null
grep -m1 "Check failed" "$L/carto.log" && echo "★ Cartographer 가 죽었다 (위 줄)"
echo "끝: $L/pose.csv $(($(wc -l < "$L/pose.csv") - 1)) 개 포즈"
