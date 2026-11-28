#!/usr/bin/env bash
# 2026-11-28 pitwall 시뮬 확인 2: new_run (무장 중 거부·멈춤 중 버림), MPC 값 거부, launch 전용 값 거부 (read_only),
# RViz 토픽 (/figure8/next_path·status_marker). 도메인 97. 실차 판이 돌 때는 돌리지 말 것. 돌리는 중에 이 파일을 고치지 말 것 (bash 가 다시 읽는다).
OUT=${1:-/tmp/pitwall_sim2}
mkdir -p "$OUT"
# 실차 판이 돌면 멈춘다 (2026-11-28 실차 판 c8_adpp_r1 도중 시뮬을 돌린 일 뒤에 넣음)
if pgrep -f '^/usr/bin/python3 .*/ak40_can_node' >/dev/null || pgrep -f 'ros2 launch amr_mpc real_all.launch.py' >/dev/null; then
  echo "실차 판 (ak40_can_node / real_all) 이 돌고 있다 — 시뮬을 돌리지 않는다"; exit 3
fi
source /opt/ros/jazzy/setup.bash
source /home/smoc/capston/capston/install/setup.bash
export ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
PIT="/usr/bin/python3 /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools/pit.py"
log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/steps.txt"; }
arm() { ros2 topic pub --once -w 1 /mpc/arm std_msgs/msg/Bool "{data: $1}" >/dev/null 2>&1; log "arm $1"; }
: > "$OUT/steps.txt"
log "launch"
setsid ros2 launch amr_mpc torque_experiment.launch.py plant:=sim use_rviz:=false controller:=mpc limit_mode:=friction \
  origin_mode:=robot shape:=circles8 width:=2.0 speed:=0.5 accel:=0.8 laps:=1 start_delay:=1.0 hold_s:=2.0 \
  > "$OUT/launch.log" 2>&1 &
LPID=$!
sleep 15
log "launch 전용 값 (read_only) — ros2 param set dt"; timeout 15 ros2 param set /figure8_reference_node dt 0.1 2>&1 | tee -a "$OUT/steps.txt"
log "MPC 범위 밖 — disturbance_alpha=0"; $PIT set disturbance_alpha=0.0 2>&1 | head -3 | tee -a "$OUT/steps.txt"
log "MPC launch 전용 — ros2 param set Np"; timeout 15 ros2 param set /torque_mpc_node Np 50 2>&1 | tee -a "$OUT/steps.txt"
log "RViz 토픽: next_path 점 수, status_marker 글자"
timeout 8 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /figure8/next_path nav_msgs/msg/Path 2>/dev/null | grep -c "position:" | sed 's/^/  next_path 점 수 /' | tee -a "$OUT/steps.txt"
timeout 8 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /figure8/status_marker visualization_msgs/msg/Marker 2>/dev/null | grep "^text" | tee -a "$OUT/steps.txt"
arm true
sleep 4
log "무장 중 pit new (거부돼야)"; $PIT new 2>&1 | tee -a "$OUT/steps.txt"
arm false
sleep 1
log "멈춤 중 pit new (판 버림)"; $PIT new 2>&1 | tee -a "$OUT/steps.txt"
$PIT show 2>&1 | head -2 | tee -a "$OUT/steps.txt"
arm true
sleep 4
arm false
log "pit show (판 2 멈춤)"; $PIT show 2>&1 | head -2 | tee -a "$OUT/steps.txt"
kill -INT -- -$LPID; sleep 10
kill -TERM -- -$LPID 2>/dev/null; sleep 2
left=$(pgrep -f '^/usr/bin/python3 /home/smoc/capston/capston/install/amr_mpc/lib/amr_mpc/sim_plant_torque_node' | wc -l)
log "sim_plant_torque 남은 프로세스: $left"
