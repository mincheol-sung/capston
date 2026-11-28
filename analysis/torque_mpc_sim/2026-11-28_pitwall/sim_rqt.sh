#!/usr/bin/env bash
# 2026-11-28 pitwall: 시뮬 스택 (도메인 97) + rqt_reconfigure 화면 확인 (rqt_snapshot.py, Qt offscreen). 실차 판 중에는 돌리지 말 것.
# 돌리는 중에 이 파일을 고치지 말 것 (bash 가 다시 읽는다).
OUT=${1:-/tmp/pitwall_rqt}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
# 실차 판이 돌면 멈춘다 (2026-11-28 실차 판 c8_adpp_r1 도중 시뮬을 돌린 일 뒤에 넣음)
if pgrep -f '^/usr/bin/python3 .*/ak40_can_node' >/dev/null || pgrep -f 'ros2 launch amr_mpc real_all.launch.py' >/dev/null; then
  echo "실차 판 (ak40_can_node / real_all) 이 돌고 있다 — 시뮬을 돌리지 않는다"; exit 3
fi
source /opt/ros/jazzy/setup.bash
source /home/smoc/capston/capston/install/setup.bash
export ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
setsid ros2 launch amr_mpc torque_experiment.launch.py plant:=sim use_rviz:=false controller:=mpc limit_mode:=friction \
  origin_mode:=robot shape:=circles8 width:=2.0 speed:=0.5 accel:=0.8 laps:=1 start_delay:=1.0 hold_s:=2.0 \
  > "$OUT/launch.log" 2>&1 &
LPID=$!
sleep 15
QT_QPA_PLATFORM=offscreen timeout 90 /usr/bin/python3 "$HERE/rqt_snapshot.py" "$OUT" 2>&1 | grep -v "^QStandardPaths\|propagateSizeHints" | tee "$OUT/rqt.txt"
kill -INT -- -$LPID; sleep 10
kill -TERM -- -$LPID 2>/dev/null; sleep 2
echo "sim_plant_torque 남은 프로세스: $(pgrep -f '^/usr/bin/python3 /home/smoc/capston/capston/install/amr_mpc/lib/amr_mpc/sim_plant_torque_node' | wc -l)" | tee -a "$OUT/rqt.txt"
