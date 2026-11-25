#!/bin/bash
# 시뮬 1 회차 (도메인 87, localhost): launch + 무장 + tools/compare_run.py 요약 + 작은 bag. 결과는 runs/
LABEL=$1; shift
OUT=/home/smoc/capston/capston/analysis/torque_mpc_sim/2026-11-22_fig8_speedmod/runs
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source /home/smoc/capston/capston/install/setup.bash
cd /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py use_rviz:=false log_prefix:=$OUT/$LABEL "$@" > "$OUT/$LABEL.log" 2>&1 < /dev/null &
LP=$!
rm -rf $OUT/${LABEL}_bag
setsid ros2 bag record -s mcap -o $OUT/${LABEL}_bag /sim/ground_truth /state_estimator/odom /figure8/path /mpc/armed /dra_mppi/reference_trajectory /torque_mpc/status /pure_pursuit/status /wheel_states /imu/base_link /torque_output/slip_guard > /dev/null 2>&1 < /dev/null &
BP=$!
sleep 6
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
/usr/bin/python3 tools/compare_run.py --label "$LABEL" --timeout 60 > "$OUT/$LABEL.json" 2> "$OUT/$LABEL.rec.err"
kill -INT -$BP 2>/dev/null
for i in $(seq 1 20); do kill -0 $BP 2>/dev/null || break; sleep 0.5; done
kill -INT -$LP 2>/dev/null
for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done
kill -KILL -$LP -$BP 2>/dev/null
tail -1 "$OUT/$LABEL.json"
