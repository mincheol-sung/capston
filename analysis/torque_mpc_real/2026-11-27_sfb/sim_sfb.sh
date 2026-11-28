#!/bin/bash
# 바퀴 공간 슬립 되먹임 동작 확인 (2026-11-27 (3)): 시뮬 circles8 (실차 sfb 판과 같은 width 2.0 · speed 1.0 · accel 1.0, motor_only) 한 판.
#   사용: sim_sfb.sh 라벨 [launch 인자...]   도메인 89, 6 s 뒤 무장, bag 22 s. 끝나면 launch 프로세스 그룹을 다 죽인다.
D=$(cd "$(dirname "$0")" && pwd); LABEL=$1; shift
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash; source /home/smoc/capston/capston/install/setup.bash
mkdir -p "$D/sim"; rm -rf "$D/sim/$LABEL"
setsid nice -n 10 /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py plant:=sim use_rviz:=false shape:=circles8 width:=2.0 speed:=1.0 accel:=1.0 "$@" \
  > "$D/sim/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 6
setsid ros2 bag record -s mcap -o "$D/sim/$LABEL" /sim/ground_truth /state_estimator/odom /torque_mpc/status /torque_mpc/slip_fb \
  /torque_mpc/cmd_raw /slip_logger/slip /wheel_states /mpc/armed /dra_mppi/reference_trajectory > "$D/sim/$LABEL.rec.log" 2>&1 < /dev/null &
RP=$!
sleep 1.5
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
sleep 22
kill -INT -$RP 2>/dev/null; for i in $(seq 1 30); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$RP 2>/dev/null
kill -INT -$LP 2>/dev/null; for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$LP 2>/dev/null
[ -f "$D/sim/$LABEL/metadata.yaml" ] || ros2 bag reindex "$D/sim/$LABEL" -s mcap > /dev/null 2>&1
grep -aqE "ARMED by ROS" "$D/sim/$LABEL.log" && a=armed || a=NOT_ARMED
echo "$LABEL $a  남은 sim_plant_torque $(pgrep -fc sim_plant_torque)"
