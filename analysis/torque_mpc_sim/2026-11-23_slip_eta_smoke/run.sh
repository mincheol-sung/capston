#!/bin/bash
# 슬립 계수 η_s 구현 확인 (2026-11-23): 같은 조건에서 slip_eta 끔/켬 시뮬 한 판씩.  사용: run.sh 라벨 [launch 인자...]
#   조건: C (friction), μ_max 1.0 (mu_hat_wheels:=off mu_hat:=1.0 — 회의 결정), 시뮬 바닥 μ 0.3 (슬립이 나게), 직선 앞 1.0 m/s a 3.0
#   도메인 88 (다른 세션 시뮬과 안 겹치게). 6 s 뒤 무장, bag 25 s.
D=$(cd "$(dirname "$0")" && pwd); LABEL=$1; shift
export ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash; source /home/smoc/capston/capston/install/setup.bash
mkdir -p "$D/runs"; rm -rf "$D/runs/$LABEL"
setsid nice -n 10 /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py plant:=sim use_rviz:=false limit_mode:=friction mu_hat_wheels:=off mu_hat:=1.0 \
  surface_mu:=0.3 shape:=line seg_distance:=1.5 seg_dir_deg:=0 speed:=1.0 accel:=3.0 "$@" > "$D/runs/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 6
setsid ros2 bag record -s mcap -o "$D/runs/$LABEL" /sim/ground_truth /state_estimator/odom /torque_mpc/status /torque_mpc/slip \
  /torque_mpc/load /torque_mpc/cmd_raw /slip_logger/slip /wheel_states /mpc/armed > "$D/runs/$LABEL.rec.log" 2>&1 < /dev/null &
RP=$!
sleep 1.5
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
sleep 25
kill -INT -$RP 2>/dev/null; for i in $(seq 1 30); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$RP 2>/dev/null
kill -INT -$LP 2>/dev/null; for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$LP 2>/dev/null
[ -f "$D/runs/$LABEL/metadata.yaml" ] || ros2 bag reindex "$D/runs/$LABEL" -s mcap > /dev/null 2>&1
grep -aqE "ARMED by ROS" "$D/runs/$LABEL.log" && a=armed || a=NOT_ARMED
echo "$LABEL $a"
