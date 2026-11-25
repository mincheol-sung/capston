#!/bin/bash
# 시뮬 한 판: launch(화면 없음) → 6 s 뒤 무장 → bag 30 s → 종료 → reindex.  사용: run.sh 라벨 [launch 인자...]
#   공통 조건은 cases.txt 의 인자 + 아래 고정값 (C, 추정기 속도, 바닥 μ 0.45, 잡음 1 배). 시뮬 IMU·SLAM 포즈도 녹화 (오프라인 재생용)
D=$(cd "$(dirname "$0")" && pwd); LABEL=$1; shift
export ROS_DOMAIN_ID=94 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash; source /home/smoc/capston/capston/install/setup.bash
mkdir -p "$D/runs"; rm -rf "$D/runs/$LABEL"
# 실차 스택이 같은 NUC 에서 돌 때는 NICE=19 로 (시뮬이 느려져 끊김이 날 수 있다 — 로그의 정지 줄로 확인)
NICE=${NICE:-0}
setsid nice -n "$NICE" /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py plant:=sim use_rviz:=false limit_mode:=friction velocity_source:=estimator \
  surface_mu:=0.45 sensor_noise_scale:=1.0 "$@" > "$D/runs/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 6
setsid ros2 bag record -s mcap -o "$D/runs/$LABEL" /sim/ground_truth /state_estimator/odom /state_estimator/status /fault_monitor/fault \
  /mpc/armed /torque_mpc/status /torque_mpc/cmd_raw /dra_mppi/reference_trajectory /figure8/path /slip_logger/slip \
  /wheel_states /imu/base_link /tracked_pose > "$D/runs/$LABEL.rec.log" 2>&1 < /dev/null &
RP=$!
sleep 1.5
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
sleep 30
kill -INT -$RP 2>/dev/null; for i in $(seq 1 30); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$RP 2>/dev/null
kill -INT -$LP 2>/dev/null; for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$LP 2>/dev/null
[ -f "$D/runs/$LABEL/metadata.yaml" ] || ros2 bag reindex "$D/runs/$LABEL" -s mcap > /dev/null 2>&1
grep -aqE "ARMED by ROS" "$D/runs/$LABEL.log" && a=armed || a=NOT_ARMED
f=$(grep -acE "정지:" "$D/runs/$LABEL.log"); echo "$LABEL $a fault_lines=$f"
