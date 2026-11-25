#!/bin/bash
# 시뮬 한 판 (도메인 지정). 사용법: run_case.sh 출력폴더 라벨 도메인 [launch 인자...]
#   ../2026-11-23_load_transfer/run_case.sh 와 같고, 기록기 시간 한도 150 s · npz 저장 · launch 로그를 지우지 않고 남긴다.
OUT=$1; LABEL=$2; DOM=$3; shift 3
mkdir -p "$OUT"
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash
source /home/smoc/capston/capston/install/setup.bash
cd /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py use_rviz:=false "$@" > "$OUT/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 6
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
/usr/bin/python3 ${REC:-tools/compare_run.py} --label "$LABEL" --timeout 150 --npz "$OUT/$LABEL.npz" > "$OUT/$LABEL.json" 2> "$OUT/$LABEL.rec.err"
kill -INT -$LP 2>/dev/null
for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done
kill -KILL -$LP 2>/dev/null
grep -E "주행 끝|8자 완주|path_follow|ERROR" "$OUT/$LABEL.log" > "$OUT/$LABEL.events" 2>/dev/null
