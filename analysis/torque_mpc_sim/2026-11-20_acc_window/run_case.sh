#!/bin/bash
# 시뮬 한 판 (도메인 지정). 사용법: run_case.sh 출력폴더 라벨 도메인 [launch 인자...]
#   tools/run_compare.sh 를 NUC install 로. 기록은 tools/compare_run.py (참값 기준).
OUT=$1; LABEL=$2; DOM=$3; shift 3
mkdir -p "$OUT"
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1   # 시뮬 플랜트 numpy BLAS 스레드가 판마다 3.4 코어를 먹었다
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash
source /home/smoc/capston/capston/install/setup.bash
cd /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py use_rviz:=false "$@" > "$OUT/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 6
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
/usr/bin/python3 ${REC:-tools/compare_run.py} --label "$LABEL" --timeout 90 > "$OUT/$LABEL.json" 2> "$OUT/$LABEL.rec.err"
kill -INT -$LP 2>/dev/null
for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done
kill -KILL -$LP 2>/dev/null
grep -v '^$' "$OUT/$LABEL.log" | grep -q "torque_mpc" || echo "no mpc in log" >> "$OUT/$LABEL.rec.err"
rm -f "$OUT/$LABEL.log"   # 용량 — 필요하면 지우지 말 것
