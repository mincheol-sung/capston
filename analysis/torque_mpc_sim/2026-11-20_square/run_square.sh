#!/bin/bash
# 정사각형 시뮬 한 판 (tools/run_compare.sh 를 NUC 설치본 install 로 옮긴 것). 사용법: run_square.sh 라벨 [launch 인자...]
LABEL=$1; shift
SP=/tmp/claude-1000/-home-smoc-capston/69d56774-629f-4670-9d6f-69486a1cc5db/scratchpad
OUT=$SP/square_runs; mkdir -p "$OUT"
export ROS_DOMAIN_ID=93 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash
source /home/smoc/capston/capston/install/setup.bash
cd /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py use_rviz:=false shape:=square "$@" > "$OUT/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 6
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
/usr/bin/python3 $SP/square_rec.py "$LABEL" 1.5 "$OUT" > "$OUT/$LABEL.json" 2> "$OUT/$LABEL.rec.err"
kill -INT -$LP 2>/dev/null
for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done
kill -KILL -$LP 2>/dev/null
cat "$OUT/$LABEL.json"
