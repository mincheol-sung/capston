#!/bin/bash
# 시뮬 비교 한 판: launch 를 화면 없이 띄우고 5 s 뒤 무장, tools/compare_run.py 로 참값 기준 추종 오차를 JSON 으로 남긴다.
#   사용법: tools/run_compare.sh 라벨 ROS_DOMAIN_ID [launch 인자...]
#     예) tools/run_compare.sh B 81
#         tools/run_compare.sh A 81 controller:=pure_pursuit surface_mu:=0.3
#   결과: $OUT/라벨.json (요약), 라벨.log (launch 출력), 라벨.rec.err.  OUT 기본 /tmp/amr_compare_runs
#   ROS_DOMAIN_ID 는 사용자 터미널과 섞이지 않게 따로 준다 (linux-pc 빌드 메모).
# 주의 (stack_changelog 2026-11-11·12)
#   · & 로 띄운 프로세스는 SIGINT 를 "무시" 로 물려받아 Ctrl+C 를 못 받는다 → SIG_DFL 로 되돌려 exec 한다.
#   · armed:=true 로 띄우면 센서가 오기 전에 fault_monitor 가 세운다 → 띄우고 5 s 뒤 /mpc/arm.
#   · 데스크톱이 바쁘면 약 1/5 판에서 시뮬 노드가 0.3 s 멈춰 fault_monitor 가 세운다 (JSON fault 에 timeout) — 그 판은 다시.
LABEL=$1; DOM=$2; shift 2
OUT=${OUT:-/tmp/amr_compare_runs}
mkdir -p "$OUT"
export ROS_DOMAIN_ID=$DOM
source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "$0")/../../../../.." && pwd)/install_pc/setup.bash"
cd "$(dirname "$0")/.."
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_experiment.launch.py use_rviz:=false "$@" > "$OUT/$LABEL.log" 2>&1 < /dev/null &
LP=$!
sleep 5
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
/usr/bin/python3 tools/compare_run.py --label "$LABEL" --timeout 90 > "$OUT/$LABEL.json" 2> "$OUT/$LABEL.rec.err"
kill -INT -$LP 2>/dev/null
for i in $(seq 1 20); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done
kill -KILL -$LP 2>/dev/null
cat "$OUT/$LABEL.json"
