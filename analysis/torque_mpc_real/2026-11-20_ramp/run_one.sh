#!/bin/bash
# 실차 램프 한 판 (2026-11-20, Claude 가 원격으로 — 로봇 옆에 사람이 있을 때만).
#   run_one.sh <fwd|back|left|right> [distance=2.0]
#   amrramp 와 같은 launch·환경. 측위 맞추기 배너(맞음)가 나와야 /mpc/arm true, 판이 끝나면 launch 를 SIGINT 로 끈다.
#   배너가 "실패"이거나 시간 안에 안 나오면 무장하지 않고 끝낸다.
DIR=$1; DIST=${2:-2.0}
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=$HERE/launch_$(date +%H%M%S)_$DIR.log
cd /home/smoc/capston/capston
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
unset ROS_STATIC_PEERS
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc torque_ramp.launch.py dir:=$DIR mu_hat:=0.6 distance:=$DIST bag:=ramp_$DIR > "$LOG" 2>&1 < /dev/null &
LP=$!
stop() { kill -INT -$LP 2>/dev/null; for i in $(seq 1 30); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$LP 2>/dev/null; }
ok=""
for i in $(seq 1 120); do
  if grep -q "측위 맞추기 끝: 맞음" "$LOG"; then ok=1; break; fi
  if grep -q "측위 맞추기 실패\|Caught exception\|ERROR\] \[launch\]" "$LOG"; then break; fi
  kill -0 $LP 2>/dev/null || break
  sleep 1
done
# (2026-11-20) 띄운 상태에서 측위 맞추기를 다시 돌리지 않는다 — /initialpose 가 그때마다 Cartographer 궤적을 새로 열어
#   세 번째에 "Non-sorted data added to queue" 로 Cartographer 가 죽었다. 안 맞으면 끄고 다시 띄우거나 로봇을 옮긴다
# FORCE_LOC=1 (2026-11-20 사용자 "65 % 로 무장"): 측위가 "넣었는데 아직 안 맞음" (위치는 잡혔고 겹침만 기준 미달) 이면 무장한다
if [ -z "$ok" ] && [ "${FORCE_LOC:-0}" = 1 ] && grep -q "넣었는데 아직 안 맞음" "$LOG"; then
  ok=1; echo "[run_one] 측위 기준 미달이지만 FORCE_LOC=1 — 무장 진행"; grep -a "check_localization-[0-9]*\] 다시" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | tail -1
fi
if [ -z "$ok" ]; then
  echo "[run_one] 측위 맞음 없음 — 무장 안 함"; grep -a "측위 맞추기\|Caught\|ERROR" "$LOG" | tail -3; stop; exit 2
fi
sleep 3
# 추정기가 실제로 출력을 내는지 (측위 보정 튐 뒤 다시 초기화 대기) — 없으면 무장하지 않는다
if ! timeout 6 ros2 topic echo --once /state_estimator/odom > /dev/null 2>&1; then echo "[run_one] 추정기 출력 없음 — 무장 안 함"; stop; exit 3; fi
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
echo "[run_one] $DIR 무장 $(date +%T)"
for i in $(seq 1 40); do
  grep -q "\[run 1\] 끝" "$LOG" && break
  sleep 1
done
sleep 1.5
grep -a "torque_ramp_node\]: \[run 1\]\|fault_monitor_node\]: 정지\|torque_ramp: " "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-260
stop
sleep 1
echo "[run_one] 끝 $(date +%T), 남은 노드 $(pgrep -fc 'amr_mpc/lib/amr_mpc/(torque_ramp|ak40_can|state_estimator)_node')"
