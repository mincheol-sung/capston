#!/bin/bash
# 실차 정사각형 한 판 (2026-11-21, Claude 원격 — 로봇 옆에 사람이 있을 때만).
#   run_square.sh <라벨> [추가 launch 인자...]
#   amrall 과 같은 launch (real_all.launch.py, 도메인 81, 피어 없음) + 공통 정사각형 인자. bag 은 real_all 의 bag:=true (bags/YYYY-MM-DD/HHMMSS).
#   측위 맞추기 배너 "맞음" + 추정기 출력 2 s 이어짐 → /mpc/arm true. 게이트 정지·fault 정지·45 s 중 먼저 오는 것에서 끝내고 launch 를 SIGINT.
#   출발 전·끝난 뒤 SLAM 포즈를 재서 0.4 m 넘게 다르면 종료 코드 5 (다음 판을 돌리지 말 것).
#   종료 코드: 0 끝, 2 측위 안 맞음 (무장 안 함), 3 추정기 출력 없음 (무장 안 함), 4 무장 뒤 끝 신호 없음, 5 출발점에서 멀어짐.
LABEL=$1; shift
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=$HERE/launch_logs/$(date +%H%M%S)_$LABEL.log; mkdir -p "$HERE/launch_logs"
cd /home/smoc/capston/capston
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
unset ROS_STATIC_PEERS
COMMON=${COMMON:-"use_mpc:=true shape:=square yaw_mode:=tangent accel:=1.5 square_v_max:=1.5 origin_mode:=robot max_track_error:=0.5 bag:=true"}   # 2026-11-21: 8자 등은 COMMON 환경변수로 덮어쓴다
echo "[run_square] $LABEL 시작 $(date +%T): $COMMON $*"
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc real_all.launch.py $COMMON "$@" > "$LOG" 2>&1 < /dev/null &
LP=$!
stop() { kill -INT -$LP 2>/dev/null; for i in $(seq 1 40); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$LP 2>/dev/null; }
pose() {   # SLAM 포즈 x y 한 줄 (3 s 안에 없으면 빈 줄)
  timeout 3 ros2 topic echo --once --field pose.position /tracked_pose 2>/dev/null | awk '/^x:/{x=$2} /^y:/{y=$2} END{if (x!="") print x, y}'
}
ok=""
for i in $(seq 1 120); do
  if grep -q "측위 맞추기 끝: 맞음" "$LOG"; then ok=1; break; fi
  if grep -q "측위 맞추기 실패\|Caught exception\|ERROR\] \[launch\]" "$LOG"; then break; fi
  kill -0 $LP 2>/dev/null || break
  sleep 1
done
if [ -z "$ok" ]; then
  echo "[run_square] 측위 맞음 없음 — 무장 안 함"; grep -a "측위 맞추기\|Caught\|ERROR\] \[launch" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | tail -3; stop; exit 2
fi
# 측위 보정 튐 뒤 추정기가 다시 초기화되고 출력을 이어 내는지 — 2 s 동안 50 개 이상
sleep 3
n=$(timeout 2.5 ros2 topic hz /state_estimator/odom --window 50 2>/dev/null | grep -c "average rate")
if [ "$n" -lt 1 ]; then echo "[run_square] 추정기 출력 없음 — 무장 안 함"; stop; exit 3; fi
P0=$(pose)
ros2 topic pub --once /mpc/arm std_msgs/msg/Bool "{data: true}" > /dev/null
TA=$(date +%s); echo "[run_square] $LABEL 무장 $(date +%T)  출발 SLAM ($P0)"
end=""
for i in $(seq 1 90); do
  if grep -a "게이트 정지\|정지(DAMPING)\|fault_monitor_node\]: 정지\|/mpc/arm false" "$LOG" | awk -v ta="$TA" '{ if (match($0, /\[[0-9]{10}\.[0-9]+\]/)) { t = substr($0, RSTART+1, RLENGTH-2); if (t+0 > ta) print } }' | grep -q .; then end=1; break; fi
  sleep 0.5
done
sleep 1.5
P1=$(pose)
grep -a "게이트 무장\|게이트 정지\|정지(DAMPING)\|fault_monitor_node\]: \|SLAM 포즈 튐\|hold 진입" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | grep -v "무장 중 검사" | cut -c1-220 | tail -6
stop
sleep 1
BAG=$(ls -td bags/$(date +%F)/[0-9]* 2>/dev/null | head -1)
echo "[run_square] $LABEL 끝 $(date +%T)  bag $BAG  끝 SLAM ($P1)  남은 노드 $(pgrep -fc 'amr_mpc/lib/amr_mpc/|torque_mpc_node|cartographer_node')"
[ -z "$end" ] && { echo "[run_square] 무장 뒤 끝 신호 없음 (45 s)"; exit 4; }
if [ -n "$P0" ] && [ -n "$P1" ]; then
  d=$(python3 -c "import math,sys; a=list(map(float,'$P0'.split())); b=list(map(float,'$P1'.split())); print(f'{math.hypot(a[0]-b[0], a[1]-b[1]):.3f}')")
  echo "[run_square] 출발점 ↔ 끝 거리 $d m"
  python3 -c "import sys; sys.exit(0 if $d <= 0.4 else 1)" || { echo "[run_square] 출발점에서 0.4 m 넘게 벗어남 — 다음 판 멈춤"; exit 5; }
fi
exit 0
