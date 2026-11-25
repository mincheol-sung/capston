#!/bin/bash
# 측위만 확인 (무장 안 함): run_square.sh 와 같은 launch → 측위 맞추기 배너 → loc_probe.py 3 번 (8 s 간격) → 끈다.
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=$HERE/launch_logs/$(date +%H%M%S)_loc_only.log; mkdir -p "$HERE/launch_logs"
cd /home/smoc/capston/capston
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
unset ROS_STATIC_PEERS
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 launch amr_mpc real_all.launch.py use_mpc:=true limit_mode:=friction shape:=square yaw_mode:=tangent accel:=1.5 square_v_max:=1.5 origin_mode:=robot > "$LOG" 2>&1 < /dev/null &
LP=$!
stop() { kill -INT -$LP 2>/dev/null; for i in $(seq 1 40); do kill -0 $LP 2>/dev/null || break; sleep 0.5; done; kill -KILL -$LP 2>/dev/null; }
for i in $(seq 1 120); do
  grep -q "측위 맞추기 끝\|측위 맞추기 실패\|Caught exception" "$LOG" && break
  sleep 1
done
grep -a "check_localization-[0-9]*\] \(지금\|찾은\|다시\|→\)\|측위 맞추기 끝\|측위 맞추기 실패" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | sed 's/.*check_localization-[0-9]*\] //' | cut -c1-140
sleep 3
/usr/bin/python3 "$HERE/loc_probe.py" 3 8
stop
echo "[loc_only] 끝 $(date +%T)"
