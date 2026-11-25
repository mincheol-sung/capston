#!/bin/bash
# 실차 한 판: real_all 띄우기 → 측위 배너 (초록) 기다림 → /mpc/arm → "8자 완주" → Ctrl+C (bag 닫힘).
# 배너가 빨강 / 120 s 안에 안 나옴 / 무장 뒤 완주 전에 DISARMED (fault_monitor·slip_guard) → 무장 안 하거나 바로 내린다.
# 사용: real_run.sh <라벨> [추가 launch 인자...]   예) real_run.sh h010 mpc_com_height:=0.10   ·  PP: real_run.sh ppb PP
# 2026-11-23 h 실차 스윕 (amrall 과 같은 환경, CAN 이 이미 UP 이라 sudo 없음)
# (set -u 는 ROS setup.bash 와 안 맞는다)
LABEL="$1"; shift
HERE="$(cd "$(dirname "$0")" && pwd)"; LOG="$HERE/logs/$LABEL.log"; mkdir -p "$HERE/logs"
# can0 이 없으면 launch 의 CAN 자동 준비가 sudo 비밀번호에서 막힌다 (재부팅 뒤, 2026-11-23 pf_c) — 먼저 손으로 올리게 알린다
if ! ip link show can0 2>/dev/null | grep -q "state UP\|,UP"; then
  echo "[run] can0 가 없거나 내려가 있다 — 먼저 이 터미널에서: bash /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools/setup_can.sh (sudo 비밀번호)"
  exit 6
fi
export AMR_WS=/home/smoc/capston/capston
cd "$AMR_WS" || exit 1
source /opt/ros/jazzy/setup.bash; source "$AMR_WS/install/setup.bash"
export ROS_DOMAIN_ID=81 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
unset ROS_STATIC_PEERS
ARGS=(accel:=3.0 auto_localize:=true bag:="$LABEL" disturbance:=true laps:=2 limit_mode:=friction localization:=true
      localize_delay:=15.0 lua:=localization_torque_2d.lua map:=smoc_lab max_track_error:=0.5 sensors:=true shape:=circles8
      slip_guard:=true speed:=0.69 speed_mod_amp:=0.5 speed_mod_cycles:=2 speed_mod_phase_deg:=-90 speed_mod_shape:=square
      use_mpc:=true velocity_source:=wheels viz:=true width:=1.6 yaw_mode:=tangent "$@")
if [ "${1:-}" = PP ]; then   # PP 151249 와 같은 인자 (controller 만 다르고 C 전용 인자 없음)
  shift; ARGS=(accel:=3.0 auto_localize:=true bag:="$LABEL" controller:=pure_pursuit laps:=2 localization:=true localize_delay:=15.0
        lua:=localization_torque_2d.lua map:=smoc_lab max_track_error:=0.5 sensors:=true shape:=circles8 speed:=0.69 speed_mod_amp:=0.5
        speed_mod_cycles:=2 speed_mod_phase_deg:=-90 speed_mod_shape:=square use_mpc:=true viz:=true width:=1.6 yaw_mode:=tangent "$@")
fi
# 같은 이름 인자는 뒤에 준 것 (명령줄) 만 남긴다 — 예) disturbance:=false 가 기본 disturbance:=true 를 덮는다
declare -A SEEN; OUT=()
for ((j=${#ARGS[@]}-1; j>=0; j--)); do k="${ARGS[j]%%:=*}"; [ -n "${SEEN[$k]:-}" ] && continue; SEEN[$k]=1; OUT=("${ARGS[j]}" "${OUT[@]}"); done
ARGS=("${OUT[@]}")
echo "[run] $LABEL: ${ARGS[*]}" | tee "$LOG"
setsid ros2 launch amr_mpc real_all.launch.py "${ARGS[@]}" </dev/null >>"$LOG" 2>&1 &
LP=$!
stop() {  # launch 묶음에 SIGINT → bag 정상 종료. 40 s 안에 안 끝나면 TERM
  echo "[run] stop: $1" | tee -a "$LOG"
  timeout 5 ros2 topic pub --once -w 1 /mpc/arm std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
  kill -INT -- -"$LP" 2>/dev/null
  sleep 1; pkill -INT -f "ros2 bag record.*_${LABEL}\$" 2>/dev/null   # 녹화기는 자기 세션 — launch 의 SIGINT 가 안 닿은 적 있음 (h010)
  for _ in $(seq 40); do kill -0 "$LP" 2>/dev/null || break; sleep 1; done
  kill -0 "$LP" 2>/dev/null && { echo "[run] TERM" | tee -a "$LOG"; kill -TERM -- -"$LP"; sleep 3; }
}
# ① 측위 배너
for i in $(seq 120); do
  grep -q "측위 맞추기 끝: 맞음" "$LOG" && break
  if grep -q "측위 맞추기 실패" "$LOG"; then grep -A1 "측위 맞추기 실패" "$LOG" | tail -2; stop "localization failed"; exit 2; fi
  kill -0 "$LP" 2>/dev/null || { echo "[run] launch died"; exit 3; }
  sleep 1
done
grep -q "측위 맞추기 끝: 맞음" "$LOG" || { stop "no banner in 120 s"; exit 2; }
echo "[run] banner OK ($(grep -c . "$LOG") lines) — arm in 2 s" | tee -a "$LOG"
sleep 2
NB=$(grep -c "DISARMED by" "$LOG")
timeout 15 ros2 topic pub --once -w 1 /mpc/arm std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
echo "[run] ARM sent $(date +%T)" | tee -a "$LOG"
# ② 완주 기다림 (circles8 2 바퀴 ≈ 30 s, 넉넉히 90 s)
for i in $(seq 90); do
  grep -q "8자 완주" "$LOG" && { sleep 3; stop "lap done"; echo "[run] DONE"; exit 0; }
  if [ "$(grep -c "DISARMED by" "$LOG")" -gt "$NB" ]; then grep "DISARMED by\|FAULT\|fault" "$LOG" | tail -3; stop "disarmed during run"; exit 4; fi
  kill -0 "$LP" 2>/dev/null || { echo "[run] launch died during run"; exit 3; }
  sleep 1
done
stop "timeout 90 s"; exit 5
