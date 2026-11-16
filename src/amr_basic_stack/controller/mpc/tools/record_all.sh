#!/bin/bash
# 실차 디버깅용 bag: 모든 토픽 (카메라 영상·점군·Cartographer 표시용만 뺀다) + 실행 정보 (2026-11-16).
#   bash tools/record_all.sh [이름] [실행 정보 문자열]
#   amr_aliases.sh 의 amrrec, real_all.launch.py 의 bag:=<이름> 이 이 스크립트를 쓴다.
#
# 이름 → 저장 폴더 (작업공간 bags/ 아래, 날짜·시각 자동. 2026-11-16 사용자 요청)
#   (없음) · true · auto · 1   →  bags/<YYYY-MM-DD>/<HHMMSS>              예) bags/2026-11-16/134512
#   diag_fast   (/ 없는 이름)  →  bags/<YYYY-MM-DD>/<HHMMSS>_diag_fast    예) bags/2026-11-16/134512_diag_fast
#   2026-11-16_5-3/diag_r1     →  bags/2026-11-16_5-3/diag_r1 (/ 가 있으면 그대로, 예전 방식)
#   /절대/경로                  →  그대로
#   RECORD_ALL_DRYRUN=1 이면 정한 폴더만 찍고 끝낸다 (시험용).
#
# 녹화: ros2 bag record --all-topics (mcap). /rosout(노드 로그), /ak40_can/status(CAN 응답 누락 누적 수),
#       /wheel_states(바퀴별 fresh), /torque_mpc/status(오차·토크·d̂·hold·U_s), /state_estimator/*, /tracked_pose, /imu/*, /scan*, /tf* …
#   뺀 것:
#     · 이름이 image_raw·image_rect_raw·/image·/points 로 끝나거나 그 아래(…/compressed 등) — RealSense 컬러·깊이 영상(약 46 MB/s)과
#       점군. 영상 압축 토픽을 구독하면 카메라 노드가 압축하느라 CPU 를 쓴다.
#     · Cartographer 표시용 /trajectory_node_list · /landmark_poses_list · /constraint_list (RViz 마커). 저장 지도의 노드 전체를
#       약 33 Hz 로 다시 보내 bag 의 약 60 % 였다 (2026-11-16 142932: 396 MiB 중 240 MiB, 135202: 1.3 GiB 중 821 MiB).
#   예상 크기: 약 2 MB/s.
#
# 같은 폴더 run_info/ (끝날 때 옮긴다 — bag 에는 파라미터·코드·시스템 상태가 안 들어간다):
#   args.txt              시작 시각, 출력, ROS_DOMAIN_ID·peers, 두 번째 인자(launch 인자 등)
#   git.txt · git.diff    커밋 해시 + git status, 커밋 안 된 변경 전체
#   params/               실험 노드 파라미터 (녹화 20 s 뒤 ros2 param dump) — MPC 가중치(W, w_delta …)가 실제로 몇이었는지
#   can_link_start.txt · can_link_end.txt   ip -s -d link show can0 — 버스 에러, error-warning/passive, bus-off·재시작 횟수
#   candump.log           CAN 원본 프레임 (candump -L, 시각은 epoch) — 응답 누락이 명령 쪽인지 응답 쪽인지
#   mpstat.txt · pidstat.txt · thermal.csv   1 s 마다 CPU(전체·프로세스별)·온도 — "상태 주기 ≠ Ts" 경고가 부하 때문인지
#   ros_log/              녹화 중 바뀐 ~/.ros/log 의 노드 로그·launch 로그 — Cartographer 최적화 시각(/rosout 으로 안 나온다) 등
# 끝내기: Ctrl+C (real_all 이면 launch 와 같이). 녹화기는 따로 세션으로 띄워 SIGINT 를 한 번만 받게 한다.
set -u
NAME="${1:-}"
INFO="${2:-}"
WS="$(cd "$(dirname "$0")/../../../../.." && pwd)"
DAY="$(date +%F)"; HMS="$(date +%H%M%S)"
case "$NAME" in
  ""|true|True|auto|1) OUT="$WS/bags/$DAY/$HMS" ;;
  /*)                  OUT="$NAME" ;;
  */*)                 OUT="$WS/bags/$NAME" ;;
  *)                   OUT="$WS/bags/$DAY/${HMS}_$NAME" ;;
esac
if [ -e "$OUT" ]; then
  OUT="${OUT}_$HMS"
  echo "[record_all] 같은 이름이 있어 ${OUT} 로 녹화한다"
fi
if [ -n "${RECORD_ALL_DRYRUN:-}" ]; then echo "$OUT"; exit 0; fi
mkdir -p "$(dirname "$OUT")"
EXCLUDE='(.*/(image_raw|image_rect_raw|image|points)(/.*)?)|(/(trajectory_node_list|landmark_poses_list|constraint_list))'
CAN="${RECORD_ALL_CAN:-can0}"
TMP="${OUT}.run_info.tmp"                 # 녹화기가 OUT 을 직접 만들어야 해서(있으면 거부) 끝날 때 OUT/run_info 로 옮긴다
mkdir -p "$TMP/params"
START=$(date +%s)
PIDS=()

# ── 시작 정보
{ date '+%F %T'; echo "출력 $OUT"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-}"; echo "$INFO"; } > "$TMP/args.txt"
{ git -C "$WS" rev-parse HEAD; git -C "$WS" status --short; } > "$TMP/git.txt" 2>&1
git -C "$WS" diff > "$TMP/git.diff" 2>&1

# ── 같이 도는 기록기: CAN 원본·카운터, CPU·온도
HAVE_CAN=0
if ip link show "$CAN" > /dev/null 2>&1; then
  HAVE_CAN=1
  ip -s -d link show "$CAN" > "$TMP/can_link_start.txt" 2>&1
  command -v candump > /dev/null && { candump -L "$CAN" > "$TMP/candump.log" 2> "$TMP/candump.err" & PIDS+=($!); }
fi
command -v mpstat > /dev/null && { mpstat 1 > "$TMP/mpstat.txt" 2>&1 & PIDS+=($!); }
command -v pidstat > /dev/null && { pidstat -u -h 1 > "$TMP/pidstat.txt" 2>&1 & PIDS+=($!); }
(
  echo "epoch,$(for z in /sys/class/thermal/thermal_zone*; do printf '%s,' "$(cat "$z/type" 2>/dev/null)"; done)"
  while :; do
    printf '%s,' "$(date +%s.%N)"
    for z in /sys/class/thermal/thermal_zone*; do printf '%s,' "$(cat "$z/temp" 2>/dev/null)"; done   # m°C
    echo; sleep 1
  done
) > "$TMP/thermal.csv" 2> /dev/null &
PIDS+=($!)

# ── 녹화 20 s 뒤 파라미터 (노드가 다 뜬 뒤)
(
  sleep 20
  for n in torque_mpc_node state_estimator_node figure8_reference_node torque_output_node ak40_can_node \
           fault_monitor_node torque_command_gate slip_logger_node torque_viz_node imu_to_base_node; do
    timeout 10 ros2 param dump "/$n" > "$TMP/params/$n.yaml" 2>/dev/null || rm -f "$TMP/params/$n.yaml"
  done
  echo "[record_all] 파라미터 저장 ($(ls "$TMP/params" | wc -l) 개 노드)"
) &
PARAM_PID=$!

# ── 녹화 (자기 세션: 터미널 Ctrl+C 가 녹화기에 직접 가지 않고, 아래 trap 이 SIGINT 를 한 번만 넘긴다)
echo "[record_all] 녹화 → $OUT  (모든 토픽, 영상·점군·카토 표시용 제외. 끝낼 때 Ctrl+C)"
#   SIGINT 을 SIG_DFL 로 되돌려 실행한다 — 부모가 SIGINT 를 무시로 물려줬으면(& 로 띄운 셸) 녹화기도 SIGINT 를 못 받는다
#   (2026-11-16 시험에서 확인, tools/run_compare.sh 와 같은 문제). 그래도 5 s 안에 안 끝나면 SIGTERM (rosbag2 는 둘 다 정상 종료).
setsid /usr/bin/python3 -c "import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])" \
  ros2 bag record --all-topics --storage mcap --exclude-regex "$EXCLUDE" -o "$OUT" &
REC=$!
STOP_T=""
trap 'if [ -z "$STOP_T" ]; then STOP_T=$(date +%s); kill -INT "$REC" 2>/dev/null; fi' INT TERM
while kill -0 "$REC" 2>/dev/null; do
  sleep 0.2
  if [ -n "$STOP_T" ] && [ $(( $(date +%s) - STOP_T )) -ge 5 ]; then kill -TERM "$REC" 2>/dev/null; STOP_T=$(( STOP_T + 1000 )); fi
done
trap - INT TERM

# ── 끝 처리 (launch 가 SIGTERM 을 보내기 전 몇 초 안에 끝나야 한다)
kill "${PIDS[@]}" "$PARAM_PID" 2>/dev/null
[ "$HAVE_CAN" = 1 ] && ip -s -d link show "$CAN" > "$TMP/can_link_end.txt" 2>&1
mkdir -p "$TMP/ros_log"
find "$HOME/.ros/log" -maxdepth 1 -type f -name '*.log' -newermt "@$START" -exec cp -p {} "$TMP/ros_log/" \; 2>/dev/null
for f in $(find "$HOME/.ros/log" -mindepth 2 -maxdepth 2 -name launch.log -newermt "@$START" 2>/dev/null); do
  cp -rp "$(dirname "$f")" "$TMP/ros_log/" 2>/dev/null
done
if [ -d "$OUT" ]; then
  mkdir -p "$OUT/run_info" && cp -rp "$TMP"/. "$OUT/run_info/" && rm -rf "$TMP"
  echo "[record_all] 끝: $OUT ($(du -sh "$OUT" | cut -f1)), run_info/ 에 CAN·CPU·온도·로그"
else
  mv "$TMP" "${OUT}_run_info"
  echo "[record_all] bag 이 만들어지지 않았다 — 실행 정보만 ${OUT}_run_info"
fi
