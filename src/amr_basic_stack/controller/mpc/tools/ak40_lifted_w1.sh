#!/bin/bash
# 바퀴를 띄운 토크 단계 시험 — 바퀴 회전자 식별 지시서 W1 (2026-11-24 사용자 승인 순서). tools/ak40_bench.py lifted 를 부른다.
#
#   bash tools/ak40_lifted_w1.sh <모터 ID 1~4> all                      그 바퀴의 전진 3 판 + 후진 3 판 (확인 한 번, 약 2 분 30 초)
#   bash tools/ak40_lifted_w1.sh <모터 ID 1~4> <fwd|rev> <판 번호>       한 판만
#   … 끝에 dry 를 붙이면 실제로 돌리지 않고 실행할 명령만 보여 준다
#     모터 ID  1 FL · 2 FR · 3 RL · 4 RR (config/ak40.yaml can_ids)
#     fwd|rev  바퀴 기준 전진 / 후진 방향 (왼쪽 바퀴의 모터 부호 뒤집기는 도구가 한다)
#
# ★ 사람이 터미널에서 직접 실행한다. "yes" 확인은 바퀴가 떠 있고 사람이 옆에 있다는 것을 사람이 확인하는 자리다 —
#   입력을 파이프로 넘겨 받는 실행 (echo yes | …) 은 거부한다 (2026-11-24: Claude 가 그렇게 돌리려다 막혔다).
#
# 순서 (바퀴 기준 토크 [N·m] · 유지 시간 [s]) — 값은 사용자 승인 (2026-11-24), 근거 analysis/torque_mpc_real/2026-11-16_jw
#   0.08 · 0.5   출발용 (정지 마찰을 넘긴다. 11-16 에는 0.05 로 안 도는 바퀴가 있었다)
#   0.045 · 4 → 0.030 · 4 → 0.040 · 4 → 0.035 · 4 → 0.045 · 4     일정 속도 8~27 rad/s (11-16 실측 범위)
#   0 · 1.5      자유 감속 (약 26 rad/s 에서 0.4~0.7 s 만에 선다)
# 안전 (도구 기본값): |ω| > 38.7 rad/s (정격) 면 토크 0, 출발 실패·응답 끊김·모터 에러면 중단, 토크 한도 0.1 N·m.
#   Ctrl+C = 즉시 감쇠 정지 뒤 모터 모드 해제 (그 판은 저장 안 됨). all 에서는 한 판이 "완료" 로 안 끝나면 남은 판을 안 돌린다.
#
# 하기 전에
#   1) 로봇을 받침대에 올려 네 바퀴가 바닥·물체에 닿지 않게 한다. 사람이 옆에 있는다.
#   2) ROS 스택 (amrall 등) 을 끈다 — 이 도구와 ak40_can_node 가 같은 모터에 명령하면 안 된다.
#   3) CAN 을 켠다: sudo bash tools/setup_can.sh   (또는 sudo ip link set can0 up type can bitrate 1000000)
#   4) 응답 확인: /usr/bin/python3 tools/ak40_bench.py status --id <ID>
# 결과: analysis/torque_mpc_real/<오늘>_lifted/YYYYMMDD_HHMM_lifted_<바퀴>_<fwd|rev>_r<판>.csv + .json
#   → /usr/bin/python3 tools/wheel_param_id.py analysis/torque_mpc_real/<오늘>_lifted --out analysis/torque_mpc_real/<오늘>_lifted
set -e
ID=$1; DIR=$2
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../../.." && pwd)
case "$ID" in 1|2|3|4) ;; *) echo "모터 ID 는 1~4 (1 FL · 2 FR · 3 RL · 4 RR)"; exit 2 ;; esac
case "$DIR" in
  all) RUNS="fwd:1 fwd:2 fwd:3 rev:1 rev:2 rev:3"; DRY=$3 ;;
  fwd|rev)
    case "$3" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2, 3 …)"; exit 2 ;; esac
    RUNS="$DIR:$3"; DRY=$4 ;;
  *) echo "방향은 fwd · rev · all"; exit 2 ;;
esac
HOLDS="0.5,4,4,4,4,4,1.5"
OUT="$WS/analysis/torque_mpc_real/$(date +%F)_lifted"
WHEEL=$(echo "FL FR RL RR" | cut -d' ' -f"$ID")

cmd_for() {     # $1 = fwd|rev, $2 = 판 번호 → CMD 배열
  local taus
  if [ "$1" = fwd ]; then taus="0.08,0.045,0.030,0.040,0.035,0.045,0"; else taus="-0.08,-0.045,-0.030,-0.040,-0.035,-0.045,0"; fi
  CMD=(/usr/bin/python3 "$HERE/ak40_bench.py" lifted --id "$ID" "--taus=$taus" --holds "$HOLDS" --out_dir "$OUT" --tag "${1}_r${2}")
}

if [ "$DRY" = "dry" ]; then
  for r in $RUNS; do cmd_for "${r%%:*}" "${r##*:}"; printf '%q ' "${CMD[@]}"; echo; done
  exit 0
fi

if pgrep -f "ak40_can_node" > /dev/null; then
  echo "ak40_can_node 가 돌고 있다 — ROS 스택을 끄고 다시 한다 (같은 모터에 두 곳에서 명령하면 안 된다)"; exit 1
fi
if ! ip link show can0 2> /dev/null | grep -q "UP"; then
  echo "can0 이 꺼져 있다 — sudo bash $HERE/setup_can.sh 로 켠 뒤 다시 한다"; exit 1
fi
if [ ! -t 0 ]; then
  echo "확인 입력은 사람이 터미널에서 직접 해야 한다 — 입력을 넘겨 받는 실행은 거부한다"; exit 1
fi
N=$(echo $RUNS | wc -w)
echo "모터 $ID ($WHEEL) 를 $N 판 돌린다: $RUNS  (한 판 약 22 초, 최대 약 27 rad/s)."
read -r -p "바퀴가 떠 있고 사람이 옆에 있으면 yes 를 친다: " OK
[ "$OK" = "yes" ] || { echo "취소"; exit 1; }
cd "$HERE/.."
LOG=$(mktemp)
for r in $RUNS; do
  cmd_for "${r%%:*}" "${r##*:}"
  echo "── $WHEEL ${r%%:*} 판 ${r##*:}"
  "${CMD[@]}" 2>&1 | tee "$LOG"
  if ! grep -q "^끝: 완료" "$LOG"; then
    echo "이 판이 '완료' 로 끝나지 않았다 — 남은 판은 돌리지 않는다"; rm -f "$LOG"; exit 1
  fi
  sleep 2          # 바퀴가 완전히 선 뒤 다음 판
done
rm -f "$LOG"
echo "$WHEEL $N 판 끝 → $OUT"
