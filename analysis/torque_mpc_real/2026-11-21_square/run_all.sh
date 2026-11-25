#!/bin/bash
# 8 판 연속: P1 C1 P2 C2 × 2 (2026-11-21, 사용자 "알아서 싹다 돌려서 비교"). 로봇 옆에 사람이 있을 때만.
#   측위 실패 (2·3) 는 그 판만 한 번 다시. 4·5 (끝 신호 없음, 출발점에서 멀어짐) 이거나 같은 판이 두 번 실패하면 전체를 멈춘다.
HERE=$(cd "$(dirname "$0")" && pwd)
MU='mu_hat_wheels:=0.486,0.495,0.513,0.469'
CASES=(
  "P1a|controller:=pure_pursuit"
  "C1a|limit_mode:=friction $MU"
  "P2a|controller:=pure_pursuit pp_lookahead_min:=0.15 pp_lookahead_max:=0.60"
  "C2a|limit_mode:=friction $MU mpc_W:=20,20,2.5,2,2,1"
  "P1b|controller:=pure_pursuit"
  "C1b|limit_mode:=friction $MU"
  "P2b|controller:=pure_pursuit pp_lookahead_min:=0.15 pp_lookahead_max:=0.60"
  "C2b|limit_mode:=friction $MU mpc_W:=20,20,2.5,2,2,1"
  "C3a|limit_mode:=friction $MU mpc_W:=20,20,10,2,2,1"
  "C3b|limit_mode:=friction $MU mpc_W:=20,20,10,2,2,1"
)
# ONLY="P1b C1b" 면 그 라벨만, TRIES (기본 2) = 판마다 측위 시도 횟수 (2026-11-21 사용자 "계속 돌려" — 남은 4 판을 3 번까지)
TRIES=${TRIES:-2}
for c in "${CASES[@]}"; do
  lab=${c%%|*}; args=${c#*|}
  if [ -n "$ONLY" ] && [[ " $ONLY " != *" $lab "* ]]; then continue; fi
  for try in $(seq 1 $TRIES); do
    bash "$HERE/run_square.sh" "$lab" $args; rc=$?
    echo "RESULT $lab try$try rc=$rc $(ls -td /home/smoc/capston/capston/bags/$(date +%F)/[0-9]* | head -1)"
    [ $rc -eq 0 ] && break
    if [ $rc -ge 4 ] || [ $try -eq $TRIES ]; then echo "ABORT at $lab (rc=$rc)"; exit $rc; fi
    sleep 5
  done
  sleep 3
done
echo "ALL_DONE"
