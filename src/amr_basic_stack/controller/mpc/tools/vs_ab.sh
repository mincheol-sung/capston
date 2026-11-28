#!/bin/bash
# MPC 속도 출처 실차 A/B (2026-11-27 사용자 요청 "꺾은선 속도 출처 바퀴랑 추정기 비교, 각 3 판"): 꺾은선 통과, 바퀴 / 추정기 3 판씩 번갈아.
#   공통: 지금 기본값 그대로 — MPC 롤러 항 0.88 N · I_z 0.03 (torque_mpc.yaml), 구동 방향 보정 3.1° · +0.8 (torque_plant.yaml drive), friction 상한.
#   w = velocity_source:=wheels  (몸체 앞·옆 속도 = 바퀴 기구학 T ω, 구동 방향 보정 각만큼 돌림 · 회전 속도는 추정기 = 자이로)
#   e = velocity_source 를 안 준다 = torque_mpc.yaml 기본 estimator (SLAM + IMU 추정기 속도)
#   조건은 drive_ab.sh · roller_ab.sh 와 같은 꺾은선 (모서리 통과 · 가속 1.5 · 최고 1.5).
#
#   bash tools/vs_ab.sh w <판>      바퀴 속도
#   bash tools/vs_ab.sh e <판>      추정기 속도 (기본)
#   bash tools/vs_ab.sh list        오늘 녹화된 판
#   … 끝에 dry 를 붙이면 실행할 명령만 보여 준다.  순서: w 1 → e 1 → w 2 → e 2 → w 3 → e 3 (모터 온도가 같은 판끼리 견주려고 번갈아)
#
# 사람이 터미널에서 직접 실행한다 (sudo 비밀번호, 측위 맞추기 배너 뒤 'a' 무장, 's'·스페이스 정지, Ctrl+C 로 끝).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../../.." && pwd)
KIND=$1; RUN=$2
[ "${!#}" = "dry" ] && DRY=1 || DRY=0
COMMON=(use_mpc:=true controller:=mpc shape:=polyline square_corner_mode:=pass square_v_max:=1.5 accel:=1.5 limit_mode:=friction)
case "$KIND" in
  list)
    ls -d "$WS/bags/$(date +%F)"/*_vs_* 2> /dev/null | sed 's|.*/|  |' || echo "  (없음)"
    exit 0 ;;
  w|e)
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2, 3)"; exit 2 ;; esac
    ARGS=("${COMMON[@]}")
    [ "$KIND" = w ] && ARGS+=(velocity_source:=wheels)
    ARGS+=("bag:=vs_${KIND}_r$RUN") ;;
  *) echo "w <판> | e <판> | list   (끝에 dry = 명령만 보기)"; exit 2 ;;
esac
if [ "$DRY" = 1 ]; then echo "amrall ${ARGS[*]}"; exit 0; fi
if [ ! -t 0 ]; then echo "사람이 터미널에서 직접 실행한다 — 입력을 넘겨 받는 실행은 거부한다"; exit 1; fi
source "$HERE/amr_aliases.sh"
echo "실행: amrall ${ARGS[*]}"
amrall "${ARGS[@]}"
