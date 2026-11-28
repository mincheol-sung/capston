#!/bin/bash
# 롤러 구름 저항 MPC 실차 A/B (2026-11-25 사용자 결정): 꺾은선 통과 (지난 비교 sq2p_mpcw 와 같은 조건), 끔 / 켬 3 판씩 번갈아.
#   끔 = 롤러 항 없는 예전 MPC.  켬 = mpc_roller_rr:=0.88 (롤러 하나당 구름 저항, 실차 옆 시험 4.99 N) + mpc_inertia_z:=0.03 (실차 회전 시험 I_eff − 바퀴 몫)
#   2026-11-26 부터 켬이 yaml 기본이다 — 그래서 끔은 mpc_roller_rr:=0.0 mpc_inertia_z:=0.06 을 직접 준다 (11-26 A/B 와 같은 두 설정)
#   근거: analysis/torque_mpc_real/2026-11-25_rot · 2026-11-25_lat, 안 analysis/torque_mpc_sim/2026-11-25_roller_plan/PLAN_KO.md
#
#   bash tools/roller_ab.sh off <판>     롤러 항 끈 예전 MPC (mpc_roller_rr:=0.0 mpc_inertia_z:=0.06)
#   bash tools/roller_ab.sh on  <판>     롤러 저항 + 회전 관성
#   bash tools/roller_ab.sh list         오늘 녹화된 판
#   … 끝에 dry 를 붙이면 실행할 명령만 보여 준다.  순서 예: off 1 → on 1 → off 2 → on 2 → off 3 → on 3 (모터 온도가 같은 판끼리 견주려고 번갈아)
#
# 사람이 터미널에서 직접 실행한다 (sudo 비밀번호, 측위 맞추기 배너 뒤 'a' 무장, 's'·스페이스 정지, Ctrl+C 로 끝).
# 공통: controller:=mpc shape:=polyline square_corner_mode:=pass square_v_max:=1.5 accel:=1.5 limit_mode:=friction velocity_source:=wheels
# 분석: 지난 비교와 같은 analysis/torque_mpc_real/2026-11-25_polyline_pass_real/summary.py 꼴 (SLAM 기준 위치·경로·헤딩·회전·슬립)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../../.." && pwd)
KIND=$1; RUN=$2
[ "${!#}" = "dry" ] && DRY=1 || DRY=0
# 2026-11-26: 구동 방향 보정 yaml 기본이 3.1° · +0.8 이 됐다 — 이 A/B (11-26 17:41~) 와 같은 조건을 지키려고 0 을 직접 준다
COMMON=(use_mpc:=true controller:=mpc shape:=polyline square_corner_mode:=pass square_v_max:=1.5 accel:=1.5 limit_mode:=friction velocity_source:=wheels drive_yaw_deg:=0 drive_slip_coef:=0)
case "$KIND" in
  list)
    ls -d "$WS/bags/$(date +%F)"/*_rr_* 2> /dev/null | sed 's|.*/|  |' || echo "  (없음)"
    exit 0 ;;
  off|on)
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2, 3)"; exit 2 ;; esac
    ARGS=("${COMMON[@]}")
    if [ "$KIND" = on ]; then ARGS+=(mpc_roller_rr:=0.88 mpc_inertia_z:=0.03); else ARGS+=(mpc_roller_rr:=0.0 mpc_inertia_z:=0.06); fi
    ARGS+=("bag:=rr_${KIND}_r$RUN") ;;
  *) echo "off <판> | on <판> | list   (끝에 dry = 명령만 보기)"; exit 2 ;;
esac
if [ "$DRY" = 1 ]; then echo "amrall ${ARGS[*]}"; exit 0; fi
if [ ! -t 0 ]; then echo "사람이 터미널에서 직접 실행한다 — 입력을 넘겨 받는 실행은 거부한다"; exit 1; fi
source "$HERE/amr_aliases.sh"
echo "실행: amrall ${ARGS[*]}"
amrall "${ARGS[@]}"
