#!/bin/bash
# 구동 방향 보정 MPC 실차 A/B (2026-11-26 사용자 결정 "MPC만 비교"): 꺾은선 통과 (roller_ab.sh 와 같은 조건), 끔 / 켬 3 판씩 번갈아.
#   끔 = 지금 MPC 기본 (롤러 항 0.88 N · I_z 0.03 켬, 구동 방향 보정 없음).  켬 = + drive_yaw_deg:=2.4 drive_slip_coef:=-2.5
#   보정: 바퀴가 굴러가는 축이 몸체 축보다 2.4° 반시계 (+ 선회 중 구심가속 1 m/s² 당 −2.5°) — 11-23 측정 (torque_plant.yaml drive 절 주석).
#     MPC 에서는 바퀴 힘 사상 (force_map) 을 2.4° 돌리고, 바퀴 속도로 구한 몸체 속도도 같은 각으로 돌린다.
#   왜: 2 m 직선 끝 왼쪽 밀림 5~8 cm = 2·sin(2.4° + 헤딩 오차) (analysis/torque_mpc_real/2026-11-25_roller_ab/RESULTS_KO.md).
#   ★ PP·어댑티브 PP 는 이 보정을 안 받으므로, PP 와 견주는 비교에는 끔 판을 쓴다 (11-23 쓰는 법: 로봇 보정은 세 제어기 공통).
#
#   bash tools/drive_ab.sh off <판>     보정 없음 (drive_yaw_deg:=0 drive_slip_coef:=0 — 2026-11-26 부터 yaml 기본이 B 라서 직접 0)
#   bash tools/drive_ab.sh on  <판>     + 구동 방향 보정 (11-23 값 2.4° · −2.5)
#   bash tools/drive_ab.sh a   <판>     후보 A: 2.4° · 선회 계수 0     (2026-11-26 다시 맞춤 — 선회 계수만 뺌)
#   bash tools/drive_ab.sh b   <판>     후보 B: 3.1° · 선회 계수 +0.8  (2026-11-26 오늘 12 판 엔코더 vs SLAM 맞춤값 그대로 — 같은 날 사용자 결정으로 yaml 기본값)
#     근거: analysis/torque_mpc_real/2026-11-26_drive_ab/kin_calib.py — 직선 어긋남 +3.1° (11-23 의 2.4° 보다 큼), 선회 계수 +0.8 ± 0.2 (11-23 −2.5 와 부호 반대).
#           지금 설정 (2.4 · −2.5) 의 남는 오차: 왼쪽 선회 +2.0°, 오른쪽 선회 −0.9° → 왼쪽 꺾은 직선은 왼쪽, 오른쪽 꺾은 직선은 오른쪽으로 남는 경향 (segment_errors.py)
#   bash tools/drive_ab.sh list         오늘 녹화된 판
#   … 끝에 dry 를 붙이면 실행할 명령만 보여 준다.  순서: off 1 → on 1 → off 2 → on 2 → off 3 → on 3
#
# 사람이 터미널에서 직접 실행한다 (sudo 비밀번호, 측위 맞추기 배너 뒤 'a' 무장, 's'·스페이스 정지, Ctrl+C 로 끝).
# 분석: analysis/torque_mpc_real/2026-11-26_drive_ab (read_bags → summary · straights · slip_phase · plot_paths_dy)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../../.." && pwd)
KIND=$1; RUN=$2
[ "${!#}" = "dry" ] && DRY=1 || DRY=0
COMMON=(use_mpc:=true controller:=mpc shape:=polyline square_corner_mode:=pass square_v_max:=1.5 accel:=1.5 limit_mode:=friction velocity_source:=wheels)
case "$KIND" in
  list)
    ls -d "$WS/bags/$(date +%F)"/*_dy_* 2> /dev/null | sed 's|.*/|  |' || echo "  (없음)"
    exit 0 ;;
  off|on|a|b)
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2, 3)"; exit 2 ;; esac
    ARGS=("${COMMON[@]}")
    case "$KIND" in
      off) ARGS+=(drive_yaw_deg:=0 drive_slip_coef:=0) ;;      # 2026-11-26 부터 yaml 기본이 3.1° · +0.8 — 끔은 직접 0 을 준다
      on) ARGS+=(drive_yaw_deg:=2.4 drive_slip_coef:=-2.5) ;;
      a)  ARGS+=(drive_yaw_deg:=2.4 drive_slip_coef:=0.0) ;;
      b)  ARGS+=(drive_yaw_deg:=3.1 drive_slip_coef:=0.8) ;;
    esac
    ARGS+=("bag:=dy_${KIND}_r$RUN") ;;
  *) echo "off <판> | on <판> | a <판> | b <판> | list   (끝에 dry = 명령만 보기)"; exit 2 ;;
esac
if [ "$DRY" = 1 ]; then echo "amrall ${ARGS[*]}"; exit 0; fi
if [ ! -t 0 ]; then echo "사람이 터미널에서 직접 실행한다 — 입력을 넘겨 받는 실행은 거부한다"; exit 1; fi
source "$HERE/amr_aliases.sh"
echo "실행: amrall ${ARGS[*]}"
amrall "${ARGS[@]}"
