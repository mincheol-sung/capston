#!/bin/bash
# 바닥 시험 묶음 (2026-11-24 사용자 승인 값) — 바퀴 회전자 식별 지시서 W4 (바닥 구름 손실) · 회의 후속 지시서 T7 (마찰계수).
# 승인된 값으로만 amrall (real_all.launch.py) 을 띄운다 — 긴 인자를 손으로 치다 틀리지 않게. 표에 없는 값은 거부한다.
#
#   bash tools/floor_tests_w4_t7.sh list                    계획표와 오늘 녹화된 판
#   bash tools/floor_tests_w4_t7.sh check                   새 스위치 (상한 옮김·마찰 모델) 첫 확인: MPC 로 직선 0.5 m, 0.2 m/s
#   bash tools/floor_tests_w4_t7.sh cv <속도> <판>           시험 A 등속 직진: 고정 PP, 보상 끔, 직선 1.1 m, 가감속 1.5 m/s²
#                                                           속도 0.2 | 0.4 | 0.6 | 0.8  (바퀴 5·10·15·20 rad/s), 속도마다 2 판
#   bash tools/floor_tests_w4_t7.sh step <모터 토크> <판>    시험 B 단계 토크: 정지에서 네 바퀴 같은 모터 토크, 0.3 m 가면 끊음
#                                                           토크 0.08 | 0.12 | 0.16 | 0.20 | 0.24 | 0.28 N·m, 토크마다 2 판
#   … 끝에 dry 를 붙이면 실행할 명령만 보여 준다
#
# 사람이 터미널에서 직접 실행한다 (sudo 비밀번호, 'a' 무장, 's'·스페이스 정지, Ctrl+C 로 끝). 쓸 수 있는 직선은 1.3 m — 출발점을 벽에서 멀리 둔다.
# 순서: check → cv (어느 속도부터든) → step 은 낮은 토크부터 한 단계씩, 단계마다 얼마나 가서 섰는지 확인하고 올린다.
#   step 의 "시간 한도" 는 거리 판정이 늦어도 그 시간에 토크를 끊는 안전장치다 (예상 시간 + 0.15 s).
#   0.24·0.28 N·m 는 0.45 s 만에 1.3 m/s 가 된다 — 정지 키를 누를 준비를 하고 돌린다.
# 조건 (모두 torque_output 구동계 마찰 보상 끔):
#   cv    controller:=pp friction_comp:=false shape:=line seg_distance:=1.1 accel:=1.5 speed:=<속도>
#   step  controller:=step step_dir:=fwd step_distance:=0.3 step_inertia_ff:=false friction_comp:=false step_tau:=<토크> step_max_time:=<표>
#         (관성 몫을 더하지 않으므로 준 값이 곧 모터 토크다 — amrstep 은 보상을 끌 수 없어 쓰지 않는다)
# 녹화: bags/<오늘>/<시분초>_<이름>.  분석: tools/floor_wheel_tests.py cv … · step … · quick <bag>
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../../../.." && pwd)
KIND=$1
[ "${!#}" = "dry" ] && DRY=1 || DRY=0
tag() { echo "$1" | tr -d '.'; }          # 0.4 → 04, 0.08 → 008

case "$KIND" in
  list)
    echo "계획 (2026-11-24 승인):"
    echo "  check                                   1 판"
    echo "  cv   0.2 0.4 0.6 0.8 m/s                 속도마다 2 판 (8 판)"
    echo "  step 0.08 0.12 0.16 0.20 0.24 0.28 N·m   토크마다 2 판 (12 판), 낮은 토크부터"
    echo "오늘 녹화된 판:"
    ls -d "$WS/bags/$(date +%F)"/*_{fricmodel_check,w4_cv_,t7_step_}* 2> /dev/null | sed 's|.*/|  |' || echo "  (없음)"
    exit 0 ;;
  check)
    ARGS=(use_mpc:=true controller:=mpc shape:=line seg_distance:=0.5 speed:=0.2 limit_mode:=friction friction_comp:=false
          mpc_cap_friction_shift:=true use_fric_model:=true disturbance:=true bag:=fricmodel_check) ;;
  cv)
    V=$2; RUN=$3
    case "$V" in 0.2|0.4|0.6|0.8) ;; *) echo "속도는 0.2 | 0.4 | 0.6 | 0.8 (승인된 값)"; exit 2 ;; esac
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2 …)"; exit 2 ;; esac
    ARGS=(use_mpc:=true controller:=pp friction_comp:=false shape:=line seg_distance:=1.1 "speed:=$V" accel:=1.5 "bag:=w4_cv_$(tag "$V")_r$RUN") ;;
  step)
    TAU=$2; RUN=$3
    case "$TAU" in
      0.08) TMAX=1.05 ;; 0.12) TMAX=0.80 ;; 0.16) TMAX=0.70 ;; 0.20) TMAX=0.65 ;; 0.24) TMAX=0.60 ;; 0.28) TMAX=0.60 ;;
      *) echo "모터 토크는 0.08 | 0.12 | 0.16 | 0.20 | 0.24 | 0.28 (승인된 값)"; exit 2 ;;
    esac
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2 …)"; exit 2 ;; esac
    ARGS=(use_mpc:=true controller:=step step_dir:=fwd "step_tau:=$TAU" step_distance:=0.3 "step_max_time:=$TMAX" step_inertia_ff:=false
          friction_comp:=false "step_log_dir:=$WS/analysis/torque_mpc_real/$(date +%F)_step" "bag:=t7_step_$(tag "$TAU")_r$RUN") ;;
  *) echo "list | check | cv <속도> <판> | step <모터 토크> <판>   (끝에 dry = 명령만 보기)"; exit 2 ;;
esac
if [ "$DRY" = 1 ]; then echo "amrall ${ARGS[*]}"; exit 0; fi
if [ ! -t 0 ]; then echo "사람이 터미널에서 직접 실행한다 — 입력을 넘겨 받는 실행은 거부한다"; exit 1; fi
source "$HERE/amr_aliases.sh"
echo "실행: amrall ${ARGS[*]}"
amrall "${ARGS[@]}"
