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
#   bash tools/floor_tests_w4_t7.sh rot <모터 토크> <ccw|cw> <판>   제자리 회전 단계 토크 (2026-11-25 이 값으로 실차 54 회):
#                                                           바퀴당 0.06 | 0.08 | 0.10 N·m 회전 꼴 [−,+,−,+], 90° 돌면 끊음, 토크·방향마다 2 판
#   bash tools/floor_tests_w4_t7.sh lat <모터 토크> <left|right> <판>   옆걸음 단계 토크 (2026-11-25 사용자 승인 값):
#                                                           바퀴당 0.08 | 0.10 | 0.12 N·m 옆 꼴 [−,+,+,−], 0.3 m 가면 끊음, 토크·방향마다 2 판 이상
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
#   rot   controller:=step step_dir:=<ccw|cw> step_angle_deg:=90 step_distance:=0.5 step_inertia_ff:=false friction_comp:=false step_tau:=<토크> step_max_time:=<표>
#         시간 한도 = 11-15·11-16 회전 bag 으로 맞춘 값 (I_eff 0.120, 바닥 회전 저항 0.46 N·m) 의 90° 도달 예상 × 1.3 + 0.15 s.
#         0.06 은 정지 마찰 근처라 안 돌 수 있다 (11-15 에 0.05 는 0°) — 그러면 시간 한도에 끊긴다. 벗어남 0.5 m 는 추정기 거짓 속도로 일찍 끊기지 않게.
#   lat   controller:=step step_dir:=<left|right> step_distance:=0.3 step_inertia_ff:=false friction_comp:=false step_tau:=<토크> step_max_time:=<표>
#         시간 한도 = 옆 저항 5.5 N (11-15 lat1 맞춤 5.0~5.4 위쪽) 일 때 0.3 m 도달 예상 × 1.3 + 0.15 s. 0.08 은 저항 근처라 천천히 가거나 안 갈 수 있다.
#         옆으로 0.3 m + 제동 거리 — 왼쪽·오른쪽을 번갈아 돌려 제자리 근처로 돌아오게 한다.
# 녹화: bags/<오늘>/<시분초>_<이름>.  분석: tools/floor_wheel_tests.py cv … · step … · rot … · lat … · quick <bag>
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
    echo "  rot  0.06 0.08 0.10 N·m × ccw·cw         토크·방향마다 2 판 (12 판), 낮은 토크부터 (11-25 실차)"
    echo "  lat  0.08 0.10 0.12 N·m × left·right     토크·방향마다 2 판 이상, 낮은 토크부터 (11-25 승인)"
    echo "오늘 녹화된 판:"
    ls -d "$WS/bags/$(date +%F)"/*_{fricmodel_check,w4_cv_,t7_step_,rot_,lat_}* 2> /dev/null | sed 's|.*/|  |' || echo "  (없음)"
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
  rot)
    TAU=$2; DIR=$3; RUN=$4
    case "$TAU" in
      0.06) TMAX=1.90 ;; 0.08) TMAX=1.30 ;; 0.10) TMAX=1.00 ;;
      *) echo "바퀴당 모터 토크는 0.06 | 0.08 | 0.10 (11-25 실차 값)"; exit 2 ;;
    esac
    case "$DIR" in ccw|cw) ;; *) echo "방향은 ccw (반시계) | cw (시계)"; exit 2 ;; esac
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2 …)"; exit 2 ;; esac
    ARGS=(use_mpc:=true controller:=step "step_dir:=$DIR" "step_tau:=$TAU" step_angle_deg:=90 step_distance:=0.5 "step_max_time:=$TMAX"
          step_inertia_ff:=false friction_comp:=false "step_log_dir:=$WS/analysis/torque_mpc_real/$(date +%F)_rot" "bag:=rot_${DIR}_$(tag "$TAU")_r$RUN") ;;
  lat)
    TAU=$2; DIR=$3; RUN=$4
    case "$TAU" in
      0.08) TMAX=3.70 ;; 0.10) TMAX=1.90 ;; 0.12) TMAX=1.45 ;;
      *) echo "바퀴당 모터 토크는 0.08 | 0.10 | 0.12 (11-25 승인된 값)"; exit 2 ;;
    esac
    case "$DIR" in left|right) ;; *) echo "방향은 left (왼쪽) | right (오른쪽)"; exit 2 ;; esac
    case "$RUN" in ''|*[!0-9]*) echo "판 번호를 준다 (1, 2 …)"; exit 2 ;; esac
    ARGS=(use_mpc:=true controller:=step "step_dir:=$DIR" "step_tau:=$TAU" step_distance:=0.3 "step_max_time:=$TMAX"
          step_inertia_ff:=false friction_comp:=false "step_log_dir:=$WS/analysis/torque_mpc_real/$(date +%F)_lat" "bag:=lat_${DIR}_$(tag "$TAU")_r$RUN") ;;
  *) echo "list | check | cv <속도> <판> | step <모터 토크> <판> | rot <모터 토크> <ccw|cw> <판> | lat <모터 토크> <left|right> <판>   (끝에 dry = 명령만 보기)"; exit 2 ;;
esac
if [ "$DRY" = 1 ]; then echo "amrall ${ARGS[*]}"; exit 0; fi
if [ ! -t 0 ]; then echo "사람이 터미널에서 직접 실행한다 — 입력을 넘겨 받는 실행은 거부한다"; exit 1; fi
source "$HERE/amr_aliases.sh"
echo "실행: amrall ${ARGS[*]}"
amrall "${ARGS[@]}"
