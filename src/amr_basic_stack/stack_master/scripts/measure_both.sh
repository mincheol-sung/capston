#!/usr/bin/env bash
# 한 거리에서 check_target 과 measure_c 를 같은 구간에 동시 실행.
#
# 왜 동시에: 두 스크립트가 보는 지표가 다르다.
#   check_target -> confidence 분포 / 클래스 / 클러스터 폭 안정성  (P_det(d) 곡선용)
#   measure_c    -> σ_r, σ_θ, bias, d_CL                          (가중치·gate 결정용)
# 따로 돌리면 서로 다른 30초를 보게 되어 두 CSV 의 행이 대응하지 않는다.
# 같은 프레임을 보게 하면 "conf 가 떨어진 그 구간에서 σ 가 어땠는지" 를 붙여서 볼 수 있다.
#
# 사용:
#   ./measure_both.sh 2.0 0 backpack           # 정면, 30초
#   ./measure_both.sh 2.0 -30 backpack         # 우 30°
#   ./measure_both.sh 2.0 -30 backpack 30
#   GT=2.034 CLASS=backpack ./measure_both.sh 2.0 0 backpack
#   ROI=1.2 ./measure_both.sh 5.0 -20 backpack # ROI 넓힘(원거리 off-axis 용)
#
# GT     = 줄자 실측 거리(m). measure_c 의 --gt-distance 로 넘어간다.
#          공칭 거리($1)와 다른 개념이라 자동 대입하지 않는다. 없으면 GT bias 는 빈 칸.
# CLASS  = YOLO 클래스 이름(예 backpack). 두 스크립트의 --class-name 으로 넘어간다.
#
# ROI 를 언제 키우나: ROI 는 공칭 위치 기준 탐색 반경(m)이라, 같은 각도 오차라도
#   호(弧) 길이가 거리에 비례해 커진다. 0.8 m 는 2 m 에서 ±22.9° 지만 5 m 에서는
#   ±9.2° 밖에 안 되어, 회전각을 눈대중하면 타깃이 ROI 를 벗어난다.
#   주의: ROI 를 바꾼 행은 check_target 의 cam_hit_pct(=P_det) 가 다른 행과
#   직접 비교되지 않는다. ROI 가 넓으면 "잡혔다"고 세는 프레임이 늘기 때문.
#   σ·bias 는 측정값끼리만 계산하므로 ROI 와 무관하다.
#
# 방위각을 재는 이유: association 게이팅은 두 센서의 방위 불확실성 차이에 민감하다.
# 0° 는 카메라가 가장 좋은 조건이라, 정면 데이터만으로는 σ_θ 모델의 근거가 안 된다.
set -u
D=${1:?"사용: ./measure_both.sh <거리m> [방위deg] [라벨] [초]   (환경변수 ROI 로 탐색반경 조정)"}
BEARING=${2:-0}
# 라벨은 필수. 기본값을 두면 타깃을 바꾼 뒤에도 옛 라벨로 조용히 기록되어
# 나중에 어느 행이 무슨 타깃이었는지 복구할 수 없다.
LABEL=${3:?"라벨을 지정하세요 (예: ./measure_both.sh 5.0 0 person)"}
DUR=${4:-30}
ROI=${ROI:-0.8}  # 어제 check_target 스윕과 동일하게 맞춤(measure_c 기본값 0.6 아님)
GT=${GT:-}       # 줄자 실측 GT 거리(m). 비면 measure_c 가 GT bias 를 빈 칸으로 둔다.
CLASS=${CLASS:-} # 카메라 클래스 필터
EXP=${EXP:-c2}   # 실험 식별자. 결과 파일이 실험 단위로 분리된다(옛 c1 과 안 섞이게)

cd "$(dirname "$0")"

if [ "$ROI" != "0.8" ]; then
  echo "※ ROI=$ROI (기본 0.8). 이 행의 cam_hit_pct 는 다른 행과 직접 비교 불가."
fi
if [ -z "$GT" ]; then
  echo "※ GT 미지정 — cam_bias_gt / lid_bias_gt 가 빈 칸이 됩니다."
  echo "  줄자로 재서  GT=2.034 ./measure_both.sh ...  형태로 넘기세요."
fi

# 인자 배열로 넘긴다. 빈 문자열을 그대로 넘기면 argparse 가 빈 값을 받아 깨진다.
GT_ARG=(); [ -n "$GT" ] && GT_ARG=(--gt-distance "$GT")
CLS_ARG=(); [ -n "$CLASS" ] && CLS_ARG=(--class-name "$CLASS")

# σ 는 파이프라인 설정의 함수다. 설정을 CSV 에 안 남기면 나중에 어느 행끼리 합칠 수
# 있는지 복구할 수 없다(예전에 yolo_conf 0.4/0.15 를 conf_min 으로 역추정해야 했다).
pget() { ros2 param get "$1" "$2" 2>/dev/null | sed 's/.*value is: //' || echo '?'; }
NOTE="cam_pos=$(pget /rgbd_detection_node camera_position_method)"
NOTE="$NOTE;roi_ratio=$(pget /rgbd_detection_node depth_roi_ratio)"
NOTE="$NOTE;min_px=$(pget /rgbd_detection_node min_valid_depth_pixels)"
NOTE="$NOTE;conf=$(pget /rgbd_detection_node yolo_conf)"
NOTE="$NOTE;lid_pos=$(pget /lidar_cluster_node cluster_position_method)"
echo "파이프라인: $NOTE"

CT_LOG=$(mktemp)
trap 'rm -f "$CT_LOG"' EXIT

# 두 스크립트에 같은 인자를 넘기는 게 핵심. 하나라도 어긋나면 두 CSV 가 대응하지 않는다.
python3 check_target.py --distance "$D" --bearing "$BEARING" --duration "$DUR" \
        --roi "$ROI" --label "$LABEL" --exp "$EXP" "${CLS_ARG[@]}" >"$CT_LOG" 2>&1 &
CT_PID=$!
python3 measure_c.py --distance "$D" --bearing "$BEARING" --duration "$DUR" \
        --roi "$ROI" --label "$LABEL" --exp "$EXP" \
        "${CLS_ARG[@]}" "${GT_ARG[@]}" --note "$NOTE"
wait "$CT_PID" || true

echo
echo "────────── check_target (같은 구간) ──────────"
cat "$CT_LOG"
