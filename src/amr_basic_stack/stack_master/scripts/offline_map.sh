#!/usr/bin/env bash
# capture.sh 로 담은 매핑 bag 을 오프라인으로 다시 돌려 맵을 만든다.
# ---------------------------------------------------------------
# 왜 실시간 결과를 안 쓰고 이걸 쓰는가 (2026-09-16 실측)
#
#   같은 bag(map_hq_20260916_001124, 319 s), 같은 mapping_hq_2d.lua:
#
#                       흐릿/점유   자유면적    조각   pbstream
#     실시간 매핑          4.021    110.3 m²    195   13.25 MB
#     오프라인 재매핑      1.986     89.7 m²    125   17.09 MB
#
#   렌더러는 동일하다 — finish_map.py 도 이 스크립트도 같은
#   cartographer_pbstream_to_ros_map -resolution 0.05 을 쓰고 PGM->PNG 는
#   Image.convert('L') 무손실이다. 차이는 순수하게 실시간 처리 쪽이다.
#
#   실시간 런에서 cartographer_node 가 35 회 멈췄다(/tracked_pose 최대 공백
#   1383.6 ms, 합 46.6 s = 전체의 14.6 %). 데이터는 오히려 적은데(291 s vs
#   319 s) 자유공간이 23 % 더 넓게 퍼졌다 — pose 가 서로 안 맞아 밖으로
#   번진 것이다. "정지 -> 손실" 의 기전 자체는 아직 확인 못 했다.
#
# lua 비교도 같은 방법으로 했다 (같은 bag, lua 만 교체):
#     mapping_2d     흐릿/점유 2.197  조각 142  constraint 40014  108.8 s
#     mapping_hq_2d  흐릿/점유 1.986  조각 125  constraint 57064  195.9 s  ← 채택
#   constraint 가 42.6 % 늘면서 평균 score 도 0.756 -> 0.764 로 같이 올랐다.
#
# 하는 일:
#   1. bag 의 토픽을 cartographer 가 기대하는 이름으로 리맵한 사본을 만든다
#      (/scan_front_carto->/scan_1, /scan_rear_carto->/scan_2, /camera/imu->/imu).
#      cartographer_offline_node 는 bag 을 직접 읽으므로 --ros-args -r 이 안 먹는다.
#   2. cartographer_offline_node 로 재매핑 -> .pbstream
#   3. cartographer_pbstream_to_ros_map 으로 렌더 -> .pgm -> .png, yaml 경로 보정
#   4. planner/global_planner/maps/<이름>/ 에 설치하고 global_planner 재빌드
#
# 사용:
#   ./offline_map.sh <bag> <맵이름>                 기본 lua = mapping_hq_2d.lua
#   ./offline_map.sh <bag> <맵이름> mapping_2d.lua  lua 지정
#   ./offline_map.sh <bag> <맵이름> --force         기존 맵 덮어쓰기
#   ./offline_map.sh <bag> <맵이름> --no-build      colcon build 생략
#
# 끝나면 측위는 이렇게:
#   ros2 launch stack_master autonomy.launch.py map:=<맵이름> ...
#
# 주의: global_planner 로 경로를 뽑으려면 생성된 yaml 에 start / goal 키를
#       직접 추가해야 한다. 예)  start: [-4.45, 4.45, 0]
# ---------------------------------------------------------------
set -eo pipefail

BAG=""; NAME=""; LUA="mapping_hq_2d.lua"; FORCE=0; BUILD=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force)    FORCE=1; shift ;;
    --no-build) BUILD=0; shift ;;
    -h|--help)  awk 'NR>1 && /^#/ {sub(/^# ?/,""); print; next} NR>1 {exit}' "$0"; exit 0 ;;
    *.lua)      LUA="$1"; shift ;;
    *)          if   [[ -z "$BAG"  ]]; then BAG="$1"
                elif [[ -z "$NAME" ]]; then NAME="$1"
                else echo "모르는 인자: $1  (--help)"; exit 2; fi; shift ;;
  esac
done
[[ -z "$BAG" || -z "$NAME" ]] && { echo "사용: $0 <bag> <맵이름> [lua] [--force] [--no-build]"; exit 2; }
[[ -d "$BAG" ]] || { echo "bag 디렉터리가 없습니다: $BAG"; exit 1; }

WS="$HOME/2026amr_ws"
CFG="$WS/src/amr_basic_stack/perception/amr_perception_bringup/config"
MAPS="$WS/src/amr_basic_stack/planner/global_planner/maps"
OUT="$MAPS/$NAME"
RES=0.05                      # lua 의 submaps.grid_options_2d.resolution 과 맞출 것
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

[[ -f "$CFG/$LUA" ]] || { echo "lua 가 없습니다: $CFG/$LUA"; exit 1; }
if [[ -e "$OUT" && $FORCE -eq 0 ]]; then
  echo "이미 있습니다: $OUT"
  echo "덮어쓰려면 --force, 아니면 다른 이름을 쓰세요."
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 가 소싱되지 않았습니다:  source /opt/ros/jazzy/setup.bash && source ~/2026amr_ws/install/setup.bash"
  exit 1
fi

echo "bag        : $BAG"
echo "맵 이름    : $NAME  -> $OUT"
echo "lua        : $LUA"
echo "해상도     : $RES m/cell"
echo

# ── 1. 토픽 리맵 사본 ────────────────────────────────────────────────────
echo "1/4  토픽 리맵 사본 생성..."
python3 - "$BAG" "$TMP/bag" <<'PY'
import sys
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
src, dst = sys.argv[1], sys.argv[2]
MAP = {'/scan_front_carto': '/scan_1', '/scan_rear_carto': '/scan_2',
       '/camera/imu': '/imu', '/odom': '/odom',
       '/tf': '/tf', '/tf_static': '/tf_static'}
r = SequentialReader()
r.open(StorageOptions(uri=src, storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
missing = [s for s in ('/scan_front_carto', '/scan_rear_carto', '/camera/imu',
                       '/odom', '/tf_static') if s not in types]
if missing:
    sys.exit(f"  bag 에 없는 필수 토픽: {', '.join(missing)}\n"
             "  매핑 중에 capture.sh 로 담은 bag 이 맞는지 확인하세요.")
w = SequentialWriter()
w.open(StorageOptions(uri=dst, storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
for s, d in MAP.items():
    if s in types:
        w.create_topic(TopicMetadata(id=0, name=d, type=types[s],
                                     serialization_format='cdr'))
n = 0
while r.has_next():
    t, data, ts = r.read_next()
    if t in MAP:
        w.write(MAP[t], data, ts); n += 1
del w
print(f"     {n} msgs")
PY

# ── 2. 오프라인 재매핑 ───────────────────────────────────────────────────
echo "2/4  cartographer_offline_node 재매핑 (bag 길이의 0.3~0.7 배 걸립니다)..."
# set -e 아래에서 실패하면 즉시 죽어 아래 진단이 안 찍힌다. 종료코드를 직접 받는다.
rc=0
ros2 run cartographer_ros cartographer_offline_node \
  -configuration_directory "$CFG" \
  -configuration_basenames "$LUA" \
  -bag_filenames "$TMP/bag" \
  -save_state_filename "$TMP/$NAME.pbstream" > "$TMP/offline.log" 2>&1 || rc=$?
if [[ $rc -ne 0 || ! -s "$TMP/$NAME.pbstream" ]]; then
  echo "  실패 (exit $rc). 로그 마지막 30 줄:"
  sed 's/\x1b\[[0-9;]*[A-Za-z]//g' "$TMP/offline.log" | tail -30 | sed 's/^/    /'
  cp "$TMP/offline.log" "/tmp/offline_${NAME}.log" 2>/dev/null \
    && echo "  전체 로그: /tmp/offline_${NAME}.log"
  exit 1
fi
# constraint score 히스토그램의 마지막 요약 줄이 품질 지표다.
echo -n "     constraint: "
grep -oE "Count: [0-9]+  Min: [0-9.]+  Max: [0-9.]+  Mean: [0-9.]+" "$TMP/offline.log" | tail -1

# ── 3. 렌더 ──────────────────────────────────────────────────────────────
echo "3/4  점유격자 렌더링..."
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename "$TMP/$NAME.pbstream" \
  -map_filestem "$TMP/$NAME" -resolution "$RES" > "$TMP/render.log" 2>&1
[[ -f "$TMP/$NAME.pgm" ]] || { echo "  렌더 실패:"; tail -10 "$TMP/render.log"; exit 1; }

# PGM -> PNG (이 스택의 다른 맵과 형식을 맞춘다) + yaml 의 절대경로를 파일명으로
python3 - "$TMP/$NAME" <<'PY'
import sys
from PIL import Image
stem = sys.argv[1]
Image.open(f'{stem}.pgm').convert('L').save(f'{stem}.png')
name = stem.rsplit('/', 1)[-1]
with open(f'{stem}.yaml') as f:
    lines = f.readlines()
with open(f'{stem}.yaml', 'w') as f:
    for ln in lines:
        f.write(f'image: {name}.png\n' if ln.startswith('image:') else ln)
PY

# ── 4. 설치 ──────────────────────────────────────────────────────────────
echo "4/4  설치 -> $OUT"
mkdir -p "$OUT"
cp "$TMP/$NAME.pbstream" "$TMP/$NAME.png" "$TMP/$NAME.yaml" "$OUT/"

python3 - "$OUT/$NAME.png" <<'PY'
import sys, numpy as np
from PIL import Image
from scipy import ndimage
a = np.array(Image.open(sys.argv[1]).convert('L'))
occ = a <= 89; free = a >= 205; unk = (a == 128)
blur = (~occ) & (~free) & (~unk)
n = ndimage.label(occ, structure=np.ones((3, 3)))[1]
dt = ndimage.distance_transform_edt(occ)
print(f"     점유 {occ.sum()}  자유 {free.sum()*0.0025:.1f} m²  "
      f"흐릿/점유 {blur.sum()/occ.sum():.3f}  "
      f"벽두께p90 {np.percentile(dt[occ],90)*2*0.05*1000:.0f} mm  조각 {n}")
PY

if [[ $BUILD -eq 1 ]]; then
  echo
  echo "global_planner 재빌드..."
  (cd "$WS" && colcon build --symlink-install --packages-select global_planner) \
    || echo "재빌드 실패 — 직접: cd $WS && colcon build --packages-select global_planner"
fi

cat <<MSG

완료
  $OUT/$NAME.pbstream   (측위 모드 입력)
  $OUT/$NAME.png
  $OUT/$NAME.yaml

측위:
  ros2 launch stack_master autonomy.launch.py map:=$NAME \\
       use_voxel_3d:=true use_amcl_bootstrap:=true use_amcl_wiggle:=true use_rviz:=false

주의: global_planner 로 경로를 뽑으려면 위 yaml 에 start / goal 키를 직접
      추가해야 합니다. 예)  start: [-4.45, 4.45, 0]
MSG
