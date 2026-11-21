#!/bin/bash
# 병렬 실행기: runner.sh 경우파일 출력폴더 [작업자수=3] [공통 인자...]
#   경우파일 한 줄 = "라벨|launch 인자".  작업자 i 는 ROS_DOMAIN_ID 93+i.  결과 json 이 있으면 건너뜀, fault 판은 한 번 다시.
#   기록기는 REC 환경변수 (기본 slip_rec.py).
D=$(cd "$(dirname "$0")" && pwd)
CASES=$1; OUT=$2; NW=${3:-3}; shift 3; COMMON="$*"
export REC=${REC:-$D/slip_rec.py}
mkdir -p "$OUT"
mapfile -t L < <(grep -v '^\s*#' "$CASES" | grep -v '^\s*$')
work() {
  local w=$1 i=$1
  while [ $i -lt ${#L[@]} ]; do
    local lab=${L[$i]%%|*} args=${L[$i]#*|}
    if [ ! -s "$OUT/$lab.json" ]; then
      for try in 1 2; do
        $D/run_case.sh "$OUT" "$lab" $((93 + w)) $COMMON $args
        grep -q '"fault": ""' "$OUT/$lab.json" 2>/dev/null && break
        mv "$OUT/$lab.json" "$OUT/$lab.fault$try.json" 2>/dev/null
      done
    fi
    i=$((i + NW))
  done
}
for w in $(seq 0 $((NW - 1))); do work $w & done
wait
echo "RUNNER_DONE $(ls $OUT/*.json 2>/dev/null | grep -vc fault) / ${#L[@]}"
