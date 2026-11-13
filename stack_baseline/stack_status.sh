#!/usr/bin/env bash
# git status 대용: 기준 시점(manifest) 대비 src/amr_basic_stack 에서 바뀐 파일을 보여 준다.
#   사용:  bash stack_baseline/stack_status.sh            # 최신 기준과 비교
#          bash stack_baseline/stack_status.sh 2026-11-10 # 특정 기준과 비교
#   새 기준 만들기:  bash stack_baseline/stack_status.sh --snapshot
# 원본 되돌리기(파일 하나):
#   tar -xzf stack_baseline/<날짜>_code.tar.gz -C /tmp/restore <경로>   후 비교·복사
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
BASE="$WS/stack_baseline"
cd "$WS/src"

list_files() {
  find amr_basic_stack -type f \
    -not -path "*/__pycache__/*" -not -path "*/.pytest_cache/*" \
    -not -path "amr_basic_stack/log/*" -not -path "amr_basic_stack/output/*" \
    | LC_ALL=C sort
}

list_code() {
  list_files | grep -E '\.(py|cpp|hpp|h|c|yaml|yml|lua|xml|txt|msg|srv|action|cfg|md|sh|rviz|urdf|xacro|json|toml)$|/CMakeLists\.txt$|/setup\.(py|cfg)$' \
    | grep -vE '/docs/.*\.(json|csv)$'
}

if [[ "${1:-}" == "--snapshot" ]]; then
  tag="$(date +%Y-%m-%d_%H%M)"
  list_files | tr '\n' '\0' | xargs -0 sha256sum > "$BASE/${tag}_manifest.sha256"
  list_code | tar -czf "$BASE/${tag}_code.tar.gz" -T -
  echo "기준 저장: $BASE/${tag}_manifest.sha256, ${tag}_code.tar.gz"
  exit 0
fi

if [[ -n "${1:-}" ]]; then
  manifest="$(ls "$BASE"/"$1"*_manifest.sha256 | tail -1)"
else
  manifest="$(ls "$BASE"/*_manifest.sha256 | tail -1)"
fi
echo "기준: $(basename "$manifest")"

tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
awk '{ $1=""; sub(/^ +/, ""); print }' "$manifest" | LC_ALL=C sort > "$tmp/base"
list_files > "$tmp/now"

echo "== 수정됨";  sha256sum -c "$manifest" 2>/dev/null | grep -E ': FAILED$' | sed 's/: FAILED$//' || true
echo "== 새 파일"; LC_ALL=C comm -13 "$tmp/base" "$tmp/now"
echo "== 삭제됨";  LC_ALL=C comm -23 "$tmp/base" "$tmp/now"
