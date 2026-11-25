#!/bin/bash
# 사용: run_all.sh [cases 파일 (기본 cases.txt)]
D=$(cd "$(dirname "$0")" && pwd); C=${1:-$D/cases.txt}
while IFS='|' read -r lab args; do [ -z "$lab" ] && continue; "$D/run.sh" "$lab" $args; done < "$C"
