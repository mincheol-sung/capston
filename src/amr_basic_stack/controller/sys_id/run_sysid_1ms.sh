#!/usr/bin/env bash
# WP3 재식별 캠페인 — 0.3 → 1.0 m/s.  로봇 NUC 에서, ROS 소싱 후 실행.
# 2단계로 나눈다. A 단계 결과(도달률·PWM 피크·tau 편차)를 보고 B 를 돌릴지 정한다.
#
#   ./run_sysid_1ms.sh A      0.3 / 0.5 / 0.7 m/s,   1.5 / 2.5 / 3.5 rad/s   (바퀴 ≤ 17.5)
#   ./run_sysid_1ms.sh B      0.85 / 1.0 m/s,        4.5 rad/s               (바퀴 ≤ 25)
#   ./run_sysid_1ms.sh fit <csv> [<csv> ...]
#
# 선행조건 (전부 확인하고 시작):
#   [ ] base.yaml max_wheel_rad_per_sec: 26.0 으로 드라이버 재시작됨 (로그에 "max_wheel_rad_per_sec=26")
#   [ ] 직진 방향으로 앞뒤 각 2.5 m 이상 빈 공간 (1.0 m/s × 2 s = 2 m, ± 교대)
#   [ ] 회전 반경 안에 장애물 없음 (4.5 rad/s = 258 °/s)
#   [ ] 배터리 16 V 이상 (로그 vbat 로 사후 확인)
#   [ ] 비상정지 손에. Ctrl+C 로도 정지명령 + CSV 저장됨
#   [ ] 지상. 공중 로그는 /odom 이 가짜
set -euo pipefail
cd "$(dirname "$0")"
WHEEL_MAX=26
case "${1:-}" in
  A) python3 wp2_step_logger.py --amps-lin 0.3,0.5,0.7  --amps-ang 1.5,2.5,3.5 --ramp 0.3 --repeats 3 --t-step 2.0 --wheel-max $WHEEL_MAX ;;
  B) python3 wp2_step_logger.py --amps-lin 0.85,1.0     --amps-ang 4.5         --ramp 0.3 --repeats 3 --t-step 2.0 --wheel-max $WHEEL_MAX ;;
  fit) shift; python3 fit_fopdt.py "$@" ;;
  *) sed -n 2,20p "$0"; exit 1 ;;
esac
