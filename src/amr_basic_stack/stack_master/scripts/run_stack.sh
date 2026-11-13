#!/usr/bin/env bash
# 센서/인지 스택 실행 — 모터는 mock.
# 핵심: PYTHONPATH 에 yolo venv 를 주입해야 YOLO 가 STUB(빈 검출)으로 안 떨어진다.
#       (설치 노드의 shebang 이 /usr/bin/python3 이고 ultralytics 는 venv 에만 있음)
# 주의: amr_system.launch.py 는 mock_mode 기본값이 false(실모터) → 반드시 명시.
set -e
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
export PYTHONPATH="/home/smoc/venvs/yolo/lib/python3.12/site-packages:$PYTHONPATH"

exec ros2 launch stack_master amr_system.launch.py \
  mock_mode:=true \
  use_joystick:=false \
  use_slam:=false \
  use_rviz:=false \
  "$@"
