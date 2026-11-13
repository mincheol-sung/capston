#!/usr/bin/env bash
# 스택 완전 정리.
# 노드가 /opt/ros 경로에도 있어 install 경로 패턴만으로는 안 잡힌다.
# 일부가 살아남으면 중복 실행되고, realsense 가 장치를 쥔 채 재실행하면
# "Device or resource busy" 로 카메라가 죽는다(복구는 initial_reset:=true 재기동).
pkill -f "ros2 launch stack_master" 2>/dev/null

for _ in $(seq 1 25); do
  pgrep -f "realsense2_camera_node" >/dev/null 2>&1 || break
  sleep 0.2
done

pkill -9 -f "realsense2_camera_node"      2>/dev/null
pkill -9 -f "rplidar_ros/lib"             2>/dev/null
pkill -9 -f "robot_state_publisher"       2>/dev/null
pkill -9 -f "robot_localization/ekf_node" 2>/dev/null
pkill -9 -f "2026amr_ws/install"          2>/dev/null

# realsense 는 종료 후 USB 핸들 반납에 시간이 걸린다
for _ in $(seq 1 15); do
  pgrep -f "realsense2_camera_node" >/dev/null 2>&1 || break
  sleep 0.2
done
sleep 2

n=$(pgrep -cf "realsense2_camera_node|rplidar_ros/lib|robot_state_publisher|2026amr_ws/install" 2>/dev/null || echo 0)
echo "잔여 프로세스: $n"
exit 0
