#!/usr/bin/env bash
# 2026-11-28 pitwall 시뮬 확인: launch 한 번에 판 3 개 (원 → poly 프리셋 → 멈춘 판 버리고 accel 바꿔 새 판), MPC 값 무장 중 변경 → 정지 때 적용.
# 실차 판이 돌 때는 돌리지 말 것. 도메인 97 (실차 81 과 분리). 끝나면 sim_plant_torque 0 개를 확인한다.

OUT=${1:-/tmp/pitwall_sim}
mkdir -p "$OUT"
# 실차 판이 돌면 멈춘다 (2026-11-28 실차 판 c8_adpp_r1 도중 시뮬을 돌린 일 뒤에 넣음)
if pgrep -f '^/usr/bin/python3 .*/ak40_can_node' >/dev/null || pgrep -f 'ros2 launch amr_mpc real_all.launch.py' >/dev/null; then
  echo "실차 판 (ak40_can_node / real_all) 이 돌고 있다 — 시뮬을 돌리지 않는다"; exit 3
fi
source /opt/ros/jazzy/setup.bash
source /home/smoc/capston/capston/install/setup.bash
export ROS_DOMAIN_ID=97 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
PIT="/usr/bin/python3 /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools/pit.py"
log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/steps.txt"; }
arm() { ros2 topic pub --once -w 1 /mpc/arm std_msgs/msg/Bool "{data: $1}" >/dev/null 2>&1; log "arm $1"; }
wait_state() {   # $1 = 기다릴 상태 (finished 등), $2 = 최대 초
  local end=$(( $(date +%s) + $2 ))
  while [ "$(date +%s)" -lt "$end" ]; do
    s=$(timeout 6 python3 -c "
import json, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('wait_state'); got = []
n.create_subscription(String, '/figure8/status', lambda m: got.append(json.loads(m.data)['state']),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
while not got: rclpy.spin_once(n, timeout_sec=0.2)
print(got[-1])" 2>/dev/null)
    [ "$s" = "$1" ] && { log "state $s"; return 0; }
    sleep 1
  done
  log "state 기다림 시간 초과 ($1, 마지막 $s)"; return 1
}
: > "$OUT/steps.txt"
log "launch"
setsid ros2 launch amr_mpc torque_experiment.launch.py plant:=sim use_rviz:=false controller:=mpc limit_mode:=friction \
  origin_mode:=robot shape:=circles8 width:=2.0 speed:=0.5 accel:=0.8 laps:=1 start_delay:=1.0 hold_s:=2.0 \
  > "$OUT/launch.log" 2>&1 &
LPID=$!
sleep 14
setsid ros2 bag record --all-topics --storage mcap -o "$OUT/bag" > "$OUT/bag.log" 2>&1 &
BPID=$!
sleep 3
log "pit show (기동 직후)"; $PIT show 2>&1 | tee -a "$OUT/steps.txt"
log "거부 시험: circles8 speed 1.5 (부하 예산 넘음)"; $PIT set speed=1.5 2>&1 | tee -a "$OUT/steps.txt"
log "거부 시험: MPC 실행 중 못 바꾸는 값 Np"; $PIT set Np=50 2>&1 | tee -a "$OUT/steps.txt"
arm true
sleep 6
log "주행 중 변경: 기준 accel 0.6 (다음 판부터), MPC W·slip_fb_scale (다음 정지 때)"
$PIT set accel=0.6 W=[40,40,10,2,2,1] slip_fb_scale=0.5 2>&1 | tee -a "$OUT/steps.txt"
wait_state finished 60
arm false
sleep 2
log "pit show (판 1 끝, 정지 뒤)"; $PIT show 2>&1 | tee -a "$OUT/steps.txt"
log "프리셋 poly (정지 중 → 바로 다음 판)"; $PIT poly accel=1.0 square_v_max=1.0 2>&1 | tee -a "$OUT/steps.txt"
arm true
sleep 7
arm false
log "판 2 도중 멈춤 → accel 0.8 로 바꿈 (멈춘 판 버림)"; $PIT set accel=0.8 2>&1 | tee -a "$OUT/steps.txt"
sleep 1
arm true
wait_state finished 60
arm false
sleep 2
log "pit show (끝)"; $PIT show 2>&1 | tee -a "$OUT/steps.txt"
kill -INT -- -$BPID; sleep 3
kill -INT -- -$LPID; sleep 10
kill -TERM -- -$LPID 2>/dev/null; kill -TERM -- -$BPID 2>/dev/null; sleep 2
left=$(pgrep -f '^/usr/bin/python3 /home/smoc/capston/capston/install/amr_mpc/lib/amr_mpc/sim_plant_torque_node' | wc -l)
log "sim_plant_torque 남은 프로세스: $left"
