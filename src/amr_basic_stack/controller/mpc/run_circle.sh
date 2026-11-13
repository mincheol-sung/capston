#!/usr/bin/env bash
# 원 선회 실차 추종 — 로봇 NUC 에서.  드라이버(base.launch.py)가 떠 있어야 한다.
#   ./run_circle.sh                       R=1 m, 0.15 m/s, 2바퀴 (부하 17 %)
#   ./run_circle.sh 2.0 0.5 2             R=2 m, 0.5 m/s, 2바퀴 (부하 44 %)
#   ./run_circle.sh 3.0 0.7 2             R=3 m, 0.7 m/s (부하 62 %)
#   ./run_circle.sh 4.0 1.0 2 0.9         R=4 m, 1.0 m/s, load_budget 0.9 (부하 87 %)
# 명령 어디서든 Ctrl+C → 정지 + CSV 저장 + 요약.
#
# 2026-09-04: mpc_circle_node(자체 MPC 루프) → circle.launch.py(fake_reference_node +
#   mpc_tracking_node) 로 바꿨다. 기준열 → 보간 → MPC 경로를 운용과 똑같이 타므로
#   여기서 나온 수치가 운용에 그대로 읽힌다. 예산 검사는 config/plant.yaml 을 직접 읽는다.
#
# 선행조건:
#   [ ] 드라이버 로그에 max_wheel_rad_per_sec 확인 (install 의 base.yaml 이 소스와 같은지)
#   [ ] /cmd_vel 발행자가 하나 — 조이스틱을 껐는지 (use_joystick:=false)
#   [ ] 원 지름 2R + 0.5 m 빈 공간, 시작 방향(로봇 +x)으로 원이 왼쪽으로 열림 (반시계)
#   [ ] /odom 50 Hz 들어옴:  ros2 topic hz /odom
set -eo pipefail                      # -u 는 ROS setup.bash 와 충돌 (AMENT_TRACE_SETUP_FILES 미정의)
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash 2>/dev/null || true
cd "$(dirname "$0")"

R=${1:-1.0}; V=${2:-0.15}; LAPS=${3:-2}; BUDGET=${4:-}
ODOM=${ODOM:-/odom}                    # 자기시험 기본. model 의 θ 가 /odom 으로 식별됐다
RAMP=${RAMP:-1.0}                      # 가감속 램프 [s]. 호장 = v·(T − RAMP) 이라 바퀴 수에 더한다

# 부하 예산 검사 — V_LIMIT·K_GEOM·load_budget 을 plant.yaml 에서 직접 읽는다 (단일 출처)
python3 - "$R" "$V" "$BUDGET" <<'PY' || exit 1
import math, sys, yaml
R, V = float(sys.argv[1]), float(sys.argv[2])
p = yaml.safe_load(open("config/plant.yaml"))
g, l = p["geometry"], p["limits"]
VL = g["wheel_radius_m"] * l["max_wheel_rad_per_sec"]
k  = (g["wheel_base_m"] + g["robot_width_m"]) / 2.0
B  = float(sys.argv[3]) if sys.argv[3] else l["load_budget"]
om = V / R
load = (V + k * om) / VL
vmax_b, vmax_1 = B * VL / (1 + k / R), VL / (1 + k / R)
print(f"[INFO] 지름 {2*R:.1f} m, ω {om:.2f} rad/s ({math.degrees(om):.0f} °/s), "
      f"명목 부하 {load*100:.0f} % (V_LIMIT {VL:.2f}, K_GEOM {k:.4f}, 예산 {B:.2f})")
if load > 0.95:
    print(f"[ERROR] 부하 {load*100:.0f} % > 95 % — 보정 예산 없음 (1.0 m/s R=1 이 99.8 % 로 발산했다). "
          f"이 반지름의 최대 속도는 {vmax_1:.2f} m/s (예산 {B:.2f} 기준 {vmax_b:.2f})"); sys.exit(1)
if load > B:
    print(f"[ERROR] 부하 {load*100:.0f} % > 예산 {B*100:.0f} %. 속도 ≤ {vmax_b:.2f} m/s 로, "
          f"또는 4번째 인자로 예산을 올려라 (예: 0.9)"); sys.exit(1)
PY

DUR=$(python3 -c "import math; print(round($LAPS*2*math.pi*$R/$V + $RAMP, 2))")
# ROS 파라미터는 '30' 을 INTEGER 로 파싱해 DOUBLE 선언과 충돌한다 → 전부 소수점 붙여 넘긴다
f() { python3 -c "print(float($1))"; }
R=$(f $R); V=$(f $V); DUR=$(f $DUR); RAMP=$(f $RAMP)

ros2 topic list | grep -q "^${ODOM}$" || { echo "[ERROR] $ODOM 없음 — 드라이버(base.launch.py)를 먼저"; exit 1; }
N=$(ros2 topic info /cmd_vel 2>/dev/null | sed -n 's/Publisher count: //p')
[ "${N:-0}" -gt 0 ] && echo "[WARN] /cmd_vel 발행자가 이미 $N 개 — 조이스틱을 껐는지 확인해라 (명령이 섞인다)"

echo "[INFO] R=$R m, v=$V m/s, ${LAPS}바퀴 = ${DUR}s, odom=$ODOM"
exec ros2 launch amr_mpc circle.launch.py radius:=$R speed:=$V duration:=$DUR ramp_s:=$RAMP \
     odom_topic:=$ODOM log_prefix:=mpc_circle_R${R}_v${V}
