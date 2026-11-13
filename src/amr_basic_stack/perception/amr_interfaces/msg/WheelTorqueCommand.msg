# 바퀴 4개 토크 명령 (토크 MPC 스택, 구조도 v3).
#
# 같은 타입을 경로의 세 구간에 쓴다. 의미는 토픽 이름으로 구분한다.
#   torque_mpc_node  -> /torque_mpc/cmd_raw        : MPC 순토크 τ* (바닥에 전달할 토크, 마찰 보상 전)
#   torque_output    -> /torque_output/cmd_raw     : τ* + τ_fric(ω) + 안전 클램프 (모터 출력축 토크)
#   command_gate     -> /wheel_torque_cmd          : 무장·freshness 를 통과한 최종 명령 → ak40_can_node
#
# 바퀴 순서는 기존 스택과 같다: [FL, FR, RL, RR] (roboclaw_mecanum_node twist_to_wheels).
# 부호는 바퀴 각속도와 같은 규약이다. vx > 0 으로 전진할 때 네 바퀴 모두 ω > 0 이고,
# 그 방향으로 미는 토크가 + 다. 모터 장착 방향 반전은 ak40_can_node 가 처리한다.

std_msgs/Header header
# header.stamp: 이 명령을 계산할 때 쓴 상태 추정값의 시각.
#   경로를 거쳐도 바꾸지 않는다. ak40_can_node 가 CAN 송신 시각과 비교해 종단 지연을 잰다.

uint8 MODE_TORQUE = 0
# torque[] 를 그대로 낸다. (AK40 MIT 모드: kp = 0, kd = 0, t_ff = torque)
uint8 MODE_DAMPING = 1
# 정지. torque[] 는 무시한다. (MIT 모드: kp = 0, v = 0, kd > 0, kd 는 ak40_can_node 설정값)
# 토크 0 은 정지가 아니라 관성 주행이므로, 게이트의 정지·무장 해제·timeout 은 이 모드를 쓴다.

uint8 mode

float64[4] torque
# 바퀴(모터 출력축) 토크 [N·m]. MODE_TORQUE 일 때만 쓰인다.
