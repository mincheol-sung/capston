# 바퀴 4개 상태 (토크 MPC 스택, 구조도 v3). ak40_can_node 가 /wheel_states 로 발행한다.
# 시뮬에서는 시뮬 플랜트가 같은 토픽·타입으로 대신 발행한다.
#
# 바퀴 순서: [FL, FR, RL, RR]. 부호는 WheelTorqueCommand 와 같다(전진 시 ω > 0).
# 모터 장착 방향 반전은 발행하는 쪽에서 이미 풀어 놓은 값이다.

uint8 FL = 0
uint8 FR = 1
uint8 RL = 2
uint8 RR = 3

std_msgs/Header header
# header.stamp: 이 상태를 받은 시각(실차: CAN 응답 수신 시각).

float64[4] position
# 바퀴 각도 [rad], 모터 출력축 기준. 드라이버 범위를 넘으면 펼친(unwrap) 누적값.

float64[4] velocity
# 바퀴 각속도 [rad/s], 모터 출력축 기준. 슬립률·MTTE·비교군 A 의 PI 가 쓴다.

float64[4] torque
# 드라이버가 상전류로 추정한 출력축 토크 [N·m]. 실제 바퀴 토크와의 차이는 실측 대상(To do 1절).

float32[4] temperature
# 모터 온도 [°C]. 드라이버가 주지 않으면 NaN.

uint8[4] error
# 드라이버 에러 코드. 0 = 정상. fault_monitor 가 본다.

bool[4] fresh
# 이번 주기에 그 바퀴의 응답을 새로 받았으면 true. false 면 위 값은 직전 값이다.
