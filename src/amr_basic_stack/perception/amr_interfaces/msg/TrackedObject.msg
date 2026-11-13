# Object Tracker 출력: ID/Position/Velocity
std_msgs/Header                   header
uint32                            track_id
int32                             class_id
string                            class_name

# pose.pose.position 과 velocity.twist.linear 의 (x, y) 만 채운다.
# orientation 은 항등 — 물체의 방위는 관측하지 않는다.
# covariance 는 6x6 row-major 이고 (x, y) 블록 [0],[1],[6],[7] 만 채운다.
# 프레임은 header.frame_id (기본 odom). base_link 가 아닌 이유는
# object_tracker/core/tracker.py 독스트링 참고.
geometry_msgs/PoseWithCovariance  pose
geometry_msgs/TwistWithCovariance velocity

# 위치-속도 교차 공분산 P_pv. row-major [x*vx, x*vy, y*vx, y*vy], 단위 m^2/s.
# pose.covariance 와 velocity.covariance 는 서로 다른 6x6 이라 이 블록을 담지
# 못한다. 상태 [x, y, vx, vy] 의 4x4 공분산은
#     P = [[P_pp,   P_pv],
#          [P_pv^T, P_vv]]
# 이고, tau 초 뒤 위치를 등속으로 외삽한 공분산은
#     Sigma(tau) = P_pp + tau (P_pv + P_pv^T) + tau^2 P_vv
# 이다. 이 항을 빼면 Sigma 를 과소평가한다. sigma_a=1.0, 12Hz 에서 1.0s coast
# 한 LOST 트랙은 tau=1s 에서 표준편차가 약 0.75 배로 나온다.
float64[4]                        position_velocity_covariance

# 운동 상태. 속도의 chi^2 검정 d^2 = v^T P_vv^-1 v ~ chi^2_2 로 판별한다.
#   UNKNOWN  아직 판별하지 않았다는 뜻이지 "정지해 있다"가 아니다. 소비자는
#            STATIC 과 같이 취급하면 안 된다 — 움직이는 물체를 놓치는 쪽이
#            위험한 오류다. UNKNOWN 이 나오는 경우는 둘이다.
#              - TENTATIVE 트랙. 속도 추정이 안 섰다. publish_tentative 가
#                false(기본)면 /tracks 에 나오지 않는다.
#              - CONFIRMED 직후. 판정 카운터는 CONFIRMED 가 된 프레임부터
#                센다. 처음 publish 된 뒤 최소 (n_dynamic - 1) 프레임(DYNAMIC
#                으로 갈 때), (n_static - 1) 프레임(STATIC 으로 갈 때) 동안
#                UNKNOWN 이다. 기본값이면 2, 4 프레임. chi^2 가 불감대
#                [4.61, 9.21] 에 머물면 더 길어진다.
uint8 MOTION_UNKNOWN = 0
uint8 MOTION_STATIC  = 1
uint8 MOTION_DYNAMIC = 2
uint8                             motion_state

# (motion_state == MOTION_DYNAMIC) 의 축약. UNKNOWN 도 false 가 되므로
# 세 상태를 구분해야 하는 소비자는 motion_state 를 볼 것.
bool                              is_dynamic

# 트랙 생성 후 경과 시간 [s].
float32                           age
