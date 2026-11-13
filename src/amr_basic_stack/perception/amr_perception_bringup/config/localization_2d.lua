-- ============================================================================
-- localization_2d.lua — 저장된 맵(.pbstream) 위에서 순수 측위만 수행.
--
-- mapping_2d.lua 를 베이스로 하고 달라지는 것만 덮어쓴다. 여기 없는 항목은
-- 매핑과 완전히 동일하다(센서 토픽, IMU 사용, 상관 스캔매칭, TF 소유권 등).
--
-- 실행 시 반드시 -load_state_filename 으로 .pbstream 을 넘겨야 한다.
-- 초기 위치는 set_initial_pose_node 가 RViz "2D Pose Estimate" 로 받아
-- FinishTrajectory + StartTrajectory(use_initial_pose) 로 재시작시킨다.
--
-- ※ 이 모드의 /map_carto 는 "새로 만든 서브맵 3 개" 가 아니다.
--   pure_localization_trimmer 는 **새 궤적의** 서브맵만 3 개로 제한하고, frozen
--   궤적 0 의 서브맵(이 맵은 47 개)은 그대로 남는다. occupancy_grid_node 는
--   /submap_list 의 모든 서브맵을 그리므로 결과는 사실상 전체 맵이다.
--   2026-09-15 실측: /map_carto 330x351~374x380, 자유셀 92 m² 로 map_server 의
--   /map(78 m²)보다 넓었다.
--   그래도 planner 와 traversability 는 /map(저장된 .png/.yaml)을 쓴다 —
--   /map_carto 는 크기와 origin 이 1.14 Hz 로 계속 바뀌어(85 개 메시지에 크기
--   29 가지, origin 32 가지) 누적 격자의 기준으로 쓸 수 없고, /map 의 벽 중
--   14~33 % 가 그 시점 서브맵에서는 자유로 나온다.
-- ============================================================================

include "mapping_2d.lua"

-- ── cartographer_ros 옵션 ──
-- RViz /initialpose 는 base_link 방향으로 주어진다. optical 프레임을 tracking_frame
-- 으로 쓰면 새 trajectory 의 로컬 좌표축이 base_link 와 약 90도 어긋나므로,
-- IMU와 같은 위치이면서 축이 로봇과 정렬된 gyro_frame 을 측위에 사용한다.
options.tracking_frame = "camera_gyro_frame"
options.pose_publish_period_sec = 1e-2         -- 매핑: 5e-3
options.trajectory_publish_period_sec = 30e-3

-- 측위는 실시간 응답이 중요하고 서브맵 생성이 적어 여유가 있다.
MAP_BUILDER.num_background_threads = 6         -- 매핑: 4

-- ── 프론트엔드 ──
-- ceres_scan_matcher.rotation_weight 는 스캔 정합 결과가 추측항법(IMU+휠)
-- 예측의 '회전'에서 벗어나는 것을 얼마나 벌하는지다. 비용함수에서
--
--   J = (w_occ/sqrt(N))·sum(1 - M(T·p))^2 + w_t·|t - t_pred|^2 + w_r·(th - th_pred)^2
--
-- 의 w_r 이고, th_pred 는 PoseExtrapolator(IMU 자이로 기반) 예측이다.
-- 낮출수록 라이다 쪽으로 자유롭게 돌고, 높일수록 자이로 예측에 묶인다.
--
-- ★ 2026-09-15 offline A/B 로 정한 값이다 (bag mpc_chain_20260914_155111 을
--   cartographer_node + bag play 로 재처리, frozen pbstream 로드, 수렴 후
--   35~90 s 구간 비교):
--
--     w_r    yawRMS  |wz|<.05  |wz|>.2    hit   끝점-벽    비고
--     실차40  2.538    1.566    3.361   0.839   70.6 mm  초기 pose 있음
--       40   2.795    1.602    3.650   0.351  348.1 mm  ★ 끝까지 못 붙음
--       10   2.677    1.666    3.553   0.841   69.5 mm  실차 재현
--        1   2.503    1.481    3.317   0.832   68.2 mm
--      0.1   2.524    1.508    3.347   0.827   69.6 mm
--
--   읽히는 것 둘:
--   (a) 회전 진동은 이 값으로 안 잡힌다. 400 배를 바꿔도 |wz|>0.2 구간
--       yawRMS 가 3.65 -> 3.32 도로 9 % 다. 실차 기준선 3.361 도와도 같다.
--       회전 중 진동의 원인은 cartographer 설정 밖에 있다.
--   (b) 40 은 초기 pose 없이 재수렴하지 못한다. 구간별 hit 이 내내
--       0.29~0.40 이었다(10 이하는 30 s 안에 붙는다). 회전을 자이로에 강하게
--       묶으면 초기 자세 오차를 풀 자유도가 없다. 실차에서 40 이 도는 것은
--       set_initial_pose_node 가 정확한 초기 pose 를 주기 때문이고,
--       측위를 잃으면 스스로 복구할 수 없다는 뜻이다.
--
--   그래서 10 을 쓴다. 1 이나 0.1 로 더 풀어도 개선이 없으므로(위 표),
--   자이로 구속을 최대한 남기면서 재수렴 능력만 확보하는 지점이다.
--   D455 자이로는 믿을 만하고 메카넘 휠 yaw 는 못 믿는다는 판단은 그대로다.
--
-- (참고) IFAC_SMOC 의 localization_2d.lua 는 0.1 을 쓴다. 이전에 이 주석은
-- 그 값이 real_time_correlative_scan_matcher.rotation_delta_cost_weight 와의
-- 혼동이라고 적었는데 사실이 아니다 — 그쪽은 ceres_scan_matcher.rotation_weight
-- 를 명시적으로 0.1 로 두고, 근거를 "known track 에서는 라이다를 믿는다" 로
-- 적어 두었다. 플랫폼이 다를 뿐 혼동은 아니었다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.

-- ── num_accumulated_range_data: 측위도 2 (매핑과 같음) ──
-- 앞/뒤 라이다는 각각 base 기준 방위 190° 만 본다. 2 = 두 장을 합쳐 360° 로
-- scan match 한다. 1 이면 반쪽 시야 하나로 match 해서 yaw 가 잡히지 않는다.
--
-- 2026-09-28 replay A/B. smoc_lab(accum 2 지도) frozen 로드, cartographer_node
-- + bag play, 초기 pose 는 bag 의 실측 /tracked_pose, use_pose_extrapolator =
-- false 로 scan match pose 만 기록. 설정당 2 회 반복 평균, accum 1 vs 2:
--
--     bag(09-23)      벽거리 p50   정지 xy std  정지 yaw std  0.5 s 상대운동 RMS
--     113151          81 vs 41 mm   4.5 vs 1.2    0.73 vs 0.02°   74 vs 20 mm
--     135325         241 vs 132      393 vs 16     5.8 vs 0.17     160 vs 13
--     135606(≤300 s)  62 vs 50       9.8 vs 1.3    1.0 vs 0.03      46 vs 28
--
--   벽거리 = 원본 스캔을 지도에 투영한 스캔별 평균 거리(1 m clip). 반쪽 스캔마다
--   pose 를 따로 맞추는 accum 1 에 유리한 지표인데도 2 가 이긴다. 정지 떨림과
--   상대운동(vs /odometry/filtered)은 두 설정에 공정한 지표다.
--   135325 에서 accum 1 은 정지 중에도 1.77 m 이탈했다. 반복 간 편차는 작고,
--   accum 2 replay 는 실차 기록과 일치한다(벽거리 p50 131 vs 132 mm).
--
-- ※ 이전(2026-09-18) 에는 bag 014121/154912 A/B 로 "측위는 1" 이라 적었다.
--   그 결론은 재현되지 않는다. bag·스크립트가 없어 원인은 확인 못 했다.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

-- ── motion filter: 측위는 매핑보다 노드를 드물게 남긴다 ──
--
-- mapping_2d.lua 는 0.5 s / 0.1 m / 0.2° 로 cartographer 기본값
-- (5 s / 0.2 m / 1°) 보다 시간 10 배·거리 2 배·각도 5 배 조여 놨다. 그 근거는
-- "매핑은 노드를 촘촘히 남길수록 맵이 좋다" 이고 매핑에서는 맞다. 측위에서는
-- 아니다 — 여기서 필요한 것은 촘촘한 맵이 아니라 안정된 pose 다.
--
-- 왜 각도만 푸는가 (2026-09-11 bag 실측, mpc_chain_20260911_0124·0149):
--   map->odom 샘플간 이동 중앙값이 회전에 강하게 의존한다.
--     |wz| 0.00~0.05 rad/s : 0.27~0.33 mm
--     |wz| 0.05~0.15       : 5.04~6.11
--     |wz| 0.15~0.30       : 7.78~8.30
--     |wz| 0.30~           : 13.42~19.16 mm      <- 41~71 배
--   기전: 각도 문턱 0.2° 는 |wz| > A×scan_hz = 0.035 rad/s(2°/s) 부터 이미
--   포화된다. 즉 아주 느린 회전에도 노드가 스캔율(10 Hz)로 생기고,
--   optimize_every_n_nodes = 20 이므로 최적화가 2 s 마다 돈다. 최적화마다
--   local_to_global 이 재배치되어 /tracked_pose 가 계단으로 튄다
--   (계단 34/34·28/28 개가 전부 /submap_list 갱신 0.2 s 안에 있었다).
--
-- 0.2° -> 3.0° 의 계산된 효과 (스캔 10 Hz, optimize_every_n_nodes 20):
--   포화 시작 |wz|      0.035 -> 0.524 rad/s
--   최적화 간격  선회 0.3 m/s + 0.3 rad/s : 2.00 -> 3.49 s
--                제자리 회전 0.5 rad/s     : 2.00 -> 2.09 s
--                |wz| 4.49 rad/s(관측 최대): 2.00 -> 2.00 s   <- 개선 없음
--   스캔율이 노드 생성을 10 Hz 로 캡하므로 최적화 간격의 하한이 20/10 = 2 s 다.
--   각도 필터는 그 하한 아래로 못 내려가고 하한에서 벗어나게만 할 수 있다.
--   그래서 고속 회전 구간은 이 값으로 해결되지 않는다 — 그쪽은
--   optimize_every_n_nodes 를 올려 빈도와 크기를 맞바꾸는 문제다.
--
-- 대가: 스캔매칭 횟수는 줄지 않는다. motion filter 는 InsertIntoSubmap 의
--   첫 줄(local_trajectory_builder_2d.cc:285)에서 돌고, ScanMatch 는 그보다
--   앞(같은 파일 237행)에서 이미 끝나 있다. 걸러지는 것은 서브맵 삽입과
--   pose graph 노드 생성뿐이다. 실제 대가는 두 가지다.
--     * 활성 서브맵에 삽입되는 스캔이 성겨져, 이후 정합의 기준 격자가 엉성해진다.
--     * 노드가 줄어 궤적간 constraint 후보가 줄어든다. 새 서브맵이 생길 때마다
--       trimmer 가 가장 오래된 것을 버리고 거기 붙은 constraint 도 함께
--       사라지므로(2026-09-11 실측: 대형 감소 5 회가 전부 TRIM 0.4~0.7 s 후),
--       노드가 적으면 회복이 느려진다.
--   회전 추종 자체는 use_pose_extrapolator = true 와 use_imu_data = true 가
--   맡는다. 나빠지면 1.0° 로 되돌리고, 부족하면 5.0° 로 올린다.
--   한 번에 하나씩 — cartographer 기본값 1.0° 는 관측 구간에서 개선이 0 이라
--   중간값으로 쓸 의미가 없다.
--
-- 시간·거리 문턱은 건드리지 않는다. 실측에서 병진 구간의 map->odom 떨림은
-- 0.27~0.33 mm 로 이미 1 셀(50 mm)의 1/150 이다.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.0)

-- ── 백엔드: 순수 측위 ──
-- 새로 만드는 서브맵을 3개만 유지한다. 이게 없으면 측위 중에도 맵이 계속
-- 자라서 메모리와 CPU 가 무한히 는다.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 측위는 매핑(100)보다 자주 최적화해야 pose 가 빨리 따라온다. 다만 최적화가
-- 돌 때마다 궤적 전체가 조금씩 재배치되므로, 너무 잦으면 그 재배치가 그대로
-- pose 떨림으로 보인다. 공식 순수측위 레퍼런스와 같은 20 을 쓴다.
-- (이전 값 3 은 레퍼런스보다 7배 잦았다.)
POSE_GRAPH.optimize_every_n_nodes = 20

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- ★ 얼어붙은 맵과의 궤적 간 매칭 정책
--
-- 순수 측위에서 새 궤적(1번)이 얼어붙은 궤적(0번)에 처음 붙는 유일한 통로가
-- GLOBAL 탐색이다. cartographer 는 노드-서브맵 쌍마다
--
--     같은 궤적이거나, 두 궤적이 최근에 이어진 적이 있으면  -> LOCAL 탐색
--     그렇지 않으면                                          -> GLOBAL 탐색
--                                                    (샘플러가 뽑았을 때만)
--
-- 로 판정한다. 갓 시작한 궤적 1 은 궤적 0 과 아직 한 번도 이어진 적이 없으므로
-- LOCAL 분기에 들어갈 수 없다. 따라서 global_sampling_ratio = 0 이면 샘플러가
-- 영원히 안 뽑히고, 정합을 시도조차 안 한 채 새로 만든 서브맵 3개 위에서만
-- 추측항법으로 흘러간다. RViz "2D Pose Estimate" 로 초기 위치를 정확히 찍어도
-- 마찬가지다. 초기 pose 는 좌표계 정렬만 해줄 뿐 궤적을 이어주지는 않는다.
--
-- 그래서 기본값 0.003 을 쓴다. 처음 한 번 붙고 나면
-- global_constraint_search_after_n_seconds(아래) 동안은 LOCAL 탐색만 쓰므로,
-- 비싼 전체 탐색은 부팅 직후에만 돈다.
--
-- 순간이동(엉뚱한 통로에 붙음)은 이 값을 0 으로 막는 게 아니라
-- constraint_builder.global_localization_min_score 로 막는다. mapping_2d.lua
-- 에서 0.75 로 올려뒀다(기본값 0.6). 그래도 튀면 그 값을 더 올릴 것.
--
-- 초기 락이 느리면 이 값을 키운다(0.01~0.05). 대신 부팅 직후 CPU 를 더 쓴다.
POSE_GRAPH.global_sampling_ratio = 0.003

-- 마지막 궤적 간 구속 이후 LOCAL 탐색을 계속 쓰는 유예 시간. 크게 잡으면 한 번
-- 붙은 뒤에는 GLOBAL 탐색이 사실상 안 돌아서, 주행 중 순간이동 위험이 없다.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1000.

return options
