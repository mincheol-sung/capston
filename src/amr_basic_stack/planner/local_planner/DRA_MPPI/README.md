# DRA-MPPI Local Planner

현재 YAML은 **1차 8배 MC + 최종 직접 적분** 혼합 방식이다.
`dynamic_risk.mode: monte_carlo`, `dynamic_risk.final_mode: gaussian_disk`로
Guide K=500, 밀도992, guide 표본1024~8192를 유지하고 최종 대표·Stop 및
terminal alignment는 Gaussian 충돌 원 직접 적분으로 검사한다.
최종 MC 표본 설정 2048~32768은 MC 복귀 시 사용하며 직접 적분에는 사용하지 않는다.
`final_mode: inherit`이면 기존처럼 1차 방법을 따르고, `mode: off`는 둘 다 끈다.
standard 전략의 weighted representative와 Stop도 같은 final_mode를 적용한다.
[방식별 비교](docs/risk_integrator_comparison_2026-11-04/RESULTS.txt)와
[기존 8배 MC 기록](docs/mc_budget_theta_2026-10-23/results.md)을 참고한다.
[혼합 방식 적용 및 ROS 검증](docs/hybrid_risk_2026-11-04/RESULTS.txt)에 현재 변경을 기록했다.

`dynamic_risk.max_obstacles: 20`은 기존 class/freshness 필터를 통과한 예측 객체의
상한이다(mode 개수가 아님). 21개 이상이면 일부를 버리지 않고 해당 주기를
`prediction_budget_hold`로 처리하여 기존 hold 명령을 발행한다. 입력 수가 다시
20개 이하가 되면 새 주기에서 재계획한다. `0`은 상한 비활성화다.
상한은 계산량 정책이며 움직이는 장애물에 대한 정지 안전성을 보장하지 않는다.
설정 변경은 노드 재시작 시 적용한다.

## 선택된 관측 대상에 대한 비용 (기본 OFF)

`config/dra_mppi.yaml`의 `observation_cost.enabled: true`로 켠다.
실행 중에는 `ros2 param set /dra_mppi_node observation_cost.enabled true`
또는 `false`로 전환할 수 있다(네임스페이스 사용 시 노드 이름 수정).
전환 시 warm start와 저장된 대상을 초기화하며, 켠 뒤 새 대상 입력이 필요하다.
나머지 관측 설정은 YAML 수정 후 노드 재시작으로 적용한다.

선택된 대상 하나를 `observation_cost.target_topic`(기본
`/dra_mppi/observation_target`)에 `geometry_msgs/PointStamped`로 발행한다.
`header.frame_id`는 `planning_frame`과 같아야 하고 좌표는 미리 해당 프레임으로
변환해야 한다. `header.stamp`는 대상 위치의 실제 ROS 시각이며 0/미래/오래된
시각은 거부한다. 원본 시각과 수신 후 경과시간 모두
`observation_cost.target_timeout`(기본 0.5초) 이내여야 한다.
OFF, 대상 없음, 잘못된 입력, 만료 시 관측 비용만 0으로 처리한다.
LiDAR에서 대상을 선정하거나 여러 대상의 우선순위를 정하는 기능은 아직 포함하지 않는다.

각 예측 시점에서 카메라 위치 `c_k = p_k + R(yaw_k) * camera_offset`,
대상 위치 `q`, 카메라 정면 `yaw_k + camera_yaw`로 다음 비용을 계산한다.

```text
e_k = wrap(atan2(q_y - c_ky, q_x - c_kx) - yaw_k - camera_yaw)
J_obs = weight / N * sum(max(0, abs(e_k) - half_fov)^2)
```

즉, 시야 안이면 0, 시야 밖이면 벗어난 각도의 제곱을 부과하는 **구간별 2차 비용**이다.
대상을 무조건 화면 중앙으로 향하게 하지는 않는다. `weight`는 기본 3.0이며,
`half_fov_deg`(기본 35도)는 여유각을 제외한 유효 수평 반시야각이다.
`camera_offset: [x, y]`(m)와 `camera_yaw_deg`는 차체 기준 카메라 외부 파라미터다.
이 기본값들은 예시이므로 실제 카메라 설치와 시야에 맞춰 설정해야 한다.

standard/guide의 샘플 평가와 최종 대표·Stop 재평가에 동일하게 더하며,
진단의 raw/weighted 비용에 `observation`으로 기록한다. 기존 경로 heading 비용과
함께 최적화하므로 관측을 보장하지는 않는다. 가림, 거리, 검출 성공률이나 실제
정보 이득을 계산하는 항은 아니다. 최종 목표 자세 정렬 단계는 목표 yaw를 따른다.
ROS 입력은 매 주기 갱신되는 한 점을 horizon 동안 고정해서 쓰며,
Python `compute(..., observation_target=...)`는 같은 planning frame의 `(2,)` 점
또는 rollout 시각 `dt, 2*dt, ..., N*dt`에 맞춘 `(N, 2)` 위치열도 받는다.

## Gap strategy

`mppi.strategy: gap`으로 설정하고 노드를 재시작하면 전·후방 LiDAR의 구역별
도달점·비용을 최적화하고 그 결과로 seed와 샘플 배분을 정한다.
기존 `standard`, `guide_conditioned`도 선택할 수
있으며 현재 YAML은 `gap`을 선택한다. `autonomy.launch.py`가
기존과 같이 이 YAML을 읽으므로 launch 파일 수정은 필요 없다.

처리 순서는 다음과 같다.

1. `/scan_front_carto`, `/scan_rear_carto`를 최근 odometry와 센서 TF로 시간 보정한다.
   각 scan의 원본 시각·수신 후 경과시간은 0.25초 이내, 두 scan의 시각 차이는
   0.05초 이내여야 한다. odometry로 보간할 수 없는 beam은 사용하지 않는다.
2. 로봇 중심 8×8m, 4cm grid에 scan을 합친다. 유효한 traversability 지도의 free
   정보로 미관측 영역을 보완하고, 어느 입력에서든 장애물이면 유지한다.
   나머지 unknown은 막힌 영역으로 취급한다.
3. 로봇 반경과 5cm 여유를 고려하여 360방향의 이동 가능 거리를 계산한다
   (1도 간격, 최대 2m, 거리 간격 2cm). 0.5m 이상 열린 연속 방향을 구역으로 삼고,
   raw LiDAR 거리 급변 지점에서 추가 분할한다. 인접 거리 차이가
   `max(0.4m, 가까운 거리×0.12)` 이상이며 양쪽 3개 beam에서 유지되어야 한다.
   실제 로봇 이동 가능 거리도 주변에서 0.25m 이상 달라져야 경계로 인정한다.
   경계 간격은 최소 20도, 구역 폭은 최소 6도다. 구역이 전혀 없을 때만 5cm
   추가 여유를 제거해 재시도하며 로봇 반경은 그대로 유지한다.
4. 일반 구역은 각도 중앙을 목표 방향으로 잡는다. 360도가 하나의 열린 구역이면
   구역은 하나로 유지하면서 nominal 이동 방향 기준 `+90°, −90°, +180°`의
   세 방향을 추가 평가한다. 기준은 차체 yaw가 아닌 nominal 종단 변위 방향이다.
   종단 변위가 1e-6m 이하이면 공통 경로 기준점 방향을 사용하고, 그 방향도
   없으면 로봇 전방을 사용한다. 정확한 상대 각도를 유지하며 1도 간격 깊이의
   양옆 값 중 작은 값을 사용하고, 실제 seed 전체의 정적 충돌도 검사한다.
   nominal 자체가 원래 방향을 담당하며 세 후보에도 동일한 최적화·검사·비용 배분을 적용한다.
   먼저 **LiDAR 깊이를 넣지 않고** 2초 동안 해당 방향으로 갈 수 있는 거리를
   최대화하여 `D_g`를 구한다. 이때 **heading 비용은 없지만 ω는 최적화 변수**이며,
   실제 비선형 메카넘 적분, 현재 속도, 축·가속도·wheel L1 제한을 적용한다.
   종단은 해당 방향선 위에 둔다. 초기 제어열도 LiDAR 깊이와 독립적으로 만든다.
   실제 목표거리를 `rho_g = min(D_g, d_g)`로 고정한 뒤 공통 진행 기준점·직진성·
   heading·입력 변화의 비용을 최소화하여 `V_actual`을 구한다. **거리 최대화와
   실제 목표점 비용 최소화, 총 두 번의 풀이**이며 회전은 두 단계 모두 허용한다.
   별도의 바퀴 한계 비교 비용 `V_wheel`은 운영에서 계산하지 않는다. 실제 문제에서는
   각 시점의 방향 투영 거리를 `d_g` 이하로 제한한다. 종단 허용오차는 1cm다.
   비용 최소화의 초기값은 거리 최대화에서 구한 제어열이다.
5. seed 전체의 swept footprint 정적 충돌, 해당 gap 안의 종단, 깊이 제한,
   **해당 gap 방향으로** 최소 0.25m 이동을 검사한다. 후진도 같은 기준이다.
   최적화 seed의 종단 오차는 기본 1cm(작은 수치 허용오차 포함), 시간 부족으로
   사용하는 재사용/규칙 기반 seed는 기존 0.35m 기준을 적용한다.
   nominal과의 XY RMS가 0.22m 미만이면서 마지막 3점 평균 거리가 0.30m 미만이면
   중복으로 제외한다. nominal 자체가 정적 충돌이면 이 중복 제외는 적용하지 않는다.
6. 통과한 후보를 같은 네 항의 비용으로 정렬하고 최대 3개를 남긴다.
   선택된 후보와 RMS 0.25m 미만으로 겹치면 중복으로 제외한다.
   seed 목적함수에는 정적·동적 장애물 비용을 넣지 않는다. LiDAR 깊이와 swept
   충돌 검사는 별도로 수행하고, 동적 위험은 최종 MPPI에서 평가한다.
   nominal보다 비용이 낮아야 한다는 조건은 두지 않는다.
   최적화 성공 후보의 정렬·배분에는 **`V_actual`만** 사용한다. 실제 목표점 풀이가
   실패하면 재검사한 이전 seed 또는 기본 seed를 사용하며 최적화 성공으로 표시하지 않는다.
7. nominal에도 같은 비용을 계산한다. nominal 최소 100개, 선택 gap별 최소
   30개를 먼저 확보하고, 남은 예산을 `20% 균등 + 80% 비용 softmax`로 배분한다.
   총합은 항상 500개이며 추가 gap이 없으면 nominal에 500개를 준다.
   각 그룹에서 MPPI를 실행하고 그룹 대표 및 독립 감속 Stop을 기존 최종
   정적·동적 검사로 검증한다. 현재 YAML의 1차 MC / 최종 Gaussian disk 설정도 적용된다.

구체적인 seed 비용은 다음과 같다. 모든 위치는 현재 로봇 기준 좌표이며,
`e_g`는 후보 방향, `e_g_perp`는 그 수직 방향이다.

```text
J_g = 8 ||p_N − p_ref||²
    + 4 mean((e_g_perpᵀ p_t)²)
    + 3 mean((ψ_t − ψ_ref,t)²)
    + mean((u_t − u_(t−1))ᵀ diag(0.5, 0.5, 0.055) (u_t − u_(t−1)))
```

`p_ref`는 현재 경로 투영 지점에서 `desired_speed × 2초` 앞의 공통 기준점이며
경로 끝을 넘지 않는다. gap이 짧아져도 기준점을 그 gap 목표점으로 바꾸지 않는다.
`ψ_ref,t`는 같은 경로의 진행 방향이며, `u_−1`에는 현재 제약 적용 속도를 쓴다.
nominal의 직진성 기준선은 자신의 종단 방향이고, 종단 이동이 없으면 공통 기준점
방향을 사용한다. SLSQP와 해석 Jacobian을 사용하며 전역 최적성을 보장하지 않는다.
과거 비교 실험의 `V_wheel`과 `V_actual`은 같은 주행 기준을 서로 다른 종단 조건에
적용한 값이다. 운영에서는 실제 목표점의 값만 구한다. 목표점이 달라지는 문제이므로
거리가 짧아질 때 비용이 반드시 증가한다는 보장은 없다.

배분은 nominal을 포함한 M개 그룹의 `ΔJ_i = J_i − min(J)`에 대해
`τ = max(1, median(ΔJ))`, `P_i = 0.2/M + 0.8 exp(−ΔJ_i/τ) / Σ exp(−ΔJ/τ)`다.
최소 개수를 제외한 예산에 `P_i`를 곱하고 소수점 잔여가 큰 순서로 정수 보정한다.
따라서 nominal도 나머지 샘플을 두고 gap들과 경쟁한다.
예전 `gap.rollouts_per_guide`는 호환성을 위해 읽지만 배분에는 사용하지 않는다.

nominal은 직전 **최종 선택 제어열**을 한 step 이동한 값이다. 최초 또는 reset
직후에는 기존 path guide를 사용한다. 이전 gap seed는 번호 대신 회전 보정된
각도 구간의 겹침(기본 Jaccard 0.5 이상)과 방향 차이(20도 이하)로 대응시킨다.
최대 0.6초, 위치 변화 0.5m 이내만 재사용하며 경과 cycle만큼 제어열을 이동한다.
재사용 때도 현재 제약·충돌·깊이·도달 가능성·비용을 다시 계산한다.
gap 번호에 따른 switching hysteresis는 적용하지 않는다.
공유하는 guide 설정은 noise와 평가 방식이며 gap 예산·시간 제한은 `gap.*`로
따로 지정한다. gap에서는 static 검사와 unknown 차단을 필수로 사용한다.
`static_cost.enabled: false`로 끄거나 실행 중 strategy/gap 설정을 바꾸는 것은 거부한다.

정상 입력에서 gap이 0개인 경우와 입력 오류를 구분한다. scan 누락·만료·과도한
시각 차이·TF 실패는 `stale_scan` hold이며 목표 자세 정렬도 같은 입력 검사를 받는다.
후보 준비·최적화 예산은 `min(140ms, 전체 180ms 마감까지 남은 시간 - MPPI 예약)`이다.
최초 주기나 장애물 예측 배열 차원이 바뀌면 100ms를 예약한다. 이후 최근 16주기의
후보 생성 이후 계산시간(선택·배분·MPPI·최종 검사 포함) 최댓값에 15ms를 더해
예약하며 최소 50ms를 유지한다. 이는 부하 추정이며 실행시간 보장은 아니다.
먼저 재사용/규칙 기반 seed를 검사하고 남은 시간에 거리 최대화 → 실제 목표점
비용 최소화를 수행한다. 후보 순서는 가장 오래 계산을 시도하지 않은 후보부터,
시도 주기가 같으면 끝내지 못한 후보부터다. 시간 초과로 마지막 후보가 계속 밀리는
것을 방지하고, 반복 실패 후보 하나가 시작 순서를 독점하지 않게 한다.
순회 이력은 seed 재사용과 같은 위치·방향·구역 겹침·시간 조건으로 대응시키며
매 주기 달라질 수 있는 gap 번호에 연결하지 않는다.
미수렴·시간 초과 해는 채택하지 않고 사전 검증된 seed를 사용한다. 재사용이나
fallback 비용도 최적해의 비용과 구분할 수 있도록 진단값에 출처를 표시한다.
gap 계산 및 MPPI 결과가 `gap.timeout`(기본 0.18초)을 넘으면 결과를 폐기하고 hold한다.
최적화의 함수 평가·반복 사이에서 시간을 확인하고 worker 종료도 기다리므로,
후보 예산/180ms는 프로세스를 강제 중단하는 실행시간 상한은 아니다.

`/dra_mppi/gap_diagnostics` (`std_msgs/String`, JSON)에 구역 수, 후보별 각도·
도달 오차·이동량·nominal과의 거리·탈락 이유·선택 gap·할당 개수, nominal/후보의
비용과 네 항의 내역, seed 출처(`optimized/reused/fallback/none`), 최적화 상태,
기하 처리·후보 처리 시간, 배분 온도를 발행한다.
후보별 `wheel_distance_m`, `actual_target_distance_m`, `actual_value_cost`,
`lidar_limited`로 도달 거리·실제 목표점·비용을 기록한다. 이전 진단 키는 호환성을
위해 유지하되 `wheel_value_cost`와 `value_difference`는 `null`,
`wheel_value_status`는 `not_computed`다. `solver_policy=reach_then_actual`로 구분한다.
`wheel_reach_status`/`actual_value_status`에 각 풀이 상태를 남기고,
완료하지 못한 단계의 비용은 `null`이다. 실제 배분 점수 `cost`의 출처는
`allocation_cost_source=optimized_actual/reused_seed/fallback_seed/none`으로 구분한다.
재사용/규칙 seed의 재평가 점수는 현재 cycle의 최적 비용이라고 표시하지 않는다.
`optimization_order`, 후보별 `schedule_rank`와 과거 시도/종료 주기로 순회 상태를,
`proposal_budget_ms`, `mppi_reserved_ms`, `post_proposal_ms`로 동적 시간 예산을 확인한다.
`/dra_mppi/rollouts`의 `gap_regions`에는 구역 경계를, 기존 guide marker에는
nominal과 선택된 중앙 seed 및 rollout을 표시한다. gap 번호와 색은 매 주기 기준이다.

[두 단계 풀이·순회·시간 예산 보완 검증](docs/gap_two_solve_scheduler_2026-11-09/IMPLEMENTATION.txt):
현재 운영 구현은 거리 최대화와 실제 목표점 비용 최소화만 수행한다. 같은 bag 추출
16장면×8회 비교에서 새 최적화 완료 후보가 206/400→323/400, 미착수 후보가
103→15개로 줄었다. 평균 전체 시간은 더 많은 후보를 계산하여 97.6→111.1ms로
늘었고, 변경 후 p95 164.1ms·최대 168.1ms였다. 128회 모두 정상 반환했다.
최종 비용 평균은 변경 전 대비 0.62% 감소했으나, 모든 장면에서 개선된 것은 아니다.
이는 snapshot 재생이며 전체 ROS 부하/폐루프 주행에서의 5Hz 보장은 아니다.
아래 기록은 각 변경 당시의 과거 구현 결과다.

[계산 시간 단축 전후 비교](docs/gap_latency_optimization_2026-11-06/RESULTS.txt):
LiDAR polygon 좌표를 일괄 계산하고 최적화 Jacobian의 단계 반복을 배열 연산으로 바꿨다.
기본 시간 예산에서 전/후 각각 16장면×8회 비교 시 평균 112.6→96.9ms,
p95 157.8→145.3ms, 최대 169.5→157.9ms였다. 시간 제한을 넉넉히 준 동일 계산량
128쌍에서는 샘플·비용·최종 선택이 정확히 일치했다. 운영 예산에서는 더 많은
gap 최적화를 완료하므로 결과가 달라질 수 있다. 500개 평가, 비용, 안전 조건과
5Hz 설정은 유지했으며, 평균 시간만으로 10Hz를 보장하지 않는다.

[wheel/LiDAR 분리 후 운영 코드 검증](docs/gap_two_stage_implementation_2026-11-06/IMPLEMENTATION.txt):
실제 bag에서 추출한 16장면을 기본 예산으로 192회 재생했다. 평균 112.3ms,
p95 155.7ms, 최대 172.7ms였고 192회 모두 정상 반환했다. 선택된 gap seed 중
193개는 이번 주기에 최적화했고, 151개는 재검증한 이전 seed, 116개는 규칙 seed였다.
완료된 실제 비용 255개 중 113개는 LiDAR로 짧아진 목표점에 다시 최적화했고,
142개는 동일 wheel 목표점의 해를 재사용했다. 비용 출처를 합쳐 최적화 성공으로
집계하지 않는다. 이는 저장 snapshot 재생이며 ROS 전체 5Hz/폐루프 보장은 아니다.

[분리 이전 운영 코드의 실제 bag 16개 장면·192회 재생 결과](docs/gap_value_implementation_2026-11-05/validation.json)는
평균 109.9ms, p95 158.6ms, 최대 190.0ms였다. 191회 정상 반환, 1회는 180ms 제한으로
결과 폐기, 200ms 초과는 0회였다. 동적 예측이 있는 장면은 6개다.
별도 cold-start 안전 검사 16회에서는 15회 정상, 1회 시간 초과 폐기였으며
최대 197.1ms였다. 정상 반환 15회는 선택 경로의 정적 충돌 없음과 동적 hard 기준을
명시적으로 assert했다(`safety_check/validation.json`).
동일 snapshot과 기록 nominal을 반복 사용하고 seed cache/worker를 유지한 검증으로,
ROS/TF/deskew/발행 및 다른 노드 부하를 포함한 전체 5Hz·폐루프 검증은 아니다.
재현 도구는 `tools/validate_gap_value_strategy.py`이며
[구현 및 검증 기록](docs/gap_value_implementation_2026-11-05/IMPLEMENTATION.txt)에 범위를 명시한다.
이전 midpoint/고정 배분 실험은 기존 `docs/gap_strategy_2026-11-04/`에 보존한다.

## Guide-conditioned strategy

`mppi.strategy: guide_conditioned`는 AERO-MPPI의 anchor/ensemble 아이디어를
메카넘에 적용한다. 지원 전략은 `guide_conditioned`, `standard`, `gap` 세 가지다.
YAML에서 strategy를 바꾸고 노드를 재시작하면 전환된다.

새 전략의 순서는 다음과 같다.

1. 현재 로봇 yaw 기준 `0°, +45°, +90°, -45°, -90°` 병진 방향을 만든다.
   각 주기의 world 이동 방향은 고정하고, 차체 yaw는 Global Path tangent를
   향하도록 독립 제어한다. 속도·wheel L1·가속도를 만족시키며 10 step rollout한
   결과가 guide이다. 종점/길이 고정이나 Global Path 재합류 구간은 없다.
   실제 global goal 근처에서는 남은 거리로 속도를 줄인다. 짧은 local tracking
   window의 끝점을 실제 종점으로 취급하지 않는다.
2. 5개 guide를 모두 유지하고 `mppi.guide_num_rollouts: 500`을 균등 분배한다.
   각 guide에는 warm-start center, 현재 anchor, Halton spline noise 후보가 있다.
   MP-MPPI motion primitive는 아직 사용하지 않는다.
3. 각 guide의 sampling/rollout/공통 비용/정적 검사/충돌확률/가중평균을
   persistent 2-worker pool에서 계산한다. 최종 대표 검사와 nominal 갱신은
   호출 thread가 담당한다. `mppi.guide_evaluation=batch`는 공통 MC 비교용이다.
4. 각 유효 guide의 평균 제어열을 다시 rollout하고, 현재 측정 속도에서
   `limits.acceleration`로 감속하는 독립 Stop과 공통 비용으로
   비교한다. 대표 5개와 Stop은 같은 최종 확률 계산기로 다시 평가한다. 기본 `final_mode: gaussian_disk`에서는
   각 원형 충돌영역을 직접 적분하므로 다른 후보의 영역 크기에 영향을 받지 않는다.
   최종 `monte_carlo` 선택 시 하나의 공통 MC batch를 사용한다. 대표 궤적의 동적 위험
   threshold와 정적 hard collision을 다시
   검사하고, safe 후보에만
   switching penalty/hysteresis를 적용한다. Stop에는 switching penalty를 부과하지 않는다.
   safe 후보가 없으면 hold를 출력하면서 `no_safe_candidate_hold`로 보고한다.
   이 hold는 움직이는 사람이나 현재 충돌 상태에서 안전 보장을 의미하지 않는다.

`static_cost.enabled: true`이면 `/traversable_map` distance field로 각 noise
rollout, 최종 대표 궤적과 Stop의 swept clearance를 평가한다. footprint가 정적
장애물과 겹치면 invalid mask로 제외하고, footprint clearance의 hinge-square 평균을 soft cost로 준다.
Anchor 충돌만으로 guide 전체를 제거하지는 않는다. 지도 누락·stale·TF 오류 때는
fail-safe hold한다. `false`이면 map을 요구하지 않고 정적 평가 없이 동작한다.
Terminal pose alignment도 동일 입력 제약과 정적/동적 안전 검사를 거친다.

실행 중 전환: `ros2 param set /dra_mppi_node static_cost.enabled true` 또는
`false`. OFF는 정적 soft cost, 충돌 검사, 지도 freshness gate를 함께 끈다.
동적 위험 검사는 이 스위치와 독립이다. 새 비용과 검증 범위는
[2026-10-22 구현 보고서](docs/guide_only_2026-10-22.md)를 참고한다.

새 전략의 제약 투영은 feasible target으로 향하는 선분 위에서 가속도를 제한하므로
측정 입력이 feasible하면 box/wheel/rate를 동시에 지킨다. 측정 속도 자체가 이미
wheel 제한을 벗어난 경우 첫 projection에서는 wheel 제한을 우선한다.
Stop 후보도 이동 후보와 같은 모델로 rollout한다. 각 body-frame 속도 성분을 매
step마다 최대 `limits.acceleration * mppi.dt`만큼 0으로 줄이므로, 정지할 때까지의
이동 거리와 회전이 동적 충돌확률 및 최종 비용에 반영된다. 정지 후 남은 horizon은
zero velocity를 유지한다.

YAML 파라미터 그룹:

- 공통: `mppi.dt`, horizon, temperature, Halton 설정, limits, cost,
  dynamic_risk, static_cost.
- standard 전용 예산: `mppi.num_rollouts` (321).
- standard 전용: `mppi.noise_std`.
- guide_conditioned 설정: `mppi.guide_*` (gap과 noise/평가 방식 공유).
- gap 전용: `gap.*` (센서·분할·도달 가능성·중복 검사·예산·시간 제한).
- `traversability.*`: 정적 비용의 지도 topic, timeout, unknown 정책을 정한다.
  지도 발행 시각뿐 아니라 `sensor_watermark`의 캡처 시각도 검사한다. 필요한 센서가
  아직 관측하지 않았거나 관측이 만료되면 주행·목표 자세 정렬 모두 정지 기준열을 낸다.
  기본 autonomy는 LiDAR 두 대를 모두 요구하며, depth는 semantic/voxel 기능을 켰을 때 요구한다.
  `transform_max_age`(기본 0.5초)는 동적 측위 TF의 최대 나이다. 오래된 TF 캐시로 주행하지 않는다.

RViz의 `/dra_mppi/rollouts`에는 세 종류만 표시한다. `guide_paths`는 5개 anchor
(모두 방향별 고정색), `guide_rollouts_<guide>`는
guide별 상위-weight 후보, `selected_path`는 최종 선택 궤적이다. 후보 표시는
`visualization_rollouts` 총예산을 guide에 균등 배분하며 500개 전체를 그리지
않는다. 고정 5방향은 서로 다른 homotopy를 보장하지 않으며, 완전한 후진 탐색을
지원하지 않는다.

`monte_carlo` 모드에서 전체 500개 rollout의 guide별 MC 영역과 예산은
`/dra_mppi/guide_mc_region_areas`, `/dra_mppi/guide_mc_sample_counts`에
각각 `[guide, timestep]` 행렬로 발행한다. 사람이 없어 실제 MC를 실행하지 않는
cycle에도 잠재 영역과 예정 sample 수를 기록하므로 bag에서 전체 영역을 복원할 수 있다.
선택 가능한 `gaussian_disk`는 MC 표본을 사용하지 않으므로 sample 수는 0이며,
JSONL의 `risk_integration_mode`, `primary_analytic_evaluations`,
`primary_quadrature_evaluations`, `final_quadrature_evaluations`,
`*_unresolved_integrals`로 실제 계산량과 실패를 확인한다.

이하 기존 전략 설명은 해당 strategy를 선택했을 때 적용된다.

메카넘 AMR용 동적 장애물 회피 local planner다. DRA-MPPI가 직접 바퀴를
제어하지 않고, 앞으로 2초 동안의 최적 pose/velocity reference를 만들어 하위
MPC에 전달한다.

```text
Global Path + Odometry + 보행자 예측
                  │
                  ▼
       DRA-MPPI: 321개 후보 비교
                  │
                  ▼
 [x, y, yaw, vx, vy, yaw_rate] × 10 step
                  │
                  ▼
           MPC → /cmd_vel → AMR
```

## 현재 설정

| 항목 | 현재값 | 의미 |
|---|---:|---|
| Planner 주기 | 5 Hz | 한 주기 시간 200 ms |
| `dt` | 0.20 s | MPPI 한 step의 시간 |
| Horizon | 10 step | 앞으로 2초 예측 |
| Rollout | guide 500개 + Stop | standard 선택 시 321개 |
| Control | `[vx, vy, yaw_rate]` | 메카넘 body-frame 속도 |
| Path lookahead | 2.0 m | local tracking path 길이 |
| Heading lookahead | 0.20 m | 경로 진행 방향을 구하는 거리 |
| Collision radius | 0.45 m | robot 0.20 m + obstacle 0.25 m |
| Risk integration | 1차 MC / 최종 Gaussian disk | Adaptive Halton 8배 + 대표 직접 적분 |
| MC density | 992 samples/m² | 기존 예산의 8배, 영역 면적에 비례 |
| MC 표본 범위 | guide 1024~8192 | common 4096~32768, 최종 직접 적분은 MC 표본 0 |

주요 설정은 [config/dra_mppi.yaml](config/dra_mppi.yaml)에 있다.

## 입출력

### 입력

| Topic | 내용 |
|---|---|
| `/global_path` | Theta* 등이 만든 XY 경로 |
| `/goal_pose` | 새 목표 입력 및 이전 goal-completion latch 해제 |
| `/odometry/filtered` | 현재 `[x,y,yaw]`와 `[vx,vy,yaw_rate]` |
| `/predictions` | 전체 클래스의 장애물별 CV/CT Gaussian mixture 미래 예측 |

### 출력

| Topic | 내용 |
|---|---|
| `/dra_mppi/reference_trajectory` | MPC가 추종할 10-step 전체 reference |
| `/dra_mppi/reference` | 첫 step reference |
| `/dra_mppi/selected_path` | 최종 선택 trajectory (`nav_msgs/Path`) |
| `/dra_mppi/rollouts` | guide anchor·표본 궤적·최종 선택 궤적 (`MarkerArray`) |
| `/dra_mppi/predictions_used` | class whitelist를 통과해 위험 계산에 쓰는 예측 |

`publish_cmd_vel: false`가 기본이다. 실제 `/cmd_vel`은 하위 MPC 하나만 발행해야
한다. DRA-MPPI `dt=0.20 s`와 MPC `Ts=0.04 s`의 차이는 MPC에서 시간 보간한다.

목표점의 `goal_tolerance`(기본 0.15 m) 안에 들어오면 이동 MPPI를 멈추고 terminal
pose 정렬 상태로 들어간다. 이 상태는 정확한 RViz goal XY로 저속 보정하면서 `2D
Goal Pose`의 화살표 방향도 함께 맞춘다. 위치 오차가 다시 0.15 m보다 조금 커져도
상태를 유지하고, `goal_terminal.exit_tolerance`(기본 0.25 m)를 넘을 때만 MPPI로
돌아간다. 이 hysteresis가 목표 경계의 정지/재출발 반복을 막는다.

위치·yaw·측정 속도가 모두 허용 범위에 들어온 상태를 기본 3주기 연속 확인한 뒤
정확한 요청 goal pose를 zero velocity로 고정한다. 이 terminal 정렬과 완료 판정은
`standard`, `guide_conditioned`, `gap`에 공통으로 적용된다.

완료 후 관성이나 외란으로 tolerance 밖으로 밀려도 planner를 다시 실행하지 않고
같은 pose와 zero velocity reference를 계속 발행한다. 새 `/goal_pose`가 들어오고
대응하는 non-empty `/global_path`를 받은 뒤에만 다음 주행을 시작한다. 도착 시 이전
selected path와 rollout marker는 삭제한다.

## 한 주기의 동작

### 1. 제어 후보 생성

`standard`는 shifted nominal, 감속 Stop, deterministic path guide와
Halton spline noise 후보를 합쳐 `mppi.num_rollouts`개를 평가한다.
`guide_conditioned`는 위에 설명한 5방향 ensemble과 독립 Stop을 사용한다.

Path guide는 현재 위치를 Global Path의 arc length에 투영하고, 매 step
`desired_speed × dt`만큼 앞의 목표점을 향한다. world-frame 목표속도를 현재 guide
yaw 기준 body-frame `vx,vy`로 변환하고 path tangent를 향하는 `yaw_rate`를 만든다.
실제 Global Path 종점에서는 acceleration limit로 계산한 braking speed를 사용한다.

Guide도 다른 후보와 같은 제약과 모든 cost를 적용받는다. Global Path 위에 동적
장애물이 있으면 `C_risk`가 커져 다른 회피 후보가 선택된다. 현재 공통 비용은
실제 global goal과 연속 투영을 위해 전체 Global Path를 받는다.

### 2. 메카넘 제약 적용

각 후보에 축별 속도, 가속도, 바퀴 결합 한계를 적용한다.

```text
control_min ≤ u_t ≤ control_max
|u_t - u_(t-1)| ≤ acceleration_limit × dt
|vx| + |vy| + yaw_lever × |yaw_rate| ≤ speed_limit_l1
```

마지막 식은 메카넘 역기구학 때문에 필요한 실제 wheel feasibility constraint다.
단순히 `vx`, `vy`, `yaw_rate`를 각각 clipping하는 것으로는 대신할 수 없다.

현재 적용 순서는 `axis clip → acceleration clip → wheel projection`이다. 마지막
wheel projection이 크게 작동하면 감속 방향 acceleration limit이 다시 깨질 수
있으며, 이 부분은 아직 남아 있는 제약 결합 문제다.

### 3. Holonomic rollout

상태와 제어는 다음과 같다.

```text
state   = [x, y, yaw]
control = [vx, vy, yaw_rate]
```

```text
x(t+1)   = x(t) + (vx cos(yaw) - vy sin(yaw)) dt
y(t+1)   = y(t) + (vx sin(yaw) + vy cos(yaw)) dt
yaw(t+1) = yaw(t) + yaw_rate dt
```

321개 제어열을 이 모델에 넣어 321개의 미래 trajectory를 만든다.

### 4. 비용 계산

`common_cost.common_costs`를 standard와 guide의 후보/대표가 공유한다.

- 경로: `w_e mean(||p-p_path||²) + w_p*(s_N-s_ref,N)² + J_terminal`.
  `s_ref,N = min(L, s_0 + dt*sum(v_ref))`: cycle 시작 상태에서 정한 공통
  속도 기준을 적분하고 경로 끝에서 제한한다. 정지·후진·과속 모두 같은 기준과
  비교하며, 진행량 자체를 제곱해 후진까지 보상하지 않는다.
- Heading: `w_psi mean(wrap(yaw-psi_path)²)`, 각도 오차는 `[-pi, pi)`.
- Motion: `mean(w_v(v_parallel-v_ref)² + duᵀ R_delta du)`.
  v_ref는 cycle 시작 상태에서 정한 공통 시퀀스. 입력 변화량에는 /dt가 없다.
  절대 회전 속도 비용 `w_omega omega²`와 `cost.rotation_weight`는 제거했다.
  `du`의 yaw 성분은 급격한 회전 입력 변화를 억제하므로 유지한다.
  비용 진단의 기존 `rotation` 묶음은 `input_delta`로 이름을 변경했다.
- Static ON: `w_s mean(max(0,1-clearance/d_safe)²)`와 swept footprint invalid.
- Dynamic: `w_d mean(CP) + w_h count(CP>epsilon)` 및 대표의 별도 임계값 검사.

상태 비용은 `x_1..x_N`, 입력은 `u_0..u_(N-1)`이다. 속도 회전에는 rollout과
일치하게 `x_0..x_(N-1)` yaw를 쓴다. `J_terminal`은 기존 near-goal
조건에서만 켜지는 `w_goal*||p_N-p_goal||²`이다.
2026-11-04 변경으로 정적·동적 비용 외 공통 목적함수는 모두 **오차에 대한
2차 형식**이다. 경로 투영, 각도 wrapping, 기구학 때문에 제어입력 전체에 대한
전역 quadratic/QP라는 뜻은 아니다. Guide 선택의 switch penalty/hysteresis는
공통 rollout 목적함수 외부의 이산 선택 규칙으로 유지한다.
기존 수치 가중치는 유지했지만 progress 단위는 m에서 m²로, heading은
무차원 cosine에서 rad²로 바뀌었으므로 기존 실험 보고서의 튜닝 결과는
그대로 적용되지 않는다. 작은 각도에서 같은 heading 강도를 원하면 기존
가중치의 약 절반을 시작점으로 사용할 수 있으나 전체 주행 재검증이 필요하다.
경로 투영은 연속적인 양방향 호 길이 창을 사용해 자기 교차 분기로의 점프를
제한한다. 후진 시 진행 좌표가 감소할 수 있다. 먼 local endpoint를 목표로
벌점화하지 않고 전체 경로의 실제 끝점 근처에서만 terminal 거리 비용을 켠다.
사용되지 않는 이전 비용 파라미터는 제거했다. 정적 충돌은 유한한 벌점 대신
무한대 비용으로 제외한다.

### 5. MPPI update

```text
rho   = min(S)
w_k   = exp(-(S_k-rho)/beta) / eta
eta   = sum(exp(-(S_k-rho)/beta))
U*    = sum(w_k × candidate_k)
```

`beta`는 `eta`가 5~10 사이가 되도록 조정되며 갱신값은 다음 planner 주기에
사용한다. 선택된 `U*`는 한 step 이동해 다음 주기의 warm start가 된다.

### 6. 전략별 최종 선택

`standard`는 단일 분포의 weighted average를 다시 rollout하고 안전성을 평가한다.
`guide_conditioned`는 guide별 대표와 독립 감속 Stop을 공통 비용으로 재평가한 뒤
안전 후보 중 하나를 선택한다. 서로 다른 guide의 제어열을 평균하지 않는다.

## 동적 장애물 충돌확률

### Prediction 형식

Optimizer 내부 형식은 다음과 같다.

```text
means        : (obstacle, mode, time, 2)
covariances  : (obstacle, mode, time, 2, 2)
mode_weights : (obstacle, mode, time)
obstacle_radii: (obstacle,)
```

한 장애물의 여러 mode는 서로 다른 장애물로 세지 않고 Gaussian mixture로 만든다.

```text
p_o(z) = sum(mode_weight[o,m] × Gaussian[o,m](z))
```

`motion_predictor`가 `/tracked_object_modes`의 CV/CT/STATIC posterior를 measurement
update 없이 전파해 이 형식의 `/predictions`를 만든다. 기본 설정은 현재 posterior
mode weight를 horizon 전체에서 유지하고, `propagate_mode_probabilities: true`이면
mean-sojourn TPM으로 매 timestep의 weight도 전파한다.

DRA-MPPI는 `prediction_filter.class_names` whitelist를 적용한 뒤에만 optimizer로
넘긴다. 기본값은 `[person]`이며 tracker와 motion predictor의 전체 객체 출력은
바꾸지 않는다. 같은 결과를 `/dra_mppi/predictions_used`로 다시 발행하므로 RViz의
prediction marker도 실제 위험 계산에 들어가는 사람 예측만 표시한다.

prediction 배열의 첫 `time_offsets` 값은 `0.0`이며 tracker의 현재 IMM posterior다.
`DYNAMIC` person은 STATIC mode를 제외하고 각 시점의 CV/CT weight를 합 1로
재정규화한다. 움직임 확률 합이 너무 작으면 현재 위치 원으로 fallback한다.
`STATIC`과 `UNKNOWN`은 세 mode의
현재 posterior를 moment matching한 위치·공분산과 객체 반경을 고정하여 모든 horizon
step에서 같은 정적 안전 원으로 사용한다. 또한 lifecycle과 measurement/semantic age가
유한하지 않거나 허용 시간을 넘으면 optimizer 입력에서 제외한다. 단, 최근 LOST person은
기본 0.3초 동안만 마지막 예측 위치의 고정 안전 원으로 유지하며 이후 제외한다.

### Adaptive Halton sampling

`standard`는 각 timestep의 전체 rollout collision circle을
덮는 직사각형을 사용한다. 면적이 `A_t`일 때 표본 수는 다음과 같다.

```text
N_t = clip(ceil(4096 × A_t), 4096, 131072)
```

이 직사각형에 2-D Halton sample을 뿌리고 모든 rollout과 장애물이 공유한다.
`adaptive_halton_mc: false`로 바꾸면 비교용 Fixed Random 방식이 실행된다.
이때 standard/mode는 `N=mc_samples`, guide 1차/최종 평가는 각각
`N=guide_mc_samples_min`, `N=final_mc_samples_min`을 사용한다.

`guide_conditioned`의 1차 평가는 guide별 영역 `A_{g,t}`를 사용한다.

```text
N_{g,t} = clip(ceil(4096 × A_{g,t}), 4096, 131072)
```

guide마다 독립적인 scrambled Halton estimator를 갖는다. 최종
대표 5개와 Stop은 공통 영역에서 `density=4096`, `N=4096..131072`으로 재평가한다.
명시적 2-worker와 BLAS 내부 thread가 중첩되지 않도록 기본 launch는 BLAS/OMP
thread 수를 1로 제한한다.

### CP 계산 순서

각 장애물 density를 rollout collision circle에 먼저 적분한다.

```text
P_o(C) ≈ area(B) / M × sum(1(sample in C) × p_o(sample))
```

표본은 모든 후보의 충돌 원을 덮는 공통 사각형 B에서 뽑는다. 내부 표본이
0개이면 평가 불충분으로 해당 marginal을 1로 두어 안전 판정을 막는다.
그다음 장애물 간 독립 가정 아래 확률을 식 (9)로 결합한다.

```text
CP = 1 - product(1 - P_o(C))
```

Gaussian PDF는 확률이 아니라 `1/m²` 단위의 density라서 1보다 클 수 있다. 따라서
PDF를 `[0,1]`로 자르지 않는다. 기존 논문 식 (11)처럼 PDF를 점별 확률로 직접
곱하지 않고, `장애물별 적분 → 실제 확률 결합` 순서로 계산한다.

유한 sample 때문에 적분된 `P_o(C)`가 드물게 `[0,1]`을 벗어나면 식 (9)에 넣기
전에 안전하게 제한한다. 문제를 숨기지 않도록 다음 진단값과 ROS warning을 남긴다.

```text
clipped_marginal_count  # 범위를 벗어나 보정한 적분값 수
raw_marginal_minimum    # 보정 전 최소값
raw_marginal_maximum    # 보정 전 최대값
empty_circle_count      # 내부 sample이 없었던 collision circle 수
```

### 배치 벡터화

장애물과 mode를 Python 반복문으로 하나씩 계산하지 않는다.

```text
모든 covariance 안정화/inverse/determinant 일괄 계산
→ 모든 obstacle/mode/sample PDF 일괄 계산
→ circle mask × obstacle density 행렬곱으로 모든 적분 계산
```

수식과 sample은 그대로 두고 NumPy가 큰 배열을 한꺼번에 처리하게 해 계산시간을
줄인다. timestep마다 계산영역과 `N_t`가 다르므로 20-step 반복은 유지한다.

## 과거 계산시간 기록 (새 비용 검증에 사용하지 않음)

아래 값은 6-mode였던 이전 `K=400`, `T=20` 기준을 개발 PC에서 측정한 역사적
baseline이다. 경로 80점, 장애물 10명의 합성 데이터로 OpenBLAS thread를 1개로
고정하고 warm-up 3회 후 50 cycle을 측정했다. 현재 2-behavior+Stop `K=321`, `T=10`
구조는 실제 bag과 NUC에서 다시 측정해야 한다.

| Prediction 조건 | Strategy | compute p50 | p95 | p99 |
|---|---|---:|---:|---:|
| 없음 (`risk=off`) | standard | 14.26 ms | 14.59 ms | 14.73 ms |
| 없음 (`risk=off`) | mode-conditioned | 15.67 ms | 15.88 ms | 16.18 ms |
| 장애물 10, prediction mode 1 | standard | 55.00 ms | 64.74 ms | 65.74 ms |
| 장애물 10, prediction mode 1 | mode-conditioned | 84.03 ms | 87.05 ms | 88.00 ms |
| 장애물 10, prediction mode 4 | standard | 55.92 ms | 66.46 ms | 67.15 ms |
| 장애물 10, prediction mode 4 | mode-conditioned | 88.83 ms | 97.38 ms | 101.85 ms |

Mode 자체의 cost는 `risk=off` 기준 p50 약 1.4 ms다. 동적 장애물이 있을 때의
차이는 여섯 최종 trajectory 때문만은 아니다. 좌우 회피 분포가 더 넓은 영역을
덮어 Adaptive MC sample 총수가 증가한다. 장애물 10, prediction mode 1 조건의
median은 standard 약 18.3k sample, mode-conditioned 약 26.7k sample이었다.
Mode-conditioned의 1차 risk p50은 약 67.0 ms였고, cached final 6-mode risk는
약 1.6 ms였다.

개발 PC의 optimizer p99만 보면 single-mode prediction에서 약 88 ms이므로
약 11.4 Hz에 해당한다. 4-mode 합성 stress 조건은 약 101.9 ms, 약 9.8 Hz다.
현재 설정 주기는 5 Hz(200 ms)이므로 optimizer 기준 여유는 있지만, 실제 전체
ROS callback 주기는 TF/path/prediction 전처리와 publish를 포함해 다시 측정해야 한다.
노드는 `cycle=... ms (optimizer=... ms, budget=200.0 ms)`를 로그로 출력하고
200 ms deadline 초과 시 warning을 발생시킨다.

같은 개발 PC의 로컬 ROS graph에서 path 80점, odometry, 단일-mode prediction
장애물 10개를 실제 topic으로 넣고 mode별 marker까지 발행한 smoke test에서는,
warm-up 이후 여섯 status sample이 `optimizer=83.7~87.5 ms`, 전체 callback
`cycle=89.5~93.4 ms`였다. ROS 전처리와 reference/RViz publish를 합친 overhead는
약 5.8~6.2 ms였다. 첫 동적-risk cycle은 cold start로 `optimizer=212.3 ms`,
`cycle=218.0 ms`가 걸려 200 ms deadline을 한 번 넘었다. 짧은 합성 smoke test이므로
실제 NUC의 p99를 대신하지 않으며, 시작 직후 deadline miss도 실제 stack에서 별도로
확인해야 한다.

Tracking, heading과 path-frame speed는 같은 polyline 투영 결과를 공유한다. 모든
rollout 위치를 각 유효 경로 선분에 투영해 가장 가까운 점과 선분 인덱스를 한 번
구한다. 연속 중복 waypoint가 만든 길이 0 선분은 제외한다. 속도 cost의 첫 step만
현재 상태의 경로 투영을 한 번 더 사용하고 이후 step은 앞서 구한 결과를 shift해서
재사용한다.

Adaptive MC 영역은 모든 rollout을 감싸므로 guide나 control-mode 후보가 기존 후보보다
바깥으로 나가는 timestep에서는 영역 `A_t`와 MC sample 수가 달라질 수 있다. 전체
후보 수와 cost 평가 개수는 400개로 동일하지만 risk 계산시간이 항상 정확히 같다고
보장할 수는 없다. 위 합성 시험에서는 optimizer p99가 200 ms 주기 안에 있었다.

추가 최적화 우선순위는 다음과 같다.

1. **Adaptive `mc_samples_min` 정확도 실험**
   - 계산영역이 작으면 `N_t`가 대부분 최소값 512에 걸리므로 density만 낮춰도
     sample 수가 줄지 않는다.
   - 합성 시험에서 최소값을 512→384→256으로 낮추자 risk p99가 대략
     45→38→30 ms로 줄었다.
   - 한 합성 장면에서는 CP MAE가 약 0.00013~0.00015였지만, 좁은 Gaussian과
     실제 보행자 데이터까지 검증하기 전에는 설정을 바꾸면 안 된다.

2. **영향이 없는 장애물/mode의 보수적 제거**
   - 모든 rollout에서 충분히 멀리 있는 Gaussian은 계산을 생략할 수 있다.
   - 단순 거리 cutoff는 위험하므로, 생략한 확률의 상한을 계산할 수 있을 때만
     적용해야 한다. 장애물이 모두 가까우면 효과가 없다.

3. **큰 임시 배열만 float32 검토**
   - 거리, mask 주변 계산을 float32로 바꾸면 메모리 대역폭을 줄일 수 있다.
   - CP threshold `0.05` 근처 결과가 달라질 수 있으므로 정확도 회귀시험이 필요하다.

4. **시각화와 ROS 전처리 주기 분리**
   - `/rollouts`와 marker publish를 더 낮은 주기로 실행할 수 있다.
   - prediction 변환/보간, TF lookup과 publish 시간도 planner 계산시간과 별도로
     측정해야 한다.

장애물별 Python thread나 multiprocessing은 배열 복사와 scheduling 비용이 생기므로
우선순위가 낮다. 다음 단계는 전체 stack profiling과 Adaptive sample 정확도 검증이
안전하다. 5 Hz의 절대 한계는 200 ms지만 다른 노드의 부하를 고려하면 planner p99를
150~160 ms 이하로 유지하고 deadline 초과 횟수를 함께 확인해야 한다.

## 현재 주의점

- feasible 측정 입력에서 convex 선분 투영으로 box/wheel/rate를 함께 지킨다.
  측정 입력 자체가 infeasible하면 초기 projection에서 wheel 제한을 우선한다.
- `/predictions`의 CV/CT process noise와 mean-sojourn time은 bag으로 보정해야 한다.
- fresh empty prediction은 무장애물이다. predictor 누락/timeout은 기본
  `prediction_fail_safe: true`에서 hold한다.
- `static_cost.enabled: true`에서는 정적 obstacle soft/hard 비용이 포함되며,
  fresh map이나 TF가 없으면 hold한다.
- 합성 benchmark가 아닌 실제 NUC 전체 stack 시간 측정이 필요하다.
- `clipped_marginal_count` 또는 `empty_circle_count`가 반복되면 MC 설정을 재조정한다.

## 빌드와 테스트

```bash
cd ~/2026amr_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to dra_mppi --symlink-install
source install/setup.bash
ros2 launch dra_mppi dra_mppi.launch.py
```

```bash
cd ~/2026amr_ws/src/amr_basic_stack
PYTHONPATH=planner/local_planner/DRA_MPPI \
  pytest -q planner/local_planner/DRA_MPPI/test
```

현재 테스트 결과와 원시 pilot 자료는 위 구현 보고서에 기록한다.

## 주요 파일

```text
DRA_MPPI/
├── config/dra_mppi.yaml       # 파라미터
├── dra_mppi/
│   ├── mppi_controller.py     # 후보 생성, 비용, MPPI update, warm start
│   ├── dynamic_risk.py        # Adaptive MC, CP, Gaussian 벡터화
│   ├── control_sampling.py    # Halton spline control noise
│   ├── mecanum_model.py       # Holonomic rollout
│   └── dra_mppi_node.py       # ROS topic, TF, publish
└── test/                      # 단위 테스트
```
