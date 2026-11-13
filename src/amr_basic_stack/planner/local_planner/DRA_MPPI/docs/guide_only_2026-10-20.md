# Guide-only 공통 비용 적용 및 검증 기록 — 2026-10-20

## 적용 상태와 스위치

`mppi.strategy: guide_conditioned`와 로봇 상대 5방향
`[0,+45,+90,-45,-90]`를 유지했다. `vx,vy,omega`를 함께 샘플링하며
새 후진 guide나 Crab/Turn 분리를 추가하지 않았다.

```bash
# 실행 중인 노드의 정적 장애물 비용 ON / OFF
ros2 param set /dra_mppi_node static_cost.enabled true
ros2 param set /dra_mppi_node static_cost.enabled false
```

기본 YAML은 기존과 같이 **true**다. YAML 변경은 다음 실행에 적용되고,
위 parameter service 변경은 재시작 없이 적용된다. 전환 시 nominal과 경로
진행 warm start를 초기화한다. ON은 정적 soft 비용, swept 충돌 후보 제외,
지도/TF freshness gate를 함께 켠다. OFF는 이 세 가지를 함께 끈다.
동적 위험, 축/바퀴/입력 변화율 제한은 이 스위치로 끄지 않는다.
TF broadcaster, 하위 MPC, command gate, 최종 구동 토픽 소유권은 수정하지 않았다.

## 시작 시 확인한 차이

- 기준 commit: `d9f3968ee0c312399722101c74e1334f225aa268`.
  작업 시작 시 저장소 내부 tracked 변경은 없었으며 외부 `../amr_project.zip`은 보존했다.
- checkout 및 상위 경로에서 `AGENTS.md`, workspace와 home 검색에서
  `AMR_STACK_CONTEXT.md`를 찾지 못했다. 실제 소스와 YAML을 기준으로 작업했다.
- guide 방향/500개 예산/5Hz/dt=.2/N=10은 요청서 참고값과 일치했다.
  standard 기본 예산은 321이며 공정 비교에서 별도로 맞췄다.
- 정적 스위치는 이미 존재했다. YAML=true, dataclass/node 자체 기본=false였다.
  기존 정적 cost는 지수형 soft + 유한 hard penalty였으며 정적 후보를
  guide 내부 평균에서 완전히 제외하지 않았다.
- 기존 공통 경로 비용은 제곱 거리 + local endpoint 거리였고, 속도에는
  횡방향 경로 복귀속도 항이 있었다. 새 objective에서는 이를 대체했다.
- 기존 공유 사각형 MC는 원 내부 표본 수로 나눈 조건부 평균을 사용했다.
  이는 내부 표본이 있을 때 별도의 조건부 추정으로 해석할 수 있지만,
  빈 영역을 0으로 처리했고 요청서의 사각형 추정식과 달랐다.
- 설치된 `amr_interfaces`에는 소스의 `sensor_watermark`가 없었다.
  먼저 `/tmp/amr-guide-validation/install`에서 현재 메시지를 빌드해 검증했다.
  실행 중인 planner/구동계는 이 작업에서 시작하지 않았다.

## 비용과 코드 대응

모든 전략의 실제 평가 진입점은 `MppiController._evaluate_rollouts`다.
`common_cost.py`가 동일 경로/heading/motion 계산을 후보와 대표에 적용한다.
기존 private 분석 helper들은 호환용으로 남았지만 공통 objective에 더하지 않는다.

| 요청 항 | 현재 코드/식 | 기존 항 처리 |
|---|---|---|
| 경로 복귀 | `common_costs`: `w_e mean(rho(e/d_ref))` | 제곱 경로 비용 대체 |
| 진행 | `-w_p (s_N-s_0)/ell_ref` | 새 항 |
| 실제 목표 | `w_goal * ||p_N-P(S)||²`를 현재 remaining arc와 목표 Euclidean 거리 모두 `goal_activation_distance` 이하일 때만 적용 | 먼 local endpoint 벌점 제거 |
| 정적 | `_static_obstacle_cost`: `w_s mean(max(0,1-d/d_safe)²)` + 충돌 시 `inf` | 지수 비용/유한 hard penalty 대체 |
| 동적 | `dynamic_risk_cost`: `w_d mean(CP) + w_h count(CP>epsilon)` | 기존 soft sum을 mean으로 변경 |
| Heading | `w_psi mean(wrap(psi-atan2(P(s+L)-P(s)))²)` | 로봇 기준 LOS를 사용하지 않음 |
| 경로 방향 속도 | `w_v mean(((v_parallel-v_ref)/v_scale)²)` | 정규화 적용 |
| 경로 법선 속도 | `w_perp mean((v_perp/v_scale)²)`, 초기 `w_perp=0` | 횡방향 복귀속도 벌점 제거 |
| 회전/입력 변화율 | `w_omega mean((omega/omega_scale)²) + mean(sum(w_a*(du/dt/a_scale)²))` | `MppiCostTerms.rotation`에 함께 기록 |

`cost.path_loss`는 `huber`/`quadratic`을 지원한다. quadratic은 같은 정규화
입력의 `.5*z²`이며 Huber 우월성을 주장하지 않는다.
`static_cost.decay`는 이제 `d_safe`[m]다. `static_cost.hard_penalty`,
`cost.cross_track_gain`, `cost.max_cross_track_speed`는 YAML 호환용이며 새 비용에
더하지 않는다. 새 YAML 가중치와 정규화 상수는 **미튜닝 초기값**이다.

Rollout 배열은 `x_1..x_N`, 입력은 `u_0..u_(N-1)`이다. 경로/정적/동적/heading
비용의 평균은 N개 미래 상태를 사용한다. World velocity는 Euler rollout과 같이
입력을 적용하기 전 `x_0..x_(N-1)` yaw로 회전한다. 접선/법선도 이 시각의
projection을 사용한다. `u_-1`은 실제 측정 body twist다. 입력 차이는 world
물리 가속도라고 부르지 않는다.

`v_ref=min(desired_speed, sqrt(2*a_min*remaining), remaining/dt)`이며 remaining은
남은 호 길이와 실제 종점 거리 중 큰 값이다. 충돌 검사와 입력 제한이 우선한다.
최종 goal yaw는 기존 15cm terminal 진입/25cm 이탈 hysteresis와 별도 pose
alignment에 연결된다. Alignment에도 입력 projection, 정적/동적 검사를 추가했다.
완료 tolerance/settle/latch는 기존 설정을 유지한다.

## 투영·footprint·안전 처리

- 연속 중복점을 제거하고 polyline에 투영한다. 최초에는 가장 가까운 선분,
  동률은 앞 선분이다. 이후 이전 s의 양방향
  `±max(projection_window, 2*실제 이동거리)` 범위에서 투영한다.
  후진은 s 감소로 드러난다. 같은 전체 경로에서는 cycle 간 시작 s도 유지한다.
  경로가 바뀌면 재초기화한다. 이는 지역 연속성 정책이며 모든 자기 교차에서
  의도한 분기를 알아내는 전역 경로 추적 보장은 아니다.
- Heading은 투영점에서 L만큼 앞선 경로점의 차이다. 끝점의 영벡터는 해당
  마지막 유효 선분을 사용한다. 모든 점이 같은 singleton 경로는 heading=0
  fallback이며 실제 종단 yaw는 ROS terminal controller가 처리한다.
- `robot_radius=.2m` 원을 정적/동적에 공통 사용한다. 현재 URDF chassis
  `.28584 × .18841m`, 중심 x 오프셋 `.00744m`와 바퀴 중심
  `(±.09491, ±.10352)m`, 반경 `.04m`를 보수적으로 포함한다.
  추가 장착물/적재물의 실측 footprint까지 검증했다는 뜻은 아니다.
- 정적 검사는 초기 footprint 및 step 사이를 반 cell 이하 간격으로 검사한다.
  EDT cell-centre 오차에 cell diagonal, sweep 오차에 half sample spacing을
  추가한다. Soft clearance도 같은 보수적 중심 clearance에서 robot radius를
  한 번 뺀다. 지도 경계, 밖, unknown(기본 occupied)을 포함한다.
- 기존 traversability 출력은 inflation=0이다. inflation>0 입력은 의도치 않은
  이중 팽창을 피하기 위해 명시적으로 거절하고 ON 상태에서 hold한다.
  지도에 포함된 동적 객체를 임의로 지우지 않는다.
- guide별 평균 제어열을 제약 투영 후 다시 rollout/평가하고, 대표끼리 평균하지
  않는다. Standard도 업데이트된 평균과 braking Stop을 다시 검사한다.
  mode 전략도 안전 후보가 없으면 움직이는 후보 대신 Stop 경로를 선택한다.
- Guide의 Stop은 기존 정책대로 안전한 이동 대표가 없을 때의 fallback이다.
  Stop도 현재 속도에서 제약 내 감속한 실제 이동 궤적으로 검사한다.
  모두 위험하면 `no_safe_candidate_hold`를 기록하며 안전하다고 주장하지 않는다.
  노드의 stale/예외 hold는 기존 처리이며 환경에 대해 충돌 회피를 보장하지 않는다.
- 모든 전략에 같은 convex box/wheel/rate projection을 적용한다. 측정 초기
  입력이 이미 infeasible이면 첫 projection은 wheel 제한을 우선한다.

## 동적 확률과 논문 해석

공통 사각형 B의 M개 균일/스크램블 Halton 표본에 대해 장애물별로
`area(B)/M * sum(1_C(z)*f_o(z))`를 계산한 뒤
`CP=1-product_o(1-p_o)`로 결합한다. Cache 재사용 시에는 원래 표본 영역 B의
면적을 유지한다. Gaussian mixture는 장애물 내부에서 먼저 합친다.
PDF 자체에 확률 결합식을 적용하지 않는다.

내부 표본 0개는 평가 불충분으로 marginal=1 처리한다. 비유한 적분값도
안전 0으로 바꾸지 않는다. 범위 밖 marginal은 [0,1]로 clip하고 개수를 기록한다.
비대칭/음의 고윳값 covariance는 거절하고 유효 PSD covariance만
`minimum_sigma`로 regularize한다. 따라서 검증 분포도 이 sigma floor를 고려한다.

1차 평가는 각 guide 안에서 표본을 공유하고 최종 대표들은 하나의 공통
사각형/표본으로 비교한다. 1차 guide별 지역 표본과 최종 공통 표본은 서로 다르다.
이는 모든 평가를 동일 표본으로 만든 구현은 아니며 비교 잡음이 남는다.
최종 모든 timestep의 추정 CP≤epsilon을 별도 검사하지만 시간 사이 동적
충돌, 예측 오차, 장애물 간 상관관계는 보장하지 않는다. CP의 시간 합/최댓값을
주행 전체 충돌 확률로 해석하지 않는다.

- [DRA-MPPI v1](https://arxiv.org/html/2506.21205v1)의 식 8/9/14와
  Remark 1을 확인했다. 확률 제약과 평균 후 안전 문제를 참고했으며,
  v1 식 11의 density 결합은 복제하지 않는다. Huber/진행/정적 hinge는 자체 설계다.
- [SMPPI v8](https://arxiv.org/html/2112.09988v8)의 식 15/22는 입력 변화와
  속도 목적을 참고했다. 본 구현의 정규화 및 path-frame 속도는 변형이고,
  input lifting이나 SMPPI 이론을 구현한 것은 아니다.
- [Biased-MPPI v2](https://arxiv.org/html/2401.09241v2)의 proposal/importance
  sampling 해석을 확인했다. 기존 코드는 `exp(-(J-min J)/temperature)` 가중치이며
  별도 likelihood ratio가 없었다. 이를 제거한 변경은 없다.
  제한된 spline 공간, deterministic anchor, 제약 투영, guide별 normalization과
  대표 선택을 사용하는 현재 방식은 경험적 MPPI 계열 최적화다. 이론적으로 정확한
  unbiased importance-sampling MPPI라고 부를 수 없다.

## 재현과 검증 범위

작업 중 확인한 호스트는 Intel Core 5 210H, logical CPU 12, RAM 약 7.3GiB,
Linux 7.0.0-34-generic, ROS Jazzy, Python 3.12.3이다.
원시 환경/실제 parameter dump/시작 부하·온도는 각 측정 디렉터리에 보관한다.
비교용 synthetic 입력은 calibration ground truth가 아니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
cd /home/smoc/2026amr_ws/src/amr_basic_stack
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD/planner/local_planner/DRA_MPPI:$PYTHONPATH"
python3 -m pytest planner/local_planner/DRA_MPPI/test -q
python3 planner/local_planner/DRA_MPPI/tools/validate_ros_pilot.py \
  --output /tmp/guide-pilot --obstacles 10 --cycles 30 --warmup 10
python3 planner/local_planner/DRA_MPPI/tools/validate_mc_accuracy.py \
  --output /tmp/guide-mc --halton
python3 planner/local_planner/DRA_MPPI/tools/validate_closed_loop.py \
  --output /tmp/guide-loop --rollouts 60 --seeds 3 --steps 100
```

ROS 도구는 domain 170..179(기본 173), localhost, `/guide_validation/...` 토픽,
`publish_cmd_vel=false`로 한정한다. Fixtures와 reference subscriber는 planner와
다른 프로세스다. 실제 센서/모터 노드는 시작하지 않는다. Static-only, dynamic
5/10/20, `--obstacles 20 --detour`, `--static-off`로 조건을 바꾼다.
동일 primary K 비교용으로 `--strategy standard --rollouts 500`,
`--strategy single_anchor --rollouts 500`,
`--strategy guide_conditioned --rollouts 500`을 지원한다. 최종 대표 평가 수와
각 전략의 MC 지역 배치/표본 예산 차이는 따로 기록해야 하며, 이것만으로 동일
계산 시간 비교가 되는 것은 아니다. 본 ROS pilot은 guide 운영 설정만 측정했다.

`timing.jsonl_path`가 비어 있지 않으면 매 callback을 비동기 JSONL로 기록한다.
조기 반환/exception/fallback을 구분하고 callback/optimizer/정적/MC/대표 재평가,
발행 간격, 예상 schedule 대비 지연/초과를 남긴다. Subscriber 수신 간격은 별도
파일이다. 2초 요약 로그를 p99 표본으로 쓰지 않는다.
callback 시간은 recorder 준비/queue 호출 직전까지이며 `record_prepare_ms`를
별도 기록한다. 기록 부하 ON/OFF 비교는 미측정이다.
Jazzy 설치본에 TimerInfo가 없어 timer 시작 phase를 monotonic clock에 고정해
예상 deadline을 계산한다. 이는 OS/kernel의 exact timer dispatch trace가 아니다.
기존 ROS timer의 sim-time 동작은 유지하고 sim time 사용 시 wall schedule
지연/deadline은 null로 표시한다. Sim 재생속도에 대한 별도 실험은 하지 않았다.

측정 수치는 [검증 결과](validation_2026-10-20/results.md)에 정리했다.
본 문서의 소스 및 원시 데이터와 과거 README의 개발 PC 수치를 구분해야 한다.

## 남은 본 검증

현재 완료 범위는 구현, 회귀 테스트, 짧은 격리 ROS pilot, 고정 분포 적분 검증,
소규모 kinematic 폐루프 pilot이다. 다음 항목은 **미완료/미측정**이다.

- 조건별 워밍업 후 10,000 timer cycle 본 측정: 5Hz이면 조건당 33분20초.
  5조건×planner 단독/실제 동시 노드 부하 2종은 최소 5시간33분20초,
  static ON/OFF까지 전 조건으로 늘리면 최소 11시간6분40초다.
  도구는 `--cycles 10000 --warmup 50`을 지원하지만 본 작업에서 실행하지 않았다.
- 실제 가능한 perception/MPC/전체 stack 동시 부하, bag 재생, 장시간 온도/CPU 추세,
  logger ON/OFF 부하 차이, 예외 주입의 장시간 fallback 통계.
- Layer B: 시각/horizon/객체 ID로 연결된 예측과 미래 관측 데이터가 없어
  coverage, reliability diagram, Brier/log loss, 보정/평가 데이터 분리 모두 미검증.
- Layer C: 실제 하위 MPC/구동 모델을 포함한 충돌률 신뢰구간, 도착/교착 시간,
  deadline miss와 위험의 관계. 작은 합성 scene의 충돌 0건은 안전 증명이 아니다.
- Guide 효과의 본 비교: 동일 시간 예산, white-noise/anchor-only 추가 ablation,
  tuning/evaluation 환경 분리 및 동일 tuning budget, 충분한 독립 환경의 paired CI.
  현재 3개 고정 환경과 seed 반복만으로 guide 우월성을 결론내릴 수 없다.
- 장착물 footprint 실측, sigma/가중치 tuning, 추정 오차에 대한 위험 margin.
  공유 영역이 커질 때 MC 오차가 크게 늘어나는 것을 확인했으며 현 epsilon이
  실제 충돌 위험의 보증값이라는 해석은 허용되지 않는다.
