# Gap별 도달점·비용 최적화와 샘플 배분: 실제 bag 검증

2026-11-05. **교수님 제안을 구체화한 오프라인 프로토타입**이다. 운영 planner와 YAML은 변경하지 않았다. 로봇 명령이나 ROS 메시지를 발행하지 않았다.

원본 MCAP 3개에서 기존 10장면과 동적 예측이 있는 6장면을 직접 다시 읽었다. 기존 10장면의 grid와 전·후방 scan ranges/valid/origins/hits가 이전 결과와 일치함을 확인했다. 새 방식과 기존 gap 방식을 각 장면 워밍업 1회 + 측정 12회씩 실행했다. 방식별 측정 192회다. 동적 장면에는 기록된 1~4개 객체의 예측을 실제 node의 정렬·필터 함수로 처리하여 최종 MPPI에 공급했다.

**결론:** gap별 최적화와 비용별 샘플 배분은 실제 데이터에서 작동했다. 새 방식 계산 시간은 평균 116.8ms, p95 171.8ms, 최대 183.0ms였다. 200ms 초과는 없었지만, 현재 내부 180ms 예산을 2회 초과했고 ROS 입력 처리 등을 제외했으므로 **안정적인 전체 5Hz를 검증한 것은 아니다.**

## 그림

- [전체 flow](00_flow.png)
- [실제 16장면의 seed와 샘플 수](01_all_16_scenes.png)
- [전체 계산 시간](02_timing.png)
- [교수님 예시와 같은 실제 결과: b2_060](scenes/b2_060.png)
- [이전에 설명하던 장면: b2_100](scenes/b2_100.png)
- [360도 열린 구역: b1_066](scenes/b1_066.png)

`scenes/`에 16개 장면별 PNG가 있다. 각 장면의 x축은 목표 방향, y축은 비용이며, 아래에 실제 정수 샘플 수를 표시했다. 빈 점은 공간 제한 없는 도달점의 비용, 채운 점은 실제 d를 적용한 도달점의 비용이다. 연결된 매끄러운 연속 비용곡선을 구한 것은 아니고 선택된 방향들에서 계산한 이산 값이다.

## 확정해서 검증한 흐름

1. 기존 전·후방 LiDAR + 지도 융합, 몸체 진입 깊이 d(theta), 거리 급변 기반 gap 분할을 유지한다.
2. 일반 gap은 각도 중앙 방향 e_g를 사용한다. 360도 전체가 한 구역이면 기하학적 gap은 하나로 유지하고 -180/-90/0/90도의 네 방향을 탐색한다. 이는 이번 프로토타입의 방향 배치 규칙이며 최적 각도 배치를 검증한 것은 아니다.
3. 각 방향으로 **도달 거리 최대화**를 수행한다. vx, vy, omega를 함께 결정하지만, 이 단계 목적함수에는 경로 heading 비용이 없다. 현재 yaw와 회전에 따른 운동학 및 wheel 제한은 포함한다.
4. 그 도달점을 목표로 **공통 비용 최소화**를 수행한다. 이 단계에는 heading 비용이 들어간다. 최대 도달점으로부터 종단 방사 거리 1cm 이내의 오차를 허용한다.
5. 수렴·구동 제약, 정적 swept-body 충돌, nominal 중복을 검사한다. 낮은 비용 순으로 상호 중복을 제거하고 최대 3개를 선택한다. 높은 비용이라는 이유만으로 전방/후방 방향을 하드 컷하지 않는다.
6. 선택된 후보의 **실제 공간에서 나온 비용**으로 총 500개 안에서 배분한다. 최종 MPPI는 기존 정적·동적 충돌 평가를 유지한다.

앞선 대화의 “목표 거리까지 비용으로 함께 선택”하는 안은 별도 대안이다. 그것은 최대 도달점에 반드시 도달할 필요가 없다. 이번 주 결과는 교수님 예시에 맞춰 **도달점을 먼저 구하고 그 점에 도달하는 비용을 평가하는 2단계 방식**이다. 거리 자유 최적화 대안의 초기 결과는 `joint_distance_variant/`에 보존했고, 주 결과와 섞지 않았다.

## 도달점 계산과 heading

현재 로봇 중심을 p0=0, gap 중앙 단위벡터를 e_g, 수직 단위벡터를 n_g라 한다. N=10, dt=0.2초, T=2초다.

먼저 아래 문제를 수치적으로 푼다.

```text
maximize     rho = e_g^T p_N
subject to   n_g^T p_N = 0
             rho >= 0.25m
             e_g^T p_t <= d_g, t=1..N
             actual nonlinear mecanum rollout
             per-axis velocity bounds
             per-axis acceleration bounds from current feasible input
             |vx_t| + |vy_t| + 0.19835 |omega_t| <= 0.78
```

현재 측정 입력이 제약을 벗어나면 기존 controller와 동일하게 축·wheel 제한으로 투영한다. 중간 궤적은 곡선일 수 있다. 위의 방사 투영 제약만으로 전체 궤적의 정적 충돌 회피를 보장하지 않으므로 별도 몸체 충돌 검사를 반드시 수행한다. 마지막 정지 조건은 넣지 않았다.

SLSQP와 비선형 운동 모델을 사용하므로 얻는 것은 **제약을 만족하는 국소적인 최대 도달 거리의 근사**다. 전역 최대 보장은 없다. 이전에 그렸던 omega=0 또는 저장 nominal omega 고정 LP 결과와 조건이 다르다. 회전은 최대 도달 거리에 도움이 되는지에 따라 결정되며, 목표 방향을 향해 차체를 돌리라는 강제 조건은 없다.

## 비용 구성

모든 gap에 공통인 진행 기준점 p_ref는 현재 경로 진행 위치에서 desired_speed*T만큼 앞선 경로점으로 정하고 경로 끝에서 제한한다. 현재 desired_speed=0.75m/s다. heading 기준 psi_ref,t도 공통 경로 진행 시간표에서 미리 정한다. 후보가 짧아졌다는 이유로 이 기준들을 줄이지 않는다.

```text
J_g(U) = 8 ||p_N - p_ref||^2
       + 4/N sum_t (n_g^T p_t)^2
       + 3/N sum_t (psi_t - psi_ref,t)^2
       + 1/N sum_t Delta_u_t^T diag(0.5, 0.5, 0.055) Delta_u_t
```

- 진행: 같은 공통 기준점까지 얼마나 나아갔는지 평가한다.
- 직진성: 해당 gap 방향의 직선에서 벗어난 정도다. 뒤쪽 gap에도 대칭 적용한다.
- heading: 공통 경로의 차체 방향을 따른다. gap 방향으로 차체를 강제 정렬하지 않는다. 실험에서는 연속적으로 펼친 동일 각도 branch를 사용했다.
- 입력 연속성: 첫 입력은 현재 측정/투영 입력과 비교한다. omega 자체를 무조건 벌점 처리하는 항은 없다.
- 정적·동적 장애물 비용은 이 값에 넣지 않는다. d 및 충돌 검사와 최종 MPPI의 장애물 비용은 유지한다.

가중치는 첫 프로토타입 설정이며 최적 튜닝을 입증하지 않았다. 잔차 제곱항이지만 cos/sin 운동 모델 때문에 제어열에 대한 단순 convex QP는 아니다. 분석적 Jacobian을 제공한 SLSQP, 단계별 maxiter=45, ftol=1e-6을 사용했다. 두 단계 모두 수렴하고 제약 오차 <=1e-5인 후보만 선택 가능하다. 종단 조건은 n_g^T p_N=0, rho_target-0.01 <= e_g^T p_N <= rho_target이며 최소 이동 0.25m도 유지한다.

raw cost를 거리로 나누거나 gap마다 다른 기준으로 정규화하지 않는다.

## 샘플 배분

현재 nominal과의 중복 기준 RMS<0.22m AND 마지막 3점 평균<0.30m, 후보 상호 RMS<0.25m를 유지했다. nominal이 정적 충돌하면 nominal 중복 제외를 하지 않는다. 선택된 후보 수를 G라 하면:

```text
nominal = 200 samples
gap pool = 300 samples
minimum per gap = 30 samples
tau = max(1, median(V_g - min(V)))
P_g = 0.2/G + 0.8 exp(-(V_g-min(V))/tau) / sum_j exp(-(V_j-min(V))/tau)
N_g = 30 + integer_apportionment((300 - 30G) P_g)
```

소수점 잔여가 큰 순서로 남는 정수를 배분하여 정확히 500개를 유지한다. 추가 후보가 없으면 nominal에 500개 모두 배분한다. 낮은 비용의 후보가 더 많은 샘플을 받으며, 높은 비용의 선택된 후보에도 최소 탐색량이 남는다. 선택되지 않은 방향까지 최소 샘플을 보장하지는 않는다.

**nominal 200 / gap pool 300은 아직 임시 배분 정책**이다. gap이 하나뿐이면 그 후보에 300개가 배분되므로, nominal보다 가치가 낮은 후보에 지나치게 배분할 수 있다. 운영 적용 전 nominal의 배분도 공통 평가값을 이용해 조절할 필요가 있다. 이번 측정은 이 정책을 그대로 사용한 결과다.

## 실제 값: b2_060

bag `mpc_chain_20260923_135606`, offset 129.335초, 동적 예측 객체 3개.

G2(28도)의 공간 제한 없는 도달점 근처에서 최적화한 종단 거리는 약 1.424m, 비용은 1.292였다. 실제 d=0.680m를 적용한 목표점에서는 종단 약 0.680m, 비용 5.317이었다. 샘플 배분에는 **5.317**을 사용했다. 최대 도달점과 실제 비용 최적화의 종단 거리는 1cm 허용 오차 때문에 조금 다를 수 있다.

| 후보 | 실제 공간에서의 비용 | 배분 |
|---|---:|---:|
| nominal | gap 비용으로 순위 비교하지 않음 | 200 |
| G1 | 3.055 | 167 |
| G2 | 5.317 | 89 |
| G3 | 45.085 | 44 |
| G4 | 51.379 | 0 — 최대 3개 제한 |

다만 서로 다른 고정 도달점의 비용을 비교하므로 “거리가 짧아지면 반드시 비용이 증가한다”는 정리는 성립하지 않는다. 목표와 반대 방향으로 멀리 가는 경우에는 짧아지면서 비용이 낮아질 수 있다. 원래 b2_100의 전방 후보들이 이 경우에 해당한다. 실제 비용값 그대로 비교했다.

## 결과와 한계

16장면에서 51개 방향을 평가했다. 46개에서 두 단계 모두 수렴했고, 5개는 비용 최소화가 반복 제한 내 수렴하지 않아 제외했다. 최대 거리 단계는 51개 모두 수렴했다. 후보 판정은 선택 35, nominal 중복 5, 비용 최적화 미수렴 5, 3개 용량 제한 5, 정적 충돌 1이었다.

두 방식 모두 측정된 192회 최종 결과는 기존 정적·동적 안전 검사에서 `ok`였다. 이는 기록된 snapshot에서의 검사 결과이며 실제 운행 안전성이나 충돌률을 증명하는 것은 아니다.

장면당 첫 측정의 기존 최종 MPPI 비용을 비교하면 새 방식이 낮은 장면 6개, 높은 장면 4개, 거의 같은 장면 6개였다. 특히 b2_079는 기존 11.503에서 새 방식 18.612로 나빠졌다. **좋은 방향에 샘플을 더 주는 아이디어가 모든 장면의 최종 성능 개선을 보장하지 않는다.** seed 생성, 후보 선택, 샘플 배분을 함께 바꾼 비교이므로 각 요소의 단독 효과도 분리되지 않았다. 고정 nominal 예산과 비용 가중치의 추가 검증이 필요하다.

## 5Hz 측정

| 방식 | 평균 | p95 | p99 | 최대 | >180ms | >200ms |
|---|---:|---:|---:|---:|---:|---:|
| 기존 gap | 82.6ms | 134.9ms | 145.5ms | 147.9ms | 0/192 | 0/192 |
| 도달점+비용 최적화 | 116.8ms | 171.8ms | 178.6ms | 183.0ms | 2/192 | 0/192 |

장면별 p50 기준으로 geometry 약 29~44ms, 도달점/비용 최적화·선택 약 13~95ms였다. 동적 객체가 있는 경우 최종 MPPI 부분은 약 68~78ms였다. b2_100은 6개 후보의 도달점/비용 계산에 약 95ms를 썼다. gap 수가 더 많은 장면에 대한 최악 실행 시간 상한은 없다.

환경: Intel Core 5 210H, OPENBLAS_NUM_THREADS=1, OMP_NUM_THREADS=1. 세부 Python/NumPy/platform 및 전체 설정은 configuration.json에 기록했다. persistent worker/sampler pool을 재사용했고 두 방식의 측정 순서를 번갈아 실행했다. 초기화와 워밍업은 집계에서 제외했다. 실행 실패로 시간을 잘라버리지 않도록 **오프라인 측정 timeout을 10초로 완화**했다. 운영 timeout을 변경한 것은 아니다.

포함: 실제 scan/map 융합, directional depth, gap 분할, seed 생성 또는 두 단계 최적화, 배분, 총 500개 MPPI, 기존 primary/final risk 평가 및 정적 검사.

제외: bag I/O, snapshot 추출, scan deskew/TF, ROS 콜백·출력, prediction 메시지 정렬, 인식/예측 노드 부하, 지도 최초 DistanceField 구성. 이 제외 항들은 원본 bag에서 데이터 준비 때 수행했지만 주기 측정에는 포함하지 않았다. 16개 독립 snapshot을 반복한 것이며 시간에 따라 이어지는 closed-loop replay가 아니다.

**판정: core 계산은 이번 데이터에서 200ms 이내였으나, 현재 내부 180ms 예산조차 2회 초과했다. 전체 ROS 5Hz 충족을 확정하거나 현재 상태 그대로 운영 적용을 권할 근거는 부족하다.**

운영 전 필요한 다음 조치는 후보 최적화 전체의 wall-time 예산과 안전한 미수렴 fallback, 인접 주기의 해 재사용, nominal 배분 보완, 그리고 실제 ROS 입력/출력 포함 deadline 검증이다. 이들은 이번 프로토타입에는 구현하지 않았다. 단순히 deadline을 늘려 5Hz 충족이라고 처리하지 않았다.

## 재현과 검증

```bash
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
export PYTHONPATH="$PWD/planner/local_planner/DRA_MPPI:$PYTHONPATH"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
/usr/bin/python3 planner/local_planner/DRA_MPPI/tools/experiment_gap_value_optimization.py --extract
/usr/bin/python3 planner/local_planner/DRA_MPPI/tools/experiment_gap_value_optimization.py --iterations 12
/usr/bin/python3 planner/local_planner/DRA_MPPI/tools/render_gap_value_experiment.py
```

- 분석적 비용·종단 제약·공간 제약 미분을 독립 중앙차분과 대조: 최대 절대 오차 약 2.9e-9.
- 저장된 모든 seed의 위치·회전 결과를 production `rollout`과 대조했다.
- 선택된 seed의 두 단계 수렴, 제약 오차, 정적 충돌, 1cm 종단 허용 오차를 확인했다.
- 모든 측정의 실제 MPPI rollout 배분 합계가 500임을 확인했다.
- 모든 `ok` 최종 궤적에 기존 정적 swept-body 검사와 동적 hard threshold 검사를 다시 확인했다.

`raw_sources.json`: bag 경로, 크기, record_ns/offset, 예측 개수, 이전 cache 일치 여부.
`raw/*.npz`: 원본 bag에서 이번에 추출한 입력·정렬된 예측.
`scene_results.json`, `*_summary.json`: 방향별 도달점·비용·선택·시간.
`timings.json`: 384개 실제 측정 기록(두 방식 각각 192개).
`*_results.npz`: 실제 seed 제어열·궤적과 최종 선택 궤적.

비용 수치와 시간은 설정·하드웨어·장면에 종속적이다. 이 결과를 임의로 다른 로봇이나 환경의 보장으로 사용하지 않는다.
