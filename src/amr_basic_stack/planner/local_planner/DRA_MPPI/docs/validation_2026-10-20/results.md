# 검증 결과 — 2026-10-20

새 비용의 검증 자료다. 과거 개발 PC 실험과 합산하지 않았다.

## 테스트와 빌드

- 회귀 테스트: **173 passed**. [pytest.txt](pytest.txt)에 원시 결과를 저장했다.
- 현재 메시지 소스와 planner의 실제 workspace 빌드 성공: [build.txt](build.txt).
- 설치된 node의 parameter ON→OFF→ON을 별도 domain 174에서 확인했다. Timer를
  취소하고 spin하지 않아 reference/구동 명령은 발행하지 않았다.
- 실제 구동 명령 시험은 실행하지 않았다.

## 실제 ROS DDS pilot (본 p99 측정 아님)

각 조건 워밍업 10회 + 측정 30회. K=500/N=10/dt=.2, guide 5개,
MC worker 2개, 지도 .05m/200×200, 시각화 32개. 예측 객체당 CV/CT 2개 mode.
조건은 순차 실행했고 인지 추론/MPC 노드는 동시 실행하지 않았다. Synthetic fixture와
독립 subscriber만 별도 프로세스로 동작했다. Planner 실제 입력/출력 DDS를 경유했다.
경로는 일반 조건 2점, 우회 조건 4점이다. 긴 실제 global path의 투영 비용과
센서 TF 변환 부하는 이 pilot으로 검증하지 않았다.

| 조건 | callback p50 | p95 | p99 | max | optimizer p99 | receive interval p99 |
|---|---:|---:|---:|---:|---:|---:|
| [static](static/summary.json) | 14.64 | 15.84 | 16.03 | 16.09 | 13.24 | 200.70 |
| [dynamic5](dynamic5/summary.json) | 34.46 | 35.95 | 36.50 | 36.72 | 33.60 | 201.83 |
| [dynamic10](dynamic10/summary.json) | 36.32 | 37.58 | 37.73 | 37.77 | 34.06 | 206.03 |
| [dynamic20](dynamic20/summary.json) | 39.80 | 41.31 | 49.10 | 52.21 | 36.42 | 201.21 |
| [detour20](detour20/summary.json) | 39.15 | 45.76 | 49.93 | 51.41 | 42.06 | 207.59 |
| [static_off](static_off/summary.json) | 9.62 | 10.78 | 10.92 | 10.97 | 8.90 | 201.09 |

단위 ms. 각 측정 구간 normal optimizer 표본은 30개, fallback/예상 deadline miss는
모두 0/30, recorder drop=0이었다. 6개 조건에서 각 40개 cycle 행을 기록했다.
수신 구간 내 message 수는 조건마다 30개였다. 시작 지연/발행 간격/조기 반환은 각
`cycles.jsonl`, `receive.jsonl`, `summary.json`에 있다. 200ms보다 조금 긴 발행/수신
간격이 관찰된다. optimizer p99<200ms는 정확한 5Hz 발행 보장이나 worst-case 보장이 아니다.

동적 20개 조건의 구간별 시간과 실제 MC 예산:

- `static_ms`: median=4.929, p99=5.121.
- `primary_mc_ms`: median=18.836, p99=19.373.
- `final_mc_ms`: median=4.193, p99=4.950.
- `representative_ms`: median=5.175, p99=5.934.
- `actual_primary_mc_samples`: median=12665.500, p99=13480.590.
- `record_prepare_ms`: median=0.007, p99=0.010.

구간은 중첩된다. 예를 들어 representative 시간에 final MC와 static이 포함되므로
서로 더해 전체 callback 시간으로 해석하면 안 된다. 기록 준비 비용만 분리했고
queue/worker를 포함한 recorder ON/OFF A/B 부하 측정은 하지 않았다.

## A. 고정 분포에서의 MC 적분 정확도

원시값: [Halton CSV](mc_halton/mc_raw.csv), [random CSV](mc/mc_raw.csv).
각 방식은 16개 분포 × 3개 공유 영역 크기 × 3개 표본 수 × 12 seeds = 1,728회.
등방 Gaussian은 ncx2 CDF를 기준으로, 이방 2-mode/1·5·10·20 장애물은 독립 원형
수치적분을 기준으로 비교했다. 원형 적분 80/160 radial node refinement 차이의 최대는
1.56e-15였다. 이는 관찰한 수렴 차이이며 엄밀한 오차 상한은 아니다.
epsilon=.05 주변, sigma=.08/.2/.6, 낮은/높은 확률, 확대 영역 및 빈 영역을 포함했다.

| Halton 공유 영역 span | M | MAE | RMSE | bias | unsafe→safe 비율 | safe→unsafe 비율 |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 128 | 0.00553 | 0.01160 | -0.00155 | 3.12% | 2.08% |
| 0 | 512 | 0.00211 | 0.00377 | -0.00063 | 0.00% | 0.00% |
| 0 | 2048 | 0.00089 | 0.00146 | 0.00000 | 0.00% | 0.00% |
| 5 | 128 | 0.39702 | 0.57738 | 0.28204 | 17.71% | 17.71% |
| 5 | 512 | 0.11057 | 0.23149 | -0.06816 | 21.88% | 4.69% |
| 5 | 2048 | 0.03154 | 0.06460 | -0.01080 | 11.46% | 5.21% |
| 50 | 128 | 0.79341 | 0.84784 | 0.79341 | 0.00% | 37.50% |
| 50 | 512 | 0.79341 | 0.84784 | 0.79341 | 0.00% | 37.50% |
| 50 | 2048 | 0.64761 | 0.75495 | 0.62642 | 3.65% | 29.69% |

span은 목표 원 외의 보조 rollout 중심을 ±(span,span)에 둔 stress 값이다.
위 오판 비율 분모는 각 그룹 전체 192회이며, unsafe 조건부/ safe 조건부 비율이 아니다.
빈 영역에서는 marginal=1로 보수적으로 처리하지만 내부 hit가 조금 있는 큰 영역의
적분까지 정확해지는 것은 아니다. **span=5m, M=512에서 unsafe→safe 오판 21.88%**가
관찰되어, 현재 표본 수/영역/epsilon 조합의 실제 안전성을 보증할 수 없다.
반복 표준편차, clipping/empty 횟수와 계산 p99는 각 `mc_summary.json`에 있다.
여기서 계산 p99는 Python 적분 호출 시간이며 ROS p99와 다른 통계다.

## B. 예측 calibration

**미검증**. 예측 시각/horizon/object ID와 연결된 미래 ground truth가 없다.
coverage/reliability/Brier/log loss를 계산하거나 synthetic exact distribution의 결과로
대체하지 않았다. 보정과 평가 데이터 분리도 아직 수행하지 않았다.

## C. 소규모 폐루프 및 guide 비교 pilot

[원시 cycle](closed_loop/closed_loop_raw.jsonl), [episode 결과](closed_loop/closed_loop_summary.json).
실제 controller와 kinematic rollout을 사용했고 하위 MPC, 물리 구동, 인지 추론은
포함하지 않았다. 같은 비용/제약/N/dt와 primary K=60을 적용했다. Standard는 primary
60개 안에 Stop이 있고 guide는 최종 재평가에 Stop이 있다. 최종 재평가 후보 수는
standard/단일 anchor/guide 각각 2/2/6개다. 동일 계산 시간 예산 비교는 아니다. MC는 각 전략의 YAML 예산을 유지하여
1차/최종 적분 표본 수까지 맞춘 본 공정 비교는 아직 수행하지 않았다.

| 방법 | 성공 | 관찰 충돌 | 시도 |
|---|---:|---:|---:|
| standard | 6 | 0 | 9 |
| single_anchor | 4 | 0 | 9 |
| guide | 6 | 0 | 9 |

환경은 open/detour/crossing5 세 개, 각 3 seeds다. 독립 환경 수는 3개이며
27개의 독립 환경으로 세지 않는다. 성공은 목표 XY .15m 진입으로 정의했고 terminal
settle/yaw 도착을 별도로 입증한 결과는 아니다. 실패 시간은 20초로 포함했다.
환경별 seed 평균을 짝지은 guide-standard 실패 포함 시간 차이는 **+0.20초**,
환경 단위 bootstrap 탐색적 95% CI는 **[0.00,+0.60]초**였다. Guide 우월성을
확인한 결과가 아니다. 동적 5개 crossing 장면은 세 방법 모두 3회 실패했다.
최소 clearance 하한, 입력 변화율, 정지 시간, 위험 초과 횟수는 episode JSON에 있다.
관찰 충돌은 모두 0건이나 세 환경만으로 실제 충돌률 신뢰구간이나 안전 보장을
제시할 수 없다. 동적 충돌 체크는 cycle 내 21개 substep이고 연속시간 증명이 아니다.

## 미측정 항목

조건별 10,000회/실제 동시 노드 부하의 본 ROS p99, bag 재생, 실제 예측 calibration,
물리/MPC 폐루프 충돌률 CI, deadline miss와 충돌 관계, 같은 시간 예산 비교, 독립
튜닝/평가 환경 및 대규모 ablation은 미완료다. 재현 명령과 필요한 시간은
[구현 보고서](../guide_only_2026-10-20.md)에 정리했다.

소스 변경 diff: [source.patch](source.patch). 추가 파일을 포함하며 raw 결과 파일은 제외했다.
