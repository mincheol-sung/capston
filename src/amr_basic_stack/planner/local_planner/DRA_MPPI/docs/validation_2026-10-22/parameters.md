# 최종 YAML 파라미터와 선정 범위

최종 파일: `../../config/dra_mppi.yaml`. 아래 값은 이 합성 검증에서 선정한
실험 설정이다. **전체 5 Hz 요구를 충족하는 운영 최적값으로 검증된 값은 아니다.**
원본 YAML은 `baseline/source_before.tar.gz`에 보존했다. ε와 하드웨어/안전 제한은
완화하지 않았다. 비용은 모두 raw SI 단위이며 J를 무차원으로 보면 아래 비용 가중치의
단위는 각 raw 항 단위의 역수다.

| 항목/정확한 설정 키 | 시작값 | 최종값 | 단위 | 시험/유효 범위 및 이유 |
|---|---:|---:|---|---|
| mppi.strategy | guide_conditioned | guide_conditioned | enum | standard 보존, 요청 기본 전략 |
| mppi.dt / horizon_steps | .2 / 10 | .2 / 10 | s / step | 2초 horizon 고정 |
| controller_frequency | 5 | 5 | Hz | 예산 200ms 고정 |
| mppi.guide_num_rollouts | 500 | 100 | 전체 후보 수 | 100/250/500 시험. 100으로 튜닝 7상황 모두 도달; 고정밀 MC 계산량을 줄이는 시험 범위의 최소값 |
| guide별 배분 | 100×5 | 20×5 | 후보 수 | 전체 K 유지, 대표 5개+Stop의 별도 재검사 비용은 추가 |
| mppi.num_rollouts (standard) | 321 | 321 | 후보 수 | 기본 보존. 공정 비교 실행은 --rollouts 100으로 명시 |
| mppi.guide_angles_deg | [0,45,90,-45,-90] | 동일 | deg | 기존 guide 방향 보존 |
| mppi.guide_noise_std | [.15,.12,.45] | 동일 | [m/s,m/s,rad/s] | 별도 noise grid 탐색 없음. 3축 자유도와 yaw 회복/우회 기능 검증, 보정된 최적 분산이라는 주장 없음 |
| mppi.spline_knots | 4 | 4 | knot 수 | 기존 spline 상관 구조 유지, 광범위 knot 튜닝 미실시 |
| mppi.guide_anchor_blend | .25 | .25 | 비율 | nominal .75 + anchor .25, 기존 warm start 유지 |
| mppi.temperature | 2 | 2 | cost 단위 | adaptive 로직 유지, 온도 최적화 grid 미실시 |
| temperature eta 범위 | [5,10] | [5,10] | sum of unnormalized weights | 감소 .9/증가 1.2 유지. K 감소 후 상대적인 유효 표본 비율 영향은 추가 운용 검증 필요 |
| mppi.guide_switch_penalty / hysteresis | .25 / .10 | 동일 | cost | 기존 이동 guide 전환 억제 유지. Stop에는 penalty 없음 |
| mppi.guide_evaluation | MC만 split | split | enum | guide 전체 작업 분할. batch는 A 재현용 |
| mppi.guide_timeout | 없음 | .18 | s | 전체 200ms 예산에서 ROS 전후처리 여유. thread drain으로 초과할 수 있으며 timeout은 hold로 보고 |
| cost.tracking_path_weight (w_e) | 8 (Huber, scale .25) | 64 | 1/m² | 새 제곱식에서 8/32/64 시험. K100 튜닝 평균 경로 오차 .142/.068/.060m, 모두 7/7 도달 |
| cost.progress_weight (w_p) | 8, scale 1 | 8 | 1/m | 진행 보상 유지, 실제 s 감소도 반영 |
| static_cost.soft_weight (w_s) | 8 | 8 | 1 | 보수적 clearance 및 hard invalid와 함께 기존 값 보존 |
| dynamic_risk.soft_weight (w_d) | 18 | 18 | 1 | 시점 확률 평균 비용, 위험 허용 완화 없음 |
| dynamic_risk.hard_penalty (w_h) | 100000 | 100000 | 1 | 초과 timestep 수 비용. 최종 대표는 비용과 별개로 배제 |
| cost.heading_weight (w_psi) | 3 (각도 제곱) | 3 (1-cos) | 1 | 함수 변경으로 수치 영향이 달라짐. 초기 ±90/180° 회복은 기능 시험; heading weight grid 미실시 |
| cost.speed_weight (w_v) | 1, speed_scale=.75 | 1.78 | s²/m² | 기존 유효 계수 1/.75²≈1.778을 raw SI 값으로 이전 |
| cost.rotation_weight (w_omega) | 1.5, omega_scale=3 | .167 | s²/rad² | 기존 유효 계수 1.5/3²≈.1667을 이전. 회복 동작 기능 시험 |
| cost.input_delta_weight (r_x,r_y,r_omega) | rate_weight [.1,.1,.1], /dt/scale | [.5,.5,.055] | [s²/m²,s²/m²,s²/rad²] | 기존 .1/(dt²*[2.25,2.25,6.75]²)≈[.494,.494,.0549]를 변화량 계수로 이전. 새 식에는 /dt 없음 |
| cost.tracking_goal_weight | 24 | 24 | 1/m² | 시작 상태가 목표 근처일 때만 별도 terminal 항; 중복 보상 범위 문서화 |
| static_cost.decay (d_safe) | .10 | .10 | m | 선호 여유. robot radius/하드 여유와 별개, 최적 실측 값은 아님 |
| cost.heading_lookahead (L) | .20 | .20 | m | 경로 자체 방향, 끝점 유효 접선 fallback. 변경하지 않음 |
| cost.desired_speed | .75 | .75 | m/s | 공통 목표; cycle 시작 remaining과 시간에 따라 감속, 후보별 속도에 맞춰 낮추지 않음 |
| cost.goal_activation_distance | .5 | .5 | m | retained terminal 활성 범위 |
| cost.projection_window | .5 | .5 | m of arc | 양방향 local projection, 최소 창; 실제 이동거리의 2배로 확장 가능 |
| dynamic_risk.mc_sample_density | 124 | 4096 | samples/m² | 512/2048/4096 제한 탐색. 2048은 별도 seed max 오차 .01838로 사전 .015 기준 실패 |
| dynamic_risk.mc_samples_min/max | 512/4096 | 4096/131072 | samples/step | common 영역. 최종 4096 밀도와 바닥값을 새 seed 200..215로 검증 |
| dynamic_risk.guide_mc_sample_density | 124 | 4096 | samples/m² | 모든 guide 같은 정확도 기준 |
| dynamic_risk.guide_mc_samples_min/max | 128/1024 | 4096/131072 | samples/guide/step | 총 표본 면적 합은 오히려 증가 가능, 비교량/메모리 별도 기록 |
| dynamic_risk.final_mc_sample_density | 124 | 4096 | samples/m² | 대표 5개+Stop 공통 영역 재검사, 영역 밖 캐시 무조건 재사용 없음 |
| dynamic_risk.final_mc_samples_min/max | 256/4096 | 4096/131072 | samples/step | 표본 부족으로 안전 판정하지 않도록 증가 |
| dynamic_risk.mc_reject_underresolved | 없음 | true | bool | cap이 요구 밀도를 못 채우면 해당 step p=1로 배제 |
| dynamic_risk.guide_parallel_workers | 2 (MC만) | 2 (전체 guide) | worker 수 | 1/2/5 비교. 장애물10개 ROS p99에서 2가 가장 낮았으나 모두 deadline 미달 |
| BLAS/OMP/MKL/NUMEXPR | launch 1 | 모두 1 | thread 수 | 명시적 pool과 중첩 방지. 다중 BLAS thread 생산 설정은 선택하지 않음 |
| dynamic_risk.hard_threshold (ε) | .05 | .05 | 시점별 모델 확률 | 보존. 실제 사고 위험 승인 기준으로 검증된 값 아님 |
| robot_radius / obstacle_radius fallback | .2 / .25 | 동일 | m | obstacle별 유효 반경 사용, 추가 장착물 실측 미완료 |
| limits.* / static_cost.enabled | 기존값 / true | 동일 / true | 각 설정 단위 | axis, acceleration, wheel L1와 정적 하드 검사를 완화하지 않음 |

튜닝은 seed 3의 open/offpath/initial yaw/curve/detour 장면으로 제한했다.
최종 기능 평가는 seed 101..103 및 20개 조건이다. MC의 처음 분리한 평가 seed
100..115에서 2048 밀도 실패를 발견한 뒤, 4096을 **새 seed 200..215**로 다시 평가했다.
목적함수 전 계수·분산·knot·blend·temperature의 공동 최적화는 하지 않았다.
따라서 표의 보존 값은 검증된 단일 최적값이 아니라, 변경 범위를 제한한 초기/이전 값이다.
