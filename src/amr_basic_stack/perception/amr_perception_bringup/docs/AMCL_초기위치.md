# AMCL 전역 측위로 cartographer 초기 위치 잡기

RViz "2D Pose Estimate" 를 손으로 찍는 대신 쓴다.

## 실행

```bash
ros2 launch stack_master autonomy.launch.py map:=<맵이름> use_amcl_bootstrap:=true
```

또는 단독으로:

```bash
ros2 launch amr_perception_bringup amcl_bootstrap.launch.py
```

**로봇을 몇 미터 주행시켜야 한다.** AMCL 은 `update_min_d`/`update_min_a` 를
넘는 이동이 있어야 파티클을 갱신한다. 제자리 회전만으로는 부족하다 — 대칭적인
복도는 서로 다른 지점에서 본 관측이 쌓여야 모호성이 풀린다.

## 신호 흐름

```
map_server /map ─┐
                 ├─► amcl (tf_broadcast: false) ─► /amcl_pose
/scan ───────────┘                                    │
                                                      ▼
/odometry/filtered ─► amcl_bootstrap_node ─► /initialpose ─► set_initial_pose_node
                      (수렴 판정 + 정합 검증)                   └► FinishTrajectory
                                                                  StartTrajectory
                                                                  ekf_carto set_pose
```

## 두 개의 관문

### 1. 공분산 + 이동 거리

`/amcl_pose` 의 x, y, yaw 표준편차가 기준 아래로 연속 N 회, 그리고 그 전에
로봇이 `min_travel` 이상 움직였을 것.

이동 거리를 같이 보는 이유: AMCL 은 움직여야 파티클을 갱신하므로, 가만히 있을
때 공분산이 작은 것은 근거가 아니다.

### 2. 스캔-지도 정합 (핵심)

**공분산만으로는 부족하다.** AMCL 은 대칭적인 실내에서 자신 있게 틀린 곳에
수렴한다. 그래서 수렴한 pose 에 현재 스캔을 얹어 보고, 끝점이 지도의 점유
셀에 실제로 붙는 비율을 센다.

2026-09-09 실측 (test_imu_scan, `match_distance` 0.20 m, 180 빔):

| 후보 pose | 정합 점수 |
|---|---|
| 정답 | **1.000** |
| 10° 회전 | 0.717 |
| 0.3 m 이동 | 0.494 |
| 1 m 이동 | 0.256 |
| 180° 뒤집기 | 0.200 |

`min_match_score: 0.55` 면 0.3 m / 10° 수준까지만 통과한다.

`match_distance` 를 키우면 구분력이 급격히 떨어진다. 지도에서 "장애물 근처" 인
셀의 비율이 0.05 m 에서 5.7 %, 0.10 m 에서 12.1 %, 0.20 m 에서 23.2 % 이고,
이 비율이 곧 엉뚱한 pose 의 기저 점수다.

검증에 실패하면 전역 측위를 다시 걸고(`max_retries` 회), 다 쓰면 직접 찍으라고
안내한다.

## 주의

- **AMCL 은 `map -> odom` TF 를 내지 않는다** (`tf_broadcast: false`).
  그 TF 의 주인은 `ekf_carto` 다. 둘이 같이 내면 TF 트리가 깨진다.
- 모션 모델은 `nav2_amcl::OmniMotionModel` 이다. 메카넘휠은 측방 이동을
  하므로 `differential` 을 쓰면 파티클이 잘못 퍼진다.
- 기동 시 `New publisher discovered on topic '/initialpose', offering
  incompatible QoS` 경고가 뜬다. 정상이다 — AMCL 이 자기가 낸 초기 위치를
  되먹지 않도록 BEST_EFFORT 로 내보내기 때문이고, `set_initial_pose_node` 는
  BEST_EFFORT 구독자라 정상 수신한다.

## 검증 (하드웨어 없이)

```bash
bash $(ros2 pkg prefix amr_perception_bringup)/share/amr_perception_bringup/tools/verify_amcl_bootstrap.sh
```

`fake_robot.py` 가 지도 위를 실제로 주행하며 스캔을 만들고, `/ground_truth` 와
비교해 오차를 잰다. 2026-09-09 실측: 8.6 m 주행 후 수렴, **위치 오차 0.122 m,
방향 0.4°**, 정합 1.00.

단위 테스트 21개 (`test/test_amcl_convergence.py`).
