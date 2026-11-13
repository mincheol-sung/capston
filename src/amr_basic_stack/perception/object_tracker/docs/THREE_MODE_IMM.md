# Semantic tracker → 3-mode IMM → DRA (2026-09-16)

## 구현 정책

1. YOLO/RGB-D는 클래스, confidence, 3D 위치를 검출한다. 카메라–LiDAR fusion은
   매칭된 객체를 FUSED(2), 카메라 단독 객체를 CAMERA(0)로 `/detections_fused`에 낸다.
   미매칭 LiDAR는 기본적으로 `/lidar_obstacles`에 분리된다.
2. Tracker는 CAMERA/FUSED의 **모든 클래스**를 받는다. person/chair라는 이름으로
   정지/이동을 미리 결정하지 않는다. 과거 bag의 LIDAR(1)가 들어와도 제외한다.
   XY/confidence가 유한해야 하며 측정 공분산은 대칭 양의 정부호여야 한다.
   공분산이 모두 0인 기존 미지정 표현만 `fallback_sigma=0.1m`로 대체한다.
   음수 비대각 원소 자체는 정상일 수 있다. 유효 검출이 0개라도 lifecycle을 갱신한다.
3. 검출 시각 TF로 위치와 공분산을 odom으로 옮긴다. 예측된 트랙과 측정을
   Mahalanobis gate + Hungarian assignment로 연결한다. CONFIRMED, TENTATIVE,
   LOST 순서로 배정해 새 조각이나 오래된 트랙이 정상 트랙의 검출을 빼앗지 않게 한다.
   최근 LOST person은 0.5s/0.6m/chi² 16 이내이고 1등 후보가 명확할 때만 복구한다.
   새 person이 확정되면, 그 트랙이 생기기 전부터 LOST였던 명확한 이전 owner만
   supersede한다. 동시에 따로 관측된 두 사람은 병합하지 않는다.
   `gate_chi2`는 **association용으로 유지**한다. 제거한 것은 속도 기반 운동 판정용
   `chi2_enter_dynamic/static`, `n_dynamic/static`이다.
4. 각 객체의 IMM은 CV(직선), CT(회전), STATIC(정지) 세 모델을 동시에 갱신한다.
   상태는 모두 `[px,py,vx,vy,omega]`. STATIC은 속도가 섞여 들어와도 위치를
   속도로 외삽하지 않고 속도/회전율을 감쇠시킨다. 위치에는 작은 random walk를 둔다.
   정지 객체에도 다른 모델을 계산하는 이유는 다시 움직이는지 관측으로 비교하기 위해서다.
5. `/tracks`의 현재 통합 상태는 `x = Σ μ_i x_i`이다. 공분산은
   `P = Σ μ_i [P_i + (x_i-x)(x_i-x)^T]`로 모델 간 차이까지 포함한다.
   `/tracked_object_modes`에는 각 모델 상태/공분산/확률 및 품질 메타데이터를 보존한다.
6. 운동 라벨은 IMM posterior를 입력으로 하는 시간 히스테리시스다.
   `P(STATIC)`으로 STATIC, `P(CV)+P(CT)`로 DYNAMIC을 판단한다.
   TENTATIVE는 UNKNOWN. 불충분/상충하는 증거도 UNKNOWN이다.
   임계값을 만족하는 **실제 측정 갱신**이 일정 시간 이어져야 확정한다.
   누락/긴 간격은 증거 시간을 끊는다. 예측만으로 확정 시간을 채우지 않는다.
7. Predictor는 세 모델의 t=0 현재값과 미래 Gaussian을 `/predictions`로 낸다.
   기본은 현재 모드 확률을 고정하고 모델별로 독립 외삽한다.
   dt=0.2s, 요청 horizon=2.5s를 올림해 2.6s까지 제공한다. DRA 기본 horizon은 2.0s.
   선택적인 `propagate_mode_probabilities`는 전이행렬/IMM mixing까지 전파한다.
8. DRA는 `prediction_filter.class_names: [person]`만 사용한다.
   CONFIRMED가 기본이며, 최근 LOST person은 measurement age ≤ 0.3s 동안 예외적으로
   유지한다. 전체 measurement age ≤ 0.5s, semantic age ≤ 1.0s 조건도 함께 적용한다.
   LOST는 이전 운동 상태와 관계없이 마지막 예측 위치의 고정 안전 원으로 바꾼다.
   수신 후 경과 시간도 age에 더한다. 유효 person의 비정상 예측 수치는
   빈 장면으로 바꾸지 않고 prediction chain fail-safe 대상으로 처리한다.

| DRA 입력 운동 상태 | 위험 계산에 쓰는 예측 |
| --- | --- |
| STATIC / UNKNOWN | 세 모드의 t=0을 확률 가중 통합한 현재 위치·공분산을 모든 미래 시각에 고정 |
| DYNAMIC | CV/CT 두 경로, 가중치는 각각 `μCV/(μCV+μCT)`, `μCT/(μCV+μCT)` |
| DYNAMIC인데 이동 모드 질량이 거의 0 | 현재 위치 고정 안전 원으로 fallback |

고정 안전 원은 단순 RViz 그림이 아니다. 고정 중심/공분산의 Gaussian과 객체 반경을
충돌확률 계산에 넣는다. 기본 객체 반경 0.25m, 로봇 반경 0.20m이며 MC 위험 비용과
임계값이 MPPI 후보 경로 평가에 반영된다. 실제 장애물 크기의 정밀 측정값은 아니다.
`/dra_mppi/predictions_used`로 DRA에 선택된 모드를 확인할 수 있다.

지도 벽 근처 STATIC 트랙을 숨기던 `grid_filter_enabled`는 **기본 false**로 바꿨다.
카메라가 검출한 정지한 사람이 벽 근처라는 이유만으로 안전 원에서 사라지는 것을 막는다.
기존 지도 필터 기능은 옵션으로 남지만 재활성화하면 같은 위험이 돌아온다.

## 초기값과 근거

| 설정 | 기본값 | 근거 수준 |
| --- | --- | --- |
| STATIC 진입 / 유지 하한 | 0.85 / 0.65 | 엔지니어링 초기값, 분류 정확도 아님 |
| DYNAMIC 진입 / 유지 하한 | 0.80 / 0.60 | 엔지니어링 초기값 |
| STATIC / DYNAMIC 증거 시간 | 0.50s / 0.30s | 실측 입력 간격을 참고한 초기값 |
| 연속 증거 최대 간격 | 0.25s | 두 실측 bag 간격 p99 약 0.20s보다 여유 |
| STATIC 위치 잡음 | 0.04m/√s | 아래 겉보기 위치 변동의 대략적 규모, 식별된 Q가 아님 |
| STATIC 속도 감쇠 | 0.10s | 혼합된 속도를 빠르게 감쇠시키는 설계값 |
| CV / CT / STATIC 평균 체류 | 3 / 1 / 3s | 기존 CV/CT 유지, STATIC은 임시값 |
| 초기 CV / CT / STATIC 확률 | 0.4 / 0.2 / 0.4 | 초기 UNKNOWN을 의도한 사전확률 |
| STATIC→CV / CV→STATIC / CT→STATIC 분기 | 0.9 / 0.5 / 0.2 | 모드 이탈 시 분기 비율, 프레임당 전이확률 아님 |
| LOST person 재연결 | 0.50s, 0.60m, chi² 16 | 거리·통계·후보 명확성 조건을 모두 적용 |
| 재연결 후보 비율 | 0.70 | 1등 cost가 2등 cost의 70% 미만일 때만 명확하다고 판단 |
| DRA LOST 안전 원 유예 | 0.30s | 짧은 가림만 보호하는 엔지니어링 초기값 |

실제 전이행렬은 위 체류시간·분기로 만든 연속시간 generator의 `expm(G·dt)`이다.
CV/CT 잡음은 기존값을 유지했다. `nis_fit.py` 진단은 세 모드를 누적하지만,
STATIC 잡음/전이/판정 임계값을 공동 최적화하는 완성된 보정 도구는 아니다.

## 최근 bag 세 개 점검

`bags/*/metadata.yaml`의 시작 시각으로 정렬한 정확히 최근 세 개를 2026-09-16에
다시 처리했다. 새로 생긴 bag 때문에 초기값을 만들 때 사용한 세 bag과는 다르다.

| bag | 검출 프레임 | CAMERA / FUSED / LIDAR 객체 수 | 간격 중앙값 / p99 |
| --- | ---: | --- | --- |
| mpc_chain_20260916_014121 | 1,540 | 1,243 / 23 / 0 | 0.0669 / 0.1338s |
| map_v2_20260916_005042 | 0 | 0 / 0 / 0 | - |
| map_hq_20260916_001124 | 0 | 0 / 0 / 0 | - |

최신 세 개 중 실제 semantic 관측이 있는 것은 첫 bag 하나뿐이다. 그 bag의 비-person
동일 클래스 인접 검출에서 odom 이동을 보정한 `거리 / √(2dt)` 중앙값은
0.0270m/√s였다. 표본 하나만으로 기존 기본값을 다시 바꾸지는 않았다.

STATIC 위치 잡음 0.04의 초기 근거는 직전 분석 스냅샷의
`mpc_chain_20260915_154912`, `mpc_chain_20260915_104547`에서 얻은
0.0337, 0.0458m/√s 중앙값이다.
**동일 개체임이 보장되지 않고, 비-person이 정지했다는 보장도 없다.**
검출 중심점 흔들림, 실제 운동, 측정 R, odom 오차가 섞인 수치이므로 Q의 통계적 추정치가 아니다.

새 tracker를 odom 보간으로 재처리한 CONFIRMED person의 상태 표본 수:

| bag | UNKNOWN | STATIC | DYNAMIC | 전체 클래스 생성 트랙 수 |
| --- | ---: | ---: | ---: | ---: |
| 014121 | 116 | 97 | 0 | 291 |
| map_v2 | 0 | 0 | 0 | 0 |
| map_hq | 0 | 0 | 0 | 0 |

이는 고유 사람 수나 정답률이 아니라 매 프레임 발행 대상 트랙의 상태 표본 수다.
014121의 앞부분 4프레임은 odom 보간 범위 밖이라 제외했다. 이 재처리에서 LOST
person의 동일 ID 복구가 18회, 이전 owner supersede가 4회 발생했다. 이는 기능이
실제로 실행됐다는 뜻일 뿐이며, 영상 정답이 없어 22건이 모두 옳았다고 볼 수 없다.
UNKNOWN/LOST가 많고 ID가 자주 만들어지는 현상이 남아 있다. DYNAMIC 표본이 적다는 것이
실제 사람이 정지했다는 뜻은 아니다. 임계값만 낮추기 전에 검출 지속성/association을 점검해야 한다.
재처리는 저장된 `/detections_fused`와 `/odometry/filtered`만 사용했다.
검출기·fusion·실제 TF·MPPI를 모두 재실행한 실차 성능 검증은 아니다.

실행 및 원자료(워크스페이스 루트 기준):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/amr_basic_stack/perception/object_tracker/tools/summarize_imm_bags.py \
  --output artifacts/imm_three_mode/summary.json
```

## 남은 위험과 다음 검증

- **카메라–LiDAR person association은 이번 변경으로 해결되지 않는다.** 강체 중심의
  위치 오차/게이트 모델은 그대로다. 두 실측 bag에서 FUSED 비율은 semantic 검출의
  약 1.7%, 3.7%에 불과했다. 이것만으로 미매칭 원인을 단정할 수는 없다.
  bbox/각도 및 사람 몸통–다리 오차 모델을 별도로 검증해야 한다.
- UNKNOWN 고정 원은 사용자가 선택한 정책이다. 실제 이동 중이지만 아직 UNKNOWN이면
  앞으로 지나갈 영역을 보호하지 못할 수 있다. 고정 공분산도 미래 불확실성을 확대하지 않는다.
- 최근 LOST person은 DRA에서 0.3s 고정 안전 원으로 유지되고 tracker에서는 0.5s 이내
  보수적 재연결을 시도한다. 0.3s보다 긴 가림에서는 DRA에서 사라지며, 빠르게 움직이던
  사람도 유예 중에는 고정 원이므로 미래 이동 영역을 충분히 덮지 못할 수 있다.
- 재연결/ownership은 class와 odom 위치만 사용한다. bbox 이력이나 appearance ID가 없어
  가까이 교차하는 사람은 후보가 모호하면 새 ID를 만든다. 동시에 별도 관측된 두 active
  person은 의도적으로 자동 병합하지 않으므로 일부 중복은 남을 수 있다.
- CV/CT도 속도가 작으면 정지를 설명할 수 있다. IMM 모드 확률은 모델 간 상대 적합도이지
  검증된 정지/이동 확률이 아니다. R/Q/전환율 변경은 판정 결과를 함께 바꾼다.
- 카메라 누락·깊이 중심 변화·클래스 흔들림·odom 오차로 ID가 끊기거나 중복될 수 있다.
  LiDAR-only로 semantic 트랙을 유지하지 않으므로 카메라 사각/가림에 특히 민감하다.
- 모든 CAMERA/FUSED 클래스를 추적해도 DRA allowlist는 person뿐이다. 사람이 든 의자는
  추적되더라도 현재 DRA에는 들어가지 않으며, 사람 반경 0.25m가 운반 물체까지 덮지는 않는다.
  raw scan/정적 장애물 경로가 남아 있다는 것만으로 충돌 회피를 보장하지 않는다.
- prediction 스트림 누락/지연은 기본 0.5s 이후 fail-safe지만, 정상 형식의 빈 메시지는
  '검출된 person 없음'으로 처리한다. 센서 실패를 빈 장면으로 내보내면 구별할 수 없다.
- 다음 bag에는 압축 컬러 영상과 camera_info, 원본 검출/융합, TF/odom, tracks/modes,
  predictions/predictions_used를 함께 기록해야 오매칭과 실제 정지/이동을 대조할 수 있다.
  이번 작업에서는 녹화/카메라 실행 설정을 바꾸지 않았다.

## 빌드·호환성·검증

ImmModeState/MotionModePrediction의 CV=0, CT=1은 유지하고 STATIC=2를 추가했다.
ImmTrackedObject에 STATIC 체류시간·전이 분기 필드가 추가돼 **메시지 wire schema가 바뀌었다**.
기존 바이너리 노드를 섞어 실행하지 말고 관련 패키지를 함께 빌드·재시작한다.
과거 `/tracked_object_modes`를 그대로 새 predictor에 재생하지 말고
`/detections_fused`부터 다시 처리한다. 원본 bag은 수정하지 않았다.

```bash
colcon build --packages-select amr_interfaces object_tracker motion_predictor dra_mppi \
  --symlink-install
source install/setup.bash
```

검증은 입력 필터, STATIC 모델, 3×3 전이행렬, 시간 판정, ROS 직렬화,
DRA 고정 원/이동 모드 선택, controller adapter 및 충돌 위험 계산을 포함한다.
실차를 구동하거나 주행 안전성을 인증한 것은 아니다.
