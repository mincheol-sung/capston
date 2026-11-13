# object_tracker — 남은 일

작성 2026-09-09, 개정 **2026-09-14** (IMM 도입 반영). 기준 커밋 `f2cbf22` + 미커밋 작업.

---

## 한 줄 현황

트래커 본체(**CV/CT IMM** / association / 생명주기 / 동정판별 / NIS 진단)와
`motion_predictor` 는 **구현·배선 완료**. `/detections_fused` 부터 DRA-MPPI 까지
신호가 끊긴 데가 없다. 테스트 136개 통과 (tracker 79, predictor 5, DRA-MPPI 52).

남은 것은 **운동모델 파라미터 7개가 전부 근거 없는 임시값**이라는 것 하나다.
이걸 재려면 로봇이 필요하고, 로봇 앞에 서기 전에 도구가 준비돼 있어야 한다.

```text
/detections_fused
  └→ object_tracker (CV/CT IMM)
       ├→ /tracks                  (TrackedObjectArray)  → dynamic_filter(stub)
       └→ /tracked_object_modes    (ImmTrackedObjectArray)
            └→ motion_predictor
                 └→ /predictions   (MultiModalObjectPredictionArray)
                      ├→ DRA-MPPI ─→ /dra_mppi/predictions_used → prediction_markers
                      └→ risk_model (stub, /risk 를 아무도 안 봄)
```

---

## 미측정 파라미터 — 이것이 남은 일의 전부다

| 파라미터 | 현재값 | 근거 | 어디서 나오나 |
|---|---|---|---|
| `sigma_a` | 1.0 | `[TODO]` 임시 | **[B]** `nis_fit.py` CV mode |
| `sigma_a_ct` | 0.7 | 없음 | **[B]** `nis_fit.py` CT mode |
| `sigma_omega_cv` | 0.15 | 없음 | 미해결 — 아래 참고 |
| `sigma_omega_ct` | 0.5 | 없음 | 미해결 — 아래 참고 |
| `sigma_omega0` | 1.0 | 없음 | 미해결 — 아래 참고 |
| `mean_sojourn_cv` | 3.0 s | 없음 | **[B]** `SojournAccumulator` (진단 등급) |
| `mean_sojourn_ct` | 1.0 s | 없음 | **[B]** `SojournAccumulator` (진단 등급) |

`predictor.yaml` 의 `propagate_mode_probabilities: false` 가 이 표에 걸려 있다.
주석대로 "sojourn/process 파라미터가 보정된 뒤" 켜는 값이라 **지금 IMM 은
예측에서 절반만 켜져 있다** — horizon 내내 mode 확률이 고정이다.

---

## 순서

```text
[A] bag 뜨기 ──→ [B] 파라미터 역산 ──→ [C] propagate_mode_probabilities 켜기
   (로봇 필요)      (로봇 없이)
        │
        └──→ [D] 보행자 R 확인 (A 와 같은 bag 으로 가능)

[E] dynamic_filter / risk_model — 위 경로에 필요 없음. 나중에.
```

**왜 이 순서인가.** `motion_predictor` 가 내는 예측 신뢰구간의 폭이
`P(t+T) = F(T) P F(T)^T + Q(T)`, `Q_vv(T) = sigma_a^2 * T` 라 `sigma_a` 가
그대로 결정한다. 미실측인 채로는 MPPI 가 회피하는 반경이 근거 없는 값이다.

---

## [A] bag 뜨기 — 로봇 필요, 1.5~2 시간

**→ 절차·수용기준·함정은 [docs/EXPERIMENT_SIGMA_A.md](EXPERIMENT_SIGMA_A.md) 에 따로 있다.
로봇 앞에 서기 전에 그것부터 읽을 것.**

요약:

| 조건 | 타깃 | 거리 | 길이 | 쓸 곳 |
|---|---|---|---|---|
| D-1 | 사람 1명, 기동(직선·**선회**·정지출발·접근·횡단) | 2~5 m | 90s × 3 run | [B] `sigma_a`, `sigma_a_ct` |
| D-2 | 사람 1명, 자연스럽게 서 있기 | 2/4/6 m | 30s × 2 run | [D] 보행자 R |
| D-3 | **backpack (C2 와 같은 것)** — 대조군 | 2/4/6 m | 30s | [D] 해석의 기준 |
| D-4 | 사람 2명 교차 | — | 30s × 2 | association 눈확인 |

핵심 3가지:

1. **로봇 정지.** `nis_fit.py --bag` 이 base_link 좌표를 그대로 쓰므로 로봇이
   움직이면 자기운동이 process noise 로 흡수돼 과대추정된다.
2. **D-1 의 선회를 빼지 말 것.** IMM 이전에는 "있으면 좋은" 운동이었지만
   지금은 **CT mode 를 excite 하는 유일한 운동**이다. 선회가 없으면
   `sigma_a_ct` 가 식별되지 않는다.
3. **D-3 을 빼지 말 것.** backpack 이 C2 예측과 맞는지를 봐야 D-2 의 차이를
   "몸 흔들림"으로 해석할 수 있다. 안 맞으면 파이프라인이 변한 것이고 그게 먼저다.

---

## [B] 파라미터 역산 — 로봇 없이, 10분

```bash
cd src/amr_basic_stack/perception/object_tracker
python3 tools/nis_fit.py --sim 1.0             # CV 궤적으로 도구 검증
python3 tools/nis_fit.py --sim-imm             # CV/CT 궤적으로 도구 검증
python3 tools/nis_fit.py --bag ~/bags/sigma_a_D1_run1
```

**출력 읽는 법.** 수렴하면 이 셋이 동시에 맞아야 한다.

| 지표 | 목표값 | 뜻 |
|---|---|---|
| 평균 NIS (mode 별) | **1.907** (2.0 아님) | 게이트 절단 때문. 2.0 을 쓰면 4.9% 계통오차 |
| 기각률 | 1.0% | 게이트가 chi^2_2(0.99) 라서 |
| c_CV, c_CT | 1.000 | 참 분산 / 필터가 주장하는 분산 |

### 실측 후 할 일

1. `core/config.py` 의 `sigma_a` / `sigma_a_ct` 와 `config/tracker.yaml`,
   그리고 **`motion_predictor/config/predictor.yaml` 의 같은 값**을 그 값으로.
   (predictor 가 트래커와 같은 모델 파라미터를 따로 들고 있다 — 셋을 같이 고쳐야 한다.)
   주석의 `[TODO]` 를 `[실측]` 으로 바꾸고 날짜·bag 이름 남길 것.
2. `PYTHONPATH=. python3 tools/detection_floor.py` 다시 돌려 **판정 하한**이
   얼마가 됐는지 확인. 보행 1.2 m/s 대비 여유가 2배 미만이면 곤란하다.
3. `PYTHONPATH=. python3 tools/motion_state_demo.py` 로 게이트가 견디는
   감속도를 재확인. 사람의 실제 정지 감속도는 1.5~4.0 m/s^2 다.

### 하한이 너무 거칠면 (0.5 m/s 넘으면)

`sigma_a` 를 association 용과 판별용으로 **분리**한다. `gate_chi2` 와
`chi2_enter_*` 가 이미 별개 파라미터라 `sigma_a_gate` 하나 추가하면 된다.
**실측 전에 하지 말 것** — 지금은 하한이 정말 문제인지 모르는 채로
손잡이만 늘리는 것이다.

---

## [C] propagate_mode_probabilities 켜기 — [B] 이후

`predictor.yaml` 에서 `false` → `true`. 지금은 horizon 내내 posterior mode
확률을 그대로 들고 가지만, `true` 면 매 미래 스텝마다 measurement-free IMM
interaction 과 `transition_matrix(dt, tau_cv, tau_ct)` 를 돌린다. 즉 **체류
시간이 맞아야 의미가 있는 스위치**다. [B] 가 sojourn 을 내기 전에 켜면
근거 없는 TPM 으로 예측 가중치를 흔드는 셈이다.

---

## [D] 보행자 R 확인 — [A]-2 bag 으로

**이게 [B] 보다 먼저여도 된다. 아니 사실 더 중요하다.**

C2 에서 잰 `sigma` 는 **강체 정지 타깃(suitcase) 반복측정값**이다. 보행자는
팔·다리·몸통이 흔들려 대표점(roi_median_3d) 자체가 움직이므로 R 이 한 자릿수
클 수 있다. 그러면 판정 하한도 게이트도 전부 이동한다.

확인법: [A]-2 의 "가만히 서 있는 사람" bag 에서 검출 위치의 표본 분산을 직접
재서 `detection_floor.py` 의 `CAM_SIGMA_R` / `LID_SIGMA_R` 예측값과 비교.

R 이 실제로 크면 `camera_lidar_fusion` 의 `PolarNoiseModel` 파라미터에
**클래스별 보정**을 넣어야 한다 (person 은 x배). 지금은 클래스 구분이 없다.

---

## [E] 나중 — 이 경로에 필요 없음

| 노드 | 현재 | 필요한가 |
|---|---|---|
| `dynamic_filter` | 43줄 stub, passthrough | SLAM 입력 정제용. DRA-MPPI 는 안 씀. **정말 별도 노드가 필요한지부터 재검토** — 트래커가 이미 `motion_state` 를 실어 보내므로 하류가 필드 하나 보면 되는 일에 노드를 띄우는 셈 |
| `risk_model` | 58줄 stub | DRA-MPPI 가 `/risk` 를 구독하지 않는다. `risk_score` 정의(TTC? 거리? 예측 분산?)가 설계 문제로 남아 있음 |

---

## 정리 항목 (언제든)

| 항목 | 내용 |
|---|---|
| **죽은 계약** | `ObjectPrediction.msg` / `ObjectPredictionArray.msg` 를 참조하는 코드가 0줄인데 `amr_interfaces/CMakeLists.txt` 에 여전히 등록돼 있다 |
| **낡은 주석** | `predictor.yaml` 의 "DRA uses 4.0 s" — 실제 2.0 s (`dra_mppi.yaml` 의 `horizon_steps: 10` × `dt: 0.20`). 값 `2.5` 자체는 맞다 (2.0 + `prediction_timeout` 0.5) |
| **얇은 테스트** | `motion_predictor` 가 5개뿐. IMM propagation 은 `F P F^T + Q` 를 유한차분으로 검산할 수 있는 부분이다 |
| **미커밋 103개** | `f2cbf22` 가 158 files 한 덩어리라 이력 추적이 어려웠다. 같은 실수를 반복하지 말 것 — 최소한 `amr_interfaces`+`object_tracker`+`motion_predictor`(IMM 한 덩어리) / `traversability` / `mpc` / `amcl` 로 끊는다 |

---

## 참고

### 명령어

```bash
cd src/amr_basic_stack/perception/object_tracker

# test/ 전체를 주면 ament boilerplate(test_copyright/flake8/pep257)가 수집 에러를 낸다.
python3 -m pytest test/test_kalman.py test/test_association.py \
    test/test_track.py test/test_tracker.py test/test_nis.py \
    test/test_imm.py test/test_motion_models.py -q          # 79개
PYTHONPATH=. python3 object_tracker/core/nis.py     # 모듈 self-check
PYTHONPATH=. python3 tools/detection_floor.py       # 거리별 판정 하한
PYTHONPATH=. python3 tools/motion_state_demo.py     # 동정 판별 시나리오
python3 tools/nis_fit.py --sim 1.0                  # CV 궤적 검증
python3 tools/nis_fit.py --sim-imm                  # CV/CT 궤적 검증
```

`PYTHONPATH=.` 가 필요한 이유: `python3 file.py` 는 cwd 가 아니라 **파일이 있는
디렉터리**를 sys.path 에 넣는다. 패키지 임포트가 있는 모듈은 이것 없이는 안 돈다.

`motion_predictor` / `DRA_MPPI` 테스트는 `amr_interfaces` 를 임포트하므로
`source /opt/ros/*/setup.bash && source install/setup.bash` 가 먼저 필요하다.

### 확정된 숫자

| 항목 | 값 | 출처 |
|---|---|---|
| `sigma_a` 단위 | **m/s^1.5** (m/s^2 아님) | `Q_vv = sigma_a^2 dt` 가 (m/s)^2 여야 함 |
| `sigma_a` 조작적 의미 | 1초 coast 시 속도 추정이 벌어지는 폭 [m/s] | `sigma_v(T)=sqrt(sigma_v0^2+sa^2 T)` |
| NIS 목표 (게이트 있음) | **1.907** | `(2-(g+2)e^(-g/2))/(1-e^(-g/2))`, MC 1.9063 |
| 게이트가 깨지는 가속도 (sa=1.0) | 7.76 m/s^2 | 이분탐색. sweep 으로 재확인 (7.06 에서 끊기기 시작) |
| 사람 정지 감속도 | 1.5~4.0 m/s^2 | 1.2 m/s 를 0.3~0.8s 에 정지 |
| 판정 하한 (sa=1.0, 1~9m) | 471~602 mm/s | `detection_floor.py` |
| tracking index Lambda (4m) | 44.5 | `sa*dt^1.5/sigma_z`. 커서 NIS 가 sa 에 민감 |
| 전환 지연 | → STATIC 167ms, → DYNAMIC 0~250ms | `motion_state_demo.py` |
| IMM 정상분포 (tau 3.0/1.0) | CV 0.75, CT 0.25 | `transition_matrix` 의 좌고유벡터 |
| 필터 CT 의 1스텝 omega sd | 0.144 rad/s | `sigma_omega_ct*sqrt(dt)` = 0.5*sqrt(1/12) |
| **MPPI horizon** | **10 × 0.20s = 2.0s** | `dra_mppi.yaml` (예전 문서의 4.0s 는 낡았다) |
| predictor horizon | 2.5s = 2.0 + `prediction_timeout` 0.5 | `predictor.yaml` |

### 밟았던 함정 (다시 밟지 말 것)

- **IMM 이 fused NIS 기반 `sigma_a` 추정을 깬다.** moment-matched fused S 는
  `sigma_a^2` 에 비례하지 않는다 — `moment_match` 가 CT mode 의 P (세기가
  `sigma_a_ct` 로 고정) 와 mode 간 벌어짐 `outer(delta, delta)` 를 함께 넣기
  때문이다. 실측한 편향 (CV 궤적, 정상분포 CT 25%):

  | 참 `sigma_a` | fused 로 추정 | CT 죽이고 추정 |
  |---|---|---|
  | 0.3 | 0.248 (**-17.2%**) | 0.297 (-0.9%) |
  | 1.0 | 1.048 (**+4.8%**) | 0.988 (-1.2%) |
  | 3.0 | 3.206 (**+6.9%**) | 3.005 (+0.2%) |

  부호가 `sigma_a` 와 `sigma_a_ct`(0.7) 의 대소를 따라 뒤집힌다. 그래서
  `ModeNisAccumulator` 로 mode-conditioned NIS 를 쓴다.
- **기각 수를 mode 누적기에도 먹여야 한다.** `NisAccumulator` 에만 먹이고
  `ModeNisAccumulator` 를 빼먹으면 고정점이 -4.9% 로 밀린다 (0.951 vs 0.984).
  절단 보정이 통째로 빠지기 때문. 배선을 고친 뒤 잔차가 -1.6% 로 돌아왔다.
- `steady_state_P()` 는 a **posteriori** P 를 돌려준다. `velocity_chi2` 용도로는
  맞지만(update 끝에서 호출되므로) innovation covariance `S = H P H^T + R` 에
  쓰면 틀린다 — 거기엔 a **priori** P 가 필요하다. 헷갈리면 `HPH/R`이 1.000 으로
  나와 "NIS 가 sigma_a 에 둔감하다"는 정반대 결론이 나온다. 실제론 1241 배다.
- 필터의 P 는 실제 오차 공분산이 **아니다**. `sigma_a=1.0`, 3m 에서 P 는
  178 mm/s 를 주장하지만 진짜 정지 물체 실측 산포는 96 mm/s (1.85배).
- NIS 표본에서 **게이트 밖으로 밀려난 개수를 세지 않으면** 분포가 오른쪽에서
  잘려 파라미터가 계통적으로 작게 나온다. 절단 평균만 쓰면 분산비 4 이상에서
  포화한다. 그래서 `assign()` 이 기각된 chi^2 을 돌려주고 MLE 를 쓴다.
- `base_link -> odom` 에서 **공분산도 같이 회전**시켜야 한다 (`R_odom = C R C^T`).
  4m 에서 오차 타원 이방성이 5.3:1 이고 장축이 시선에 수직이라(라이다가 거리를,
  카메라가 방위를 잡아서) 안 돌리면 Kalman gain 이 통째로 틀어진다.
- 시연 시나리오에 **계단 입력을 쓰지 말 것.** 1.2 m/s 를 한 프레임(83.3ms)에
  죽이면 14.4 m/s^2 라 게이트가 깨지고 트랙 ID 가 새로 생긴다. 사람은 그렇게
  못 멈춘다. ramp 로 넣어야 한다.
