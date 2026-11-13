# 세미나 — camera-LiDAR late fusion

작성 2026-09-14. 대상 코드 `sensor_fusion/camera_lidar_fusion_node.py`.
근거 실험 `stack_master/scripts/EXPERIMENT_C2.md` / `EXPERIMENT_C2_RESULTS.txt` (2026-09-04 실측).

> 이 노드는 커밋 `edddcbe` 에서 `detection_fusion_node.py` 라는 이름으로 들어갔고,
> 현재 `camera_lidar_fusion_node.py` 로 rename 된 상태가 **아직 미커밋**이다.
> 발표 전에 커밋해서 슬라이드의 파일명과 리포가 어긋나지 않게 할 것.

이 문서의 모든 수치는 노드의 **현재 기본 파라미터를 직접 import 해서** 재계산한 값이다.
재현: `python3 docs/verify_fusion_tables.py` (아래 부록 A).

---

## 0. 한 줄 요약

> 두 센서의 관측을 **object level 에서** 짝짓고(association) 합친다(fusion).
> 짝짓기 문턱과 합치는 가중치가 둘 다 하나의 실측 모델 `R(d)` 에서 나온다.
> "거리는 LiDAR, 방위는 camera" 는 if 문이 아니라 그 식의 결과다.

발표에서 지켜야 할 선 하나 — **검증된 것과 가정인 것을 섞지 않는다.**
R(d)·δ(d)·gate 는 정지 조건(v_rel = 0)에서 실측됐다. 운동 중의 유효성은
측정된 바 없고, 그게 11절이다.

---

## 1. 시스템 — 어디에 붙어 있는가

```text
 D455 color+depth ──▶ rgbd_detection_node ──▶ /detections        (source=0 CAMERA)
                        YOLO11n / OpenVINO        15 Hz, base_link
                        roi_median_3d                            │
                                                                 ├─▶ camera_lidar_fusion_node
 RPLIDAR S2 ×2 ──▶ scan_merger ──▶ lidar_cluster_node ──▶ /lidar_clusters  (source=1 LIDAR)
                     /scan            ABD 클러스터링      10~12 Hz, base_link
                                                                 │
                                                                 ▼
                                                        /detections_fused  (source=2 FUSED)
                                                                 │
                                                  ┌──────────────┴──────────────┐
                                                  ▼                             ▼
                                          detection_markers              object_tracker
                                             (RViz)                    /tracks (odom)
```

토픽 계약 — `amr_interfaces/DetectedObject`:

| 필드 | 의미 | 이 노드가 채우는 값 |
|---|---|---|
| `position` | base_link xy 평면 대표점 [m] | 융합 결과 `z` 또는 passthrough |
| `position_covariance` | 2×2 measurement covariance, row-major [m²] | FUSED 면 `P`, 단독이면 `R(d)` |
| `source` | 0=CAMERA / 1=LIDAR / 2=FUSED | 아래 3절 분기 결과 |
| `class_id`, `bbox` | 카메라에서만 나온다 | LiDAR 단독이면 `-1` / 폭[m] |

**출력 토픽을 따로 두는 이유**: `/detections` 는 순수 카메라 관측으로 유지해야 한다.
`measure_c.py` 가 두 센서를 독립 관측으로 보고 σ 를 재는데, 융합 출력을 재면
자기참조가 되어 다시 측정할 수 없다.

---

## 2. 왜 late fusion 인가

sensor fusion 을 합치는 층위로 분류하면:

| 층위 | 합치는 대상 | 대표 방식 | 이 로봇에서 |
|---|---|---|---|
| **early** (low-level, point-level) | raw 측정값 | LiDAR 점을 이미지로 투영해 bbox 안 점만 취함 (frustum) | **불가** |
| **mid** (feature-level, deep fusion) | 중간 feature tensor | BEV feature concat (BEVFusion 류) | 범위 밖 |
| **late** (object-level, high-level) | 각 센서가 독립 생성한 object | detection-to-detection association + 공분산 융합 | **채택** |

early fusion 이 성립하지 않는 이유 — **RPLIDAR S2 는 2D 다.**
스캔 평면 한 줄(base_link 기준 z = 0.178 m)만 이미지에 떨어진다. bbox 하나에
대응되는 투영점이 수십 개가 아니라 **한 줄**이므로, frustum 안에서 점군을 모아
3D box 를 회귀한다는 전제 자체가 없다. 거기에 extrinsic calibration(카메라↔LiDAR
6-DOF)을 따로 잡아야 한다.

late fusion 이 얻는 것:

1. **URDF + TF 가 이미 extrinsic 이다.** 두 관측이 처음부터 `base_link` 에 있다.
   (남은 잔차는 6절의 yaw misalignment 한 항.)
2. **센서 하나가 죽어도 파이프라인이 산다.** 6 m 이상에서 카메라 검출률이 0 이
   되는데(8절), 그 구간은 LiDAR 단독 관측이 그대로 흘러간다.
3. **각 센서의 검출기를 독립으로 교체·튜닝할 수 있다.** YOLO 를 바꿔도 ABD 는 그대로.

late fusion 이 잃는 것 (발표에서 먼저 말할 것):

- 한 센서만으로는 검출이 안 되는 약한 신호를 살릴 수 없다. early/mid fusion 은
  살린다. 여기서는 각 센서가 **혼자서** 검출에 성공해야 융합 후보가 된다.
- association 이 틀리면 **서로 다른 물체의 위치를 평균낸다.** early fusion 에는
  이 실패 모드가 없다. → 그래서 5~7절 전체가 association 이야기다.

---

## 3. 노드의 한 프레임 — 6단계

`_on_pair(cam_msg, lid_msg)` 한 번에 일어나는 일:

```text
① ApproximateTime 동기화     sync_slop = 67 ms, queue = 10
                               ↓  (frame_id 불일치면 즉시 return)
② LiDAR yaw 보정             p ← laser_xy + R(-43.67 mrad)(p - laser_xy)
                               ↓
③ 대표점 규약 이동 δ(d)       LiDAR centroid → camera 앞면 규약        ← 5절
                               ↓
④ chi² 게이팅                 chi2[i,j] = dᵀ S⁻¹ d,  S = R_C + R_L + Q ← 6절
                               ↓
⑤ Hungarian 배정              assign(chi2, gate=9.21)                  ← 7절
                               ↓
⑥ 공분산 가중 융합            P = (R_C⁻¹ + R_L⁻¹)⁻¹                    ← 8절
                               ↓
   matched → FUSED / 나머지 → passthrough(CAMERA or LIDAR)
```

**④⑤ 와 ⑥ 은 다른 일이고 순서가 있다.** 물체가 둘 이상이면 어느 카메라 검출과
어느 LiDAR 클러스터가 같은 물체인지부터 정해야 한다. 두 단계 모두 `R(d)` 를
쓰지만 역할이 다르다:

| | association (④⑤) | fusion (⑥) |
|---|---|---|
| R 의 역할 | "이 차이가 잡음으로 설명되는가"의 **문턱** | "어느 쪽을 얼마나 믿을까"의 **저울** |
| Q(δ 잔차) 포함 | **포함** | 미포함 |
| 출력 | 짝 또는 기각 | 위치 `z` 와 공분산 `P` |

---

## 4. 측정 모델 R(d) — 극좌표에서 만들고 회전시킨다

```text
R(d, θ) = J(θ) · diag(σ_r(d)², σ_t(d)²) · J(θ)ᵀ ,   J(θ) = rot(atan2(y, x))
```

**xy 대각행렬로 두면 안 된다.** C2 실측에서 반경/횡 오차 비가 2 m 에서 2.0 배,
4 m 에서 3.5 배, 6 m 에서 5.5 배다. 오차 타원이 시선 방향으로 길쭉하고, 그
길쭉함이 거리에 따라 심해진다. 이 anisotropy 가 사라지면 8절의 방향별 가중치
분리가 나오지 않는다.

C2 회귀 결과 (단위 mm, d 는 m):

| 성분 | 모델 | 유효 범위 | 근거 |
|---|---|---|---|
| σ_C,r | `0.321 · d^2.741` | 2 ~ 5 m, R²(log) = 0.9981 | 5 조건, 최대잔차 6.2% |
| σ_C,t | `0.560 · d^1.450` | R²(log) = 0.945 | |
| σ_L,r | `0.331 · d^0.354` | 1.1 ~ 4.09 m, 최대잔차 15.6% | |
| σ_L,r (d ≥ 4.5) | `1.6` 상수 | 5~7 m 실측 1.20~1.88 mm | 멱함수 이탈 |
| σ_L,t | `6.5` 상수 | — | **가정**. 11절 참조 |

**σ_C,r 의 지수 2.741 은 어디서 왔는가** — 발표에서 한 장 쓸 가치가 있다.

- stereo depth 오차의 교과서 값은 `d²` 다. `z = fB/D` 를 disparity `D` 로
  미분하면 `dz/dD = -z²/(fB)`. (Hartley & Zisserman 계열, 또는 Szeliski
  *Computer Vision* 12장 stereo 정확도.)
- 관측값은 `d^2.741` 로 0.741 만큼 더 붙는다. 이것은 소자 잡음이 아니라
  **`roi_median_3d` 라는 추정량의 성질**이다: bbox 픽셀 폭이 `1/d` 로 줄면
  ROI 안 유효 픽셀 수 N 은 `1/d²` 로 줄고, median 의 표준오차는 `1/√N` 이므로
  `d` 에 비례해 커진다. stereo 의 `d²` 에 이 항이 곱해져 `d³` 에 가까워진다.
  관측된 2.741 은 그 사이다.
- **따라서 파이프라인(대표점 정의, ROI 비율)을 바꾸면 σ 는 다른 확률변수가 되고
  이 계수들을 다시 재야 한다.** C1(2026-08-20)이 `1.33 · d^1.81` 로 지수가 다른
  것은 모순이 아니라 대표점이 bbox 중심 11×11 median **1점**이었기 때문이다
  (면적이 거리와 무관 → `1/√N` 항이 없음).

σ_L 은 S2 거리 소자의 intrinsic noise 가 아니다. `LaserScan → ABD clustering →
cluster representative position → TF` 전체의 변동, 즉 **object-level measurement
uncertainty** 다. 같은 파일 안 ABD 의 `sigma_r = 0.03` (데이터시트 accuracy
±30 mm) 과 57 배 차이나는데, 이건 오류가 아니라 서로 다른 확률변수다.

---

## 5. δ(d) — 대표점 정의 차이는 잡음이 아니다

카메라는 물체 **앞면**을, LiDAR 는 **앞면 호의 centroid** 를 잰다. 여기에 ABD 의
`D_max` 가 거리에 비례해 커지며 배경을 흡수하는 성분이 더해진다.

```text
δ_r(d) = -10.98 mm · d^1.281      (음수 = 카메라가 더 가깝다)
δ_t(d) = 13.0 mm / d  [rad]       (횡방향 상수 13 mm)
```

LiDAR 관측을 카메라 규약으로 옮긴다. **카메라 규약으로 통일하는 이유: 충돌
회피에 필요한 것은 가장 가까운 표면이지 centroid 가 아니다.**

**빼지 않으면 게이트가 진짜 쌍을 스스로 기각한다.** C2 실측 bias 를 실측 σ 로
정규화한 값 (Q 미포함, radial only):

| d [m] | 실측 bias_CL_r [mm] | σ_comb,r [mm] | χ² | 판정 (문턱 9.21) |
|---|---|---|---|---|
| 1.10 | −13.5 | 0.539 | **626** | 기각 |
| 2.10 | −22.9 | 2.491 | **85** | 기각 |
| 3.09 | −39.5 | 7.088 | **31** | 기각 |
| 4.09 | −66.2 | 15.259 | **19** | 기각 |
| 5.05 | −96.5 | 27.226 | **13** | 기각 |

χ² 가 거리에 따라 **줄어드는** 이유가 요점이다 — bias 는 `d^1.281` 로 커지는데
분모 σ_C,r 은 `d^2.741` 로 훨씬 빨리 커진다. 즉 계통 오차는 **근거리에서 가장
치명적**이다. "멀수록 어렵다" 는 직관이 여기서는 뒤집힌다.

> 발표 팁: 이 표 하나가 "왜 bias 를 σ 와 따로 다뤄야 하는가" 의 답이다.
> bias 를 σ 에 흡수시키면(게이트를 넓히면) 근거리 오배정이 같이 늘어난다.

---

## 6. gating — chi-square 문턱

```text
d   = p_cam - p_lid                      (δ 보정 후)
S   = R_C(d) + R_L(d) + Q                 Q = J diag(q_r², q_t²) Jᵀ
χ²  = dᵀ S⁻¹ d   ≤  χ²_{2, 0.99} = 9.21
```

`d` 는 2차원이고 두 관측이 독립이므로 `S` 는 합이다. 정규화된 `χ²` 는
자유도 2 의 chi-square 분포를 따르고, 0.99 분위수가 9.21 이다.
(Bar-Shalom, *Estimation with Applications to Tracking and Navigation* 2.3절 —
validation gate. 교과서는 track-to-measurement 를 다루지만 구조는 동일하다.)

**Q 가 왜 필요한가** — `q_r = 5.08 mm`, `q_t = 6.95 mm` 는 δ 모델의 회귀 잔차다.
이게 없으면 1 m 에서 σ_comb 가 **0.46 mm** 라, δ 모델이 몇 mm 만 틀려도 χ² 가
폭발한다. Q 는 게이트에만 들어가고 융합 가중치에는 넣지 않는다 — 모델 불확실성
이지 측정 잡음이 아니기 때문이다.

**유클리드 문턱("몇 cm 이내")을 쓰지 않는 이유**는 게이트 반경이 방향과 거리에
따라 10 배 이상 달라지기 때문이다:

| d [m] | 시선방향 [mm] | 횡방향 [mm] |
|---|---|---|
| 1 | 15.5 | 28.9 |
| 2 | 16.8 | 29.2 |
| 3 | 25.1 | 30.1 |
| 4 | 46.2 | 31.5 |
| 5 | 81.9 | 33.8 |
| 6 | 133.3 | 36.8 |

1 m 에서 시선방향은 15.5 mm 인데 6 m 에서 133.3 mm 다. 고정 반경을 쓰면 근거리
에서 헐겁고 원거리에서 빡빡하다. 반대로 횡방향은 28.9 → 36.8 mm 로 거의 안
변한다 — **게이트가 원이 아니라 거리에 따라 모양이 바뀌는 타원**이다.

검증: C2 의 진짜 쌍 8,370 개에 이 게이트를 적용한 **통과율 99.50%**. 설계값 99%
와 일치한다. 3 m 횡방향 예측 30.1 mm 에 대해 실제 노드에 30 mm 어긋난 쌍을
넣으면 통과, 40 mm 를 넣으면 기각한다.

한 가지 더 — p95 를 그대로 게이트로 쓰지 않았다. 관측된
`d_CL_p95(d) = 48.3 mm/m · d` 인데, 이 값의 대부분은 yaw misalignment 와 δ_r
이라는 **계통 성분**이다. 계통 성분을 보정하고 남은 잡음에 χ² 게이트를 씌우는
것이 옳고, p95 를 쓰면 3 m 에서 149 mm 로 필요보다 5 배 넓다.

---

## 7. assignment — Hungarian 이 필요한 이유

게이트를 통과한 후보가 여럿이면 배정 문제가 된다. `assign(chi2, gate)` 는
**게이트 유효 짝의 개수를 먼저 최대화하고, 그 다음 총 χ² 비용을 최소화**한다.

greedy(최솟값부터 집기)가 틀리는 최소 반례 — `association.py` 의 self-check:

```text
chi2 =  [[2.0, 0.5],
         [8.0, 1.0]]

greedy   : 최솟값 0.5 인 (0,1) 을 먼저 집고 (1,0)=8.0 이 남는다  → 총 8.5
Hungarian: (0,0)+(1,1) = 2.0 + 1.0                              → 총 3.0
```

구현 세부 — 각 행에 **명시적 unmatched 옵션**을 둔다. `cost` 를 `n × (m+n)` 로
만들고 오른쪽 `n` 열을 "배정 안 함" 비용으로 채운다. 게이트 밖 짝은
`unmatched_cost × (n+2)` 라는 더 큰 값을 받아, **금지된 짝이 유효한 짝을 밀어낼
수 없다.** (게이트 밖을 그냥 `inf` 로 두면 `linear_sum_assignment` 가 해를 못
찾는 경우가 생긴다.)

`scipy.optimize.linear_sum_assignment` = Jonker-Volgenant. 복잡도 O(n³).
프레임당 물체 수가 10 단위라 문제되지 않는다.

---

## 8. fusion — covariance-weighted mean

```text
P = (R_C⁻¹ + R_L⁻¹)⁻¹
z = P (R_C⁻¹ p_C + R_L⁻¹ p_L)
```

두 독립 Gaussian 관측의 ML 추정이고, information form 으로 보면 **정보행렬의
합**이다. Kalman update 에서 prior 대신 한쪽 관측을 넣은 것과 같은 식이다.

R 이 anisotropic 이라 **가중치가 방향별로 갈린다** (노드 기본값에서 재계산):

| d [m] | σ_C,r [mm] | σ_L,r [mm] | **카메라 반경 가중 [%]** | σ_C,t [mm] | σ_L,t [mm] | **카메라 방위 가중 [%]** |
|---|---|---|---|---|---|---|
| 1 | 0.32 | 0.33 | 51.53 | 0.56 | 6.50 | 99.3 |
| 2 | 2.15 | 0.42 | 3.74 | 1.53 | 6.50 | 94.8 |
| 3 | 6.52 | 0.49 | 0.56 | 2.75 | 6.50 | 84.8 |
| 4 | 14.35 | 0.54 | **0.14** | 4.18 | 6.50 | **70.7** |
| 5 | 26.45 | 1.60 | 0.36 | 5.78 | 6.50 | 55.9 |
| 6 | 43.59 | 1.60 | 0.13 | 7.53 | 6.50 | 42.7 |

반경 가중이 4 m 에서 0.14% 로 sub-1% 이므로 소수 둘째 자리까지 쓴다. 5 m 에서
0.36% 로 **다시 올라가는 것**은 σ_L,r 이 4.5 m 에서 1.6 mm 상수 바닥으로 갈아
타기 때문이다 (계단 불연속). 물리가 아니라 모델의 이음매다.

**4 m 에서 카메라 가중치가 반경 0.14% / 방위 70.7%.** "거리는 LiDAR, 방위는
카메라" 라는 경험칙이 규칙이 아니라 식에서 나온다. 이게 이 발표의 핵심 그림이다.

σ_C,r 과 σ_L,r 의 교차 거리:

```text
0.321 · d^2.741 = 0.331 · d^0.354
  →  d* = (0.331/0.321)^(1/2.387) = 1.0129 m,   σ = 0.333 mm
```

1 m 를 넘는 모든 거리에서 거리 추정은 LiDAR 가 우위다. 그러나 이것을
"1 m 이상이면 LiDAR" 라는 if 문으로 구현하지 **않았다** — 위 표가 연속적으로
그 결정을 내린다.

융합 결과의 나머지 필드:
- `class_id`, `bbox`, `confidence` 는 카메라에서 가져온다 (LiDAR 에 없음).
- `position.z` 는 **카메라가 본 높이**를 유지한다. LiDAR 의 z 는 스캔 평면 높이
  0.178 m 라는 상수일 뿐 물체의 높이가 아니다.

---

## 9. 짝이 안 맞은 것들 — passthrough 가 기능이다

| 경우 | source | 물리적 의미 |
|---|---|---|
| 카메라만 | 0 (CAMERA) | 스캔 평면(0.178 m) 위/아래의 물체, 또는 LiDAR 가림 |
| LiDAR 만 | 1 (LIDAR) | 6.5 m 초과(카메라 검출률 0), 또는 YOLO 미학습 클래스 |

LiDAR 단독이면 `class_name` 이 `lidar_cluster`, `class_id = -1` 로 비어 있다.
**직전 카메라 확인 클래스를 붙들고 있는 것은 track 수준의 상태이지 frame 수준의
일이 아니다** → `object_tracker` 의 몫. 이 노드는 트랙 유지·ID·속도 추정·클래스
latching 을 하지 않는다.

검출률 절벽 (C2 8절):

| d [m] | 1.1 | 2.1 | 3.09 | 4.09 | 5.05 | 5.10 | 6.05 | 7.10 |
|---|---|---|---|---|---|---|---|---|
| cam_hit [%] | 100 | 100 | 100 | 100 | 97.6 | 92.3 | 92.8 | **0** |
| lid_hit [%] | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |

점진적 열화가 아니라 절벽이다. 원인은 depth 품질이 아니라 **detector
confidence** — 6 m 의 conf 평균이 0.195 로 실험 내내 고정한 `yolo_conf = 0.15`
에 거의 붙어 있고, 7 m 에서는 분포 전체가 문턱 아래로 내려간다.

---

## 10. 계획에 없던 발견 — ABD 의 D_max 가 σ_L 을 오염시킨다

C2 에서 σ_L 이 5 m 에서 peak(1.86 mm)를 찍고 b_L 이 포화하는 비단조성이 나왔다.
설명하려다 나온 것:

```text
D_max(r) = r · sin(Δφ)/sin(λ − Δφ) + k·σ_r
         = r · sin(0.5°)/sin(19.5°) + 3 × 0.03
         = 0.0261 · r + 0.09   [m]
```

| r [m] | 인접 점 간격 [mm] | D_max [mm] | 배수 |
|---|---|---|---|
| 1.1 | 9.6 | 118.8 | 12.4 |
| 2.1 | 18.3 | 144.9 | 7.9 |
| 4.1 | 35.8 | 197.2 | 5.5 |
| 7.1 | 62.0 | 275.6 | 4.4 |

클러스터 폭 실측 (참폭 300 mm): 1.1 m 에서 +2.9%, 5.1 m 에서 +36.7%,
7.2 m 에서 +46.4%. **강체인데 폭이 커진다 = 배경 점을 흡수하고 있다.**
붙었다 떨어졌다 하면 centroid 가 이동하고, 그게 σ_L 의 5 m peak 와
σ_L,t 의 10~13 mm 로 나타난다.

실제로 `σ_L,t = 0.974 × (w_sd/2)`, r = 0.923, n = 42 로 클러스터 폭의 시간
표준편차와 강하게 상관한다. 즉 **LiDAR 방위 오차는 각분해능이 아니라 환경
(배경까지의 간격 대 D_max)의 함수다.**

`D_max` 가 r 에 비례하는 것 자체는 설계 의도다(같은 물체 위 점 간격도 r 에
비례). 문제는 계수 — λ = 20° 가 7.1 m 에서 4.4 배 마진을 준다. 재튜닝 근거가
이 데이터다.

---

## 11. 한계 — 여기부터는 가정이다

발표에서 이 절을 **넘기지 말 것.** 위 숫자들이 언제 무너지는지가 여기 있다.

### (1) 두 관측의 시각차 Δt 가 모델에 없다 — 가장 큰 구멍

`ApproximateTime` 이 짝지어 주는 두 관측은 같은 시각이 아니다(C2 실측 평균
22.2 ms, 최대 66.6 ms). 그런데 residual 은 둘을 같은 시각으로 취급한다:

```text
d = p_cam(t_C) − p_lid(t_L) = (측정 잡음) + v_rel · Δt
                                              └─ S = R_C + R_L + Q 어디에도 없다
```

C2 가 정지 조건이라 측정 자체에서 이 항이 0 이었기 때문이다. 결과로 **움직일 때
진짜 쌍이 기각된다.** base_link 기준이므로 **물체가 정지해 있어도 로봇이 주행하면
걸린다.** 정지 장애물, Δt = 22.2 ms, 순수 radial:

| v_robot | d=1 m | d=2 m | d=3 m | d=4 m |
|---|---|---|---|---|
| 0.5 m/s | 4.7 | 4.0 | 1.8 | 0.5 |
| 0.8 m/s | **12.1** | **10.3** | 4.6 | 1.4 |
| 1.0 m/s | **18.9** | **16.1** | 7.2 | 2.1 |
| 1.5 m/s | **42.6** | **36.2** | **16.2** | 4.8 |

(굵게 = χ² > 9.21 기각). 허용 radial 속도로 바꿔 보면:

| d [m] | gate_r [mm] | Δt=22.2 ms | Δt=66.6 ms |
|---|---|---|---|
| 1 | 15.5 | 0.70 m/s | 0.23 m/s |
| 2 | 16.8 | 0.76 m/s | 0.25 m/s |
| 3 | 25.1 | 1.13 m/s | 0.38 m/s |
| 4 | 46.2 | 2.08 m/s | 0.69 m/s |

**근거리일수록 심하다** — σ_C,r 이 `d^2.741` 이라 1~2 m 에서 combined σ 가 5 mm
대이기 때문이다.

**기각은 에러로 드러나지 않는다.** `_passthrough` 로 빠져 같은 물체가
CAMERA + LIDAR 두 개로 나가고, `object_tracker` 에서 트랙이 갈라진다.

여기에 **intra-scan 방위각 시간**이 더해진다. `LaserScan` 의 stamp 는 scan
**시작** 시각인데(`rplidar_node.cpp:241`), `scan_merger` 가
`time_increment`/`scan_time` 을 0 으로 지워 복원도 안 된다(`scan_merger.py:240`).
드라이버가 `angle_min ≈ -π` 로 두므로 정면(방위 0)은 sweep 중간 =
**stamp + 약 50 ms**(10 Hz). 항상 같은 부호의 systematic lag 이고, 1 m/s 에서
50 mm — 2 m 게이트 반경(16.8 mm)의 3 배다.

**고치는 순서** (순서가 중요하다):

1. **ego-motion 보상.** 로봇 자기 속도는 EKF/odom 으로 알고 있으므로 근사가
   아니라 정확히 뺄 수 있다. tf2 에 이미 있다 —
   `lookup_transform_full(target='base_link', target_time=t_C, source='base_link',
   source_time=t_L, fixed_frame='odom')`.
2. **t_L 을 방위각까지 반영해 계산.** `scan_merger` 가 sweep 타이밍을 보존하고
   `lidar_cluster_node` 가 클러스터 구성 점들의 평균 측정 시각을 실어야 한다.
   이것 없이 1) 만 하면 위 50 ms 상수 lag 이 남는다.
3. **남은 물체 자체 운동만 S 로 흡수**: `S += (σ_v · Δt)² I`.
   1)2) 없이 3) 만 하면 σ_v 를 크게 잡아야 하고, 그러면 4 m 이하에서 게이트가
   무력화돼 오배정이 는다.
4. **`sync_slop` 은 그 다음.** 보상 전에는 줄이는 게 맞고 보상 후에는 키우는 게
   맞다 — 방향이 정반대라 보상 구현 전에 slop 만 건드리면 안 된다.

**검증법(타깃 이동 불필요)**: 정지 장애물 하나 두고 속도를 바꿔가며 직선
접근하며 χ² 와 source 분포를 로깅한다. 위 표가 맞다면 2 m 부근 0.8 m/s 에서
FUSED 가 사라지고 CAMERA + LIDAR 두 개로 갈라지는 것이 보여야 한다.

> 대책으로 `gate_chi2` 를 키우지 말 것. Δt 항은 거리가 아니라 **속도**의
> 함수라, 문턱을 키우면 정지 상태의 오배정만 같이 늘어난다.

### (2) 오배정률(false association rate)이 정당화되지 않았다

C2 는 **단일 객체 실험**이라 거짓 쌍의 분포 자체가 데이터에 없다. 게이트 통과율
99.50% 는 **진짜 쌍**에 대한 수치다. 거짓 쌍을 얼마나 걸러내는지는 다물체
실험이 필요하다. 즉 현재 근거는 "**놓치지 않는다**"만 있고 "**잘못 붙이지 않는다**"
는 없다.

### (3) σ_L,t = 6.5 mm 상수는 임시

10절에서 본 대로 σ_L,t 는 거리가 아니라 환경의 함수다. 물리적으로 맞는 것은
클러스터 폭의 시간 변동으로 적응 추정하는 것인데, 트랙 ID 가 있어야 폭 이력을
쌓을 수 있으므로 `object_tracker` 이후로 미뤘다.
→ **8절 표에서 방위 가중치가 5.5 m 부근에서 역전되는 것은 검증된 결과가 아니라
이 가정의 결과다.**

### (4) yaw 보정값의 귀속이 미확정

0° 조건 33 run 에서 두 센서 방위 차가 `-38.90 ± 2.18 mrad` (= -2.229°,
변동계수 5.6%)로 거리 5.5 배 범위에서 일정했다. 거리에 비례하는 상수 각도이므로
대표점 차이(mm 상수)도 parallax(1/d)도 아닌 **실제 각도 어긋남**이다.
θ 의 변동계수(12%)가 y 의 변동계수(43%)보다 작은 쪽이 LiDAR 라 LiDAR yaw 로
귀속했으나, **절대 방위 기준을 따로 재지 않아 카메라 쪽일 가능성이 남아 있다.**

노드가 쓰는 값이 `-43.67 mrad` 로 실측 `-38.90` 과 다른 것은 회전 중심 때문이다
— 실측은 base_link 원점 기준인데 노드는 `laser_front` 장착점
`(86.3, -20.5) mm` 를 중심으로 회전시킨다. lever arm 으로 대부분 설명되지만
전부는 아니다.

**근본 해결은 URDF(`laser_front` rpy_offset) 수정이다.** 그래야 `/scan` 을 쓰는
SLAM 과 costmap 도 같이 맞는다. URDF 를 고치면 이 파라미터를 **반드시 0 으로
되돌릴 것** — 안 그러면 double correction 이 된다. 노드가 시작할 때 WARN 을 찍는다.

### (5) 보행자에 대한 R 은 아직 없다

C2 의 타깃은 **정지 강체(suitcase)** 다. 사람을 쓰지 않은 것은 호흡·자세 변화가
σ 에 섞이는데 그건 measurement uncertainty 가 아니기 때문이다. 그 판단은 C2 의
목적에서는 옳지만, 트래커가 필요로 하는 R 은 다르다.
→ `object_tracker/docs/EXPERIMENT_SIGMA_A.md` 가 그 후속 실험이다.

### (6) σ_C 모델의 유효 범위는 2 ~ 5 m

아래로는 depth 양자화 바닥(16UC1, q = 1 mm → `q/√12` = 0.289 mm), 위로는 검출
편향(검출 성공 프레임만 σ 에 들어가면 "쉬운 프레임"으로 치우쳐 σ 가 아래로
편향)이 지배한다. 6 m 대는 외삽값 44.6 mm 를 보수적 상한으로 쓰고, 7 m 이상은
카메라 관측이 없다.

---

## 12. 실험 방법론에서 건질 것 — C1 → C2 가 가르친 것

발표에서 시간이 남으면 이쪽이 질문을 많이 받는다.

| 원칙 | C1 에서 무엇을 잃었나 | C2 에서 어떻게 고쳤나 |
|---|---|---|
| **GT 를 반드시 기록한다** | `nominal_d` 만 있어 "카메라가 5 m 에서 -333 mm 로 가깝게 본다"가 센서 편향인지 배치 오차인지 **원리적으로** 구분 불가 | 줄자로 base_link→타깃 앞면 실측, 반올림 금지. 결과: b_C 는 ±16 mm 무편향 — C1 의 -333 mm 는 배치 오차였다 |
| **유효성은 행이 아니라 열 단위** | LiDAR 가 벽을 잡은 run 을 통째로 버려 5 m 전체를 잃음 | 열 단위로 거름. C1 도 되살리니 회귀 지수가 1.38 → 1.81 로 이론값 2 에 붙었다 |
| **파이프라인 설정을 실험 시작 시점에 못 박는다** | 세션 중간에 `yolo_conf` 를 0.4 → 0.15 로 바꿔 전후 데이터를 합칠 수 없게 됨 | launch 인자로 고정 + 매 run `ros2 param get` 을 CSV `note` 에 자동 기록. 62 run 전부 고유값 1개 |
| **σ 로는 안 잡히는 실패가 있다** | 엉뚱한 물체(벽)를 잡아도 **벽은 안 움직이므로 σ 가 오히려 작다** | GT 대비 bias 로 판별. 배치 실수 11 run 을 이 근거로 폐기 |
| **raw 를 남긴다** | 요약만 남기면 n_eff·bootstrap·자기상관을 사후에 못 구함 | run 당 raw CSV 1개 (59 파일) |

마지막 줄 — **파라미터가 실측에서 나왔다는 것을 코드가 스스로 말하게 한다.**
`camera_lidar_fusion_node.py` 의 module docstring 이 C2 문서·커밋 해시·측정
조건(v_rel = 0)·유효 범위를 전부 명시한다. 1년 뒤에 이 계수를 건드릴 사람은
"이게 왜 0.321 인가" 를 파일 안에서 답할 수 있어야 한다.

---

## 13. 발표 구성 제안 (30분 + Q&A)

| # | 슬라이드 | 분 | 핵심 메시지 |
|---|---|---|---|
| 1 | 문제 설정 — 2D LiDAR + RGB-D | 2 | 하드웨어가 fusion 층위를 결정한다 |
| 2 | early / mid / late 비교표 (2절) | 3 | 2D 라이다에서 early 는 성립 안 함 |
| 3 | 노드 그래프 + 토픽 계약 (1절) | 2 | `/detections` 를 왜 따로 유지하나 |
| 4 | 6단계 파이프라인 (3절) | 2 | association ≠ fusion |
| 5 | R(d) 와 anisotropy (4절) | 4 | 지수 2.741 의 해석 ← 질문 많이 나옴 |
| 6 | δ(d) 와 무보정 χ² 표 (5절) | 4 | **계통 오차는 근거리가 치명적** |
| 7 | 게이트 반경 표 (6절) | 3 | 원이 아니라 거리 의존 타원 |
| 8 | Hungarian 반례 (7절) | 2 | greedy 8.5 vs Hungarian 3.0 |
| 9 | **가중치 표 (8절)** | 4 | if 문 없이 "거리는 LiDAR, 방위는 카메라" |
| 10 | Δt 미해결 + 속도 표 (11-1) | 4 | 검증된 것과 가정의 경계 |
| 11 | 실험 방법론 5원칙 (12절) | — | 시간 남으면 |

가장 강한 슬라이드는 **9번**(가중치가 식에서 나온다)과 **6번**(χ² 가 거리에 따라
줄어든다)이다. 10번을 빼지 말 것 — 한계를 먼저 말하는 발표가 방어하기 쉽다.

---

## 14. 예상 질문

**Q. 왜 EKF/UKF 로 두 센서를 직접 융합하지 않나?**
A. 한다 — `object_tracker` 가 그 층이다. 이 노드는 **같은 시각의 두 관측을 하나로
줄이는 frame-level 연산**이고, 트래커는 **시간축을 따라 상태를 추정하는 track-level
연산**이다. 둘을 합치면 트래커가 매 프레임 관측 개수가 달라지는 문제를 떠안고,
association 실패가 트랙 상태를 오염시킨다. 분리하면 이 노드의 출력
(`position_covariance`)이 그대로 트래커의 `R` 이 된다.

**Q. Q(δ 잔차)를 융합 가중치에도 넣어야 하지 않나?**
A. Q 는 δ 모델의 **모델 불확실성**이지 측정 잡음이 아니다. 게이트는 "이 차이가
설명되는가"를 묻기 때문에 모델 불확실성을 포함해야 하지만, 융합 가중치는 "어느
관측이 더 정확한가"를 묻기 때문에 각 센서의 R 만 들어가야 한다. Q 를 양쪽에
똑같이 더하면 가중치가 1:1 쪽으로 끌려가 LiDAR 의 거리 우위가 희석된다.

**Q. χ²_{2,0.99} = 9.21 의 0.99 는 어떻게 정했나?**
A. 진짜 쌍을 1% 놓치는 것을 허용한다는 설계값이고, C2 진짜 쌍 8,370 개 실측
통과율 99.50% 로 확인했다. 다만 이 값의 반대편(거짓 쌍 기각률)은 아직 근거가
없다 — 11-(2).

**Q. 2D LiDAR 인데 스캔 평면 밖 물체는?**
A. 카메라 단독(source=0)으로 나간다. 스캔 평면 높이는 base_link 기준 0.178 m.
낮은 장애물·공중 돌출물은 카메라만 본다.

**Q. 6 m 이상에서 클래스를 모르는 LiDAR 트랙은 어떻게 쓰나?**
A. 이 노드는 클래스를 비워 보내고, 클래스 latching 은 `object_tracker` 가 한다.
이 노드에서 하면 **frame 수준에서 track 수준의 상태를 들고 있게 되어** 트랙이
끊겼을 때 잘못된 클래스가 남는다.

---

## 부록 A — 재현 스크립트

본문의 표(4·6·8·10·11절)는 노드의 현재 기본 파라미터를 import 해서 계산한 값이다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/2026amr_ws/install/setup.bash
python3 src/amr_basic_stack/perception/sensor_fusion/docs/verify_fusion_tables.py
```

파라미터를 바꾸면 표도 같이 바뀐다. 발표 직전에 한 번 돌려서 문서와 코드가
갈라지지 않았는지 확인할 것.

## 부록 B — 참고 문헌 / 표준 교재 대응

| 주제 | 어디를 보면 되나 |
|---|---|
| validation gate, chi-square gating | Bar-Shalom, *Estimation with Applications to Tracking and Navigation*, 2.3 |
| GNN / Hungarian assignment | Blackman & Popoli, *Design and Analysis of Modern Tracking Systems*, 6장 |
| information-form fusion (covariance-weighted mean) | 위 Bar-Shalom 8장 (track-to-track fusion) — **주의: 그 장은 상관된 트랙을 다루고, 여기는 독립 관측이라 단순 합이 맞다** |
| stereo depth 오차 `d²` | Szeliski, *Computer Vision: Algorithms and Applications*, 12장 |
| Adaptive Breakpoint Detector | Borges & Aldon (2004), *Line Extraction in 2D Range Images for Mobile Robotics* |
| early/mid/late fusion 분류 | 자율주행 survey 다수. 다만 **2D LiDAR 를 다루는 문헌은 드물다** — 이 시스템의 late fusion 선택이 survey 의 기본 가정(3D LiDAR) 밖이라는 점을 명시할 것 |

리포 내부 문서:

| 파일 | 내용 |
|---|---|
| `stack_master/scripts/EXPERIMENT_C2.md` | 실험 정의 — 고정할 파라미터, 프로토콜, 수용 기준 |
| `stack_master/scripts/EXPERIMENT_C2_RESULTS.txt` | 결과 해석 전문 (11절 구성) |
| `sensor_fusion/camera_lidar_fusion_node.py` | module docstring 에 late fusion 근거·미해결 Δt 전부 |
| `sensor_fusion/lidar_cluster_node.py` | ABD, σ_r 과 σ_L 의 구분, 클러스터 측정 시각 미구현 |
| `object_tracker/core/association.py` | 같은 구조의 track-to-measurement association |
| `object_tracker/docs/EXPERIMENT_SIGMA_A.md` | 보행자 R 과 process noise 후속 실험 |
