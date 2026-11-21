# 실차 시험 절차 — 작은 circles8 + 순항 속도 sin 변조로 A·B·C 슬립 비교 (2026-11-20 작성, 아직 안 함)

목적: 지금 circles8 (R 0.5 m, 0.7 m/s 등속) 은 바닥 마찰의 절반 정도만 써서 슬립이 적고, 그 조건에서는 A 가 C 보다 슬립이 작았다
(`amr/발표/랩미팅/2026-11-16_ABC_실차비교.md`). 바닥 한계 근처까지 요구하는 기준에서 다시 비교한다.
관련: 체크리스트 `real_robot_checklist.md` 5-0·5-4·6, μ̂ 절차 `mu_surface_procedure.md`, 결정 기록 `discussion_2026-11-16_slip_experiment.md`.

## 0. 왜 이 조건인가 (계산·시뮬 근거)

| 항목 | 값 | 출처 |
|---|---|---|
| 바닥 μ̂ (롤러 정의 √2·a/g) | 옆 0.16–0.29 (중앙 약 0.26, 3 회차), 앞 ≥ 0.34–0.40 (4 회차 모두 바퀴 전압 한계에 먼저 닿아 **하한**) | 2026-11-17 램프 CSV 재분석 (`analysis/torque_mpc_real/2026-11-17_ramp/`) |
| tangent 등속의 상한 | 기준 부하 v(1 + K_GEOM/R) ≤ load_budget × 1.55 m/s 이면 v²/R 최대 = cap²/(4·K_GEOM) (R = K_GEOM). 예산 0.65 (11-20 전): 1.28 m/s² → μ̂ 0.184. **예산 1.0·K_GEOM 0.1932 (지금, changelog 2026-11-20 (1)·11-19 (7)): 3.11 m/s² → μ̂ 0.448** | 해석식, R 0.1–0.8 m 수치 탐색과 같음 |
| 그래서 | 예산 1.0 에서는 작은 원 등속으로도 한계를 넘길 수 있다 (R 0.3 에서 등속 최고 0.943 m/s). 이 절차는 등속 0.45 를 **슬립이 적은 기준선**으로 두고, 변조(접선 + 구심 가속을 한 대각 바퀴 쌍에 겹침, 필요 μ̂ 99 % 값 0.29–0.33)를 슬립 조건으로 쓴다 | `core/figure8.py` speed_mod_* 로 탐색 (예산 0.65 · K_GEOM 0.198 일 때) |
| 시뮬 (μ 0.26, C 는 균일 μ̂ 0.25, 옛 기하·예산 0.65 코드, 3 회차씩) | 변조: C λ RMS 0.098–0.107 < A 0.188–0.199 (겹치지 않음). 등속: A 0.054–0.057 < C 0.079–0.085. B 는 6 회차 모두 track_error 1.5 m 로 정지 | `torque_experiment.launch.py plant:=sim surface_mu:=0.26` |

- yaw_mode 는 **tangent 만**: Pure Pursuit (A) 는 (v_x, ω_z) 만 내고 v_y = 0 이라 fixed 기준을 못 따른다 (`baseline/pure_pursuit.py`)
- 시뮬 바닥은 방향 구분 없는 μ 하나다. 실차는 앞이 더 세서 A 의 슬립이 시뮬만큼 안 늘 수 있다 — 그걸 보는 것이 이 시험

## 1. 시험 전에 (한 번)

- [ ] **코드 반영 확인**: 2026-11-20 NUC `~/capston/capston` (smc 작업 트리, 커밋 전) 에 넣었다. build/amr_mpc 의 launch·config·파이썬은 src 링크라 다시 빌드할 필요 없음.
      확인: 기동 로그에 "속도 변조 ε …" 줄이 나오는지. 같은 변경이 이 컴퓨터의 `~/capston_jjm` (jjm 브랜치, 커밋 전) 에도 있다
- [ ] **교수님께 공유**: 결정 1 (2026-11-15) 은 "기준은 속도, 손잡이는 순항 속도, accel 0.4 고정". 변조도 속도 기준이지만 순항 중 접선 가속이 최대 1.47 m/s² 생긴다 (accel 과 따로)
- [ ] **바닥 공간**: circles8 `width:=1.2` → 경로 경계상자 앞뒤 0.6 m × 좌우 1.2 m, 로봇 외접반경 0.18 m 더해 0.96 × 1.56 m. B 가 벗어날 것을 감안해 사방 1 m 이상 비우기
- [ ] **C 의 μ̂**: `torque_plant.yaml surface.mu_hat_wheels` 지금 값 [0.432, 0.44, 0.456, 0.417] (FL FR RL RR, NUC 작업 트리 — 커밋 18f0816 의 [0.432, 0.285, 0.456, 0.350] 에서 FR·RR 이 바뀜, 2026-11-20 S2 C 기동 로그로 확인) 그대로 — 바퀴별 모터 특성 차이·무게중심을 반영해 사용자가 정한 값 (2026-11-19 (11)).
      τ̄ = [0.1093 0.1113 0.1154 0.1055] N·m. 이 값이 켜져 있으면 `mu_hat:=` 는 상한을 바꾸지 않는다 → C 명령에 `mu_hat` 를 주지 않는다.
      기동 로그에서 바퀴별 τ̄ 네 값이 위와 같은지 확인. 시뮬 예측(아래 5 절)은 균일 μ̂ 0.25 로 돌린 것이라 C 는 조건이 다르다 — 같은 조건 시뮬은 아직 안 함

## 2. 매 시험 날 준비

- [ ] `real_robot_checklist.md` 5-0 전부: 두 사람 수동 정지 배치, 배터리 셀 ≥ 3.5 V (쉬는 상태), `check_localization.py` 겹침 ≥ 70 %, `/tracked_pose` 10 s 안정
- [ ] `amrall` 은 **한 번에 하나만**. 이전 launch 를 Ctrl+C 로 끄고 `pgrep -af ros2` 로 남은 노드가 없는지 본 뒤 다음 회차 (노드가 겹치면 CAN·라이다를 서로 뺏는다)
- [ ] 바퀴·바닥 청소 (먼지가 μ 를 바꾼다), 같은 자리에서 출발 (바닥 테이프)

## 3. 단계 — 약한 것부터

공통 인자 (아래 `$T`):

```bash
T="use_mpc:=true shape:=circles8 width:=1.2 yaw_mode:=tangent laps:=1 max_track_error:=0.5 bag:=true"
```

- `max_track_error:=0.5`: 기본 1.5 m 는 작은 방에서 위험하다. 시뮬에서 A·C 의 최대 오차는 86 mm 이하라 셋 다 같은 문턱이어도 A·C 결과에는 영향 없음
- 결과는 `bags/<날짜>/<시분초>` (record_all.sh). 회차마다 아래 기록표에 bag 이름을 적는다

| 단계 | 조건 | 속도 [m/s] | 접선 가속 최대 [m/s²] | 기준 부하 / 예산 | 주행 시간 | 회차 | 넘어갈 조건 |
|---|---|---|---|---|---|---|---|
| S1 | 작은 원 확인, A 만 | 등속 0.30 | 0.40 (출발·정지) | 0.493 / 1.550 | 13.3 s | 1 | 완주, 경로가 바닥 표시 안에 |
| S2 | 약한 변조, A 다음 C | 0.24–0.45 (`speed:=0.35 speed_mod_amp:=0.3 speed_mod_cycles:=6`) | 0.40 | 0.748 / 1.550 | 12.2 s | 각 1 | 둘 다 완주, 기동 로그에 변조 줄 |
| **본 1** | 등속 | 0.45 | 0.40 | 0.740 / 1.550 | 9.5 s | A·C·B 각 3 | — |
| **본 2** | 변조 | 0.20–0.60 (`speed:=0.40 speed_mod_amp:=0.5 speed_mod_cycles:=10`) | 1.47 | 0.986 / 1.550 | 11.9 s | A·C·B 각 3 | — |

(숫자는 `core/figure8.py` 로 계산, 예산 1.0 · K_GEOM 0.1932: `Figure8(width=1.2, curve="circles", yaw_mode="tangent", laps=1, …)` 의 `max_tangential_accel()`·`peak_load(K_GEOM)`·`T_path`)

### 명령

```bash
amrall $T controller:=pure_pursuit speed:=0.3                                                     # S1
```

```bash
amrall $T controller:=pure_pursuit speed:=0.35 speed_mod_amp:=0.3 speed_mod_cycles:=6             # S2 A
```

```bash
amrall $T limit_mode:=friction speed:=0.35 speed_mod_amp:=0.3 speed_mod_cycles:=6    # S2 C
```

본 시험 (P 에 등속 또는 변조를 넣는다):

```bash
P="speed:=0.45"                                                       # 본 1 등속
P="speed:=0.40 speed_mod_amp:=0.5 speed_mod_cycles:=10"               # 본 2 변조
```

```bash
amrall $T $P controller:=pure_pursuit                       # A
```

```bash
amrall $T $P limit_mode:=friction                          # C (바퀴별 μ̂ 기본값)
```

```bash
amrall $T $P limit_mode:=motor_only                         # B (라운드 마지막)
```

### 순서

- **라운드 = A → C → B 한 번씩**, 같은 조건으로 3 라운드. 같은 비교군을 연달아 3 번 하지 않는다 (배터리 전압·바퀴 온도가 비교군 하나에만 몰리지 않게)
- 본 1 (등속) 3 라운드 → 배터리 확인 → 본 2 (변조) 3 라운드
- B 는 시뮬에서 6 회차 모두 발산했다. 실차에서 첫 B 가 0.5 m 로 서면 그 결과도 기록하고 다음으로 (B 의 결과 = "정지"). 두 사람 정지 절차를 B 때 특히
- 라운드 사이 배터리 셀 전압 (쉬는 상태). 3.5 V 아래면 그날 중지 — 다음 날 같은 순서로 이어서 하고, 날짜가 다른 회차는 표에 표시

### 매 회차

1. `amrall …` → 기동 로그 확인: (C) 바퀴별 τ̄ [0.1093 0.1113 0.1154 0.1055], "두 원 8자: 지름 0.60 m", 부하 %, (변조면) "속도 변조 ε … → … m/s, 접선 가속 최대 … m/s²"
2. 준비 끝·error 0 → `a` 무장
3. 완주 + 유지 5 s 뒤 Ctrl+C. 로그에 Traceback·"process has died" 없음, "DAMPING 뒤 모터 모드 해제" 확인
4. 기록표에 한 줄

## 4. 기록표

| 날짜 | 시각 | 단계 | 비교군 | 조건 | bag | 완주 / 정지 이유 (`/fault_monitor/fault`) | 배터리 (전·후) | 메모 |
|---|---|---|---|---|---|---|---|---|
| | | | | | | | | |

## 5. 분석

비교는 2026-11-16 실차 비교와 같은 스크립트·지표로 (`amr/발표/랩미팅/분석_스크립트/`, NUC 작업공간 루트에서, `source install/setup.bash` 뒤):

```bash
Z=amr/발표/랩미팅/분석_스크립트
for d in $A $B $C; do /usr/bin/python3 src/amr_basic_stack/controller/mpc/tools/plot_slip.py $d; /usr/bin/python3 $Z/slip_bins.py $d; done
/usr/bin/python3 $Z/wheel_sat.py $A $B $C
/usr/bin/python3 $Z/torque_stats.py $A $B $C
/usr/bin/python3 $Z/track_compare.py $A $B $C
```

- **λ 비교 속도 구간을 바꾼다**: 11-16 비교는 \|v̂\| 0.6–0.9 m/s 를 썼다. 이번 속도는 0.20–0.60 이라 `slip_bins.py` 의 **0.3–0.6 m/s 칸**을 쓴다 (분모가 ω_ε 근처인 저속은 λ 가 정의상 커진다)
- `slip_bins.py` 의 "순항 평균" 은 arm 뒤 6–13 s 로 고정돼 있다 — 이번 주행은 9.5–11.9 s 라 그 줄은 무시
- 이 스크립트들이 width 1.2 · 변조 기준에서 그대로 도는지는 **아직 확인 안 함** (11-16 bag 으로만 돌렸다). 첫 S1 bag 으로 먼저 돌려 본다
- 판정: **3 회차 범위 (최소–최대) 가 겹치지 않는 것만** 결론으로 (11-16 문서와 같은 규칙)

### 볼 것 (시뮬 예측과 맞는지)

| 질문 | 시뮬 (μ 0.26, 3 회차) | 실차에서 볼 지표 |
|---|---|---|
| 변조에서 C 가 A 보다 슬립이 작은가 | C λ RMS 0.098–0.107 vs A 0.188–0.199 | λ RMS 0.3–0.6 m/s, \|λ\| > 0.1 비율, 최악 바퀴 |
| 변조가 A 의 슬립을 늘리는가 | A 0.054–0.057 → 0.188–0.199 | A 본 1 vs 본 2 |
| C 는 대신 늦어지는가 | C 최대 오차 61–86 mm vs A 35–37 mm | 시간 기준 \|e_xy\| 최대, 경로 기준 거리 |
| A 의 토크가 바닥 한계를 넘는가 | A \|τ\| 최대 0.197–0.223, C 0.103–0.105 N·m | `torque_stats.py` 95 %·최대 |
| B 가 실차에서도 발산하는가 | 6/6 정지 | 완주 수, `wheel_sat.py` 뒤집힘 |

## 6. 끝낼 때

- `real_robot_checklist.md` 6 절. `pgrep -af ros2` 로 남은 노드 없음
- bag 이름과 기록표를 `analysis/torque_mpc_real/<날짜>_speedmod/` 에 (bag 자체는 gitignore, NUC `bags/` 에만)
