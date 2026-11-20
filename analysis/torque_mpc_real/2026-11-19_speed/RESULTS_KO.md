# 2026-11-19 저녁 — 0.7 m/s 기준에서 실제로 0.7 로 갔나 (bag 173910, PP)

판: `bags/2026-11-19/173910` — `controller:=pure_pursuit shape:=circles8 width:=2.0 yaw_mode:=tangent speed:=0.7 crossing_blend_m:=0.2`, est_mode kf.
기준: 호길이 6.28 m, 순항 0.70, 가감속 0.40 m/s² (0 → 0.7 에 1.75 s), 부하 63 % (예산 65 %, V_LIMIT 1.55 m/s).

| 속도 출처 (`speed_check.py`) | 기준 순항 구간 중앙 | 평균 | 10–90 % | ≥ 0.65 시간 |
|---|---|---|---|---|
| 기준 | 0.700 | 0.686 | 0.656–0.700 | 90 % |
| 엔코더 pinv(T⁻¹)ω (k 0.1932) | 0.706 | 0.713 | 0.636–0.806 | 87 % |
| SLAM 중심 (IMU 점 보정, 0.5 s 차분 `speed_plot.py`) | 0.696 | — | 0.563–0.821 | — |
| 추정기 (kf) | 0.723 | 0.731 | 0.543–0.898 | 75 % |
| PP 명령 v_x | 0.703 | 0.710 | 0.647–0.811 | 89 % |

- 순항 8.2 s 동안 간 거리: 기준 5.64 m, SLAM 중심 5.82 m, 엔코더 5.84 m → 실제로 0.7 m/s 로 갔다. 엔코더 ≈ SLAM 이라 순항 중 큰 헛돎 없음.
- 진행 방향 오차 (SLAM 중심 − 기준): 순항 평균 −2.3 cm (= −33 ms 뒤처짐), 최소 −18.1 cm (12.8 s, 두 번째 원 끝). 뒤처지면 PP 가 k_along 으로
  0.9 m/s 까지 올려 따라잡는다 (그림 `speed_173910.png`). SLAM 속도의 0.5 s 안 흔들림(0.45–1.0)은 엔코더에 없는 것이 많다 — SLAM 10 Hz 보정 흔들림과
  실제 속도 변동을 이 자료로는 못 나눈다.

## 같은 저녁 174102 (speed 1.0) — 기준 노드가 시작하자마자 종료 (exit 1)
같은 인자로 figure8_reference_node 만 따로 띄워 재현: "기준 부하 1.386 > 예산 1.007" (부하 = v·(1 + k/R), 예산 = 0.65 × V_LIMIT 1.55,
V_LIMIT = r × 정격 38.75 rad/s). load_budget 0.65 그대로일 때 허용 최고 속도 (노드로 확인):

| width (R) | 통과 | 거부 |
|---|---|---|
| 2.0 (0.5 m) | 0.72 | 0.74 |
| 3.0 (0.75 m) | 0.80 | 0.82 |
| 4.0 (1.0 m) | 0.84 | 0.86 |

```
/usr/bin/python3 analysis/torque_mpc_real/2026-11-19_speed/speed_check.py bags/2026-11-19/173910
/usr/bin/python3 analysis/torque_mpc_real/2026-11-19_speed/speed_plot.py bags/2026-11-19/173910 analysis/torque_mpc_real/2026-11-19_speed/speed_173910.png
```


## 같은 저녁 180025 (est_mode slam) vs 180149 (est_mode kf) — 나머지 인자 같음
`amrall use_mpc:=true limit_mode:=friction use_w:=true crossing_blend_m:=0.2 shape:=circles8 width:=3.0 yaw_mode:=tangent speed:=1.0 load_budget:=0.9`
(바퀴별 μ̂ (11) 기본 → τ̄ [0.1093 0.0721 0.1154 0.0886]). 둘 다 완주, 정지 없음. 한 판씩이라 판 사이 흔들림과 추정기 차이를 아직 못 나눈다.

| 항목 | slam (180025) | kf (180149) | 도구 |
|---|---|---|---|
| SLAM 기준 위치 RMS / 최대 mm | 68.9 / 156 | 100.3 / 208 | compare_slam.py |
| e_t 평균 (진행, − 뒤처짐) / e_c 평균 mm | −11.5 / +16.3 | −52.4 / +5.4 | 〃 |
| 요 RMS / 최대 ° | 4.80 / 13.1 | 4.60 / 12.0 | 〃 |
| 순항 중앙 속도 엔코더 / SLAM | 0.957 / 0.944 | 0.972 / 0.972 | speed_check.py |
| 순항 8.4 s 거리 기준 / SLAM | 8.18 / 7.89 m | 8.18 / 8.10 m | 〃 |
| 진행 오차 구간 평균 가속 / 순항 앞 / 순항 뒤 / 감속 mm | +20 / −31 / −10 / −18 | −4 / −59 / −51 / −96 | lag_sat.py |
| 상한 닿음 % FL FR RL RR (아무 바퀴) | 47 65 33 34 (82) | 38 47 17 21 (63) | 〃 |
| 추정기 − SLAM 중심 앞뒤 평균 / RMS mm | −2.3 / 45.4 | −0.5 / 33.8 | est_compare.py |
| 추정기 − SLAM 옆 RMS mm, 요 RMS ° | 25.8, 1.37 | 22.5, 1.50 | 〃 |
| 추정기 v_x − 엔코더 RMS m/s | 0.161 | 0.130 | 〃 |
| 추정기 한 주기 튐 > 1 cm (합) | 120 번 (571 cm) | 64 번 (257 cm) | 〃 |

- 추정기 출력은 kf 가 낫다: SLAM 과 앞뒤 RMS 34 vs 45 mm, 엔코더 속도와 RMS 0.130 vs 0.161, 한 주기 튐 절반. 둘 다 SLAM 대비 평균 편향 ≤ 2.3 mm.
- 추종은 slam 판이 나았지만 원인은 추정기 편향이 아니다 (편향 ≈ 0). 두 판의 속도·진행 오차 모양이 같은 자리에서 같이 꺼진다
  (그림 `kf_vs_slam_w3_v1.png`: 5.5 s −15/−19 cm, 9.5 s −12/−19 cm) — 트랙과 상한이 만든 뒤처짐이고, kf 판이 그 자리에서 더 깊다.
- 두 판 모두 FR 상한(0.0721)에 47~65 % 붙어 있다 — width 3.0·1.0 m/s 정상 원 옆힘만 FR 의 84 % (계산, stack_changelog (12)). 1.0 에 못 미친 주원인.
- 추정기 v_x 빠른 몫 지연 (상호상관): slam +84 ms (상관 0.55), kf +142 ms (0.62) — 상관이 낮아 숫자를 믿기 어렵다.

```
/usr/bin/python3 src/amr_basic_stack/controller/mpc/tools/compare_slam.py bags/2026-11-19/180025 bags/2026-11-19/180149
/usr/bin/python3 analysis/torque_mpc_real/2026-11-19_speed/est_compare.py bags/2026-11-19/180025 bags/2026-11-19/180149
/usr/bin/python3 analysis/torque_mpc_real/2026-11-19_speed/lag_sat.py bags/2026-11-19/180025 bags/2026-11-19/180149
/usr/bin/python3 analysis/torque_mpc_real/2026-11-19_speed/kf_vs_slam_plot.py bags/2026-11-19/180025 bags/2026-11-19/180149 analysis/torque_mpc_real/2026-11-19_speed/kf_vs_slam_w3_v1.png
```
