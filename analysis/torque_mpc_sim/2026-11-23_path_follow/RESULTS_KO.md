# path following (기준 시각을 로봇 진행에 묶기) — 시뮬 확인 (2026-11-23)

계기: 실차 계단 변조 8자 17 판에서 위치 오차 제곱의 66~86 % 가 진행 방향 뒤처짐 (최대 −0.29 m), 경로 이탈 (contouring) 은 14~34 %
(`analysis/torque_mpc_real/2026-11-23_load_real/contour_check.py`). 슬립을 막으려 토크를 줄이면 늦고, 늦은 만큼 오차로 잡혔다.
사용자 제안: 경로만 주고 알아서 따라가게 해서 오차·슬립·랩타임을 비교 (path following — Aguiar·Hespanha·Kokotović 2008).

구현: `figure8_reference_node` `path_follow_lead_s` (launch `path_follow_lead:=0.05`, 주면 발행 50 Hz), `core/path_follow.py`.
  주행 구간에서 t_proj = 로봇 (x, y) 의 경로 정사영 시각, t_ref ← clamp(t_ref, t_proj, t_proj + lead).
  정사영 지점 기준 속력 < 0.1 m/s 또는 정사영 앞 lead + 0.25 s 안에 기준 감속이 있으면 위쪽 상한을 안 건다 (아래 "멈춤").
조건: circles8 width 1.6, laps 1, speed 0.69 · accel 3.0 계단 변조 (마찰 밖 기준), 바닥 μ 0.55, 플랜트 h 0.2, C 는 mpc_com_height 0.2. 참값 λ.
`runner.sh cases.txt runs 2 …` / `cases_pf.txt runs2` (기록기 `pf_rec.py`: 주행 시간·contouring·참값 슬립·npz).

| 판 | 주행 시간 | contouring RMS / 최대 | 시간 오차 RMS | yaw RMS | |λ|>0.5 % | |λ|>0.1 % | λ RMS |
|---|---|---|---|---|---|---|---|
| C 시간 추종 (runs/Ctrack) | 9.79 s | 22.6 / 77 mm | 32 mm | 0.3° | 0 | 7.4 | 0.040 |
| C path following (runs2/Cpf) | 9.54 s | 24.0 / 74 mm | 29 mm | 0.9° | 0 | 12 | 0.039 |
| PP 시간 추종 (runs/PPtrack) | 9.77 s | 157 / 407 mm | 246 mm | 8.9° | 21.7 | 31 | 0.333 |
| PP path following (runs2/PPpf) | **10.90 s (+1.2 s)** | **137 / 356 mm** | 165 mm | 10.8° | 22.8 | 33 | 0.339 |

- 의도대로 동작: PP 는 늦은 만큼 기준이 기다려 주행이 1.2 s 길어지고 경로 이탈이 157 → 137 mm. 슬립은 그대로 (PP 는 바퀴 속도를 밀어붙인다).
- 시뮬 C 는 마찰 모델이 정확해 거의 안 늦는다 — 기준이 기다릴 일이 없어 시간 추종과 같다. 차이는 실차 (C 도 최대 0.29 m 늦음) 에서 볼 것.
- 판마다 1 번.

## 멈춤 — 첫 구현 (runs/Cpf) 에서 C 가 끝점 6.5 cm 앞에서 140 s 섰다 (dbg/bag)
- 기준 시각이 끝 감속 (10.02 s~) 직전 9.94 s 에 묶임 → MPC 가속도 앞먹임 (acc_dt 0.2 s 중앙 차분) 이 감속을 담아 F·U_s = [−11.7, 0.6, 0.6] N (브레이크),
  위치 오차 +19 mm (앞으로) 와 맞서 F·τ ≈ 0 (τ ≈ [−0.07, −0.08, +0.06, +0.07] — F 의 영공간 꼴 [−1, −1, 1, 1]). 기준 시각이 안 움직이니 앞먹임도 그대로.
- PP 는 가속도 앞먹임이 없어 (v_ref 만) 안 멈췄다.
- 고침: 정사영 앞 lead + 0.25 s 안에 감속이 있으면 묶지 않는다 (`path_follow.decel_ahead`). 늦음은 가속·고속 선회에서 쌓이고 감속에서는 로봇이 따라잡는다.
