# 2026-11-27 슬립 되먹임 이득 만들기 — 분석 스크립트

2026-11-28 복구: 이 스크립트들은 11-27 세션 (567b0424) 의 scratchpad `slip1127/` 에서 썼고, 그 폴더는 11-28 디스크 정리 때 지워졌다.
세션 기록 (`~/.claude/projects/-home-smoc-capston/567b0424-….jsonl`) 의 마지막 Write 내용 + 그 뒤 Edit 을 차례로 적용해 되살렸다. 그림 (png) 은 없다 — 다시 돌려야 한다.

| 파일 | 하는 일 |
|---|---|
| `yarr_sweep.py` | **배율 훑기 (11-27 19:24~19:44 `*_yarr_mpc` 21 판)**: 배율별 > 50 % · > 25 % 슬립 (네 바퀴 최대), 평균 λ, 미끄럼 속도 r·e_ω RMS, 헛돎 몫, 바퀴별 > 50 %, 위치·헤딩·속도·요 속도 RMS, 틱간 Δτ, 더한 Δτ, 모터 온도, SLAM 튐 — changelog 11-27 (4)·(5) 의 표 |
| `plot_gain_sweep.py` | 배율 훑기 그림 |
| `plot_sfb_compare.py` · `plot_sfb_run.py` | 되먹임 켬/끔 판 비교·판 하나 시간축 |
| `kw_design.py` | K_ω 설계 (lqr · ik) 계산 |
| `track_err.py` · `state_err.py` | 추종 오차 (`*_trk.npz`) · 상태 오차 |
| `plot_mpc_pp_track.py` · `rr_why.py` · `wheel_mu_lambda.py` · `sim_check.py` · `sim_sfb.sh` · `qscreen.cpp` · `slip_check.py` | 그 밖 점검 |
| `nominal_vs_feedback.py` | (원래 리포에 있던 것) 되먹임 두 입력 x̂_w · x_e 비교 |

순서 (yarr_sweep): bag → `../2026-11-25_polyline_pass_real/read_bags.py` (npz) → `track_err.py` (`*_trk.npz`) → `yarr_sweep.py <npz 폴더> <bag 날짜 폴더>`.
11-27 bag 원본은 외장 SSD T9 `capston_bags/2026-11-27/` 에 있다 (11-28 옮김). 결과 숫자는 `amr/notes_amr/stack_changelog.md` 11-27 (2)~(6) 과 `torque_mpc.yaml` slip_feedback 주석.
