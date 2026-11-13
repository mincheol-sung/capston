---
title: "10. 연습문제 로드맵"
aliases:
  - "MPC 1장 Problems"
tags:
  - mpc
  - exercises
  - roadmap
source_pages: "PDF p.66-69 / book p.39-42"
---

# 연습문제 로드맵

1장의 problems는 단순 계산보다 "MPC 설계 절차를 손으로 재구성할 수 있는가"를 확인하는 흐름이다.

| 문제 | 주제 | 먼저 볼 노트 |
|---|---|---|
| 1.1 | 1장 road map과 핵심 수식 정리 | [[00_MPC1_Index]], [[08_Key_Equations]] |
| 1.2 | 운전 상황을 predictive control로 해석 | [[01_MPC_Basic_Idea]] |
| 1.3 | constant input disturbance가 있는 plant의 augmented model | [[02_Augmented_State_Space_Model]], [[05_MIMO_MPC]] |
| 1.4 | $Y$, $\Delta U$, $J_{min}$, $r_w$ 영향 계산 | [[03_One_Window_Optimization]] |
| 1.5 | single tank 1차 plant의 MPC 설계 및 simulation | [[02_Augmented_State_Space_Model]], [[09_MATLAB_Workflows]] |
| 1.6 | time-delay transfer function을 state-space로 보고 MPC 설계 | [[03_One_Window_Optimization]], [[04_Receding_Horizon_Control]] |
| 1.7 | noisy quadratic signal에서 미지 계수 추정 observer | [[06_State_Estimation]] |
| 1.8 | under-damped system의 MPC + observer + disturbance rejection | [[06_State_Estimation]], [[07_State_Estimate_MPC]] |
| 1.9 | unmodelled time delay에 대한 robustness와 $r_w$ tuning | [[04_Receding_Horizon_Control]] |
| 1.10 | DC motor observability, augmented model, MPC gain, observer 설계 | [[06_State_Estimation]], [[07_State_Estimate_MPC]] |

## 풀이 공통 순서

1. plant model을 $A_m,B_m,C_m$ 또는 transfer function으로 정리한다.
2. transfer function이면 discrete-time state-space model로 바꾼다.
3. augmented model $A,B,C$를 만든다.
4. controllability와 observability를 확인한다.
5. $N_p,N_c,\bar R$를 지정한다.
6. $F,\Phi,\bar R_s$를 만든다.
7. $K_y,K_{mpc}$ 또는 $\Delta U$를 계산한다.
8. 폐루프 eigenvalue를 확인한다.
9. observer가 필요하면 $K_{ob}$를 설계한다.
10. simulation에서 $\Delta u$, $u$, state, output을 샘플마다 갱신한다.

## 문제별 힌트

- 1.3: constant disturbance는 차분을 취하면 사라지지만, augmented model에 적분 동작이 남아 steady-state disturbance rejection을 돕는다.
- 1.4: $r_w=0$과 $r_w>0$에서 $\Delta U$와 $J_{min}$를 비교하면 "빠른 제어 vs 입력 억제" tradeoff가 보인다.
- 1.5: tank level은 1차 시스템이므로 $x_m(k)=y(k)$로 두면 augmented state가 $[\Delta y(k),y(k)]^T$가 된다.
- 1.6: time delay는 state-space 차수를 키운다. 긴 delay plant는 $N_p$ 선택의 영향을 강하게 받는다.
- 1.7: 상태를 $[ \beta(k-10)^2,\ \beta(k-10),\ \beta]^T$처럼 잡으면 미지 계수 $\beta$가 상태로 들어간다.
- 1.8-1.9: under-damped plant와 delay mismatch에서는 $r_w$가 robustness tuning knob로 중요하다.
- 1.10: speed measurement와 position measurement의 observability 차이를 비교하는 문제다.

