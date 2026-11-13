---
title: "MPC 용어집"
aliases:
  - "MPC Glossary"
tags:
  - mpc
  - glossary
  - notation
source_pages: "MPC 1장 전체"
---

# MPC 용어집

| 용어/기호 | 의미 |
|---|---|
| $u(k)$ | plant에 실제로 들어가는 조작변수 또는 control input |
| $\Delta u(k)$ | $u(k)-u(k-1)$. MPC가 최적화하는 입력 변화량 |
| $y(k)$ | plant output |
| $r(k)$ | set-point 또는 reference |
| $x_m(k)$ | 원 plant state |
| $x(k)$ | augmented state. 보통 $[\Delta x_m(k)^T,y(k)^T]^T$ |
| $A_m,B_m,C_m$ | 원 plant의 state-space matrices |
| $A,B,C$ | MPC 설계용 augmented model matrices |
| $N_p$ | prediction horizon. 미래 출력을 예측하는 샘플 수 |
| $N_c$ | control horizon. 미래 입력 변화량을 자유변수로 두는 샘플 수 |
| $T_p$ | moving horizon window의 시간 길이 |
| $Y$ | 예측 출력 벡터 |
| $\Delta U$ | 미래 입력 변화량 벡터 |
| $F$ | 현재 상태 $x(k_i)$가 미래 출력에 주는 영향을 모은 행렬 |
| $\Phi$ | 미래 입력 변화량 $\Delta U$가 미래 출력에 주는 영향을 모은 행렬 |
| $R_s$ | optimization window 안의 reference vector |
| $\bar R_s$ | constant reference를 쌓는 basis vector |
| $J$ | 비용함수. 출력 오차와 입력 변화량 penalty를 포함 |
| $\bar R$ | 입력 변화량 weight matrix |
| $r_w$ | $\bar R=r_wI$에서 쓰는 tuning parameter |
| Hessian | 여기서는 $\Phi^T\Phi+\bar R$ |
| $K_y$ | reference gain |
| $K_{mpc}$ | predictive controller의 state feedback gain |
| Embedded integrator | $\Delta u$를 적분해 $u$를 만드는 구조 |
| Moving horizon window | 길이는 고정이고 시작점이 매 샘플 이동하는 planning window |
| Receding horizon control | 최적 제어열 중 첫 입력만 적용하고 다음 샘플에서 다시 최적화하는 방식 |
| Observer | 출력 측정값으로 상태를 추정하는 동역학 시스템 |
| $K_{ob}$ | observer gain |
| $\hat x(k)$ | 추정 상태 |
| $\tilde x(k)$ | state estimation error, $x(k)-\hat x(k)$ |
| Observability | 출력과 입력 기록으로 초기 상태를 결정할 수 있는 성질 |
| Detectability | 관측 불가능 mode가 모두 안정한 성질 |
| Controllability | 입력으로 모든 state mode를 조작할 수 있는 성질 |
| Stabilizability | 제어 불가능 mode가 모두 안정한 성질 |
| MIMO | multi-input multi-output |
| SISO | single-input single-output |

## 자주 헷갈리는 쌍

### Prediction horizon vs control horizon

- $N_p$: 미래 출력을 어디까지 볼 것인가.
- $N_c$: 미래 입력 변화량을 몇 개까지 독립적으로 조정할 것인가.

보통 $N_c \leq N_p$이다.

### $u$ vs $\Delta u$

- $u(k)$: plant에 실제 들어간다.
- $\Delta u(k)$: optimizer가 계산한다.
- 관계: $u(k)=u(k-1)+\Delta u(k)$

### $x_m$ vs $x$

- $x_m$: 원 plant state.
- $x$: MPC용 augmented state. 출력 $y$와 증분 상태 $\Delta x_m$를 포함한다.

### Controller pole vs observer pole

- controller pole: $A-BK_{mpc}$의 eigenvalue.
- observer pole: $A-K_{ob}C$의 eigenvalue.
- 상태추정 기반 MPC에서는 두 집합을 분리해서 설계할 수 있다.

