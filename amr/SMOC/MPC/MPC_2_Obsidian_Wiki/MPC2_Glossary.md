---
title: "MPC 2장 용어집"
aliases:
  - "MPC 2 Glossary"
  - "제약 MPC 용어집"
tags:
  - mpc
  - glossary
  - notation
  - constraints
source_pages: "MPC 2장 전체"
---

# MPC 2장 용어집

1장 기호는 [[MPC_Glossary|1장 용어집]]에 있다. 여기서는 2장에서 새로 나오는 것만 정리한다.

## 제약 관련

| 용어/기호 | 의미 |
|---|---|
| $\Delta u^{min}$, $\Delta u^{max}$ | 제어 증분의 하한과 상한 |
| $u^{min}$, $u^{max}$ | 제어 진폭의 하한과 상한. **증분변수 기준**이므로 $u_{ss}$ 를 뺀 값 |
| $y^{min}$, $y^{max}$ | 출력의 하한과 상한 |
| $u_{ss}$ | 제어신호의 정상상태값. 물리 한계를 증분 한계로 바꿀 때 뺀다 |
| $s_v$ | 슬랙 변수. 출력 제약을 소프트로 만들 때 쓴다 |
| $U^{min}$, $U^{max}$ | $u^{min}$, $u^{max}$ 를 $N_c$ 개 쌓은 열벡터 |
| $\Delta U^{min}$, $\Delta U^{max}$ | $\Delta u^{min}$, $\Delta u^{max}$ 를 $N_c$ 개 쌓은 열벡터 |
| $C_1$ | $u(k_i-1)$ 을 $N_c$ 번 복사하는 열벡터 |
| $C_2$ | $\Delta u$ 를 누적합으로 바꾸는 하삼각 1 행렬 |
| $M$, $\gamma$ | 제약 데이터 행렬과 벡터. $M\Delta U\le\gamma$ |
| Hard constraint | 반드시 지켜야 하는 물리적 제약. 주로 $u$, $\Delta u$ |
| Soft constraint | 슬랙 변수로 완화한 제약. 주로 $y$, $x$ |

## 최적화 관련

| 용어/기호 | 의미 |
|---|---|
| Quadratic Programming (QP) | 이차 목적함수를 선형 제약 아래에서 최소화하는 문제 |
| Decision variable | 최적화하는 변수. MPC에서는 $\Delta U$ |
| $E$, $F$ | 표준 QP의 Hessian과 1차항 계수. MPC에서는 $2(\Phi^T\Phi+\bar R)$ 와 $-2\Phi^T(R_s-Fx)$ |
| $x^0$ | 무제약 전역 최적해 $-E^{-1}F$ |
| Lagrange multiplier $\lambda$ | 등식제약을 목적함수에 흡수시킬 때 붙는 승수. dual variable과 같다 |
| Active constraint | $M_ix=\gamma_i$ 로 등호가 성립하는 제약 |
| Inactive constraint | $M_ix<\gamma_i$ 로 여유가 있는 제약 |
| $S_{act}$ | 활성 제약의 인덱스 집합 |
| $M_{act}$, $\gamma_{act}$ | 활성 제약만 남긴 데이터 행렬과 벡터 |
| Kuhn-Tucker (KKT) conditions | 부등식 제약 최적화의 필요조건 네 개 |
| Complementary slackness | $\lambda^T(Mx-\gamma)=0$. 승수와 여유 중 하나는 항상 0 |
| Working set | Active set method에서 매 스텝 활성으로 가정하는 제약 집합 |
| Feasible / Infeasible | 제약을 모두 만족하는 해가 존재하는가 |
| Primal method | 결정변수를 기준으로 푸는 방법. Active set method가 대표 |
| Dual method | Lagrange 승수를 기준으로 푸는 방법. Hildreth가 대표 |
| Dual variable | Lagrange 승수의 최적화 문헌식 표현 |
| $H$, $K$ | Dual QP의 데이터. $H=ME^{-1}M^T$, $K=\gamma+ME^{-1}F$ |
| Hildreth QP | 성분별 Gauss-Seidel에 음수 사영을 붙인 dual QP 알고리즘 |
| Shadow price | 제약을 완화했을 때 목적함수가 개선되는 비율. $\lambda_i$ 의 해석 |
| Constraint conflict | 서로 다른 제약이 동시에 활성이 되어 선형 종속이 되는 상황 |

## 자주 헷갈리는 쌍

### $u$ 물리값 vs $u$ 증분값

MPC의 $u(k)$ 는 증분 변수다. 실제 물리량은 $u_{\text{물리}}=u+u_{ss}$ 다. 밸브가 15~80%이고 정상값이 30%면 제약은 $-15\le u\le 50$ 이 된다. 이 변환을 빠뜨리는 것이 2장에서 가장 흔한 실수다.

### 활성 제약 vs 위반된 제약

- 활성: $M_ix=\gamma_i$. 최적해가 경계에 놓인 정상 상태다.
- 위반: $M_ix>\gamma_i$. 제약을 지키지 못한 상태이며 QP를 다시 풀어야 한다.

무제약 해가 제약을 위반한다는 것은 그 제약이 활성이 될 가능성이 높다는 신호일 뿐, 반드시 활성이 되는 것은 아니다.

### 등식제약 개수 vs 부등식제약 개수

- 등식: 결정변수 개수 이하여야 한다. 같으면 최적화 여지가 없고, 크면 실행 불가능이다.
- 부등식: 개수 제한이 없다. 단 **활성** 제약의 개수가 결정변수 개수 이하여야 한다.

### Primal vs Dual

- Primal: 변수 $x$, 제약 $Mx\le\gamma$ (복잡함).
- Dual: 변수 $\lambda$, 제약 $\lambda\ge 0$ (단순함).

Dual의 제약이 성분별로 독립적이라는 점이 Hildreth를 가능하게 만든다.

### 첫 샘플 제약 vs 전 구간 제약

- 첫 샘플: 제약 개수가 적고 실제 적용되는 제어값을 왜곡하지 않는다.
- 전 구간: 제약 개수가 $N_c$ 배가 되고, 실행되지 않을 미래 제약이 활성화되어 첫 제어값을 왜곡할 수 있다.

성능 차이는 대개 미미하다.

### 변화율 제약 vs 진폭 제약

| 항목 | 변화율 | 진폭 |
|---|---|---|
| $\Delta U$ 와의 관계 | 항등 행렬 | 누적합 행렬 $C_2$ |
| 우변 | 상수 | $u(k_i-1)$ 포함 |
| 매 샘플 갱신 | 불필요 | 필요 |
| 활성일 때의 신호 | 성분이 경계값 | 성분이 0 |

## 연결

- 1장 용어: [[MPC_Glossary|MPC 용어집]]
- 수식 색인: [[14_Key_Equations|핵심 수식 정리]]
- 전체 목차: [[00_MPC2_Index|MPC 2장 인덱스]]
