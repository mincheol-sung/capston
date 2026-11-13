---
title: "MPC 1장 - Discrete-time MPC for Beginners"
aliases:
  - "MPC 1장"
  - "Discrete-time MPC for Beginners"
tags:
  - mpc
  - control
  - chapter-summary
source: "MPC_1장.pdf"
source_pages: "PDF p.28-69 / book p.1-42"
created: 2026-08-13
---

# MPC 1장 - Discrete-time MPC for Beginners

한 문장으로 정리하면, [[Model Predictive Control]]은 현재 상태에서 미래 출력 궤적을 예측하고, 출력 오차와 입력 변화량의 균형을 최소화하는 미래 제어열을 계산한 뒤 첫 제어입력만 적용하고 다음 샘플에서 다시 푸는 제어 방식이다.

## 1장 학습 지도

```mermaid
flowchart LR
    A["현재 정보 x(k_i)"] --> B["증분/적분기 포함 모델"]
    B --> C["미래 출력 예측<br/>Y = F x(k_i) + Phi Delta U"]
    C --> D["비용함수 J 최소화"]
    D --> E["최적 제어열 Delta U"]
    E --> F["첫 입력 Delta u(k_i)만 적용"]
    F --> G["새 측정값으로 반복"]
    G --> A
```

## 노트 목록

- [[01_MPC_Basic_Idea|01. MPC 기본 직관]]
- [[02_Augmented_State_Space_Model|02. 내장 적분기 상태공간 모델]]
- [[03_One_Window_Optimization|03. 한 최적화 창에서의 예측제어]]
- [[04_Receding_Horizon_Control|04. Receding Horizon Control]]
- [[05_MIMO_MPC|05. MIMO 예측제어]]
- [[06_State_Estimation|06. 상태추정과 관측기]]
- [[07_State_Estimate_MPC|07. 상태추정 기반 MPC]]
- [[08_Key_Equations|08. 핵심 수식 정리]]
- [[09_MATLAB_Workflows|09. MATLAB 구현 흐름]]
- [[10_Problems_Roadmap|10. 연습문제 로드맵]]
- [[MPC_Glossary|MPC 용어집]]
- [[_작성_유의사항|작성 유의사항 (수식·마크다운)]]

## 다음 장

- [[00_MPC2_Index|MPC 2장 인덱스]] — 같은 비용함수를 제약조건 아래에서 최소화하는 QP 문제로 확장한다. 1장의 $F$, $\Phi$, $K_{mpc}$ 를 그대로 쓴다.

## 응용 대조

- [[WP4 ↔ MPC 1장 대응]] — 메카넘 AMR 궤적추종 MPC([[Controller — 궤적추종 MPC (WP4)|WP4]])가 1장 구조에서 어디가 같고 어디가 다른지 절별로 대조.

## 1장에서 반드시 잡을 개념

- [[Moving Horizon Window]]: 일정한 길이의 미래 창을 매 샘플마다 앞으로 이동한다.
- [[Prediction Horizon]] $N_p$: 미래 출력을 몇 샘플 앞까지 예측할지 정한다.
- [[Control Horizon]] $N_c$: 미래 제어 변화량 $\Delta u$를 몇 개의 자유변수로 둘지 정한다.
- [[Embedded Integrator]]: $u$가 아니라 $\Delta u$를 최적화해서 적분 동작을 모델에 포함한다.
- [[Receding Horizon Control]]: 계산한 제어열 중 첫 샘플만 실제 plant에 넣는다.
- [[Observer]]: 실제로 측정되지 않는 상태를 출력 측정으로 추정한다.

## 읽는 순서

1. [[01_MPC_Basic_Idea|기본 직관]]에서 "계획을 매번 다시 세운다"는 그림을 잡는다.
2. [[02_Augmented_State_Space_Model|증분 상태공간 모델]]에서 왜 $\Delta u$를 입력으로 쓰는지 본다.
3. [[03_One_Window_Optimization|한 창에서의 최적화]]에서 $Y=Fx+\Phi\Delta U$와 비용함수를 이해한다.
4. [[04_Receding_Horizon_Control|receding horizon]]에서 최적 제어열을 실제 폐루프 제어법으로 바꾼다.
5. [[05_MIMO_MPC|MIMO 확장]]으로 차원과 controllability 조건을 확인한다.
6. [[06_State_Estimation|관측기]]와 [[07_State_Estimate_MPC|상태추정 MPC]]로 실제 구현 구조를 완성한다.

## 기억할 결론

MPC의 설계 핵심은 행렬 $F$, $\Phi$, $\bar R$를 만들고

$$
\Delta U = (\Phi^T\Phi+\bar R)^{-1}\Phi^T(R_s-Fx(k_i))
$$

를 계산하는 것이다. 실제 입력은 $\Delta U$ 전체가 아니라 첫 원소 $\Delta u(k_i)$만 사용한다.

