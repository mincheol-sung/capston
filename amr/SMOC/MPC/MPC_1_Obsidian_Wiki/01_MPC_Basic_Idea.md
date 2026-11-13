---
title: "01. MPC 기본 직관"
aliases:
  - "MPC 기본 직관"
  - "Predictive Control Intuition"
tags:
  - mpc
  - control
  - intuition
source_pages: "PDF p.28-30 / book p.1-3"
---

# MPC 기본 직관

[[Model Predictive Control]]의 목표는 미래의 조작변수 궤적 $u$를 계산해서 미래의 plant output $y$가 원하는 동작에 가깝게 움직이도록 하는 것이다. 계산은 제한된 시간 창 안에서 수행되고, 창의 시작점에 있는 현재 정보가 예측의 출발점이 된다.

## 일상 계획 비유

책은 하루 업무 계획을 예로 든다. 9시에 앞으로 8시간 계획을 세우지만, 실제로는 첫 1시간만 실행한다. 10시가 되면 진행 상황을 측정하고, 다시 10시부터 8시간짜리 계획을 세운다. 이 과정을 목표가 끝날 때까지 반복한다.

MPC도 같은 구조다.

| 계획 비유 | MPC 대응 |
|---|---|
| 현재 진행 상황 확인 | 상태 $x(k_i)$ 측정 또는 추정 |
| 앞으로 8시간 계획 | moving horizon window |
| 업무 수행 방식 선택 | 조작변수 $u$, 또는 증분 $\Delta u$ |
| 제한된 능력/시간/자원 | hard/soft constraints |
| 가장 좋은 계획 선택 | 비용함수 $J$ 최소화 |
| 첫 1시간만 실행 | 첫 제어입력만 plant에 적용 |
| 다음 시간에 다시 계획 | receding horizon 반복 |

## 세 가지 필수 도구

- 모델: 현재 행동이 미래 출력에 어떤 영향을 주는지 예측한다.
- 측정: 현재 plant 상태나 출력 정보를 얻는다.
- 제어 구현: 계산된 첫 제어입력을 plant에 적용한다.

## 핵심 용어

- [[Moving Horizon Window]]: 현재 시각 $t_i$에서 $t_i+T_p$까지의 시간창이다. 길이 $T_p$는 일정하지만 시작점 $t_i$가 계속 앞으로 이동한다.
- [[Prediction Horizon]]: 미래를 얼마나 멀리 예측할지 정하는 길이다. 이 장의 이산시간 표기에서는 주로 $N_p$를 쓴다.
- [[Receding Horizon Control]]: 최적 제어 궤적 전체를 계산하되 첫 샘플만 실행하고 나머지는 버린다.
- 현재 정보 $x(t_i)$ 또는 $x(k_i)$: 미래 예측의 초기조건이다. 직접 측정할 수도 있고 [[Observer]]로 추정할 수도 있다.
- 비용함수 $J$: set-point와 예측 출력 사이의 오차, 그리고 제어입력 변화량의 크기를 수치화한 목적함수다.

## 모델 구조 선택

1장에서 책은 여러 predictive control 모델 구조를 빠르게 비교한다.

- FIR/step response 모델: 공정 시간지연, 응답시간, gain을 직관적으로 보여주지만 안정한 plant에 적합하고 차수가 커질 수 있다.
- transfer function 모델: GPC 계열에서 많이 쓰였지만 multivariable plant 처리에는 한계가 있다.
- state-space 모델: 이 책의 기본 설계 프레임이다. 현재 정보가 상태변수로 표현되며, [[LQR]]과의 연결도 자연스럽다.

## 이 장의 진행

1. [[02_Augmented_State_Space_Model|내장 적분기 모델]]을 만든다.
2. [[03_One_Window_Optimization|한 최적화 창]]에서 미래 출력과 최적 $\Delta U$를 계산한다.
3. [[04_Receding_Horizon_Control|receding horizon]]으로 폐루프 제어법을 만든다.
4. [[05_MIMO_MPC|MIMO 시스템]]으로 확장한다.
5. [[06_State_Estimation|관측기]]와 [[07_State_Estimate_MPC|상태추정 MPC]]로 실제 구현 형태를 완성한다.

