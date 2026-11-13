---
title: "12. SISO 제약 제어의 닫힌형 해와 제약 순위"
aliases:
  - "Closed-form Constrained Control"
  - "SISO 닫힌형 해"
  - "Ranking of Constraints"
  - "제약 순위"
tags:
  - mpc
  - laguerre
  - constraints
  - closed-form
  - siso
source_pages: "PDF p.47-51 / book p.131-135"
---

# SISO 제약 제어의 닫힌형 해와 제약 순위

복잡한 문제에 단순화된 해가 있는가는 엔지니어가 늘 던지는 질문이다. 다입력 다출력에서는 서로 얽힌 변수가 많아 QP가 활성 제약을 찾는 가장 효과적인 도구다. 그러나 SISO에서는 변수가 $u(k)$, $\Delta u(k)$, $y(k)$ 셋뿐이라 **해석적 해**를 찾을 수 있고, 관측기도 없앨 수 있다. 계산력이 제한된 마이크로컨트롤러 구현에 맞는 접근이다.

## 왜 첫 샘플의 제약만 보면 되는가

전통적 MPC는 최적화 창 안의 모든 변수에 제약을 걸어 SISO에서도 제약 수가 빠르게 늘어난다. 제약이 많을 때 활성 제약을 골라내는 데는 QP가 최선이다. 그러나 이동구간 원리로 돌아가면, 창 전체에 대해 최적해를 구하지만 **구현되는 것은 첫 샘플뿐**이다.

따라서 첫 샘플의 제약($\Delta u$ 든 $u$ 든)이 활성이면, 그 활성 제약을 등식 제약으로 두고 최적해를 구하며 나머지 제약은 해에 영향을 주지 않는다. 나머지 제약이 해에 영향을 주는 경우는 그것들이 활성이 되면서 첫 샘플의 제약은 비활성인 경우뿐이다. 실무적으로는 창 안의 변수 중 첫 샘플에만 제약을 걸고, 나머지는 비활성이거나 폐루프 성능에 미치는 영향이 작다고 가정하고 무시할 수 있다.

이 논리에 따르면 SISO 시스템의 제약은 제어변수, 제어 증분, 출력변수 각각의 상한과 하한, 여섯 개다. 각각의 닫힌형 해를 차례로 본다. 공통 설정은 비용함수

$$
J=\eta^T\Omega\eta+2\eta^T\Psi x(k_i)
$$

와 무제약 해 $\eta=-\Omega^{-1}\Psi x(k_i)$ 이며, $x(k_i)$ 에는 설정치 정보가 들어 있다.

## 제어 증분 제약 $\Delta u^{min}\le\Delta u(k)\le\Delta u^{max}$

$\Delta u(k_i)=L(0)^T\eta$ 이므로 세 가지 경우로 나뉜다.

1. $\Delta u^{min}<L(0)^T\eta<\Delta u^{max}$ 이면 $\eta=-\Omega^{-1}\Psi x(k_i)$ 가 그대로 최적해다. 전역 최적해가 제약을 만족한다.
2. $L(0)^T\eta\le\Delta u^{min}$ 이면 이 제약이 활성 제약이 되어 등식이 된다. 최적해는 $\Delta u(k_i)=\Delta u^{min}$ 을 만드는 $\eta$ 인데, $\eta$ 값을 굳이 구할 필요 없이 **바로 $\Delta u(k_i)=\Delta u^{min}$ 을 취한다.**
3. $L(0)^T\eta\ge\Delta u^{max}$ 이면 마찬가지로 $\Delta u(k_i)=\Delta u^{max}$ 다.

$\Delta u(k_i)$ 를 얻으면 $u(k_i)=u(k_i-1)+\Delta u(k_i)$ 다.

왜 $\eta$ 를 구하지 않아도 되는가. 2장의 닫힌형 해 $\lambda_{act}=-(M_{act}\Omega^{-1}M_{act}^T)^{-1}(\gamma_{act}+M_{act}\Omega^{-1}\Psi x)$, $\eta=-\Omega^{-1}(\Psi x+M_{act}^T\lambda_{act})$ 에 $M_{act}=L(0)^T$ 를 넣고 $L(0)^T\eta$ 를 계산하면 정확히 $\gamma_{act}$, 즉 $\Delta u^{min}$ 이 나온다. 활성 등식 제약은 정의상 등호로 만족되므로 답은 제약값 자체다. 구현되는 것이 $\Delta u(k_i)$ 뿐이므로 $\eta$ 의 나머지 정보는 필요 없다.

## 제어 진폭 제약 $u^{min}\le u(k)\le u^{max}$

직전 제어신호 $u(k_i-1)$ 을 알고 있을 때 역시 세 경우다.

1. $u^{min}<u(k_i-1)+L(0)^T\eta<u^{max}$ 이면 전역 최적해가 답이다.
2. $u(k_i-1)+L(0)^T\eta\le u^{min}$ 이면 $u(k_i)=u^{min}$ 이고, 여기서 $\Delta u(k_i)=u^{min}-u(k_i-1)$ 을 얻는다.
3. $u(k_i-1)+L(0)^T\eta\ge u^{max}$ 이면 $u(k_i)=u^{max}$ 이고 $\Delta u(k_i)=u^{max}-u(k_i-1)$ 이다.

한계에서 계산한 $\Delta u(k_i)$ 정보가 중요하다. 이 값이 **피드백 변수로 상태 갱신에 쓰이기 때문**이다. 2장 [[01_Motivational_Examples|Example 2.2]]에서 "포화된 실제 $\Delta u$ 를 관측기에 알려주라"고 한 것과 같은 이유다. 잘린 값을 모델에 반영하지 않으면 모델과 플랜트가 어긋난다.

## 출력 제약 $y^{min}\le y(k)\le y^{max}$

상태공간 모델 $(A,B,C)$ 로 설계했을 때, 한 스텝 뒤의 상태와 출력 예측은

$$
x(k_i+1\mid k_i)=Ax(k_i)+BL(0)^T\eta
$$

$$
y(k_i+1\mid k_i)=CAx(k_i)+CBL(0)^T\eta
$$

이다. 출력 제약은 이 한 스텝 예측에 건다.

1. $y^{min}<CAx(k_i)+CBL(0)^T\eta<y^{max}$ ($\eta=-\Omega^{-1}\Psi x(k_i)$) 이면 무제약 해가 답이다.
2. 하한이 위반되어 $CAx(k_i)+CBL(0)^T\eta\le y^{min}$ 이면 이 제약이 활성이 된다. 제어 제약처럼 값을 바로 읽을 수는 없지만, 등식 제약으로 취급하면 해석적 해가 있다. 만족시킬 부등식은

$$
-CBL(0)^T\eta\le-y^{min}+CAx(k_i)
$$

이고, $M_{act}=-CBL(0)^T$ 로 두면 Lagrange 승수는

$$
\lambda_{act}=-(M_{act}\Omega^{-1}M_{act}^T)^{-1}\big(-y^{min}+CAx(k_i)+M_{act}\Omega^{-1}\Psi x(k_i)\big)
$$

이고, 최적 파라미터는

$$
\eta=-\Omega^{-1}\big(\Psi x(k_i)+M_{act}^T\lambda_{act}\big)
$$

이다.

3. 상한이 위반되면 부등식은

$$
CBL(0)^T\eta\le y^{max}-CAx(k_i)
$$

이고, $M_{act}=CBL(0)^T$ 로

$$
\lambda_{act}=-(M_{act}\Omega^{-1}M_{act}^T)^{-1}\big(y^{max}-CAx(k_i)+M_{act}\Omega^{-1}\Psi x(k_i)\big)
$$

를 구한 뒤 같은 식으로 $\eta$ 를 얻는다.

이 식들은 2장 [[09_Closed_Form_Active_Set|활성집합을 알 때의 닫힌형 해]] $\lambda_{act}=-(M_{act}E^{-1}M_{act}^T)^{-1}(\gamma_{act}+M_{act}E^{-1}F)$ 에 $E=\Omega$, $F=\Psi x(k_i)$, $\gamma_{act}=\mp y^{min/max}\pm CAx(k_i)$ 를 넣은 것이다. $M_{act}$ 가 행벡터 하나이므로 $M_{act}\Omega^{-1}M_{act}^T$ 는 스칼라이고, 역행렬은 나눗셈 한 번이다. 이것이 마이크로컨트롤러에서 가능한 이유다.

## 제약 순위 (Ranking of the Constraints)

세 종류 제약이 모두 위반되거나 둘이 동시에 위반되면 어떻게 하는가. 위반된 것들을 모두 활성 제약으로 놓고 풀면 되는가. **답은 아니오다.** 주어진 $k_i$ 와 $x(k_i)$ 에서 제약 최적해 $\eta$ 는 **활성 제약이 하나일 때만** 존재한다.

이유는 세 종류의 제약이 선형 종속이기 때문이다. $M_{act}$ 는 $\Delta u$ 제약에서 $\pm L(0)^T$, $u$ 제약에서도 $\pm L(0)^T$, $y$ 제약에서 $\pm CBL(0)^T$ 다. 셋 다 같은 행벡터 $L(0)^T$ 의 스칼라 배다. 두 개를 함께 쌓으면 $M_{act}$ 의 행이 종속이 되어 $\det(M_{act}\Omega^{-1}M_{act}^T)=0$ 이고 $\lambda_{act}$ 가 존재하지 않는다. 결정변수 $\eta$ 로 표현하면 첫 샘플에 관한 어떤 제약도 결국 $L(0)^T\eta$ 하나에 대한 조건이라서, 서로 다른 값을 동시에 요구할 수 없는 것이다. [[10_Constraints_on_u|Example 3.8]]의 충돌이 바로 이 상황이었다.

교재의 $u$ 제약에 대한 $M_{act}$ 표기에는 $\pm L(0)^T\Delta t$ 처럼 $\Delta t$ 가 붙어 있는데, 이산시간에서 $u(k_i)=u(k_i-1)+L(0)^T\eta$ 이므로 $\Delta t$ 는 필요 없다. 연속시간 판의 표기가 남은 것으로 보이며, 결론(선형 종속)에는 영향이 없다.

그래서 제약에 **중요도 순위**를 매기고, 가장 중요한 것을 만족시키며 덜 중요한 것은 놓아 준다.

가장 중요한 것은 **제어신호에 관한 제약**이다. 진폭이든 증분이든 물리적 하드 한계이기 때문이다. 덜 중요한 것은 **출력 제약**이다. 출력 제약의 효과는 예측에 쓰는 모델의 정확도에 의존한다. 출력 제약은 한 스텝 예측 $y(k_i+1\mid k_i)=CAx(k_i)+CBL(0)^T\eta$ 에 걸리므로 모델이 틀리면 제약도 틀린다. 관측기를 쓰면 $x(k_i)$ 대신 $\hat x(k_i)$ 가 들어가 추정 오차까지 제약에 영향을 준다. 2장 [[02_Constraint_Formulation|§2.3.1]]에서 출력 제약을 소프트로 두라고 한 것과 같은 판단이다.

순위를 진폭 $u$ > 증분 $\Delta u$ > 출력 $y$ 로 정했을 때 구현 순서는 다음과 같다.

1. 가장 덜 중요한 $y$ 제약이 위반되면 그것을 활성 제약으로 두고 $y$ 제약을 만족하는 $\eta$ 를 구한다.
2. 이 $\eta$ 가 $\Delta u$ 제약을 만족하는지 확인한다. 아니면 $\Delta u$ 를 $\Delta u$ 제약에 맞게 고친다. 고친 $\Delta u$ 로 $u$ 제약을 확인하고, 아니면 $u$ 를 $u$ 제약에 맞게 고친다.

덜 중요한 것부터 처리하고 중요한 것으로 덮어쓰는 순서다. 마지막에 처리된 제약이 최종적으로 지켜지므로, 가장 중요한 제약을 마지막에 둔다. [[13_MATLAB_DC_Motor|motor2.m]] 의 루프가 정확히 이 순서로 짜여 있다.

## 2장 QP 접근과의 비교

| 항목 | QP (Hildreth) | SISO 닫힌형 |
|---|---|---|
| 적용 범위 | MIMO, 임의 개수의 제약 | SISO, 첫 샘플 제약 |
| 활성 제약 판별 | 반복으로 자동 | 무제약 해를 한계와 비교 |
| 계산 | 반복 루프, 행렬 연산 | 비교문 몇 개, 나눗셈 한 번 |
| 충돌 처리 | 타협된 준최적해 | 순위에 따라 하나만 만족 |
| 결과 | 같음 (첫 샘플 제약이면 동일해) | 같음 |

교재는 Tutorial 3.9 뒤에서 "이 닫힌형 해가 QP를 썼을 때와 동일한 결과를 낸다"고 확인한다. 2장에서 "수정된 포화가 SISO 진폭 제약에서는 정확히 최적해"라고 했던 관찰의 일반화다.

## 연결

- 2장의 닫힌형 해: [[09_Closed_Form_Active_Set|2장 - 활성집합을 알 때의 닫힌형 해]]
- 선형 종속과 충돌: [[10_Constraints_on_u|Example 3.8]], [[04_QP_Equality_Constraints|2장 - 종속 제약]]
- 출력 제약을 조심해야 하는 이유: [[02_Constraint_Formulation|2장 - 운전 제약의 종류]], [[13_Output_Constraints|2장 - 출력 제약과 제약 충돌]]
- 구현: [[13_MATLAB_DC_Motor|DC 모터 튜토리얼 motor1.m / motor2.m]]

## → 검증

- $\Delta u$ 제약이 활성일 때 $\eta$ 를 구하지 않고 $\Delta u(k_i)=\Delta u^{min}$ 을 바로 쓰는 것이 왜 정확한지 2장 닫힌형 해로 설명할 수 있는가?
- 세 종류 제약의 $M_{act}$ 가 모두 $L(0)^T$ 의 배수인 이유를 말할 수 있는가?
- 출력 제약이 제어 제약보다 순위가 낮은 두 가지 이유를 말할 수 있는가?
- 구현 순서에서 왜 가장 중요한 제약을 마지막에 처리하는지 설명할 수 있는가?
