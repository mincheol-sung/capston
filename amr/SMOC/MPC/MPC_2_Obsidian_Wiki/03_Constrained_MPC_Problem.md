---
title: "03. 제약 있는 MPC 문제의 최종 형태"
aliases:
  - "Constrained MPC Problem"
  - "M gamma 데이터 행렬"
tags:
  - mpc
  - constraints
  - qp
source_pages: "PDF p.9-10 / book p.51-52"
---

# 제약 있는 MPC 문제의 최종 형태

[[02_Constraint_Formulation|앞 노트]]에서 모든 제약을 $\Delta U$ 에 대한 선형 부등식으로 바꿨다. 이제 그것을 원래 비용함수와 결합하면 2장의 문제 정의가 완성된다.

## 목적함수

1장의 비용함수 $J=(R_s-Y)^T(R_s-Y)+\Delta U^T\bar R\Delta U$ 에 $Y=Fx(k_i)+\Phi\Delta U$ 를 대입하고 전개하면

$$
J=(R_s-Fx(k_i))^T(R_s-Fx(k_i))-2\Delta U^T\Phi^T(R_s-Fx(k_i))+\Delta U^T(\Phi^T\Phi+\bar R)\Delta U
$$

세 항의 역할은 다음과 같다.

- 첫 항은 $\Delta U$ 와 무관한 상수라 최적화에 영향을 주지 않는다.
- 둘째 항이 1차항이고, 표준 QP의 $x^TF$ 에 해당한다.
- 셋째 항이 2차항이고, Hessian은 $2(\Phi^T\Phi+\bar R)$ 다.

**목적함수는 1장과 완전히 동일하다.** 2장에서 바뀌는 것은 이 함수를 최소화할 때 제약이 붙는다는 것뿐이다.

## 제약

$$
\begin{bmatrix}
M_1 \\
M_2 \\
M_3
\end{bmatrix}
\Delta U
\le
\begin{bmatrix}
N_1 \\
N_2 \\
N_3
\end{bmatrix}
$$

데이터 행렬은 다음과 같다.

$$
M_1=
\begin{bmatrix}
-C_2 \\
C_2
\end{bmatrix},
\quad
N_1=
\begin{bmatrix}
-U^{min}+C_1u(k_i-1) \\
U^{max}-C_1u(k_i-1)
\end{bmatrix}
$$

$$
M_2=
\begin{bmatrix}
-I \\
I
\end{bmatrix},
\quad
N_2=
\begin{bmatrix}
-\Delta U^{min} \\
\Delta U^{max}
\end{bmatrix}
$$

$$
M_3=
\begin{bmatrix}
-\Phi \\
\Phi
\end{bmatrix},
\quad
N_3=
\begin{bmatrix}
-Y^{min}+Fx(k_i) \\
Y^{max}-Fx(k_i)
\end{bmatrix}
$$

| 블록 | 대상 | 특징 |
|---|---|---|
| $M_1,N_1$ | 제어 진폭 $u$ | 우변에 $u(k_i-1)$ 이 들어가므로 매 샘플 갱신 |
| $M_2,N_2$ | 제어 증분 $\Delta u$ | 우변이 상수라 갱신 불필요 |
| $M_3,N_3$ | 출력 $y$ | 우변에 $Fx(k_i)$ 가 들어가므로 매 샘플 갱신 |

압축 표기로

$$
M\Delta U\le\gamma
$$

## 문제의 성질

- $\Phi^T\Phi+\bar R$ 는 Hessian이고 양의 정부호로 가정한다.
- $J$ 가 이차식이고 제약이 선형 부등식이므로, 최적 예측제어를 찾는 문제는 **표준 QP 문제**가 된다.
- $M$ 의 행 개수는 제약 개수이고 열 개수는 $\Delta U$ 의 차원이다.

제약을 전부 부과하면 제약 개수는

$$
4\times m\times N_c+2\times q\times N_p
$$

가 된다. 여기서 $m$ 은 입력 개수, $q$ 는 출력 개수다. 교재는 결과만 주는데 내역은 이렇다.

- $u$ 진폭: 상한과 하한 2개 $\times m\times N_c=2mN_c$
- $\Delta u$ 증분: 상한과 하한 2개 $\times m\times N_c=2mN_c$
- 둘을 합쳐 $4mN_c$
- 출력: 상한과 하한 2개 $\times q\times N_p=2qN_p$

보통 제약 개수가 결정변수 차원보다 크다. 등식제약이었다면 이것만으로 실행 불가능이지만, 부등식이라 괜찮다. 이유는 [[05_QP_Inequality_KKT_ActiveSet|부등식제약 노트]]에 있다.

## 자연스럽게 떠오르는 질문

이동구간 제어는 첫 번째 제어 이동만 적용하고 나머지 계산된 미래 제어신호는 버린다. 그렇다면 미래 전체 궤적에 대해 제어신호와 시스템 출력에 제약을 걸 필요가 있는가?

교재는 이 질문을 뒤로 미루고 [[10_Constraints_on_Rate_of_Change|2.5절]]에서 실제로 두 방식을 비교한다. 결론을 미리 말하면, 차이는 매우 작은데 계산량은 $N_c$ 배가 된다. 심지어 미래 전 구간에 제약을 걸면 실행되지도 않을 제약이 활성화되어 첫 제어값을 왜곡시키는 경우도 있다.

## 연결

- 앞 단계: [[02_Constraint_Formulation|운전 제약의 종류와 수식화]]
- 다음 단계: [[04_QP_Equality_Constraints|등식제약 QP와 Lagrange 승수]]
- 수식 색인: [[14_Key_Equations|핵심 수식 정리]]
- 비용함수의 출처: [[03_One_Window_Optimization|1장 한 최적화 창에서의 예측제어]]
