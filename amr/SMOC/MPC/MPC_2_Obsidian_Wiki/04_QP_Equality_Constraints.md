---
title: "04. 등식제약 QP와 Lagrange 승수"
aliases:
  - "Quadratic Programming"
  - "Lagrange Multipliers"
  - "Equality Constraints"
tags:
  - mpc
  - qp
  - optimization
  - lagrange
source_pages: "PDF p.11-16 / book p.53-58"
---

# 등식제약 QP와 Lagrange 승수

표준 QP는 문헌 관례를 따라 결정변수를 $x$ 로 쓴다. MPC와의 대응은 다음과 같다.

| QP 기호 | MPC 대응 |
|---|---|
| $x$ | $\Delta U$ |
| $E$ | $2(\Phi^T\Phi+\bar R)$ |
| $F$ | $-2\Phi^T(R_s-Fx(k_i))$ |
| $M$, $\gamma$ | [[03_Constrained_MPC_Problem\|제약 데이터 행렬]] |

표준형은

$$
J=\frac12x^TEx+x^TF
$$

$$
Mx\le\gamma
$$

이고, $E$ 는 일반성을 잃지 않고 대칭이며 양의 정부호로 둔다.

교재가 직접 구현을 권하는 이유가 있다. MPC의 수치해가 응용의 걸림돌로 여겨지곤 하는데, 핵심만 이해하면 필요한 프로그램을 직접 만들 수 있다. 직접 만들면 문제가 생겼을 때 코드에 접근할 수 있고, 실시간 응용을 위한 안전 jacket 소프트웨어도 작성할 수 있다. 산업 현장에서는 이 점이 매우 중요하다.

## 2.4.1 기하학적 직관

- 각 선형 등식 제약은 **초평면**을 정의한다.
- 양의 정부호 이차함수의 등고면은 **초타원면**이다.
- 제약 최소점은 가능집합 경계와 최소화 초타원면의 **접점**에 놓인다.

## Example 2.4 - 손으로 푸는 가장 쉬운 예

$$
\min J=(x_1-2)^2+(x_2-2)^2 \quad\text{subject to}\quad x_1+x_2=1
$$

무제약 전역 최소는 $x_1=2$, $x_2=2$ 다. 제약에서 $x_2=1-x_1$ 이므로 대입하면

$$
J=(x_1-2)^2+(1-x_1-2)^2=2x_1^2-2x_1+5
$$

미분해서 $\partial J/\partial x_1=4x_1-2=0$ 이므로 $x_1=0.5$, 따라서 $x_2=0.5$ 다.

$(x_1,x_2)$ 평면에서 등식제약은 직선이고 목적함수 등고선은 원이다. 제약 최소점은 직선과 원이 접하는 점이다.

## Lagrange 승수법

일반 해법으로 넘어가자. Lagrange 식은

$$
J=\frac12x^TEx+x^TF+\lambda^T(Mx-\gamma)
$$

제약 $Mx=\gamma$ 가 만족되면 $\lambda^T(Mx-\gamma)=0$ 이므로 원래 목적함수와 값이 같다. 이제 이 식을 $x$ (차원 $n$)와 $\lambda$ (차원 $m$)에 대한 $n+m$ 변수 함수로 보고 편미분을 0으로 놓는다.

$$
\frac{\partial J}{\partial x}=Ex+F+M^T\lambda=0
$$

$$
\frac{\partial J}{\partial\lambda}=Mx-\gamma=0
$$

$\lambda$ 의 원소를 **Lagrange 승수**라고 부른다.

### 닫힌형 해 유도

교재는 결과만 주는데, 전개는 다음과 같다. 첫 식에서

$$
x=-E^{-1}(M^T\lambda+F)
$$

이것을 $Mx=\gamma$ 에 넣으면

$$
-ME^{-1}M^T\lambda-ME^{-1}F=\gamma
$$

$$
ME^{-1}M^T\lambda=-(\gamma+ME^{-1}F)
$$

정리하면

$$
\lambda=-(ME^{-1}M^T)^{-1}\left(\gamma+ME^{-1}F\right)
$$

$$
x=-E^{-1}(M^T\lambda+F)
$$

### 의미 있는 분해

$x$ 를 두 항으로 나눠 쓸 수 있다.

$$
x=-E^{-1}F-E^{-1}M^T\lambda=x^0-E^{-1}M^T\lambda
$$

첫 항 $x^0=-E^{-1}F$ 는 제약 없이 $J$ 를 최소화하는 전역 최적해이고, 둘째 항은 등식제약 때문에 생기는 보정항이다.

**제약 최적해 = 무제약 최적해 + 제약에 의한 보정항.** 이 그림은 [[07_Hildreth_QP|Hildreth]]의 Example 2.10, 2.11에서도 똑같이 반복된다. 2장 전체를 관통하는 구조다.

## Example 2.5 - 등식제약 2개, 변수 3개

$$
\min J=\frac12x^TEx+x^TF
$$

$$
x_1+x_2+x_3=1
$$

$$
3x_1-2x_2-3x_3=1
$$

$$
E=
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix},
\quad
F=
\begin{bmatrix}
-2 \\
-3 \\
-1
\end{bmatrix}
$$

### 1단계 - 무제약 해

$$
x^0=-E^{-1}F=
\begin{bmatrix}
2 & 3 & 1
\end{bmatrix}^T
$$

### 2단계 - 제약을 행렬로

$$
M=
\begin{bmatrix}
1 & 1 & 1 \\
3 & -2 & -3
\end{bmatrix},
\quad
\gamma=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
$$

### 3단계 - 필요한 양

$$
ME^{-1}M^T=
\begin{bmatrix}
3 & -2 \\
-2 & 22
\end{bmatrix},
\quad
ME^{-1}F=
\begin{bmatrix}
-6 \\
3
\end{bmatrix}
$$

$E=I$ 이므로 $ME^{-1}M^T=MM^T$ 다. 성분을 확인하면 $(1,1)$ 은 $1+1+1=3$, $(1,2)$ 는 $3-2-3=-2$, $(2,2)$ 는 $9+4+9=22$ 다. $MF$ 는 1행이 $-2-3-1=-6$, 2행이 $-6+6+3=3$ 이다.

### 4단계 - 가역성 확인

$$
\det(ME^{-1}M^T)=3\cdot 22-(-2)^2=62\ne 0
$$

### 5단계 - 승수와 해

$$
\lambda=-(ME^{-1}M^T)^{-1}\left(\gamma+ME^{-1}F\right)=
\begin{bmatrix}
1.6452 \\
-0.0323
\end{bmatrix}
$$

$$
\begin{bmatrix}
x_1 \\
x_2 \\
x_3
\end{bmatrix}
=x^0-E^{-1}M^T\lambda
=
\begin{bmatrix}
2 \\
3 \\
1
\end{bmatrix}
-
\begin{bmatrix}
1 & 3 \\
1 & -2 \\
1 & -3
\end{bmatrix}
\begin{bmatrix}
1.6452 \\
-0.0323
\end{bmatrix}
=
\begin{bmatrix}
0.4516 \\
1.2903 \\
-0.7419
\end{bmatrix}
$$

검산하면 $0.4516+1.2903-0.7419=1.0000$ 이고 $3(0.4516)-2(1.2903)-3(-0.7419)=1.0000$ 이다.

여기서 미리 봐 둘 것이 있다. $\lambda_2=-0.0323<0$ 이다. 등식제약에서는 아무 문제가 없지만, 이것이 부등식 제약이었다면 "이 제약은 사실 활성이 아니다"라는 신호다. [[05_QP_Inequality_KKT_ActiveSet|Example 2.9]]에서 이 사실을 그대로 사용한다.

## Example 2.6 - 제약이 종속이면 실행 불가능

$$
E=
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix},
\quad
F=
\begin{bmatrix}
-2 \\
-2
\end{bmatrix}
$$

$$
x_1+x_2=1
$$

$$
2x_1+2x_2=6
$$

두 제약은 $(x_1,x_2)$ 평면에서 평행한 두 직선, 즉 $x_1+x_2=1$ 과 $x_1+x_2=3$ 이다. 교점이 없으므로 두 식을 동시에 만족하는 해가 존재하지 않는다.

Lagrange 승수 쪽에서도 같은 결론이 나온다.

$$
M=
\begin{bmatrix}
1 & 1 \\
2 & 2
\end{bmatrix},
\quad
ME^{-1}M^T=
\begin{bmatrix}
2 & 4 \\
4 & 8
\end{bmatrix},
\quad
\det=0
$$

역행렬이 없으므로 닫힌형 해 공식을 쓸 수 없다.

**교훈 1. 제약 최적해를 구하려면 선형 등식 제약들이 선형 독립이어야 한다.**

## Example 2.7 - 제약 개수도 문제가 된다

Example 2.5와 같은 $E$, $F$ 에 제약 하나를 더 추가한다.

$$
x_1+x_2+x_3=1
$$

$$
3x_1-2x_2-3x_3=1
$$

$$
x_1-3x_2+2x_3=1
$$

추가된 제약은 앞의 두 개와 독립이다. 그러면 제약 개수 3과 결정변수 개수 3이 같아지므로 가능해가 유일하게 결정된다.

$$
x=M^{-1}\gamma=
\begin{bmatrix}
0.6552 & 0.0690 & 0.2759
\end{bmatrix}^T
$$

가능해가 하나뿐이므로 그것이 곧 제약 최적해다. 목적함수 $J$ 는 아무 역할도 하지 못한다. 최적화를 진행할 이유가 없다.

**교훈 2.** $n$ 을 결정변수 개수, $m_{eq}$ 를 등식제약 개수라 하면

| 조건 | 결과 |
|---|---|
| $m_{eq}<n$ | 정상적인 제약 최적화. 최적화할 자유도가 있다 |
| $m_{eq}=n$ | 가능해가 유일. 최적화가 무의미하고 그 해가 답이다 |
| $m_{eq}>n$ | 일반적으로 가능해가 없다. 실행 불가능 |

즉 등식제약 개수는 결정변수 개수 이하여야 한다.

## 연결

- 앞 단계: [[03_Constrained_MPC_Problem|제약 있는 MPC 문제의 최종 형태]]
- 다음 단계: [[05_QP_Inequality_KKT_ActiveSet|부등식제약, KKT 조건, Active Set]]
- 수식 색인: [[14_Key_Equations|핵심 수식 정리]]
