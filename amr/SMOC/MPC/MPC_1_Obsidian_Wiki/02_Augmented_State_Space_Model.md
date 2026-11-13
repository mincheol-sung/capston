---
title: "02. 내장 적분기 상태공간 모델"
aliases:
  - "Augmented State Space Model"
  - "Embedded Integrator Model"
tags:
  - mpc
  - state-space
  - embedded-integrator
source_pages: "PDF p.31-34 / book p.4-7"
---

# 내장 적분기 상태공간 모델

MPC 설계 모델은 plant의 state-space model에서 출발한다. 1장의 핵심은 입력 $u(k)$ 자체가 아니라 입력 변화량 $\Delta u(k)$를 최적화하도록 plant 모델을 확장하는 것이다.

## 원래 SISO plant 모델

$$
x_m(k+1)=A_mx_m(k)+B_mu(k)
$$

$$
y(k)=C_mx_m(k)
$$

여기서 $u$는 조작변수, $y$는 출력, $x_m$은 plant 상태다.

## 증분 변수

$$
\Delta x_m(k+1)=x_m(k+1)-x_m(k)
$$

$$
\Delta x_m(k)=x_m(k)-x_m(k-1)
$$

$$
\Delta u(k)=u(k)-u(k-1)
$$

원래 plant 방정식의 차분을 취하면 다음과 같다.

$$
\Delta x_m(k+1)=A_m\Delta x_m(k)+B_m\Delta u(k)
$$

## 확장 상태벡터

출력까지 상태에 포함해 새 상태를 정의한다.

$$
x(k)=
\begin{bmatrix}
\Delta x_m(k)^T & y(k)^T
\end{bmatrix}^T
$$

SISO에서는 다음 augmented model을 얻는다.

$$
\begin{bmatrix}
\Delta x_m(k+1) \\
y(k+1)
\end{bmatrix}
=
\begin{bmatrix}
A_m & o_m^T \\
C_mA_m & 1
\end{bmatrix}
\begin{bmatrix}
\Delta x_m(k) \\
y(k)
\end{bmatrix}
+
\begin{bmatrix}
B_m \\
C_mB_m
\end{bmatrix}
\Delta u(k)
$$

$$
y(k)=
\begin{bmatrix}
o_m & 1
\end{bmatrix}
x(k)
$$

이를 간단히

$$
x(k+1)=Ax(k)+B\Delta u(k), \quad y(k)=Cx(k)
$$

라고 쓴다.

## 왜 적분기가 내장되는가

실제 입력은

$$
u(k)=u(k-1)+\Delta u(k)
$$

로 복원된다. 즉, 제어기 내부에 $1/(1-q^{-1})$ 형태의 이산시간 적분 동작이 들어간다.

이 구조의 장점은 다음과 같다.

- 상수 reference tracking에서 정상상태 오차를 없애기 쉽다.
- 상수 입력 외란을 rejection하는 데 유리하다.
- 구현 시 정상상태 입력 $u_{ss}$나 정상상태 plant state를 별도로 알 필요가 줄어든다.
- MIMO plant에서 steady-state 정보를 모두 계산해야 하는 부담을 낮춘다.

주의할 점도 있다.

- 출력 수만큼 $\lambda=1$인 적분기 모드가 추가된다.
- 큰 prediction horizon $N_p$에서 수치 민감도가 생길 수 있다.
- 작은 $N_p$, $N_c$만으로 안정성이 자동 보장되지는 않는다.

## Example 1.1 핵심

주어진 plant가

$$
A_m=
\begin{bmatrix}
1 & 1 \\
0 & 1
\end{bmatrix},
\quad
B_m=
\begin{bmatrix}
0.5 \\
1
\end{bmatrix},
\quad
C_m=
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$

일 때 augmented model은

$$
A=
\begin{bmatrix}
1 & 1 & 0 \\
0 & 1 & 0 \\
1 & 1 & 1
\end{bmatrix},
\quad
B=
\begin{bmatrix}
0.5 \\
1 \\
0.5
\end{bmatrix},
\quad
C=
\begin{bmatrix}
0 & 0 & 1
\end{bmatrix}
$$

가 된다. 이 $A$의 eigenvalue는 모두 $\lambda=1$이다. 두 개는 원 plant의 double integrator에서 오고, 하나는 augmented model의 embedded integrator에서 온다.

## 연결

- 다음 단계: [[03_One_Window_Optimization|한 최적화 창에서의 예측제어]]
- 구현 흐름: [[09_MATLAB_Workflows|MATLAB 구현 흐름]]
- MIMO 확장: [[05_MIMO_MPC|MIMO 예측제어]]

