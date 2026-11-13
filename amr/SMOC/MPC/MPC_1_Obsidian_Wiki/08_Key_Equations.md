---
title: "08. 핵심 수식 정리"
aliases:
  - "MPC 1장 핵심 수식"
tags:
  - mpc
  - equations
  - reference
source_pages: "PDF p.31-65 / book p.4-38"
---

# 핵심 수식 정리

이 노트는 1장 계산을 빠르게 다시 펼칠 때 쓰는 수식 색인이다.

## 1. Plant model

$$
x_m(k+1)=A_mx_m(k)+B_mu(k)
$$

$$
y(k)=C_mx_m(k)
$$

## 2. Increment variables

$$
\Delta x_m(k)=x_m(k)-x_m(k-1)
$$

$$
\Delta u(k)=u(k)-u(k-1)
$$

$$
\Delta x_m(k+1)=A_m\Delta x_m(k)+B_m\Delta u(k)
$$

## 3. Augmented model

$$
x(k)=
\begin{bmatrix}
\Delta x_m(k)^T & y(k)^T
\end{bmatrix}^T
$$

SISO:

$$
A=
\begin{bmatrix}
A_m & o_m^T \\
C_mA_m & 1
\end{bmatrix},
\quad
B=
\begin{bmatrix}
B_m \\
C_mB_m
\end{bmatrix},
\quad
C=
\begin{bmatrix}
o_m & 1
\end{bmatrix}
$$

MIMO:

$$
A=
\begin{bmatrix}
A_m & 0 \\
C_mA_m & I_q
\end{bmatrix},
\quad
B=
\begin{bmatrix}
B_m \\
C_mB_m
\end{bmatrix},
\quad
C=
\begin{bmatrix}
0_{q\times n_1} & I_q
\end{bmatrix}
$$

## 4. Prediction equation

$$
Y=Fx(k_i)+\Phi\Delta U
$$

$$
F=
\begin{bmatrix}
CA \\
CA^2 \\
\vdots \\
CA^{N_p}
\end{bmatrix}
$$

$$
\Phi=
\begin{bmatrix}
CB & 0 & \cdots & 0 \\
CAB & CB & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
CA^{N_p-1}B & CA^{N_p-2}B & \cdots & CA^{N_p-N_c}B
\end{bmatrix}
$$

## 5. Cost function

$$
R_s=\bar R_s r(k_i)
$$

$$
J=(R_s-Y)^T(R_s-Y)+\Delta U^T\bar R\Delta U
$$

with

$$
\bar R=r_wI
$$

in the simplest SISO tuning case.

## 6. Optimal control sequence

$$
\Delta U=(\Phi^T\Phi+\bar R)^{-1}\Phi^T(R_s-Fx(k_i))
$$

또는 reference vector를 분리해서

$$
\Delta U
=
(\Phi^T\Phi+\bar R)^{-1}
(\Phi^T\bar R_s r(k_i)-\Phi^TFx(k_i))
$$

로 쓴다.

## 7. Receding horizon control law

$$
\Delta u(k_i)=K_yr(k_i)-K_{mpc}x(k_i)
$$

$$
K_y=
\begin{bmatrix}
1 & 0 & \cdots & 0
\end{bmatrix}
(\Phi^T\Phi+\bar R)^{-1}\Phi^T\bar R_s
$$

$$
K_{mpc}=
\begin{bmatrix}
1 & 0 & \cdots & 0
\end{bmatrix}
(\Phi^T\Phi+\bar R)^{-1}\Phi^TF
$$

MIMO에서는 첫 $m$개 원소 선택 행렬을 쓴다.

$$
\begin{bmatrix}
I_m & 0_m & \cdots & 0_m
\end{bmatrix}
$$

## 8. Closed-loop matrix

$$
x(k+1)=(A-BK_{mpc})x(k)+BK_yr(k)
$$

$$
\det[\lambda I-(A-BK_{mpc})]=0
$$

## 9. Observer

$$
\hat x_m(k+1)
=A_m\hat x_m(k)+B_mu(k)+K_{ob}(y(k)-C_m\hat x_m(k))
$$

$$
\tilde x_m(k+1)=(A_m-K_{ob}C_m)\tilde x_m(k)
$$

Observability matrix:

$$
L_o=
\begin{bmatrix}
C_m \\
C_mA_m \\
C_mA_m^2 \\
\vdots \\
C_mA_m^{n-1}
\end{bmatrix}
$$

## 10. State-estimate MPC

$$
\hat x(k+1)=A\hat x(k)+B\Delta u(k)+K_{ob}(y(k)-C\hat x(k))
$$

$$
\Delta U=(\Phi^T\Phi+\bar R)^{-1}\Phi^T(R_s-F\hat x(k_i))
$$

$$
\Delta u(k_i)=K_yr(k_i)-K_{mpc}\hat x(k_i)
$$

Combined closed-loop characteristic equations:

$$
\det(\lambda I-(A-K_{ob}C))=0
$$

$$
\det(\lambda I-(A-BK_{mpc}))=0
$$

## 계산 순서

1. plant $A_m,B_m,C_m$을 준비한다.
2. augmented $A,B,C$를 만든다.
3. $N_p,N_c,r_w$를 고른다.
4. $F,\Phi,\bar R_s$를 만든다.
5. $K_y,K_{mpc}$를 계산한다.
6. 폐루프 eigenvalue를 확인한다.
7. 상태가 모두 측정되지 않으면 $K_{ob}$를 설계한다.
8. simulation loop에서 $\Delta u$, $u$, plant state, observer state를 갱신한다.

