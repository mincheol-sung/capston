---
title: "03. 한 최적화 창에서의 예측제어"
aliases:
  - "One Optimization Window"
  - "Prediction Equation"
tags:
  - mpc
  - optimization
  - prediction
source_pages: "PDF p.34-41 / book p.7-14"
---

# 한 최적화 창에서의 예측제어

현재 샘플을 $k_i$라고 하자. MPC는 현재 상태 $x(k_i)$를 알고 있다고 보고, 앞으로 $N_p$ 샘플 동안의 출력을 예측한다. 미래 제어 자유변수는 $N_c$개의 입력 변화량이다.

## 제어 자유변수

$$
\Delta U =
\begin{bmatrix}
\Delta u(k_i) &
\Delta u(k_i+1) &
\cdots &
\Delta u(k_i+N_c-1)
\end{bmatrix}^T
$$

$N_c$는 [[Control Horizon]]이고, 보통 $N_c \leq N_p$로 둔다.

## 미래 상태와 출력 예측

상태공간 모델

$$
x(k+1)=Ax(k)+B\Delta u(k), \quad y(k)=Cx(k)
$$

에서 미래 출력은 순차적으로 다음처럼 계산된다.

$$
y(k_i+1|k_i)=CAx(k_i)+CB\Delta u(k_i)
$$

$$
y(k_i+2|k_i)=CA^2x(k_i)+CAB\Delta u(k_i)+CB\Delta u(k_i+1)
$$

일반적으로

$$
y(k_i+N_p|k_i)
=CA^{N_p}x(k_i)
+CA^{N_p-1}B\Delta u(k_i)
+\cdots
+CA^{N_p-N_c}B\Delta u(k_i+N_c-1)
$$

이다.

## Compact prediction form

예측 출력 벡터를

$$
Y=
\begin{bmatrix}
y(k_i+1|k_i) &
y(k_i+2|k_i) &
\cdots &
y(k_i+N_p|k_i)
\end{bmatrix}^T
$$

라고 두면,

$$
Y=Fx(k_i)+\Phi\Delta U
$$

가 된다.

$$
F=
\begin{bmatrix}
CA \\
CA^2 \\
CA^3 \\
\vdots \\
CA^{N_p}
\end{bmatrix}
$$

$$
\Phi=
\begin{bmatrix}
CB & 0 & 0 & \cdots & 0 \\
CAB & CB & 0 & \cdots & 0 \\
CA^2B & CAB & CB & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
CA^{N_p-1}B & CA^{N_p-2}B & CA^{N_p-3}B & \cdots & CA^{N_p-N_c}B
\end{bmatrix}
$$

$\Phi$는 Toeplitz-like lower triangular 구조를 가진다.

## 비용함수

set-point가 optimization window 동안 일정하다고 가정하면,

$$
R_s=\bar R_s r(k_i), \quad
\bar R_s=
\begin{bmatrix}
1 & 1 & \cdots & 1
\end{bmatrix}^T
$$

이다. 비용함수는

$$
J=(R_s-Y)^T(R_s-Y)+\Delta U^T\bar R\Delta U
$$

로 둔다.

- 첫 항: 예측 출력이 reference에 가까워지도록 만든다.
- 둘째 항: 입력 변화량 $\Delta U$가 너무 커지지 않도록 벌점을 준다.
- $\bar R=r_w I$로 두며 $r_w$는 tuning parameter다.

## 최적해

위 식에 $Y=Fx(k_i)+\Phi\Delta U$를 대입하고 $J$를 $\Delta U$에 대해 최소화하면,

$$
\Delta U=(\Phi^T\Phi+\bar R)^{-1}\Phi^T(R_s-Fx(k_i))
$$

를 얻는다.

여기서

$$
H=\Phi^T\Phi+\bar R
$$

는 optimization에서 Hessian 역할을 한다. $H^{-1}$이 존재하고 $H$가 양의 정부호에 가깝게 잘 조건화되어야 수치적으로 안정적이다.

## $r_w$의 의미

| $r_w$ | 해석 | 응답 특성 |
|---|---|---|
| 작음 또는 0 | 입력 변화량에 벌점을 거의 주지 않음 | 빠르지만 공격적 |
| 큼 | 입력 변화량을 강하게 억제 | 느리지만 부드럽고 신중 |

Example 1.2에서는 $a=0.8$, $b=0.1$, $N_p=10$, $N_c=4$, $x(k_i)=[0.1,0.2]^T$, $r(k_i)=1$일 때 다음 차이를 보인다.

- $r_w=0$: $\Delta U=[7.2,-6.4,0,0]^T$. 출력이 빠르게 set-point에 도달하지만 제어 변화가 크다.
- $r_w=10$: $\Delta U=[0.1269,0.1034,0.0829,0.065]^T$. 입력 변화가 작고 분산되지만 출력 도달이 느려진다.

## 연결

- 다음 단계: [[04_Receding_Horizon_Control|Receding Horizon Control]]
- 수식 모음: [[08_Key_Equations|핵심 수식 정리]]
- MATLAB 행렬 생성: [[09_MATLAB_Workflows|MATLAB 구현 흐름]]

