---
title: "14. 핵심 수식 정리"
aliases:
  - "MPC 2장 핵심 수식"
  - "Constrained MPC Equations"
tags:
  - mpc
  - equations
  - reference
  - constraints
source_pages: "PDF p.5-27, p.39-40 / book p.47-69, p.81-82"
---

# 핵심 수식 정리

2장 계산을 빠르게 다시 펼칠 때 쓰는 수식 색인이다. 1장 수식은 [[08_Key_Equations|1장 핵심 수식 정리]]에 있다.

## 1. 운전 제약의 기본형

$$
\Delta u^{min}\le\Delta u(k)\le\Delta u^{max}
$$

$$
u^{min}\le u(k)\le u^{max}
$$

$$
y^{min}\le y(k)\le y^{max}
$$

소프트 출력 제약:

$$
y^{min}-s_v\le y(k)\le y^{max}+s_v
$$

증분변수 변환:

$$
u^{min}=u^{min}_{\text{물리}}-u_{ss},
\qquad
u^{max}=u^{max}_{\text{물리}}-u_{ss}
$$

## 2. $\Delta U$ 로의 파라미터화

$$
U=C_1u(k_i-1)+C_2\Delta U
$$

$$
C_1=
\begin{bmatrix}
I \\
I \\
\vdots \\
I
\end{bmatrix},
\qquad
C_2=
\begin{bmatrix}
I & 0 & \cdots & 0 \\
I & I & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
I & I & \cdots & I
\end{bmatrix}
$$

$$
Y=Fx(k_i)+\Phi\Delta U
$$

## 3. 양방향 부등식의 표준형 변환

$$
\begin{bmatrix}
-I \\
I
\end{bmatrix}
\Delta U
\le
\begin{bmatrix}
-\Delta U^{min} \\
\Delta U^{max}
\end{bmatrix}
$$

## 4. 제약 있는 MPC 문제

$$
J=(R_s-Fx(k_i))^T(R_s-Fx(k_i))-2\Delta U^T\Phi^T(R_s-Fx(k_i))+\Delta U^T(\Phi^T\Phi+\bar R)\Delta U
$$

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

압축 표기:

$$
M\Delta U\le\gamma
$$

전부 부과했을 때의 제약 개수:

$$
4\times m\times N_c+2\times q\times N_p
$$

## 5. 표준 QP

$$
J=\frac12x^TEx+x^TF
$$

$$
Mx\le\gamma
$$

MPC 대응은 $x\leftrightarrow\Delta U$, $E\leftrightarrow 2(\Phi^T\Phi+\bar R)$, $F\leftrightarrow-2\Phi^T(R_s-Fx(k_i))$ 다.

## 6. 등식제약 해 (Lagrange)

$$
J=\frac12x^TEx+x^TF+\lambda^T(Mx-\gamma)
$$

$$
Ex+F+M^T\lambda=0
$$

$$
Mx-\gamma=0
$$

$$
\lambda=-(ME^{-1}M^T)^{-1}\left(\gamma+ME^{-1}F\right)
$$

$$
x=-E^{-1}(M^T\lambda+F)=x^0-E^{-1}M^T\lambda
$$

$$
x^0=-E^{-1}F
$$

## 7. Kuhn-Tucker 조건

$$
Ex+F+M^T\lambda=0
$$

$$
Mx-\gamma\le 0
$$

$$
\lambda^T(Mx-\gamma)=0
$$

$$
\lambda\ge 0
$$

활성집합 표현:

$$
M_ix-\gamma_i=0,\ \ \lambda_i\ge 0 \qquad i\in S_{act}
$$

$$
M_ix-\gamma_i<0,\ \ \lambda_i=0 \qquad i\notin S_{act}
$$

## 8. 활성집합을 알 때의 닫힌형 해

$$
\lambda_{act}=-(M_{act}E^{-1}M_{act}^T)^{-1}(\gamma_{act}+M_{act}E^{-1}F)
$$

$$
x=-E^{-1}(F+M_{act}^T\lambda_{act})
$$

## 9. Dual 문제

$$
\max_{\lambda\ge 0}\ \min_{x}\left[\frac12x^TEx+x^TF+\lambda^T(Mx-\gamma)\right]
$$

$$
x=-E^{-1}(F+M^T\lambda)
$$

$$
H=ME^{-1}M^T
$$

$$
K=\gamma+ME^{-1}F
$$

$$
\min_{\lambda\ge 0}\left(\frac12\lambda^TH\lambda+\lambda^TK+\frac12\gamma^TE^{-1}\gamma\right)
$$

## 10. Hildreth 갱신식

$$
\lambda_i^{m+1}=\max(0,\ w_i^{m+1})
$$

$$
w_i^{m+1}=-\frac{1}{h_{ii}}\left[k_i+\sum_{j=1}^{i-1}h_{ij}\lambda_j^{m+1}+\sum_{j=i+1}^{n}h_{ij}\lambda_j^{m}\right]
$$

$$
x=-E^{-1}(F+M^T\lambda^*)
$$

## 계산 순서

1. 플랜트 운전 한계를 정의한다. 입력, 입력 증분, 상태, 출력의 min과 max.
2. 정상상태 정보를 고려해서 이 한계들을 증분변수 기준 파라미터로 표현한다.
3. 미래 제어궤적을 파라미터화해서, min과 max 값을 $\Delta u(k_i),\ldots,\Delta u(k_i+N_c-1)$ 을 변수로 하는 부등식으로 만든다.
4. 설계 목표를 "원래 오차함수를 부등식 제약 아래에서 최소화"로 재정의한다.
5. 매 샘플마다 QP를 풀어 결정변수의 최적해를 구한다.
6. 첫 원소만 플랜트에 적용하고 다음 샘플에서 반복한다.

## 관련 문헌

| 접근 | 문헌 |
|---|---|
| Primal 방법 (고전적 주류) | Muske and Rawlings 1993, Ricker 1985, Zafiriou 1991 |
| MPC 전용 내점법 | Rao et al. 1998, Gopal and Biegler 1998, Hansson 2000 |
| 해석적 해 시도 | Bemporad et al. 1999, Seron et al. 2000 |
| 안정 시스템은 미래 제어 제약을 줄여도 성능 저하 미미 | Zheng 1999 |
| 가중최소제곱 반복 (Lawson 알고리즘) | Rossiter and Kouvaritakis 1993 |
| $N_c$ 가 1 또는 2인 SISO GPC의 최적해 | Tsang and Clarke 1988 |

## 연결

- 1장 수식: [[08_Key_Equations|1장 핵심 수식 정리]]
- 문제 정의: [[03_Constrained_MPC_Problem|제약 있는 MPC 문제의 최종 형태]]
- 구현: [[08_MATLAB_QPhild|QPhild.m]]
- 용어: [[MPC2_Glossary|MPC 2장 용어집]]
