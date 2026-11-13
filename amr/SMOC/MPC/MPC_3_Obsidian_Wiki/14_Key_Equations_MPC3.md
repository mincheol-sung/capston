---
title: "14. 핵심 수식 정리"
aliases:
  - "MPC 3장 핵심 수식"
  - "Laguerre MPC Equations"
tags:
  - mpc
  - equations
  - reference
  - laguerre
source_pages: "PDF p.2-25, p.34-38, p.47-50 / book p.86-109, p.118-122, p.131-134"
---

# 핵심 수식 정리

3장 계산을 빠르게 다시 펼칠 때 쓰는 수식 색인이다. 1장과 2장 수식은 [[08_Key_Equations|1장 핵심 수식]]과 [[14_Key_Equations|2장 핵심 수식]]에 있다.

## 1. 이산시간 Laguerre 함수

z-영역:

$$
\Gamma_N(z)=\frac{\sqrt{1-a^2}}{1-az^{-1}}\left(\frac{z^{-1}-a}{1-az^{-1}}\right)^{N-1},
\qquad
\Gamma_k(z)=\Gamma_{k-1}(z)\frac{z^{-1}-a}{1-az^{-1}}
$$

상태공간 (시간 영역):

$$
L(k+1)=A_lL(k),
\qquad
L(0)^T=\sqrt{\beta}
\begin{bmatrix}
1 & -a & a^2 & \cdots & (-1)^{N-1}a^{N-1}
\end{bmatrix},
\qquad
\beta=1-a^2
$$

$$
A_l=
\begin{bmatrix}
a & 0 & 0 & \cdots \\
\beta & a & 0 & \cdots \\
-a\beta & \beta & a & \cdots \\
a^2\beta & -a\beta & \beta & \ddots \\
\vdots & & & \ddots
\end{bmatrix}
\quad\text{(Toeplitz 하삼각, 첫 열 } [a,\ \beta,\ -a\beta,\ a^2\beta,\ \ldots]^T)
$$

직교정규성:

$$
\sum_{k=0}^{\infty}l_i(k)l_j(k)=\delta_{ij}
$$

$a=0$ 이면 $A_l$ 은 시프트 행렬, $l_k(i)=\delta(i-k+1)$, 즉 펄스 기저.

## 2. 임펄스 응답의 Laguerre 전개

$$
H(k)=\sum_{i=1}^{N}c_il_i(k),
\qquad
c_i=\sum_{k=0}^{\infty}H(k)l_i(k)
$$

## 3. 제어궤적의 파라미터화

$$
\Delta u(k_i+k)=\sum_{j=1}^{N}c_j(k_i)l_j(k)=L(k)^T\eta,
\qquad
\eta=
\begin{bmatrix}
c_1 & c_2 & \cdots & c_N
\end{bmatrix}^T
$$

MIMO: 입력 $i$ 마다 $(a_i,N_i)$, $\Delta u_i(k)=L_i(k)^T\eta_i$, $\eta^T=[\eta_1^T\ \cdots\ \eta_m^T]$.

## 4. 예측식

$$
x(k_i+m\mid k_i)=A^mx(k_i)+\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T\eta=A^mx(k_i)+\phi(m)^T\eta
$$

$$
y(k_i+m\mid k_i)=CA^mx(k_i)+\sum_{i=0}^{m-1}CA^{m-i-1}BL(i)^T\eta
$$

$$
\phi(m)^T=\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T
\qquad(n\times N)
$$

MIMO: $\phi(m)^T=\sum_{j=0}^{m-1}A^{m-j-1}[B_1L_1(j)^T\ \ B_2L_2(j)^T\ \cdots\ B_mL_m(j)^T]$.

## 5. 비용함수

출력오차 형태:

$$
J=\sum_{m=1}^{N_p}\big(r(k_i)-y(k_i+m\mid k_i)\big)^2+\eta^TR_L\eta
$$

DLQR 형태 ($Q=C^TC$, 상태 마지막 성분을 $y-r$ 로 둠):

$$
J=\sum_{m=1}^{N_p}x(k_i+m\mid k_i)^TQx(k_i+m\mid k_i)+\eta^TR_L\eta
$$

전개:

$$
J=\eta^T\Omega\eta+2\eta^T\Psi x(k_i)+\sum_{m=1}^{N_p}x(k_i)^T(A^T)^mQA^mx(k_i)
$$

$$
\Omega=\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L,
\qquad
\Psi=\sum_{m=1}^{N_p}\phi(m)QA^m
$$

직교정규성에 의한 치환 (충분히 큰 $N_p$):

$$
\Delta U^T\bar R\Delta U=r_w\sum_m\Delta u(k_i+m)^2=r_w\,\eta^T\Big(\sum_mL(m)L(m)^T\Big)\eta\approx\eta^TR_L\eta
$$

## 6. 무제약 최적해와 최소 비용

$$
\eta=-\Omega^{-1}\Psi x(k_i)
$$

$$
J_{min}=x(k_i)^TP_{dmpc}x(k_i),
\qquad
P_{dmpc}=\sum_{m=1}^{N_p}(A^T)^mQA^m-\Psi^T\Omega^{-1}\Psi
$$

## 7. 합성곱 합의 재귀식

$$
S_c(1)=BL(0)^T,
\qquad
S_c(m)=AS_c(m-1)+S_c(1)(A_l^{m-1})^T,
\quad m=2,\ldots,N_p
$$

$S_c(m)=\phi(m)^T$. MIMO에서는 입력 블록별로 따로 돌린다.

## 8. 이동구간 제어와 피드백 이득

$$
\Delta u(k_i)=L(0)^T\eta,
\qquad
\Delta u(k)=-K_{mpc}x(k),
\qquad
K_{mpc}=L(0)^T\Omega^{-1}\Psi
$$

MIMO:

$$
K_{mpc}=
\begin{bmatrix}
L_1(0)^T & o_2^T & \cdots & o_m^T \\
o_1^T & L_2(0)^T & \cdots & o_m^T \\
\vdots & \vdots & \ddots & \vdots \\
o_1^T & o_2^T & \cdots & L_m(0)^T
\end{bmatrix}
\Omega^{-1}\Psi,
\qquad
x(k+1)=(A-BK_{mpc})x(k)
$$

피드백 상태: $x(k_i)=[\Delta x_m(k_i)^T\ \ y(k_i)-r(k_i)]^T$.

## 9. DLQR (수렴의 목표)

$$
A^T\Big(P_\infty-P_\infty B(R+B^TP_\infty B)^{-1}B^TP_\infty\Big)A+Q-P_\infty=0
$$

$$
K=(R+B^TP_\infty B)^{-1}B^TP_\infty A,
\qquad
\Delta u(k)=-Kx(k)
$$

SISO, $Q=C^TC$, $R=r_w$ 일 때 폐루프 극점은 다음의 단위원 안쪽 근:

$$
1+\frac{1}{r_w}\frac{G_m(z)G_m(z^{-1})}{(z-1)(z^{-1}-1)}=0
$$

MIMO ($R=r_wI$):

$$
\det\Big(I+\frac{1}{r_w}\frac{G_m(z)G_m(z^{-1})^T}{(z-1)(z^{-1}-1)}\Big)=0
$$

## 10. 제약의 $\eta$ 표현

증분 (시점 $m$):

$$
\Delta u^{min}\le
\mathrm{blkdiag}\big(L_1(m)^T,\ldots,L_m(m)^T\big)\eta\le\Delta u^{max}
$$

진폭 (시점 $k$):

$$
u^{min}\le
\mathrm{blkdiag}\Big(\sum_{i=0}^{k-1}L_1(i)^T,\ldots,\sum_{i=0}^{k-1}L_m(i)^T\Big)\eta+u(k_i-1)\le u^{max}
$$

표준형:

$$
\begin{bmatrix}
M \\ -M
\end{bmatrix}
\eta\le
\begin{bmatrix}
\Delta U^{max} \\ -\Delta U^{min}
\end{bmatrix},
\qquad
\begin{bmatrix}
M \\ -M
\end{bmatrix}
\eta\le
\begin{bmatrix}
U^{max}-\bar u(k_i-1) \\ -U^{min}+\bar u(k_i-1)
\end{bmatrix}
$$

QP: $\min\ \eta^T\Omega\eta+2\eta^T\Psi x(k_i)$ s.t. $M\eta\le\gamma$. `QPhild(Omega,Psi*x,M,gamma)`.

## 11. 안정성 (Theorem 3.1)

Lyapunov 함수 $V(x(k),k)=J_{min}$, 종단 제약 $x(k+N_p\mid k)=0$ 하에서

$$
V(x(k+1),k+1)-V(x(k),k)\le-x(k+1)^TQx(k+1)-\Delta u(k)^TR\Delta u(k)<0
$$

종단 제약 없이는 큰 $N_p$, 큰 $N$ 에서 $x(k+N_p\mid k)^TQx(k+N_p\mid k)\approx 0$ (DLQR 수렴) 을 근거로 삼는다.

## 12. SISO 닫힌형 해 (첫 샘플 제약)

$\Delta u$ 제약: $L(0)^T\eta$ 가 한계를 넘으면 $\Delta u(k_i)=\Delta u^{min}$ 또는 $\Delta u^{max}$.

$u$ 제약: $u(k_i-1)+L(0)^T\eta$ 가 한계를 넘으면 $u(k_i)=u^{min/max}$, $\Delta u(k_i)=u^{min/max}-u(k_i-1)$.

$y$ 제약 ($y(k_i+1\mid k_i)=CAx(k_i)+CBL(0)^T\eta$):

$$
\lambda_{act}=-(M_{act}\Omega^{-1}M_{act}^T)^{-1}\big(\gamma_{act}+M_{act}\Omega^{-1}\Psi x(k_i)\big),
\qquad
\eta=-\Omega^{-1}\big(\Psi x(k_i)+M_{act}^T\lambda_{act}\big)
$$

하한 위반: $M_{act}=-CBL(0)^T$, $\gamma_{act}=-y^{min}+CAx(k_i)$. 상한 위반: $M_{act}=CBL(0)^T$, $\gamma_{act}=y^{max}-CAx(k_i)$.

세 종류의 $M_{act}$ 는 모두 $L(0)^T$ 의 배수이므로 한 번에 하나만 활성일 수 있다. 순위: $u$ > $\Delta u$ > $y$. 처리 순서는 $y$ → $\Delta u$ → $u$.

## 13. 튜닝 절차 (3.9절 요약)

1. $Q=C^TC$ 로 두고 $R$ 을 대각행렬로 고른다. 작은 원소가 빠른 응답에 대응한다.
2. $(Q,R)$ 이 정해지면 목표 궤적(DLQR)이 고정된다. $a$ 는 폐루프 지배 고유값의 실수부 크기 정도로 잡고, $N$ 은 궤적이 더 이상 변하지 않을 때까지 늘린다.
3. 어려운 응용에서는 입력별 $(a_i,N_i)$ 로 미세조정한다. 큰 $a$ 와 작은 $N$ 은 느리게 감쇠하는 제어신호에 대응한다. $N$ 을 늘리면 $(Q,R)$ 이 정한 궤적으로 수렴한다.
4. $N_p$ 는 Laguerre 함수가 충분히 감쇠하는 범위를 포함하도록 크게 잡는다. 너무 크면 수치 악조건 (4장).

## 코드 ↔ 수식 대응

| 코드 | 수식 |
|---|---|
| `[Al,L0]=lagd(a,N)` | $A_l$, $L(0)$ |
| `[Omega,Psi]=dmpc(A,B,a,N,Np,Q,R)` | $\Omega$, $\Psi$ |
| `eta=-(Omega\Psi)*Xf` | $\eta=-\Omega^{-1}\Psi x(k_i)$ |
| `deltau=Lzerot*eta` / `L0'*eta` | $\Delta u(k_i)=L(0)^T\eta$ |
| `K=L_m*(Omega\Psi)` | $K_{mpc}$ |
| `[M,Lzerot]=Mdu(a,N,n_in,Nc)` | 증분 제약 행렬, $L(m)^T$ 블록을 쌓음 |
| `M=Mu(a,N,n_in,Nc)` | 진폭 제약 행렬, $\sum_iL(i)^T$ 블록을 쌓음 |
| `eta=QPhild(Omega,Psi*Xf,M,gamma)` | 제약 QP 해 |
| `M_act=C*B*L0'; E=C*A;` | $CBL(0)^T$, $CA$ (출력 제약 닫힌형) |

## 연결

- 1장 수식: [[08_Key_Equations|1장 핵심 수식 정리]]
- 2장 수식: [[14_Key_Equations|2장 핵심 수식 정리]]
- 용어: [[MPC3_Glossary|MPC 3장 용어집]]
- 전체 목차: [[00_MPC3_Index|MPC 3장 인덱스]]
