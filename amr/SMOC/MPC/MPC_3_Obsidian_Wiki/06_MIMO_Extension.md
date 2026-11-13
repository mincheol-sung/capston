---
title: "06. MIMO 시스템으로의 확장"
aliases:
  - "MIMO Laguerre MPC"
  - "다입력 Laguerre"
tags:
  - mpc
  - laguerre
  - mimo
source_pages: "PDF p.22-24 / book p.106-108"
---

# MIMO 시스템으로의 확장

SISO에서 세운 식을 다입력 시스템으로 넓힌다. 핵심은 **입력마다 Laguerre 극점 $a_i$ 와 항 수 $N_i$ 를 독립적으로 고른다**는 것이다. 어떤 입력의 증분 제어를 빨리 감쇠시키고 싶으면 그 입력의 $a_i$ 를 0으로, 천천히 감쇠시키고 싶으면 0보다 크게 잡을 수 있다. 액추에이터마다 속도가 다른 실제 플랜트에서 이 자유도가 유용하다.

## 입력별 파라미터화

입력 벡터와 입력행렬을 열 단위로 나눈다.

$$
\Delta u(k)=
\begin{bmatrix}
\Delta u_1(k) & \Delta u_2(k) & \cdots & \Delta u_m(k)
\end{bmatrix}^T,
\qquad
B=
\begin{bmatrix}
B_1 & B_2 & \cdots & B_m
\end{bmatrix}
$$

$m$ 은 입력 개수, $B_i$ 는 $B$ 의 $i$ 번째 열이다. $i$ 번째 제어신호는 자기만의 $(a_i,N_i)$ 로

$$
\Delta u_i(k)=L_i(k)^T\eta_i,
\qquad
L_i(k)^T=
\begin{bmatrix}
l_1^i(k) & l_2^i(k) & \cdots & l_{N_i}^i(k)
\end{bmatrix}
$$

로 표현한다. $\eta_i$ 는 $N_i\times 1$ 이다.

## 예측식

$B\Delta u(k)=\sum_iB_i\Delta u_i(k)=\sum_iB_iL_i(k)^T\eta_i$ 이므로 상태 예측은

$$
x(k_i+m\mid k_i)=A^mx(k_i)+\sum_{j=0}^{m-1}A^{m-j-1}
\begin{bmatrix}
B_1L_1(j)^T & B_2L_2(j)^T & \cdots & B_mL_m(j)^T
\end{bmatrix}
\eta
=A^mx(k_i)+\phi(m)^T\eta
$$

이다. 파라미터 벡터와 데이터 행렬은 입력별 조각을 이어 붙인 것이다.

$$
\eta^T=
\begin{bmatrix}
\eta_1^T & \eta_2^T & \cdots & \eta_m^T
\end{bmatrix}
$$

$$
\phi(m)^T=\sum_{j=0}^{m-1}A^{m-j-1}
\begin{bmatrix}
B_1L_1(j)^T & B_2L_2(j)^T & \cdots & B_mL_m(j)^T
\end{bmatrix}
$$

$\eta$ 의 총 차원은 $\sum_iN_i$ 이고, $\phi(m)^T$ 는 $n\times\sum_iN_i$ 다.

$k$ 번째 블록

$$
\phi(m)_k^T=\sum_{j=0}^{m-1}A^{m-j-1}B_kL_k(j)^T
$$

는 SISO의 $S_c(m)$ 과 구조가 같다. 따라서 [[04_Minimization_and_Convolution_Sum|재귀식]] $S_c(m)=AS_c(m-1)+S_c(1)(A_l^{m-1})^T$ 를 **입력별로 따로** 돌린 뒤 블록을 옆으로 이어 붙이면 된다. 다변수 문제를 부분시스템으로 분해해서 합성곱 합을 계산하는 것이다. [[07_MATLAB_dmpc|dmpc.m]] 의 안쪽 `for kk=2:n_in` 루프가 이 블록별 재귀다.

## 비용함수와 최적해

비용함수는 SISO와 같은 꼴이다.

$$
J=\eta^T\Big(\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L\Big)\eta
+2\eta^T\Big(\sum_{m=1}^{N_p}\phi(m)QA^m\Big)x(k_i)
+\sum_{m=1}^{N_p}x(k_i)^T(A^T)^mQA^mx(k_i)
=\eta^T\Omega\eta+2\eta^T\Psi x(k_i)+\cdots
$$

$$
\Omega=\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L,
\qquad
\Psi=\sum_{m=1}^{N_p}\phi(m)QA^m
$$

$R_L$ 은 이제 블록 대각행렬로, $i$ 번째 블록이 $r_{w,i}I_{N_i}$ 다. 입력 $i$ 의 가중 $r_{w,i}$ 가 그 입력의 계수 $N_i$ 개 전부에 똑같이 붙는다. 이유는 SISO와 같다. 각 입력의 Laguerre 함수가 직교정규이므로 $\sum_m\Delta u_i(m)^2\approx\eta_i^T\eta_i$ 다.

무제약 최적해는

$$
\eta=-\Omega^{-1}\Psi x(k_i)
$$

이다.

## 이동구간 제어와 피드백 이득

첫 샘플의 제어 증분은 입력별로 $\Delta u_i(k_i)=L_i(0)^T\eta_i$ 이므로

$$
\Delta u(k_i)=
\begin{bmatrix}
L_1(0)^T & o_2^T & \cdots & o_m^T \\
o_1^T & L_2(0)^T & \cdots & o_m^T \\
\vdots & \vdots & \ddots & \vdots \\
o_1^T & o_2^T & \cdots & L_m(0)^T
\end{bmatrix}
\eta
$$

이다. $o_k^T$ 는 $L_k(0)^T$ 와 같은 차원의 영 행벡터다. 이 블록 대각행렬은 각 입력의 계수 블록에서 자기 $L_i(0)^T$ 만 뽑아내는 역할을 한다. [[08_MATLAB_Simulation|simuuc.m]] 에서 `Lzerot` 이라는 이름으로 넘어오는 행렬이 이것이다.

미래 기준궤적이 예측구간 안에서 상수라고 가정하고 $k_i$ 를 $k$ 로 바꾸면 선형 상태 피드백

$$
\Delta u(k)=-K_{mpc}x(k),
\qquad
K_{mpc}=
\begin{bmatrix}
L_1(0)^T & o_2^T & \cdots & o_m^T \\
o_1^T & L_2(0)^T & \cdots & o_m^T \\
\vdots & \vdots & \ddots & \vdots \\
o_1^T & o_2^T & \cdots & L_m(0)^T
\end{bmatrix}
\Omega^{-1}\Psi
$$

이고, 폐루프는

$$
x(k+1)=(A-BK_{mpc})x(k)
$$

다. 폐루프 안정성과 성능은 $A-BK_{mpc}$ 의 고유값 위치로 확인한다. [[07_MATLAB_dmpc|Tutorial 3.3]]에서 5입력 4출력 시스템에 대해 이 고유값을 `dlqr` 의 고유값과 겹쳐 그려 프로그램을 검증한다.

## SISO 식과의 대응

| 항목 | SISO | MIMO |
|---|---|---|
| 파라미터 | $(a,N)$ 하나 | 입력별 $(a_i,N_i)$ |
| $\eta$ 차원 | $N$ | $\sum_iN_i$ |
| $\phi(m)^T$ | $S_c(m)$ | 입력별 $S_c$ 블록을 옆으로 이어 붙임 |
| $R_L$ | $r_wI_N$ | 블록 대각 $\mathrm{diag}(r_{w,1}I_{N_1},\ldots,r_{w,m}I_{N_m})$ |
| $\Delta u(k_i)$ 추출 | $L(0)^T$ | 블록 대각 $L_i(0)^T$ 행렬 |

## 연결

- SISO 식의 유도: [[04_Minimization_and_Convolution_Sum|비용함수 최소화와 합성곱 합]]
- 구현: [[07_MATLAB_dmpc|dmpc.m]], [[08_MATLAB_Simulation|simuuc.m / simuob.m]]
- 1장의 MIMO 취급: [[05_MIMO_MPC|1장 - MIMO MPC]]
- 제약에서도 입력별 블록 구조가 그대로 쓰인다: [[09_Constraints_on_Delta_u|Δu 제약]]

## → 검증

- 입력별로 $(a_i,N_i)$ 를 다르게 고르는 이유를 액추에이터 속도의 관점에서 설명할 수 있는가?
- $\phi(m)^T$ 의 $k$ 번째 블록이 왜 SISO의 $S_c(m)$ 과 같은 재귀식으로 계산되는지 말할 수 있는가?
- $\Delta u(k_i)$ 를 $\eta$ 에서 뽑아내는 블록 대각행렬의 크기가 $m\times\sum_iN_i$ 인 이유를 말할 수 있는가?
