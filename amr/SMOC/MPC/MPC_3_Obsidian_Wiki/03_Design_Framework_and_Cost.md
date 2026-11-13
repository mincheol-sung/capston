---
title: "03. 설계 프레임워크와 비용함수"
aliases:
  - "Design Framework"
  - "Laguerre DMPC Cost Function"
  - "eta 파라미터화"
tags:
  - mpc
  - laguerre
  - cost-function
  - prediction
source_pages: "PDF p.8-12 / book p.92-96"
---

# 설계 프레임워크와 비용함수

이 노트는 3장의 핵심 치환을 다룬다. 결정변수를 $\Delta U$ 에서 Laguerre 계수 벡터 $\eta$ 로 바꾸고, 그에 맞게 예측식과 비용함수를 다시 쓴다. 이 치환 하나로 제어구간 $N_c$ 가 사라지고 $(a,N)$ 이 그 자리를 대신한다.

## 3.3.1 설계 프레임워크

시각 $k_i$ 에서 미래 제어궤적 $\Delta u(k_i),\Delta u(k_i+1),\ldots$ 을 **안정한 동적 시스템의 임펄스 응답**으로 간주한다. 그러면 [[02_Laguerre_System_Modelling|앞 노트]]에서 임펄스 응답을 근사했던 것과 똑같이, Laguerre 함수 $l_1(k),\ldots,l_N(k)$ 로 이 궤적을 표현할 수 있다. 임의의 미래 샘플 $k$ 에서

$$
\Delta u(k_i+k)=\sum_{j=1}^{N}c_j(k_i)l_j(k)
$$

이다. $k_i$ 는 이동구간 창의 시작 시각, $k$ 는 창 안에서의 미래 샘플, $N$ 은 전개 항 수, $c_j$ 는 계수다. 계수는 창의 시작 시각 $k_i$ 의 함수이고, 매 창마다 새로 구한다.

벡터로 쓰면

$$
\Delta u(k_i+k)=L(k)^T\eta,
\qquad
\eta=
\begin{bmatrix}
c_1 & c_2 & \cdots & c_N
\end{bmatrix}^T
$$

이다. $L(k)$ 는 [[01_Laguerre_Functions|차분방정식]] $L(k+1)=A_lL(k)$ 로 생성되는 Laguerre 함수 벡터다.

이 프레임워크에서 **제어구간 $N_c$ 는 사라진다.** 대신 항 수 $N$ 이 극점 $a$ 와 짝을 이루어 궤적의 복잡도를 기술한다. 예를 들어 $a$ 를 크게 잡으면 함수가 천천히 감쇠하므로 적은 $N$ 으로도 긴 제어구간을 표현할 수 있다. $a=0$, $N=N_c$ 로 두면 앞 장의 방법이 그대로 복원된다.

## 예측식

상태공간 모델 $(A,B,C)$ 에 입력 $\Delta u(\cdot)$ 가 들어가고 초기상태가 $x(k_i)$ 일 때, 미래 $m$ 스텝 뒤의 상태 예측은

$$
x(k_i+m\mid k_i)=A^mx(k_i)+\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T\eta
$$

이다. 1장의 예측식에서 $\Delta u(k_i+i)$ 자리에 $L(i)^T\eta$ 를 넣은 것뿐이다. 출력 예측은

$$
y(k_i+m\mid k_i)=CA^mx(k_i)+\sum_{i=0}^{m-1}CA^{m-i-1}BL(i)^T\eta
$$

이다. 상태와 출력 예측이 모두 $\Delta U$ 대신 **계수 벡터 $\eta$ 에 대한 선형식**이 되었다. 이제 최적화 대상은 $\eta$ 다.

## 3.3.2 비용함수

1장과 2장의 행렬-벡터 형식은 이해하기 쉽고 구현도 간단하지만, $F$ 와 $\Phi$ 같은 큰 데이터 행렬을 만들어 저장해야 한다. 입출력이 많은 시스템에서는 메모리 부담이 크다. Laguerre 함수를 쓰면 비용함수를 다른 방식으로 조립할 수 있고, 특히 프로그래밍 관점에서 편하다.

### 원래 비용함수

SISO 시스템에서 시각 $k_i$ 의 설정치 $R_s$ 는 예측구간 안에서 상수이고, 원래 비용함수는

$$
J=(R_s-Y)^T(R_s-Y)+\Delta U^T\bar R\Delta U
$$

$$
Y=
\begin{bmatrix}
y(k_i+1\mid k_i) & y(k_i+2\mid k_i) & \cdots & y(k_i+N_p\mid k_i)
\end{bmatrix}^T
$$

$$
\Delta U=
\begin{bmatrix}
\Delta u(k_i) & \Delta u(k_i+1) & \cdots & \Delta u(k_i+N_c-1)
\end{bmatrix}^T,
\qquad
R_s^T=
\begin{bmatrix}
1 & 1 & \cdots & 1
\end{bmatrix}r(k_i)
$$

이며 $\bar R$ 은 같은 원소 $r_w$ 를 가진 대각행렬이다.

### 합 형태로 다시 쓰기

$Y$ 와 $\Delta U$ 가 벡터이므로 위 비용함수는 성분별 합으로

$$
J=\sum_{m=1}^{N_p}\big(r(k_i)-y(k_i+m\mid k_i)\big)^T\big(r(k_i)-y(k_i+m\mid k_i)\big)+\eta^TR_L\eta
$$

와 같다. $R_L$ 은 대각원소가 $r_w\ge 0$ 인 $N\times N$ 대각행렬이다.

여기서 $\Delta U^T\bar R\Delta U$ 가 $\eta^TR_L\eta$ 로 바뀌는 대목이 직교정규성이 쓰이는 자리다. 교재가 한 줄로 넘어가는 이 단계를 풀어 쓰면 다음과 같다.

$$
\Delta U^T\bar R\Delta U=\sum_{m=0}^{N_p}\Delta u(k_i+m)^Tr_w\Delta u(k_i+m)
$$

에 $\Delta u(k_i+m)=L(m)^T\eta$ 를 넣으면

$$
=r_w\sum_{m=0}^{N_p}\eta^TL(m)L(m)^T\eta
=r_w\,\eta^T\Big(\sum_{m=0}^{N_p}L(m)L(m)^T\Big)\eta
$$

이고, 괄호 안의 행렬은 [[01_Laguerre_Functions|Example 3.1]]에서 본 유한합 $S$ 다. $N_p$ 가 충분히 크면

$$
\sum_{m=0}^{N_p}l_i(m)l_j(m)=0\ (i\ne j),
\qquad
\sum_{m=0}^{N_p}l_i(m)l_j(m)=1\ (i=j)
$$

이므로 $S\approx I$ 가 되어 $\Delta U^T\bar R\Delta U\approx r_w\eta^T\eta=\eta^TR_L\eta$ 다. 따라서 **"충분히 큰 $N_p$"라는 조건은 $a$ 와 $N$ 의 선택에 묶여 있다.** $a=0.9$ 에서 $N_p=20$ 으로 잡으면 이 근사가 성립하지 않는다는 것을 Example 3.1이 이미 보여 주었다.

$N$ 개의 계수 각각에 같은 가중 $r_w$ 가 붙는 것도 같은 이유다. 직교정규 기저에서는 궤적의 에너지가 계수 제곱합과 같으므로, 궤적에 대한 가중이 그대로 계수에 대한 대각 가중이 된다.

### DLQR 형태의 비용함수

교재는 여기서 비용함수를 한 번 더 바꿔 쓴다.

$$
J=\sum_{m=1}^{N_p}x(k_i+m\mid k_i)^TQx(k_i+m\mid k_i)+\eta^TR_L\eta
$$

가중행렬은 $Q\ge 0$, $R_L>0$ 이고, $Q$ 의 차원은 상태변수 개수, $R_L$ 의 차원은 $\eta$ 의 차원과 같다. 이렇게 쓰는 이유는 **이산시간 LQR(DLQR)과 연결하기 위해서**다. DLQR의 방대한 고전 결과를 MPC의 해석, 튜닝, 설계에 그대로 끌어다 쓸 수 있게 된다. 이 연결은 [[05_Optimal_Trajectory_and_DLQR|Example 3.3~3.5]]에서 수치로 확인한다.

#### 조정기 설계 - $r(k)=0$

설정치가 0이면 $Q=C^TC$ 로 고르는 순간 두 비용함수가 같아진다. $x^TC^TCx=(Cx)^T(Cx)=y^2$ 이기 때문이다. 증강 모델에서 $r(k)$ 는 운전점으로부터의 증분 변화를 뜻하므로 $r(k)=0$ 은 "플랜트를 현재 정상상태에 유지한다"는 뜻이고, 이 형태는 외란 제거와 폐루프 안정화가 목적인 조정기(regulator) 설계에서 전통적으로 쓰인다.

#### 설정치 포함 - $r(k)\ne 0$

설정치가 0이 아니면 상태변수를 다시 정의한다. 증강 모델의 출력행렬은

$$
C=
\begin{bmatrix}
0 & 0 & \cdots & 0 & 1
\end{bmatrix}
$$

이고 상태벡터는 $x(k)=[\Delta x_m(k)^T\ \ y(k)]^T$ 다. 여기서

$$
x_r(k_i)=
\begin{bmatrix}
0 & 0 & \cdots & 0 & r(k_i)
\end{bmatrix}^T
$$

라는 벡터를 잡으면 $r(k_i)=Cx_r(k_i)$ 다. 설정치는 최적화 창 안에서 상수이므로, 창 안의 상태를

$$
x(k_i+m\mid k_i)=
\begin{bmatrix}
\Delta x_m(k_i+m\mid k_i)^T & y(k_i+m\mid k_i)-r(k_i)
\end{bmatrix}^T
$$

로 잡으면 마지막 성분이 출력오차가 되고, $Q=C^TC$ 인 비용함수

$$
J=\sum_{m=1}^{N_p}x(k_i+m\mid k_i)^TQx(k_i+m\mid k_i)+\eta^TR_L\eta
$$

는 출력오차 최소화 비용함수와 완전히 같아진다. 마지막 성분만 $y-r$ 로 바꾸었으므로 $x^TC^TCx=(y-r)^2$ 이기 때문이다.

이 재정의는 **예측에 쓰는 모델은 전혀 바꾸지 않는다.** $r$ 은 상수이므로 $\Delta(y-r)=\Delta y$ 이고, 증강 모델의 동역학은 그대로다. 바뀌는 것은 피드백에 넣는 상태벡터의 마지막 성분뿐이다. [[08_MATLAB_Simulation|simuuc.m]] 에서 `Xf=[xm-xm_old;(y-sp(:,kk+1))]` 로 조립하는 것이 바로 이 상태다.

## 1장 표기와의 대응 (교재 생략)

1장에서 비용함수는 $J=\Delta U^T(\Phi^T\Phi+\bar R)\Delta U-2\Delta U^T\Phi^T(R_s-Fx(k_i))$ 였다. 상태 마지막 성분을 $y-r$ 로 두면 $R_s-Fx(k_i)$ 의 역할을 $-Fx(k_i)$ 가 대신한다. 따라서 [[04_Minimization_and_Convolution_Sum|다음 노트]]에서 나오는 $\Omega$ 와 $\Psi$ 는 대략

$$
\Omega\ \leftrightarrow\ \Phi^T\Phi+\bar R,
\qquad
\Psi\ \leftrightarrow\ \Phi^TF
$$

에 대응한다. 단 열 공간이 $\Delta U$ 가 아니라 $\eta$ 로 바뀌어 있다. 두 표기를 오갈 때 이 대응을 기억하면 헷갈리지 않는다.

## 연결

- 기저 함수: [[01_Laguerre_Functions|이산시간 Laguerre 함수]]
- 최적해와 $\Omega$, $\Psi$ 의 계산: [[04_Minimization_and_Convolution_Sum|비용함수 최소화와 합성곱 합]]
- 1장의 예측식과 비용함수: [[03_One_Window_Optimization|1장 - 한 최적화 창에서의 예측제어]]
- 증강 모델과 $\Delta u$ 가 결정변수인 이유: [[02_Augmented_State_Space_Model|1장 - 증분 상태공간 모델]]

## → 검증

- $\Delta U^T\bar R\Delta U=\eta^TR_L\eta$ 가 성립하려면 어떤 조건이 필요한지, Example 3.1과 연결해 말할 수 있는가?
- $Q=C^TC$ 로 두면 DLQR 형태 비용함수가 출력오차 비용함수와 같아지는 이유를 한 줄 계산으로 보일 수 있는가?
- 설정치를 상태벡터에 넣을 때 예측 모델이 바뀌지 않는 이유를 말할 수 있는가?
