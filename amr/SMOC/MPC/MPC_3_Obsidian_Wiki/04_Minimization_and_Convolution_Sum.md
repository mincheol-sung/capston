---
title: "04. 비용함수 최소화, 합성곱 합, 이동구간 제어"
aliases:
  - "Omega Psi"
  - "Convolution Sum"
  - "K_mpc Laguerre"
  - "P_dmpc"
tags:
  - mpc
  - laguerre
  - optimization
  - receding-horizon
source_pages: "PDF p.13-15 / book p.97-99"
---

# 비용함수 최소화, 합성곱 합, 이동구간 제어

[[03_Design_Framework_and_Cost|앞 노트]]에서 세운 비용함수를 $\eta$ 에 대해 최소화한다. 결과는 두 행렬 $\Omega$ 와 $\Psi$ 로 압축되고, 이 두 행렬을 만드는 재귀식이 [[07_MATLAB_dmpc|dmpc.m]] 의 뼈대다.

## 3.3.3 목적함수의 최소화

상태 예측을

$$
x(k_i+m\mid k_i)=A^mx(k_i)+\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T\eta=A^mx(k_i)+\phi(m)^T\eta
$$

로 쓴다. 여기서

$$
\phi(m)^T=\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T
$$

이다. 차원을 확인해 두면 뒤가 편하다. $A^{m-i-1}B$ 는 $n\times 1$, $L(i)^T$ 는 $1\times N$ 이므로 $\phi(m)^T$ 는 $n\times N$ 이고, $\phi(m)$ 의 행 개수 $N$ 은 $\eta$ 의 행 개수와 같다.

이를 비용함수 $J=\sum_{m=1}^{N_p}x(k_i+m\mid k_i)^TQx(k_i+m\mid k_i)+\eta^TR_L\eta$ 에 넣고 전개하면

$$
J=\eta^T\Big(\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L\Big)\eta
+2\eta^T\Big(\sum_{m=1}^{N_p}\phi(m)QA^m\Big)x(k_i)
+\sum_{m=1}^{N_p}x(k_i)^T(A^T)^mQA^mx(k_i)
$$

이다. 전개 과정은 $(A^mx+\phi^T\eta)^TQ(A^mx+\phi^T\eta)$ 를 세 항으로 나눈 것이고, 교차항 $x^T(A^T)^mQ\phi^T\eta$ 는 스칼라라 전치해도 같으므로 두 배가 되어 $2\eta^T\phi QA^mx$ 가 된다.

무제약 최소를 위해 $\eta$ 로 편미분하면

$$
\frac{\partial J}{\partial\eta}=2\Big(\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L\Big)\eta+2\Big(\sum_{m=1}^{N_p}\phi(m)QA^m\Big)x(k_i)
$$

이고, 괄호 안 행렬의 역행렬이 존재한다고 가정하면 $\partial J/\partial\eta=0$ 에서

$$
\eta=-\Big(\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L\Big)^{-1}\Big(\sum_{m=1}^{N_p}\phi(m)QA^m\Big)x(k_i)
$$

이다. 표기를 줄여

$$
\Omega=\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L,
\qquad
\Psi=\sum_{m=1}^{N_p}\phi(m)QA^m
$$

로 두면

$$
\eta=-\Omega^{-1}\Psi x(k_i)
$$

이다. $\Omega$ 는 $N\times N$ Hessian(의 절반), $\Psi$ 는 $N\times n$ 이다. 역행렬 존재 가정은 $R_L>0$ 이면 자동으로 만족된다. $\sum\phi Q\phi^T$ 는 반양정치이고 여기에 양정치 $R_L$ 을 더하면 양정치가 되기 때문이다. $r_w=0$ 으로 두면 이 보장이 사라지므로 주의한다.

### 비용의 최소값

$J_{min}$ 은 뒤의 [[11_Stability_Analysis|안정성 해석]]에서 Lyapunov 함수로 쓰이므로 값이 필요하다. 완전제곱으로 정리하면

$$
J=\big(\eta+\Omega^{-1}\Psi x(k_i)\big)^T\Omega\big(\eta+\Omega^{-1}\Psi x(k_i)\big)
-x(k_i)^T\Psi^T\Omega^{-1}\Psi x(k_i)
+\sum_{m=1}^{N_p}x(k_i)^T(A^T)^mQA^mx(k_i)
$$

이다. 첫 항은 $\eta=-\Omega^{-1}\Psi x(k_i)$ 에서 0이 되므로

$$
J_{min}=x(k_i)^T\Big(\sum_{m=1}^{N_p}(A^T)^mQA^m-\Psi^T\Omega^{-1}\Psi\Big)x(k_i)=x(k_i)^TP_{dmpc}x(k_i)
$$

$$
P_{dmpc}=\sum_{m=1}^{N_p}(A^T)^mQA^m-\Psi^T\Omega^{-1}\Psi
$$

이다. 완전제곱이 맞는지는 첫 항을 전개해 보면 된다. $\eta^T\Omega\eta+2\eta^T\Psi x+x^T\Psi^T\Omega^{-1}\Psi x$ 가 나오고, 마지막 항을 빼 주면 원래 $J$ 의 앞 두 항과 일치한다.

$J_{min}$ 이 상태의 이차형식이라는 사실은 DLQR의 $x^TP_\infty x$ 와 같은 구조다. 이 유사성이 [[05_Optimal_Trajectory_and_DLQR|DLQR로의 수렴]]과 안정성 논의의 출발점이다.

## 3.3.4 합성곱 합 (Convolution Sum)

$\Omega$ 와 $\Psi$ 를 만들려면 매 $m$ 마다

$$
S_c(m)=\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T
$$

를 계산해야 한다. 정의대로 하면 $m$ 마다 $m$ 개의 항을 더해야 하므로 비용이 $N_p^2$ 에 비례한다. 교재는 이것을 재귀식으로 바꾼다.

$$
S_c(1)=BL(0)^T
$$

$$
S_c(2)=ABL(0)^T+BL(1)^T=ABL(0)^T+BL(0)^TA_l^T=AS_c(1)+S_c(1)A_l^T
$$

$$
S_c(3)=A^2BL(0)^T+ABL(1)^T+BL(2)^T=AS_c(2)+S_c(1)(A_l^2)^T
$$

여기서 쓴 것은 Laguerre 함수의 차분방정식 $L(k+1)=A_lL(k)$, 즉 $L(k)^T=L(0)^T(A_l^k)^T$ 다. 일반항은

$$
S_c(m)=AS_c(m-1)+S_c(1)(A_l^{m-1})^T,
\qquad
m=2,3,\ldots,N_p
$$

이다.

$S_c(3)$ 의 두 번째 등호가 왜 성립하는지 한 번 확인해 두자. $AS_c(2)=A^2BL(0)^T+ABL(1)^T$ 이고, 남는 항 $BL(2)^T=BL(0)^T(A_l^2)^T=S_c(1)(A_l^2)^T$ 다. 즉 "앞 항에 $A$ 를 곱하고, 새로 들어오는 마지막 항 하나만 $S_c(1)$ 에 $A_l$ 의 거듭제곱을 곱해서 더한다"는 구조다. 이 재귀식 덕분에 각 $m$ 에서 행렬곱 두 번으로 끝나고, $F$ 와 $\Phi$ 같은 큰 행렬을 저장할 필요가 없다. 이것이 [[03_Design_Framework_and_Cost|앞 노트]]에서 말한 "프로그래밍 관점의 이점"의 실체다.

$S_c(m)$ 이 곧 $\phi(m)^T$ 이므로, 매 $m$ 에서

$$
\Omega\leftarrow\Omega+\phi(m)Q\phi(m)^T,
\qquad
\Psi\leftarrow\Psi+\phi(m)QA^m
$$

을 누적하면 된다. [[07_MATLAB_dmpc|dmpc.m]] 의 `for i=2:Np` 루프가 정확히 이 계산이다.

## 3.3.5 이동구간 제어

최적 $\eta$ 를 얻으면 첫 샘플의 제어 증분은

$$
\Delta u(k_i)=L(0)^T\eta
$$

이다. $\Delta u(k_i+0)=L(0)^T\eta$ 이므로 당연하다. $L(0)^T=\sqrt{1-a^2}[1\ -a\ a^2\ \cdots\ (-1)^{N-1}a^{N-1}]$ 은 $a$ 와 $N$ 만 정해지면 상수 벡터다.

$\eta$ 가 $x(k_i)$ 의 선형함수이므로 $k_i$ 를 $k$ 로 바꾸면 선형 상태 피드백

$$
\Delta u(k)=-K_{mpc}x(k)
$$

$$
K_{mpc}=L(0)^T\Big(\sum_{m=1}^{N_p}\phi(m)Q\phi(m)^T+R_L\Big)^{-1}\sum_{m=1}^{N_p}\phi(m)QA^m=L(0)^T\Omega^{-1}\Psi
$$

가 된다. $a=0$ 이면 $L(0)^T=[1\ 0\ \cdots\ 0]$ 이므로 "첫 원소만 뽑는다"는 1장의 $K_{mpc}=[1\ 0\ \cdots\ 0](\Phi^T\Phi+\bar R)^{-1}\Phi^TF$ 와 같은 식이 된다.

설정치 정보는 [[03_Design_Framework_and_Cost|앞 노트]]대로 상태벡터 안에 들어 있다.

$$
x(k_i)=
\begin{bmatrix}
\Delta x_m(k_i)^T & e(k_i)
\end{bmatrix}^T,
\qquad
e(k_i)=y(k_i)-r(k_i)
$$

그래서 1장의 $K_y r(k_i)$ 같은 별도 설정치 항이 나타나지 않는다. 관측기를 쓰면 실제 상태 대신 추정 상태 $\hat x(k_i)$ 를 넣는다.

## 한눈에 보는 계산 순서

1. $(a,N)$ 을 정하고 `lagd` 로 $A_l$, $L(0)$ 을 만든다.
2. $S_c(1)=BL(0)^T$ 에서 시작해 재귀식으로 $S_c(m)$, $m=2,\ldots,N_p$ 를 만든다.
3. 매 $m$ 에서 $\Omega$ 와 $\Psi$ 를 누적하고, 마지막에 $\Omega$ 에 $R_L$ 을 더한다.
4. $\eta=-\Omega^{-1}\Psi x(k_i)$, $\Delta u(k_i)=L(0)^T\eta$.
5. 폐루프 확인은 $K_{mpc}=L(0)^T\Omega^{-1}\Psi$ 와 $A-BK_{mpc}$ 의 고유값으로 한다.

## 연결

- 비용함수의 유래: [[03_Design_Framework_and_Cost|설계 프레임워크와 비용함수]]
- 이 식들로 실제 궤적을 계산한 예: [[05_Optimal_Trajectory_and_DLQR|최적 궤적과 DLQR로의 수렴]]
- MIMO로 확장: [[06_MIMO_Extension|MIMO 확장]]
- 코드: [[07_MATLAB_dmpc|dmpc.m]]
- 1장의 대응식: [[04_Receding_Horizon_Control|1장 - 이동구간 제어]]

## → 검증

- $\phi(m)^T$ 의 차원이 $n\times N$ 인 이유를 말할 수 있는가?
- $S_c(m)=AS_c(m-1)+S_c(1)(A_l^{m-1})^T$ 를 $m=3$ 에 대해 손으로 확인할 수 있는가?
- $a=0$ 일 때 $K_{mpc}=L(0)^T\Omega^{-1}\Psi$ 가 1장의 $K_{mpc}$ 와 같아지는 이유를 말할 수 있는가?
- $J_{min}=x^TP_{dmpc}x$ 가 왜 완전제곱에서 나오는지 설명할 수 있는가?
