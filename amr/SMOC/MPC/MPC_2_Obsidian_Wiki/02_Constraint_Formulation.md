---
title: "02. 운전 제약의 종류와 수식화"
aliases:
  - "Operational Constraints"
  - "제약 수식화"
  - "Example 2.3"
tags:
  - mpc
  - constraints
  - formulation
source_pages: "PDF p.5-10 / book p.47-52"
---

# 운전 제약의 종류와 수식화

Example 2.2에서 한 일은 "제약이 걸렸을 때 $\Delta u(k)$ 를 상황에 맞게 고친 것"이었다. 예측제어는 이것을 최적화로 체계화한다. 그러려면 먼저 운전 제약을 제어변수와 플랜트 변수에 대한 **선형 부등식**으로 표현해야 한다.

## 자주 쓰이는 제약 3종

응용에서 마주치는 제약은 크게 세 종류다. 앞의 둘은 제어변수 $u(k)$ 에, 셋째는 출력 $y(k)$ 또는 상태 $x(k)$ 에 걸린다. 설명은 SISO로 하고 뒤에서 MIMO로 확장한다.

### 제어변수 변화율 제약

제어신호가 한 스텝에 얼마나 움직일 수 있는지에 대한 하드 제약이다.

$$
\Delta u^{min}\le\Delta u(k)\le\Delta u^{max}
$$

교재가 "less than **plus equal to**"라고 굳이 강조하는 이유가 있다. 등호가 성립하는 경우, 즉 해가 제약 경계에 정확히 놓이는 경우가 바로 최적해가 되기 때문이다. 뒤의 [[05_QP_Inequality_KKT_ActiveSet|Active Set]]에서 $M_ix=\gamma_i$ 를 등식으로 다루는 근거가 이 등호다.

활용 방법 두 가지를 기억해 두면 좋다.

- **방향성 제약**: $u(k)$ 가 증가만 가능하고 감소는 불가능하다면 $0\le\Delta u(k)\le\Delta u^{max}$ 로 둔다.
- **슬루율 제한**: 한 스텝에 0.1 단위 이상 움직일 수 없다면 $-0.1\le\Delta u(k)\le 0.1$ 이다.

### 제어변수 진폭 제약

모든 제약 중 가장 흔하다. 밸브를 100% 넘게 열 수 없고 전압이 정해진 범위를 벗어날 수 없다. 물리적 하드 제약이다.

$$
u^{min}\le u(k)\le u^{max}
$$

여기서 가장 자주 나오는 실수를 짚고 간다. MPC의 $u(k)$ 는 **증분 변수**이지 실제 물리량이 아니다.

$$
u_{\text{물리}}=u_{\text{증분}}+u_{ss}
$$

따라서 제약도 정상상태값 $u_{ss}$ 를 빼서 변환해야 한다. 교재의 예를 그대로 옮기면, 밸브가 15%에서 80% 사이에서 동작하고 정상 운전값이 30%일 때

$$
u^{min}=15-30=-15\%, \qquad u^{max}=80-30=50\%
$$

가 된다.

### 출력 제약

플랜트 출력의 운전 범위도 지정할 수 있다.

$$
y^{min}\le y(k)\le y^{max}
$$

실무에서는 슬랙 변수 $s_v>0$ 를 넣어 소프트 제약으로 쓴다.

$$
y^{min}-s_v\le y(k)\le y^{max}+s_v
$$

왜 소프트로 해야 하는가. 출력 제약이 활성화되면 제어와 증분 제어가 크게 변하도록 요구한다. 그러면 $u$ 와 $\Delta u$ 가 자기 제약을 위반해서 **제약 충돌**이 생긴다. 플랜트 운전에서는 보통 제어변수 제약이 더 본질적이므로, 출력 제약을 $s_v$ 를 크게 잡아 완화시켜 충돌을 해소한다. 이 상황의 실제 사례는 [[13_Output_Constraints|출력 제약과 제약 충돌]]에 있다.

상태변수 제약도 마찬가지다. 측정 가능하면 상태에, 아니면 관측기 상태에 제약을 걸 수 있는데, 같은 이유로 소프트 제약 형태여야 한다.

### MIMO 설정

입력이 여러 개면 각 입력에 독립적으로 제약을 지정한다. 상한이 $[\Delta u_1^{max}\ \Delta u_2^{max}\ \cdots\ \Delta u_m^{max}]$, 하한이 $[\Delta u_1^{min}\ \cdots\ \Delta u_m^{min}]$ 이면

$$
\Delta u_i^{min}\le\Delta u_i(k)\le\Delta u_i^{max}, \qquad i=1,\ldots,m
$$

진폭도 똑같이

$$
u_i^{min}\le u_i(k)\le u_i^{max}, \qquad i=1,\ldots,m
$$

이다. 출력과 상태도 각각 독립적으로 쓴다. 요컨대 MIMO는 각 입출력마다 따로 쓰면 그것으로 끝이다.

## 2.3.2 최적해의 일부로서의 제약조건

핵심은 **파라미터화**다. 제약을 받는 변수들을 예측제어 설계에서 쓴 것과 동일한 파라미터 벡터 $\Delta U$ 로 표현한다. 그러면 모든 제약이 $\Delta U$ 에 대한 선형 방정식과 부등식이 된다. 최적화 문헌에서는 $\Delta U$ 를 **결정변수**라고 부른다.

이동구간 방식이므로 최적화 창이 매 스텝 이동하고, 제약도 매 창마다 다시 세팅된다. 덕분에 창 시작마다 제약을 바꿀 수 있고, 제약 제어 문제를 수치적으로 다룰 수 있게 된다.

시각 $k_i$ 에서 미래를 내다보므로, 예를 들어 처음 세 샘플에 대해서는

$$
\Delta u^{min}\le\Delta u(k_i)\le\Delta u^{max}
$$

$$
\Delta u^{min}\le\Delta u(k_i+1)\le\Delta u^{max}
$$

$$
\Delta u^{min}\le\Delta u(k_i+2)\le\Delta u^{max}
$$

가 된다. 원칙적으로는 예측구간 전체에 제약을 걸지만, 계산 부하를 줄이려고 일부 샘플 시점에만 거는 경우가 많다. 두 방식의 실제 비교는 [[10_Constraints_on_Rate_of_Change|변화율 제약]]과 [[11_Constraints_on_Amplitude|진폭 제약]] 노트에 있다.

## Example 2.3 - 제약을 $\Delta U$ 의 함수로 표현하기

모터 제어에서 입력 전압이 2 V에서 6 V로 제한되어 있다. 제어신호의 정상상태값은 4 V이고 제어구간은 $N_c=4$ 다. 처음 두 샘플 $\Delta u(k_i)$, $\Delta u(k_i+1)$ 에 대한 제약을 $\Delta U$ 로 표현하자.

결정변수는

$$
\Delta U=
\begin{bmatrix}
\Delta u(k_i) &
\Delta u(k_i+1) &
\Delta u(k_i+2) &
\Delta u(k_i+3)
\end{bmatrix}^T
$$

$u$ 를 $\Delta U$ 로 누적해서 쓰면

$$
u(k_i)=u(k_i-1)+\Delta u(k_i)=u(k_i-1)+
\begin{bmatrix}
1 & 0 & 0 & 0
\end{bmatrix}
\Delta U
$$

$$
u(k_i+1)=u(k_i-1)+\Delta u(k_i)+\Delta u(k_i+1)=u(k_i-1)+
\begin{bmatrix}
1 & 1 & 0 & 0
\end{bmatrix}
\Delta U
$$

정상상태값을 빼서 증분변수로 바꾸면 $u^{min}=2-4=-2$, $u^{max}=6-4=2$ 이므로

$$
\begin{bmatrix}
-2 \\
-2
\end{bmatrix}
\le
\begin{bmatrix}
1 \\
1
\end{bmatrix}
u(k_i-1)
+
\begin{bmatrix}
1 & 0 & 0 & 0 \\
1 & 1 & 0 & 0
\end{bmatrix}
\Delta U
\le
\begin{bmatrix}
2 \\
2
\end{bmatrix}
$$

여기서 두 가지를 관찰할 수 있다. 첫째, 제약이 $\Delta U$ 에 대해 선형이므로 QP로 풀 수 있다. 둘째, $u(k_i-1)$ 이 제약식 안에 들어 있으므로 제약식을 **매 샘플마다 갱신**해야 한다. 실시간 구현에서 놓치기 쉬운 부분이다.

## 양쪽 부등식을 한 방향으로 통일하기

QP 표준형은 $Mx\le\gamma$ 한 방향만 쓴다. 그래서 $\Delta U^{min}\le\Delta U\le\Delta U^{max}$ 를 두 개의 부등식으로 쪼갠다.

$$
-\Delta U\le-\Delta U^{min}
$$

$$
\Delta U\le\Delta U^{max}
$$

행렬 형태로 묶으면

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

이 절차는 제어 제약이든 출력 제약이든 모든 종류에 동일하게 적용된다.

## 진폭 제약의 일반형

모든 미래 샘플에 대해 $u$ 를 $\Delta U$ 로 표현하면 다음 구조가 나온다.

$$
\begin{bmatrix}
u(k_i) \\
u(k_i+1) \\
u(k_i+2) \\
\vdots \\
u(k_i+N_c-1)
\end{bmatrix}
=
\begin{bmatrix}
I \\
I \\
I \\
\vdots \\
I
\end{bmatrix}
u(k_i-1)
+
\begin{bmatrix}
I & 0 & 0 & \cdots & 0 \\
I & I & 0 & \cdots & 0 \\
I & I & I & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
I & I & I & \cdots & I
\end{bmatrix}
\begin{bmatrix}
\Delta u(k_i) \\
\Delta u(k_i+1) \\
\Delta u(k_i+2) \\
\vdots \\
\Delta u(k_i+N_c-1)
\end{bmatrix}
$$

앞의 열벡터를 $C_1$, 하삼각 행렬을 $C_2$ 라고 부른다. $u$ 가 $\Delta u$ 의 누적합이므로 $C_2$ 가 lower-triangular ones 구조가 되는 것은 당연하다. SISO면 $I$ 자리에 1이 들어간다.

이 표기로 제어 진폭 제약은

$$
-(C_1u(k_i-1)+C_2\Delta U)\le-U^{min}
$$

$$
(C_1u(k_i-1)+C_2\Delta U)\le U^{max}
$$

가 된다. $U^{min}$ 과 $U^{max}$ 는 $u^{min}$, $u^{max}$ 를 $N_c$ 개 쌓은 열벡터다.

증분 제약은

$$
-\Delta U\le-\Delta U^{min}, \qquad \Delta U\le\Delta U^{max}
$$

출력 제약은 $Y=Fx(k_i)+\Phi\Delta U$ 를 그대로 넣어서

$$
Y^{min}\le Fx(k_i)+\Phi\Delta U\le Y^{max}
$$

로 쓴다.

## 연결

- 이 결과를 조립한 최종 형태: [[03_Constrained_MPC_Problem|제약 있는 MPC 문제의 최종 형태]]
- 앞 단계: [[01_Motivational_Examples|동기부여 예제]]
- $Y=Fx+\Phi\Delta U$ 의 유도: [[03_One_Window_Optimization|1장 한 최적화 창에서의 예측제어]]
- 실제 적용 예: [[11_Constraints_on_Amplitude|진폭 제약 예측제어]]
