---
title: "06. 상태추정과 관측기"
aliases:
  - "State Estimation"
  - "Observer"
  - "관측기"
tags:
  - mpc
  - observer
  - state-estimation
source_pages: "PDF p.54-61 / book p.27-34"
---

# 상태추정과 관측기

MPC 설계에서는 $x(k_i)$가 사용된다. 하지만 실제 plant에서는 모든 상태변수를 직접 측정할 수 없는 경우가 많다. 그래서 출력 측정값으로 상태를 추정하는 [[Observer]]가 필요하다.

## Open-loop prediction의 한계

plant가

$$
x_m(k+1)=A_mx_m(k)+B_mu(k)
$$

라고 하자. 단순히 같은 모델로 추정하면,

$$
\hat x_m(k+1)=A_m\hat x_m(k)+B_mu(k)
$$

이고 error는

$$
\tilde x_m(k+1)=A_m\tilde x_m(k)
$$

이므로 초기 오차 $\tilde x_m(0)$에 대한 해는

$$
\tilde x_m(k)=A_m^k\tilde x_m(0)
$$

가 된다. 이 방식은 plant pole에 전적으로 의존한다.

### 왜 unit circle인가: continuous-time과 discrete-time의 해

“고유값이 음수여야 안정하다”는 기억은 continuous-time 시스템에서 나온 것이다. Continuous-time 오차 시스템

$$
\dot{\tilde x}(t)=A\tilde x(t)
$$

의 해는

$$
\tilde x(t)=e^{At}\tilde x(0)
$$

이다. 하나의 eigenmode는 $e^{\lambda t}$ 형태이므로 시간이 지날수록 0으로 수렴하려면

$$
\boxed{\operatorname{Re}(\lambda_i(A))<0}
$$

이어야 한다. 고유값이 실수일 때만 이를 간단히 “음수여야 한다”고 말할 수 있다.

반면 discrete-time 오차 시스템

$$
\tilde x(k+1)=A\tilde x(k)
$$

의 해는

$$
\tilde x(k)=A^k\tilde x(0)
$$

이다. 하나의 eigenmode는 $\lambda^k$ 형태이므로

$$
\lim_{k\to\infty}\lambda^k=0
\quad\Longleftrightarrow\quad
|\lambda|<1
$$

이다. 따라서 discrete-time 시스템의 asymptotic stability 조건은

$$
\boxed{\rho(A)=\max_i|\lambda_i(A)|<1}
$$

이며, 모든 eigenvalue가 complex plane의 unit circle 안에 있어야 한다.

| Eigenvalue 예 | 시간에 따른 mode | 판단 |
|---|---|---|
| $\lambda=0.8$ | $0.8^k\to0$ | 양수지만 stable |
| $\lambda=-0.8$ | 부호가 번갈아 바뀌며 $(-0.8)^k\to0$ | oscillatory stable |
| $\lambda=-1.2$ | $\lvert-1.2\rvert^k\to\infty$ | 음수지만 unstable |
| $\lvert\lambda\rvert=1$ | 진폭이 줄지 않음 | asymptotically stable이 아님 |

두 안정영역은 서로 다른 규칙이 아니라 sampling 관계

$$
z=e^{sT_s}
$$

로 연결된다. $s=\sigma+j\omega$라면

$$
|z|=|e^{sT_s}|=e^{\sigma T_s}
$$

이므로

$$
\operatorname{Re}(s)=\sigma<0
\quad\Longleftrightarrow\quad
|z|<1
$$

이다.

> [!important] 이 절에서 말하는 안정성
> 여기서 안정하다는 것은 단순히 오차가 bounded하다는 뜻이 아니라 $\lim_{k\to\infty}\tilde x_m(k)=0$이 되는 **asymptotic stability**를 뜻한다. 따라서 unit circle 위의 pole은 일반적으로 허용되지 않는다.

Open-loop predictor에서는 다음과 같이 정리된다.

- $A_m$의 모든 eigenvalue가 unit circle 안에 있으면 estimation error가 0으로 점근 수렴한다.
- 하나라도 unit circle 밖에 있으면 해당 error mode가 발산한다.
- unit circle 위에 있으면 해당 mode가 감쇠하지 않아 error가 0으로 수렴하지 않는다.
- 안정한 경우에도 수렴 속도는 plant pole로 정해지므로 설계자가 직접 조절하기 어렵다.

## Observer 구조

측정 출력 $y(k)$를 사용해 추정치를 보정한다.

$$
\hat x_m(k+1)
=A_m\hat x_m(k)+B_mu(k)+K_{ob}(y(k)-C_m\hat x_m(k))
$$

여기서 $K_{ob}$는 observer gain이다. error dynamics는

$$
\tilde x_m(k+1)=(A_m-K_{ob}C_m)\tilde x_m(k)
$$

가 된다. 즉, $K_{ob}$를 통해 observer error의 pole 위치와 수렴 속도를 설계할 수 있다.

### 왜 $(A_m-K_{ob}C_m)^k$가 되는가

다음과 같이 observer error transition matrix를 하나의 기호로 두자.

$$
F_o=A_m-K_{ob}C_m
$$

그러면 error dynamics는

$$
\tilde x_m(k+1)=F_o\tilde x_m(k)
$$

가 된다. 초기 오차에서 한 sample 진행하면

$$
\tilde x_m(1)=F_o\tilde x_m(0)
$$

이고, 두 sample 진행하면 앞에서 구한 $\tilde x_m(1)$을 다시 대입하여

$$
\begin{aligned}
\tilde x_m(2)
&=F_o\tilde x_m(1)\\
&=F_o\left(F_o\tilde x_m(0)\right)\\
&=F_o^2\tilde x_m(0)
\end{aligned}
$$

를 얻는다. 같은 과정을 반복하면

$$
\boxed{
\tilde x_m(k)=F_o^k\tilde x_m(0)
=(A_m-K_{ob}C_m)^k\tilde x_m(0)
}
$$

가 된다. 여기서 $F_o^k$는 행렬의 각 원소를 $k$승한다는 뜻이 아니라

$$
F_o^k=\underbrace{F_oF_o\cdots F_o}_{k\text{번}}
$$

처럼 동일한 state transition을 $k$번 연속 적용한다는 뜻이다.

$F_o$가 diagonalizable하여 $F_o=V\Lambda V^{-1}$라고 하면

$$
F_o^k=V\Lambda^kV^{-1}
$$

이고

$$
\Lambda^k=
\operatorname{diag}(\lambda_1^k,\lambda_2^k,\ldots,\lambda_n^k)
$$

이다. 따라서 observer error의 각 mode가 $\lambda_i^k$로 변화하며, 모든 mode가 0으로 수렴하려면 $|\lambda_i(F_o)|<1$이어야 한다.

### Continuous-time observer와의 대응

Continuous-time plant와 observer를

$$
\dot x=A_cx+B_cu,
$$

$$
\dot{\hat x}
=A_c\hat x+B_cu
+L_c(y-C_c\hat x)
$$

라고 하자. $\tilde x=x-\hat x$를 미분하고 두 식을 빼면

$$
\begin{aligned}
\dot{\tilde x}
&=\dot x-\dot{\hat x}\\
&=(A_c-L_cC_c)\tilde x
\end{aligned}
$$

이므로 continuous-time에서도 observer error matrix는 $A_c-L_cC_c$ 형태로 얻어진다. 다만 미분방정식의 해는 행렬 거듭제곱이 아니라 matrix exponential이다.

$$
\boxed{
\tilde x(t)
=e^{(A_c-L_cC_c)t}\tilde x(0)
}
$$

두 경우를 비교하면 다음과 같다.

| 구분 | Error dynamics | 초기조건에 대한 해 |
|---|---|---|
| Continuous-time | $\dot{\tilde x}=F_c\tilde x$ | $\tilde x(t)=e^{F_ct}\tilde x(0)$ |
| Discrete-time | $\tilde x(k+1)=F_d\tilde x(k)$ | $\tilde x(k)=F_d^k\tilde x(0)$ |

Continuous-time error system을 sampling time $T_s$로 정확히 이산화하면

$$
F_d=e^{F_cT_s}
$$

이므로 $t=kT_s$에서

$$
F_d^k
=\left(e^{F_cT_s}\right)^k
=e^{F_ckT_s}
=e^{F_ct}
$$

가 된다. 즉, discrete-time의 $F_d^k$는 continuous-time의 $e^{F_ct}$에 대응한다.

> [!note] Gain 설계와 오차해는 다른 단계
> $(A_m-K_{ob}C_m)^k$는 $K_{ob}$를 계산하는 공식이 아니다. 먼저 pole placement 또는 Kalman filter로 $K_{ob}$를 설계하고, 그 결과 만들어진 error transition matrix가 초기 오차를 시간에 따라 어떻게 전파하는지를 나타낸 식이다. 또한 continuous-time gain $L_c$와 discrete-time gain $K_{ob}$는 일반적으로 같은 숫자가 아니므로 sampling model에 맞춰 설계해야 한다.

따라서 discrete-time Luenberger observer가 asymptotically stable하기 위한 조건은

$$
\boxed{
\rho(A_m-K_{ob}C_m)<1
}
$$

이다. 위의 open-loop predictor는 $K_{ob}=0$인 특별한 경우라서 $A_m$ 자체가 stable해야 오차가 수렴한다. 반면 $(A_m,C_m)$이 observable이면 plant가 unstable하더라도 $K_{ob}$를 설계하여 $A_m-K_{ob}C_m$의 eigenvalue를 unit circle 안으로 배치할 수 있다. Detectable한 경우에도 적어도 모든 unobservable mode가 unit circle 안에 있으면 안정한 observer를 구성할 수 있다.

## Observability

discrete-time linear system이 observable이려면 observability matrix

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

의 rank가 상태 차원 $n$과 같아야 한다.

관측 가능하지 않은 mode가 unstable하거나 unit circle 위에 있으면 observer로 그 상태를 안정적으로 복원할 수 없다. 관측 불가능 mode가 stable이면 observable은 아니어도 detectable일 수 있다.

## Example 1.7 - Pendulum observer

### 문제 설정

선형화된 단진자의 운동방정식은

$$
\ddot\theta+\omega^2\theta=u
$$

이다. 상태를

$$
x_1=\theta,\qquad x_2=\dot\theta
$$

로 정하면 continuous-time 모델은

$$
\begin{bmatrix}
\dot x_1\\
\dot x_2
\end{bmatrix}
=
\begin{bmatrix}
0&1\\
-\omega^2&0
\end{bmatrix}
\begin{bmatrix}
x_1\\x_2
\end{bmatrix}
+
\begin{bmatrix}
0\\1
\end{bmatrix}u
$$

이고 측정 가능한 출력은 각속도뿐이므로

$$
y=\dot\theta=x_2
=\begin{bmatrix}0&1\end{bmatrix}x
$$

이다. 목표는 $x_2$만 측정하여 측정하지 않는 각도 $x_1=\theta$까지 추정하는 것이다.

$\omega=2\ \mathrm{rad/s}$, $T_s=0.1\ \mathrm{s}$로 이산화하면

$$
A_m=
\begin{bmatrix}
0.9801&0.0993\\
-0.3973&0.9801
\end{bmatrix},\qquad
B_m=
\begin{bmatrix}
0.0050\\
0.09930
\end{bmatrix},\qquad
C_m=\begin{bmatrix}0&1\end{bmatrix}
$$

을 얻는다.

### 모델만 사용하면 왜 추정이 실패하는가

교재에서는 $u(k)=0$으로 두고 실제 초기상태와 추정 초기상태를 각각

$$
x(0)=
\begin{bmatrix}1\\0\end{bmatrix},\qquad
\hat x(0)=
\begin{bmatrix}0.3\\0\end{bmatrix}
$$

로 설정한다. 따라서 초기오차는

$$
\tilde x(0)=x(0)-\hat x(0)
=\begin{bmatrix}0.7\\0\end{bmatrix}
$$

이다. 이 pendulum에는 damping이 없어서 $A_m$의 pole은 unit circle 위에 있다. 따라서 open-loop prediction의 오차

$$
\tilde x(k)=A_m^k\tilde x(0)
$$

가 감쇠하지 않고 계속 진동한다. 모델이 정확하더라도 잘못 추측한 초기 각도 $0.3$이 실제 각도 $1$로 저절로 맞춰지지 않는 이유다.

### 각속도만으로 각도를 추정할 수 있는가

Observability matrix는

$$
\mathcal O=
\begin{bmatrix}
C_m\\C_mA_m
\end{bmatrix}
=
\begin{bmatrix}
0&1\\
-0.3973&0.9801
\end{bmatrix}
$$

이다. 행렬식은

$$
\det(\mathcal O)=0.3973\neq0
$$

이므로 rank는 2이고 시스템은 observable이다. 물리적으로는 현재 각도 $x_1$이 다음 순간의 각가속도와 각속도에 $-\omega^2x_1$을 통해 영향을 주기 때문에, 각속도의 시간 변화를 관찰하면 각도를 역으로 알아낼 수 있다는 뜻이다.

### Observer gain 계산

Observer gain을

$$
K_{ob}=
\begin{bmatrix}
j_1\\j_2
\end{bmatrix}
$$

라고 하면

$$
A_m-K_{ob}C_m
=
\begin{bmatrix}
0.9801&0.0993-j_1\\
-0.3973&0.9801-j_2
\end{bmatrix}
$$

이다. 따라서 observer error system의 characteristic polynomial은

$$
\begin{aligned}
&\det\left(\lambda I-(A_m-K_{ob}C_m)\right)\\
&=(\lambda-0.9801)(\lambda+j_2-0.9801)
-0.3973(j_1-0.0993).
\end{aligned}
$$

원하는 observer pole을 $0.1$, $0.2$로 선택했으므로 desired characteristic polynomial은

$$
(\lambda-0.1)(\lambda-0.2)
=\lambda^2-0.3\lambda+0.02
$$

이다. 두 characteristic polynomial의 같은 차수 계수를 일치시키면

$$
j_2-1.9602=-0.3
$$

에서 $j_2\approx1.6601$을 얻고, 상수항을 일치시키면

$$
0.9801(0.9801-j_2)
-0.3973(j_1-0.0993)=0.02
$$

에서 $j_1\approx-1.6284$를 얻는다. 따라서

$$
\boxed{
K_{ob}=
\begin{bmatrix}
-1.6284\\
1.6601
\end{bmatrix}
}
$$

이다. 소수점 반올림 때문에 직접 계산한 마지막 자리는 조금 달라질 수 있다.

### Observer를 적용하면

$u(k)=0$일 때 observer는

$$
\hat x(k+1)
=A_m\hat x(k)
+K_{ob}\left(x_2(k)-\hat x_2(k)\right)
$$

로 동작한다. 측정된 각속도와 예측 각속도의 차이를 두 상태 모두에 주입하며, $j_1$은 이 residual을 이용해 측정하지 않은 각도 추정값을 보정한다. 오차 pole이 $0.1$, $0.2$이므로

$$
\tilde x(k)=(A_m-K_{ob}C_m)^k\tilde x(0)
$$

의 각 mode가 빠르게 감쇠한다. 교재 Figure 1.4에서 open-loop 추정은 실제 궤적과 계속 어긋나지만, observer를 사용한 추정은 몇 sample 안에 실제 각도와 각속도에 겹쳐진다.

### Observer pole $0.1$, $0.2$는 어떻게 선택했는가

Observable한 system에서 원하는 observer pole을 배치할 수 있다는 것은 이론적 결과지만, 구체적으로 $0.1$, $0.2$를 선택한 것은 유일한 정답이 아니라 설계자의 선택이다. 교재에서는 빠른 수렴을 명확하게 보여주기 위해 unit circle의 중심에 가까운 pole을 사용했다.

Settling-time 기준으로 pole의 크기를 정할 때 사용하는 기호는 다음과 같다.

| 기호 | 이름 | 의미와 단위 |
|---|---|---|
| $T_s$ | sampling time 또는 sampling period | 측정값을 읽고 observer를 갱신하는 시간 간격, 단위는 s/sample |
| $T_{\mathrm{set}}$ | settling time | observer error가 허용 범위 안으로 들어오기를 원하는 실제 시간, 단위는 s |
| $\epsilon$ | 허용 잔류오차 비율 | settling 완료로 판단할 무차원 기준. $0.02$는 초기오차의 2%, $0.05$는 5% |
| $N$ | settling에 허용된 sample 수 | $N=\left\lceil T_{\mathrm{set}}/T_s\right\rceil$ |
| $r$ | dominant observer pole의 반지름 | $r=\max_i\lvert\lambda_i(A_m-K_{ob}C_m)\rvert$ |

Dominant error mode가 대략 $r^k$로 감소한다고 보면

$$
\frac{\lVert\tilde x(N)\rVert}{\lVert\tilde x(0)\rVert}
\lesssim r^N
$$

이다. $N$ sample 뒤 오차를 초기오차의 $\epsilon$ 이하로 만들고 싶다면

$$
r^N\le\epsilon
$$

이어야 하므로 pole 반지름을 대략

$$
\boxed{
r\le\epsilon^{1/N},
\qquad
N=\left\lceil\frac{T_{\mathrm{set}}}{T_s}\right\rceil
}
$$

로 선택할 수 있다. 이 식은 “무조건 pole을 0.1이나 0.2에 둔다”가 아니라 원하는 시간 성능으로부터 pole 범위를 정하는 방법이다.

#### 교재 pole의 settling time 해석

이 예제에서는

$$
T_s=0.1\ \mathrm{s},
\qquad
r=\max(0.1,0.2)=0.2
$$

이다. 2% criterion, 즉 $\epsilon=0.02$를 사용하면 필요한 sample 수는

$$
N\ge
\frac{\ln(0.02)}{\ln(0.2)}
\approx2.43
$$

이므로 정수 sample로는 $N=3$이다. 따라서 대략적인 settling time은

$$
T_{\mathrm{set}}\approx NT_s
=3\times0.1
=0.3\ \mathrm{s}
$$

이다. 실제로 dominant mode는

$$
0.2^1=0.2,
\qquad
0.2^2=0.04,
\qquad
0.2^3=0.008
$$

이어서 세 sample 뒤에는 초기 modal error의 약 $0.8\%$만 남는다.

#### Continuous-time pole과 비교

Discrete pole $z_i$를 동일한 감쇠율을 갖는 continuous pole $s_i$로 환산하면

$$
s_i=\frac{\ln z_i}{T_s}
$$

이다. 이 예제에서는

$$
z_1=0.1
\quad\Rightarrow\quad
s_1\approx-23.03\ \mathrm{s}^{-1}
$$

$$
z_2=0.2
\quad\Rightarrow\quad
s_2\approx-16.09\ \mathrm{s}^{-1}
$$

이므로 pendulum의 고유주파수 $2\ \mathrm{rad/s}$보다 훨씬 빠른 observer dynamics를 선택한 것이다. 일반적으로 continuous-time에서 원하는 pole을 먼저 정했다면

$$
z_i=e^{s_iT_s}
$$

로 discrete pole을 얻을 수 있다.

Complex conjugate pole을

$$
z_{1,2}=re^{\pm j\theta}
$$

로 선택한다면 $r$은 error envelope의 감쇠속도를, $\theta/T_s$는 error가 진동하는 angular frequency를 결정한다.

#### 빠른 pole의 trade-off

- pole을 원점에 가깝게 두면 초기 추정오차가 빠르게 사라진다.
- 너무 빠른 pole은 큰 observer gain을 요구하여 measurement noise를 크게 증폭할 수 있다.
- 모델 오차, sensor delay, discretization 오차에 더 민감해질 수 있다.
- pole을 unit circle에 가깝게 두면 noise에는 덜 민감할 수 있지만 추정오차가 오래 남는다.

따라서 실제 설계에서는 settling-time 요구조건으로 pole 반지름의 시작값을 정한 다음, noise와 모델 불확실성을 포함한 simulation으로 조정한다. Observer bandwidth를 controller bandwidth보다 몇 배 빠르게 두는 방법도 자주 쓰는 heuristic이지만, 센서 noise가 큰 시스템에는 그대로 적용하면 안 된다.

> [!note] Settling-time 식의 정확도
> $r^N$ 계산은 dominant eigenmode를 이용한 설계 근사다. Eigenvector의 condition, non-normal transient, repeated pole, noise 때문에 실제 state-error norm의 settling time은 달라질 수 있으므로 최종값은 simulation이나 실험으로 확인해야 한다.

> [!summary] Example 1.7의 핵심
> 각도를 직접 측정하지 않아도 각도가 각속도 dynamics에 영향을 주므로 system은 observable하다. 따라서 $K_{ob}$로 error pole을 unit circle 안의 원하는 위치에 배치하여 초기 추정오차를 제거할 수 있다.

## Example 1.8 - DC motor와 observability

### 문제 설정

DC motor를 기계적 시정수 $1/(s+1)$과 위치 적분기 $1/s$의 직렬 연결로 모델링한다. $x_1$을 shaft angular velocity, $x_2$를 shaft angular position으로 두면

$$
\begin{bmatrix}
\dot x_1\\
\dot x_2
\end{bmatrix}
=
\begin{bmatrix}
-1&0\\
1&0
\end{bmatrix}
\begin{bmatrix}
x_1\\x_2
\end{bmatrix}
+
\begin{bmatrix}
1\\0
\end{bmatrix}u
$$

이다. 여기서는 위치가 아니라 회전속도만 측정한다.

$$
y=x_1=\begin{bmatrix}1&0\end{bmatrix}x
$$

$T_s=0.1\ \mathrm{s}$로 이산화한 모델은

$$
A_m=
\begin{bmatrix}
0.9048&0\\
0.0952&1
\end{bmatrix},\qquad
B_m=
\begin{bmatrix}
0.0952\\
0.0048
\end{bmatrix},\qquad
C_m=\begin{bmatrix}1&0\end{bmatrix}
$$

이다. Open-loop pole은 $0.9048$과 $1$이다. $\lambda=1$은 위치가 속도를 적분하여 누적되는 mode다.

### 왜 속도만으로 위치를 알 수 없는가

Observability matrix는

$$
\mathcal O=
\begin{bmatrix}
C_m\\C_mA_m
\end{bmatrix}
=
\begin{bmatrix}
1&0\\
0.9048&0
\end{bmatrix}
$$

이고

$$
\det(\mathcal O)=0,
\qquad
\operatorname{rank}(\mathcal O)=1<2
$$

이므로 system은 observable하지 않다. $\mathcal O$의 두 번째 열이 모두 0이라는 것은 초기 위치 $x_2(0)$가 현재와 미래의 속도 출력에 전혀 나타나지 않는다는 뜻이다.

물리적으로 초기 위치가 서로 다른 두 motor가 같은 속도로 움직이면 속도 측정 기록은 완전히 동일하다. 속도를 적분하면 위치 변화량은 계산할 수 있지만, 알 수 없는 초기 위치 offset까지 복원할 수는 없다.

### Gain을 어떻게 골라도 pole 1이 남는 이유

Observer gain을

$$
K_{ob}=
\begin{bmatrix}
j_1\\j_2
\end{bmatrix}
$$

라고 하면

$$
A_m-K_{ob}C_m
=
\begin{bmatrix}
0.9048-j_1&0\\
0.0952-j_2&1
\end{bmatrix}
$$

이다. 이 행렬은 lower triangular이므로 characteristic polynomial은 대각성분만으로

$$
\boxed{
\det\left(\lambda I-(A_m-K_{ob}C_m)\right)
=(\lambda-0.9048+j_1)(\lambda-1)
}
$$

이 된다.

- $j_1$은 첫 번째 pole $0.9048-j_1$을 이동시킬 수 있다.
- $j_2$는 비대각성분에만 들어가므로 eigenvalue를 바꾸지 못한다.
- 두 번째 pole $\lambda=1$은 어떤 $j_1,j_2$를 선택해도 그대로 남는다.

예를 들어 첫 pole을 $0.1$로 옮기려면

$$
0.9048-j_1=0.1
\quad\Rightarrow\quad
j_1=0.8048
$$

로 둘 수 있지만, observer pole은 여전히 $0.1$과 $1$이다. 따라서 원하는 pole $0.1$, $0.2$를 동시에 만드는 것은 불가능하다.

### Observable과 detectable의 차이

관측 불가능한 mode가 unit circle 안에 있다면 그 mode를 직접 추정하지 못하더라도 오차가 스스로 감소하므로 system은 detectable할 수 있다. 그러나 이 예제의 관측 불가능한 pole은

$$
\lambda=1
$$

로 unit circle 위에 있어 오차가 감소하지 않는다. 따라서 이 system은 observable하지 않을 뿐 아니라 detectable하지도 않다. 속도만 측정해서는 shaft position의 초기 offset을 점근적으로 복원할 수 없다.

위치를 측정하도록 $C_m=[0\;1]$로 바꾸면 observability matrix는

$$
\mathcal O_{\mathrm{pos}}=
\begin{bmatrix}
0&1\\
0.0952&1
\end{bmatrix}
$$

가 되고 determinant가 $-0.0952$로 0이 아니므로 두 상태를 모두 관측할 수 있다. 즉, 센서가 무엇을 측정하는지가 observer 설계 가능성을 결정한다.

> [!summary] Example 1.8의 핵심
> 위치는 속도에 영향을 주지 않으므로 속도 출력에는 초기 위치 정보가 없다. 이 구조적 정보 손실 때문에 위치 mode $\lambda=1$은 observer gain으로 이동시킬 수 없다.

## Kalman filter

Pole placement는 원하는 observer pole을 직접 지정하지만, measurement noise와 process disturbance를 얼마나 믿어야 하는지는 알려주지 않는다. Kalman filter는 noise의 covariance를 이용해 **추정오차 covariance를 최소화하는 observer gain**을 계산한다. 교재는 multi-output observer를 동기로 소개하지만 Kalman filter는 SISO와 MIMO 모두에 사용할 수 있다.

### Stochastic model과 두 noise의 의미

Noise가 있는 discrete-time plant를

$$
x_m(k+1)=A_mx_m(k)+B_mu(k)+d(k)
$$

$$
y(k)=C_mx_m(k)+\xi(k)
$$

로 둔다. 여기서 두 noise의 역할은 다르다.

| 기호 | 위치 | 의미 | 대표적인 예 |
|---|---|---|---|
| $d(k)$ | state equation | process noise 또는 모델이 설명하지 못한 상태 변화 | 외란, 부하 토크, 미모델링 dynamics, 파라미터 오차의 영향 |
| $\xi(k)$ | output equation | measurement noise | sensor noise, ADC noise, encoder quantization |

두 번째 기호는 zeta가 아니라 xi $\xi$(크사이)다. 엄밀히 말하면 $d(k)$가 모델 파라미터 오차 자체인 것은 아니며, 모델 불일치와 외란이 다음 상태에 미치는 영향을 additive random variable로 근사한 것이다.

교재에서는 두 noise를 zero-mean white noise로 가정한다.

$$
\mathbb E[d(k)]=0,
\qquad
\mathbb E[\xi(k)]=0
$$

$$
\mathbb E[d(k)d(\tau)^T]
=\Theta\delta(k-\tau)
$$

$$
\mathbb E[\xi(k)\xi(\tau)^T]
=\Gamma\delta(k-\tau)
$$

여기서

$$
\delta(k-\tau)=
\begin{cases}
1,&k=\tau\\
0,&k\neq\tau
\end{cases}
$$

이므로 서로 다른 시각의 noise는 상관되지 않는다고 가정한다. 또한 일반적으로 process noise와 measurement noise 사이도

$$
\mathbb E[d(k)\xi(\tau)^T]=0
$$

으로 가정한다.

#### 확률 기호를 처음 보는 경우

먼저 noise가 vector가 아니라 숫자 하나인 scalar라고 생각하면 각 식의 의미가 분명해진다.

##### $\mathbb E[\cdot]$는 여러 번 실험했을 때의 평균

$$
\mathbb E[d(k)]=0
$$

은 $d(k)$가 매번 0이라는 뜻이 아니다. 같은 조건에서 실험을 여러 번 반복했을 때 $k$번째 noise가

$$
0.2,\quad-0.1,\quad0.3,\quad-0.4,\ldots
$$

처럼 매번 다르게 발생하고, 그 값들의 이론적인 평균이 0이라는 뜻이다.

$$
\boxed{
\text{zero mean}
=\text{noise가 장기적으로 한쪽 방향에 치우치지 않음}
}
$$

따라서 개별 sample의 noise는 0이 아닐 수 있다. 센서 출력에 항상 $+0.2$가 붙는 constant bias는 평균이 0이 아니므로 zero-mean noise가 아니다.

##### 같은 시각의 noise를 곱하면 variance

Scalar noise에서 $k=\tau$이면

$$
\mathbb E[d(k)^2]=\Theta
$$

가 된다. 평균이 0일 때 $\mathbb E[d(k)^2]$는 noise의 variance이며, noise가 평균 주변에 얼마나 크게 퍼지는지를 나타낸다. 예를 들어

$$
\Theta=0.04
$$

라면 standard deviation은

$$
\sigma_d=\sqrt{\Theta}=0.2
$$

이다. $\Theta$는 noise의 진폭 자체가 아니라 **noise 크기의 제곱에 대한 평균**이라는 점이 중요하다.

##### 서로 다른 시각의 noise를 곱하는 이유

$$
\mathbb E[d(k)d(\tau)]
$$

는 $k$번째 noise와 $\tau$번째 noise가 함께 움직이는지를 나타낸다. 현재 noise가 양수일 때 다음 noise도 양수가 되는 경향이 있다면 두 시각의 noise는 correlated되어 있다.

반대로 현재 noise를 알아도 다음 noise를 예측하는 데 도움이 되지 않는다고 가정하면

$$
\mathbb E[d(k)d(\tau)]=0,
\qquad k\neq\tau
$$

로 둔다. 평균이 0이므로 이 곱의 평균은 두 시각 사이의 covariance와 같다.

##### $\delta(k-\tau)$는 두 경우를 합쳐 쓰는 switch

Kronecker delta

$$
\delta(k-\tau)=
\begin{cases}
1,&k=\tau\\
0,&k\neq\tau
\end{cases}
$$

를 사용하면

$$
\mathbb E[d(k)d(\tau)]
=\Theta\delta(k-\tau)
$$

라는 한 식이 다음 두 문장을 동시에 표현한다.

$$
k=\tau
\quad\Rightarrow\quad
\mathbb E[d(k)^2]=\Theta
$$

$$
k\neq\tau
\quad\Rightarrow\quad
\mathbb E[d(k)d(\tau)]=0
$$

즉, 각 시각에서 noise의 variance는 $\Theta$이고 서로 다른 시각의 noise는 uncorrelated하다는 뜻이다. 여기서 $k,\tau$는 연속적인 시간이 아니라 discrete sample index다.

##### 이것이 white noise라는 의미

이 문맥에서 white noise는 다음 성질을 가진다.

- 평균이 0이라 한쪽 방향으로 치우치지 않는다.
- 서로 다른 sample의 noise가 시간적으로 uncorrelated하다.
- 이전 noise 값만으로 다음 noise 값을 선형적으로 예측할 수 없다.

“White”라는 이름은 모든 주파수 대역에 power가 고르게 분포한다는 frequency-domain 성질에서 유래한다.

##### Vector에서는 왜 transpose $T$가 붙는가

State noise가 두 성분을 가진 vector라고 하자.

$$
d(k)=
\begin{bmatrix}
d_1(k)\\d_2(k)
\end{bmatrix}
$$

그러면 column vector와 row vector의 outer product는

$$
d(k)d(k)^T
=
\begin{bmatrix}
d_1(k)^2&d_1(k)d_2(k)\\
d_2(k)d_1(k)&d_2(k)^2
\end{bmatrix}
$$

가 된다. 이 행렬에 expectation을 취한 것이 covariance matrix $\Theta$다.

$$
\Theta=
\begin{bmatrix}
\operatorname{Var}(d_1)&\operatorname{Cov}(d_1,d_2)\\
\operatorname{Cov}(d_2,d_1)&\operatorname{Var}(d_2)
\end{bmatrix}
$$

- 대각원소는 각 noise 성분의 variance다.
- 비대각원소는 같은 시각에 두 noise 성분이 함께 움직이는 정도다.
- Transpose를 붙여야 $n\times1$ vector 두 개로 $n\times n$ covariance matrix를 만들 수 있다.

Measurement noise도 완전히 같은 방식으로 $\Gamma$를 만든다.

$$
\mathbb E[\xi(k)\xi(\tau)^T]
=\Gamma\delta(k-\tau)
$$

마지막으로

$$
\mathbb E[d(k)\xi(\tau)^T]=0
$$

은 plant 쪽 process noise와 sensor 쪽 measurement noise가 uncorrelated하다고 가정한 것이다. Uncorrelated와 statistically independent는 일반적으로 같은 조건이 아니지만, 두 noise가 jointly Gaussian이면 uncorrelated로부터 independence가 따라온다.

> [!summary] 세 식의 직관
> $\mathbb E[d(k)]=0$은 평균적인 치우침이 없다는 뜻이고, $\mathbb E[d(k)^2]=\Theta$는 한 sample에서의 noise 크기를, $\mathbb E[d(k)d(\tau)]=0$ for $k\neq\tau$는 시간적인 기억이 없다는 뜻이다.

> [!note] White noise 가정의 범위
> 일정한 sensor bias나 지속적인 외란은 zero-mean white noise가 아니다. 이런 오차를 추정하려면 bias 또는 disturbance를 별도 state로 추가하는 augmented model이 필요하다.

### Covariance 행렬의 의미와 차원

State estimation error를

$$
\tilde x_m(k)=x_m(k)-\hat x_m(k)
$$

라고 하면 error covariance는

$$
P(k)=\mathbb E\left[\tilde x_m(k)\tilde x_m(k)^T\right]
$$

이다. $P(k)$의 대각원소는 각 state error의 variance이고, 비대각원소는 state error 사이의 covariance다. 교재의 초기조건은

$$
P(0)=\mathbb E\left[
(x_m(0)-\hat x_m(0))(x_m(0)-\hat x_m(0))^T
\right]
$$

이다. 초기상태를 확실히 알면 $P(0)$을 작게, 초기 추정이 불확실하면 크게 둔다.

| 행렬 또는 벡터 | 차원 | 의미 |
|---|---:|---|
| $d(k)$ | $n\times1$ | process noise |
| $\xi(k)$ | $q\times1$ | measurement noise |
| $\Theta$ | $n\times n$ | process-noise covariance |
| $\Gamma$ | $q\times q$ | measurement-noise covariance |
| $P(k)$ | $n\times n$ | state-estimation error covariance |
| $K_{ob}(k)$ | $n\times q$ | observer gain |

$\Theta$와 $P$는 positive semidefinite이고, innovation covariance의 역행렬을 안정적으로 계산하려면 보통 $\Gamma$는 positive definite로 둔다.

### Innovation은 새 측정값이 주는 정보

현재 측정값과 예측 출력의 차이를 innovation 또는 residual이라고 한다.

$$
\nu(k)=y(k)-C_m\hat x_m(k)
$$

교재의 predictor form에서 $\hat x_m(k)$는 현재 측정 $y(k)$를 반영하기 전의 one-step prior estimate $\hat x_m(k\mid k-1)$에 해당하고, $P(k)$도 prior error covariance $P(k\mid k-1)$에 해당한다. Innovation covariance는

$$
S(k)=\Gamma+C_mP(k)C_m^T
$$

이다. 첫 항은 sensor 자체의 불확실성이고, 둘째 항은 state prediction의 불확실성이 output으로 투영된 값이다. Kalman gain은 이 두 불확실성의 상대적 크기를 이용해 innovation을 state correction에 얼마나 반영할지 결정한다.

### Observer error covariance에서 gain 유도

교재에서 사용하는 predictor-form observer는

$$
\hat x_m(k+1)
=A_m\hat x_m(k)+B_mu(k)
+K_{ob}(k)\nu(k)
$$

이다. 여기서 바로 plant 식을 빼기 전에, 먼저 innovation $\nu(k)$에 측정식을 대입해 두자. 측정식과 추정 오차의 정의는

$$
y(k)=C_mx_m(k)+\xi(k),
\qquad
\tilde x_m(k)=x_m(k)-\hat x_m(k)
$$

이므로

$$
\begin{aligned}
\nu(k)
&=y(k)-C_m\hat x_m(k)\\
&=C_mx_m(k)+\xi(k)-C_m\hat x_m(k)\\
&=C_m\bigl(x_m(k)-\hat x_m(k)\bigr)+\xi(k)\\
&=C_m\tilde x_m(k)+\xi(k)
\end{aligned}
$$

가 된다. 즉 innovation에는 **현재 추정 오차가 출력에 나타난 부분** $C_m\tilde x_m(k)$와 **측정 잡음** $\xi(k)$가 함께 들어 있다. 이를 observer 식에 먼저 대입하면

$$
\hat x_m(k+1)
=A_m\hat x_m(k)+B_mu(k)
+K_{ob}(k)\bigl(C_m\tilde x_m(k)+\xi(k)\bigr)
$$

이다. 이제 plant 식에서 이 observer 식을 뺀다. 아래 전개에서는 식을 간단히 쓰기 위해 $K_{ob}=K_{ob}(k)$로 표기한다.

$$
\begin{aligned}
\tilde x_m(k+1)
&=x_m(k+1)-\hat x_m(k+1)\\
&=\bigl(A_mx_m(k)+B_mu(k)+d(k)\bigr)\\
&\quad-\bigl(A_m\hat x_m(k)+B_mu(k)
+K_{ob}(C_m\tilde x_m(k)+\xi(k))\bigr)\\
&=A_m\bigl(x_m(k)-\hat x_m(k)\bigr)
+d(k)-K_{ob}C_m\tilde x_m(k)-K_{ob}\xi(k)\\
&=A_m\tilde x_m(k)-K_{ob}C_m\tilde x_m(k)
+d(k)-K_{ob}\xi(k)\\
&=(A_m-K_{ob}C_m)\tilde x_m(k)
+d(k)-K_{ob}\xi(k).
\end{aligned}
$$

따라서

$$
\boxed{
\tilde x_m(k+1)
=(A_m-K_{ob}C_m)\tilde x_m(k)
+d(k)-K_{ob}\xi(k)
}
$$

를 얻는다. 여기서 $(A_m-K_{ob}C_m)\tilde x_m(k)$는 기존 추정 오차의 전파, $d(k)$는 process noise에 의해 새로 생긴 오차, $-K_{ob}\xi(k)$는 측정값을 보정에 사용하면서 유입된 measurement noise를 뜻한다. $K_{ob}$를 크게 하면 기존 오차를 강하게 보정하지만 측정 잡음도 더 강하게 받아들이므로, Kalman gain은 이 둘의 균형을 잡는다.

Noise들이 서로 독립이고 평균이 0이라는 가정을 사용하면 다음 시각의 error covariance는

$$
\begin{aligned}
P(k+1)
={}&(A_m-K_{ob}C_m)P(k)(A_m-K_{ob}C_m)^T\\
&+\Theta+K_{ob}\Gamma K_{ob}^T
\end{aligned}
$$

이다. 이 식을 전개하면

$$
\begin{aligned}
P(k+1)
={}&A_mPA_m^T
-A_mPC_m^TK_{ob}^T
-K_{ob}C_mPA_m^T\\
&+K_{ob}(C_mPC_m^T+\Gamma)K_{ob}^T
+\Theta
\end{aligned}
$$

가 된다. 여기서는 가독성을 위해 $P=P(k)$로 썼다. $\operatorname{tr}(P(k+1))$, 즉 전체 state error variance의 합을 $K_{ob}$에 대해 최소화하면

$$
-2A_mPC_m^T
+2K_{ob}(C_mPC_m^T+\Gamma)=0
$$

이고, 따라서 교재 식 (1.64)의 optimal gain을 얻는다.

$$
\boxed{
K_{ob}(k)
=A_mP(k)C_m^T
(\Gamma+C_mP(k)C_m^T)^{-1}
}
$$

이 gain을 covariance 식에 대입하면 교재 식 (1.65)의 Riccati recursion이 된다.

$$
\boxed{
\begin{aligned}
P(k+1)=A_m\{P(k)
&-P(k)C_m^T
(\Gamma+C_mP(k)C_m^T)^{-1}\\
&\qquad C_mP(k)\}A_m^T+\Theta
\end{aligned}
}
$$

> [!important] Kalman optimal의 의미
> 위 gain은 주어진 linear model과 noise covariance 가정 아래에서 mean-square estimation error를 최소화한다. Noise가 Gaussian이면 conditional mean을 주는 MMSE estimator이고, Gaussian이 아니더라도 주어진 second-order statistics 아래에서 optimal linear estimator다.

### 왜 교재 gain에는 $A_m$이 앞에 붙는가

많은 교재나 software에서는 measurement correction과 time prediction을 두 단계로 분리한다. 현재 측정값을 반영하는 measurement-update gain을

$$
L(k)=P(k)C_m^TS(k)^{-1}
$$

라고 하면 measurement update는

$$
\hat x_m(k\mid k)
=\hat x_m(k\mid k-1)
+L(k)\left[y(k)-C_m\hat x_m(k\mid k-1)\right]
$$

이고, measurement를 반영한 posterior covariance는

$$
P(k\mid k)
=P(k)-P(k)C_m^TS(k)^{-1}C_mP(k)
$$

이다. 이어서 time update는

$$
\hat x_m(k+1\mid k)
=A_m\hat x_m(k\mid k)+B_mu(k)
$$

$$
P(k+1\mid k)
=A_mP(k\mid k)A_m^T+\Theta
$$

이다. 두 covariance 단계를 합치면 교재의 Riccati recursion이 되고, 두 state-estimate 단계를 한 식으로 합치면 innovation 앞의 gain은

$$
K_{ob}(k)=A_mL(k)
$$

가 된다. 따라서 교재 식에 $A_m$이 붙어 있는 이유는 correction 결과를 현재 state가 아니라 **다음 state prediction에 바로 반영하는 predictor form**을 사용했기 때문이다.

> [!warning] Gain 표기 비교 시 주의
> 어떤 자료의 Kalman gain에 $A_m$이 없다고 해서 식이 틀린 것은 아니다. 그 gain이 current-state measurement update용 $L(k)$인지, next-state predictor용 $K_{ob}(k)=A_mL(k)$인지 observer equation과 시간 인덱스를 함께 확인해야 한다.

### 한 iteration의 실제 계산 순서

교재의 predictor form은 매 sample마다 다음 순서로 계산할 수 있다.

1. Innovation 계산

$$
\nu(k)=y(k)-C_m\hat x_m(k)
$$

2. Innovation covariance 계산

$$
S(k)=\Gamma+C_mP(k)C_m^T
$$

3. Observer gain 계산

$$
K_{ob}(k)=A_mP(k)C_m^TS(k)^{-1}
$$

4. 다음 state estimate 계산

$$
\hat x_m(k+1)
=A_m\hat x_m(k)+B_mu(k)+K_{ob}(k)\nu(k)
$$

5. 다음 error covariance 계산

$$
P(k+1)=A_m\left[P(k)-P(k)C_m^TS(k)^{-1}C_mP(k)\right]A_m^T+\Theta
$$

실제 수치 구현에서는 역행렬을 직접 만들기보다 linear solve를 사용하고, symmetry와 positive semidefiniteness를 더 잘 보존하기 위해 Joseph-form covariance update를 사용할 수 있다.

### $\Theta$와 $\Gamma$의 tuning 직관

| 설정 | Kalman filter의 해석 | 일반적인 결과 |
|---|---|---|
| $\Theta$ 증가 | 모델 예측을 덜 신뢰 | gain이 커지는 방향, 측정을 더 빠르게 추종 |
| $\Gamma$ 증가 | sensor 측정을 덜 신뢰 | gain이 작아지는 방향, 더 부드럽지만 느린 추정 |
| $P(0)$ 증가 | 초기 state estimate를 덜 신뢰 | 초기 correction이 커지는 방향 |

Scalar이고 $C_m=1$인 경우 measurement-update gain은

$$
L=\frac{P}{P+\Gamma}
$$

이므로 직관이 분명하다. $P$가 $\Gamma$보다 훨씬 크면 $L$이 1에 가까워져 측정을 많이 믿고, $\Gamma$가 훨씬 크면 $L$이 0에 가까워져 모델을 많이 믿는다. $\Theta$를 키우면 다음 단계의 $P$가 커지므로 이후 gain이 커지는 방향으로 작용한다.

### Steady-state Riccati equation

LTI system에서 $A_m,C_m,\Theta,\Gamma$가 일정하면 Riccati recursion은 조건이 만족될 때 일정한 행렬 $P(\infty)$로 수렴한다. Steady-state에서는 $P(k+1)=P(k)=P(\infty)$이므로

$$
\boxed{
\begin{aligned}
P(\infty)=A_m\{P(\infty)
&-P(\infty)C_m^T
(\Gamma+C_mP(\infty)C_m^T)^{-1}\\
&\qquad C_mP(\infty)\}A_m^T+\Theta
\end{aligned}
}
$$

를 얻는다. 이것이 discrete-time algebraic Riccati equation, DARE다. Steady-state observer gain은

$$
\boxed{
K_{ob}(\infty)
=A_mP(\infty)C_m^T
(\Gamma+C_mP(\infty)C_m^T)^{-1}
}
$$

이다.

교재가 제시한 수렴 조건은 다음과 같다.

- $(C_m,A_m)$이 detectable해야 한다. 즉, unobservable mode가 있다면 모두 unit circle 안에 있어야 한다.
- $(A_m,\Theta^{1/2})$가 stabilizable해야 한다.

이 조건 아래에서 stabilizing solution에 수렴하면

$$
\rho\left(A_m-K_{ob}(\infty)C_m\right)<1
$$

이 보장되어 steady-state observer error dynamics가 asymptotically stable하다.

### Offline 계산과 online 계산

교재의 LTI predictive-control 응용에서는 Riccati recursion을 offline에서 $P(k+1)\approx P(k)$가 될 때까지 반복하여 $K_{ob}(\infty)$를 저장한다. 실제 실시간 loop에서는 constant gain으로

$$
\hat x_m(k+1)
=A_m\hat x_m(k)+B_mu(k)
+K_{ob}(\infty)\left[y(k)-C_m\hat x_m(k)\right]
$$

만 계산하면 된다. 따라서 매 sample마다 Riccati equation을 다시 풀 필요가 없다.

반면 time-varying model, 시간에 따라 달라지는 noise covariance, finite-horizon filter, EKF/UKF와 같은 nonlinear filter에서는 covariance와 gain을 online으로 갱신할 수 있다.

> [!summary] Kalman filter의 핵심
> Model prediction과 sensor measurement 중 어느 쪽을 얼마나 믿을지를 $\Theta$, $\Gamma$, $P$로 정량화하고, 다음 state의 mean-square estimation error가 최소가 되도록 $K_{ob}$를 자동으로 선택하는 observer다.

## Observer dynamics tuning

> [!note] 기호 읽기
> 이 절의 $\Theta$는 **세타**, $\Gamma$는 **감마**라고 읽는다. $\Theta$는 앞의 MIMO MPC에서 사용한 prediction matrix $\Phi$(파이)와 다른 기호다.

### 왜 Kalman filter도 tuning이 필요한가

이론적으로 $\Theta$와 $\Gamma$는 실제 disturbance와 sensor noise의 covariance다. 그러나 실무에서는 외란의 통계 특성이 정확히 알려져 있지 않고, 모델 오차를 하나의 white process noise로 완벽하게 표현하기도 어렵다. 따라서 교재는 다음 세 행렬을 먼저 선택한 뒤 observer 응답을 보면서 수정하는 절차를 제안한다.

$$
\Theta=\operatorname{Cov}(d),
\qquad
\Gamma=\operatorname{Cov}(\xi),
\qquad
P(0)=\operatorname{Cov}\bigl(x(0)-\hat x(0)\bigr)
$$

- $\Theta$: plant model에 들어오는 process noise 또는 모델 불확실성
- $\Gamma$: sensor에서 발생하는 measurement noise
- $P(0)$: 초기 상태 추정값에 대한 불확실성

각 covariance matrix의 대각 원소는 각 변수 자체의 variance이고, 비대각 원소는 서로 다른 변수의 오차가 함께 움직이는 정도인 covariance다. 알려진 상관관계가 없다면 처음에는 대각행렬로 두는 경우가 많지만, covariance matrix가 positive semidefinite가 되도록 해야 하므로 비대각 원소를 임의로 크게 넣으면 안 된다.

선택한 $\Theta$, $\Gamma$, $P(0)$으로 Riccati equation을 반복한다.

$$
K_{ob}(i)
=A_mP(i)C_m^T
\left(\Gamma+C_mP(i)C_m^T\right)^{-1}
$$

$$
\begin{aligned}
P(i+1)
={}&A_m\left[P(i)-P(i)C_m^T
\left(\Gamma+C_mP(i)C_m^T\right)^{-1}
C_mP(i)\right]A_m^T+\Theta
\end{aligned}
$$

$P(i+1)\approx P(i)$가 되면 steady-state gain $K_{ob}(\infty)$를 얻는다. 그다음에는 계산이 끝났다고 보지 않고 다음 항목을 확인해야 한다.

1. $A_m-K_{ob}C_m$의 eigenvalue가 unit circle 안에 있는가?
2. 초기 estimation error가 원하는 시간 안에 감소하는가?
3. measurement noise가 $\hat x$에 지나치게 크게 나타나지 않는가?
4. 모델 오차와 parameter 변화가 있어도 observer가 안정적인가?

교재가 말하는 tuning은 이 결과를 확인하면서 covariance 원소를 수정하고 Riccati equation을 다시 푸는 trial-and-error 과정이다.

### $\Theta$, $\Gamma$, $P(0)$을 바꾸면 생기는 효과

#### $\Theta$를 크게 할 때

$\Theta$를 크게 둔다는 것은 **모델 예측 과정에 큰 불확실성이 계속 추가된다**고 보는 것이다. 그러면 $P$가 커지고 일반적으로 Kalman gain도 커져 측정값을 더 적극적으로 반영한다.

- 장점: 모델 오차나 외란을 빠르게 따라갈 수 있다.
- 단점: sensor noise가 추정 상태에 더 많이 전달되어 응답이 거칠어질 수 있다.

#### $\Gamma$를 크게 할 때

$\Gamma$를 크게 둔다는 것은 **센서 측정값을 덜 신뢰한다**는 뜻이다. Innovation covariance의 분모가 커지므로 일반적으로 Kalman gain은 작아진다.

- 장점: noisy measurement에 덜 민감하고 추정값이 매끄러워진다.
- 단점: 모델 의존도가 커지고 실제 상태 변화나 외란을 늦게 따라갈 수 있다.

#### $P(0)$을 크게 할 때

$P(0)$을 크게 두면 초기 추정값 $\hat x(0)$을 불확실하게 본다. 따라서 초기 Kalman gain이 커져 측정값을 이용해 초기 오차를 빠르게 수정하는 경향이 있다. 반대로 $P(0)$이 작으면 초기 추정값을 신뢰한다.

정상상태 Riccati solution이 유일하게 수렴하는 조건에서는 $P(0)$의 영향이 시간이 지나며 사라지므로, $P(0)$은 주로 **초기 transient**에 영향을 준다. 반면 $\Theta$와 $\Gamma$는 steady-state gain에도 계속 영향을 준다.

> [!tip] 절대 크기보다 상대적 크기
> $\Theta$와 $\Gamma$를 같은 비율로 동시에 확대하면 steady-state Kalman gain은 대체로 변하지 않고 covariance의 척도만 달라진다. Observer가 model과 measurement 중 무엇을 더 믿을지는 주로 $\Theta$와 $\Gamma$의 **상대적 크기와 방향별 분포**가 결정한다.

### 교재의 pole-radius tuning

공분산을 바꾸어 가며 원하는 observer 속도를 찾는 과정은 MIMO system에서 특히 번거롭다. 교재는 observer error pole이 미리 정한 반지름 $\alpha$의 원 안에 들어가도록 하는 간단한 방법을 제시한다.

먼저 noise가 없는 observer error dynamics는

$$
\tilde x_m(k+1)
=(A_m-K_{ob}C_m)\tilde x_m(k)
$$

이다. 이산시간에서 안정하려면 모든 eigenvalue가 unit circle 안에 있어야 한다. 교재는 안정 조건 $\lvert\lambda_i\rvert<1$보다 더 작은 영역

$$
\boxed{\lvert\lambda_i\rvert<\alpha},
\qquad 0<\alpha<1
$$

을 목표로 삼는다. 이를 위해 다음과 같이 scaling한 가상의 system을 만든다.

$$
\boxed{
\hat A_m=\frac{A_m}{\alpha},
\qquad
\hat C_m=\frac{C_m}{\alpha}
}
$$

여기서 행렬 위의 hat은 state estimate가 아니라 **pole-radius 설계를 위해 scaling한 matrix**라는 뜻이다. 이 행렬들을 사용하여 Riccati equation을 풀고 gain $\hat K_{ob}$를 구한다. Kalman filter의 안정성 조건이 만족되면 transformed observer matrix

$$
\hat A_m-\hat K_{ob}\hat C_m
$$

의 eigenvalue는 unit circle 안에 놓인다.

### 왜 원래 observer pole이 $\alpha$배가 되는가

Scaling 정의를 이용하면 원래 observer error matrix는

$$
\begin{aligned}
A_m-\hat K_{ob}C_m
&=\alpha\hat A_m-\hat K_{ob}\alpha\hat C_m\\
&=\alpha\left(\hat A_m-\hat K_{ob}\hat C_m\right)
\end{aligned}
$$

로 정리된다. 어떤 행렬 $M$의 모든 eigenvalue는 $\alpha M$에서 $\alpha$배가 되므로

$$
\lambda_i\left(A_m-\hat K_{ob}C_m\right)
=\alpha\lambda_i\left(\hat A_m-\hat K_{ob}\hat C_m\right)
$$

이다. Transformed pole이 $\lvert\lambda_i\rvert<1$이면 원래 observer pole은

$$
\left\lvert
\lambda_i\left(A_m-\hat K_{ob}C_m\right)
\right\rvert<\alpha
$$

를 만족한다. 따라서 $\hat K_{ob}$를 원래 observer에 적용하면 모든 error pole이 반지름 $\alpha$인 원 안에 들어간다. 교재의 characteristic equation도 같은 내용을 나타낸다.

$$
\begin{aligned}
&\det\left[zI-\left(A_m-\hat K_{ob}C_m\right)\right]\\
&=\det\left[zI-\alpha\left(\hat A_m-\hat K_{ob}\hat C_m\right)\right]=0
\end{aligned}
$$

### $\alpha$를 어떻게 선택하는가

- $\alpha$가 $1$에 가까움: 허용 pole 영역이 넓고 observer가 비교적 느리고 완만할 수 있다.
- $\alpha$가 작음: pole을 원점 가까이 제한하므로 nominal estimation error가 빠르게 감소한다.
- 지나치게 작은 $\alpha$: gain이 공격적으로 변하고 measurement noise amplification 및 모델 불확실성에 대한 민감도가 커질 수 있다.

Sampling time이 $T_s$이고, estimation error를 $T_{set}$초 안에 초기값의 $\epsilon$배 이하로 줄이고 싶다면 지배적인 pole 반지름 $r$에 대해 대략

$$
r^{T_{set}/T_s}\leq\epsilon
$$

이므로 설계 시작값을

$$
\boxed{
\alpha\approx\epsilon^{T_s/T_{set}}
}
$$

로 잡을 수 있다. 여기서

- $T_s$: sampling period
- $T_{set}$: 원하는 settling time
- $\epsilon$: settling이 끝났다고 판단할 허용 오차 비율

이다. 예를 들어 $T_s=0.1\ \mathrm{s}$, $T_{set}=2\ \mathrm{s}$, $\epsilon=0.02$라면

$$
\alpha\approx0.02^{0.1/2}\approx0.822
$$

를 초기 후보로 사용할 수 있다. 이 식은 dominant pole에 따른 envelope를 이용한 설계 지침이며, complex pole, repeated pole, non-normal matrix에서는 실제 transient를 simulation으로 반드시 확인해야 한다.

### 실무적인 반복 절차

1. Sensor 정지 데이터와 모델 오차 자료가 있으면 이를 이용해 $\Gamma$와 $\Theta$의 초기값을 정한다.
2. 초기 상태를 얼마나 알고 있는지에 따라 $P(0)$을 정한다.
3. 원하는 settling time으로부터 $\alpha$의 초기 후보를 정한다.
4. $\hat A_m=A_m/\alpha$, $\hat C_m=C_m/\alpha$로 Riccati equation을 수렴시켜 $\hat K_{ob}$를 구한다.
5. 원래 system의 $A_m-\hat K_{ob}C_m$ eigenvalue와 transient response를 확인한다.
6. 추정값이 너무 noisy하면 $\Gamma$ 또는 $\alpha$를 키우고, 외란 추종이 너무 느리면 $\Theta$를 키우거나 $\alpha$를 줄이는 방향으로 조정한다.
7. Measurement noise, process disturbance, 초기 오차, parameter variation을 함께 넣어 robustness를 확인한다.

> [!warning] 빠른 observer가 항상 좋은 것은 아니다
> 작은 $\alpha$는 nominal error convergence를 빠르게 만들지만 sensor noise까지 빠르게 추종할 수 있다. 최종 tuning은 pole 위치 하나만 보고 결정하지 않고, transient speed, noise amplification, actuator와 controller의 응답, model uncertainty를 함께 평가해야 한다.

## 연결

- 상태추정 MPC: [[07_State_Estimate_MPC|상태추정 기반 MPC]]
- MIMO 조건: [[05_MIMO_MPC|MIMO 예측제어]]
- 수식 모음: [[08_Key_Equations|핵심 수식 정리]]
