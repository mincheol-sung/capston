---
title: "05. MIMO 예측제어"
aliases:
  - "MIMO MPC"
  - "Predictive Control of MIMO Systems"
tags:
  - mpc
  - mimo
  - state-space
source_pages: "PDF p.49-54 / book p.22-27"
---

# MIMO 예측제어

SISO에서 만든 MPC 구조는 state-space formulation 덕분에 MIMO 시스템으로 자연스럽게 확장된다. 핵심 차이는 벡터와 행렬의 차원이다.

## Plant 모델

입력 $m$개, 출력 $q$개, 상태 $n_1$개를 가진 plant를 생각한다. 1장에서는 $q \leq m$을 가정한다. 출력 수가 입력 수보다 많으면 각 measured output을 독립적으로 zero steady-state error로 제어하기 어렵기 때문이다.

외란을 포함한 plant는 다음과 같다.

$$
x_m(k+1)=A_mx_m(k)+B_mu(k)+B_d\omega(k)
$$

$$
y(k)=C_mx_m(k)
$$

외란 $\omega(k)$는 integrated white noise로 가정한다.

$$
\omega(k)-\omega(k-1)=\epsilon(k)
$$

## MIMO augmented model

증분 상태와 출력을 묶어

$$
x(k)=
\begin{bmatrix}
\Delta x_m(k)^T & y(k)^T
\end{bmatrix}^T
$$

로 두면,

$$
\begin{bmatrix}
\Delta x_m(k+1) \\
y(k+1)
\end{bmatrix}
=
\begin{bmatrix}
A_m & 0 \\
C_mA_m & I_q
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
+
\begin{bmatrix}
B_d \\
C_mB_d
\end{bmatrix}
\epsilon(k)
$$

$$
y(k)=
\begin{bmatrix}
0_{q\times n_1} & I_q
\end{bmatrix}
x(k)
$$

간단히 쓰면,

$$
x(k+1)=Ax(k)+B\Delta u(k)+B_\epsilon\epsilon(k), \quad y(k)=Cx(k)
$$

이다.

## 추가되는 적분기 모드

augmented model의 characteristic polynomial은

$$
\rho(\lambda)=(\lambda-1)^q\det(\lambda I-A_m)
$$

로 정리된다. 따라서 plant eigenvalue에 더해 출력 수 $q$만큼 $\lambda=1$인 적분기 모드가 추가된다.

## Controllability와 observability 조건

augmented model은 integrator를 추가하므로 controllability와 observability를 반드시 확인해야 한다.

책의 핵심 정리는 다음이다.

- plant model $(A_m,B_m,C_m)$이 controllable/observable인 minimal realization이라고 하자.
- plant transfer function이 $G_m(z)=C_m(zI-A_m)^{-1}B_m$라고 하자.
- augmented transfer function은

$$
G(z)=\frac{z}{z-1}G_m(z)
$$

가 된다.

### 증강 전달함수 유도

증강 모델의 행렬은

$$
A=
\begin{bmatrix}
A_m & 0\\
C_mA_m & I_q
\end{bmatrix},\qquad
B=
\begin{bmatrix}
B_m\\
C_mB_m
\end{bmatrix},\qquad
C=\begin{bmatrix}0&I_q\end{bmatrix}
$$

이므로

$$
zI-A=
\begin{bmatrix}
zI-A_m & 0\\
-C_mA_m & (z-1)I_q
\end{bmatrix}
$$

이다. 블록 하삼각행렬에 대해

$$
\begin{bmatrix}
A_{11}&0\\
A_{21}&A_{22}
\end{bmatrix}^{-1}
=
\begin{bmatrix}
A_{11}^{-1}&0\\
-A_{22}^{-1}A_{21}A_{11}^{-1}&A_{22}^{-1}
\end{bmatrix}
$$

이므로 다음을 얻는다.

$$
(zI-A)^{-1}=
\begin{bmatrix}
(zI-A_m)^{-1}&0\\[2mm]
\dfrac{1}{z-1}C_mA_m(zI-A_m)^{-1}&\dfrac{1}{z-1}I_q
\end{bmatrix}
$$

> [!warning] 원문 식의 오탈자
> 교재에는 위 역행렬의 아래쪽 블록에 $(1-z^{-1})$가 인쇄되어 있지만, $(z-1)I_q$의 역행렬은 $(z-1)^{-1}I_q$이므로 올바른 표현은 $\dfrac{1}{z-1}$이다. 원문 표기대로 계산하면 식 (1.41)이 나오지 않는다.

$R=(zI-A_m)^{-1}$라고 두고 전달함수를 계산하면

$$
\begin{aligned}
G(z)
&=C(zI-A)^{-1}B\\
&=\frac{1}{z-1}
C_m\left[A_mR+I\right]B_m.
\end{aligned}
$$

여기서 $I=(zI-A_m)R$이므로

$$
A_mR+I
=A_mR+(zI-A_m)R
=zR
$$

이고, 최종적으로

$$
\boxed{
G(z)=\frac{z}{z-1}C_m(zI-A_m)^{-1}B_m
=\frac{z}{z-1}G_m(z)
}
$$

를 얻는다. 즉, 증강 과정에서 $z=1$인 적분기 pole이 추가된다.

augmented model이 controllable/observable이려면 $G_m(z)$가 $z=1$에서 zero를 가지면 안 된다. SISO에서는 $z=1$ zero가 있으면 $z/(z-1)$의 적분기 pole과 pole-zero cancellation이 생겨 integral control이 불가능해질 수 있다.

### MATLAB 예제: pole-zero cancellation과 minimal realization

다음 전달함수를 생각한다.

$$
G_m(z)=\frac{z-0.1}{(z-0.1)(z-0.9)}
$$

$z=0.1$에서 pole과 zero가 상쇄되므로 입출력 관점에서는

$$
G_m(z)=\frac{1}{z-0.9}
$$

와 같다. 하지만 상쇄하기 전 전달함수를 상태공간 모델로 변환하면 다음과 같은 2상태 실현을 얻을 수 있다.

$$
A_m=
\begin{bmatrix}
1&-0.09\\
1&0
\end{bmatrix},\qquad
B_m=
\begin{bmatrix}
1\\0
\end{bmatrix},\qquad
C_m=\begin{bmatrix}1&-0.1\end{bmatrix}
$$

이 실현의 controllability matrix와 observability matrix는

$$
\mathcal C=
\begin{bmatrix}B_m&A_mB_m\end{bmatrix}
=
\begin{bmatrix}
1&1\\
0&1
\end{bmatrix},
\qquad
\operatorname{rank}(\mathcal C)=2
$$

$$
\mathcal O=
\begin{bmatrix}
C_m\\C_mA_m
\end{bmatrix}
=
\begin{bmatrix}
1&-0.1\\
0.9&-0.09
\end{bmatrix},
\qquad
\operatorname{rank}(\mathcal O)=1
$$

이다. 즉, 두 상태는 모두 controllable이지만 한 상태가 unobservable이므로 이 실현은 minimal realization이 아니다. 상쇄된 $z=0.1$ 모드가 입출력 전달함수에 나타나지 않는 내부 모드다.

교재의 MATLAB 코드는 다음과 같다.

```matlab
numd = [1 -0.1];
dend = conv([1 -0.1], [1 -0.9]);
sys1 = tf(numd, dend);
sys = ss(sys1, 'min');
[Am, Bm, Cm, Dm] = ssdata(sys);
```

현재 MATLAB에서 교재의 2상태 비최소 실현과 rank 계산을 확실히 재현하려면, 교재에 제시된 행렬을 직접 구성한 뒤 `minreal`을 적용할 수 있다. 예시 샘플링 시간은 $T_s=1$로 두었다.

```matlab
Ts = 1;
Af = [1 -0.09; 1 0];
Bf = [1; 0];
Cf = [1 -0.1];
Df = 0;

sys_full = ss(Af, Bf, Cf, Df, Ts);
rank_ctrb = rank(ctrb(Af, Bf));
rank_obsv = rank(obsv(Af, Cf));

sys_min = minreal(sys_full);
[Am, Bm, Cm, Dm] = ssdata(sys_min);
```

실행 결과는 `rank_ctrb = 2`, `rank_obsv = 1`이며, `minreal`이 상태 하나를 제거하여 차수가 2에서 1로 감소한다. 참고로 `ss(tfModel)`은 MATLAB 버전에 따라 처음부터 최소 차수 실현을 반환할 수 있으므로, 비최소 2상태 모델의 rank를 확인하려는 목적에는 위처럼 행렬을 직접 넣는 편이 명확하다.

교재에 제시된 최소 실현 결과는

$$
A_m=0.9,\qquad B_m=-0.9806,\qquad C_m=-1.0198
$$

이며 상태가 하나뿐이다. 실제로 $C_mB_m\approx1$이므로

$$
C_m(zI-A_m)^{-1}B_m
\approx\frac{1}{z-0.9}
$$

가 되어 상쇄 후 전달함수와 일치한다. 상태공간 실현은 similarity transformation에 따라 $B_m,C_m$ 값이 다르게 나올 수 있으므로, 정확히 같은 숫자보다 **상태 차수가 2에서 1로 감소하고 전달함수가 보존되는지**를 확인하는 것이 중요하다.

> [!note] MATLAB 실행 결과의 숫자가 다른 경우
> 현재 설치된 MATLAB에서 위 코드를 실행하면 $A_m=0.9$, $B_m=-0.9950$, $C_m=-1.0050$이 얻어진다. 교재의 값과 다르지만 $C_mB_m\approx1$이고 전달함수는 동일하다. 이는 상태좌표 선택 차이이며 오류가 아니다.

관련 MATLAB 흐름은 [[09_MATLAB_Workflows#Minimal realization 확인|Minimal realization 확인]]을 참고한다.

## MIMO 예측과 최적화

MIMO에서도 SISO와 계산 원리는 같지만, 각 원소가 scalar가 아니라 vector 또는 block matrix가 된다. 증강 모델을 다시 쓰면

$$
x(k+1)=Ax(k)+B\Delta u(k)+B_{\epsilon}\epsilon(k),
\qquad y(k)=Cx(k)
$$

이다. 여기서 차원은

$$
A\in\mathbb R^{n\times n},\qquad
B\in\mathbb R^{n\times m},\qquad
C\in\mathbb R^{q\times n}
$$

이며 $m$은 입력 수, $q$는 출력 수, $n=n_1+q$는 증강 상태 수다.

### 미래 입력과 출력 벡터

현재 시각을 $k_i$라고 하고 control horizon을 $N_c$, prediction horizon을 $N_p$라고 하자. 앞으로 최적화할 입력 변화량을

$$
\Delta U=
\begin{bmatrix}
\Delta u(k_i)^T &
\Delta u(k_i+1)^T &
\cdots &
\Delta u(k_i+N_c-1)^T
\end{bmatrix}^T
$$

로 정의한다. 각 $\Delta u(k)$가 $m\times1$이므로

$$
\Delta U\in\mathbb R^{mN_c}
$$

이다. 예측 출력 벡터는

$$
Y=
\begin{bmatrix}
\hat y(k_i+1\mid k_i)^T &
\hat y(k_i+2\mid k_i)^T &
\cdots &
\hat y(k_i+N_p\mid k_i)^T
\end{bmatrix}^T
$$

이며, 각 출력이 $q\times1$이므로

$$
Y\in\mathbb R^{qN_p}
$$

이다. 기호 $\hat y(k_i+j\mid k_i)$는 시각 $k_i$에서 가진 정보를 이용해 $k_i+j$의 출력을 예측한다는 뜻이다.

### 상태 예측식을 한 단계씩 전개

표기를 간단히 하기 위해 $x_i=x(k_i)$, $\Delta u_j=\Delta u(k_i+j)$라고 하자. 첫 번째 미래 상태는

$$
x(k_i+1\mid k_i)
=Ax_i+B\Delta u_0+B_{\epsilon}\epsilon(k_i+1\mid k_i)
$$

이다. 여기서는 교재의 표기를 따라 $k_i+j-1$에서 $k_i+j$로 상태가 전이될 때 유입되는 잡음을 $\epsilon(k_i+j\mid k_i)$로 표시한다. 이는 앞의 모델식에서 사용한 잡음 인덱스를 미래 상태 시각에 맞춰 다시 붙인 것이다. 이 식을 다시 상태방정식에 대입하면 두 번째 미래 상태는

$$
\begin{aligned}
x(k_i+2\mid k_i)
={}&A^2x_i+AB\Delta u_0+B\Delta u_1\\
&+AB_{\epsilon}\epsilon(k_i+1\mid k_i)
+B_{\epsilon}\epsilon(k_i+2\mid k_i)
\end{aligned}
$$

가 된다. 같은 방식으로 반복하면 $j$단계 미래 상태는

$$
\begin{aligned}
x(k_i+j\mid k_i)
={}&A^jx_i
+\sum_{\ell=0}^{\min(j-1,N_c-1)}
A^{j-1-\ell}B\Delta u_{\ell}\\
&+\sum_{\ell=1}^{j}
A^{j-\ell}B_{\epsilon}\epsilon(k_i+\ell\mid k_i)
\end{aligned}
$$

로 쓸 수 있다. $j>N_c$일 때 합이 $N_c-1$에서 끝나는 이유는 control horizon 이후에는

$$
\Delta u(k_i+N_c)=\Delta u(k_i+N_c+1)=\cdots=0
$$

으로 두기 때문이다. 즉, 그 이후에는 입력 $u$를 마지막 값으로 유지한다.

### 미래 잡음이 예측식에서 사라지는 이유

$\epsilon(k)$를 평균이 0인 white noise라고 가정하면 현재 정보만으로 아직 발생하지 않은 잡음의 값을 알 수 없다. 따라서 조건부 평균은

$$
\mathbb E\left[\epsilon(k_i+j\mid k_i)\right]=0,
\qquad j\ge1
$$

이다. MPC가 사용하는 것은 실제 미래 출력 자체가 아니라 미래 출력의 조건부 기대값이므로, 위 상태 예측식에서 잡음 항의 기대값이 모두 0이 된다.

$$
\hat y(k_i+j\mid k_i)
=CA^jx_i
+\sum_{\ell=0}^{\min(j-1,N_c-1)}
CA^{j-1-\ell}B\Delta u_{\ell}
$$

잡음을 무시한다는 뜻은 아니다. 현재까지 측정된 잡음의 영향은 $x_i$ 또는 $\hat x_i$에 반영되고, 다음 샘플에서 새 측정값을 받으면 receding horizon 계산을 다시 수행한다.

### $F$와 $\Phi$ 행렬 구성

각 시점의 출력 예측식을 세로로 쌓으면

$$
\boxed{Y=Fx(k_i)+\Phi\Delta U}
$$

를 얻는다. 여기서 자유응답 행렬은

$$
F=
\begin{bmatrix}
CA\\
CA^2\\
CA^3\\
\vdots\\
CA^{N_p}
\end{bmatrix}
\in\mathbb R^{qN_p\times n}
$$

이고, 강제응답 행렬은

$$
\Phi=
\begin{bmatrix}
CB&0&0&\cdots&0\\
CAB&CB&0&\cdots&0\\
CA^2B&CAB&CB&\cdots&0\\
\vdots&\vdots&\vdots&\ddots&\vdots\\
CA^{N_p-1}B&CA^{N_p-2}B&CA^{N_p-3}B&\cdots&CA^{N_p-N_c}B
\end{bmatrix}
\in\mathbb R^{qN_p\times mN_c}
$$

이다. $\Phi$의 각 block은 $q\times m$이며, $(j,\ell)$ block은 $j\ge\ell$일 때 $CA^{j-\ell}B$, 그 외에는 0이다. 따라서 과거의 입력 변화는 이후의 모든 예측 출력에 영향을 주지만 미래의 입력 변화는 이전 출력에 영향을 줄 수 없는 lower block-triangular 구조가 된다.

예를 들어 $N_p=3$, $N_c=2$이면

$$
\begin{bmatrix}
\hat y_1\\
\hat y_2\\
\hat y_3
\end{bmatrix}
=
\begin{bmatrix}
CA\\CA^2\\CA^3
\end{bmatrix}x_i
+
\begin{bmatrix}
CB&0\\
CAB&CB\\
CA^2B&CAB
\end{bmatrix}
\begin{bmatrix}
\Delta u_0\\
\Delta u_1
\end{bmatrix}.
$$

### Reference vector와 비용함수

MIMO set-point는

$$
r(k_i)=
\begin{bmatrix}
r_1(k_i)&r_2(k_i)&\cdots&r_q(k_i)
\end{bmatrix}^T
\in\mathbb R^q
$$

이다. prediction horizon 동안 set-point가 일정하다고 가정하면

$$
R_s=\bar R_s r(k_i),
\qquad
\bar R_s=
\begin{bmatrix}
I_q\\I_q\\\vdots\\I_q
\end{bmatrix}
\in\mathbb R^{qN_p\times q}
$$

로 쓸 수 있다. 교재의 기본 비용함수는

$$
J=(R_s-Y)^T(R_s-Y)+\Delta U^T\bar R\Delta U
$$

이다. 첫 항은 출력 추종 오차를 줄이고, 둘째 항은 입력 변화가 지나치게 커지는 것을 막는다. 일반적인 MIMO 입력 가중치는

$$
\bar R=I_{N_c}\otimes R_u,
\qquad R_u\in\mathbb R^{m\times m}
$$

로 구성하며 $\bar R\in\mathbb R^{mN_c\times mN_c}$이다. $R_u$의 대각 원소를 다르게 주면 각 actuator의 움직임을 서로 다르게 제한할 수 있다.

> [!note] 출력 가중치의 일반화
> 교재 식은 출력 가중치를 identity로 둔 경우다. 출력별 중요도가 다르면 $J=(R_s-Y)^T\bar Q(R_s-Y)+\Delta U^T\bar R\Delta U$로 두고, 아래 식의 $\Phi^T\Phi$를 $\Phi^T\bar Q\Phi$로 바꾸면 된다.

### 최적 제어열 유도

$Y=Fx_i+\Phi\Delta U$를 비용함수에 대입하고

$$
e=R_s-Fx_i-\Phi\Delta U
$$

라고 두면

$$
J=e^Te+\Delta U^T\bar R\Delta U
$$

이다. $J$를 $\Delta U$에 대해 미분하면

$$
\frac{\partial J}{\partial\Delta U}
=-2\Phi^T(R_s-Fx_i-\Phi\Delta U)
+2\bar R\Delta U
$$

이고, 이를 0으로 두면

$$
(\Phi^T\Phi+\bar R)\Delta U
=\Phi^T(R_s-Fx_i)
$$

를 얻는다. 따라서 unconstrained MPC의 최적 제어열은

$$
\boxed{
\Delta U^*
=(\Phi^T\Phi+\bar R)^{-1}
(\Phi^T\bar R_s r(k_i)-\Phi^TFx(k_i))
}
$$

이다. 주요 행렬 차원은 다음과 같다.

| 행렬 | 차원 |
|---|---:|
| $F$ | $qN_p\times n$ |
| $\Phi$ | $qN_p\times mN_c$ |
| $\Phi^T\Phi$ | $mN_c\times mN_c$ |
| $\Phi^TF$ | $mN_c\times n$ |
| $\Phi^T\bar R_s$ | $mN_c\times q$ |
| $\bar R$ | $mN_c\times mN_c$ |

증강 상태 $x=[\Delta x_m^T\;y^T]^T$에서는 초기 출력 $y(k_i)$가 모든 미래 출력에 동일하게 전달되므로 $F$의 마지막 $q$개 열이 $\bar R_s$와 같다. 따라서 교재가 언급한 것처럼 $\Phi^T\bar R_s$는 $\Phi^TF$의 마지막 $q$개 열과 같다.

### 첫 번째 입력만 적용하는 이유

최적화 결과 $\Delta U^*$에는 앞으로 $N_c$번 사용할 입력 변화량이 모두 들어 있지만, 실제 plant에는 첫 번째 $m$개 원소만 적용한다. 선택행렬을

$$
S=\begin{bmatrix}I_m&0_m&\cdots&0_m\end{bmatrix}
\in\mathbb R^{m\times mN_c}
$$

라고 하면

$$
\Delta u(k_i)=S\Delta U^*
$$

이다. 이를 feedback 형태로 정리하면

$$
\Delta u(k_i)=K_yr(k_i)-K_{mpc}x(k_i)
$$

이며

$$
K_y=S(\Phi^T\Phi+\bar R)^{-1}\Phi^T\bar R_s,
\qquad
K_{mpc}=S(\Phi^T\Phi+\bar R)^{-1}\Phi^TF
$$

이다. 실제 입력은

$$
u(k_i)=u(k_i-1)+\Delta u(k_i)
$$

로 갱신한다. 다음 샘플에서는 새 상태 또는 상태추정값으로 전체 최적화를 다시 계산하므로, 이번에 계산한 두 번째 이후의 입력 변화량은 그대로 사용하지 않는다. 이것이 receding horizon 원리다.

> [!important] 이 해의 적용 범위
> 위의 역행렬 형태는 입력·출력 제약조건이 없는 unconstrained MPC의 폐형식 해다. actuator saturation, 입력 변화율, 출력 제한을 포함하면 같은 예측행렬을 사용하되 quadratic programming 문제로 풀어야 한다.

## 연결

- SISO 모델: [[02_Augmented_State_Space_Model|내장 적분기 상태공간 모델]]
- 폐루프 구조: [[04_Receding_Horizon_Control|Receding Horizon Control]]
- 상태추정: [[06_State_Estimation|상태추정과 관측기]]
