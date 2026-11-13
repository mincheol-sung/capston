---
title: "13. MATLAB 튜토리얼 - DC 모터의 닫힌형 제약 제어 (motor1.m / motor2.m)"
aliases:
  - "Tutorial 3.9"
  - "motor1.m"
  - "motor2.m"
  - "DC Motor Constrained Control"
tags:
  - mpc
  - matlab
  - laguerre
  - constraints
  - closed-form
  - dc-motor
source_pages: "PDF p.51-58 / book p.135-142"
---

# MATLAB 튜토리얼 - DC 모터의 닫힌형 제약 제어

[[12_Closed_Form_SISO|SISO 닫힌형 해]]를 DC 모터 위치 제어에 구현한다. 관측기를 쓰지 않도록 상태변수를 입출력의 과거값으로 고르는 것이 첫 번째 요점이고, 제약을 QP 없이 비교문으로 처리하는 것이 두 번째 요점이다. 계산 여력이 작은 마이크로컨트롤러를 염두에 둔 구현이다. 모터로 설명하지만 $n$ 차 SISO 시스템으로 바로 확장된다.

## 모델

DC 모터의 연속시간 전달함수는

$$
G(s)=\frac{K}{s(Ts+1)}
$$

이고, 입력은 전압, 출력은 각위치다. 영차홀드 이산화 모델은

$$
G(z)=\frac{b_1z+b_2}{z^2+a_1z+a_2}
$$

이다. 차분방정식으로 쓰면

$$
y(k+2)+a_1y(k+1)+a_2y(k)=b_1u(k+1)+b_2u(k)
$$

다.

### 관측기가 필요 없는 상태 선택

상태벡터를 측정 가능한 값들로 고른다.

$$
x_m(k)=
\begin{bmatrix}
y(k) & y(k-1) & u(k-1)
\end{bmatrix}^T
$$

$$
\begin{bmatrix}
y(k+1) \\ y(k) \\ u(k)
\end{bmatrix}
=
\begin{bmatrix}
-a_1 & -a_2 & b_2 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
y(k) \\ y(k-1) \\ u(k-1)
\end{bmatrix}
+
\begin{bmatrix}
b_1 \\ 0 \\ 1
\end{bmatrix}
u(k),
\qquad
y(k)=
\begin{bmatrix}
1 & 0 & 0
\end{bmatrix}
x_m(k)
$$

첫 행이 차분방정식 $y(k+1)=-a_1y(k)-a_2y(k-1)+b_1u(k)+b_2u(k-1)$ 그대로다. $u(k-1)$ 이 상태에 들어간 이유는 분자에 $b_2$ 항, 즉 한 스텝 지난 입력이 필요하기 때문이다. 2장 [[13_Output_Constraints|Example 2.20]]과 같은 선택이다.

증강 상태벡터는 $x(k)=[\Delta y(k)\ \ \Delta y(k-1)\ \ \Delta u(k-1)\ \ y(k)]^T$ 이고

$$
\begin{bmatrix}
\Delta y(k+1) \\ \Delta y(k) \\ \Delta u(k) \\ y(k+1)
\end{bmatrix}
=
\begin{bmatrix}
-a_1 & -a_2 & b_2 & 0 \\
1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
-a_1 & -a_2 & b_2 & 1
\end{bmatrix}
\begin{bmatrix}
\Delta y(k) \\ \Delta y(k-1) \\ \Delta u(k-1) \\ y(k)
\end{bmatrix}
+
\begin{bmatrix}
b_1 \\ 0 \\ 1 \\ b_1
\end{bmatrix}
\Delta u(k),
\qquad
y(k)=
\begin{bmatrix}
0 & 0 & 0 & 1
\end{bmatrix}
x(k)
$$

이다. $\Delta y(k)=y(k)-y(k-1)$, $\Delta u(k)=u(k)-u(k-1)$ 이다. 시각 $k_i$ 에서 플랜트 입출력의 현재값과 과거값만 있으면 $x(k_i)$ 가 그대로 알려지므로 **관측기가 필요 없다.** 설정치는 마지막 성분을 $y(k_i)-r(k_i)$ 로 바꿔 넣는다.

## Tutorial 3.9 설정

$T=1$, $K=1$ 로 정규화하고 샘플링 $h=0.1$, $Q=C^TC$, $R=0.1$ 이다. Laguerre 파라미터는 처음에 $a=0.5$, $N=1$ 로 두고, $N_p=46$ 이다. Part A는 입력 제약, Part B는 입력과 출력 제약을 다룬다.

$N=1$ 이라는 선택에 주목할 만하다. $N=1$ 이면 $\eta$ 가 스칼라이고 $\Omega$ 도 스칼라라서 `Omega\Psi` 가 나눗셈 한 번이다. 마이크로컨트롤러에서 이보다 가벼울 수 없다. 성능이 부족하면 $N$ 을 늘린다.

## Part A - $u$ 와 $\Delta u$ 에 대한 제약 (motor1.m)

### 1~2단계 - 이산화와 플랜트 상태공간 모델

```matlab
K=1;  % gain of the motor
T=1;  % time constant of the motor
h=0.1; % sampling interval
num=K;
den=conv([1 0],[T 1]);
[numd,dend]=c2dm(num,den,h);
Ap=zeros(3,3);
Ap(1,:)=[-dend(1,2:3) numd(1,3)];
Ap(2,:)=[1 0 0];
Ap(3,:)=[0 0 0];
Bp(1,1)=numd(1,2);
Bp(2,1)=0;
Bp(3,1)=1;
Cp(1,1)=1;
Cp(1,2)=0;
Cp(1,3)=0;
Dp(1,1)=0;
```

`c2dm` 이 돌려주는 `dend=[1 a1 a2]`, `numd=[0 b1 b2]` 에서 `Ap(1,:)=[-a1 -a2 b2]`, `Bp(1,1)=b1` 으로 위의 상태공간 모델을 그대로 채운다.

### 3단계 - 증강 모델

```matlab
n1=3;
m1=1;
n_in=1;
A=eye(n1+m1,n1+m1);
A(1:n1,1:n1)=Ap;
A(n1+1:n1+m1,1:n1)=Cp*Ap;
B=zeros(n1+m1,n_in);
B(1:n1,:)=Bp;
B(n1+1:n1+m1,:)=Cp*Bp;
C=zeros(m1,n1+m1);
C(:,n1+1:n1+m1)=eye(m1,m1);
```

[[07_MATLAB_dmpc|testexa.m]] 과 같은 조립이다. `Cp*Ap` 가 4행의 $[-a_1\ -a_2\ b_2]$ 를, `Cp*Bp` 가 $b_1$ 을 만든다.

### 4~5단계 - dmpc 호출과 폐루프 확인

```matlab
Q=C'*C;
R=0.3;
a=0.7;
N=1;  %you can increase N
Np=46;
[Omega,Psi]=dmpc(A,B,a,N,Np,Q,R);
[A1,L0]=lagd(a,N);
K_mpc=L0'*(Omega\Psi);
Acl=A-B*K_mpc;
Pole_close=eig(Acl);
```

교재 본문은 $R=0.1$, $a=0.5$ 라고 적었는데 코드에는 `R=0.3`, `a=0.7` 이 들어 있다. 원문의 불일치이므로 둘 중 하나로 통일해서 쓰면 된다. `Pole_close` 로 무제약 폐루프 극점이 단위원 안에 있는지 먼저 확인한다.

### 6단계 - 초기조건과 제약 한계

```matlab
xm=zeros(n1,1);
N_sim=170;
u=0;
y=0;
y_delta_k=0;
y_delta_k_m1=0;
u_delta_k_m1=0;
r=[ones(1,200) ones(1,100) ones(1,200)];
Xf=[y_delta_k; y_delta_k_m1; u_delta_k_m1; y-r(1,1)];
u_min=-0.3;
u_max=0.2;
deltau_min=-0.1;
deltau_max=0.1;
up=0.0;
```

`Xf` 가 증강 상태 $[\Delta y(k)\ \Delta y(k-1)\ \Delta u(k-1)\ y(k)-r(k)]^T$ 다. `up` 은 직전 제어 $u(k_i-1)$ 이다.

교재의 `r` 은 세 구간이 모두 1이라 사실상 상수 설정치이고 `N_sim=170` 인데, Fig. 3.14는 250 샘플 동안 설정치가 1에서 $-1$ 로 바뀌는 응답을 보여 준다. 그림을 재현하려면 `r=[ones(1,120) -ones(1,130)]` 정도로 부호를 바꾸고 `N_sim=250` 으로 늘려야 한다.

### 7단계 - 닫힌형 제약 제어 루프

```matlab
for kk=1:N_sim;
eta=-(Omega\Psi)*Xf;
deltau=L0'*eta;
if (deltau>deltau_max) deltau=deltau_max;end
if (deltau<deltau_min) deltau=deltau_min;end
u=up+deltau;
if (u>u_max) deltau=u_max-up; u=u_max; end
if (u<u_min) deltau=u_min-up; u=u_min; end
deltau1(1,kk)=deltau;   %save data
u1(1,kk)=u;     %save data
y1(1,kk)=y;     %save data
%plant simulation
yp=y;
xm_old=xm;
xm=Ap*xm+Bp*u;       % calculate xm(k+1)
y=Cp*xm;             %calculate y(k+1)
%updating feedback state variable Xf
y_delta_k_m1=y_delta_k;
y_delta_k=y-yp;
u_delta_k_m1=deltau;
Xf=[y_delta_k; y_delta_k_m1;u_delta_k_m1;y-r(1,kk+1)];
up=u;
end
```

루프의 앞 여섯 줄이 [[12_Closed_Form_SISO|닫힌형 해]] 전부다.

| 코드 | 닫힌형 해의 어느 경우 |
|---|---|
| `eta=-(Omega\Psi)*Xf; deltau=L0'*eta;` | 무제약 해 $\Delta u(k_i)=L(0)^T\eta$ |
| `if (deltau>deltau_max) deltau=deltau_max` | $\Delta u$ 상한 활성 → $\Delta u(k_i)=\Delta u^{max}$ |
| `if (deltau<deltau_min) deltau=deltau_min` | $\Delta u$ 하한 활성 → $\Delta u(k_i)=\Delta u^{min}$ |
| `u=up+deltau` | $u(k_i)=u(k_i-1)+\Delta u(k_i)$ |
| `if (u>u_max) deltau=u_max-up; u=u_max` | $u$ 상한 활성 → $\Delta u(k_i)=u^{max}-u(k_i-1)$ |
| `if (u<u_min) deltau=u_min-up; u=u_min` | $u$ 하한 활성 → $\Delta u(k_i)=u^{min}-u(k_i-1)$ |

순서가 $\Delta u$ 다음 $u$ 인 것은 [[12_Closed_Form_SISO|제약 순위]]대로다. $u$ 제약이 더 중요하므로 마지막에 처리해 최종적으로 지켜지게 한다. 그리고 $u$ 제약에서 고쳐진 `deltau` 가 `u_delta_k_m1=deltau` 로 다음 상태에 들어간다. 잘린 실제 증분을 모델에 알려주는 것이다.

`Xf` 갱신은 $\Delta y(k+1)=y(k+1)-y(k)$ (`y-yp`), 한 칸 밀린 $\Delta y(k)$, 방금 적용한 $\Delta u(k)$, 그리고 다음 설정치와의 오차 `y-r(1,kk+1)` 로 이루어진다. 관측기 방정식이 없다.

### 8~10단계 - 실시간 구현과 결과

실제 DC 모터에 구현하면 플랜트 시뮬레이션 네 줄이 사라지고, 제어신호 $u$ 는 전압으로 나가며 $y$ 는 각위치 측정값으로 들어온다. 마이크로컨트롤러에서는 데이터 저장도 생략할 수 있다.

```matlab
k=0:(N_sim-1);
figure(1)
subplot(211)
plot(k,y1,'b')
ylabel('y')
xlabel('Sampling Instant')
subplot(212)
plot(k,u1,'b')
ylabel('u')
xlabel('Sampling Instant')
figure(2)
plot(k,deltau1,'g')
ylabel('Delta u')
xlabel('Sampling Instant')
```

Fig. 3.14는 제약 유무를 비교한다. 제약이 걸리면 $u$ 와 $\Delta u$ 가 모두 한계 안에 머문다. 설정치 변화 직후 $\Delta u$ 가 $-0.1$ 하한과 $u$ 가 $-0.3$ 하한에 닿고, 그 대가로 $y$ 의 응답이 무제약보다 조금 느려진다. **이 닫힌형 해가 QP를 썼을 때와 동일한 결과를 낸다**는 것을 확인할 수 있다.

## Part B - 출력 제약 (motor2.m)

출력 제약의 닫힌형 해는 조금 복잡하다. 출력 제약은 폐루프 불안정과 성능 저하를 부를 수 있으므로 상대적으로 조심해서 도입해야 한다. 설정치는 0이고, 입력 계단 외란을 제거하는 문제로 시연한다. Part A 위에 짓는다.

### 1~2단계 - 외란, 제약, 활성 제약 행렬

`motor1.m` 을 `motor2.m` 으로 복사하고 폐루프 시뮬레이션 전에 다음을 넣는다.

```matlab
r=[ones(1,120) -zeros(1,120) ones(1,200)]*0;
d=[ones(1,120) -zeros(1,120) ones(1,200)];
Xf=[y_delta_k; y_delta_k_m1;u_delta_k_m1;y-r(1,1)];
u_min=-1.5;
u_max=1.5;
deltau_min=-0.4;
deltau_max=0.4;
y_min=-0.14;
y_max=0.14;
M_act=C*B*L0';
E=C*A;
y_bar_min=y_min-r(1,1);
y_bar_max=y_max-r(1,1);
```

`r` 은 끝에 `*0` 을 곱했으므로 전부 0이다 (조정기 문제). 입력 외란 `d` 는 처음 120 샘플 동안 1, 다음 120 샘플 동안 0, 그 뒤 다시 1이다. 외란이 켜지고 꺼질 때마다 출력이 흔들리고 제약이 활성이 된다.

`M_act=C*B*L0'` 는 [[12_Closed_Form_SISO|닫힌형 해]]의 $M_{act}=CBL(0)^T$ 이고, `E=C*A` 는 $CA$ 다. 한 스텝 뒤 출력 예측 $y(k_i+1\mid k_i)=CAx(k_i)+CBL(0)^T\eta$ 를 만들기 위한 두 조각이다. 증강 상태의 마지막 성분이 $y-r$ 이므로 이 예측은 사실 $(y-r)(k_i+1)$ 이고, 그래서 한계도 `y_bar_min=y_min-r` 로 설정치를 뺀 값을 쓴다.

### 3단계 - 출력 제약이 포함된 루프

```matlab
for kk=1:N_sim;
eta=-(Omega\Psi)*Xf;
deltau=L0'*eta;
beta=E*Xf+M_act*eta;
if (beta<y_bar_min)
 M_act1=-M_act;
lambda=-inv(M_act1*inv(Omega)*M_act1')*(-y_bar_min+E*Xf+M_act1*(-eta));
eta=-inv(Omega)*(Psi*Xf+M_act1'*lambda);
deltau=L0'*eta;
end
if (beta>y_bar_max)
lambda=-inv(M_act*inv(Omega)*M_act')*(y_bar_max-E*Xf+M_act*(-eta));
eta=-inv(Omega)*(Psi*Xf+M_act'*lambda);
deltau=L0'*eta; end
if(deltau>deltau_max) deltau=deltau_max;end
if (deltau<deltau_min) deltau=deltau_min;end
u=u+deltau;
if (u>u_max) deltau=u_max-up; u=u_max;end
if (u<u_min) deltau=u_min-up; u=u_min; end
 deltau1(1,kk)=deltau;
 u1(1,kk)=u;
 y1(1,kk)=y;
%plant simulation
yp=y;
xm_old=xm;
xm=Ap*xm+Bp*(u+d(kk));      % calculate xm(k+1)
y=Cp*xm;             %calculate y(k+1)
%updating feedback state variable Xf
y_delta_k_m1=y_delta_k;
y_delta_k=y-yp;
u_delta_k_m1=deltau;
Xf=[y_delta_k; y_delta_k_m1;u_delta_k_m1;y-r(1,kk+1)];
y_bar_min=y_min-r(1,kk+1);
y_bar_max=y_max-r(1,kk+1);
up=u;
end
```

Part A 루프에 출력 제약 블록이 앞에 붙었다.

| 코드 | 수식 |
|---|---|
| `beta=E*Xf+M_act*eta` | 무제약 해로 예측한 $y(k_i+1\mid k_i)=CAx+CBL(0)^T\eta$ |
| `if (beta<y_bar_min)` | 하한 위반 → 활성 |
| `M_act1=-M_act` | $M_{act}=-CBL(0)^T$ |
| `lambda=-inv(M_act1*inv(Omega)*M_act1')*(-y_bar_min+E*Xf+M_act1*(-eta))` | $\lambda_{act}=-(M_{act}\Omega^{-1}M_{act}^T)^{-1}(-y^{min}+CAx+M_{act}\Omega^{-1}\Psi x)$ |
| `eta=-inv(Omega)*(Psi*Xf+M_act1'*lambda)` | $\eta=-\Omega^{-1}(\Psi x+M_{act}^T\lambda_{act})$ |
| `if (beta>y_bar_max)` 블록 | 상한 위반, $M_{act}=CBL(0)^T$ 로 같은 계산 |

`M_act1*(-eta)` 가 $M_{act}\Omega^{-1}\Psi x(k_i)$ 인 이유는 `eta` 가 이 시점에 무제약 해 $-\Omega^{-1}\Psi x(k_i)$ 이기 때문이다. `inv(Omega)` 는 스칼라($N=1$)이므로 실제 계산은 나눗셈이다.

출력 제약 블록 뒤에 $\Delta u$ 제약, 그 뒤에 $u$ 제약이 온다. **$y$ → $\Delta u$ → $u$** 순서로, [[12_Closed_Form_SISO|제약 순위]]에서 정한 "덜 중요한 것부터 처리하고 중요한 것으로 덮어쓴다"는 구현 그대로다. 출력 제약으로 얻은 $\Delta u$ 가 $\Delta u$ 한계를 넘으면 잘리고, 그 결과가 $u$ 한계를 넘으면 다시 잘린다. 최종적으로 반드시 지켜지는 것은 $u$ 제약이다.

플랜트 시뮬레이션에서 `Bp*(u+d(kk))` 로 입력 외란이 더해지고, 제어기는 `d` 를 모른다. 매 샘플 `y_bar_min`, `y_bar_max` 도 다음 설정치로 갱신한다. 이 예제에서는 $r=0$ 이라 상수지만, 설정치가 바뀌는 문제에서는 필요한 줄이다.

### 4~6단계 - 결과

Fig. 3.15 ($y^{min}=-0.14$, $y^{max}=0.14$)를 보면 모든 제약이 만족된다. 그러나 출력을 한계 안에 붙잡기 위해 **$u$ 에 스파이크가 생기고, 출력 제약이 활성이 될 때마다 $\Delta u$ 에도 스파이크가 생긴다.** 외란이 들어와 출력이 한계에 닿는 순간 제어기가 급하게 반대로 밀기 때문이다.

성능 저하를 보려고 한계를 $y^{min}=-0.12$, $y^{max}=0.12$ 로 좁히면 (Fig. 3.16) 폐루프 응답이 **제약 안에서 진동**한다. 출력이 한계에 닿아 제어가 급변하고, 그 급변이 출력을 반대 한계로 보내고, 다시 급변하는 순환이다. 이것이 출력 제약을 부과할 때의 어려움이며, 출력 제약이 꼭 필요하다면 각별히 주의해야 한다는 교훈이다. 2장에서 출력 제약을 소프트로 두라고 한 이유가 여기서 눈에 보인다.

## 연결

- 닫힌형 해의 이론과 제약 순위: [[12_Closed_Form_SISO|SISO 닫힌형 해]]
- `dmpc.m`, `lagd.m`: [[07_MATLAB_dmpc|dmpc.m]], [[02_Laguerre_System_Modelling|lagd.m]]
- 관측기 없는 상태 선택의 2장 예: [[13_Output_Constraints|2장 - Example 2.20]]
- 포화된 $\Delta u$ 를 모델에 알려주는 이유: [[01_Motivational_Examples|2장 - Example 2.2]]
- 2장의 DC 모터 연습문제(관측기 사용)와 비교: [[15_Problems_Roadmap|2장 - 연습문제 2.3]]

## → 검증

- 상태벡터 $[\Delta y(k)\ \Delta y(k-1)\ \Delta u(k-1)\ y(k)]$ 의 네 성분을 코드의 어느 변수가 만드는지 짚을 수 있는가?
- Part A 루프에서 $u$ 제약이 $\Delta u$ 제약보다 뒤에 오는 이유를 말할 수 있는가?
- `beta=E*Xf+M_act*eta` 가 무엇의 예측값이고 왜 `y_bar_min=y_min-r` 과 비교하는지 설명할 수 있는가?
- 출력 한계를 0.14에서 0.12로 좁혔을 때 진동이 생기는 메커니즘을 말할 수 있는가?
