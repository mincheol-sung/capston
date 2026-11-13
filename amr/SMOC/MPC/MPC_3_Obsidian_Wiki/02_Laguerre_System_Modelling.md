---
title: "02. Laguerre 네트워크를 이용한 시스템 표현과 lagd.m"
aliases:
  - "Laguerre System Modelling"
  - "Tutorial 3.1"
  - "lagd.m"
  - "lagdmodel.m"
tags:
  - mpc
  - laguerre
  - matlab
  - system-identification
source_pages: "PDF p.6-9 / book p.90-93"
---

# Laguerre 네트워크를 이용한 시스템 표현과 lagd.m

Laguerre 네트워크는 원래 시스템 식별 분야에서 안정한 시스템의 임펄스 응답을 표현하는 데 쓰였다 (Wahlberg 1991). 이 노트는 그 표현 방식과, 3장 전체에서 계속 호출되는 함수 `lagd.m` 을 만드는 Tutorial 3.1을 정리한다. 예측제어 설계는 이 "임펄스 응답을 Laguerre 함수로 표현한다"는 발상을 $\Delta u$ 궤적에 그대로 옮긴 것이다.

## 임펄스 응답의 Laguerre 전개

안정한 시스템의 임펄스 응답을 $H(k)$ 라 하면, $N$ 개의 항으로

$$
H(k)=c_1l_1(k)+c_2l_2(k)+\cdots+c_Nl_N(k)
$$

로 쓴다. 계수 $c_i$ 는 직교정규성 덕분에

$$
c_i=\sum_{k=0}^{\infty}H(k)l_i(k)
$$

로 정해진다. 이 식은 Fourier 급수의 계수를 구하는 식과 같은 구조다. 왜 그런지 1차원으로 보면 이렇다. $H$ 를 벡터, $l_i$ 를 서로 수직인 단위벡터라고 생각하면, $H$ 의 $l_i$ 방향 성분은 내적 $\langle H,l_i\rangle$ 하나로 결정되고 다른 축의 영향을 받지 않는다. 직교정규가 아니었다면 계수를 구하려고 $N\times N$ 연립방정식을 풀어야 했을 것이다.

같은 이유로 이 계수는 제곱오차

$$
J_{SE}=\sum_{k=1}^{\infty}\Big(H(k)-\sum_{i=1}^{N}c_il_i(k)\Big)^2
$$

도 최소화한다. 그리고 $N$ 을 늘리면 $a$ 를 어떻게 골랐든 근사는 좋아진다. 다만 $a$ 를 잘 고르면 훨씬 적은 $N$ 으로 같은 정확도에 도달한다. 설계 파라미터는 극점 $a$ 와 항 수 $N$ 두 개뿐이다.

## Tutorial 3.1 - 임펄스 응답 근사 실험

이산시간 시스템

$$
G(z)=\frac{z-0.1}{(z-0.8)(z-0.9)}
$$

의 임펄스 응답을 3차 Laguerre 네트워크($N=3$)로 근사하고, $N$ 과 $a$ 가 정확도에 미치는 영향을 관찰한다.

### 1단계 - lagd.m

`lagd.m` 은 주어진 $a$ 와 $N$ 에 대해 $A_l$ 과 $L(0)$ 을 만든다. $A_l$ 은 Toeplitz 행렬이므로 첫 열만 만들고 나머지 열은 첫 열을 아래로 밀어서 얻는다.

```matlab
function [A,L0]=lagd(a,N)
v(1,1)=a;
L0(1,1)=1;
for k=2:N
v(k,1)=(-a).^(k-2)*(1-a*a);
L0(k,1)=(-a).^(k-1);
end
L0=sqrt((1-a*a))*L0;
A(:,1)=v;
for i=2:N
A(:,i)=[zeros(i-1,1);v(1:N-i+1,1)];
end
```

코드와 [[01_Laguerre_Functions|앞 노트]]의 식을 대응시키면 다음과 같다.

| 코드 | 의미 |
|---|---|
| `v(1,1)=a` | $A_l$ 첫 열의 첫 원소 $a$ |
| `v(k,1)=(-a).^(k-2)*(1-a*a)` | 첫 열의 $k$ 번째 원소 $(-a)^{k-2}\beta$. $k=2$ 면 $\beta$, $k=3$ 이면 $-a\beta$ |
| `L0(k,1)=(-a).^(k-1)` 후 `sqrt(1-a*a)` 곱 | $l_k(0)=\sqrt{\beta}(-a)^{k-1}$ |
| `A(:,i)=[zeros(i-1,1);v(1:N-i+1,1)]` | $i$ 번째 열은 첫 열을 $i-1$ 칸 내린 것 |

이 함수는 이후 [[07_MATLAB_dmpc|dmpc.m]], [[09_Constraints_on_Delta_u|Mdu.m]], [[10_Constraints_on_u|Mu.m]], [[13_MATLAB_DC_Motor|motor1.m]] 에서 계속 호출된다.

### 2단계 - lagdmodel.m

임펄스 응답 데이터를 만든다. `dimpulse` 가 전달함수에서 임펄스 응답을 생성한다.

```matlab
clear
numd=[1 -0.1];
dend=conv([1 -0.8],[1 -0.9]);
N_sim=60;
k=0:(N_sim-1);
H=dimpulse(numd,dend,k);
```

Laguerre 함수를 $N_{sim}$ 샘플만큼 생성한다. 차분방정식 $L(k+1)=A_lL(k)$ 를 그대로 돈다.

```matlab
a=0.8;
N=3;
[A1,L0]=lagd(a,N);
L(:,1)=L0;
for kk=2:N_sim;
L(:,kk)=A1*L(:,kk-1);
end
```

계수를 계산하고 모델 응답을 만든다. `L(1,:)*H` 가 곧 $\sum_kH(k)l_1(k)$ 다.

```matlab
c1=L(1,:)*H;
c2=L(2,:)*H;
c3=L(3,:)*H;
H_model=c1*L(1,:)+c2*L(2,:)+c3*L(3,:);
figure
plot(k,H)
hold on
plot(k,H_model,'LineWidth',2,'Color',[.8 0 0])
set(gca,'FontSize',20,'FontName','helvetica')
legend('data','model')
xlabel('Sampling Instant')
ylabel('Impulse Response')
```

직교정규성은 작업공간에서 다음으로 확인한다.

```matlab
L(:,1)*L(:,1)'
L(:,1)*L(:,2)'
```

교재의 이 두 줄은 열벡터 하나끼리의 외적을 찍는 것이라 그대로는 $N\times N$ 행렬이 나온다. 의도는 시간 방향 내적 $\sum_kl_i(k)l_j(k)$ 이므로 `L(1,:)*L(1,:)'` 와 `L(1,:)*L(2,:)'` 로 읽는 것이 맞다. 앞이 1, 뒤가 0에 가까우면 된다.

### 3단계 - 관찰

$N$ 을 4로 늘려 보고, $a$ 를 바꿔 보라는 것이 실험의 요지다. 특히 $a=0$ 을 시도해 본다.

Fig. 3.3은 $a=0.8$ 에서 $N=3$ 과 $N=4$ 의 결과다. $N=4$ 에서 모델이 눈에 띄게 좋아진다. 이 실험을 다시 돌려 보면 계수는 다음과 같이 나온다.

| $N$ | $c_1$ | $c_2$ | $c_3$ | $c_4$ | 제곱오차 합 |
|---|---|---|---|---|---|
| 3 | 4.381 | 6.869 | 2.376 | - | 0.924 |
| 4 | 4.381 | 6.869 | 2.376 | 0.910 | 0.096 |

여기서 눈여겨볼 것이 있다. $N$ 을 3에서 4로 늘려도 $c_1$, $c_2$, $c_3$ 은 **바뀌지 않는다.** 직교정규 기저에서는 각 계수가 독립적으로 결정되기 때문이다. 항을 추가하는 것은 이미 구한 계수를 건드리지 않고 오차만 줄인다. 이 성질이 뒤에서 "N을 늘리면 최적 궤적으로 수렴한다"는 주장의 바탕이 된다.

교재가 강조하는 비교는 이것이다. $N=4$ 이면 임펄스 응답의 동역학을 잡는 데 파라미터 4개가 든다. $a=0$ 인 펄스 연산자 모델은 같은 일에 60개가 든다. **$a$ 를 적절히 고르면 Laguerre 네트워크가 펄스 모델보다 훨씬 효율적이다.** 이것이 3장 전체를 관통하는 주장이고, 예측제어에서 $\Delta u$ 궤적에 대해 같은 절약이 일어난다.

## 연결

- 함수의 정의와 직교정규성: [[01_Laguerre_Functions|이산시간 Laguerre 함수]]
- 이 아이디어를 $\Delta u$ 궤적에 적용: [[03_Design_Framework_and_Cost|설계 프레임워크와 비용함수]]
- 같은 근사를 제어 궤적에 대해 수치로 확인: [[05_Optimal_Trajectory_and_DLQR|최적 궤적과 DLQR로의 수렴]]
- 연습문제 3.1이 이 튜토리얼의 직접 응용이다: [[15_Problems_Roadmap_MPC3|연습문제 로드맵]]

## → 검증

- $c_i=\sum_kH(k)l_i(k)$ 가 왜 제곱오차를 최소화하는 계수인지 "직교"라는 말로 설명할 수 있는가?
- `lagd.m` 에서 `v(k,1)=(-a).^(k-2)*(1-a*a)` 가 $A_l$ 의 어느 원소를 만드는지 말할 수 있는가?
- $N$ 을 늘렸을 때 기존 계수가 바뀌지 않는 이유를 말할 수 있는가?
