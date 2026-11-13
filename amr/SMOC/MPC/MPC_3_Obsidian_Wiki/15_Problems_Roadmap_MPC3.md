---
title: "15. 연습문제 로드맵"
aliases:
  - "MPC 3장 Problems"
  - "3장 연습문제"
tags:
  - mpc
  - exercises
  - roadmap
  - laguerre
source_pages: "PDF p.60-64 / book p.144-148"
---

# 연습문제 로드맵

3장의 problems는 두 갈래다. 3.1~3.5는 Laguerre 모델과 SISO 설계를 손에 익히는 문제이고, 3.6~3.9는 MIMO 플랜트(이중 탱크, 식품 압출기)에 대해 무제약 설계 → 관측기 → 제약까지 한 벌을 완성하는 문제다.

| 문제 | 주제 | 먼저 볼 노트 |
|---|---|---|
| 3.1 | 시간지연 플랜트의 Laguerre 모델 | [[02_Laguerre_System_Modelling]] |
| 3.2 | Laguerre 모델의 상태공간 실현과 증강 | [[01_Laguerre_Functions]], [[02_Augmented_State_Space_Model]] |
| 3.3 | Laguerre 모델 기반 DMPC 설계와 모델 불일치 | [[04_Minimization_and_Convolution_Sum]], [[08_MATLAB_Simulation]] |
| 3.4 | 모델 차수 $n$ 증가의 효과 | [[02_Laguerre_System_Modelling]] |
| 3.5 | 불안정 비최소위상 시스템 | [[05_Optimal_Trajectory_and_DLQR]], [[11_Stability_Analysis]] |
| 3.6 | 이중 탱크 MIMO 설계, $a$ 의 효과 | [[06_MIMO_Extension]], [[07_MATLAB_dmpc]] |
| 3.7 | 이중 탱크 제약 제어 | [[09_Constraints_on_Delta_u]], [[10_Constraints_on_u]] |
| 3.8 | 식품 압출기 MIMO 설계와 관측기 | [[07_MATLAB_dmpc]], [[08_MATLAB_Simulation]] |
| 3.9 | 식품 압출기 제약 제어 | [[09_Constraints_on_Delta_u]] |

## 3.1 시간지연 플랜트의 Laguerre 모델

Laguerre 모델은 시스템 식별 문헌에서 기저함수 모델이라 불리며 (Wang and Cluett 2000, Wahlberg 1991), 플랜트 실험 데이터로 추정된다. 이 모델을 예측제어에 쓰면 (Finn et al. 1993) 관측기가 필요 없다 (Problem 3.2). 연속시간 플랜트

$$
G(s)=\frac{e^{-3s}}{(10s+1)^3}
$$

을 이산시간 Laguerre 전달함수 모델

$$
G_A(z)=\frac{\sqrt{1-a^2}}{1-az^{-1}}\left(c_1+c_2\frac{z^{-1}-a}{1-az^{-1}}+c_3\Big(\frac{z^{-1}-a}{1-az^{-1}}\Big)^2+\cdots+c_n\Big(\frac{z^{-1}-a}{1-az^{-1}}\Big)^{n-1}\right)
$$

로 근사한다. 샘플링 $\Delta t=1$, $n=3$, $a=0.9$ 다.

풀이 순서는 [[02_Laguerre_System_Modelling|Tutorial 3.1]] 그대로다. 먼저 $G(s)$ 를 이산화해 이산 전달함수를 얻고 (시간지연 3초는 $\Delta t=1$ 에서 $z^{-3}$), 임펄스 응답을 만들고, $a=0.9$, $N=3$ 의 Laguerre 함수와 내적해 $c_1,c_2,c_3$ 을 구한다. 시정수 10의 3차 지연 시스템은 응답이 매우 느리므로 $a=0.9$ 처럼 큰 극점이 필요하고, [[01_Laguerre_Functions|Example 3.1]]에서 본 대로 임펄스 응답 데이터도 90 샘플 이상 길게 잡아야 한다.

## 3.2 Laguerre 모델의 상태공간 실현

3.1에 이어, Laguerre 함수의 상태공간 실현 $L(k+1)=A_lL(k)$ 를 이용한다. 상태벡터 $x_m(k)$ 를 $L(k)$ 에, $A_m$ 을 $A_l$ 에, $B_m$ 을 $L(0)$ 에 대응시키고 $C_m$ 을 Laguerre 계수 벡터 $[c_1\ c_2\ \cdots\ c_n]$ 으로 두면 Laguerre 전달함수 $G_A(z)$ 가 이 상태공간 모델로 실현됨을 확인한다.

이유는 이렇다. 입력 $u$ 에 대한 응답이 $\sum_kH(k)u(\cdot)$ 의 합성곱이고 $H(k)=C_mL(k)=C_mA_l^kL(0)$ 이므로, $x_m(k+1)=A_lx_m(k)+L(0)u(k)$, $y=C_mx_m$ 의 임펄스 응답이 정확히 $H(k)$ 다. 이 실현의 상태변수는 입력신호로부터 측정 가능하므로 관측기가 필요 없다. 이 모델에 적분기를 붙인 증강 상태공간 모델까지 구한다. [[02_Augmented_State_Space_Model|1장의 증강 절차]]를 그대로 쓰면 된다.

## 3.3 Laguerre 모델 기반 DMPC와 모델 불일치

3.2의 이산 상태공간 모델로 플랜트 (3.75)에 대해 계단 입력 외란을 제거하고 계단 설정치를 따르는 DMPC를 설계한다. $Q=C^TC$, $R=1$, $N_p=30$, $N=3$, $a=0.5$ 다. $k=0$ 에 계단 설정치 $r=1$, $k=30$ 에 계단 입력 외란이 들어오고, $k=0,\ldots,100$ 을 시뮬레이션한다. 공칭 시뮬레이션에서는 Laguerre 모델 $G_A(z)$ 를 플랜트로 쓰고, 초기조건은 0이다.

그 다음 $G(s)$ 를 직접 이산화한 $G(z)$ 를 플랜트로 두고 다시 시뮬레이션해 공칭 응답과 비교한다. 성능 차이의 원인은 **모델 불일치**다. $n=3$ 의 Laguerre 모델은 실제 플랜트의 근사이므로, 특히 시간지연을 정확히 표현하지 못하는 부분에서 예측 오차가 생긴다. 3.1에서 얻은 임펄스 응답 근사 그림에서 오차가 큰 구간을 보면 어디서 차이가 날지 예측할 수 있다.

## 3.4 모델 차수를 늘리면

Laguerre 모델은 본질적으로 근사 모델이다. 성능 요구가 높을수록 모델이 정확해야 하고, 정확도는 모델 항 수 $n$ 을 늘려 개선한다. 3.1의 $n$ 을 8로 올려 Laguerre 모델이 얼마나 좋아지는지 보고, 다른 파라미터는 그대로 둔 채 이 고차 모델로 3.3을 반복해 폐루프 응답을 비교한다. [[02_Laguerre_System_Modelling|Tutorial 3.1]]에서 본 "직교정규 기저에서는 $n$ 을 늘려도 기존 계수가 바뀌지 않는다"는 사실 덕분에 $c_1\sim c_3$ 은 3.1의 값 그대로이고 $c_4\sim c_8$ 만 추가된다.

## 3.5 불안정 비최소위상 시스템

$$
G(z)=\frac{0.3(z-1.1)}{(z-0.3)(z-1.5)}
$$

는 극점 하나($1.5$)와 영점($1.1$)이 단위원 밖에 있다. 적분 동작을 갖는 DMPC를 설계한다. $Q=C^TC$, $R=0.1$, $N_p=28$, $N=3$, $a=1/1.5$ 다. $a$ 를 **불안정 극점의 역수**로 잡는 것이 힌트다. 이 파라미터로 안정한 폐루프가 나오는지 확인한다.

불안정 시스템에서는 $N_p$ 가 특히 중요하다 (4장). $N_p$ 를 줄이고 늘리면서 폐루프 안정성과 Hessian $\Omega$ 의 수치 조건(조건수)이 어떻게 변하는지 관찰한다. 불안정 플랜트에서 $A^m$ 이 $m$ 에 따라 지수적으로 커지므로 $\Omega$ 와 $\Psi$ 의 원소도 커지고, $N_p$ 가 크면 조건수가 급격히 나빠진다. [[11_Stability_Analysis|안정성 노트]]에서 "예제 플랜트가 안정했기 때문에 큰 $N_p$ 없이 수렴을 얻었다"고 한 단서가 여기서 시험된다.

## 3.6 이중 탱크 - MIMO 설계와 $a$ 의 효과

직렬 연결된 두 탱크의 모델은

$$
\begin{bmatrix}
\dot x_1(t) \\ \dot x_2(t)
\end{bmatrix}
=
\begin{bmatrix}
-\frac{\beta}{\alpha_1} & \frac{\beta}{\alpha_1} \\
\frac{\beta}{\alpha_2} & -\frac{\beta}{\alpha_2}
\end{bmatrix}
\begin{bmatrix}
x_1(t) \\ x_2(t)
\end{bmatrix}
+
\begin{bmatrix}
\frac{1}{\alpha_1} & 0 \\
0 & \frac{1}{\alpha_2}
\end{bmatrix}
\begin{bmatrix}
u_1(t) \\ u_2(t)
\end{bmatrix},
\qquad
y(t)=
\begin{bmatrix}
1 & 0 \\ 0 & 1
\end{bmatrix}
\begin{bmatrix}
x_1(t) \\ x_2(t)
\end{bmatrix}
$$

이다. $\alpha_1,\alpha_2>0$ 은 각 탱크의 동역학 계수, $\beta$ 는 두 탱크의 연결 계수, $x_1,x_2$ 는 수위, $u_1,u_2$ 는 유입 밸브다 (Fig. 3.17). 시스템 행렬의 고유값이 $(0,-1.5)$ 이므로 적분 모드가 있다. 행 합이 0인 행렬이라 고유값 0이 생기는 것이고, 물리적으로는 두 탱크의 총 수위가 유입에 대해 적분기처럼 동작한다는 뜻이다.

$\alpha_1=2$, $\alpha_2=1$, $\beta=1$, $\Delta t=0.1$ 이다. 두 수위가 계단 기준입력을 정상상태 오차 없이 따르도록 DMPC를 설계한다. $Q=C^TC$, $R=0.1I$, $N_p=20$, $N_1=N_2=3$ 을 고정하고 $a_1=a_2$ 를 0.0, 0.5, 0.8 로 바꾸며 폐루프 계단응답의 변화를 관찰한다. 폐루프 응답에 오버슈트를 원치 않으면 어떤 $a$ 를 쓸지 답한다. [[05_Optimal_Trajectory_and_DLQR|Example 3.5의 Case B]]에서 본 대로 $N$ 이 작을 때 $a$ 를 키우면 이득이 줄고 응답이 부드러워진다는 것이 판단 근거다.

## 3.7 이중 탱크 - 제약 제어

3.6에 운전 제약을 넣는다. $a_1=a_2=0.8$, $N_1=N_2=3$, $N_p=20$, $Q=C^TC$, $R=0.1I$ 로 $\Omega$, $\Psi$ 를 계산한다. 초기조건 0에서 기준 $r_1=1$, $r_2=0.5$ 로 탱크를 채우고, 수위가 도달하면 두 번째 탱크 기준은 유지한 채 첫 탱크 기준을 두 배로 올린다.

```matlab
Nsim=100;
sp=0.5*ones(2,Nsim);
sp(1,1:N_sim/2)=ones(1,Nsim/2);
sp(1,N_sim/2+1:N_sim)=2*ones(1,Nsim/2);
```

제어신호 제약은

$$
-0.3\le\Delta u_1(k)\le 0.3,\quad
-0.1\le\Delta u_2(k)\le 0.1,\quad
0\le u_1(k)\le 2,\quad
-2\le u_2(k)\le 0.5
$$

다. [[09_Constraints_on_Delta_u|Mdu.m]] 과 [[10_Constraints_on_u|Mu.m]] 을 $n_{in}=2$ 로 호출해 $M$ 을 만들고, `gamma` 에는 입력별 한계를 샘플마다 쌓는다. 진폭 제약의 우변에는 직전 $u$ 가 들어가므로 매 샘플 갱신한다. $u_1\ge 0$ 처럼 하한이 0인 제약은 초기 $u=0$ 에서 바로 활성이 될 수 있으니 첫 샘플의 양립 여부를 [[10_Constraints_on_u|Example 3.8]]의 방식으로 먼저 산수해 본다.

## 3.8 식품 압출기 - MIMO 설계와 관측기

압출은 회전 스크루로 식품 재료를 배럴을 통해 좁은 다이로 밀어내는 연속 공정이다. $u_1,u_2$ 는 스크루 속도와 액체 펌프 속도, $y_1,y_2$ 는 비기계에너지(SME)와 모터 토크다. 연속시간 모델은

$$
\begin{bmatrix}
y_1 \\ y_2
\end{bmatrix}
=
\begin{bmatrix}
G_{11} & G_{12} \\ G_{21} & G_{22}
\end{bmatrix}
\begin{bmatrix}
u_1 \\ u_2
\end{bmatrix}
$$

$$
G_{11}=\frac{0.21048s+0.00245}{s^3+0.302902s^2+0.066775s+0.002186},
\qquad
G_{12}=\frac{-0.001313s^2+0.000548s-0.000052}{s^4+0.210391s^3+0.105228s^2+0.00777s+0.000854}
$$

$$
G_{21}=\frac{0.000976s-0.000226}{s^3+0.422036s^2+0.091833s+0.003434},
\qquad
G_{22}=\frac{-0.00017}{s^2+0.060324s+0.006836}
$$

이다. 설계 절차는 다음과 같다.

1. $\Delta t=3$ 초로 이산 모델을 얻는다. [[07_MATLAB_dmpc|Tutorial 3.3]]이 개별 전달함수를 MIMO 시스템으로 묶어 이산 상태공간 모델을 얻는 방법을 보여 준다 (`tf` → `ss(...,'min')` → `c2dm`).
2. 계단 설정치 추종과 상수 외란 제거를 위한 DMPC를 설계한다. $N_p=28$, $N_1=N_2=4$, $a_1=a_2=0.6$, $R=I$, $Q=C^TC$ 다. 폐루프 피드백 이득 행렬과 폐루프 극점의 위치를 구한다.
3. $R=0$ 과 $R=6I$ 로 반복해 $R=I$ 와 비교하고 관찰을 정리한다. $R=0$ 에서는 [[04_Minimization_and_Convolution_Sum|$\Omega$ 의 가역성]]이 보장되지 않을 수 있다는 점에 주의한다.
4. `dlqr` 로 관측기를 설계한다. `Kob=dlqr(A',C',Qob,Rob)'`, $Q_{ob}=I_1$, $R_{ob}=0.1I_2$ 다. 관측기 이득 $K_{ob}$ 를 구한다. 쌍대성으로 $(A^T,C^T)$ 에 LQR을 설계하면 관측기 이득이 나온다.
5. 관측기로 상태를 추정해 기준 추종을 시뮬레이션한다. 설정치는 $r_1=3$, $r_2=0$ 이고, 제어신호 정상상태 $u_{ss}=[300\ 2000]^T$, 출력 정상상태 $y_{ss}=[639\ 42]^T$ 다. 실제값 $u_{act}=u+u_{ss}$, $y_{act}=y+y_{ss}$ 로 결과를 표시한다. 증분 변수와 물리 변수의 구분은 [[02_Constraint_Formulation|2장 §2.3.1]]의 주의점이다.
6. 외란 제거와 잡음 감쇠를 시뮬레이션한다. 출력 $y_2$ 에 표준편차 0.1 의 백색잡음을 더해 측정잡음을 흉내 내고, 40 샘플 뒤에 $u_2$ 에 단위 계단 입력 외란을 넣는다. 정상상태값을 포함한 실제 신호로 표시한다.

[[08_MATLAB_Simulation|simuob.m]] 이 5번과 6번의 뼈대다.

## 3.9 식품 압출기 - 제약 제어

3.8에 운전 제약을 넣는다. $N_p=28$, $N_1=N_2=4$, $a_1=a_2=0.8$, $R=I$, $Q=C^TC$ 로 비용함수 $J=\eta^T\Omega\eta+2\eta^T\Psi\hat x(k_i)$ 를 만든다. 제약은 제어신호의 변화에 대해

$$
-0.1\le\Delta u_1(k)\le 0.1,
\qquad
-0.1\le\Delta u_2(k)\le 0.1
$$

이다. 급격한 요구 변화로부터 스크루와 펌프를 보호하는 제약이다. 3.8의 관측기를 쓰고, SME는 일정하게 유지하면서 모터 토크에 단위 계단 설정치 변화를 주어 시뮬레이션한다. [[08_MATLAB_Simulation|simuob.m]] 에서 `eta` 계산 한 줄을 `QPhild(Omega,Psi*Xf,M,gamma)` 로 바꾸고, `M` 은 [[09_Constraints_on_Delta_u|Mdu.m]] 으로 만든다. 처음 몇 샘플에만 제약을 걸어도 충분하다는 것이 3.6절의 결론이다.

## 풀이 공통 순서

1. 플랜트 모델을 정리하고 이산 상태공간으로 바꾼다. MIMO는 `tf` → `ss(...,'min')` → `c2dm`.
2. 증강 모델 $A$, $B$, $C$ 를 만든다.
3. 입력별 $(a_i,N_i)$ 와 $N_p$, $Q=C^TC$, $R$ 을 정하고 `dmpc` 로 $\Omega$, $\Psi$ 를 만든다.
4. `lagd` 로 $L_i(0)$ 을 만들어 $K_{mpc}$ 와 $A-BK_{mpc}$ 의 고유값을 확인한다. `dlqr` 과 비교하면 검증이 된다.
5. 관측기가 필요하면 `dlqr(A',C',Qob,Rob)'` 로 설계한다.
6. `simuuc` 또는 `simuob` 로 무제약 폐루프를 먼저 확인한다.
7. 제약이 있으면 `Mdu`, `Mu` 로 $M$ 을 만들고, `gamma` 를 매 샘플 갱신하며 `QPhild` 로 푼다. 첫 샘플에서 제약이 양립하는지 미리 산수한다.
8. 제약이 실제로 만족되는지, 승수가 수렴하는지(활성 제약이 충돌하지 않는지) 확인한다.

## 연결

- 2장 연습문제: [[15_Problems_Roadmap|2장 연습문제 로드맵]]
- 1장 연습문제: [[10_Problems_Roadmap|1장 연습문제 로드맵]]
- 도구 모음: [[07_MATLAB_dmpc|dmpc.m]], [[08_MATLAB_Simulation|simuuc.m / simuob.m]], [[09_Constraints_on_Delta_u|Mdu.m]], [[10_Constraints_on_u|Mu.m]], [[08_MATLAB_QPhild|QPhild.m]]
