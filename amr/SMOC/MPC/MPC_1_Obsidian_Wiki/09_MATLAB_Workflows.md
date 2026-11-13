---
title: "09. MATLAB 구현 흐름"
aliases:
  - "MPC MATLAB Workflow"
  - "mpcgain"
  - "reced"
tags:
  - mpc
  - matlab
  - implementation
source_pages: "PDF p.33-34, p.40-41, p.47-48, p.52-53 / book p.6-7, p.13-14, p.20-21, p.25-26"
---

# MATLAB 구현 흐름

이 노트는 1장의 Tutorial 1.1-1.3을 "무엇을 계산하는가" 중심으로 재정리한 것이다.

## Tutorial 1.1 - augmented design model

목표: continuous-time state-space model을 discrete-time model로 바꾼 뒤, MPC용 augmented model을 만든다.

흐름:

1. $A_c,B_c,C_c,D_c$를 정의한다.
2. sampling interval $\Delta t$를 정한다.
3. discrete-time model $A_d,B_d,C_d,D_d$를 만든다.
4. 출력 차원 $m_1$, 상태 차원 $n_1$, 입력 차원 $n_{in}$을 확인한다.
5. augmented matrices를 만든다.

핵심 구조는 다음 block matrix를 코드로 채우는 것이다.

$$
A_e=
\begin{bmatrix}
A_d & 0 \\
C_dA_d & I
\end{bmatrix},
\quad
B_e=
\begin{bmatrix}
B_d \\
C_dB_d
\end{bmatrix},
\quad
C_e=
\begin{bmatrix}
0 & I
\end{bmatrix}
$$

결과:

$$
x(k+1)=A_ex(k)+B_e\Delta u(k), \quad y(k)=C_ex(k)
$$

## Tutorial 1.2 - MPC gain matrix

목표: $F$, $\Phi$를 만들고 $\Phi^T\Phi$, $\Phi^TF$, $\Phi^T\bar R_s$를 계산한다.

핵심 아이디어:

- $F$는 $CA,CA^2,\ldots,CA^{N_p}$를 쌓은 행렬이다.
- $\Phi$는 첫 열을 만든 뒤 아래로 shift하면서 만드는 Toeplitz 구조다.
- $\bar R_s$는 $N_p$개의 1로 구성된 reference vector basis다.

계산 결과로 저장할 값은 $\Phi^T\Phi$, $\Phi^TF$, $\Phi^T\bar R_s$ 세 가지다.

이 값들로

$$
\Delta U=(\Phi^T\Phi+\bar R)^{-1}(\Phi^T\bar R_s r-\Phi^TFx)
$$

를 계산한다.

## Tutorial 1.3 - receding horizon simulation

목표: $\Delta U$를 매 샘플 새로 계산하고 첫 $\Delta u$만 적용한다.

Simulation loop의 상태:

- plant state: $x_m$
- feedback state: $X_f=[x_m(k)-x_m(k-1); y(k)]$
- input memory: $u(k-1)$
- reference: $r(k)$

반복 계산은 다음 의사코드로 보면 된다.

```text
for each sample k:
    solve DeltaU from current Xf and r(k)
    take deltau as the first element of DeltaU
    update u by integrating deltau
    propagate plant state and output
    rebuild Xf from state increment and output
```

## Minimal realization 확인

pole-zero cancellation이 있는 모델은 상태공간으로 변환했을 때 불필요한 상태를 포함할 수 있다. 다음 코드는 전체 실현의 controllability와 observability를 확인하고 `minreal`로 최소 실현을 구한다.

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

실행 결과는 `rank_ctrb = 2`, `rank_obsv = 1`이다. `minreal` 적용 후에는 $z=0.1$의 상쇄 모드가 제거되어 상태 차수가 1이 되고 전달함수 $1/(z-0.9)$는 그대로 유지된다. 이 MATLAB 버전에서는 $A_m=0.9$, $B_m=-0.9950$, $C_m=-1.0050$이 출력되며, 교재와 숫자가 달라도 동일한 최소 실현이다. `ss(tfModel)`은 처음부터 최소 차수 실현을 반환할 수 있으므로, 비최소 모델의 rank를 확인할 때는 위처럼 교재의 행렬을 직접 구성한다. 자세한 계산은 [[05_MIMO_MPC#MATLAB 예제: pole-zero cancellation과 minimal realization|MIMO MATLAB 예제]]를 참고한다.

## 구현 체크리스트

- $N_c \leq N_p$인지 확인한다.
- $\Phi^T\Phi+\bar R$가 ill-conditioned인지 확인한다.
- $r_w$를 키우면 입력 변화는 줄고 응답은 느려진다.
- $A-BK_{mpc}$의 eigenvalue로 폐루프 안정성을 확인한다.
- 상태가 모두 측정되지 않으면 [[06_State_Estimation|observer]]를 설계한다.
- observer를 붙인 경우 [[07_State_Estimate_MPC|상태추정 기반 MPC]]의 separation principle을 확인한다.

## 연결

- 행렬 정의: [[08_Key_Equations|핵심 수식 정리]]
- 최적화: [[03_One_Window_Optimization|한 최적화 창에서의 예측제어]]
- receding horizon: [[04_Receding_Horizon_Control|Receding Horizon Control]]
