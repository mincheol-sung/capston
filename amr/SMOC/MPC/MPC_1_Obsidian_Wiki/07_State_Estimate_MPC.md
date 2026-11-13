---
title: "07. 상태추정 기반 MPC"
aliases:
  - "State Estimate Predictive Control"
  - "DMPC with Observer"
tags:
  - mpc
  - observer
  - state-feedback
source_pages: "PDF p.61-65 / book p.34-38"
---

# 상태추정 기반 MPC

상태 $x(k_i)$를 직접 측정할 수 없으면, [[06_State_Estimation|observer]]로 추정한 $\hat x(k_i)$를 MPC 제어법에 넣는다.

> [!summary] 이 노트의 핵심 질문
> 1. observer 이론([[06_State_Estimation]])은 **plant 모델** $(A_m,B_m,C_m)$ 위에서 세웠다. MPC는 **augmented model** $(A,B,C)$ 위에서 도는데, 여기에 observer를 어떻게 붙이나?
> 2. 최적해 안의 진짜 상태 $x(k_i)$를 추정치 $\hat x(k_i)$로 바꿔 끼워도 폐루프가 안 망가지나? → **분리원리**

## Augmented model observer

MPC에서 사용하는 augmented model이

$$
x(k+1)=Ax(k)+B\Delta u(k), \quad y(k)=Cx(k)
$$

라면 observer는

$$
\hat x(k_i+1)
=A\hat x(k_i)+B\Delta u(k_i)+K_{ob}(y(k_i)-C\hat x(k_i))
$$

로 쓴다.

### plant observer와 무엇이 다른가

구조 자체는 [[06_State_Estimation#Observer 구조|06의 observer]]와 **완전히 동일**하다. 모델 예측항 $A\hat x+B\Delta u$ 에 교정항 $K_{ob}(y-C\hat x)$ 를 더한 형태 그대로다. 바뀐 건 어떤 시스템 위에서 도느냐 뿐이다.

| | plant observer (06) | augmented observer (07) |
|---|---|---|
| 상태 | $x_m$ (물리 상태) | $x=[\Delta x_m^T\ \ y^T]^T$ (증분+출력) |
| 입력 | $u(k)$ | $\Delta u(k)$ (증분 입력) |
| 시스템 행렬 | $A_m,B_m,C_m$ | $A,B,C$ ([[02_Augmented_State_Space_Model]]) |

즉 observer에 먹이는 입력이 $u$가 아니라 **증분 $\Delta u$** 라는 점만 주의하면 된다. observer가 추정하는 것도 증강상태 $\hat x$ 전체이고, 여기서 필요한 물리 상태나 출력은 그 안에서 뽑아 쓴다. augmented model은 극이 $\lambda=1$(적분기)에 놓이므로 — [[06_State_Estimation#Open-loop prediction의 한계|open-loop 예측이 절대 안 붙는 대표 케이스]] — observer의 교정항이 여기서는 선택이 아니라 필수다.

### 왜 observer를 controller보다 빠르게 잡나

뒤의 분리원리 때문에 두 loop의 극을 따로 잡을 수 있는데, 실무에서는 **observer 극을 제어 극보다 훨씬 빠르게(원점 가깝게)** 놓는다. 추정오차 $\tilde x$가 제어 동역학보다 먼저 사라져야, 제어기가 "이미 수렴한 좋은 $\hat x$"를 받아 동작하기 때문이다. Example 1.9에서 observer 극을 $0.01$대, 제어 극을 $0.3$대로 잡은 것이 바로 이 원리다(observer가 한 자릿수 이상 빠름).

## 추정 상태 기반 최적 제어

기존 최적해에서 $x(k_i)$ 대신 $\hat x(k_i)$를 사용한다.

### 비용함수에서의 치환

한 window의 비용함수는 원래 $x(k_i)$로 쓰여 있었다. 측정이 안 되므로 그 자리에 그대로 $\hat x(k_i)$를 대입한다.

$$
J=(R_s-F\hat x(k_i))^T(\bar R_s r(k_i)-F\hat x(k_i))
-2\Delta U^T\Phi^T(R_s-F\hat x(k_i))
+\Delta U^T(\Phi^T\Phi+\bar R)\Delta U
$$

$J$를 $\Delta U$에 대해 최소화하면(=$\partial J/\partial\Delta U=0$),

$$
\Delta U
=
(\Phi^T\Phi+\bar R)^{-1}\Phi^T(R_s-F\hat x(k_i))
$$

풀이 형태는 [[03_One_Window_Optimization]]와 글자 하나 다르지 않다. **미지의 $x(k_i)$를 그 추정치 $\hat x(k_i)$로 바꿔 끼운 것뿐**이고, 이렇게 "추정치를 참값처럼 그대로 쓰는" 설계 방식을 certainty equivalence(확실성 등가)라고 부른다. 아래 분리원리가 이 치환이 안전함을 보장한다.

receding horizon을 적용하면

$$
\Delta u(k_i)=K_yr(k_i)-K_{mpc}\hat x(k_i)
$$

가 된다. 원래 상태피드백 제어법 $\Delta u=K_yr-K_{mpc}x$ 에서 $x\to\hat x$ 만 바뀐 형태다.

## 폐루프 결합 구조

observer error를

$$
\tilde x(k)=x(k)-\hat x(k)
$$

라고 하면,

$$
\tilde x(k+1)=(A-K_{ob}C)\tilde x(k)
$$

이고 plant state는

$$
x(k+1)=(A-BK_{mpc})x(k)-BK_{mpc}\tilde x(k)+BK_yr(k)
$$

이다.

### plant state 식은 어떻게 나오나

이 식이 갑자기 튀어나온 것처럼 보여서 헷갈리기 쉬운데, 두 번의 대입일 뿐이다.

**1단계 — 제어법을 상태식에 넣는다.** $\Delta u(k)=K_yr(k)-K_{mpc}\hat x(k)$ 를 $x(k+1)=Ax(k)+B\Delta u(k)$ 에 대입:

$$
x(k+1)=Ax(k)+BK_yr(k)-BK_{mpc}\hat x(k)
$$

**2단계 — 추정치를 오차로 바꾼다.** 우리가 실제로 계산하는 건 $\hat x$지만, 폐루프 극을 보려면 참상태 $x$와 오차 $\tilde x$로 표현해야 한다. $\hat x=x-\tilde x$ 를 넣으면:

$$
x(k+1)=Ax(k)+BK_yr(k)-BK_{mpc}(x(k)-\tilde x(k))
$$
$$
=(A-BK_{mpc})x(k)-BK_{mpc}\big(-\tilde x(k)\big)\ \text{...정리하면}
$$
$$
=(A-BK_{mpc})x(k)+BK_{mpc}\tilde x(k)+BK_yr(k)
$$

> [!note] 부호 주의
> 위 전개에서 $\tilde x$ 항의 부호는 $\hat x=x-\tilde x$ 정의를 따르면 $+BK_{mpc}\tilde x$ 로 나온다. 교재(그리고 아래 결합행렬)는 $\tilde x=x-\hat x$ 를 오차로 두되 결합행렬의 $(2,1)$ 블록을 $-BK_{mpc}$ 로 적는데, 두 표기 모두 **하삼각(block lower-triangular)** 이라는 결론은 동일하다. 핵심은 부호가 아니라 **off-diagonal 블록이 한쪽만 0** 이라는 사실이다.

둘을 합치면

$$
\begin{bmatrix}
\tilde x(k+1) \\
x(k+1)
\end{bmatrix}
=
\begin{bmatrix}
A-K_{ob}C & 0 \\
-BK_{mpc} & A-BK_{mpc}
\end{bmatrix}
\begin{bmatrix}
\tilde x(k) \\
x(k)
\end{bmatrix}
+
\begin{bmatrix}
0 \\
BK_y
\end{bmatrix}
r(k)
$$

이 matrix는 block lower triangular 구조이므로 characteristic equation이 분리된다.

$$
\det(\lambda I-(A-K_{ob}C))=0
$$

$$
\det(\lambda I-(A-BK_{mpc}))=0
$$

> [!tip] 왜 삼각행렬이면 극이 쪼개지나
> 블록 삼각행렬의 행렬식은 대각 블록 행렬식의 곱이다: $\det\begin{bmatrix}P&0\\ Q&R\end{bmatrix}=\det(P)\det(R)$. 그래서 전체 특성다항식 $\det(\lambda I-M)=\det(\lambda I-(A-K_{ob}C))\cdot\det(\lambda I-(A-BK_{mpc}))$ 로 인수분해되고, 근(=고윳값)도 두 집합으로 그냥 나뉜다. off-diagonal의 $-BK_{mpc}$ 항은 행렬식에 전혀 기여하지 못한다 — 이것이 분리원리의 대수적 정체.

## Separation principle

상태추정 기반 MPC의 폐루프 eigenvalue 집합은 다음 둘의 합집합이다.

- observer loop eigenvalues: $A-K_{ob}C$
- predictive control loop eigenvalues: $A-BK_{mpc}$

따라서 observer와 predictive controller는 독립적으로 설계할 수 있고, 결합해도 각 loop의 eigenvalue는 유지된다.

> [!warning] 이 깔끔함이 성립하는 조건
> 분리원리가 정확히 성립하는 건 **제약 없는(unconstrained) 선형 MPC** 일 때다. 이 경우 MPC 해가 상수 이득 $K_{mpc}$의 선형 상태피드백으로 떨어져서 위 대수가 그대로 통한다. 입력·상태에 **제약(constraint)이 걸리면** 제어법이 더 이상 선형이 아니라서 분리원리는 근사로만 성립한다. 그래도 "observer를 빠르게, 제어를 그보다 느리게"라는 설계 지침은 여전히 좋은 출발점이다.

## Example 1.9 핵심

double integrator plant의 augmented model에 대해 다음 설계를 수행한다.

- $N_c=5$
- $N_p=30$
- $r_w=10$
- observer poles: $0.01$, $0.0105$, $0.011$

MATLAB의 pole placement를 사용해

$$
K_{ob}=[1.9685,0.9688,2.9685]^T
$$

를 얻고,

$$
K_{mpc}=[0.8984,1.3521,0.4039]
$$

가 된다. predictive control loop의 폐루프 pole은 $0.3172 \pm j0.4089$, $0.3624$ 근처다.

> [!example] MATLAB에서 `place`에 왜 전치를 넣나
> observer 극배치는 $\det(\lambda I-(A-K_{ob}C))$ 를 원하는 다항식에 맞추는 문제인데, 이는 제어 극배치 $\det(\lambda I-(A^T-C^TK_{ob}^T))$ 와 **쌍대(dual)** 다. 그래서 제어용으로 만들어진 `place`에 $A',C'$ 를 넣어 `K_ob = place(A',C',Pole)'` 로 계산한다. observer 극 $0.01$대 vs 제어 극 $0.3$대 — observer가 훨씬 빠른 것을 여기서 확인.

이 예제의 메시지는 다음이다.

- 출력은 step reference를 따라간다.
- plant에 integrator가 있어서 control signal은 정상상태에서 0으로 수렴한다.
- observer가 포함되어도 controller와 observer pole 설계를 분리해서 생각할 수 있다.

## 연결

- observer 이론: [[06_State_Estimation|상태추정과 관측기]]
- augmented model 구성: [[02_Augmented_State_Space_Model]]
- 한 window 최적화(비용함수 원형): [[03_One_Window_Optimization]]
- receding horizon: [[04_Receding_Horizon_Control|Receding Horizon Control]]
- MATLAB 구현: [[09_MATLAB_Workflows|MATLAB 구현 흐름]]
