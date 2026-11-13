---
title: "MPC 3장 용어집"
aliases:
  - "MPC 3 Glossary"
  - "Laguerre MPC 용어집"
tags:
  - mpc
  - glossary
  - notation
  - laguerre
source_pages: "MPC 3장 전체"
---

# MPC 3장 용어집

1장 기호는 [[MPC_Glossary|1장 용어집]], 2장 기호는 [[MPC2_Glossary|2장 용어집]]에 있다. 여기서는 3장에서 새로 나오는 것만 정리한다.

## Laguerre 함수 관련

| 용어/기호 | 의미 |
|---|---|
| Laguerre 함수 $l_i(k)$ | 직교정규 기저함수. $\Gamma_i(z)$ 의 역 z-변환 |
| $\Gamma_i(z)$ | $i$ 번째 Laguerre 네트워크의 z-전달함수. 1차 저역통과 뒤에 전역통과 필터 $i-1$ 개 |
| $a$ | Laguerre 극점, 스케일링 인자. $0\le a<1$. 감쇠 속도를 결정 |
| $N$ | 전개 항 수. $\eta$ 의 차원. 2장의 $N_c$ 를 대신함 |
| $\beta$ | $1-a^2$ |
| $L(k)$ | $[l_1(k)\ \cdots\ l_N(k)]^T$. $L(k+1)=A_lL(k)$ |
| $A_l$ | Laguerre 상태행렬. 첫 열 $[a,\beta,-a\beta,a^2\beta,\ldots]^T$ 의 Toeplitz 하삼각 |
| $L(0)$ | 초기조건 $\sqrt{\beta}[1,-a,a^2,\ldots]^T$. $\Delta u(k_i)=L(0)^T\eta$ 에 쓰임 |
| Orthonormality (직교정규성) | $\sum_kl_i(k)l_j(k)=\delta_{ij}$. 계수 가중을 대각으로 만들어 주는 성질 |
| Scaling factor | $a$ 의 다른 이름 |
| Pulse operator | $a=0$ 일 때의 Laguerre 함수. 1·2장의 $\Delta U$ 기저 |
| `lagd(a,N)` | $A_l$ 과 $L(0)$ 을 만드는 MATLAB 함수 |

## 설계 관련

| 용어/기호 | 의미 |
|---|---|
| $\eta$ | Laguerre 계수 벡터 $[c_1\ \cdots\ c_N]^T$. 3장의 결정변수 |
| $c_j(k_i)$ | 창 시작 시각 $k_i$ 에서의 $j$ 번째 계수 |
| $\phi(m)^T$ | $\sum_{i=0}^{m-1}A^{m-i-1}BL(i)^T$. 상태 예측에서 $\eta$ 의 계수 행렬 ($n\times N$) |
| $S_c(m)$ | 합성곱 합. $\phi(m)^T$ 와 같으며 재귀식으로 계산 |
| $\Omega$ | $\sum_m\phi(m)Q\phi(m)^T+R_L$. Hessian. 코드의 `E`/`Omega` |
| $\Psi$ | $\sum_m\phi(m)QA^m$. 코드의 `H`/`Psi` |
| $R_L$ | $\eta$ 에 대한 가중 (대각, 입력별 $r_{w,i}I_{N_i}$). 코드의 `R_para` |
| $P_{dmpc}$ | $J_{min}=x^TP_{dmpc}x$ 의 행렬 |
| $K_{mpc}$ | $L(0)^T\Omega^{-1}\Psi$. 상태 피드백 이득 |
| `Lzerot` | MIMO에서 $\Delta u(k_i)$ 를 $\eta$ 에서 뽑는 블록 대각 $L_i(0)^T$ 행렬 |
| $x_r(k_i)$ | $[0\ \cdots\ 0\ r(k_i)]^T$. 설정치를 상태 차원으로 올린 벡터 |
| $e(k_i)$ | $y(k_i)-r(k_i)$. 피드백 상태의 마지막 성분 |
| DLQR | 이산시간 선형 이차 조정기. $N$ 이 클 때 DMPC가 수렴하는 목표 |
| $P_\infty$, $K$ | DLQR의 Riccati 해와 이득 |
| $N_0$ | 제약 활성 구간이 끝나고 LQR 궤적으로 복귀하는 샘플 시각 |

## 제약과 안정성 관련

| 용어/기호 | 의미 |
|---|---|
| `Mdu(a,N,n_in,Nc)` | $\Delta u$ 제약 행렬. $L(m)^T$ 블록을 $N_c$ 개 쌓음 |
| `Mu(a,N,n_in,Nc)` | $u$ 제약 행렬. $\sum_iL(i)^T$ 누적 블록을 쌓음 |
| $\bar u(k_i-1)$ | 직전 제어값을 $N_c$ 번 쌓은 벡터. 진폭 제약 우변 |
| Terminal-state constraint | 종단 상태 등식 제약 $x(k+N_p\mid k)=0$ |
| Lyapunov 함수 $V$ | 안정성 증명에서 쓰는 $J_{min}$ |
| Dual-mode control | 예측제어로 종단 집합에 넣고 선형 피드백으로 전환하는 구조 |
| Terminal constraint set $X_0$ | 제약이 비활성인 원점 근방 집합 |
| Closed-form solution (SISO) | 첫 샘플 제약만 보고 비교문으로 푸는 해 |
| Ranking of constraints | 충돌 시 $u$ > $\Delta u$ > $y$ 순으로 만족시키는 규칙 |
| $M_{act}$ (3장) | $\pm L(0)^T$ ($\Delta u$, $u$), $\pm CBL(0)^T$ ($y$). 모두 $L(0)^T$ 의 배수 |

## 자주 헷갈리는 쌍

### $N$ vs $N_c$

- $N_c$ (1·2장): 제어구간. $\Delta U$ 의 차원. 그 이후 $\Delta u=0$ 으로 강제.
- $N$ (3장): 전개 항 수. $\eta$ 의 차원. $\Delta u$ 는 Laguerre 감쇠로 자연히 0에 접근.

$a=0$ 이면 $N=N_c$ 로 같아진다.

### $a$ vs $R$

- $R$ (또는 $r_w$): 목표 궤적(DLQR)을 결정한다. 작으면 빠른 응답.
- $a$: 목표 궤적을 얼마나 잘 근사하는가에 영향. $N$ 이 크면 영향이 작고, $N$ 이 작으면 미세조정 손잡이.

### $\Omega$, $\Psi$ vs $\Phi^T\Phi+\bar R$, $\Phi^TF$

같은 역할이다. 결정변수가 $\Delta U$ 에서 $\eta$ 로 바뀌면서 이름이 바뀌었을 뿐이다. 계산 방식은 다르다. 1장은 큰 행렬 $F$, $\Phi$ 를 만들어 곱하고, 3장은 합성곱 합 재귀로 누적한다.

### 임펄스 응답 근사의 계수 vs 제어궤적의 계수

- 임펄스 응답 근사 (Tutorial 3.1): 주어진 신호를 사영. $N$ 을 늘려도 기존 계수 불변.
- 제어궤적 (Example 3.2): 비용함수 최소화의 결합 문제. $N$ 을 늘리면 기존 계수도 조금 바뀜.

### 유한합 직교정규성 vs 무한합 직교정규성

직교정규성은 무한합에서 정확히 성립한다. 유한합 $\sum_{k=0}^{N_p}$ 이 항등행렬에 가까우려면 $N_p$ 가 Laguerre 함수의 감쇠 구간보다 길어야 한다. $a$ 가 클수록 더 긴 $N_p$ 가 필요하다.

### 종단 등식 제약 vs 종단 집합

- 등식 제약 $x(k+N_p)=0$: 증명은 깔끔하지만 결정변수를 잡아먹고 다른 제약과 종속이 될 수 있다.
- 종단 집합 $X_0$: 원점 근방 집합으로 완화. 안에서는 $K_{lqr}$ 이 제약 없이 안정화.

## 연결

- 1장 용어: [[MPC_Glossary|MPC 용어집]]
- 2장 용어: [[MPC2_Glossary|MPC 2장 용어집]]
- 수식 색인: [[14_Key_Equations_MPC3|3장 핵심 수식 정리]]
- 전체 목차: [[00_MPC3_Index|MPC 3장 인덱스]]
