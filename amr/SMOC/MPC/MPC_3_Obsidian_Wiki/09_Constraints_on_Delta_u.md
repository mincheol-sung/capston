---
title: "09. 제어 증분 Δu 에 대한 제약과 Mdu.m"
aliases:
  - "Mdu.m"
  - "Tutorial 3.7"
  - "Example 3.6"
  - "Laguerre 제약 제어"
tags:
  - mpc
  - laguerre
  - constraints
  - matlab
source_pages: "PDF p.34-38 / book p.118-122"
---

# 제어 증분 Δu 에 대한 제약과 Mdu.m

3.6절은 2장의 QP 방법을 Laguerre 파라미터화에 적용한다. 결정변수가 $\eta$ 로 바뀌었으므로 제약도 $\eta$ 에 대한 선형 부등식으로 번역해야 한다. 이 노트는 그중 증분 $\Delta u$ 제약을 다룬다.

## Laguerre 파라미터화가 제약에 주는 두 가지 이점

첫째, **제약을 걸 미래 시점을 자유롭게 고를 수 있다.** 2장에서는 $\Delta U$ 의 원소가 곧 미래 샘플이라 제약 위치가 $N_c$ 개로 고정되었다. Laguerre에서는 $\Delta u(k_i+m)=L(m)^T\eta$ 가 임의의 $m$ 에 대해 성립하므로, 원하는 $m$ 들만 골라 제약을 걸 수 있다. 대규모 시스템에서 제약 개수와 온라인 계산량을 줄이는 데 쓸 수 있다.

둘째, **감쇠가 보장된다.** Laguerre 함수에는 지수 감쇠 인자가 들어 있으므로 $\Delta u$ 는 과도 구간이 지나면 반드시 0으로 간다. 그래서 대부분의 경우 과도 구간에만 제약을 걸어도 충분하고, 이것이 다시 제약 개수를 줄인다.

## 3.6.1 제약 문제의 형태

시각 $k_i$ 의 상태 $x(k_i)$ 가 주어지고 $\Delta u$ 의 하한과 상한이 $\Delta u^{min}$, $\Delta u^{max}$ 일 때, 최적화 문제는

$$
\min_\eta J=\eta^T\Omega\eta+2\eta^T\Psi x(k_i)
$$

$$
\text{s.t.}\quad\Delta u^{min}\le\Delta u(k_i+m)\le\Delta u^{max},
\qquad m=0,1,2,\ldots
$$

이다. SISO에서 $\Delta u(k_i+m)=L(m)^T\eta$ 이므로 제약은 $\Delta u^{min}\le L(m)^T\eta\le\Delta u^{max}$ 다. 다입력이면 $\Delta u^{min}$, $\Delta u^{max}$ 가 입력별 한계를 담은 벡터이고, 미래 시점 $m$ 에서의 제약은

$$
\Delta u^{min}\le
\begin{bmatrix}
L_1(m)^T & o_2^T & \cdots & o_m^T \\
o_1^T & L_2(m)^T & \cdots & o_m^T \\
\vdots & \vdots & \ddots & \vdots \\
o_1^T & o_2^T & \cdots & L_m(m)^T
\end{bmatrix}
\eta\le\Delta u^{max}
$$

이다. $m$ 은 제약을 걸고 싶은 미래 시점들의 집합을 돌고, $o_k$ 는 $L_k(m)$ 과 같은 차원의 영벡터다. 가운데 행렬이 [[06_MIMO_Extension|MIMO 확장]]의 블록 대각 $L_i(0)^T$ 행렬을 시점 $m$ 으로 옮긴 것이다.

## Tutorial 3.7 - Mdu.m

제약을 처음 $N_c$ 개 샘플 전부에 건다고 할 때, 부등식

$$
M\eta\le\Delta U^{max},
\qquad
-M\eta\le-\Delta U^{min}
$$

의 데이터 행렬 $M$ 을 만드는 함수다. $\Delta U^{max}$, $\Delta U^{min}$ 은 상한과 하한을 $N_c$ 번 쌓은 벡터다. 입력 개수 $n_{in}$, 제약 샘플 수 $N_c$, 극점 벡터 $a=[a_1\ a_2\ \cdots]$, 항 수 벡터 $N=[N_1\ N_2\ \cdots]$ 를 받는다.

### 첫 블록 ($m=0$)

```matlab
function [M,Lzerot]=Mdu(a,N,n_in,Nc)
%a and N are for the Laguerre functions
%n_in is the number of inputs
%Nc is the number of constraints
N_pa=sum(N);
M=zeros(n_in,N_pa);
M_du1=zeros(n_in,N_pa);
k0=1;
[Al,L0]=lagd(a(k0),N(k0));
M_du1(1,1:N(1))=L0';
cc=N(1);
for k0=2:n_in;
    [Al,L0]=lagd(a(k0),N(k0));
    M_du1(k0,cc+1:cc+N(k0))=L0';
    cc=cc+N(k0);
end
Lzerot=M_du1;
M=M_du1;
```

`M_du1` 은 $n_{in}\times\sum N_i$ 행렬로, 입력 $k_0$ 의 행에 그 입력의 $L_{k_0}(0)^T$ 를 자기 열 블록 위치에 놓는다. 이것이 $m=0$ 의 블록 대각행렬이고, 동시에 [[08_MATLAB_Simulation|simuuc.m]] 이 $\Delta u(k_i)$ 를 뽑는 데 쓰는 `Lzerot` 이다. 그래서 두 번째 출력으로 같이 돌려준다.

### 나머지 블록 ($m=1,\ldots,N_c-1$)

```matlab
for kk=2:Nc
    k0=1;
    [Al,L0]=lagd(a(k0),N(k0));
    L=Al^(kk-1)*L0;
    M_du1(1,1:N(1))=L';
    cc=N(1);
    for k0=2:n_in;
        [Al,L0]=lagd(a(k0),N(k0));
        L=Al^(kk-1)*L0;
        M_du1(k0,cc+1:cc+N(k0))=L';
        cc=cc+N(k0);
    end
    M=[M;M_du1];
end
```

시점 $m=kk-1$ 의 Laguerre 벡터는 $L(m)=A_l^mL(0)$ 이므로 `L=Al^(kk-1)*L0` 로 만들고, 같은 블록 대각 구조로 `M_du1` 을 채운 뒤 `M` 아래에 쌓는다. 완성된 `M` 은 $(n_{in}N_c)\times\sum N_i$ 이고, $j$ 번째 행 블록이 시점 $m=j-1$ 의 제약이다.

교재는 두 경우로 시험하라고 한다. $a=0$, $N=3$, $n_{in}=1$, $N_c=3$ 이면 $L(0)=[1\ 0\ 0]$, $L(1)=[0\ 1\ 0]$, $L(2)=[0\ 0\ 1]$ 이므로 `M` 은 $3\times 3$ 항등행렬이 나와야 한다. 2장의 $\Delta U$ 제약 행렬과 정확히 같다. 두 번째는 $a=[0.2\ 0.3]$, $N=[3\ 3]$, $n_{in}=2$, $N_c=5$ 로, `M` 이 $10\times 6$ 이고 각 행에 0이 아닌 원소가 3개씩 자기 입력 블록에만 있으면 된다.

## Example 3.6 - 감쇠가 작은 2차 시스템

$$
G(s)=\frac{1}{s^2+2\xi\omega s+\omega^2},
\qquad
\xi=0.1,\ \omega=3
$$

을 $\Delta t=0.1$ 로 샘플링하고, $Q=C^TC$ ($C=[0\ 0\ 1]$), $R=0.3$, $N_p=46$, $k_i=10$ 에서 $x(k_i)=[0.1\ 0.2\ 0.3]^T$ 로 둔다. 한 최적화 창 안에서 제약

$$
-1\le\Delta u(k_i+m)\le 0.25
$$

를 만족하는 최적 $\eta$ 와 $\Delta u(k_i+m)$ 을 구한다.

### 무제약 해

감쇠비가 0.1이라 심하게 진동하는 시스템이고, 최적 제어궤적도 진동한다. $a=0.7$, $N=8$ 로 잡으면 무제약 DMPC와 DLQR의 제어신호와 출력이 거의 같다 (Fig. 3.9). 참고로 $a=0$ 인 전통적 방식으로 같은 성능을 내려면 $N$ 이 약 40 필요하다. 파라미터 8개 대 40개다.

### 제약 해

응답 전체에서 제약이 지켜지도록 $\Delta u$ 의 처음 15개 샘플에 제약을 건다. 상한과 하한이므로 부등식은 30개다. 데이터 행렬이 크므로 스크립트로 처리한다.

```matlab
[Omega,Psi]=dmpc(A,B,a,N,Np,Q,R);
Deltau_min=-1;
Deltau_max=0.25;
Nc=15;
X0=[0.1;0.2;0.3];
[M0,Lzerot]=Mdu(a,N,1,Nc);
M=[M0;-M0];
gamma=[Deltau_max*ones(Nc,1);-Deltau_min*ones(Nc,1)];
eta=QPhild(Omega,Psi*X0,M,gamma);
```

`M=[M0;-M0]` 과 `gamma=[Deltau_max;-Deltau_min]` 의 짝이 2장의 표준형 $\begin{bmatrix}M\\-M\end{bmatrix}\eta\le\begin{bmatrix}\Delta U^{max}\\-\Delta U^{min}\end{bmatrix}$ 이다. `QPhild` 는 [[08_MATLAB_QPhild|2장에서 만든 Hildreth 솔버]]다.

결과가 Fig. 3.10이다. 30개 제약 중 **활성 제약은 3개뿐**이고, Fig. 3.10a에서 눈으로 찾을 수 있다. 처음 두 샘플에서 하한 $-1$ 이 활성이고, 17번째 샘플($k_i=10$ 에서 $m=7$)에서 상한 $0.25$ 가 활성이다. 무제약 해의 첫 두 샘플이 약 $-1.41$, $-0.96$ 으로 하한을 넘고, 진동의 첫 양의 봉우리가 $0.34$ 정도로 상한을 넘기 때문이다. Fig. 3.9와 3.10을 비교하면 활성 제약이 생겼는데도 성능 저하는 거의 없다.

이 예제가 말하는 것은 세 가지다.

- 제약을 15개 샘플에 걸었지만 실제로 의미 있는 것은 3개다. 나머지 27개는 비활성이며 해에 영향을 주지 않는다.
- 활성 제약이 어디 생기는지는 무제약 궤적을 보면 대략 예측된다. 궤적이 한계를 넘는 곳이다.
- Laguerre 함수의 감쇠 때문에 15개 이후에는 제약을 걸 필요가 없다. 궤적이 이미 한계 안으로 들어와 있다.

## 2장 방식과의 비교

| 항목 | 2장 ($\Delta U$) | 3장 ($\eta$) |
|---|---|---|
| 제약 행렬 $M$ | 항등행렬 $\begin{bmatrix}I\\-I\end{bmatrix}$ | Laguerre 벡터 $L(m)^T$ 를 쌓은 행렬 |
| 제약 가능한 시점 | $0,\ldots,N_c-1$ 로 고정 | 임의의 $m$ 을 선택 가능 |
| 결정변수 차원 | $N_c$ | $N$ ($N\ll N_c$ 가능) |
| $m\ge N_c$ 이후의 $\Delta u$ | 0으로 강제 | Laguerre 감쇠로 자연히 0에 접근 |

$a=0$, $N=N_c$ 이면 두 열이 완전히 같아진다.

## 연결

- QP 표준형과 Hildreth: [[03_Constrained_MPC_Problem|2장 - 제약 있는 MPC 문제]], [[07_Hildreth_QP|2장 - Hildreth QP]], [[08_MATLAB_QPhild|QPhild.m]]
- 진폭 제약: [[10_Constraints_on_u|u 에 대한 제약과 Mu.m]]
- 같은 시스템의 무제약 튜닝: [[05_Optimal_Trajectory_and_DLQR|Example 3.5]]
- 2장에서 $\Delta u$ 제약을 첫 원소에만 건 예: [[10_Constraints_on_Rate_of_Change|2장 - 변화율 제약]]

## → 검증

- $a=0$, $N=3$, $N_c=3$ 에서 `Mdu` 의 출력이 항등행렬이 되는 이유를 $L(m)$ 의 값으로 설명할 수 있는가?
- Example 3.6에서 30개 제약 중 3개만 활성인 것을 무제약 궤적의 어떤 특징과 연결할 수 있는가?
- Laguerre 파라미터화에서 제약 시점을 자유롭게 고를 수 있는 이유를 한 식으로 말할 수 있는가?
