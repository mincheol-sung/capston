---
title: "10. 제어 진폭 u 에 대한 제약, Mu.m, 제약 충돌"
aliases:
  - "Mu.m"
  - "Tutorial 3.8"
  - "Example 3.7"
  - "Example 3.8"
  - "Example 3.9"
tags:
  - mpc
  - laguerre
  - constraints
  - constraint-conflict
  - matlab
source_pages: "PDF p.38-43 / book p.122-127"
---

# 제어 진폭 u 에 대한 제약, Mu.m, 제약 충돌

제어 진폭 제약은 실무에서 가장 자주 부과되는 제약이므로 데이터 행렬을 정확히 만드는 것이 중요하다. $u$ 는 $\Delta u$ 의 누적합이므로 $\eta$ 로 표현하려면 Laguerre 벡터를 누적해야 한다. 이 노트는 그 행렬 `Mu.m` 과, 두 종류의 제약이 동시에 활성일 때 생기는 충돌을 다룬다.

## 3.6.2 진폭 제약의 $\eta$ 표현

$u^{min}\le u(k)\le u^{max}$ 이고 $u(k)=u(k_i-1)+\sum_{i=0}^{k-1}\Delta u(k_i+i)$ 이므로, 미래 시점 $k=1,2,\ldots$ 에서

$$
u^{min}\le
\begin{bmatrix}
\sum_{i=0}^{k-1}L_1(i)^T & o_2^T & \cdots & o_m^T \\
o_1^T & \sum_{i=0}^{k-1}L_2(i)^T & \cdots & o_m^T \\
\vdots & \vdots & \ddots & \vdots \\
o_1^T & o_2^T & \cdots & \sum_{i=0}^{k-1}L_m(i)^T
\end{bmatrix}
\eta+u(k_i-1)\le u^{max}
$$

이다. $u(k_i-1)$ 은 직전 제어신호이고, $o_k^T$ 는 $L_k(0)^T$ 와 같은 차원의 영 행벡터다. 2장의 누적합 행렬 $C_2$ 가 하던 일을 여기서는 $\sum_iL(i)^T$ 가 한다.

## Tutorial 3.8 - Mu.m

부등식

$$
M\eta\le U^{max}-\bar u(k_i-1),
\qquad
-M\eta\le-U^{min}+\bar u(k_i-1)
$$

의 $M$ 을 만든다. $\bar u(k_i-1)$ 은 과거 $u$ 값을 $N_c$ 번 쌓은 벡터다. 프로그램은 [[09_Constraints_on_Delta_u|Mdu.m]] 에 누적 한 줄을 더한 것이다.

```matlab
function M=Mu(a,N,n_in,Nc)
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
M=M_du1;
Ms=M_du1;
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
Ms=Ms+M_du1;
M=[M;Ms];
end
```

`Mdu.m` 과의 차이는 `Ms=Ms+M_du1; M=[M;Ms];` 두 줄이다. `M_du1` 은 시점 $m$ 의 블록 대각 $L(m)^T$ 행렬이고, `Ms` 는 그것을 $m=0$ 부터 현재까지 누적한 $\sum_{i=0}^{m}L(i)^T$ 다. 첫 행 블록은 $u(k_i)=u(k_i-1)+L(0)^T\eta$ 에 해당하고, `Mdu` 의 첫 블록과 같다.

시험은 $a=0$, $N=3$, $n_{in}=1$, $N_c=3$ 으로 한다. $L(m)$ 이 표준기저이므로 `M` 은 2장의 $C_2=\begin{bmatrix}1&0&0\\1&1&0\\1&1&1\end{bmatrix}$ 이 나와야 한다.

## Example 3.7 - $u$ 에만 제약

[[09_Constraints_on_Delta_u|Example 3.6]]과 같은 시스템($\xi=0.1$, $\omega=3$, $\Delta t=0.1$, $Q=C^TC$, $R=0.3$, $N_p=46$, $x(k_i)=[0.1\ 0.2\ 0.3]^T$, $a=0.7$, $N=8$)에서 $u(k_i-1)=6$ 이라 하고, 모든 미래 샘플에 대해

$$
1.8\le u(k_i+m)\le 4,
\qquad m=0,1,\ldots,46
$$

을 부과한다. 모든 미래 샘플에 걸었으므로 부등식은 $46\times 2=92$ 개다.

```matlab
[Omega,Psi]=dmpc(A,B,a,N,Np,Q,R);
X0=[0.1;0.2;0.3];
up=6;
u_min=1.8;
u_max=4;
M0=Mu(a,N,1,Np);
M=[M0;-M0];
gamma=[(u_max-up)*ones(Nc,1);(-u_min+up)*ones(Nc,1)];
eta=QPhild(Omega,Psi*X0,M,gamma);
```

교재 스크립트는 `M0=Mu(a,N,1,Np)` 로 만들고 `gamma` 는 `ones(Nc,1)` 로 쌓는데, `Nc` 가 이 스크립트 안에서 정의되지 않았다. 92개 제약이 되려면 `Nc=Np` 여야 하므로 그렇게 두고 읽는다.

`gamma` 에 `up` 이 들어가는 것이 진폭 제약의 특징이다. $u^{max}-u(k_i-1)=4-6=-2$, $-u^{min}+u(k_i-1)=-1.8+6=4.2$ 다. 실시간에서는 매 샘플 `up` 이 바뀌므로 `gamma` 를 루프 안에서 다시 만들어야 한다. 2장의 [[11_Constraints_on_Amplitude|Example 2.16]]과 같은 주의점이다.

결과는 Fig. 3.11이다. 활성 제약은 4개이고 모든 제약이 만족된다. $u$ 가 6에서 출발해 상한 4에 닿고, 나중에 하한 1.8 근처에서 다시 활성이 된다.

## Example 3.8 - $\Delta u$ 와 $u$ 를 함께 걸면 충돌한다

Example 3.6의 $-1\le\Delta u\le 0.25$ 와 Example 3.7의 $1.8\le u\le 4$ 를 한꺼번에 부과한다. 따로 걸었을 때는 각각 잘 되었지만, 함께 걸면 **첫 샘플에서 동시에 만족될 수 없다.** 그 결과 최적해가 타협된다.

$N_p=48$ 로 두고 모든 샘플에 걸면 제약은 $4\times N_p=192$ 개다.

```matlab
[Omega,Psi]=dmpc(A,B,a,N,Np,Q,R);
X0=[0.1;0.2;0.3];
up=6;
Deltau_min=-1;
Deltau_max=0.25;
u_min=1.8;
u_max=4;
Nc=Np;
[M1,Lzerot]=Mdu(a,N,1,Nc);
M0=Mu(a,N,1,Nc);
M=[M0;-M0;M1;-M1];
gamma=[(u_max-up)*ones(Nc,1);(-u_min+up)*ones(Nc,1);
Deltau_max*ones(Nc,1); -Deltau_min*ones(Nc,1)];
eta=QPhild(Omega,Psi*X0,M,gamma);
```

`M` 은 $u$ 상한, $u$ 하한, $\Delta u$ 상한, $\Delta u$ 하한의 네 블록이고 `gamma` 도 같은 순서다.

### 왜 충돌하는가 (교재 생략 계산)

첫 샘플만 보자. $u(k_i)=u(k_i-1)+\Delta u(k_i)=6+\Delta u(k_i)$ 다. $u(k_i)\le 4$ 가 되려면 $\Delta u(k_i)\le-2$ 여야 하는데, $\Delta u$ 의 하한은 $-1$ 이다. 즉 $\Delta u(k_i)\le-2$ 와 $\Delta u(k_i)\ge-1$ 을 동시에 만족하는 값이 없다. 두 제약은 모두 $L(0)^T\eta$ 에 대한 것이므로 활성집합에서 선형 종속이 되고, 2장 [[04_QP_Equality_Constraints|Example 2.6]]에서 본 대로 $M_{act}\Omega^{-1}M_{act}^T$ 가 특이행렬이 되어 $\lambda_{act}$ 가 정의되지 않는다.

교재 본문은 이 충돌을 "$u^{min}\ge 1.8$ 과 $\Delta u^{min}\ge-1$ 이 첫 샘플에서 둘 다 활성"이라고 적고 있는데, $u(k_i-1)=6$ 에서 첫 샘플에 부딪히는 것은 하한 1.8이 아니라 **상한 4** 다. 뒤의 Example 3.9에서 $\Delta u^{min}$ 을 $-2$ 로 바꾸자 해결되는 것($6-2=4$)이 이 해석을 뒷받침한다.

### 결과

Fig. 3.12에 $\Delta u$, $u$, $y$ 가 무제약 응답과 함께 있다. Fig. 3.12b를 보면 $u$ 에 대한 제약 일부가 지켜지지 않는다. 승수를 살펴보면 활성 제약이 8개이고, **활성 제약이 충돌하므로 승수가 수렴하지 못한다.** 88회 반복 후의 양수 승수는

$$
\lambda_{act}=
\begin{bmatrix}
34.2944 & 0.0545 & 0.0261 & 0.0654 & 0.0240 & 0.0474 & 34.4705 & 0.0136
\end{bmatrix}
$$

이다. 34 근처의 두 값이 서로 종속인 두 제약(첫 샘플의 $u$ 제약과 $\Delta u$ 제약)에 대응한다. 두 승수가 함께 커지면서 서로 상쇄하려 하는 것이 수렴 실패의 모습이다. 수치적으로 $M_{act}$ 에 행 종속이 있고, 그 결과 제약이 타협된다.

그래도 Hildreth 알고리즘은 멈추지 않는다. 최대 반복 후 타협된 준최적해를 내놓고 시뮬레이션은 계속된다. [[07_Hildreth_QP|2장]]에서 강조한 "역행렬이 없으므로 충돌 상황에서도 계산이 이어진다"는 성질이 여기서 다시 발현된다.

교재의 추가 코멘트는 제약 개수에 관한 것이다. 192개를 걸었지만 활성은 8개뿐이고 나머지는 해에 영향이 없다. 게다가 이동구간 제어에서 실제로 구현되는 것은 첫 샘플 $\Delta u(k_i)=L(0)^T\eta=-1$ 뿐이므로, **이 예제에서 결과에 영향을 준 것은 사실상 첫 샘플의 제약뿐**이다.

## Example 3.9 - $\Delta u^{min}$ 을 $-2$ 로 완화

$\Delta u$ 제약을

$$
-2\le\Delta u(k)\le 0.25
$$

로 바꾸고 $u$ 제약 $1.8\le u(k)\le 4$ 는 그대로 둔다. $u(9)=6$ 에서 $\Delta u=-2$ 이면 $u=4$ 로 상한에 정확히 닿으므로 첫 샘플의 두 제약이 양립한다. 같은 스크립트에서 `Deltau_min=-2` 만 바꾸면 승수가

$$
\begin{bmatrix}
0.2307 & 0.0549 & 0.0242 & 0.0654 & 0.0268 & 0.0480 & 0.0112
\end{bmatrix}
$$

로 수렴한다. 활성 제약은 7개이고 34 같은 큰 값이 사라졌다. Fig. 3.13에서 모든 제약이 만족된다.

Example 3.8과 3.9를 나란히 보면 제약 설계의 원칙이 드러난다. 진폭 제약과 증분 제약을 함께 걸 때는 **초기조건에서 첫 샘플의 두 제약이 양립하는지 먼저 산수로 확인**해야 한다. $|u^{max}-u(k_i-1)|$ 또는 $|u^{min}-u(k_i-1)|$ 이 $\Delta u$ 한계보다 크면 충돌이다.

## 제약을 몇 개나 걸어야 하는가

Laguerre 함수가 지수적으로 감쇠하므로 $\Delta u(k_i+j)$ 에 대한 제약은 어느 초기 구간이 지나면 자동으로 만족된다. 따라서 보통은 **$\Delta u(k_i+j)$ 와 $u(k_i+j)$ 의 처음 몇 개 샘플에만** 제약을 건다. MIMO 시스템에서는 제약 개수가 입력 수에 비례해 빠르게 늘어나므로 더욱 그렇다. 2장의 [[10_Constraints_on_Rate_of_Change|Example 2.15]]에서 본 "미래 제약이 첫 제어값을 왜곡한다"는 문제도 함께 피할 수 있다.

## 연결

- 증분 제약과 `Mdu.m`: [[09_Constraints_on_Delta_u|Δu 에 대한 제약]]
- 제약 충돌의 이론: [[04_QP_Equality_Constraints|2장 - 종속 제약과 실행 불가능]], [[13_Output_Constraints|2장 - 출력 제약과 제약 충돌]]
- 충돌 상황에서의 Hildreth 거동: [[07_Hildreth_QP|2장 - Hildreth QP]]
- 안정성 논의에서 이 예제가 다시 쓰인다 ($N_0=33$): [[11_Stability_Analysis|안정성 해석]]
- 충돌을 피하는 닫힌형 접근 (제약 순위): [[12_Closed_Form_SISO|SISO 닫힌형 해]]

## → 검증

- `Mu.m` 이 `Mdu.m` 과 다른 두 줄이 무엇이고, 왜 그것이 누적합 $C_2$ 에 해당하는지 말할 수 있는가?
- Example 3.8의 충돌을 $u(k_i-1)=6$, $u^{max}=4$, $\Delta u^{min}=-1$ 세 숫자로 설명할 수 있는가?
- 승수 벡터에서 34 근처의 값 두 개가 나타나는 것이 무엇을 뜻하는지 말할 수 있는가?
- 192개 제약 중 실제로 결과에 영향을 준 것이 첫 샘플의 제약뿐인 이유를 이동구간 원리로 설명할 수 있는가?
