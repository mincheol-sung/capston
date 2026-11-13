---
title: "08. MATLAB 구현 - 폐루프 시뮬레이션 simuuc.m / simuob.m"
aliases:
  - "simuuc.m"
  - "simuob.m"
  - "testexa1.m"
  - "Tutorial 3.4"
  - "Tutorial 3.5"
  - "Tutorial 3.6"
tags:
  - mpc
  - matlab
  - simulation
  - observer
source_pages: "PDF p.31-34 / book p.115-118"
---

# MATLAB 구현 - 폐루프 시뮬레이션 simuuc.m / simuob.m

무제약 예측제어의 다변수 폐루프 시뮬레이션 프로그램이다. 제약을 넣기 전에 무제약 성능을 먼저 확인해야 하므로 순서상 여기가 먼저다. 첫 프로그램은 관측기 없이 상태를 직접 쓰고, 세 번째 프로그램은 관측기를 포함한다. 자기 설계 프로그램과 짝을 이루는 자기 시뮬레이션 프로그램을 직접 써 두는 것이 실제 구현에서 큰 도움이 된다는 것이 교재의 권고다.

## Tutorial 3.4 - simuuc.m (관측기 없음)

```matlab
function [u1,y1,deltau1,k]=simuuc(xm,u,y,sp,Ap,Bp,Cp,N_sim,Omega,Psi,Lzerot)
%closed-loop simulation without constraints
[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);
Xf=[xm;(y-sp(:,1))];
for kk=1:N_sim;
eta=-(Omega\Psi)*Xf;
deltau=Lzerot*eta;
u=u+deltau;
deltau1(:,kk)=deltau;
u1(1:n_in,kk)=u;
y1(1:m1,kk)=y;
%plant simulation
xm_old=xm;
xm=Ap*xm+Bp*u;       % calculate xm(k+1)
y=Cp*xm;             % calculate y(k+1)
%updating feedback state variable Xf
Xf=[xm-xm_old;(y-sp(:,kk+1))];
end
k=0:(N_sim-1);
```

입력 인자는 플랜트 초기상태 `xm`, 초기 제어 `u`, 초기 출력 `y`, 설정치 `sp`, 플랜트 모델 `Ap,Bp,Cp`, 시뮬레이션 길이 `N_sim`, [[07_MATLAB_dmpc|dmpc.m]] 이 만든 `Omega,Psi`, 그리고 $\Delta u(k_i)$ 를 $\eta$ 에서 뽑는 블록 대각 `Lzerot` 이다. 설정치 `sp` 는 행 개수가 출력 개수, 열 개수가 `N_sim` 보다 큰 행렬이다.

루프 안의 순서는 이렇다.

1. `eta=-(Omega\Psi)*Xf` : $\eta=-\Omega^{-1}\Psi x(k)$
2. `deltau=Lzerot*eta` : $\Delta u(k)=L(0)^T\eta$ (MIMO에서는 블록 대각)
3. `u=u+deltau` : 증분을 누적해 실제 제어 $u(k)=u(k-1)+\Delta u(k)$
4. 플랜트 한 스텝 전진
5. `Xf=[xm-xm_old;(y-sp(:,kk+1))]` : 다음 피드백 상태

마지막 줄이 이 프로그램의 핵심이다. 피드백에 쓰는 상태는 증강 상태 $x(k)=[\Delta x_m(k)^T\ \ y(k)-r(k)]^T$ 이고, 이것은 [[03_Design_Framework_and_Cost|설정치를 상태벡터에 넣는 방식]]에서 정의한 그대로다. `xm-xm_old` 가 $\Delta x_m(k+1)$, `y-sp(:,kk+1)` 가 $e(k+1)=y(k+1)-r(k+1)$ 이다. 관측기가 없으므로 $\Delta x_m$ 을 플랜트 상태의 차분으로 직접 만든다. 설정치 인덱스가 `kk+1` 인 것은 이 `Xf` 가 다음 스텝 $k+1$ 에서 쓰일 상태이기 때문이다.

## Tutorial 3.5 - testexa1.m 으로 simuuc.m 검증

[[07_MATLAB_dmpc|Tutorial 3.3]]의 `testexa.m` 을 `testexa1.m` 으로 복사하고 뒤에 다음을 붙인다.

```matlab
y=zeros(m1,1);
u=zeros(n_in,1);
xm=zeros(n1,1);
N_sim=100;
r1=ones(1,N_sim+10);
r2=zeros(1,N_sim+10);
r3=zeros(1,N_sim+10);
r4=zeros(1,N_sim+10);
sp=[r1;r2;r3;r4];
[M,Lzerot]=Mdu(a,N,n_in,1);
[u1,y1,deltau1,k]=simuuc(xm,u,y,sp,Ap,Bp,Cp,N_sim,Omega,Psi,Lzerot);
```

초기조건은 모두 0이고, 설정치는 $y_1$ 만 1로 계단 변화시키고 나머지 세 출력은 0으로 유지한다. `sp` 의 열 개수를 `N_sim+10` 으로 넉넉히 잡은 것은 루프 안에서 `sp(:,kk+1)` 을 참조하기 때문이다.

`Lzerot` 은 [[09_Constraints_on_Delta_u|Tutorial 3.7의 Mdu.m]] 이 두 번째 출력으로 돌려주는 행렬이다. `Mdu(a,N,n_in,1)` 처럼 제약 샘플 수를 1로 부르면 첫 출력 `M` 도 같은 행렬이 된다. 순서상 Tutorial 3.7이 뒤에 나오지만, `Mdu.m` 의 앞부분만 있으면 여기서 쓸 수 있다.

`y1` 과 `u1` 을 그리면 Fig. 3.8이다. $y_1$ 이 1로 올라가고 나머지 출력은 잠깐 흔들린 뒤 0으로 돌아오며, 다섯 제어신호가 서로 다른 정상값으로 정착한다. 5입력 4출력이므로 정상상태 입력은 유일하지 않은데, 이 조합은 비용함수가 고른 것이다.

## Tutorial 3.6 - simuob.m (관측기 포함)

관측기가 있으면 피드백 변수가 추정 상태 $\hat x$ 로 바뀐다. 프로그램은 `simuuc.m` 과 거의 같다.

```matlab
function [u1,y1,deltau1,k]=simuob(xm,u,y,sp,Ap,Bp,Cp,A,B,C,N_sim,Omega,Psi,K_ob,Lzerot)
%closed-loop simulation without constraints
[ny,n]=size(C);
[n,nu]=size(B);
X_hat=zeros(n,1);
for kk=1:N_sim;
    Xsp=[zeros(n-ny,1);sp(:,kk)];
    eta=-(Omega\Psi)*(X_hat-Xsp);
    deltau=Lzerot*eta;
    u=u+deltau;  %update u
    deltau1(:,kk)=deltau;
    u1(1:nu,kk)=u;            %keep u
    y1(1:ny,kk)=y;            %keep y
    X_hat=A*X_hat+K_ob*(y-C*X_hat)+B*deltau;
  %u and y  to  generate X_hat(k+1)
    %plant simulation
    xm=Ap*xm+Bp*u;       % calculate xm(k+1)
    y=Cp*xm;             % calculate y(k+1)
end
k=0:(N_sim-1);
```

`simuuc.m` 과 다른 점은 세 가지다.

- 증강 모델 `A,B,C` 와 관측기 이득 `K_ob` 를 추가로 받는다.
- 설정치가 `Xsp=[zeros(n-ny,1);sp(:,kk)]` 로 상태와 같은 차원의 벡터로 들어간다. 이것이 [[03_Design_Framework_and_Cost|$x_r(k_i)$]] 다. `X_hat-Xsp` 를 하면 마지막 성분이 $\hat y-r$ 이 되어 `simuuc.m` 의 `Xf` 와 같은 역할을 한다.
- 상태 갱신이 `X_hat=A*X_hat+K_ob*(y-C*X_hat)+B*deltau` 로 관측기 방정식이 된다. 플랜트 상태 `xm` 은 출력을 만드는 데만 쓰이고 제어기는 보지 못한다.

설정치 `sp` 가 시뮬레이션에 들어가는 방식은 실제 구현에서도 그대로 쓸 수 있다는 것이 교재의 지적이다. `sp` 는 출력 개수만큼의 행과 시뮬레이션 길이보다 많은 열을 가진 신호이므로, 실시간에서는 매 샘플 현재 설정치 한 열만 읽으면 된다.

## 제약 제어로 가는 한 줄 수정

이 프로그램에 제약을 넣는 데 필요한 수정은 작다. $X_f=\hat x-X_{sp}$ 로 두고

```matlab
eta=QPhild(Omega,Psi*Xf,M,gamma);
```

로 `eta` 계산 한 줄만 바꾸면 된다. `M` 과 `gamma` 는 [[09_Constraints_on_Delta_u|3.6절]]에서 정의한다. [[08_MATLAB_QPhild|2장의 QPhild.m]] 은 비용함수를 $\frac12\eta^TH\eta+\eta^Tf$ 로 받는데, 여기 비용함수 $\eta^T\Omega\eta+2\eta^T\Psi x$ 는 이것의 2배이므로 최소점이 같다. 그래서 `H=Omega`, `f=Psi*Xf` 로 그냥 넘겨도 된다. 연속시간 제약 제어 튜토리얼(7.3절)이 같은 수정 절차를 보여 준다.

## 연결

- `Omega`, `Psi` 를 만드는 코드: [[07_MATLAB_dmpc|dmpc.m]]
- `Lzerot` 을 만드는 코드: [[09_Constraints_on_Delta_u|Mdu.m]]
- 제약 QP 솔버: [[08_MATLAB_QPhild|2장 - QPhild.m]]
- 관측기 설계: [[06_State_Estimation|1장 - 상태 추정]], [[07_State_Estimate_MPC|1장 - 관측기를 쓴 MPC]]
- 관측기를 쓰지 않는 상태 선택: [[13_MATLAB_DC_Motor|DC 모터 튜토리얼]]

## → 검증

- `Xf=[xm-xm_old;(y-sp(:,kk+1))]` 의 두 블록이 증강 상태의 어느 성분인지, 설정치 인덱스가 왜 `kk+1` 인지 말할 수 있는가?
- `simuob.m` 에서 `Xsp` 를 빼는 것이 `simuuc.m` 의 어떤 연산과 같은지 설명할 수 있는가?
- 제약을 넣을 때 `QPhild` 에 `Omega` 와 `Psi*Xf` 를 그대로 넘겨도 되는 이유를 말할 수 있는가?
