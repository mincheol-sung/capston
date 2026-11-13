---
title: "07. MATLAB 구현 - dmpc.m 과 testexa.m"
aliases:
  - "dmpc.m"
  - "Tutorial 3.2"
  - "Tutorial 3.3"
  - "testexa.m"
tags:
  - mpc
  - matlab
  - laguerre
  - implementation
source_pages: "PDF p.24-30 / book p.108-114"
---

# MATLAB 구현 - dmpc.m 과 testexa.m

교재가 "가장 중요한 튜토리얼 중 하나"라고 부르는 부분이다. 여기서 만드는 `dmpc.m` 은 이산시간 시스템을 다루는 이후 모든 장에서 재사용된다. 함수는 다입력 시스템에 대해 비용함수

$$
J=\eta^TE\eta+2\eta^THx(k_i)
$$

의 데이터 행렬 $E$ 와 $H$ 를 만든다. 이 노트의 표기로는 $E=\Omega$, $H=\Psi$ 다. 상태 가중 $Q$ 와 입력 가중 $R$ 을 받는다.

## Tutorial 3.2 - dmpc.m

### 1단계 - 차원 설정과 $R$ 의 변환

```matlab
function [E,H]=dmpc(A_e,B_e,a,N,Np,Q,R);
%A_e;B_e define the extended state-space model when
% integrator is used
%they can also be other forms of state-space models
% a contains the Laguerre pole locations for each input
%N  the number of terms for each input
%Np prediction horizon
%Q weight on the state variables
%R weight on the input variables assumed to be diagonal.
% The cost function is J= eta ^T E eta +2 eta ^T H x(k_i)
[n,n_in]=size(B_e);
N_pa=sum(N); %the dimension of eta
E=zeros(N_pa,N_pa);
H=zeros(N_pa,n);
R_para=zeros(N_pa,N_pa);
n0=1;
ne=N(1);
for i=1:n_in-1;
R_para(n0:ne,n0:ne)= R(i,i)*eye(N(i),N(i));
n0=n0+N(i);
ne=ne+N(i+1);
end
R_para(n0:N_pa,n0:N_pa)=R(n_in,n_in)*eye(N(n_in),N(n_in));
```

`a` 와 `N` 은 입력 개수만큼의 벡터다. `N_pa=sum(N)` 이 $\eta$ 의 차원이다. `R_para` 는 [[06_MIMO_Extension|MIMO 확장]]의 블록 대각 $R_L$ 로, 입력 $i$ 의 가중 `R(i,i)` 를 그 입력의 계수 $N_i$ 개에 똑같이 붙인다. 마지막 입력은 루프 밖에서 따로 처리하는데, 루프 안에서 `ne=ne+N(i+1)` 로 다음 블록의 끝을 미리 계산하기 때문에 마지막 입력에서는 `N(i+1)` 이 없기 때문이다.

### 2단계 - $m=1$ 항의 준비

```matlab
S_in=zeros(n,N_pa);
[Al,L0]=lagd(a(1),N(1));
S_in(:,1:N(1))=B_e(:,1)*L0';
In_s=1;
for jj=2:n_in;
[Al,L0]=lagd(a(jj),N(jj));
In_s=N(jj-1)+In_s;
In_e=In_s+N(jj)-1;
S_in(:,In_s:In_e)=B_e(:,jj)*L0';
end
S_sum=S_in;
phi=S_in;
E=(phi)'*Q*(phi);
H=phi'*Q*A_e;
```

`S_in` 은 $S_c(1)=[B_1L_1(0)^T\ \ B_2L_2(0)^T\ \cdots]$ 이다. 입력 $jj$ 의 블록은 열 `In_s:In_e` 에 들어가고, `B_e(:,jj)*L0'` 가 $B_jL_j(0)^T$ ($n\times N_j$) 다. `phi` 가 $\phi(1)^T$ 이므로 `E` 와 `H` 의 첫 항 $\phi(1)Q\phi(1)^T$, $\phi(1)QA$ 를 여기서 넣는다.

### 3단계 - $m=2,\ldots,N_p$ 의 재귀 누적

```matlab
for i=2:Np;
Eae=A_e^i;
%calculate the finite sum S for each input
%For each sample i
%calculate input number 1
%specify the L0 and state matrix Al
%associated with the first input
[Al,L0]=lagd(a(1),N(1));
% Laguerre function associated with input number 1
S_sum(:,1:N(1))=A_e*S_sum(:,1:N(1))+...
S_in(:,1:N(1))*(Al^(i-1))';
%move on to input number 2 and so on
In_s=1;
for kk=2:n_in;
[Al,L0]=lagd(a(kk),N(kk));
In_s=N(kk-1)+In_s;
In_e=In_s+N(kk)-1;
S_sum(:,In_s:In_e)=A_e*S_sum(:,In_s:In_e)+...
S_in(:,In_s:In_e)*(Al^(i-1))';
end
phi=S_sum;
E=E+phi'*Q*phi;
H=H+phi'*Q*Eae;
end
E=E+R_para;
```

교재 코드는 줄바꿈 없이 이어져 있지만 내용은 위와 같다. 바깥 루프 `i` 는 예측구간, 안쪽 루프 `kk` 는 입력 번호다.

| 코드 | 수식 |
|---|---|
| `Eae=A_e^i` | $A^m$ |
| `S_sum(:,blk)=A_e*S_sum(:,blk)+S_in(:,blk)*(Al^(i-1))'` | $S_c(m)=AS_c(m-1)+S_c(1)(A_l^{m-1})^T$, 입력 블록별 |
| `phi=S_sum` | $\phi(m)^T$ |
| `E=E+phi'*Q*phi` | $\Omega\leftarrow\Omega+\phi(m)Q\phi(m)^T$ |
| `H=H+phi'*Q*Eae` | $\Psi\leftarrow\Psi+\phi(m)QA^m$ |
| `E=E+R_para` | 마지막에 $R_L$ 을 더함 |

`S_sum` 은 누적된 합성곱 합, `S_in` 은 $S_c(1)$ 을 보관해 두고 매번 $A_l^{m-1}$ 을 곱해 새 항을 만드는 데 쓴다. [[04_Minimization_and_Convolution_Sum|합성곱 합]]의 재귀식이 그대로 코드가 된 것이다.

이 프로그램은 입력, 상태, 출력 개수가 임의인 시스템에 대해 예측제어 비용함수를 만든다. 매우 중요하므로 다른 응용에 쓰기 전에 반드시 검증해야 하고, 교재는 `dlqr` 결과와 비교하는 방식으로 검증한다.

## Tutorial 3.3 - testexa.m 으로 검증

### 대상 시스템

메틸 메타크릴레이트와 비닐 아세테이트의 공중합 연속교반탱크반응기(CSTR) 모델이다 (Rao et al. 1998). 5입력 4출력 전달함수 행렬

$$
G(s)=
\begin{bmatrix}
g_{11} & g_{12} & g_{13} & g_{14} & g_{15} \\
g_{21} & g_{22} & g_{23} & g_{24} & g_{25} \\
g_{31} & g_{32} & g_{33} & g_{34} & g_{35} \\
g_{41} & g_{42} & g_{43} & g_{44} & g_{45}
\end{bmatrix}
$$

이고 각 원소는

$$
g_{11}=\frac{0.34}{0.85s+1},\quad
g_{12}=\frac{0.21}{0.42s+1},\quad
g_{13}=\frac{0.50(0.50s+1)}{12s^2+0.4s+1},\quad
g_{14}=0,\quad
g_{15}=\frac{6.46(0.9s+1)}{0.07s^2+0.3s+1}
$$

$$
g_{21}=\frac{-0.41}{2.41s+1},\quad
g_{22}=\frac{0.66}{1.51s+1},\quad
g_{23}=\frac{-0.3}{1.45s+1},\quad
g_{24}=0,\quad
g_{25}=\frac{-3.72}{0.8s+1}
$$

$$
g_{31}=\frac{0.3}{2.54s+1},\quad
g_{32}=\frac{0.49}{1.54s+1},\quad
g_{33}=\frac{-0.71}{1.35s+1},\quad
g_{34}=\frac{-0.2}{2.72s+1},\quad
g_{35}=\frac{-4.71}{0.008s^2+0.41s+1}
$$

$$
g_{41}=g_{42}=g_{43}=g_{44}=0,\quad
g_{45}=\frac{1.02}{0.07s^2+0.31s+1}
$$

이다. 정규화된 입력은 단량체 MMA 유량 $u_1$, 단량체 VA 유량 $u_2$, 개시제 $u_3$, 연쇄이동제 $u_4$, 반응기 재킷 온도 $u_5$ 이고, 출력은 중합체 생산율 $y_1$, 중합체 내 MMA 몰분율 $y_2$, 평균 분자량 $y_3$, 반응기 온도 $y_4$ 다. 입출력 개수가 다르고 0인 원소가 많아 프로그램의 일반성을 시험하기 좋다.

### 1~3단계 - 전달함수 입력

`testexa.m` 을 만들고 분자와 분모를 하나씩 지정한다. 첫 행만 옮기면 다음과 같고, 나머지 행도 같은 방식이다. 0인 원소는 분자 0, 분모 1로 둔다.

```matlab
%First row
n11=0.34;   d11=[0.85 1];
n12=0.21;   d12=[0.42 1];
n13=0.50*[0.50 1];   d13=[12 0.4 1];
n14=0;      d14=1;
n15=6.46*[0.9 1];    d15=[0.07 0.3 1];
%second row
n21=-0.41;  d21=[2.41 1];
n22=0.66;   d22=[1.51 1];
n23=-0.3;   d23=[1.45 1];
n24=0;      d24=1;
n25=-3.72;  d25=[0.8 1];
%third row
n31=0.3;    d31=[2.54 1];
n32=0.49;   d32=[1.54 1];
n33=-0.71;  d33=[1.35 1];
n34=-0.20;  d34=[2.71 1];
n35=-4.71;  d35=[0.008 0.41 1];
%fourth row
n41=0; d41=1;  n42=0; d42=1;  n43=0; d43=1;  n44=0; d44=1;
n45=1.02;   d45=[0.07 0.31 1];
```

교재 본문의 $g_{34}$ 분모는 $2.72s+1$ 인데 코드에는 `d34=[2.71 1]` 로 되어 있다. 원문의 불일치이므로 어느 쪽을 쓰든 결과에 큰 차이는 없다.

### 4단계 - MIMO 전달함수, 최소 실현, 이산화

```matlab
h=1;  %sampling interval
Gs=tf({n11 n12 n13 n14 n15;n21 n22 n23 n24 n25;
n31 n32 n33 n34 n35;n41 n42 n43 n44 n45},
{d11 d12 d13 d14 d15;d21 d22 d23 d24 d25;
d31 d32 d33 d34 d35;d41 d42 d43 d44 d45});
Gsmin=ss(Gs,'min');
[Ac,Bc,Cc,Dc]=ssdata(Gsmin);
[Ap,Bp,Cp,Dp]=c2dm(Ac,Bc,Cc,Dc,h,'zoh');
[m1,n1]=size(Cp);
[n1,n_in]=size(Bp);
```

`ss(Gs,'min')` 이 최소 실현을 만든다. 전달함수 행렬을 그대로 상태공간으로 바꾸면 중복된 상태가 많이 생기므로 꼭 필요한 단계다. `c2dm` 으로 영차홀드 이산화한다.

### 5단계 - 입력별 Laguerre 파라미터

```matlab
a1=0.5; a2=0.5; a3=0.5; a4=0.5; a5=0.5;
N1=10; N2=10; N3=10; N4=10; N5=10;
a=[a1 a2 a3 a4 a5];
N=[N1 N2 N3 N4 N5];
Np=100;
```

다섯 입력 모두 $a_i=0.5$, $N_i=10$ 이므로 $\eta$ 의 차원은 50이다. $N_p=100$ 은 $a=0.5$ 의 감쇠 구간(약 15 샘플)보다 훨씬 길어 직교정규 근사에 여유가 있다.

### 6단계 - 적분기 증강과 가중행렬

```matlab
A_e=eye(n1+m1,n1+m1);
A_e(1:n1,1:n1)=Ap;
A_e(n1+1:n1+m1,1:n1)=Cp*Ap;
B_e=zeros(n1+m1,n_in);
B_e(1:n1,:)=Bp;
B_e(n1+1:n1+m1,:)=Cp*Bp;
C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);
Q=C_e'*C_e;
R=0.1*eye(n_in,n_in);
```

[[02_Augmented_State_Space_Model|1장의 증강 모델]] $A=\begin{bmatrix}A_p & 0\\ C_pA_p & I\end{bmatrix}$, $B=\begin{bmatrix}B_p\\ C_pB_p\end{bmatrix}$, $C=[0\ \ I]$ 를 그대로 조립한 것이다. `A_e` 를 항등행렬로 시작해 좌상단과 좌하단만 덮어쓰면 우하단의 $I$ 가 자동으로 남는다. $Q=C_e^TC_e$ 이므로 출력오차만 벌점을 받고, $R=0.1I$ 다.

### 7단계 - dmpc 호출과 피드백 이득

```matlab
[Omega,Psi]=dmpc(A_e,B_e,a,N,Np,Q,R);
L_m=zeros(n_in,sum(N));
[Al,L0]=lagd(a(1),N(1));
L_m(1,1:N(1))=L0';
In_s=1;
for jj=2:n_in;
[Al,L0]=lagd(a(jj),N(jj));
In_s=N(jj-1)+In_s;
In_e=In_s+N(jj)-1;
L_m(jj,In_s:In_e)=L0';
end
K=L_m*(Omega\Psi);
Acl=A_e-B_e*K;
```

`L_m` 이 [[06_MIMO_Extension|MIMO 확장]]의 블록 대각 $L_i(0)^T$ 행렬이고, `K=L_m*(Omega\Psi)` 가 $K_{mpc}$, `Acl` 이 폐루프 시스템행렬이다.

### 9~11단계 - DLQR과 비교

```matlab
[X,Y,Z]=dlqr(A_e,B_e,Q,R);
figure
plot(Z,'ro')
hold on
plot(eig(Acl),'b*')
```

`dlqr` 의 세 번째 출력 `Z` 가 DLQR 폐루프 고유값이다. 이것과 `eig(Acl)` 을 복소평면에 겹쳐 그린다. Fig. 3.7a는 $a=0.5$ 에서 두 고유값 집합이 거의 일치함을, Fig. 3.7b는 $a=0$ (펄스 연산자)에서 차이가 생김을 보여 준다. 같은 $N_i=10$ 이라도 펄스 10개는 이 플랜트의 최적 궤적을 잡기에 부족하다는 뜻이다.

**두 고유값 집합이 거의 일치하면 프로그램이 검증된 것이다.** $a$ 와 $N$ 을 바꿔 가며 고유값이 어떻게 움직이는지 보는 것도 이 튜토리얼의 목적이다.

## 왜 dlqr 과의 비교가 검증이 되는가

[[05_Optimal_Trajectory_and_DLQR|앞 노트]]에서 본 대로, $N$ 이 충분히 크고 $N_p$ 가 충분히 길면 DMPC의 무제약 해는 같은 $(Q,R)$ 의 DLQR로 수렴한다. `dlqr` 은 MATLAB이 제공하는 신뢰할 수 있는 기준이므로, 폐루프 고유값이 겹치면 `dmpc.m` 이 $\Omega$ 와 $\Psi$ 를 올바르게 만들었다고 결론 내릴 수 있다. 반대로 $a=0$ 에서 차이가 나는 것은 오류가 아니라, 펄스 기저의 표현력 한계를 보여 주는 정상적인 결과다.

## 연결

- 이론: [[04_Minimization_and_Convolution_Sum|비용함수 최소화와 합성곱 합]], [[06_MIMO_Extension|MIMO 확장]]
- `lagd.m`: [[02_Laguerre_System_Modelling|Laguerre 네트워크를 이용한 시스템 표현]]
- 이 `Omega`, `Psi` 를 받아 폐루프를 돌리는 코드: [[08_MATLAB_Simulation|simuuc.m / simuob.m]]
- 제약 제어에서의 사용: [[09_Constraints_on_Delta_u|Example 3.6]]
- 1장의 대응 함수 `mpcgain.m`: [[09_MATLAB_Workflows|1장 - MATLAB 워크플로]]

## → 검증

- `S_in` 과 `S_sum` 의 역할 차이를 재귀식의 두 항과 연결해 말할 수 있는가?
- `R_para` 가 블록 대각행렬이 되는 이유를 직교정규성으로 설명할 수 있는가?
- `dmpc.m` 의 출력 `E`, `H` 가 이론의 어느 기호에 해당하고, 비용함수 $J=\eta^TE\eta+2\eta^THx$ 의 최소해가 무엇인지 쓸 수 있는가?
- Fig. 3.7에서 $a=0$ 일 때 고유값이 어긋나는 것이 왜 프로그램 오류가 아닌지 설명할 수 있는가?
