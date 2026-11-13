---
title: "01. 이산시간 Laguerre 함수"
aliases:
  - "Discrete-time Laguerre Networks"
  - "Laguerre 함수"
  - "Example 3.1"
tags:
  - mpc
  - laguerre
  - orthonormal
  - basis-function
source_pages: "PDF p.1-5 / book p.85-89"
---

# 이산시간 Laguerre 함수

3장의 출발점은 "미래 제어궤적 $\Delta u(k_i+m)$ 을 어떤 기저(basis)로 표현할 것인가"라는 질문이다. 1장과 2장은 이 궤적을 $N_c$ 개의 펄스로 표현했고, 3장은 이것을 **직교정규(orthonormal) Laguerre 함수**로 바꾼다. 이 노트는 그 함수의 정의, 상태공간 표현, 직교정규성을 정리한다.

## 왜 기저를 바꾸는가

1장과 2장에서 최적화하는 벡터는

$$
\Delta U=
\begin{bmatrix}
\Delta u(k_i) & \Delta u(k_i+1) & \cdots & \Delta u(k_i+N_c-1)
\end{bmatrix}^T
$$

이고, $k=N_c,\ldots,N_p$ 에 대해서는 $\Delta u(k)=0$ 이라고 가정한다. $\Delta U$ 의 각 원소는 이산 $\delta$ 함수를 써서

$$
\Delta u(k_i+i)=
\begin{bmatrix}
\delta(i) & \delta(i-1) & \cdots & \delta(i-N_c+1)
\end{bmatrix}
\Delta U
$$

로 쓸 수 있다. 여기서 $\delta(i)=1$ ($i=0$), $\delta(i)=0$ ($i\ne 0$) 이고 $\delta(i-d)$ 는 펄스를 $d$ 만큼 뒤로 민 것이다. 즉 지금까지의 설계는 **펄스 연산자(pulse operator)** 를 기저로 쓰고 $\Delta U$ 를 그 계수 벡터로 본 것과 같다.

문제는 두 가지다. 첫째, 앞선 예제들에서 $N_c$ 이후에 버린 $\Delta u$ 가 0은 아니었다. 크기가 작을 뿐이었다. 둘째, 샘플링이 빠르거나 동역학이 복잡하거나 폐루프 성능 요구가 높으면 $\Delta u$ 를 제대로 표현하기 위해 매우 많은 파라미터가 필요해지고, 수치적으로 나쁜 조건의 해와 무거운 온라인 계산으로 이어진다.

펄스 대신 **감쇠하는 직교정규 함수 집합**으로 궤적을 근사하면, 긴 제어구간을 적은 수의 파라미터로 실현할 수 있다. 이것이 Laguerre 함수를 쓰는 이유이며, 그 근거는 [[02_Laguerre_System_Modelling|다음 노트]]의 임펄스 응답 근사 실험에서 눈으로 확인한다.

## z-영역 정의

이산시간 Laguerre 네트워크의 z-변환은

$$
\Gamma_1(z)=\frac{\sqrt{1-a^2}}{1-az^{-1}}
$$

$$
\Gamma_2(z)=\frac{\sqrt{1-a^2}}{1-az^{-1}}\cdot\frac{z^{-1}-a}{1-az^{-1}}
$$

$$
\Gamma_N(z)=\frac{\sqrt{1-a^2}}{1-az^{-1}}\left(\frac{z^{-1}-a}{1-az^{-1}}\right)^{N-1}
$$

이다. $a$ 는 네트워크의 극점이고 안정성을 위해 $0\le a<1$ 이다. 사용자가 고르는 자유 파라미터이며 **스케일링 인자(scaling factor)** 라고도 부른다.

이웃한 항 사이에는

$$
\Gamma_k(z)=\Gamma_{k-1}(z)\,\frac{z^{-1}-a}{1-az^{-1}}
$$

의 관계가 있다. 즉 1차 저역통과 필터 뒤에 전역통과(all-pass) 필터 $\frac{z^{-1}-a}{1-az^{-1}}$ 를 직렬로 계속 붙인 구조다 (Fig. 3.1). 전역통과 필터는 크기 응답이 1이므로, 항이 늘어나도 각 $\Gamma_k$ 의 크기 응답은 같고 위상만 달라진다. 이것이 직교정규성의 주파수 영역 근거다.

주파수 영역에서 직교정규성은

$$
\frac{1}{2\pi}\int_{-\pi}^{\pi}\Gamma_m(e^{jw})\Gamma_m(e^{jw})^*dw=1
$$

$$
\frac{1}{2\pi}\int_{-\pi}^{\pi}\Gamma_m(e^{jw})\Gamma_n(e^{jw})^*dw=0 \qquad m\ne n
$$

로 표현된다. $A^*$ 는 복소켤레다.

## 시간 영역 - 상태공간 표현

예측제어 설계에서는 시간 영역 함수 $l_k(n)$ 이 필요하다. 역 z-변환을 직접 하면 간결한 식이 나오지 않으므로, 네트워크의 상태공간 실현을 쓴다.

$\Gamma_1$ 의 역변환을 $l_1(k)$, $\Gamma_2$ 의 역변환을 $l_2(k)$ 라 하고 벡터로 묶으면

$$
L(k)=
\begin{bmatrix}
l_1(k) & l_2(k) & \cdots & l_N(k)
\end{bmatrix}^T
$$

이고, 이 벡터는 차분방정식

$$
L(k+1)=A_lL(k)
$$

를 만족한다. $A_l$ 은 $N\times N$ 행렬로 $a$ 와 $\beta=1-a^2$ 의 함수이며, 초기조건은

$$
L(0)^T=\sqrt{\beta}
\begin{bmatrix}
1 & -a & a^2 & -a^3 & \cdots & (-1)^{N-1}a^{N-1}
\end{bmatrix}
$$

이다. $N=5$ 이면

$$
A_l=
\begin{bmatrix}
a & 0 & 0 & 0 & 0 \\
\beta & a & 0 & 0 & 0 \\
-a\beta & \beta & a & 0 & 0 \\
a^2\beta & -a\beta & \beta & a & 0 \\
-a^3\beta & a^2\beta & -a\beta & \beta & a
\end{bmatrix},
\qquad
L(0)=\sqrt{\beta}
\begin{bmatrix}
1 \\ -a \\ a^2 \\ -a^3 \\ a^4
\end{bmatrix}
$$

### $A_l$ 이 왜 이런 모양인지 (교재 생략 유도)

교재는 결과만 주는데, 유도는 짧다. $\Gamma_k(z)(1-az^{-1})=\Gamma_{k-1}(z)(z^{-1}-a)$ 를 시간 영역으로 옮기면

$$
l_k(n)-al_k(n-1)=l_{k-1}(n-1)-al_{k-1}(n)
$$

이고, $n$ 을 하나 밀어 정리하면

$$
l_k(n+1)=al_k(n)+l_{k-1}(n)-al_{k-1}(n+1)
$$

이다. $k=1$ 은 분모만 있으므로 $l_1(n+1)=al_1(n)$ 이고, 이것이 $A_l$ 의 1행 $[a\ 0\ \cdots]$ 이다. $k=2$ 에 대입하면

$$
l_2(n+1)=al_2(n)+l_1(n)-a\cdot al_1(n)=\beta\,l_1(n)+a\,l_2(n)
$$

으로 2행 $[\beta\ a\ 0\ \cdots]$ 이 나온다. $k=3$ 은

$$
l_3(n+1)=al_3(n)+l_2(n)-a\big(\beta l_1(n)+al_2(n)\big)=-a\beta\,l_1(n)+\beta\,l_2(n)+a\,l_3(n)
$$

으로 3행 $[-a\beta\ \beta\ a\ 0\cdots]$ 이다. 한 행 내려갈 때마다 앞 행을 오른쪽으로 한 칸 밀고 첫 열에 $(-a)$ 를 한 번 더 곱한 값이 붙는다. 그래서 $A_l$ 은 첫 열만 만들면 나머지는 그 열을 아래로 밀어서 얻는 **Toeplitz 하삼각 행렬**이 된다. [[02_Laguerre_System_Modelling|lagd.m]] 이 정확히 그렇게 코딩되어 있다.

초기조건도 같은 식으로 나온다. z-변환의 초기값 정리 $l(0)=\lim_{z\to\infty}\Gamma(z)$ 에서 $\Gamma_1(\infty)=\sqrt{\beta}$ 이고, 전역통과 인자는 $z\to\infty$ 에서 $-a$ 로 가므로 $l_k(0)=\sqrt{\beta}(-a)^{k-1}$ 이다.

## 시간 영역 직교정규성

주파수 영역의 성질은 시간 영역에서

$$
\sum_{k=0}^{\infty}l_i(k)l_j(k)=0 \qquad i\ne j
$$

$$
\sum_{k=0}^{\infty}l_i(k)l_j(k)=1 \qquad i=j
$$

로 나타난다. 이 성질이 [[03_Design_Framework_and_Cost|비용함수]]에서 $\Delta U^T\bar R\Delta U$ 를 $\eta^TR_L\eta$ 로 바꾸는 데 쓰인다. 직교정규가 아니라면 계수 벡터 $\eta$ 에 대한 가중치가 대각행렬이 되지 않고, 뒤에서 보는 모든 간결한 식이 무너진다.

## 특수한 경우 $a=0$

$a=0$ 이면 $\beta=1$ 이고

$$
A_l=
\begin{bmatrix}
0 & 0 & 0 & \cdots & 0 & 0 \\
1 & 0 & 0 & \cdots & 0 & 0 \\
0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 1 & \cdots & 0 & 0 \\
\vdots & \ddots & \ddots & \ddots & \ddots & \vdots \\
0 & \cdots & 0 & 0 & 1 & 0
\end{bmatrix},
\qquad
L(0)^T=
\begin{bmatrix}
1 & 0 & 0 & \cdots & 0
\end{bmatrix}
$$

이다. $A_l$ 은 한 칸씩 아래로 미는 시프트 행렬이므로 $l_1(k)=\delta(k)$, $l_2(k)=\delta(k-1)$, $\ldots$, $l_N(k)=\delta(k-N+1)$ 이 된다. 즉 **$a=0$ 이면 Laguerre 함수는 펄스 집합으로 퇴화**하고, $N=N_c$ 로 두면 1장과 2장의 설계와 완전히 같아진다. 3장의 방법은 앞 장의 일반화이지 대체가 아니라는 뜻이다.

## Example 3.1 - $a=0.5$ 와 $a=0.9$ 의 비교

$a$ 가 0.5 와 0.9 인 두 경우에 대해 처음 세 Laguerre 함수를 만들고, 유한합으로 직교정규성을 확인한다.

처음 세 함수의 차분방정식은

$$
\begin{bmatrix}
l_1(k+1) \\ l_2(k+1) \\ l_3(k+1)
\end{bmatrix}
=
\begin{bmatrix}
a & 0 & 0 \\
1-a^2 & a & 0 \\
-a(1-a^2) & 1-a^2 & a
\end{bmatrix}
\begin{bmatrix}
l_1(k) \\ l_2(k) \\ l_3(k)
\end{bmatrix}
$$

이고 초기조건은 $l_1(0)=\sqrt{1-a^2}$, $l_2(0)=-a\sqrt{1-a^2}$, $l_3(0)=a^2\sqrt{1-a^2}$ 다. 이를 반복 계산한 것이 Fig. 3.2다.

관찰은 다음과 같다.

- $a=0.5$ 이면 15 샘플 안에 0으로 감쇠한다. $a=0.9$ 이면 약 50 샘플이 걸린다. 극점 $a$ 가 감쇠 속도를 결정한다.
- $a$ 가 작을수록 초기값이 크다. 특히 첫 함수가 그렇다. $l_1(0)=\sqrt{1-a^2}$ 이므로 $a=0.5$ 이면 0.866, $a=0.9$ 이면 0.436 이다.

유한합

$$
S_1=\sum_{k=0}^{19}L(k)L(k)^T \ (a=0.5),
\qquad
S_2=\sum_{k=0}^{50}L(k)L(k)^T \ (a=0.9)
$$

를 계산하면

$$
S_1=
\begin{bmatrix}
1.0000 & -0.0000 & -0.0000 \\
-0.0000 & 1.0000 & -0.0000 \\
-0.0000 & -0.0000 & 1.0000
\end{bmatrix},
\qquad
S_2=
\begin{bmatrix}
1.0000 & -0.0003 & -0.0012 \\
-0.0003 & 0.9970 & -0.0129 \\
-0.0012 & -0.0129 & 0.9430
\end{bmatrix}
$$

이다. $S_1$ 은 4자리까지 항등행렬이지만 $S_2$ 는 아니다. $a=0.9$ 에서 샘플 수를 90으로 늘리면

$$
S_2=\sum_{k=0}^{90}L(k)L(k)^T=
\begin{bmatrix}
1.0000 & -0.0000 & -0.0000 \\
-0.0000 & 1.0000 & -0.0000 \\
-0.0000 & -0.0000 & 0.9998
\end{bmatrix}
$$

로 항등행렬에 훨씬 가까워진다.

이 예제가 말하는 것은 하나다. **직교정규성은 무한합에서 성립하는 성질이고, 유한합으로 그것을 흉내 내려면 함수가 충분히 감쇠할 만큼 긴 구간이 필요하다.** 그 구간의 길이는 $a$ 와 $N$ 에 의존한다. 뒤에서 예측구간 $N_p$ 를 고를 때 이 사실이 그대로 기준이 된다. $N_p$ 가 Laguerre 함수의 감쇠 구간보다 짧으면 비용함수에서 쓴 직교정규 근사가 깨진다.

## 연결

- 이 함수로 임펄스 응답을 근사하는 실험: [[02_Laguerre_System_Modelling|Laguerre 네트워크를 이용한 시스템 표현]]
- 이 함수로 $\Delta u$ 궤적을 표현하는 설계: [[03_Design_Framework_and_Cost|설계 프레임워크와 비용함수]]
- 앞 장의 펄스 기저: [[03_One_Window_Optimization|1장 - 한 최적화 창에서의 예측제어]]
- 용어: [[MPC3_Glossary|MPC 3장 용어집]]

## → 검증

- $a=0.5$, $N=3$ 일 때 $A_l$ 과 $L(0)$ 을 손으로 쓸 수 있는가? ($A_l$ 의 2행은 $[0.75\ 0.5\ 0]$, 3행은 $[-0.375\ 0.75\ 0.5]$, $L(0)=[0.866\ -0.433\ 0.217]^T$)
- $a=0$ 이면 왜 1장의 $\Delta U$ 설계와 같아지는지 $A_l$ 의 모양으로 설명할 수 있는가?
- 유한합 $S_2$ 가 항등행렬이 아닌 이유를 "감쇠 구간"이라는 말로 설명할 수 있는가?
