---
title: "07. Hildreth 이차계획법"
aliases:
  - "Hildreth QP"
  - "Hildreth Quadratic Programming"
  - "Example 2.10"
  - "Example 2.11"
tags:
  - mpc
  - qp
  - hildreth
  - algorithm
source_pages: "PDF p.21-25 / book p.63-67"
---

# Hildreth 이차계획법

Luenberger(1969)와 Wismer, Chattergy(1978)가 제시한 알고리즘으로, [[06_Primal_Dual_Method|dual 문제]]를 푸는 데 쓴다. 2장에서 가장 실용적인 도구이며, **행렬 역변환이 없다**는 것이 실시간 구현에서의 결정적 장점이다.

## 아이디어

탐색 방향을 기저벡터 $e_i=[0\ 0\ \cdots\ 1\ \cdots\ 0]^T$ 로 잡는다. 즉 $\lambda$ 벡터를 한 번에 한 성분씩만 변화시킨다.

한 성분에 대한 절차는 이렇다.

1. 현재 $\lambda\ge 0$ 이 주어진 상태에서 성분 $\lambda_i$ 하나에 주목한다.
2. 목적함수를 이 한 변수만의 이차함수로 보고 최소화하는 $\lambda_i$ 를 구한다.
3. 그 값이 음수라면 $\lambda_i=0$ 으로 설정한다.
4. 어느 쪽이든 목적함수는 감소한다.
5. 다음 성분 $\lambda_{i+1}$ 로 넘어간다.

전체 성분을 한 번씩 도는 것을 한 사이클, 즉 1회 반복으로 보고 $\lambda^m$ 에서 $\lambda^{m+1}$ 로 간다고 쓴다.

## 갱신 공식

$$
\lambda_i^{m+1}=\max(0,\ w_i^{m+1})
$$

$$
w_i^{m+1}=-\frac{1}{h_{ii}}\left[k_i+\sum_{j=1}^{i-1}h_{ij}\lambda_j^{m+1}+\sum_{j=i+1}^{n}h_{ij}\lambda_j^{m}\right]
$$

여기서 $h_{ij}$ 는 $H=ME^{-1}M^T$ 의 $(i,j)$ 원소이고, $k_i$ 는 $K=\gamma+ME^{-1}F$ 의 $i$ 번째 원소다.

이 식이 어디서 나오는지는 교재가 생략하는데, 유도는 짧다. Dual 목적함수 $J(\lambda)=\frac12\lambda^TH\lambda+\lambda^TK+\text{const}$ 를 $\lambda_i$ 에 대해서만 편미분하면

$$
\frac{\partial J}{\partial\lambda_i}=h_{ii}\lambda_i+\sum_{j\ne i}h_{ij}\lambda_j+k_i=0
$$

$$
\lambda_i=-\frac{1}{h_{ii}}\left[k_i+\sum_{j\ne i}h_{ij}\lambda_j\right]
$$

여기서 $j<i$ 는 이번 사이클에서 이미 갱신된 새 값 $\lambda_j^{m+1}$ 을 쓰고, $j>i$ 는 아직 갱신되지 않은 옛 값 $\lambda_j^{m}$ 을 쓴다. 이것은 정확히 **Gauss-Seidel 반복법**이며, 여기에 음수를 0으로 자르는 사영을 추가한 것이다. projected Gauss-Seidel이라고도 부른다.

공식에서 두 종류의 $\lambda$ 가 섞여 있다는 점을 놓치면 안 된다. 앞쪽 합은 새 값, 뒤쪽 합은 옛 값이다.

수렴한 $\lambda^*$ 는 0 또는 양수만 가지므로 primal 해는

$$
x=-E^{-1}(F+M^T\lambda^*)
$$

## 알고리즘의 특징

| 특징 | 설명 |
|---|---|
| 행렬 역변환 불필요 | 원소 단위 탐색이라 $H^{-1}$ 이 필요 없다 |
| 수렴 조건 | 활성 제약이 선형 독립이고 개수가 결정변수 개수 이하이면 dual 변수가 수렴한다 |
| 조건 위반 시 | 고정된 값들로 수렴하지 않고 최대 반복 횟수에서 종료한다 |
| 실시간 강점 | 역행렬이 없으므로 계산이 중단 없이 계속된다. 제약 충돌 상황에서도 타협된 준최적해를 내놓는다 |

마지막 항목이 왜 중요한지 강조할 필요가 있다. 수치적으로 불안정한 제약 문제에서 자동으로 회복하는 능력은 플랜트 안전을 위해 무엇보다 중요하다. 일반 QP 솔버는 실행 불가능이면 에러를 내고 멈추지만, Hildreth는 그럴듯한 값을 계속 내준다. 이 성질이 실제로 발현되는 사례가 [[13_Output_Constraints|Example 2.20]]이다.

조건이 만족될 때 Hildreth의 1차원 탐색은 $\lambda^*$ 의 집합으로 수렴하는 것이 알려져 있다. $\lambda^*$ 는 비활성 제약에 대해 0이고 활성 제약에 대해 양수다. 양수 성분만 모은 벡터를 $\lambda^*_{act}$ 라 하면

$$
\lambda^*_{act}=-(M_{act}E^{-1}M_{act}^T)^{-1}(\gamma_{act}+M_{act}E^{-1}F)
$$

여기서 $M_{act}$, $\gamma_{act}$ 는 $\lambda^*$ 의 0인 원소에 대응하는 행을 삭제한 것이다. 수렴 증명은 유계인 $\lambda^*_{act}$ 의 존재에 의존하며, 이는 사실상 $(M_{act}E^{-1}M_{act}^T)^{-1}$ 의 존재로 결정된다.

## Example 2.10 - Hildreth 손계산

$$
\min J=\frac12x^TEx+F^Tx,
\quad
E=
\begin{bmatrix}
2 & -1 \\
-1 & 1
\end{bmatrix},
\quad
F=
\begin{bmatrix}
-1 \\
0
\end{bmatrix}
$$

제약은 $0\le x_1$, $0\le x_2$, $3x_1+2x_2\le 4$ 다.

### 표준형

$0\le x_1$ 은 $-x_1\le 0$ 이므로

$$
\begin{bmatrix}
-1 & 0 \\
0 & -1 \\
3 & 2
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}
\le
\begin{bmatrix}
0 \\
0 \\
4
\end{bmatrix}
$$

### 무제약 전역 최적해

$$
E^{-1}=\frac{1}{2\cdot 1-(-1)(-1)}
\begin{bmatrix}
1 & 1 \\
1 & 2
\end{bmatrix}
=
\begin{bmatrix}
1 & 1 \\
1 & 2
\end{bmatrix}
$$

$$
\begin{bmatrix}
x_1^0 \\
x_2^0
\end{bmatrix}
=-E^{-1}F=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
$$

3번 제약을 확인하면 $3(1)+2(1)=5>4$ 이므로 위반이다.

### H와 K

$$
H=ME^{-1}M^T=
\begin{bmatrix}
1 & 1 & -5 \\
1 & 2 & -7 \\
-5 & -7 & 29
\end{bmatrix}
$$

$$
K=\gamma+ME^{-1}F=
\begin{bmatrix}
0 \\
0 \\
4
\end{bmatrix}
+
\begin{bmatrix}
1 \\
1 \\
-5
\end{bmatrix}
=
\begin{bmatrix}
1 \\
1 \\
-1
\end{bmatrix}
$$

중간 계산은 $E^{-1}F=[-1\ -1]^T$ 이고, $ME^{-1}F$ 는 1행이 1, 2행이 1, 3행이 $-5$ 다.

### 반복

초기값 $\lambda_1^0=\lambda_2^0=\lambda_3^0=0$ 에서 시작한다.

$k=1$ 일 때

$$
w_1^1+1=0 \ \Rightarrow\ w_1^1=-1, \quad \lambda_1^1=\max(0,-1)=0
$$

$$
\lambda_1^1+2w_2^1+1=0 \ \Rightarrow\ w_2^1=-0.5, \quad \lambda_2^1=0
$$

$$
-5\lambda_1^1-7\lambda_2^1+29w_3^1-1=0 \ \Rightarrow\ w_3^1=0.0345, \quad \lambda_3^1=0.0345
$$

$k=2$ 일 때

$$
w_1^2+\lambda_2^1-5\lambda_3^1+1=0 \ \Rightarrow\ w_1^2=-0.8276, \quad \lambda_1^2=0
$$

$$
\lambda_1^2+2w_2^2-7\lambda_3^1+1=0 \ \Rightarrow\ w_2^2=-0.3793, \quad \lambda_2^2=0
$$

$$
-5\lambda_1^2-7\lambda_2^2+29w_3^2-1=0 \ \Rightarrow\ w_3^2=0.0345, \quad \lambda_3^2=0.0345
$$

$\lambda^2=\lambda^1$ 이므로 수렴했다.

$$
\lambda^*=
\begin{bmatrix}
0 & 0 & 0.0345
\end{bmatrix}^T
$$

### Primal 해

$$
x^*=
\begin{bmatrix}
x_1^0 \\
x_2^0
\end{bmatrix}
-E^{-1}M^T\lambda^*
=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
-
\begin{bmatrix}
0.1724 \\
0.2414
\end{bmatrix}
=
\begin{bmatrix}
0.8276 \\
0.7586
\end{bmatrix}
$$

검산하면 $3(0.8276)+2(0.7586)=4.0000$ 으로 3번 제약이 정확히 활성이고, $x_1$ 과 $x_2$ 는 모두 양수라 1, 2번 제약이 비활성이다. 이는 $\lambda_1=\lambda_2=0$ 과 일치한다.

여기서도 구조는 같다. **제약 최적해는 무제약 전역 최적해에 활성 제약에 의한 보정항을 더한 것이다.**

## Example 2.11 - 박스 제약

제약이 $0\le x_1\le 1$, $0\le x_2\le 1$ 이고 목적함수가

$$
J=\frac12\left[(x_1-2)^2+(x_2-2)^2\right]
$$

### 표준형

$$
\begin{bmatrix}
1 & 0 \\
-1 & 0 \\
0 & 1 \\
0 & -1
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}
\le
\begin{bmatrix}
1 \\
0 \\
1 \\
0
\end{bmatrix}
$$

목적함수를 전개하면 $J=\frac12x^TEx+F^Tx+\text{const}$ 이고

$$
E=
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix},
\quad
F=
\begin{bmatrix}
-2 & -2
\end{bmatrix}^T,
\quad
\begin{bmatrix}
x_1^0 \\
x_2^0
\end{bmatrix}
=-E^{-1}F=
\begin{bmatrix}
2 \\
2
\end{bmatrix}
$$

$x_1$ 과 $x_2$ 의 상한이 모두 위반된다.

### H와 K

$$
H=ME^{-1}M^T=
\begin{bmatrix}
1 & -1 & 0 & 0 \\
-1 & 1 & 0 & 0 \\
0 & 0 & 1 & -1 \\
0 & 0 & -1 & 1
\end{bmatrix}
$$

$$
K=\gamma+ME^{-1}F=
\begin{bmatrix}
1 \\
0 \\
1 \\
0
\end{bmatrix}
+
\begin{bmatrix}
-2 \\
2 \\
-2 \\
2
\end{bmatrix}
=
\begin{bmatrix}
-1 \\
2 \\
-1 \\
2
\end{bmatrix}
$$

### 반복

$\lambda^0=0$ 에서 시작한다. $k=1$ 일 때

$$
w_1^1-1=0 \ \Rightarrow\ \lambda_1^1=\max(0,1)=1
$$

$$
-\lambda_1^1+w_2^1+2=0 \ \Rightarrow\ w_2^1=-1, \quad \lambda_2^1=0
$$

$$
w_3^1-1=0 \ \Rightarrow\ \lambda_3^1=1
$$

$$
-\lambda_3^1+w_4^1+2=0 \ \Rightarrow\ w_4^1=-1, \quad \lambda_4^1=0
$$

$k=2$ 일 때

$$
w_1^2-\lambda_2^1-1=0 \ \Rightarrow\ w_1^2=1, \quad \lambda_1^2=1
$$

$$
-\lambda_1^2+w_2^2+2=0 \ \Rightarrow\ w_2^2=-1, \quad \lambda_2^2=0
$$

$$
w_3^2-\lambda_4^1-1=0 \ \Rightarrow\ w_3^2=1, \quad \lambda_3^2=1
$$

$$
-\lambda_3^2+w_4^2+2=0 \ \Rightarrow\ w_4^2=-1, \quad \lambda_4^2=0
$$

$\lambda^2=\lambda^1$ 이므로 수렴했다. 최적해는 $\lambda_1^*=1$, $\lambda_2^*=0$, $\lambda_3^*=1$, $\lambda_4^*=0$ 이다.

### Primal 해

비활성 제약을 삭제하면 활성은 1번과 3번, 즉 $x_1\le 1$ 과 $x_2\le 1$ 이다.

$$
x^*=x^0-E^{-1}\left(
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
1 \\
1
\end{bmatrix}
\right)
=
\begin{bmatrix}
2 \\
2
\end{bmatrix}
-
\begin{bmatrix}
1 \\
1
\end{bmatrix}
=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
$$

직관적으로도 맞다. 무제약 최적해 $(2,2)$ 가 박스 밖에 있고 등고선이 원이므로, 가장 가까운 박스 모서리 $(1,1)$ 이 답이다.

## 연결

- 앞 단계: [[06_Primal_Dual_Method|Primal-Dual 방법]]
- 코드 구현: [[08_MATLAB_QPhild|MATLAB 구현 - QPhild.m]]
- 활성집합을 알 때: [[09_Closed_Form_Active_Set|닫힌형 해]]
- 실전 적용: [[10_Constraints_on_Rate_of_Change|변화율 제약 예측제어]]
