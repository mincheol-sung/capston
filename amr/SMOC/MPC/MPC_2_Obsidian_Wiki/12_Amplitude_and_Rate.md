---
title: "12. 진폭과 변화율 동시 제약"
aliases:
  - "Constraints on Amplitude and Rate of Change"
  - "Example 2.18"
  - "Example 2.19"
tags:
  - mpc
  - constraints
source_pages: "PDF p.35-36 / book p.77-78"
---

# 진폭과 변화율 동시 제약

설계 사양에 두 제약이 함께 요구되는 경우가 많다. 방법은 단순하다. 두 제약을 한꺼번에 쌓아서 더 큰 선형 부등식 집합으로 만들면 된다. 여기서도 먼저 시각 $k_i$ 에만 거는 경우를 보고, 그 다음 모든 미래 제어 이동으로 확장한다.

플랜트와 설계 사양은 [[10_Constraints_on_Rate_of_Change|Example 2.14]]와 같다.

$$
-1.5\le\Delta u(k)\le 3, \qquad -3\le u(k)\le 6
$$

## Example 2.18 - 시각 $k_i$ 에만 제약

$\Delta u(k_i)$ 와 $u(k_i)$ 에만 제약을 부과하면 네 개의 부등식이 된다.

$$
\begin{bmatrix}
1 & 0 & 0 \\
-1 & 0 & 0 \\
1 & 0 & 0 \\
-1 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
\Delta u(k_i) \\
\Delta u(k_i+1) \\
\Delta u(k_i+2)
\end{bmatrix}
\le
\begin{bmatrix}
3.0000 \\
1.5000 \\
6-u(k_i-1) \\
3+u(k_i-1)
\end{bmatrix}
$$

| 행 | 역할 |
|---|---|
| 1, 2 | 변화율 제약 |
| 3, 4 | 진폭 제약 |

왼쪽 행렬의 1행과 3행이 똑같다는 점을 눈여겨보아야 한다. $\Delta u(k_i)$ 와 $u(k_i)$ 는 둘 다 첫 성분만 뽑는 행으로 표현되기 때문이다. $u(k_i)=\Delta u(k_i)+u(k_i-1)$ 이므로 $\Delta U$ 에 곱해지는 부분이 같고, 차이는 오직 우변에만 있다. 즉 두 제약은 같은 방향의 서로 다른 한계선이다.

이 사실은 뒤에서 중요해진다. 두 제약이 동시에 활성이 되면 **선형 종속**이 되고, [[04_QP_Equality_Constraints|Example 2.6]]의 교훈에 따라 실행 불가능 상황이 만들어진다. 실제 사례는 [[13_Output_Constraints|출력 제약 노트]]에 있다.

목적함수 $J$ 를 이 제약 아래에서 최소화하면 두 제약이 모두 만족되고, 무제약 최적제어와 비교했을 때 성능 변화가 작다.

## Example 2.19 - 모든 원소에 제약

같은 제약을 $\Delta U$ 의 모든 성분과 $u(k_i)$, $u(k_i+1)$, $u(k_i+2)$ 모두에 부과하면 열두 개의 부등식이 된다.

$$
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1 \\
-1 & 0 & 0 \\
0 & -1 & 0 \\
0 & 0 & -1 \\
1 & 0 & 0 \\
1 & 1 & 0 \\
1 & 1 & 1 \\
-1 & 0 & 0 \\
-1 & -1 & 0 \\
-1 & -1 & -1
\end{bmatrix}
\begin{bmatrix}
\Delta u(k_i) \\
\Delta u(k_i+1) \\
\Delta u(k_i+2)
\end{bmatrix}
\le
\begin{bmatrix}
3 \\
3 \\
3 \\
1.5 \\
1.5 \\
1.5 \\
6-u(k_i-1) \\
6-u(k_i-1) \\
6-u(k_i-1) \\
3+u(k_i-1) \\
3+u(k_i-1) \\
3+u(k_i-1)
\end{bmatrix}
$$

| 행 블록 | 내용 |
|---|---|
| 1-3 | $\Delta u$ 상한 |
| 4-6 | $\Delta u$ 하한 |
| 7-9 | $u$ 상한. 누적합 행렬 $C_2$ |
| 10-12 | $u$ 하한 |

### 결과

모든 제약이 만족되고, 출력 오버슈트가 약간 더 작아진다. 성능이 소폭 개선되기는 하지만 Example 2.18과의 차이는 매우 작다.

## 정리

| 제약 방식 | 예제 | 부등식 개수 |
|---|---|---|
| $\Delta u$ 만, 첫 원소 | 2.14 | 2 |
| $\Delta u$ 만, 전체 | 2.15 | 6 |
| $u$ 만, 첫 샘플 | 2.16 | 2 |
| $u$ 만, 전체 | 2.17 | 6 |
| $\Delta u+u$, 첫 시점 | 2.18 | 4 |
| $\Delta u+u$, 전체 | 2.19 | 12 |

$N_c=3$ 인 작은 문제에서도 제약 개수가 2에서 12까지 여섯 배 차이가 난다. $N_c$ 가 커지면 격차는 더 벌어지고, 얻는 것은 오버슈트 몇 퍼센트다.

## 연결

- 앞 단계: [[11_Constraints_on_Amplitude|진폭 제약 예측제어]]
- 다음 단계: [[13_Output_Constraints|출력 제약과 제약 충돌]]
- 선형 종속의 위험: [[04_QP_Equality_Constraints|등식제약 QP와 Lagrange 승수]]
