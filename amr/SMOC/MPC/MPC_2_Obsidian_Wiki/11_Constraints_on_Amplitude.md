---
title: "11. 진폭 제약 예측제어"
aliases:
  - "Constraints on Amplitude"
  - "Example 2.16"
  - "Example 2.17"
tags:
  - mpc
  - constraints
  - amplitude
source_pages: "PDF p.31-34 / book p.73-76"
---

# 진폭 제약 예측제어

제어신호 $u(k)$ 의 진폭도 제약을 걸 중요한 대상이다. [[10_Constraints_on_Rate_of_Change|변화율 제약]]과 같은 플랜트, 같은 설계 사양을 쓰고 제약만 바꾼다.

$$
-3\le u(k)\le 6
$$

변화율 제약과 결정적으로 다른 점이 하나 있다. 진폭 제약은 과거 제어값 $u(k_i-1)$ 이 제약식 우변에 들어가므로 **매 샘플마다 제약식을 갱신**해야 한다.

## Example 2.16 - 첫 샘플만 제약

시각 $k_i$ 에서

$$
u(k_i)=\Delta u(k_i)+u(k_i-1),
\qquad
\Delta u(k_i)=
\begin{bmatrix}
1 & 0 & 0
\end{bmatrix}
\Delta U
$$

이므로 제약은

$$
-3\le
\begin{bmatrix}
1 & 0 & 0
\end{bmatrix}
\Delta U+u(k_i-1)
\le 6
$$

표준형으로 옮기면

$$
\begin{bmatrix}
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
6-u(k_i-1) \\
3+u(k_i-1)
\end{bmatrix}
$$

### 계산 과정 추적

**1.** $k_i=1$ 이고 초기조건 $u(k_i-1)=0$ 을 가정한다. 무제약 최적해는

$$
\Delta U=
\begin{bmatrix}
6.1083 & 2.3334 & -0.5861
\end{bmatrix}^T
$$

이므로 $u(k_i)=0+6.1083=6.108>6$ 으로 제약을 위반한다. QP로 풀면

$$
\Delta U=
\begin{bmatrix}
6.0000 & 2.4010 & -0.5291
\end{bmatrix}^T
$$

이고 $u(1)=6$ 으로 제약을 만족한다.

**2.** 갱신된 상태와 $u(1)=6$ 이라는 정보로 $k_i=2$ 에서 풀면

$$
\Delta U=
\begin{bmatrix}
0 & 1.8921 & -0.8643
\end{bmatrix}^T
$$

이고 $u(2)=6+0=6$ 으로 제약을 만족한다.

첫 성분이 정확히 0이 된 것은 우연이 아니다. $u$ 가 이미 상한 6에 도달했으므로 더 올릴 수 없어서 증분이 0이 되어야 한다. 진폭 제약이 활성일 때 나타나는 전형적 패턴이다.

**3.** 시간이 흐르면서 예측제어는 매 샘플마다 지정된 제약을 만족하는 최적 제어를 찾는다.

### 결과

제약이 만족되고, 출력 응답이 무제약 경우와 거의 같다.

## Example 2.17 - 모든 원소에 제약

같은 제약 $-3\le u(k)\le 6$ 을 모든 제어신호 원소에 부과한다. $u$ 를 $\Delta U$ 로 누적 표현해야 한다.

$$
u(k_i+1)=\Delta u(k_i+1)+u(k_i)=\Delta u(k_i+1)+\Delta u(k_i)+u(k_i-1)
$$

$$
u(k_i+2)=\Delta u(k_i+2)+\Delta u(k_i+1)+\Delta u(k_i)+u(k_i-1)
$$

행렬 형태로 묶으면 [[02_Constraint_Formulation|앞에서 정의한]] $C_1$, $C_2$ 가 그대로 나온다.

$$
\begin{bmatrix}
u(k_i) \\
u(k_i+1) \\
u(k_i+2)
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 0 \\
1 & 1 & 0 \\
1 & 1 & 1
\end{bmatrix}
\begin{bmatrix}
\Delta u(k_i) \\
\Delta u(k_i+1) \\
\Delta u(k_i+2)
\end{bmatrix}
+
\begin{bmatrix}
u(k_i-1) \\
u(k_i-1) \\
u(k_i-1)
\end{bmatrix}
$$

상하한을 모두 쓰면 여섯 개의 부등식이다.

$$
\begin{bmatrix}
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
6-u(k_i-1) \\
6-u(k_i-1) \\
6-u(k_i-1) \\
3+u(k_i-1) \\
3+u(k_i-1) \\
3+u(k_i-1)
\end{bmatrix}
$$

### 결과

$u(k_i+1)$ 과 $u(k_i+2)$ 에 대한 추가 제약 두 개가 최적화 과정에서 활성화된다. 그 증거는 최적해 $\Delta U$ 의 해당 성분들이 0이 되는 것이다.

교재는 여기에 짧은 단서를 단다. 엄밀히 말하면 활성 제약은 Lagrange 승수가 양수인 것으로 판별한다. 그러나 제어 진폭이 한계에 도달하면 제약을 만족시키기 위해 대응하는 $\Delta U$ 성분이 정확히 0이 되어야 한다. $u$ 가 더 이상 움직일 수 없기 때문이다. 그래서 진폭 제약에서는 "$\Delta U$ 성분이 0"이 활성 제약의 시각적 신호가 된다.

Example 2.16과 비교하면 플랜트 출력, 제어신호, $\Delta u(k)$ 모두 차이가 미미하다. 그런데 제약 개수는 세 배다.

미래 전체에 제약을 걸어도 성능 이득은 거의 없고 계산량만 세 배가 된다. 실시간 구현에서는 첫 샘플, 또는 처음 몇 샘플에만 제약을 거는 것이 합리적이다.

## 변화율 제약과의 실무적 차이

| 항목 | 변화율 제약 | 진폭 제약 |
|---|---|---|
| $\Delta U$ 와의 관계 | 항등 행렬 | 누적합 행렬 $C_2$ |
| 우변 | 상수 | $u(k_i-1)$ 포함 |
| 매 샘플 갱신 | 불필요 | **필요** |
| 활성일 때의 신호 | 해당 성분이 경계값 | 해당 성분이 0 |

## 연결

- 앞 단계: [[10_Constraints_on_Rate_of_Change|변화율 제약 예측제어]]
- 다음 단계: [[12_Amplitude_and_Rate|진폭과 변화율 동시 제약]]
- $C_1$, $C_2$ 정의: [[02_Constraint_Formulation|운전 제약의 종류와 수식화]]
