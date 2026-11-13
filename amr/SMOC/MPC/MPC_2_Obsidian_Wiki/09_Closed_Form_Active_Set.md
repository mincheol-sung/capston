---
title: "09. 활성집합을 알 때의 닫힌형 해"
aliases:
  - "Closed-form Solution"
  - "Active Set Guessing"
  - "Example 2.12"
  - "Example 2.13"
tags:
  - mpc
  - qp
  - active-set
  - closed-form
source_pages: "PDF p.26-27 / book p.68-69"
---

# 활성집합을 알 때의 닫힌형 해

[[07_Hildreth_QP|Hildreth 절차]]가 만들어내는 $\lambda^*$ 는 0들과 활성 제약에 대응하는 양수 성분으로 이루어진다. 양수 성분만 모은 벡터를 $\lambda^*_{act}$ 라 하면

$$
\lambda^*_{act}=-(M_{act}E^{-1}M_{act}^T)^{-1}(\gamma_{act}+M_{act}E^{-1}F)
$$

여기서 $M_{act}$ 와 $\gamma_{act}$ 는 $\lambda^*$ 의 0인 원소에 대응하는 행을 삭제한 제약 데이터다.

의미는 분명하다. **활성 제약을 사전에 정확히 알 수 있다면, 반복 없이 한 번의 행렬 계산으로 제약 최적화 문제의 해를 얻을 수 있다.**

## Example 2.12 - Example 2.10 재풀이

[[07_Hildreth_QP|Example 2.10]]의 3번 제약이 활성임을 사전에 안다고 하자. 그러면 $\lambda_1^*=\lambda_2^*=0$ 이므로

$$
M_{act}=
\begin{bmatrix}
3 & 2
\end{bmatrix},
\qquad
\gamma_{act}=4
$$

필요한 양을 계산하면

$$
M_{act}E^{-1}M_{act}^T=
\begin{bmatrix}
3 & 2
\end{bmatrix}
\begin{bmatrix}
1 & 1 \\
1 & 2
\end{bmatrix}
\begin{bmatrix}
3 \\
2
\end{bmatrix}
=
\begin{bmatrix}
5 & 7
\end{bmatrix}
\begin{bmatrix}
3 \\
2
\end{bmatrix}
=29
$$

$$
M_{act}E^{-1}F=
\begin{bmatrix}
3 & 2
\end{bmatrix}
\begin{bmatrix}
-1 \\
-1
\end{bmatrix}
=-5
$$

$$
\lambda^*_{act}=-\frac{1}{29}(4-5)=\frac{1}{29}=0.0345
$$

Example 2.10에서 반복으로 얻은 결과와 같다.

## Example 2.13 - Example 2.11 재풀이

[[07_Hildreth_QP|Example 2.11]]의 활성 제약이 1번과 3번이라고 가정한다. 그러면 $\lambda_2^*=\lambda_4^*=0$ 이고

$$
M_{act}=
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix},
\qquad
\gamma_{act}=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
$$

$$
\lambda^*_{act}=-(M_{act}E^{-1}M_{act}^T)^{-1}(\gamma_{act}+M_{act}E^{-1}F)
=-\left(
\begin{bmatrix}
1 \\
1
\end{bmatrix}
+
\begin{bmatrix}
-2 \\
-2
\end{bmatrix}
\right)
=
\begin{bmatrix}
1 \\
1
\end{bmatrix}
$$

$\lambda^*_{act}$ 의 원소가 모두 양수이므로 이 해를 최적해로 받아들인다. Example 2.11의 결과와 같다.

## 활성 제약 추측 기법의 실무적 가치

교재가 강조하는 요점은 이렇다.

- 계산속도가 결정적인 응용에서 매우 유용하다. 활성 제약을 맞게 추측하면 닫힌형 해로 즉시 풀린다.
- 활성 제약 집합의 일부만 탐색하고 나머지를 추측으로 채워도 계산속도가 올라간다.

추측이 맞았는지 검증하는 기준은 두 가지다.

1. $\lambda^*_{act}$ 의 원소가 모두 양수인가.
2. 활성으로 두지 않은 나머지 제약이 모두 만족되는가.

둘 다 통과하면 그 추측이 옳다. 하나라도 실패하면 [[05_QP_Inequality_KKT_ActiveSet|Active set method]]처럼 집합을 고쳐서 다시 시도한다.

## 1장 예제와의 연결

[[01_Motivational_Examples|Example 2.2]]의 "수정된 포화"가 왜 SISO 진폭 제약에 대해 최적해였는지가 여기서 설명된다. SISO이고 첫 샘플에만 제약을 걸면 활성 제약이 최대 하나다. 활성 제약이 하나면 $M_{act}$ 가 한 행뿐이라 위 공식이 스칼라 계산이 되고, 그 결과가 곧 "제어값을 경계로 자르고 그에 맞는 $\Delta u$ 를 다시 계산"하는 것과 일치한다.

이 아이디어의 문헌적 뿌리는 Tsang and Clarke(1988)이다. 제어구간이 1이나 2인 SISO GPC에 대해 활성 제약을 추측하고 닫힌형 해를 적용하는 방식으로 최적해를 유도했다.

## 연결

- 앞 단계: [[08_MATLAB_QPhild|MATLAB 구현 - QPhild.m]]
- 알고리즘 원리: [[07_Hildreth_QP|Hildreth 이차계획법]]
- 다음 단계: [[10_Constraints_on_Rate_of_Change|변화율 제약 예측제어]]
- 최적성의 근거가 된 예제: [[01_Motivational_Examples|동기부여 예제]]
