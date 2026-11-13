---
title: "06. Primal-Dual 방법"
aliases:
  - "Primal-Dual Method"
  - "Dual Problem"
  - "Dual Variable"
tags:
  - mpc
  - qp
  - duality
source_pages: "PDF p.20-21 / book p.62-63"
---

# Primal-Dual 방법

[[05_QP_Inequality_KKT_ActiveSet|Active set method]]는 primal 방법에 속한다. 결정변수를 기준으로 해를 찾기 때문이다. 이 방식에는 실무적인 한계가 있다.

- 활성 제약을 최적 결정변수와 **함께** 찾아야 한다.
- 제약이 많으면 계산 부하가 상당히 커진다.
- Example 2.9에서 보았듯 프로그래밍이 간단하지 않다.

Dual 방법은 활성이 아닌 제약을 체계적으로 식별해서 제거할 수 있다. Lagrange 승수를 최적화 문헌에서 **dual variable**이라고 부르는데, 이 방법은 제약 최소화 문제의 최적해를 찾는 매우 간단한 프로그래밍 절차로 이어진다.

## Dual 문제 유도

실행 가능하다고 가정한다. 즉 $Mx<\gamma$ 인 $x$ 가 존재한다. 그러면 primal 문제는 다음과 동치다.

$$
\max_{\lambda\ge 0}\ \min_{x}\left[\frac12x^TEx+x^TF+\lambda^T(Mx-\gamma)\right]
$$

왜 동치인지는 교재가 넘어가지만 확인해 두면 좋다. 안쪽 최소화를 잠깐 잊고 $\max_{\lambda\ge 0}\lambda^T(Mx-\gamma)$ 만 보자.

- $Mx\le\gamma$ 인 가능해라면 $\lambda^T(Mx-\gamma)\le 0$ 이므로 최대값은 $\lambda=0$ 일 때 0이다.
- $Mx>\gamma$ 로 위반한다면 해당 $\lambda_i$ 를 무한대로 보내서 $+\infty$ 가 된다.

즉 바깥의 최대화가 가능하지 않은 $x$ 를 자동으로 무한대 벌점 처리하는 역할을 한다.

### 안쪽 최소화

안쪽 최소화는 무제약이므로 미분해서 0으로 놓는다.

$$
Ex+F+M^T\lambda=0
$$

$$
x=-E^{-1}(F+M^T\lambda)
$$

### 대입과 정리

$$
\frac12x^TEx+x^TF+\lambda^T(Mx-\gamma)=\frac12x^TEx+x^T(F+M^T\lambda)-\lambda^T\gamma
$$

여기서 $F+M^T\lambda=-Ex$ 이므로

$$
=\frac12x^TEx-x^TEx-\lambda^T\gamma=-\frac12x^TEx-\lambda^T\gamma
$$

$x=-E^{-1}(F+M^T\lambda)$ 를 넣으면

$$
-\frac12(F+M^T\lambda)^TE^{-1}(F+M^T\lambda)-\lambda^T\gamma
$$

$$
=-\frac12\lambda^TME^{-1}M^T\lambda-\lambda^T(\gamma+ME^{-1}F)-\frac12F^TE^{-1}F
$$

### Dual 문제

$$
\max_{\lambda\ge 0}\left(-\frac12\lambda^TH\lambda-\lambda^TK-\frac12F^TE^{-1}F\right)
$$

$$
H=ME^{-1}M^T
$$

$$
K=\gamma+ME^{-1}F
$$

부호를 뒤집어 최소화 형태로 쓰면

$$
\min_{\lambda\ge 0}\left(\frac12\lambda^TH\lambda+\lambda^TK+\frac12\gamma^TE^{-1}\gamma\right)
$$

따라서 dual도 $\lambda$ 를 결정변수로 하는 QP 문제다.

## Dual이 왜 좋은가

한 줄로 요약하면 이렇다. Primal의 제약은 복잡한 $Mx\le\gamma$ 이지만, dual의 제약은 단순한 $\lambda\ge 0$ 이다.

"각 변수가 0 이상"이라는 제약은 성분별로 완전히 독립적이다. 그래서 한 성분씩 순회하며 음수면 0으로 잘라주기만 하면 된다. 이것이 다음 노트의 [[07_Hildreth_QP|Hildreth 알고리즘]]이다.

## Dual 해에서 Primal 해로

Dual 목적함수

$$
J=\frac12\lambda^TH\lambda+\lambda^TK+\frac12\gamma^TE^{-1}\gamma
$$

를 $\lambda\ge 0$ 아래에서 최소화하는 승수 집합을 $\lambda_{act}$ 라 하고, 대응하는 제약을 $M_{act}$, $\gamma_{act}$ 라 하자. 그러면 primal 변수는

$$
x=-E^{-1}F-E^{-1}M_{act}^T\lambda_{act}
$$

로 얻는다. 계산에서 이 제약들은 등식제약처럼 취급된다.

## 연결

- 앞 단계: [[05_QP_Inequality_KKT_ActiveSet|부등식제약, KKT 조건, Active Set]]
- 다음 단계: [[07_Hildreth_QP|Hildreth 이차계획법]]
- 수식 색인: [[14_Key_Equations|핵심 수식 정리]]
