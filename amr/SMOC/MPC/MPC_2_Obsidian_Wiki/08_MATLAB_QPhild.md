---
title: "08. MATLAB 구현 - QPhild.m"
aliases:
  - "QPhild"
  - "Tutorial 2.1"
  - "Hildreth MATLAB"
tags:
  - mpc
  - matlab
  - qp
  - implementation
source_pages: "PDF p.25-26 / book p.67-68"
---

# MATLAB 구현 - QPhild.m

Tutorial 2.1의 목표는 [[07_Hildreth_QP|Hildreth 프로그래밍]]으로 제약 최적화 문제를 푸는 함수를 만드는 것이다. 풀 문제는

$$
\min J=\frac12\eta^TH\eta+\eta^Tf
$$

$$
A_{cons}\eta\le b
$$

이다.

## 프로그램의 큰 흐름

1. `QPhild.m` 이라는 새 파일을 만든다.
2. 프로그램은 먼저 전역 최적해를 구하고 모든 제약이 만족되는지 검사한다. 만족되면 그 해를 바로 반환하고, 아니면 쌍대변수 계산을 시작한다.
3. 쌍대 QP의 행렬을 세팅하고 Lagrange 승수를 반복적으로 계산한다.
4. 수렴한 승수로 활성 제약에 의한 보정을 더해 최종 해를 만든다.

## 1단계 - 전역 최적해와 위반 검사

```matlab
function eta=QPhild(H,f,A_cons,b);
% E=H;
% F=f;
% M=A_cons;
% gamma=b;
% eta =x
[n1,m1]=size(A_cons);
eta=-H\f;
kk=0;
for i=1:n1
if (A_cons(i,:)*eta>b(i)) kk=kk+1;
 else
kk=kk+0;
 end
end
if (kk==0) return; end
```

코드 해설은 다음과 같다.

- `eta=-H\f` 는 $\eta^0=-H^{-1}f$, 즉 무제약 전역 최적해다.
- `for` 루프는 각 제약행에 대해 위반 여부를 세어 `kk` 에 누적한다.
- `if (kk==0) return; end` 가 중요하다. 위반이 하나도 없으면 QP를 풀 필요 없이 바로 반환한다. 실제 폐루프에서 제약이 활성이 아닌 샘플이 대부분이므로 계산 절약 효과가 크다.
- MATLAB의 백슬래시 연산자는 역행렬을 명시적으로 만들지 않고 선형시스템을 푼다.

## 2단계 - 쌍대 QP 세팅과 승수 반복

QP 절차에서 $i$ 번째 Lagrange 승수는 대응 제약이 활성이 아니면 0이고 활성이면 양수다. 승수를 반복적으로 계산해야 한다.

```matlab
P=A_cons*(H\A_cons');
d=(A_cons*(H\f)+b);
[n,m]=size(d);
x_ini=zeros(n,m);
lambda=x_ini;
al=10;
for km=1:38
%find the elements in the solution vector one by one
% km could be larger if the Lagranger multiplier has a slow
% convergence rate.
lambda_p=lambda;
for i=1:n
w= P(i,:)*lambda-P(i,i)*lambda(i,1);
w=w+d(i,1);
la=-w/P(i,i);
lambda(i,1)=max(0,la);
end
al=(lambda-lambda_p)'*(lambda-lambda_p);
if (al<10e-8); break; end
end
```

## 코드와 수식의 대응

| 코드 | 수식 | 설명 |
|---|---|---|
| `P` | $H=ME^{-1}M^T$ | `H\A_cons'` 가 $E^{-1}M^T$ 에 해당 |
| `d` | $K=\gamma+ME^{-1}F$ | dual의 1차항 계수 |
| `lambda_p` | $\lambda^m$ | 이전 사이클 값, 수렴 판정용 |
| `w= P(i,:)*lambda-P(i,i)*lambda(i,1)` | $\sum_{j\ne i}h_{ij}\lambda_j$ | 대각항을 빼서 $j\ne i$ 합을 만든다 |
| `w=w+d(i,1)` | $+\,k_i$ | |
| `la=-w/P(i,i)` | $w_i^{m+1}$ | Hildreth 갱신식 |
| `lambda(i,1)=max(0,la)` | $\lambda_i^{m+1}=\max(0,w_i^{m+1})$ | 음수 사영 |
| `al=(lambda-lambda_p)'*(lambda-lambda_p)` | $\lVert\lambda^{m+1}-\lambda^m\rVert^2$ | 수렴 척도 |

핵심 트릭이 하나 숨어 있다. `lambda` 를 제자리에서 갱신하기 때문에, `P(i,:)*lambda` 에서 $j<i$ 는 자동으로 새 값 $\lambda^{m+1}$ 이 되고 $j>i$ 는 옛 값 $\lambda^m$ 이 된다. 덕분에 Hildreth 갱신식의 두 합이 코드 한 줄로 구현된다. Gauss-Seidel 반복의 전형적인 구현 방식이다.

`km=1:38` 은 최대 반복 횟수다. 승수의 수렴이 느리면 늘리면 된다. 역행렬을 쓰지 않으므로 수렴하지 않아도 프로그램이 죽지 않고 그때까지의 근사해를 반환한다. 이것이 실시간 안전성의 근거다.

## 3단계 - 최종 해

승수의 원소가 모두 0 또는 양수이므로, 승수 벡터와 제약식을 직접 이용해 활성 제약에 의한 $\eta$ 의 변화량을 계산할 수 있다.

```matlab
eta=-H\f -H\A_cons'*lambda;
```

대응 수식은

$$
\eta=-H^{-1}f-H^{-1}A_{cons}^T\lambda
\quad\longleftrightarrow\quad
x=-E^{-1}F-E^{-1}M^T\lambda^*
$$

이다.

## 테스트

교재는 [[07_Hildreth_QP|Example 2.10과 2.11]]에서 만든 데이터 행렬로 프로그램을 시험하라고 한다. 제약 최적해가 예제의 해와 같으면 구현이 맞다.

```matlab
% Example 2.10
H = [2 -1; -1 1];  f = [-1; 0];
A_cons = [-1 0; 0 -1; 3 2];  b = [0; 0; 4];
eta = QPhild(H,f,A_cons,b)
% 기대값: [0.8276; 0.7586]

% Example 2.11
H = [1 0; 0 1];  f = [-2; -2];
A_cons = [1 0; -1 0; 0 1; 0 -1];  b = [1; 0; 1; 0];
eta = QPhild(H,f,A_cons,b)
% 기대값: [1; 1]
```

## MPC 루프에 붙일 때

`QPhild` 를 예측제어에 쓰려면 매 샘플마다 다음 순서로 인자를 만들면 된다.

```text
for each sample k:
    build Xf from state increment and output
    H      = 2*(Phi'*Phi + Rbar)
    f      = -2*Phi'*(Rs - F*Xf)
    A_cons = stack of M1, M2, M3
    b      = stack of N1, N2, N3   % u(k-1)과 F*Xf가 들어가므로 매번 갱신
    DeltaU = QPhild(H, f, A_cons, b)
    deltau = first element of DeltaU
    u      = u + deltau
    propagate plant and observer
```

여기서 잊기 쉬운 것은 `b` 의 갱신이다. 진폭 제약 블록에는 $u(k_i-1)$ 이, 출력 제약 블록에는 $Fx(k_i)$ 가 들어 있으므로 루프 안에서 다시 계산해야 한다. 자세한 구조는 [[03_Constrained_MPC_Problem|데이터 행렬 노트]]를 참고한다.

## 구현 체크리스트

- 전역 최적해가 제약을 만족하면 즉시 반환하는 분기를 반드시 넣는다.
- `P(i,i)` 가 0에 가까우면 나눗셈이 불안정해진다. 제약 행이 중복되지 않았는지 확인한다.
- 최대 반복 횟수에 걸려 끝나는 경우가 잦다면 제약 충돌을 의심한다.
- 진폭 제약과 출력 제약의 우변은 매 샘플 갱신한다.
- 결과 $\Delta U$ 의 첫 원소만 적용한다.

## 연결

- 알고리즘 원리: [[07_Hildreth_QP|Hildreth 이차계획법]]
- 인자 만들기: [[03_Constrained_MPC_Problem|제약 있는 MPC 문제의 최종 형태]]
- 1장 구현 흐름: [[09_MATLAB_Workflows|1장 MATLAB 구현 흐름]]
- 다음 단계: [[09_Closed_Form_Active_Set|활성집합을 알 때의 닫힌형 해]]
