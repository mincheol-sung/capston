---
title: "Wang MPC 교재 학습 가이드"
subtitle: "메카넘 휠 AMR 슬립 제어 MPC 구현을 위한 읽을 범위와 핵심 식"
date: "2026년 10월 6일"
---

**1장 1.2~1.5절과 2장 2.3·2.4.4~2.4.5·2.5절만 보면 돼요.** 3~9장은 이 과제에 거의 필요 없어요. 이 책은 Δu를 최적화하고 모델에 적분기를 넣는 방식을 쓰는데, 질량이나 마찰 모델이 조금 틀려도 정상상태 속도 오차가 남지 않아요. 하중·노면 변화에 강건해야 하는 과제 목표와도 잘 맞아요.

::: {custom-style="Source Note"}
참고 교재: Liuping Wang, *Model Predictive Control System Design and Implementation Using MATLAB®*, Springer, 2009. 이 문서의 쪽수와 식 번호는 모두 이 책 기준이에요.
:::

# 1. 읽을 범위 (책 쪽수 기준)

| 구분 | 절 | 쪽 | 가져갈 것 |
|:--:|:------|:--:|:----------------|
| 필수 | 1.2 증강 모델 | 4–7 | 식 (1.5), Tutorial 1.1 |
| 필수 | 1.3 예측·최적화 | 7–14 | 식 (1.12)(1.16), Tutorial 1.2 `mpcgain.m` |
| 필수 | 1.4 Receding horizon | 15–21 | 식 (1.29), Tutorial 1.3 `reced.m` (제어 루프 뼈대) |
| 필수 | 1.5 MIMO | 22–27 | 식 (1.38), (1.44)~(1.46) |
| 필수 | 2.3 제약 정식화 | 47–52 | 식 (2.18)~(2.26), 출력 제약을 soft로 거는 식 (2.9) |
| 필수 | 2.4.4~2.4.5 Hildreth QP | 63–68 | Tutorial 2.1 `QPhild.m` (quadprog를 쓰면 생략 가능) |
| 필수 | 2.5 제약 예제 | 69–81 | 제약 행렬 만드는 예시, 특히 2.5.4 출력 제약 |
| 선택 | 3.5~3.6 MATLAB 튜토리얼 | 108–127 | MIMO 코드 예시 (Laguerre 극점 a=0이면 1·2장 방식과 같음) |
| 선택 | 1.6.3 칼만 필터 | 33 | SLAM 속도 잡음이 클 때만 |
| 선택 | 4장 | 149~ | $N_p$를 수백 단위로 키워 수치 문제가 생길 때만 (예제 1.6 참고) |
| 선택 | 10.2~10.3 | 333–349 | 실기 구현 (MCU, Simulink embedded function) |

건너뛰어도 되는 곳은 다음과 같아요.

- 1.6~1.7 관측기: 상태를 SLAM과 엔코더로 모두 측정한다고 가정했으니 필요 없어요.
- 2.4.1~2.4.3, 2.4.6: QP 이론이에요.
- 3장의 나머지 부분, 5~9장(연속시간·고전 MPC)

# 2. 코드에 들어가는 식

## ① 증강 모델 (식 1.38)

플랜트 $x_m(k+1)=A_mx_m(k)+B_mu(k)$, $y=C_mx_m$을 다음과 같이 바꿔요.

$$A=\begin{bmatrix}A_m & 0\\ C_mA_m & I_q\end{bmatrix},\quad B=\begin{bmatrix}B_m\\ C_mB_m\end{bmatrix},\quad C=\begin{bmatrix}0 & I_q\end{bmatrix},\quad x(k)=\begin{bmatrix}x_m(k)-x_m(k-1)\\ y(k)\end{bmatrix}$$

## ② 예측 (식 1.44)

$$Y=F\,x(k)+\Phi\,\Delta U$$

$$F=\begin{bmatrix}CA\\ CA^2\\ \vdots\\ CA^{N_p}\end{bmatrix},\qquad \Phi=\begin{bmatrix}CB & 0 & \cdots & 0\\ CAB & CB & \cdots & 0\\ \vdots & \vdots & \ddots & \vdots\\ CA^{N_p-1}B & CA^{N_p-2}B & \cdots & CA^{N_p-N_c}B\end{bmatrix}$$

## ③ QP 행렬

식 (2.24)를 식 (2.27) 꼴로 정리하고, 출력 가중치 $\overline{Q}$를 추가한 형태예요.

$$H=2\left(\Phi^T\overline{Q}\Phi+\overline{R}\right),\qquad f=-2\,\Phi^T\overline{Q}\left(R_s-F\,x(k)\right)$$

- $R_s$는 미래 기준값을 쌓은 벡터예요. 기준이 일정하면 책처럼 $r(k)$를 $N_p$번 반복하면 돼요.
- 제약이 없으면 $\Delta U=-H^{-1}f$로 끝나고, 이 식은 식 (1.45)와 같아요.

## ④ 제약을 MΔU ≤ γ 꼴로 (식 2.18~2.25)

$$\begin{bmatrix}-C_2\\ C_2\\ -I\\ I\\ -\Phi_s\\ \Phi_s\end{bmatrix}\Delta U\le\begin{bmatrix}-U_{\min}+C_1u(k-1)\\ U_{\max}-C_1u(k-1)\\ -\Delta U_{\min}\\ \Delta U_{\max}\\ -Y_{s,\min}+F_s\,x(k)\\ Y_{s,\max}-F_s\,x(k)\end{bmatrix}$$

- **1~2행 (토크 크기 한계):** $C_1$은 $I_m$을 $N_c$개 세로로 쌓은 행렬이고, $C_2$는 $I_m$ 블록으로 된 하삼각 행렬이에요.
- **3~4행 (토크 변화율 한계)**
- **5~6행 (슬립 제약):** $\Phi_s, F_s$는 $\Phi, F$에서 슬립 출력에 해당하는 행만 골라낸 거예요. 상한은 $\lambda_{\max}|\omega^{\mathrm{ref}}|+\delta$를 넣으면 돼요.

## ⑤ 풀고 적용하기 (Tutorial 2.1, 식 1.46)

- `ΔU = QPhild(H, f, M, γ)`로 풀어요. `quadprog(H, f, M, γ)`를 써도 돼요.
- $\Delta U$의 앞 $m$개(바퀴 4개분)만 꺼내서 $u(k)=u(k-1)+\Delta u(k)$로 적용해요.
- 다음 스텝 상태는 `Xf = [xm - xm_old; y]`로 만들어요 (Tutorial 1.3).

# 3. 책 코드를 그대로 쓰면 안 되는 부분

- **`mpcgain.m`은 SISO 전용이에요.**
    - 앞부분(증강 모델 만들기)은 MIMO에서도 그대로 돌아가요.
    - F와 Φ를 만드는 for문은 출력 1개·입력 1개 기준으로 인덱싱돼 있어서, $q$행×$m$열 블록 단위로 고쳐야 해요.
    - `reced.m`도 `DeltaU(1,1)` 대신 앞의 4개를 꺼내야 해요.
- **출력 가중치 $\overline{Q}$를 넣어야 해요.** 책 1·2장 식은 출력 가중치가 단위행렬이에요. 단위가 다른 $v_x, v_y$ [m/s]와 $\omega_z$ [rad/s]의 비중을 맞추려면 ③처럼 $\overline{Q}$를 넣어야 해요.
- **슬립은 "가중치 0인 출력"으로 넣으세요.**
    - 출력을 $y=[v_x,v_y,\omega_z,s_1,\dots,s_4]$로 잡아요.
    - 슬립 4개는 $\overline{Q}$에서 가중치를 0으로 두고, ④의 5~6행에서만 써요.
    - 1.5.1절의 "출력 수 ≤ 입력 수" 조건은 모든 출력을 오차 없이 따라갈 때 필요한 조건이에요. 여기서는 따라갈 출력이 3개로 토크 4개보다 적으니 만족해요.
- **토크는 보내기 전에 물리 한계로 한 번 더 자르세요.**
    - 입력 제약과 출력 제약이 충돌하는 예제 2.20에서, Hildreth는 출력 제약을 지키고 입력 제약을 넘겼어요.
    - 자른 값을 $u(k-1)$로 저장해야 다음 스텝 계산이 실제 토크와 어긋나지 않아요. 예제 2.2의 식 (2.4)~(2.5)와 같은 처리예요.
- **`QPhild.m`의 반복 횟수(38회)는 늘리세요.**
    - 슬립 제약까지 넣으면 제약 행이 수백 개가 되어 38회로는 부족할 수 있어요.
    - 계산이 무거우면 2.5절 예제들처럼 제약을 예측 구간의 앞 몇 스텝에만 걸어도 돼요.
- **`c2dm`은 예전 함수라 최신 MATLAB에는 없을 수 있어요.** `c2d(ss(Ac,Bc,Cc,0), Ts)`로 바꿔 쓰세요.
- **모델을 매 스텝 재선형화한다면** F와 Φ도 루프 안에서 다시 만들면 돼요.

각속도 제어 방식으로 가도 바뀌는 건 $A_m, B_m$뿐이고, 나머지 식은 똑같아요.

# 4. 검증 팁

책 튜토리얼이 권하는 방법이에요.

- MIMO로 고친 코드를 먼저 SISO 설정(Ap=0.8, Bp=0.1, Np=10, Nc=4)으로 돌려서, 예제 1.2의 $\Phi^T\Phi$ 값과 비교해 보세요.
- `QPhild`는 예제 2.10과 2.11의 답으로 확인하세요.
