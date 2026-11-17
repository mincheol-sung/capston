# 메카넘 AMR 토크 입력 MPC 1판 — 리눅스(ROS 2) 구현 지시서

| 항목 | 내용 |
|---|---|
| 기준 자료 | 랩미팅 발표 최종본 `261108_안정민_v15.pptx` (2026-11-13, 20쪽). 이 문서의 **식-N은 발표 최종본의 번호**를 따른다 |
| 보조 자료 | 설계 문서 `mecanum_torque_mpc_v1_edu.html` (식-11, 식-15, 식-22, 설정값·통과 시험), 구조도 `amr_controller_structure_v3.html` (노드·토픽). 둘 다 claude.ai 프로젝트 「AMR 개발」에 있다. 이 문서에서 가져온 식은 「설계 식-N」으로 표기한다 |
| 대상 | `torque_mpc_node` 하나 (ROS 2 Jazzy, Ubuntu 24.04, Intel NUC 15 Pro) |
| 작성 | 2026-11-16 |

---

## 0. 작업 규칙 (구현자는 먼저 읽을 것)

1. **식과 기호는 발표 최종본을 그대로 따른다.** 모델·비용·제약을 바꾸지 않는다. 발표에 없는 결정은 이 문서에서 **[구현 선택]** 으로 표시한 것만 따르고, 그 밖의 새 결정이 필요하면 구현을 멈추고 사용자에게 묻는다.
2. **수치를 지어내지 않는다.** 이 문서에 값이 없거나 "측정 필요"로 적힌 값은 파라미터로 두고 `TODO(측정)` 주석을 단다.
3. **용어는 교과서 표준 용어만 쓴다** (외란 추정치, 목표 입력, 입력 편차 항, 입력 변화 항, 등가 질량, 예측 구간, 제어 구간, 영공간 성분, 후퇴 지평 등). 코드 주석에는 대응 식 번호(`# 식-14`)를 적는다.
4. **담당 범위는 MPC 노드 하나뿐이다.** 기존 노드(`state_estimator`, `torque_output`, `ak40_can_node`, `command_gate`, `fault_monitor`)는 수정하지 않는다. 인터페이스가 이 문서와 다르면 MPC 노드의 입력 어댑터에서 맞추고, 차이를 보고한다.
5. **단계마다 시험을 통과시킨 뒤 다음 단계로 간다** (10절 순서). 각 단계가 끝나면 시험 결과(수치)를 보고한다.

---

## 1. 범위와 제어 루프

```
 시간 레퍼런스 맵 (CSV, 관성 좌표)          state_estimator (100 Hz)
   p_r(t), ṗ_r(t)  [기준 변환 없음]          p̂ (map), v̂ (몸체 좌표)
          │                                        │
          ▼                                        ▼
 ┌──────────────────────── torque_mpc_node (100 Hz) ───────────────────────┐
 │ ① 좌표 변환  ṗ̂ = R(θ̂) v̂,  θ̂ 언랩           → X̂                         │
 │ ② 외란 추정치  d̂ = X̂ − A X̂₋ − B(θ̂₋)U₋ − E ŵ₋  (식-9)  → â_d = d̂_ṗ / T_s  │
 │ ③ 기준: 보간·언랩 → Y_r, W_r, p̈_r → 목표 입력 U_s (식-14)                 │
 │ ④ 예측 행렬 F, Φ_U, Φ_w, Φ_d (식-12·13)                                   │
 │ ⑤ 비용 → H, g (식-16·17·18)   ⑥ 제약 τ̄, Δτ̄ (식-19·20·21) → OSQP         │
 │ ⑦ U* = 𝐔*[0:4] 발행 (순토크, N·m)                                         │
 └──────────────────────────────────┬──────────────────────────────────────┘
                                    ▼ U*
            torque_output (+τ_fric(ω), 전류 환산, 안전 클램프)  ← 기존 설계
                                    ▼ /wheel_torque_cmd
                         ak40_can_node → AK40-10 × 4               ← 기존 설계
```

- 피드포워드 블록과 포화 블록은 없다. 피드포워드는 비용 안의 기준 궤적 $Y_r$(예견)과 목표 입력 $U_s$ 로 들어가고, 포화는 QP의 하드 제약이 대신한다 (발표 4쪽).
- 슬립 제어기는 없다. 미끄럼 억제는 토크 상한 $\bar\tau$ (식-21)가 맡는다.
- **기준 궤적은 관성 좌표로 받는다.** 플래너의 몸체 속도를 관성 좌표로 돌리는 기준 변환은 두지 않는다. 플래너 연동은 추후 논의 사항이므로, 1판은 주행 전 1회 만든 **시간 레퍼런스 맵**(CSV)을 읽는다 (구조도 v3의 PLANNER 블록과 같은 방식).

---

## 2. 기호, 규약, 수치

### 2.1 좌표와 부호 규약

| 항목 | 규약 |
|---|---|
| 관성 좌표계 {O} | 지면 고정 프레임. 발표는 odom, 구조도 v3는 SLAM의 map으로 적었다. 상태와 기준이 **같은 고정 프레임**이면 식은 같다 → 파라미터 `fixed_frame` (기본 `map`) |
| 몸체 좌표계 {B} | `base_link`: +x 앞, +y 왼쪽, +z 위. 방향각 θ는 반시계가 양 |
| 바퀴 순서 | $U=[\tau_{FL},\ \tau_{FR},\ \tau_{RL},\ \tau_{RR}]^\top$ |
| 토크 부호 | 양의 토크 = 그 바퀴가 전진 방향으로 구르는 토크 |
| 롤러 배열 | X 배열. 접지 롤러 축 방향: FL·RR $(1,-1)$, FR·RL $(1,1)$ (CAD→URDF 검산 결과와 식-2의 부호가 일치) |
| 영공간 벡터 | $n=\tfrac12[1,\ 1,\ -1,\ -1]^\top$ (앞·뒤 바퀴 쌍이 서로 미는 조합, 몸체 힘 0) |

### 2.2 수치

| 기호 | 값 | 출처·비고 |
|---|---|---|
| $r$ | 0.04 m | 발표 |
| $\ell=l_x+l_y$ | 0.19835 m | 발표·설계 문서. **9월 CAD→URDF 문서는 $l_x+l_y$ = 0.094875 + 0.10005 = 0.194925 m** → 차이 1.7 %, 실측으로 확정할 것 |
| $m$ | 4 kg + 적재 하중 | 총질량(바퀴·적재 포함). 적재를 바꿀 때마다 갱신 |
| $J_w$ | 9.7×10⁻⁴ kg·m² | 바퀴 하나의 회전 관성(회전자 환산 포함) |
| $I_z$ | **측정 필요** | CAD 또는 제자리 회전 가속 시험. 시뮬레이션 임시값은 8.2절 |
| $m_w=4J_w/r^2$ | 2.425 kg | 식-3 |
| $m_{eq}=m+m_w$ | 6.425 kg (4 kg) | 식-3 |
| $I_{eq}=I_z+4J_w\ell^2/r^2$ | $I_z$ + 0.0954 kg·m² | 식-3 |
| $T_s$, $N_p$, $N_c$ | 10 ms, 50, 5 | 예측 구간 0.5 s, 결정변수 $4N_c$ = 20개 |
| $\tau_{lim}$ | 1.3 N·m | AK40-10 정격 (구조도 v3) |
| $\tau_{fric,max}$ | **측정 필요** | torque_output 마찰 보상 모델의 최댓값 |
| $\hat\mu$ | 노면마다 측정 | 토크 램프 시험 |
| $c_s$ | 0.8 | 가감속 하중 이동 여유 계수 |

---

## 3. 수식 명세 (발표 최종본)

### 3.1 상태와 입력 (발표 5·10쪽)

$$X=\begin{bmatrix}p\\ \dot p\end{bmatrix},\quad p=\begin{bmatrix}x\\y\\\theta\end{bmatrix},\quad U=\begin{bmatrix}\tau_{FL}\\\tau_{FR}\\\tau_{RL}\\\tau_{RR}\end{bmatrix}$$

- $p$: {O}에서 본 포즈, $\dot p$: {O} 성분 속도. 출력 $Y=CX$, $C=I_6$.
- 상태 추정기는 몸체 속도 $\hat v=[\hat v_x,\ \hat v_y,\ \hat{\dot\theta}]^\top$ 를 주므로 노드 입력부에서 $\hat{\dot p}=R(\hat\theta)\hat v$ 로 돌린다.
- $U$ 는 바퀴축 마찰을 뺀 **순토크**이다. 마찰 보상은 torque_output이 더한다.

### 3.2 토크 → 몸체 힘 (식-2, 발표 5쪽)

$$F_x=\frac{\tau_{FL}+\tau_{FR}+\tau_{RL}+\tau_{RR}}{r},\quad F_y=\frac{-\tau_{FL}+\tau_{FR}+\tau_{RL}-\tau_{RR}}{r},\quad M_z=\frac{\ell\,(-\tau_{FL}+\tau_{FR}-\tau_{RL}+\tau_{RR})}{r}$$

$$\begin{bmatrix}F_x\\F_y\\M_z\end{bmatrix}=(T^{-1})^\top U,\qquad (T^{-1})^\top=\frac1r\begin{bmatrix}1&1&1&1\\-1&1&1&-1\\-\ell&\ell&-\ell&\ell\end{bmatrix}$$

### 3.3 등가 질량과 운동방정식 (식-3·5, 발표 6–8쪽)

$$m_w=\frac{4J_w}{r^2},\quad m_{eq}=m+m_w,\quad I_{eq}=I_z+\frac{4J_w\ell^2}{r^2},\quad M=\mathrm{diag}(m_{eq},\ m_{eq},\ I_{eq})$$

$$m_{eq}\ddot x=F_x\cos\theta-F_y\sin\theta-m_w\dot\theta\dot y,\qquad m_{eq}\ddot y=F_x\sin\theta+F_y\cos\theta+m_w\dot\theta\dot x,\qquad I_{eq}\ddot\theta=M_z$$

### 3.4 제어 모델 (식-6, 발표 9쪽)

$$\ddot p=G(\theta)\,U+w,\qquad G(\theta)=M^{-1}R(\theta)(T^{-1})^\top\ (3\times4),\qquad w=\frac{m_w}{m_{eq}}\,\dot\theta\begin{bmatrix}-\dot y\\\dot x\\0\end{bmatrix},\qquad R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta&0\\\sin\theta&\cos\theta&0\\0&0&1\end{bmatrix}$$

검산 (설계 문서 06절): $\theta_r$ = 90°, $\dot p_r=[0,\ 1,\ 0.417]^\top$ 이면 $w=[-0.157,\ 0,\ 0]^\top$ m/s² ($2.425\times0.417/6.425$ = 0.157).

### 3.5 연속 시간 상태공간 모델 (식-7, 발표 10쪽)

$$\dot X=A_cX+B_c(\theta)U+E_cw,\qquad A_c=\begin{bmatrix}0&I_3\\0&0\end{bmatrix},\quad B_c(\theta)=\begin{bmatrix}0_{3\times4}\\G(\theta)\end{bmatrix},\quad E_c=\begin{bmatrix}0_{3\times3}\\I_3\end{bmatrix}$$

비선형 요소 $G(\theta)$ 와 $w$ 에는 **기준 궤적 값**을 넣는다 (LTV). 야코비안 선형화는 하지 않는다.

### 3.6 ZOH 이산화 (식-8, 발표 11쪽)

$$X(k+1)=A\,X(k)+B_k\,U(k)+E\,w(k),\qquad A=\begin{bmatrix}I_3&T_sI_3\\0&I_3\end{bmatrix},\quad B_k=\begin{bmatrix}\tfrac{T_s^2}{2}G_k\\T_sG_k\end{bmatrix},\quad E=\begin{bmatrix}\tfrac{T_s^2}{2}I_3\\T_sI_3\end{bmatrix}$$

- $G_k=G(\theta_{r,k})$: k번째 스텝의 **기준 방향각**을 넣은 식-6의 $G$.
- $A_c^2=0$ 이므로 행렬 지수의 급수가 두 항에서 끝나며, 위 식은 근사가 아니라 정확해이다. `scipy.linalg.expm` 을 쓰지 않는다.

### 3.7 외란 추정치 (식-9, 발표 11쪽) + 저역통과 (설계 식-11)

$$\hat d(k)=\hat X(k)-A\,\hat X(k-1)-B\big(\hat\theta(k-1)\big)\,U(k-1)-E\,\hat w(k-1)\ \in\mathbb R^6$$

$$\hat d_f(k)=(1-\alpha)\,\hat d_f(k-1)+\alpha\,\hat d(k),\qquad 0<\alpha\le1$$

시작값은 $\alpha=1$ (식-9 그대로)이다.

- 잔차 계산의 $B$, $\hat w$ 에는 기준이 아니라 **추정값**($\hat\theta(k-1)$, $\hat{\dot p}(k-1)$)을 쓴다. 실제로 일어난 일을 재는 식이기 때문이다. 예측(3.8절)의 $w_j$ 는 기준값이다.
- $U(k-1)$ 은 직전 주기에 발행한 순토크 $U^*$ 이다.
- $\hat d_f$ 는 매 주기 갱신하고, 한 번의 예측 구간 안에서는 상수로 쓴다.
- 검산: 모델에 없는 1 N의 힘이 $x$ 방향으로 작용하면 ($m_{eq}$ = 6.425 kg, $a$ = 0.1556 m/s²) $\hat d=[7.78\times10^{-6},\ 0,\ 0,\ 1.556\times10^{-3},\ 0,\ 0]^\top$ 이다.

### 3.8 예측 (식-12·13, 발표 12–13쪽)

예측 구간 안은 상대 인덱스를 쓴다: $X_0=\hat X(k)$, $U_i$, $w_i$, $B_i=B(\theta_{r,i})$ 는 $k+i$ 에서 $k+i+1$ 까지 작용한다.

$$X_j=A^jX_0+\sum_{i=0}^{j-1}A^{j-1-i}\big(B_iU_i+Ew_i\big)+\Big(\sum_{\sigma=0}^{j-1}A^\sigma\Big)\hat d_f$$

닫힌 형 블록 (행렬 거듭제곱을 계산하지 않는다):

$$A^j=\begin{bmatrix}I&jT_sI\\0&I\end{bmatrix},\quad A^{j-1-i}B_i=\begin{bmatrix}(j-i-\tfrac12)T_s^2\,G_i\\T_s\,G_i\end{bmatrix},\quad A^{j-1-i}E=\begin{bmatrix}(j-i-\tfrac12)T_s^2\,I\\T_s\,I\end{bmatrix},\quad \sum_{\sigma=0}^{j-1}A^\sigma=\begin{bmatrix}jI&\tfrac{j(j-1)}2T_sI\\0&jI\end{bmatrix}$$

쌓은 형태 (식-13):

$$Y=\begin{bmatrix}X_1\\\vdots\\X_{N_p}\end{bmatrix}=F\,X_0+\Phi_U\,\mathbf U+\Phi_w\,W_r+\Phi_d\,\hat d_f,\qquad \mathbf U=\begin{bmatrix}U_0\\\vdots\\U_{N_c-1}\end{bmatrix}$$

| 행렬 | 크기 | 블록 | 갱신 |
|---|---|---|---|
| $F$ | 300×6 | $F_j=A^j$ | 상수 |
| $\Phi_U$ | 300×20 | $(j,i)$ 블록 $=A^{j-1-i}B_i$ ($i<j$) | **매 주기** (기준 방향각) |
| $\Phi_w$ | 300×150 | $(j,i)$ 블록 $=A^{j-1-i}E$ ($i<j$) | 상수 |
| $\Phi_d$ | 300×6 | $\sum_{\sigma=0}^{j-1}A^\sigma$ | 상수 |

- **제어 구간 이후 토크 유지(move blocking)**: $i\ge N_c$ 인 스텝은 $U_i=U_{N_c-1}$ 이다. 따라서 $\Phi_U$ 의 마지막 블록 열은 $\sum_{i=N_c-1}^{j-1}A^{j-1-i}B_i$ 이다.
- $\Phi_U$ 는 `numpy` 로 벡터화해서 만든다 (`np.einsum` 또는 누적합). 스텝별 이중 루프는 시험용 기준 구현에만 쓴다.

### 3.9 기준 궤적 처리 (발표 14쪽, 설계 식-15)

시간 레퍼런스 맵에서 $t_j=t_k+jT_s$ ($j=0,\dots,N_p$, 51점) 의 $p_{r,j}$, $\dot p_{r,j}$ 를 선형 보간한다. $t_k$ 는 **상태 메시지의 타임스탬프**이다.

$$X_{r,j}=\begin{bmatrix}p_{r,j}\\\dot p_{r,j}\end{bmatrix},\quad Y_r=\begin{bmatrix}X_{r,1}\\\vdots\\X_{r,N_p}\end{bmatrix},\quad w_j=\frac{m_w}{m_{eq}}\dot\theta_{r,j}\begin{bmatrix}-\dot y_{r,j}\\\dot x_{r,j}\\0\end{bmatrix},\quad W_r=\begin{bmatrix}w_0\\\vdots\\w_{N_p-1}\end{bmatrix}$$

기준 가속도는 기준 속도의 중앙 차분이다 ($\Delta t$ = 0.2 s, 플래너 격자 간격과 같게 두어 꺾은선 속도의 계단을 사다리꼴로 편다). 기준이 가속도를 주면 그 값을 쓴다 (파라미터 `accel_source`).

$$\ddot p_{r,j}=\frac{\dot p_r(t_j+\Delta t/2)-\dot p_r(t_j-\Delta t/2)}{\Delta t}$$

각도 언랩 (설계 식-15): 첫 점을 현재 $\hat\theta$ 에 가장 가까운 가지로 옮기고, 이후 점은 앞 점과 연속이 되게 한다.

$$\theta_{r,0}\leftarrow\hat\theta+\mathrm{wrap}(\theta_{r,0}-\hat\theta),\qquad \theta_{r,j}\leftarrow\theta_{r,j-1}+\mathrm{wrap}(\theta_{r,j}-\theta_{r,j-1}),\qquad \mathrm{wrap}(\cdot)\in(-\pi,\ \pi]$$

- 검산: $\hat\theta$ = 3.0 rad, $\theta_{r,0}$ = −3.1 rad 이면 $\theta_{r,0}$ = 3.1832 rad (오차는 −6.1이 아니라 0.183 rad).
- $\hat\theta$ 자체도 연속값이어야 한다. Odometry의 쿼터니언은 ±π로 접혀 있으므로 노드가 $\hat\theta_k=\hat\theta_{k-1}+\mathrm{wrap}(\psi_k-\psi_{k-1})$ 로 누적한다.

### 3.10 목표 입력 (식-14, 발표 14쪽)

로봇이 이미 기준 궤적 위에 있고 모델이 맞다고 가정할 때 추종에 필요한 토크이다. 식-6을 $U$ 에 대해 최소 노름으로 푼다 (9쪽 계산의 역순: $M$ → $R^\top$ → 토크 분배).

$$U_{s,j}=G(\theta_{r,j})^{+}\big(\ddot p_{r,j}-w_j-\hat a_d\big)=\big((T^{-1})^\top\big)^{+}R(\theta_{r,j})^\top M\big(\ddot p_{r,j}-w_j-\hat a_d\big),\qquad \hat a_d=\frac1{T_s}\begin{bmatrix}\hat d_{f,\dot x}\\\hat d_{f,\dot y}\\\hat d_{f,\dot\theta}\end{bmatrix}$$

$$\big((T^{-1})^\top\big)^{+}=\frac r4\begin{bmatrix}1&-1&-1/\ell\\1&1&1/\ell\\1&1&-1/\ell\\1&-1&1/\ell\end{bmatrix},\qquad \mathbf U_s=\begin{bmatrix}U_{s,0}\\\vdots\\U_{s,N_c-1}\end{bmatrix}$$

- $(T^{-1})^\top$ 의 행이 서로 직교하므로 의사역행렬 $G^\top(GG^\top)^{-1}$ 이 위의 닫힌 형이다. 네 열이 모두 $n$ 과 직교하므로 $n^\top U_{s,j}=0$ (영공간 성분 0).
- 검산 (통과 시험 1의 값): 4 kg, 1 m/s, 반지름 2.4 m 등속 선회, $\theta_r$ = 90°, $\hat a_d=0$ → 몸체 구심력 $F_s=[0,\ 1.67,\ 0]$ N → $U_s=[-0.0167,\ 0.0167,\ 0.0167,\ -0.0167]$ N·m.

### 3.11 비용 (식-16, 발표 15쪽)

$$J=\sum_{j=1}^{N_p}\|X_j-X_{r,j}\|_Q^2+\sum_{j=0}^{N_c-1}\|\Delta U_j\|_{R_\Delta}^2+\sum_{j=0}^{N_c-1}\|U_j-U_{s,j}\|_{R_U}^2$$

$$\|e\|_Q^2=e^\top Qe,\quad \Delta U_j=U_j-U_{j-1},\quad U_{-1}=U(k-1),\quad R_\Delta=r_\Delta I_4,\quad R_U=r_UI_4$$

- 추종 오차 항: 예측 구간 전체의 기준 궤적 반영(예견).
- 입력 변화 항: 이웃 스텝 토크의 차이. 정상상태에서 0. 토크 급변 억제.
- 입력 편차 항: 목표 입력과의 차이. 정상상태에서 0. 과구동으로 생기는 영공간 성분 억제 ($U_s$ 가 최소 노름 해라 영공간 성분 0).

### 3.12 QP 표준형 (식-17·18, 발표 16·20쪽)

$$[D\mathbf U-e_1\otimes U_{-1}]_j=\Delta U_j,\quad \bar Q=I_{N_p}\otimes Q,\quad \bar R_\Delta=I_{N_c}\otimes R_\Delta,\quad \bar R_U=I_{N_c}\otimes R_U$$

위가 식-17, 아래가 식-18이다.

$$J=\mathbf U^\top H\,\mathbf U+2g^\top\mathbf U+\mathrm{const}$$

$$H=\Phi_U^\top\bar Q\,\Phi_U+D^\top\bar R_\Delta D+\bar R_U,\qquad g=\Phi_U^\top\bar Q\big(FX_0+\Phi_wW_r+\Phi_d\hat d_f-Y_r\big)-D^\top\bar R_\Delta(e_1\otimes U_{-1})-\bar R_U\mathbf U_s$$

- $D$: 대각 블록 $I_4$, 바로 아래 블록 $-I_4$ 인 $4N_c\times4N_c$ 차분 행렬. $e_1\otimes U_{-1}$: 첫 블록만 $U_{-1}$.
- $H$ 는 헤시안의 ½ 이다. OSQP의 $\tfrac12\mathbf U^\top P\mathbf U+q^\top\mathbf U$ 꼴에는 $P=2H$, $q=2g$ 를 넘긴다 ($P=H$, $q=g$ 도 최소점은 같다).
- $\bar Q$ 가 대각이므로 $\Phi_U^\top(\bar Q\Phi_U)$ 는 `Phi.T @ (qdiag[:, None] * Phi)` 한 번으로 계산한다.

### 3.13 제약 (식-19·20·21, 발표 17쪽)

$$-\bar\tau\le\tau_{i,j}\le\bar\tau,\qquad -\Delta\bar\tau\le\tau_{i,j}-\tau_{i,j-1}\le\Delta\bar\tau,\qquad \tau_{i,-1}=\tau_i(k-1)$$

$$\bar\tau=\min\Big(\tau_{lim}-\tau_{fric,max},\ \ \frac{r}{\sqrt2}\,\hat\mu\,\frac{mg}{4}\,c_s\Big),\qquad \Delta\bar\tau=\bar\tau/3$$

위 두 식이 순서대로 식-19·20(상한, 변화율)과 식-21(상한값)이다.

행렬 꼴 (OSQP의 $l\le A\mathbf U\le u$):

$$\begin{bmatrix}-\bar\tau\mathbf 1\\-\Delta\bar\tau\mathbf 1+e_1\otimes U_{-1}\end{bmatrix}\le\begin{bmatrix}I_{4N_c}\\D\end{bmatrix}\mathbf U\le\begin{bmatrix}\bar\tau\mathbf 1\\\Delta\bar\tau\mathbf 1+e_1\otimes U_{-1}\end{bmatrix}$$

- 제약 행렬 $[I;\ D]$ 는 상수이다. 매 주기 바뀌는 것은 $l$, $u$ 의 $U_{-1}$ 부분뿐이다.
- 검산: $\hat\mu$ = 0.3, $c_s$ = 0.8 → $\bar\tau$ = 0.0666 / 0.1665 / 0.3330 N·m (4 / 10 / 20 kg), $\Delta\bar\tau$ = 0.0222 N·m (4 kg). 정격 1.3 N·m보다 훨씬 작으므로 이 노면에서는 마찰 항이 항상 상한을 정한다.
- 검산 (설계 문서 08절): $N_c$ = 2, 한 바퀴, $\tau_{i,-1}$ = 0.1, $\Delta\bar\tau$ = 0.03 → $\tau_{i,0}\in[0.07,\ 0.13]$.

### 3.14 출력 (설계 식-22)

$$U^*(k)=\mathbf U^*[0{:}4],\qquad U_{cmd}(k)=U^*(k)+\tau_{fric}(\omega(k))$$

$\tau_{fric}(\omega)$ 는 torque_output이 더한다.

나머지 $\mathbf U^*[4{:}]$ 는 다음 주기의 웜 스타트에만 쓴다.

---

## 4. 한 주기의 계산 순서

| 단계 | 계산 | 식 | 언제 |
|---|---|---|---|
| 0 | $A$, $E$, $F$, $\Phi_w$, $\Phi_d$, $D$, $D^\top\bar R_\Delta D+\bar R_U$, $((T^{-1})^\top)^+$, $[I;\ D]$, $\bar\tau$, $\Delta\bar\tau$, OSQP 설정(희소 구조 고정) | 식-8·13·17·21 | 시작 시, 파라미터 변경 시 |
| 1 | 상태 수신 → $\hat\theta$ 언랩, $\hat{\dot p}=R(\hat\theta)\hat v$, $\hat X$ | 식-1 | 매 주기 |
| 2 | $\hat d$ → $\hat d_f$ (저역통과), $\hat a_d$ | 식-9, 설계 식-11 | 매 주기 |
| 3 | 51점 보간(앞뒤 0.1 s 여유) → 언랩 → $Y_r$, $W_r$, $\ddot p_{r,j}$ → $\mathbf U_s$ | 식-14, 설계 식-15 | 매 주기 |
| 4 | $G_i=M^{-1}R(\theta_{r,i})(T^{-1})^\top$, $i=0..N_p-1$ → $\Phi_U$ | 식-12·13 | 매 주기 |
| 5 | $H$, $g$ | 식-18 | 매 주기 |
| 6 | $l$, $u$ 갱신 → OSQP 갱신(`Px`, `q`, `l`, `u`) → 웜 스타트(지난 $\mathbf U^*$ 를 한 블록 민 것) → 풀이 | 식-19·20 | 매 주기 |
| 7 | $U^*$ 발행, $\hat X_{prev}\leftarrow\hat X$, $U_{prev}\leftarrow U^*$, $\hat w_{prev}\leftarrow\hat w(\hat X)$, 로그 | 설계 식-22 | 매 주기 |

**[구현 선택] 주기 구동 방식**: MPC 계산은 **상태 메시지 수신 콜백**에서 돌린다 (상태 추정기가 100 Hz로 발행). 이유: 식-9는 $\hat X(k)$ 와 $\hat X(k-1)$ 이 정확히 $T_s$ 떨어져 있다고 가정한다. 별도 타이머로 돌리면 추정기와 위상이 어긋나 같은 상태를 두 번 읽는 주기가 생기고, 그때 잔차는 $\hat d_x\approx-T_s\hat{\dot x}$ (1 m/s에서 −1 cm)가 되어 예측이 크게 틀어진다. 수신 간격이 $T_s$ ± 2 ms를 벗어난 주기는 $\hat d$ 갱신을 건너뛰고 이전 값을 유지하며, 횟수를 로그에 남긴다.

---

## 5. 소프트웨어 구조

### 5.1 의존성

```bash
sudo apt install python3-numpy python3-scipy python3-pytest python3-matplotlib
python3 -m pip install --user --break-system-packages "osqp>=1.0,<2"   # 검증은 osqp 1.1.3
```

- 언어는 Python(rclpy)으로 시작한다. 근거: 결정변수 20개, 제약 40행(양측)의 작은 QP이고, 같은 식의 Python 시제품이 한 주기 중앙값 1.0 ms, 상위 1 % 1.8 ms였다 (2.1 GHz 클라우드 코어 1개, 8.4절).
- **[구현 선택] C++ 전환 조건**: 실차 NUC에서 계산 시간 상위 1 %가 2 ms(주기의 20 %)를 넘거나 지터가 문제가 되면, 코어(`controller.py`)만 C++(Eigen + OSQP)로 옮긴다. 단위 시험 기대값은 그대로 쓴다.
- 기존 워크스페이스에 `wheel_mpc` 노드의 `ReferenceHorizon.horizon(now, T_s, N_p)` 가 있으면 **보간 부분만** 재사용한다 ($T_s$ 0.04 → 0.01, $N_p$ 20 → 50). 몸체 → 관성 기준 변환은 쓰지 않는다.

### 5.2 패키지 구성

```
<기존 ROS 2 워크스페이스>/src/torque_mpc/          (ament_python)
├── torque_mpc/
│   ├── params.py         # 물리·설계 파라미터와 유도량 (m_w, m_eq, I_eq, M, τ̄, Δτ̄)      식-3, 식-21
│   ├── model.py          # (T⁻¹)ᵀ, 의사역행렬, R(θ), G(θ), w, A, B(θ), E                 식-2, 6, 8
│   ├── prediction.py     # F, Φ_U(벡터화 + 기준 루프 구현), Φ_w, Φ_d                     식-12, 13
│   ├── reference.py      # 시간 레퍼런스 맵 로드, 보간, 언랩, 중앙 차분, Y_r, W_r          설계 식-15
│   ├── target_input.py   # U_s                                                          식-14
│   ├── disturbance.py    # d̂, 저역통과, 갱신 건너뛰기 규칙                                식-9, 설계 식-11
│   ├── qp.py             # H, g, l, u, OSQP 래퍼(희소 구조 고정, 웜 스타트)               식-16~20
│   ├── controller.py     # TorqueMPC.step(X̂, t_k) → U*, info   (ROS 비의존, 4절 순서)
│   ├── node.py           # rclpy 노드: 구독·발행·상태 머신·안전
│   └── figure8.py        # 8자 시간 레퍼런스 맵 생성기 → CSV
├── sim/
│   ├── plant.py          # 강체 플랜트(식-5), 슬립 플랜트(8.2절)
│   ├── run_pass_tests.py # 통과 시험 1~4, 그래프 저장
│   └── sim_plant_node.py # ROS 폐루프 시험용 플랜트 노드
├── test/                 # pytest 단위 시험 (8.1절)
├── config/torque_mpc.yaml
└── launch/torque_mpc.launch.py
```

`controller.py` 는 ROS를 import하지 않는다. 오프라인 시뮬레이션, 단위 시험, ROS 노드가 같은 코어를 쓴다.

### 5.3 ROS 2 인터페이스

| 방향 | 토픽 (파라미터 이름) | 형식 | 주기 | 비고 |
|---|---|---|---|---|
| 입력 | 상태 (`state_topic`) | `nav_msgs/Odometry` | 100 Hz | pose: `fixed_frame` 기준, twist: `base_link`(몸체) 기준. **기존 추정기의 실제 토픽·형식을 확인해 맞춘다** |
| 입력 | 시간 레퍼런스 맵 (`reference_csv`) | CSV 파일 | 주행 전 1회 | 5.4절 형식 |
| 출력 | 순토크 (`tau_topic`) | `std_msgs/Float64MultiArray` | 100 Hz | `[FL, FR, RL, RR]` N·m. **torque_output의 기존 입력 토픽·형식에 맞춘다** |
| 출력 | 디버그 (`~/debug`) | `std_msgs/Float64MultiArray` | 100 Hz | 아래 배치 |
| 서비스 | `~/start`, `~/stop` | `std_srvs/Trigger` | — | 기준 시계 시작 / 정지 래치 |

디버그 배열 배치 (slip_logger·rosbag 기록용):

| 인덱스 | 내용 |
|---|---|
| 0 | $t_k$ (기준 시계, s) |
| 1–6 | $\hat X$ |
| 7–12 | $X_{r,0}$ |
| 13–16 | $U^*$ |
| 17–20 | $U_{s,0}$ |
| 21–26 | $\hat d_f$ |
| 27 | 활성 제약 수 |
| 28 | QP 반복 수 |
| 29 | 계산 시간 (ms) |
| 30 | OSQP 상태 코드 |
| 31 | $\hat d$ 갱신 건너뜀 플래그 |

QoS: 상태 구독은 `SensorDataQoS`(depth 1), 토크 발행은 reliable(depth 1).

### 5.4 시간 레퍼런스 맵 (CSV)

```
t,x,y,theta,xdot,ydot,thetadot[,xddot,yddot,thetaddot]
```

- `fixed_frame` 기준, `theta` 는 연속값(언랩), `t` 는 시작 시각 0 기준 초, 균일 격자(10 ms 권장).
- 마지막 행 뒤는 마지막 포즈·속도 0으로 유지한다. 첫 행 앞(중앙 차분의 −0.1 s)은 첫 행 값으로 유지한다.
- **[구현 선택]** `anchor_to_start_pose: true` 이면 `~/start` 순간의 로봇 포즈로 기준 전체를 강체 변환(평행 이동 + 회전, 속도는 회전만)한다. 바퀴를 띄운 시험과 임의 위치 출발에 쓴다.

`figure8.py`: 반지름 $R$ = 2.4 m 원 두 개를 원점에서 이은 8자(한 바퀴 $4\pi R$ = 30.2 m, 구조도 v3의 30 m 8자). 원 A(중심 $(0, R)$)는 반시계, 원 B(중심 $(0,-R)$)는 시계 방향. 방향각 = 경로 접선, 횡속도 기준 0, $\dot\theta_r=v\kappa$ ($\kappa=\pm1/R$). 속도 프로파일: 0에서 가속 0.5 m/s² **[구현 선택]** 으로 $v$ (1판 시험 1 m/s)까지, 등속, 같은 감속으로 정지. 바퀴 수 파라미터(기본 2).

### 5.5 상태 머신과 안전

| 상태 | 동작 |
|---|---|
| `IDLE` | 토크 0 발행. 상태 수신만 확인 |
| `RUN` | `~/start` 로 진입. 기준 시계 $t=0$, **$\hat X_{prev}$, $U_{prev}$, $\hat d_f$, 웜 스타트를 0으로 초기화** |
| `STOP` | 토크 0 발행. 진입할 때 위와 같이 초기화. 명령 이력이 끊기면 잔차가 뜻을 잃기 때문이다 |

**[구현 선택] 전이 조건** (값은 시작값, 시험하며 조정):

- 시작 거부: `~/start` 시점의 위치 오차 > 0.1 m 또는 방향 오차 > 10°. $B$ 를 기준 방향각으로 지었기 때문에 방향 오차가 크면 토크 방향이 틀린다 (설계 문서 12절: 5°에서 힘의 9 %가 횡으로 샌다).
- 상태 수신 끊김: 마지막 수신 후 30 ms(3주기) → `STOP`. 감속은 command_gate의 정지 처리(감속 토크·댐핑)에 맡긴다.
- QP 실패(OSQP 상태가 solved가 아님): 지난 해를 한 블록 민 값의 첫 블록을 상자·변화율 제약으로 잘라 발행. 3주기 연속 실패 → `STOP`.
- 기준 종료: 마지막 기준점(정지)을 계속 추종하다가, 기준 종료 후 0.5 s 지나고 $\|\hat{\dot p}_{xy}\|$ < 0.02 m/s 이면 정지 래치(`STOP`).
- 파라미터($m$, $\hat\mu$, $I_z$ 등) 변경은 `IDLE`/`STOP` 에서만 받는다. 주행 중 $\bar\tau$ 가 $\Delta\bar\tau$ 이상 줄면 $|U_{-1}|>\bar\tau+\Delta\bar\tau$ 가 되어 QP가 실행 불가능해진다.

### 5.6 파라미터 파일 (`config/torque_mpc.yaml`)

```yaml
torque_mpc_node:
  ros__parameters:
    # 물리 (2.2절)
    r: 0.04
    ell: 0.19835            # TODO(측정): CAD 문서 0.194925와 대조
    m: 4.0                  # 총질량 + 적재
    Iz: -1.0                # TODO(측정): 음수면 노드가 시작을 거부
    Jw: 9.7e-4
    # 설계 (설계 문서 10절 시작값)
    Ts: 0.01
    Np: 50
    Nc: 5
    Q_diag: [100.0, 100.0, 50.0, 1.0, 1.0, 0.5]   # 위치 1 cm ≈ 속도 0.1 m/s ≈ 방향 0.8°
    r_delta: 1.0e-2
    r_u: 1.0e-2                                   # r_delta 이하에서 시작
    alpha: 1.0                                    # 토크가 떨리면 0.1까지
    dt_acc: 0.2
    accel_source: central_difference              # 또는 reference
    # 제약
    mu_hat: 0.3             # 노면마다 측정
    c_s: 0.8
    tau_lim: 1.3
    tau_fric_max: 0.0       # TODO(측정)
    dtau_ratio: 3.0         # Δτ̄ = τ̄ / dtau_ratio
    # QP
    osqp_eps_abs: 1.0e-7
    osqp_eps_rel: 1.0e-7
    osqp_max_iter: 4000
    osqp_polishing: false
    # 인터페이스
    fixed_frame: map
    state_topic: /state_estimator/odom   # TODO: 기존 추정기 토픽
    tau_topic: /torque_mpc/tau_cmd       # TODO: torque_output 입력 토픽
    reference_csv: ""
    anchor_to_start_pose: false
    # 안전 [구현 선택]
    state_timeout: 0.03
    dt_tolerance: 0.002
    start_max_pos_err: 0.1
    start_max_yaw_err_deg: 10.0
    max_consecutive_qp_fail: 3
    stop_hold_time: 0.5
    stop_speed: 0.02
    d_skip_threshold: 1.0e-3             # 7.4절
```

---

## 6. 상태 추정기에 요구할 사항 (담당자에게 전달)

MPC 노드는 추정기를 고치지 않는다. 다음은 추정기 담당자와 합의할 항목이다.

1. 100 Hz 발행, 메시지 타임스탬프는 측정 시각.
2. 포즈는 `fixed_frame`, 속도는 몸체 좌표. 단위 m, rad, m/s, rad/s.
3. **SLAM 보정 시점 표시**: 10 Hz SLAM 리셋이 포즈를 계단처럼 바꾸면 식-9의 잔차에 그 계단이 그대로 들어간다 (7.4절). 보정 시점을 알려 주면(별도 플래그 토픽 또는 공분산 필드) MPC가 그 주기의 $\hat d$ 갱신을 건너뛴다.
4. **정지 시 속도 0**: 차가 서 있는데 추정 속도가 0이 아니면, MPC는 그 속도를 실제로 믿고 토크를 낸다. 그 토크가 정지 마찰을 넘으면 로봇이 실제로 밀린다. `/wheel_states` 의 바퀴 속도가 모두 작을 때 속도를 0으로 두는 영속도 갱신(ZUPT, zero-velocity update)과 정지 중 IMU 바이어스 재추정을 요청한다.

---

## 7. 구현할 때 생기는 문제 (발표 범위 밖)

### 7.1 QP 허용오차

토크 크기가 10⁻² N·m 단위이다. OSQP 기본 허용오차 10⁻³ 은 부록 D 해 0.0241 N·m의 4 %에 해당하는 크기라, 해의 정확도를 보장하지 못한다. `eps_abs = eps_rel = 1e-7` 로 둔다 (시제품: 75회 반복, 닫힌 형 해와의 차 3×10⁻¹¹ N·m). `polishing` 은 osqp 1.1.x에서 `verbose=False` 여도 주기마다 안내 문구를 출력하므로 끈다.

### 7.2 OSQP 희소 구조 고정

$P=2H$ 는 20×20 밀집 행렬이다. 설정할 때 **상삼각 전체 원소(0 포함)를 명시적으로 넣은 CSC**를 만들고, 그 `indices`/`indptr` 순서를 저장한다. 매 주기에는 같은 순서로 `data` 만 채워 `update(Px=...)` 로 넘긴다. 밀집 행렬에서 매번 `scipy.sparse.csc_matrix(np.triu(2*H))` 로 새로 만들면 우연히 0인 원소가 빠져 구조가 바뀌고, `Px` 길이가 달라지거나 값이 엉뚱한 자리에 들어간다.

### 7.3 바퀴 순서와 부호

토크 배열 순서와 부호가 하나라도 틀리면 영공간 성분이 몸체 힘으로 바뀌어 발산한다. 실차 전 점검(8.5절)에서 바퀴별로 확인한다. 좌우 모터는 장착 방향이 반대이므로 부호 반전은 `ak40_can_node` 또는 `torque_output` 의 기존 설정에서 처리되는지 확인한다.

### 7.4 SLAM 리셋과 외란 추정치

예측에서 $\hat d$ 의 위치 성분은 $j$ 번 더해진다 ($\Phi_d$ 의 $jI$). 한 주기에 포즈가 SLAM 보정으로 1 cm 뛰면 $\hat d_x$ = 0.01 m가 되고, 예측 구간 끝($j$ = 50)의 위치 예측이 0.5 m 밀린다. QP는 그 주기에 큰 토크를 내려 하고, 변화율 제약 $\Delta\bar\tau$ 만이 그것을 막는다.

**[구현 선택]** 대책 순서:
1. 추정기가 보정 시점을 알려 주면 그 주기의 $\hat d$ 갱신을 건너뛴다 (이전 $\hat d_f$ 유지, $\hat X_{prev}$ 는 갱신).
2. 알릴 수 없으면 $|\hat d_x|$, $|\hat d_y|$ 가 `d_skip_threshold` (1×10⁻³ m)를 넘는 주기를 건너뛴다. 정상 크기는 1 N 외란에 7.8×10⁻⁶ m이므로 두 자릿수 이상 크다.
3. 그래도 토크가 10 Hz로 떨리면 $\alpha$ 를 줄인다 ($\alpha$ = 0.1이면 시정수 $T_s/\alpha$ = 0.1 s).

### 7.5 선회 중 외란은 관성 좌표에서 회전한다

몸체에 붙은 외란(질량 오차, 진행 방향 구름 저항)은 선회 중 관성 좌표에서 방향이 돈다. $\hat d$ 는 예측 구간 안에서 상수이므로 이 회전을 따라가지 못해 정상상태 오차가 정확히 0이 되지는 않는다. 크기는 작다: 시제품에서 제어기 10 kg, 플랜트 20 kg, 1 m/s, 반지름 2.4 m일 때 위치 오차 0.16 mm. 직선 주행과 관성 좌표에서 일정한 외란(경사 등)은 오차 0이다. 통과 시험 2(b)의 기준을 이에 맞춘다.

---

## 8. 시험

### 8.1 단위 시험 (pytest, ROS 없이)

| 번호 | 조건 | 기대값 | 허용오차 |
|---|---|---|---|
| U1 | 유도량, $m$ = 4 kg | $m_w$ = 2.425, $m_{eq}$ = 6.425, $I_{eq}-I_z$ = 0.09541 | 1e-6 |
| U2 | $\bar\tau$, $\hat\mu$ = 0.3, $c_s$ = 0.8, $\tau_{fric,max}$ = 0 | 0.06659 / 0.16648 / 0.33296 N·m (4/10/20 kg), $\Delta\bar\tau$ = 0.02220 (4 kg) | 1e-5 |
| U3 | 식-2 ↔ 의사역행렬 | $(T^{-1})^\top\big((T^{-1})^\top\big)^+=I_3$, $n^\top\big((T^{-1})^\top\big)^+=0$ | 1e-12 |
| U4 | 닫힌 형 블록 | $A^j$, $\sum A^\sigma$ 를 `np.linalg.matrix_power` 결과와 비교; 벡터화 $\Phi_U$ 를 이중 루프 구현과 비교 | 1e-12 |
| U5 | $w$ ($\theta_r$ = 90°, $\dot p_r=[0,1,0.417]$) | $[-0.1574,\ 0,\ 0]$ m/s² | 1e-4 |
| U6 | 언랩 ($\hat\theta$ = 3.0, $\theta_{r,0}$ = −3.1) | 3.18319 rad | 1e-5 |
| U7 | $\hat d$: 1 N 외란, 한 스텝 | $[7.782\times10^{-6},0,0,1.5564\times10^{-3},0,0]$ | 1e-9 |
| U8 | 부록 D: $N_p=N_c$ = 1, 제약 없음, 정지, 기준 1 cm 전방, $Q$ = diag(100,100,100,1,1,1), $r_\Delta=r_U$ = 0.001 | $U^*$ = 0.0241 N·m × 4 (네 바퀴 같음, 영공간 성분 0) | 1e-4 |
| U9 | $N_p$ = 50, 기준 = 현재 포즈(정지) | $U^*=0$ | 1e-9 |
| U10 | 목표 입력: 4 kg, 1 m/s, R 2.4 m, $\theta_r$ = 90°, 해석 가속도 | $U_s=[-0.01667,\ 0.01667,\ 0.01667,\ -0.01667]$, $n^\top U_s=0$ | 1e-4 |
| U11 | U10 상태에서 $\hat X=X_{r,0}$, $U_{-1}=U_{s,0}$, 한 주기 | $U^*\approx U_{s,0}$ (시제품 차 4.9×10⁻⁵) | 1e-4 |
| U12 | 제약 행 ($N_c$ = 2, $\tau_{i,-1}$ = 0.1, $\Delta\bar\tau$ = 0.03) | $\tau_{i,0}$ 범위 [0.07, 0.13] | 1e-12 |
| U13 | OSQP 해 vs 닫힌 형 $-H^{-1}g$ (활성 제약 없는 경우) | 일치 | 1e-8 |

U8의 손 계산 (발표 20쪽): 가속도 $4t/(0.04\times6.425)$ = 15.56 $t$ → 위치 변화 7.78×10⁻⁴ $t$, 속도 변화 0.156 $t$. $J(t)=100(7.78\times10^{-4}t-0.01)^2+(0.156t)^2+0.004t^2+0.004t^2$, $dJ/dt=0$ → $t^*=1.56\times10^{-3}/0.0648$ = 0.0241. U8은 기본 제약을 끄고 돌린다 ($\Delta\bar\tau$ = 0.0222 < 0.0241 이라 켜면 변화율 제약이 걸린다).

### 8.2 오프라인 폐루프 시뮬레이션 (ROS 없이)

**강체 플랜트**: 식-5를 참 파라미터로 그대로 적분한다. RK4, 내부 1 ms, 토크는 10 ms 동안 ZOH. 모델 밖 외력 $f_{ext}$(관성 좌표)를 더할 수 있게 한다:
$\ddot p=M_t^{-1}\big(R(\theta)(T^{-1})^\top U+f_{ext}\big)+w_t$.

**슬립 플랜트** (통과 시험 3용): 바퀴 회전 자유도 4개를 더하고, 접지력을 롤러 축 방향의 정규화한 쿨롱 마찰로 둔다. 바퀴 위치 $(x_i,y_i)$: FL $(+l_x,+l_y)$, FR $(+l_x,-l_y)$, RL $(-l_x,+l_y)$, RR $(-l_x,-l_y)$; 롤러 축 단위벡터 $e_i=[1,\ c_i]/\sqrt2$, $c=[-1,\ 1,\ 1,\ -1]$.

$$v_{c,i}=\begin{bmatrix}v_x-\dot\theta y_i\\v_y+\dot\theta x_i\end{bmatrix},\quad s_i=e_i^\top v_{c,i}-\frac{r\omega_i}{\sqrt2},\quad f_i=-\mu N_i\tanh(s_i/s_0),\quad N_i=\frac{mg}4$$

$$m\,\ddot p_{xy}=R_2(\theta)\sum_i f_ie_i,\qquad I_z\ddot\theta=\sum_i\frac{f_i(x_ic_i-y_i)}{\sqrt2},\qquad J_w\dot\omega_i=\tau_i-\frac{r}{\sqrt2}f_i$$

- $s_0$ 는 시뮬레이션 정규화 속도(예 0.01 m/s), 내부 적분 0.1 ms. $\mu$ 를 크게 두면 강체 플랜트와 같아져야 한다 (시제품: $\mu$ = 5, $s_0$ = 0.002에서 미끄럼 속도 < 0.6 mm/s, 선회 추종 오차 4×10⁻⁶ m) — 플랜트 식 자체의 검산이다.
- $I_z$ 시뮬레이션 임시값: 한 변 $\ell$ 의 정사각 균일 판 근사 $m\cdot2\ell^2/12$ = 0.026 kg·m² (4 kg). **실차 파라미터로 쓰지 않는다.**

### 8.3 통과 시험 (설계 문서 11절 + 시제품 참고값)

참고값은 이 문서의 식대로 만든 Python 시제품(강체 플랜트, $I_z$ = 0.026 임시값)의 결과이다. 구현이 같은 결과를 내는지 대조하는 데 쓴다.

| 시험 | 조건 | 통과 기준 | 시제품 참고값 |
|---|---|---|---|
| 1a 직선 | 플랜트 = 모델, 4 kg, 0.5 m/s² 가속 → 1 m/s 등속 | 등속 구간 오차 → 0 | 6×10⁻¹³ m |
| 1b 등속 선회 | 플랜트 = 모델, 4 kg, 1 m/s, R 2.4 m | 10 s 이후 $\vert e_{xy}\vert$ < 1e-6 m, $U^*\to U_s$ (±1e-4), $\vert n^\top U^*\vert$ < 1e-9 | $\vert e_{xy}\vert$ 5.8×10⁻⁸ m, $U^*=[-0.01667, 0.01667, 0.01667, -0.01667]$, $\vert n^\top U^*\vert$ 10⁻¹³ |
| 2a 질량 갱신 | 플랜트 20 kg, 제어기 $m$ = 20 | 1b와 같은 등급 | $\vert e_{xy}\vert$ 7×10⁻⁷ m |
| 2b 질량 미갱신 | 플랜트 20 kg, 제어기 $m$ = 10, 선회 | 과도 후 $\vert e_{xy}\vert$ < 1 mm, $\hat d_f$ 속도 성분이 선회를 따라 회전 (7.5절) | $\vert e_{xy}\vert$ 1.6×10⁻⁴ m |
| 2c 모델 밖 외력 | 강체 플랜트에 −1 N ($x$), 직선 1 m/s | 등속 구간 오차 0, $\hat a_{d,x}$ = −1/6.425 = −0.1556 m/s² | 오차 6×10⁻¹³ m, $\hat a_{d,x}$ = −0.15564 |
| 3 마찰 상한 | 슬립 플랜트 $\mu$ = 0.3, 4 kg, 2 m/s² 가속 기준 → 1 m/s | 가속 중 $\vert U\vert =\bar\tau$ (활성 제약 > 0), 미끄럼이 상한 없을 때보다 작음, 발산 없음 | 상한 있음: $\max\vert U\vert$ 0.0666, 최대 미끄럼 속도 5.5 mm/s, 최대 추종 오차 0.25 m, 3 s에 오차 0 / 상한 없음($\bar\tau$ = 1.3): 미끄럼 속도 37.8 m/s, 최대 오차 0.51 m, 3 s에 0.43 m |
| 4 8자 두 바퀴 | 강체 플랜트, 4 kg, 1 m/s, 5.4절 기준 | $e_\theta$ 에 2π 점프 없음, $\vert U\vert \le\bar\tau$ | $\max\vert e_{xy}\vert$ 3.6 mm, $\max\vert e_\theta\vert$ 0.027 rad(곡률 전환점), $\max\vert U\vert$ 0.0536 N·m |

로그: $\hat X$, $X_{r,0}$, $U^*$, $U_s$, $\hat d_f$, 활성 제약 수, QP 반복 수, 계산 시간. 시험마다 위치 오차·토크·$\hat d_f$ 시계열 그래프를 `sim/out/` 에 저장한다.

### 8.4 ROS 2 폐루프 시험

`sim_plant_node` 가 강체(또는 슬립) 플랜트를 100 Hz로 적분해 `nav_msgs/Odometry` 를 발행하고 토크를 구독한다. `torque_mpc_node` 와 묶어 시험 1b·4를 다시 돌린다.

- 통과 기준: 8.3절과 같은 결과, 계산 시간 상위 1 % < 2 ms, 상태 수신 간격 이탈 횟수 0.
- 시제품 계산 시간 (2.1 GHz 클라우드 코어): 중앙값 1.0 ms, 상위 1 % 1.8 ms.
- rosbag2로 상태·토크·디버그 토픽을 기록한다.
- (선택) Isaac Sim의 `amr_description` 모델로 같은 시험을 한다.

### 8.5 실차 전 점검 (바퀴를 띄운 상태)

1. 바퀴별 부호: 한 바퀴씩 +0.02 N·m (토크 출력 노드 경유)를 짧게 주고, 그 바퀴가 전진 방향으로 도는지, `/wheel_states` 의 속도 부호가 양인지 확인한다.
2. 영공간 확인: $U=0.02\,[1,1,-1,-1]$ 을 주면 앞·뒤 쌍이 서로 반대로 돈다 (바닥에서는 몸체가 움직이지 않는 조합).
3. 정지 기준으로 `RUN` → 토크가 0 근처에 머무는지 (추정기 정지 드리프트 확인, 6절 4항).
4. 상태 토픽을 끊어 30 ms 안에 `STOP` 과 토크 0이 되는지, `~/stop` 으로 정지 래치가 되는지.

### 8.6 실차 시험 순서

저속 직선 → 등속 선회(1 m/s, R 2.4 m) → 8자 한 바퀴 → 8자 두 바퀴 → 적재 10/20 kg. 매번 rosbag2 기록. $\hat d_f$ 의 시계열은 "모델이 어디서 얼마나 틀렸는가"의 기록이며 2판에서 되살릴 항을 정하는 근거이다 (속도와 무관하게 일정 → 구름 저항, 속도에 비례 → 점성 마찰, 가속에 비례 → 하중 이동, 선회에서만 큼 → $I_z$ 또는 $J_w$).

---

## 9. 실차 전 측정 항목 (발표 18쪽 + 설계 문서 10절)

| 항목 | 방법 | 쓰이는 곳 |
|---|---|---|
| 총질량 $m$ | 저울 (적재 조건마다) | $M$, $\bar\tau$ |
| 요 관성 $I_z$ | CAD, 또는 제자리 회전 가속 시험: $U=\tau[-1,1,-1,1]$ → $M_z=4\ell\tau/r$, 측정한 $\ddot\theta$ 로 $I_{eq}=M_z/\ddot\theta$, $I_z=I_{eq}-4J_w\ell^2/r^2$ | $I_{eq}$ |
| 바퀴 관성 $J_w$ | 데이터시트 회전자 관성 × 감속비² + 바퀴 관성, 또는 바퀴를 띄운 가속 시험 | $m_w$, $I_{eq}$ |
| 노면 마찰계수 $\hat\mu$ | 토크 램프 시험 (기존 설계 절차). 식-21의 마찰 항을 거꾸로 쓰면 $\hat\mu=\sqrt2\,\tau_{slip}/(r\,mg/4)$ | $\bar\tau$ |
| 마찰 보상 최댓값 $\tau_{fric,max}$ | 바퀴를 띄운 등속 시험 | $\bar\tau$ 의 모터 항 |
| 팔 길이 합 $\ell$ | 실측 $l_x+l_y$ (2.2절 불일치 해소) | $(T^{-1})^\top$, $I_{eq}$ |

---

## 10. 작업 순서와 완료 기준

| 단계 | 내용 | 완료 기준 |
|---|---|---|
| M0 | 기존 워크스페이스 조사: `wheel_mpc`(`ReferenceHorizon`), `state_estimator`, `torque_output`, `ak40_can_node`, `command_gate` 의 토픽 이름·메시지 형식·바퀴 순서·부호 | 조사 결과를 표로 보고하고, 5.3절과 다른 점을 사용자에게 확인받음 |
| M1 | `params`·`model`·`prediction`·`reference`·`target_input`·`disturbance`·`qp`·`controller` | 8.1절 U1–U13 통과 |
| M2 | `sim/plant.py`, `run_pass_tests.py` | 8.3절 시험 1–4 통과, 그래프 저장 |
| M3 | `node.py`, `sim_plant_node.py`, launch, yaml | 8.4절 통과, 계산 시간 보고 |
| M4 | 바퀴를 띄운 점검 | 8.5절 1–4 통과 (사용자 입회) |
| M5 | 실차 시험 | 8.6절 순서대로, 매 시험 rosbag 보고 |

M4·M5는 사용자가 로봇 옆에 있을 때만 진행한다.

---

## 11. 대응표

| 이 문서 | 발표 최종본 | 설계 문서 |
|---|---|---|
| 3.2 토크 → 몸체 힘 | 5쪽, 식-2 | 식-2 |
| 3.3 등가 질량·운동방정식 | 6–8쪽, 식-3·5 | 식-3–5 |
| 3.4 제어 모델 | 9쪽, 식-6 | 식-6 |
| 3.5 연속 상태공간 | 10쪽, 식-7 | 식-7 |
| 3.6 ZOH | 11쪽, 식-8 | 식-8 |
| 3.7 외란 추정치 | 11쪽, 식-9 | 식-9–11 |
| 3.8 예측 | 12–13쪽, 식-12·13 | 식-12·13 |
| 3.9 기준·언랩 | 14쪽 | 식-14·15 |
| 3.10 목표 입력 | 14쪽, 식-14 | 식-14 |
| 3.11 비용 | 15쪽, 식-16 | 식-16 |
| 3.12 QP | 16·20쪽, 식-17·18 | 식-17·18 |
| 3.13 제약 | 17쪽, 식-19–21 | 식-19–21 |
| 3.14 출력 | — | 식-22 |

교재: Wang (2009) 1–2장, Rawlings–Mayne–Diehl (2017) 1.5절, Pannocchia & Rawlings (2003), Muske & Badgwell (2002), Franklin–Powell–Workman 4장, Meriam & Kraige 5.7절, Lynch & Park 13.2절, Nocedal & Wright 16장.
