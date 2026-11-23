// 토크 입력 MPC (구조도 v3, 『메카넘 토크 MPC 1판 설계』 식-1~22).  amr_mpc/torque/model.py 와 같은 식이다 —
// 단 2026-11-16 부터 제어기는 m_w 를 늘 M 에 넣고(Python 플랜트는 use_wheel_inertia 일 때만) w 항은 use_w 로 따로 켠다.
// ---------------------------------------------------------------
// ROS 에 의존하지 않는다 (노드는 torque_mpc_node.cpp, 시험은 test/test_torque_mpc.cpp).
//
// 상태 (관성 좌표, map):  x = [x, y, θ, ẋ, ẏ, θ̇]          입력: τ = [FL, FR, RL, RR] 바퀴 순토크 [N·m]
//   Ẍ = G(θ) τ + s_w·w,   G(θ) = M⁻¹ R(θ) (T⁻¹)ᵀ,   w = M⁻¹ [μ ω_z S Ẋ_xy; 0]                (식-5·6)
//   M = diag(m+μ, m+μ, I_z+μk²),  μ = m_w = 4J_w/r²  — 바퀴 등가 질량은 늘 넣는다 (식-3·5, 지시서 2026-11-16).
//   s_w = use_w: w 항만 켜고 끈다 (기본 끔, 교수님 지시 "구현하되 끈 채로"). 꺼지면 w 를 더하지 않는다 (0 을 곱하지 않음).
//   2026-11-16 전에는 use_wheel_inertia 하나가 μ(=M) 와 w 를 같이 켜고 껐다 (끄면 M = diag(m, m, I_z)).
// 이산화 (ZOH, 정확):  x(k+1) = A x(k) + B(k) τ(k) + E w(k) [+ d̂]                               (식-8)
//   A = [I, Ts·I; 0, I],  B(k) = [Ts²/2·G(θ_r(k)); Ts·G(θ_r(k))],  E = [Ts²/2·I; Ts·I]
// 결정변수는 절대 토크 T = [τ_0 … τ_{Nc−1}] (4Nc).  Nc 이후는 τ_{Nc−1} 유지.
// 비용 (식-16):  Σ_{j=1}^{Np} ‖x_j − x_r,j‖²_W + Σ_{j<Nc} w_Δ‖τ_j − τ_{j−1}‖² + r_U Σ_{j<Nc} ‖τ_j − U_s,j‖²
//                [+ q_null Σ_{j<Nc} (nᵀτ_j)² — 2026-11-16 전 구현의 영공간 벌점. 지시서 기본 0]
//   목표 입력 (식-14):  U_s,j = ((T⁻¹)ᵀ)⁺ R(θ_r,j)ᵀ M (p̈_r,j − w_j − â_d),   â_d = d̂_ṗ / Ts (외란 추정치를 쓸 때만)
//     기준 위에서 모델이 맞으면 추종에 필요한 토크(최소 노름 → nᵀU_s = 0). 입력 편차 항이 과도에서 토크를 이쪽으로 당기고
//     영공간 성분을 0 으로 보낸다. 정상상태에서 U = U_s 라 비용 0 (선회 구심 토크 유지).
//   기준 가속도 p̈_r: step() 의 acc_ref 로 받는다 (노드: 기준 속도의 중앙 차분 Δt = 0.2 s, 지시서 3.9).
//     없으면 지평 점에서 p̈ = R(θ_r)(u̇_r + ω_r S u_r), u̇_r 은 이웃 점 차분 (등속 선회에서 정확).
//   구현 지시서 amr/자료/torque_mpc_v1_implementation.md 3.10~3.12 (2026-11-16).
// 제약 (식-19~21):  |τ_i,j| ≤ τ̄_i,   |τ_i,j − τ_i,j−1| ≤ Δτ̄_i
//   DAQP 에는  변수 bound(±τ̄, j=0 은 변화율과 교집합)  +  일반 제약 (j ≥ 1 의 차분 행) 으로 넘긴다.
//   j=0 에서 상한이 변화율보다 우선한다 (torque_mpc_todo.md: 상한이 한 스텝에 Δτ̄ 보다 크게 내려가도 해가 있게):
//     lo = clamp(τ_prev − Δτ̄, −τ̄, τ̄),  hi = clamp(τ_prev + Δτ̄, −τ̄, τ̄)
// 외란 추정치 (식-9~11):  d̂(k) = x̂(k) − A x̂(k−1) − B(θ̂(k−1)) τ(k−1) − s_w·E ŵ(k−1)
//   ŵ(k−1) 은 추정 속도 x̂(k−1) 로 쓰는 시점에 계산한다 (s_w 도 그때 — 지시서 6절의 "실제 값 저장, 쓸 때 s_w" 와 같다).
//   항상 계산해 d_hat() 으로 내주고(로그), dist_enable 일 때만 예측에 더한다 (기본 끔, 지시서 2026-11-16).
//   dist_accel_only (2026-11-18, 시험용, 기본 끔): 위치 성분 [x, y, θ] 잔차를 버리고 0.5·Ts·(속도 잔차) 로 바꾼다
//   — 일정한 가속도 a 의 ZOH 한 스텝 Δp = ½Ts²a, Δv = Ts a 와 같은 꼴 (input disturbance model). 추정기 위치·속도가
//   서로 안 맞아 생기는 위치 잔차가 지평에서 j 배로 쌓이지 않게. 건너뛰기 판단은 원래 위치 잔차로 한다.
//   dist_body_frame (2026-11-21): 잔차의 x·y 성분 (위치·속도) 을 R(θ̂)ᵀ 로 몸체 좌표로 돌린 뒤 거르고, 쓸 때 지평 스텝마다
//   R(θ_r,j) 로 다시 돌린다 (d̂ 는 몸체 좌표로 저장 — d_hat() 도 몸체 좌표). 원을 돌 때 경로 기준으로 일정한 외란이 월드 좌표에서는
//   ω 로 도는 벡터라, 월드 좌표에서 느리게 거르면 정상상태 몫까지 지워진다 (실차 8자 2026-11-21: 시정수 1 s 를 더하면
//   곡률 중심 쪽 â_d 랩 평균 +0.16 → +0.06 m/s², 몸체 좌표로 거르면 +0.13 이고 흔들림 0.32 → 0.08 — analysis/torque_mpc_real/2026-11-21_square/ 14 절).
//   dist_freeze_on_sat: 직전 주기에 낸 τ 가 상한 (0.98 τ̄) 에 붙었거나 목표 입력 U_s 가 상한을 넘었으면 갱신하지 않는다
//   (anti-windup, conditional integration — 기준이 상한 밖이라 못 따라가는 몫을 외란으로 배우지 않게).
//   dist_acc_max / dist_yaw_acc_max: 거른 뒤 |â_d,xy| = |d̂_ṗ,xy|/Ts, |α_d| = |d̂_θ̇|/Ts 의 상한 [m/s², rad/s²] (≤ 0 이면 없음).
// 외란 관측기 이득 (2026-11-21, offset-free MPC — Rawlings·Mayne·Diehl 2017 1장, Pannocchia & Rawlings 2003):
//   위 식-11 의 d̂ ← (1−α)d̂ + α d 는 증강 모델 [x; d] 의 Luenberger 관측기 (L_d = α I) 와 같다 (d − d̂ = y − x̂⁻).
//   dist_kalman 이면 L 을 잡음 모델로 정한다: 축 a ∈ {x, y, θ} 마다 가속도 외란 a_k 를 random walk (변화율 σ_q /√s) 로 두고
//   잔차 [r_p; r_v] = [½Ts²; Ts] a_k + v,  v ~ N(0, diag(σ_p², σ_v²)) 를 측정으로 — 정상상태 Kalman 이득 K_a (1×2) 를 Riccati 반복으로 구한다.
//   위치 잔차도 그 잡음 크기만큼의 무게로 쓴다 (accel_only 는 무시). 저장은 â 이고 d̂ = [½Ts² â; Ts â].
// 구심 feedforward 계수 ρ (2026-11-21): 곡률 중심 쪽 구심 몫 c_n = n̂_c·M(p̈_r − w)  (n̂_c = 기준 속도의 법선, ω_r 부호로 곡률 중심 쪽)
//   중 η = 1 − ρ 만큼을 모델 밖 안쪽 힘으로 본다 → 가속도 외란 a_ρ = η c_n n̂_c / m_eq.  U_s 에서 빼고 (rhs −= M a_ρ) 예측 x_{i+1} 에
//   [½Ts² a_ρ; Ts a_ρ] 로 더한다 — 외란이 예측 모델에도 있어야 offset-free (Rawlings·Mayne·Diehl 2017 1장 "Tracking, Disturbances, and Zero Offset").
//   ρ = ff_normal_scale 고정이거나, ff_normal_estimate 면 d̂ 와 한 Kalman 필터로 같이 추정 (structured disturbance model):
//     상태 z = [d_x, d_y, η] (몸체 좌표, random walk 분산 [kalman_q_x², kalman_q_y², ff_normal_q²]·Ts / 스텝),
//     측정 = 몸체 좌표 잔차 [r_p; r_v] 의 x·y = [½Ts²; Ts] ⊗ (d + η g) + v,   g = (c_n/m_eq) R(θ̂)ᵀ n̂_c  (직전 주기 기준의 값).
//   Pannocchia & Rawlings 2003 의 x⁺ = Ax + Bu + B_d d 에서 B_d 열 하나가 기준에 따라 바뀌는 꼴이다 (시변 — 논문은 LTI 만 다룬다).
//   8자에서 n̂_c 가 왼원·오른원에 몸체 ±y 로 뒤집히므로 몸체 일정 몫 d 와 구심 비례 몫 η 가 갈린다 (한 원만 돌면 d_y + η g 합만 정해지고
//   공분산이 그 방향으로 남는다). 잔차는 보상 여부와 무관한 "공칭 모델과 실제의 차" 라 d̂ 와 η̂ 가 서로의 학습을 멈추지 않는다.
//   (먼저 만든 RLS 판은 오차로 feedback 힘을 써서 — feedback-error learning 꼴, Kawato 1987 — d̂ 가 오프셋을 지우면 학습이 멈추고
//    둘이 같은 오차를 나눠 가졌다: 8자 시험 ρ̂ 0.92 (참값 0.7), 반경 +6.5 mm. stack_changelog 2026-11-21.)
//   직선·저속 (|ω_r| < 0.3, v_r < 0.2, |c_n| < x_min) 은 g = 0 — η 는 안 보이고 분산만 는다 (상한 10 P0). dist_kalman·dist_body_frame 필요.
//   리셋하면 d 몫은 지우고 η̂ 와 그 분산은 남긴다 (로봇·바닥의 성질).
// 제약 있는 목표 입력 (target_projection, 2026-11-21): U_s 를 바퀴별로 [−τ̄, τ̄] 에 자른다 (상자에 대한 사영) — 기준이 상한 밖일 때
//   입력 편차 항이 낼 수 없는 목표 쪽으로 끌지 않게.
//
// 선형화 방식 (TorqueTuning::linearization, torque_mpc.yaml 의 linearization.mode)
//   reference      1판 그대로. 위 B(k), w(k) 에 지평 각 스텝의 기준 방향 θ_r(k)·기준 속도를 넣는다 (기준궤적 따라 LTV).
//   current_state  교수님 지시 "현재 스텝 기준 linearized model" (successive linearization).
//                  매 주기 현재 추정 상태 x̂(k) 와 직전 토크 τ(k−1) 에서 연속 모델 ẋ = f(x, τ) 를 야코비안으로 선형화:
//                    ẋ ≈ A_c x + B_c τ + c_c,   c_c = f(x̂, τ₀) − A_c x̂ − B_c τ₀      (비평형점이라 상수항이 남는다)
//                    A_c = [0, I; ∂(Gτ₀)/∂θ, ∂w/∂Ẋ],   B_c = [0; G(θ̂)]
//                  증강행렬 [[A_c, B_c, c_c], [0]] 의 행렬지수로 ZOH 이산화 (A_d, B_d, c_d) 하고,
//                  그 한 모델(LTI)을 지평 Np 스텝 전체에 쓴다:  x(j+1) = A_d x(j) + B_d τ(j) + c_d [+ d̂].
//                  다음 주기에 새 x̂ 로 다시 선형화한다. 외란 추정치도 이 모델의 한 스텝 예측 잔차로 구한다.
//                  θ̇ = 0, τ₀ = 0, 바퀴 관성 끔이면 A_d, B_d 가 위 식-8 의 A, B(θ̂) 와 같아진다.
//   비용·제약·결정변수는 두 방식이 같다. 달라지는 것은 예측 행렬 Φ 와 영입력 응답뿐이다.
//
// 슬립 계수 η_s (2026-11-23, 교수님 회의 "η 토크 상한" — 선택 기능, 기본 끔. 위 ρ 절의 η = 1 − ρ 와 다른 기호다):
//   바퀴별 상한 τ̄_i 에 η_s,i ∈ [0, 1] 을 곱한다 (지평 모든 스텝에 같은 값 — 앞으로의 슬립은 예측하지 않는다).
//     λ_i  = (ω_i − ω_nom,i)·sgn(ω_nom,i) / max(|ω_i|, |ω_nom,i|, ω_ε),  ω_nom = T⁻¹ v_body (추정기 몸체 속도) — slip_logger 와 같은 식
//     λ̃_i = 인과 1 차 LPF (f_c, y += α(λ − y), α = Δt/(τ + Δt), τ = 1/(2π f_c))
//     η_s,i = 1 / (1 + exp((|λ̃_i| − λ_c)/s))   (logistic, λ_c 에 대칭: η 0.95 at λ_c − ln19·s, 0.05 at λ_c + ln19·s)
//   계산은 SlipEta (아래), 적용은 TorqueMPC::set_slip_scale. 변화율 한계 Δτ̄ 는 곱하기 전 τ̄ 기준 그대로 (하중 이동과 같음),
//   j=0 은 상자가 변화율보다 우선이라 η 가 한 주기에 떨어져도 해가 있다. slip_alloc 이면 목표 입력 U_s 도 η 로 가중 배분
//   (min Σ (τ_i / (c_i η_i))², η 아래 한계 slip_alloc_floor — 두 바퀴가 0 이면 F W⁻¹ Fᵀ 가 특이). 세 바퀴만으로도 rank 3.
//   값 근거: analysis/torque_mpc_real/2026-11-23_slip_lpf/RESULTS_KO.md 5~7 절 (λ_c 0.15, s 0.017, f_c 10 Hz, ω_ε 2.5).
// ---------------------------------------------------------------
#pragma once

#include <Eigen/Dense>

#include <string>
#include <utility>
#include <vector>

namespace amr_mpc_cpp
{

using TVec3 = Eigen::Vector3d;
using TVec4 = Eigen::Vector4d;
using TVec6 = Eigen::Matrix<double, 6, 1>;
using TMat34 = Eigen::Matrix<double, 3, 4>;
using TMat6 = Eigen::Matrix<double, 6, 6>;
using TMat64 = Eigen::Matrix<double, 6, 4>;

enum class TorqueLimitMode {MotorOnly, Friction};   // 비교군 B / C

// 문자열 → 모드. 모르는 값·아직 없는 friction_mtte 는 std::invalid_argument
TorqueLimitMode parse_limit_mode(const std::string & s);

enum class TorqueLinearization {Reference, CurrentState};   // 머리주석 "선형화 방식"

// "reference" | "current_state".  모르는 값은 std::invalid_argument
TorqueLinearization parse_linearization(const std::string & s);

// config/torque_plant.yaml
struct TorqueModel
{
  double wheel_radius = 0.04;
  double wheel_base = 0.1897;
  double robot_width = 0.207;
  double mass = 4.0;                 // 몸체 + 적재 [kg]
  double inertia_z = 0.06;           // [kg·m²]
  double J_w = 9.7e-4;               // [kg·m²]  — m_w = 4J_w/r² 로 M 에 늘 들어간다
  bool use_w = false;                // w 항 (식-6) 스위치 — 예측·목표 입력·외란 추정치 잔차에 같이
  double tau_rated = 1.3;            // τ_lim [N·m]
  double tau_fric_max = 0.0;         // [N·m]
  double mu_hat = 0.3;
  // 바퀴별 수직력 몫 [FL, FR, RL, RR] (합 1, 2026-11-19). N_i = 몫_i·m g. 기본 1/4 씩 = 1판 식-21 의 mg/4.
  //   상한 모드 friction 의 τ_μ,i 에만 쓰인다 (동역학 M 은 무게중심이 가운데라고 그대로 둔다)
  TVec4 wheel_load_share = TVec4::Constant(0.25);
  // 바퀴별 μ̂ [FL, FR, RL, RR] (2026-11-19 (11)). 2026-11-22 부터 N_i 기준 값 — τ_μ,i = (r/√2)·μ̂_i·N_i·c_s, N_i = 몫_i·m g
  //   (실측 |τ_i| / ((r/√2)·N_i) 로 낸 값. 11-21 까지는 mg/4 기준이었다). 0 이하인 바퀴는 μ̂_i 자리에 mu_hat
  TVec4 mu_hat_wheels = TVec4::Constant(-1.0);
  double c_s = 0.8;
  double gravity = 9.81;
  // 구동계 방향 보정 (2026-11-23, torque_plant.yaml drive — 기본 0 = 보정 없음). 실차에서 바퀴가 굴러가는 축이 몸체 축 (SLAM) 보다
  //   drive_yaw 만큼 반시계, 선회 중에는 구심가속 a_c = |v|·ω_z 에 비례해 옆으로 미끄러진다:  v_실제,xy = R(drive_yaw + slip_coef·a_c)·(T ω)_xy.
  //   drive_yaw 는 force_map 의 xy 행 (→ G·B·U_s·야코비안, kinematic–static duality) 과 엔코더 몸체 속도에,
  //   slip_coef 는 엔코더 몸체 속도에만 (미끄럼이라 힘 사상은 안 돌린다). 근거 analysis/torque_mpc_real/2026-11-23_horizon_real 5·6 절.
  double drive_yaw = 0.0;            // [rad]
  double slip_coef = 0.0;            // [rad / (m/s²)]
  double slip_ac_max = 2.0;          // [m/s²] a_c 를 이 안으로 자른다 (잰 범위)

  double k() const {return 0.5 * (wheel_base + robot_width);}
  double mu() const {return 4.0 * J_w / (wheel_radius * wheel_radius);}     // m_w (식-3)
  double m_eff() const {return mass + mu();}
  double I_eff() const {return inertia_z + mu() * k() * k();}
  // (T⁻¹)ᵀ : 바퀴 토크 → 몸체 [F_x, F_y, M_z].  drive_yaw ≠ 0 이면 xy 행을 R(drive_yaw) 로 돌린 값
  TMat34 force_map() const;
  // 엔코더 몸체 속도 보정 (2026-11-23): v_xy ← R(drive_yaw + slip_coef·a_c) v_xy,  a_c = |v_xy|·wz (wz = 자이로 ω_z, |a_c| ≤ slip_ac_max).
  //   ω_z 성분은 그대로. 보정값이 0 이면 v 그대로
  TVec3 correct_wheel_velocity(const TVec3 & v, double wz) const;
  // 바퀴 기구학 v = T ω (몸체 [v_x, v_y, ω_z], 바퀴 [FL, FR, RL, RR] rad/s, 미끄럼 없음 가정).
  //   T = (r/4)[1 1 1 1; −1 1 1 −1; (−1 1 −1 1)/k] — force_map 의 T⁻¹ 의 유사역행렬 (2026-11-18, velocity_source wheels)
  TVec3 wheel_body_velocity(const TVec4 & omega) const
  {
    const double q = 0.25 * wheel_radius;
    return TVec3(
      q * (omega(0) + omega(1) + omega(2) + omega(3)),
      q * (-omega(0) + omega(1) + omega(2) - omega(3)),
      q / k() * (-omega(0) + omega(1) - omega(2) + omega(3)));
  }
  // 무슬립 바퀴 속도 ω_nom = T⁻¹ v (몸체 [v_x, v_y, ω_z] → [FL, FR, RL, RR] rad/s). wheel_body_velocity 의 역 (2026-11-23 슬립 계수 η_s).
  //   ω_i = (v_x ∓ v_y ∓ k ω_z)/r — amr_mpc/torque/model.py TorquePlant.Tinv 와 같다. drive_yaw 는 안 쓴다 (slip_logger λ 정의 그대로)
  TVec4 wheel_nominal_speeds(const TVec3 & v) const
  {
    const double kk = k();
    return TVec4(v(0) - v(1) - kk * v(2), v(0) + v(1) + kk * v(2), v(0) + v(1) - kk * v(2), v(0) - v(1) + kk * v(2)) / wheel_radius;
  }
  // G(θ) = M⁻¹ R(θ) (T⁻¹)ᵀ
  TMat34 G(double theta) const;
  // w = M⁻¹ [μ ω_z S Ẋ_xy; 0]   (Xdot = [ẋ, ẏ, θ̇]).  use_w 가 꺼지면 0 (계산하지 않는다)
  TVec3 w(const TVec3 & Xdot) const;
  // use_w 와 상관없이 w 의 실제 값 (기록·시험용)
  TVec3 w_value(const TVec3 & Xdot) const;
  double tau_motor_limit() const {return tau_rated - tau_fric_max;}                       // 비교군 B
  double tau_friction_limit() const;                                                      // τ_μ, 식-21 (N = mg/4, 평균 바퀴)
  TVec4 tau_friction_limits() const;                                                      // 바퀴별 τ_μ,i = (r/√2)·μ̂_i·N_i·c_s
  // 2026-11-23 하중 이동: 몸체 가속도 (a_x 앞, a_y 왼쪽, 관성 가속도의 몸체 성분) 에서 바퀴별 수직력 [N] (FL FR RL RR).
  //   N_i = 몫_i·m g − s_x,i·m a_x h / (2 L) − s_y,i·m a_y h / (2 W),  s_x = [+ + − −] (앞), s_y = [+ − + −] (왼쪽), L = wheel_base, W = robot_width.
  //   강체 네 바퀴를 앞뒤·좌우 축마다 강체 보로 본 정역학 근사 (서스펜션·차체 비틀림 없음). 아래로 min_frac·정지 하중에서 자른다
  TVec4 wheel_normal_forces(double ax, double ay, double h, double min_frac) const;
  // 2026-11-22 바닥 토크 사상 P: 무슬립이면 바퀴 i 가 바닥에 전하는 토크 τ_g = P τ  (나머지 J_w ω̇_i 는 바퀴 자신을 돌린다).
  //   ω = Fᵀ v (F = force_map), 몸체 M_b v̇ = F τ_g, 바퀴 J_w ω̇ = τ − τ_g  →  (M_b + J_w F Fᵀ) v̇ = F τ (= M_eff, 코드의 M)
  //   →  P = I − J_w Fᵀ M_eff⁻¹ F.  앞으로 고른 토크면 바닥 몫 1 − m_w/m_eff (4.56 kg · J_w 8.9e-4 → 0.672).
  //   근거·수치: analysis/torque_mpc_real/2026-11-21_line_accel/GROUND_CAP_REVIEW_KO.md
  Eigen::Matrix4d ground_map() const;
};

// 선형 모델  ẋ = A x + B τ + c (연속)  또는  x(k+1) = A x(k) + B τ(k) + c (이산)
struct TorqueLinearModel
{
  TMat6 A = TMat6::Zero();
  TMat64 B = TMat64::Zero();
  TVec6 c = TVec6::Zero();
};

// 연속 비선형 모델 ẋ = f(x, τ) = [Ẋ; G(θ)τ + w(Ẋ)]
TVec6 torque_dynamics(const TorqueModel & md, const TVec6 & x, const TVec4 & tau);
// (x0, τ0) 에서의 야코비안 선형화. 그 점에서 f 와 값·1차 미분이 같다
TorqueLinearModel linearize_continuous(const TorqueModel & md, const TVec6 & x0, const TVec4 & tau0);
// ZOH 정확 이산화 (τ, c 를 한 샘플 동안 상수로) — 증강행렬의 행렬지수
TorqueLinearModel discretize_zoh(const TorqueLinearModel & cont, double Ts);

// config/torque_mpc.yaml
struct TorqueTuning
{
  double Ts = 0.01;
  int Np = 50;
  int Nc = 5;
  TVec6 W = (TVec6() << 100.0, 100.0, 50.0, 1.0, 1.0, 0.5).finished();
  double w_delta = 0.01;             // r_Δ
  double r_u = 0.01;                 // r_U — 입력 편차 항 (식-16 셋째 항, 지시서 3.11)
  double q_null = 0.0;               // 영공간 벌점 (2026-11-16 전 구현 방식). 지시서는 입력 편차 항이 대신한다 → 0
  TorqueLimitMode limit_mode = TorqueLimitMode::MotorOnly;
  // 구조체 기본값은 1판 방식(reference) — 1판 수치 예 시험이 이 값에 묶여 있다.
  // 노드는 torque_mpc.yaml 의 linearization.mode 를 읽고, 그 기본값도 reference 다 (2026-11-14).
  TorqueLinearization linearization = TorqueLinearization::Reference;
  double rate_fraction = 1.0 / 3.0;
  // 2026-11-22 마찰 상한을 어디에 거나: false = 바퀴 토크 τ 자체 |τ_i| ≤ τ̄_i (1판, 기본)
  //   true = 바닥 토크 |(P τ)_i| ≤ τ̄_i (일반 제약 4·Nc 행) + 바퀴 토크는 모터 한계 |τ_i| ≤ τ_lim − τ_fric,max. limit_mode friction 일 때만
  bool cap_on_ground = false;
  bool constrained = true;           // false 면 QP 대신 H T = −g (자기검사용, 1판 09절)
  bool dist_enable = false;
  double dist_alpha = 1.0;
  // 외란 추정치 갱신 건너뛰기 (지시서 4절·7.4절). 건너뛰면 d̂_f 를 그대로 두고 x̂_prev 는 갱신한다
  double dist_dt_tol = 0.002;        // [s] 상태 간격이 Ts ± 이 값을 벗어나면 (step 의 dt_state 로 판단, ≤ 0 이면 검사 안 함)
  double dist_skip_xy = 1e-3;        // [m] 잔차 위치 성분 |d̂_x|·|d̂_y| 가 이보다 크면 (SLAM 보정 계단). ≤ 0 이면 검사 안 함
  bool dist_accel_only = false;      // true: 위치 잔차 = ½Ts·속도 잔차 (가속도 외란 모델, 머리주석). false: 1판 식-9 그대로
  // 2026-11-21 (머리주석). 구조체 기본값은 예전 동작 — 노드는 torque_mpc.yaml disturbance 블록을 읽는다
  bool dist_body_frame = false;      // true: 몸체 좌표로 추정·저장, 쓸 때 R(θ_r,j) 로 돌린다. false: 월드 좌표 (1판)
  bool dist_freeze_on_sat = false;   // true: 직전 τ 가 0.98 τ̄ 에 붙었거나 U_s 가 τ̄ 를 넘으면 갱신하지 않는다
  double dist_acc_max = -1.0;        // [m/s²] |â_d,xy| 상한 (≤ 0 이면 없음)
  double dist_yaw_acc_max = -1.0;    // [rad/s²] |α_d| 상한 (≤ 0 이면 없음)
  // 2026-11-21 (정석 offset-free 쪽으로, 머리주석 "외란 관측기 이득"): true 면 α 대신 정상상태 Kalman 이득.
  //   축마다 (x, y, θ) 가속도 외란 a 를 random walk 로 두고 잔차 [r_p; r_v] = [½Ts²; Ts] a + v 를 측정으로 — Riccati 반복으로 K (1×2).
  bool dist_kalman = false;
  TVec3 dist_kq = TVec3(0.3, 0.3, 0.8);          // [m/s²/√s, rad/s²/√s] 외란 변화율 (random walk) 표준편차
  TVec3 dist_kr_pos = TVec3(1.6e-3, 1.6e-3, 8.7e-4);   // [m, rad] 한 스텝 위치 잔차 잡음 (실차 212502)
  TVec3 dist_kr_vel = TVec3(0.017, 0.029, 0.079);      // [m/s, rad/s] 한 스텝 속도 잔차 잡음 (실차 212502)
  // 2026-11-21 제약 있는 목표 입력 (target calculation, Muske & Rawlings 1993): U_s 를 상한 상자 [−τ̄, τ̄] 에 사영 (바퀴별 clamp)
  bool target_projection = false;
  // 2026-11-21 구심 feedforward 계수 ρ — 머리주석 "구심 feedforward 계수".
  //   estimate 가 꺼져 있으면 ff_normal_scale 고정 (1 = 1판), 켜져 있으면 d̂ 와 같은 Kalman 필터로 η = 1 − ρ 추정
  double ff_normal_scale = 1.0;
  bool ff_normal_estimate = false;   // dist_kalman·dist_body_frame 필요
  double ff_normal_q = 0.03;         // [1/√s] η random walk 세기
  double ff_normal_min = 0.4;
  double ff_normal_max = 1.2;
  double ff_normal_P0 = 0.04;        // η 초기 분산 (σ 0.2). 분산 상한은 10 배
  double ff_normal_x_min = 0.3;      // [N] 이보다 작은 구심 feedforward 는 g = 0 (직선·저속)
  int daqp_iter_limit = 1000;
  // 2026-11-23 하중 이동 (머리주석 "하중 이동"). load_com_height ≤ 0 이면 끔 (예전 동작 그대로).
  //   지평 스텝마다 기준 가속도로 바퀴별 수직력 N_i,j (TorqueModel::wheel_normal_forces) 를 내어
  //   (1) load_weighted_alloc: 목표 입력 U_s 를 마찰 사용률 가중 — min Σ (τ_i / c_i)², c_i ∝ μ̂_i N_i,j  (가중 유사역행렬, F U_s = 몸체 힘 그대로)
  //   (2) load_dynamic_cap: limit_mode friction 의 τ̄_i 를 N_i,j / N_i,0 배 (지평 스텝마다 다른 상자). 변화율 한계는 정지 τ̄ 기준 그대로
  double load_com_height = 0.0;      // [m] 무게중심 높이 h (리포에 실측 없음 — URDF 차체 상자 중심 0.105)
  double load_min_frac = 0.2;        // N_i ≥ 이 값 × 정지 N_i (바퀴가 뜨는 것까지는 안 본다)
  bool load_weighted_alloc = true;
  bool load_dynamic_cap = true;
  // 2026-11-23 슬립 계수 η_s (머리주석). 상한에 곱하는 값은 set_slip_scale 로 (기본 1 = 예전 그대로)
  bool slip_alloc = true;            // η_s < 1 인 바퀴가 있으면 U_s 를 η_s 로 가중 배분
  double slip_alloc_floor = 0.05;    // 가중 배분에서 η_s 아래 한계
};

// 슬립 계수 η_s 계산 (머리주석 "슬립 계수 η_s"). ROS 무관 — 노드가 /wheel_states 마다 update 를 부른다
struct SlipEtaParams
{
  double lambda_c = 0.15;            // η_s = 0.5 인 |λ̃|
  double s = 0.017;                  // logistic 폭 (η 0.95 → 0.05 구간 = 2·ln19·s)
  double fc = 10.0;                  // [Hz] 1 차 LPF 차단 주파수
  double omega_eps = 2.5;            // [rad/s] λ 분모 아래 한계 (slip_logger omega_eps 와 같게)
};

class SlipEta
{
public:
  explicit SlipEta(const SlipEtaParams & p);
  // slip_logger.slip_ratio 와 같은 식 (4,)
  static TVec4 slip_ratio(const TVec4 & omega, const TVec4 & omega_nom, double omega_eps);
  static double eta_of(double lambda_f, double lambda_c, double s);
  // omega: 바퀴 각속도, omega_nom: T⁻¹ v_body, valid[i] false 인 바퀴 (응답 없음·에러·NaN) 는 λ̃ 를 그대로 둔다. dt [s] (≤ 0 이면 0.01)
  const TVec4 & update(const TVec4 & omega, const TVec4 & omega_nom, const bool valid[4], double dt);
  void reset();
  const TVec4 & lambda() const {return lam_;}      // 이번 원신호 λ (valid 아닌 바퀴는 직전 값)
  const TVec4 & lambda_f() const {return lam_f_;}  // LPF 뒤 λ̃
  const TVec4 & eta() const {return eta_;}
  const SlipEtaParams & params() const {return p_;}

private:
  SlipEtaParams p_;
  double tau_ = 0.0;
  TVec4 lam_ = TVec4::Zero(), lam_f_ = TVec4::Zero(), eta_ = TVec4::Ones();
};

struct TorqueStepStats
{
  int iters = 0;
  int active = 0;          // 활성 제약 수
  int fallback = 0;        // QP 실패 → 무제약 해를 상한으로 깎았다
  int exitflag = 0;
  int d_skipped = 0;       // 이번 주기 외란 추정치 갱신을 건너뜀 (1 = 상태 간격, 2 = 잔차 계단, 3 = 포화 중 멈춤)
  int rho_updated = 0;     // 이번 주기 η 를 본 갱신 (g ≠ 0) 1 / 아님 0
  double rho_g = 0.0;      // |g| = c_n / m_eq [m/s²] (직전 주기 기준)
  double rho_r = 0.0;      // 몸체 속도 잔차 / Ts 의 ĝ 방향 성분 [m/s²] — η g + d·ĝ 의 측정
};

// 지평의 한 점: (포즈 X_r = [x, y, θ],  몸체 속도 u_r = [v_x, v_y, ω_z]).  ReferenceHorizon::horizon 의 원소와 같다
using TorqueRefPoint = std::pair<TVec3, TVec3>;

class TorqueMPC
{
public:
  TorqueMPC(const TorqueModel & model, const TorqueTuning & tuning);

  // 명령 이력이 끊길 때 (정지, hold 진입, 재시작). x̂_prev, τ_prev, d̂ 를 버린다 (1판 09절)
  void reset();

  // x_hat: 현재 상태 (관성 좌표).  ref: 지평 Np+1 점 — ref[j] 의 시각 = 지금 + j·Ts  (j = 0 … Np)
  // 반환: 이번 주기 순토크 τ*(k).  θ 는 접힌 값이어도 된다 (안에서 식-15 로 언랩)
  // acc_ref: (선택) 기준 가속도 p̈_r,j (관성 좌표), j = 0 … Nc−1. 없거나 짧으면 지평 점에서 계산 (머리주석)
  // dt_state: (선택) 이번 상태와 지난 상태의 시각 차 [s] — 외란 추정치 갱신 건너뛰기 판단용. ≤ 0 이면 검사 안 함
  TVec4 step(const TVec6 & x_hat, const std::vector<TorqueRefPoint> & ref,
    const std::vector<TVec3> * acc_ref = nullptr, double dt_state = -1.0);

  // 상한을 밖에서 줄 때 (MTTE 등 바퀴별 온라인 상한용 자리). 음수면 모드에서 계산한 값
  void set_tau_bar_override(const TVec4 & tau_bar) {tau_bar_override_ = tau_bar;}
  // 2026-11-23 슬립 계수 η_s (머리주석): 다음 step 의 상자 τ̄_i,j 에 곱한다. [0, 1] 로 자른다. 1 이면 예전 그대로
  void set_slip_scale(const TVec4 & eta) {slip_scale_ = eta.cwiseMax(0.0).cwiseMin(1.0);}
  const TVec4 & slip_scale() const {return slip_scale_;}

  TVec4 tau_bar() const;                          // 이번에 쓰는 토크 상한 (바퀴별)
  double dtau_bar_of(double tau_bar) const {return tu_.rate_fraction * tau_bar;}
  bool cap_on_ground() const {return ground_;}     // 2026-11-22 마찰 상한이 바닥 토크에 걸렸나 (TorqueTuning::cap_on_ground)
  const TVec6 & d_hat() const {return d_hat_;}    // 필터 후 외란 추정치 (dist_body_frame 이면 몸체 좌표)
  TVec6 d_hat_world(double theta) const;          // 방향 θ 에서의 월드 좌표 값 (월드 저장이면 그대로)
  const Eigen::Matrix<double, 3, 2> & dist_kalman_gain() const {return K_kal_;}   // 축별 [K_pos, K_vel]
  double ff_normal_rho() const {return tu_.ff_normal_estimate ? rho_ : tu_.ff_normal_scale;}   // 지금 쓰는 ρ
  double ff_normal_P() const {return Pz_(2, 2);}                // η̂ 분산
  // 스칼라 random walk a 를 측정 [½Ts²; Ts] a + v (잡음 σ_p, σ_v) 로 볼 때 정상상태 Kalman 이득 (Riccati 반복). 시험·기록용 공개
  static Eigen::RowVector2d steady_kalman_gain(double Ts, double q_rate, double r_pos, double r_vel);
  const TVec4 & tau_prev() const {return tau_prev_;}
  const TorqueStepStats & stats() const {return stats_;}
  const TorqueModel & model() const {return md_;}
  const TorqueTuning & tuning() const {return tu_;}
  const Eigen::VectorXd & solution() const {return T_;}        // T* 전체 (4Nc)
  const Eigen::VectorXd & target_input() const {return Us_;}   // 이번 주기 목표 입력 U_s (4Nc, 식-14)
  static TVec4 null_dir() {return TVec4(0.5, 0.5, -0.5, -0.5);}
  // 2026-11-23 하중 이동: 이번 주기 지평 스텝별 상한 τ̄_i,j (4 × Nc, 하중 이동 끄면 열마다 tau_bar()) 와 스텝 0 의 수직력 N_i [N]
  const Eigen::Matrix<double, 4, Eigen::Dynamic> & tau_bar_steps() const {return tb_steps_;}
  const TVec4 & normal_forces_now() const {return n_now_;}

private:
  void update_disturbance(const TVec6 & x_hat, double dt_state);
  bool solve_qp(const TVec4 & tau_bar);   // 상자 = tb_steps_ (스텝별), 변화율 = 이 tau_bar (정지 상한) 기준

  TorqueModel md_;
  TorqueTuning tu_;
  int nU_ = 0;

  TVec6 x_prev_ = TVec6::Zero();
  TVec4 tau_prev_ = TVec4::Zero();
  bool has_prev_ = false;
  TorqueLinearModel lin_prev_;       // current_state: 직전 주기에 쓴 이산 모델 (외란 추정치용)
  bool has_lin_ = false;
  TVec6 d_hat_ = TVec6::Zero();
  Eigen::Matrix<double, 3, 2> K_kal_ = Eigen::Matrix<double, 3, 2>::Zero();   // dist_kalman: 축별 [K_pos, K_vel]
  TVec3 a_hat_ = TVec3::Zero();      // dist_kalman: 가속도 외란 추정 (저장 좌표). d_hat_ = [½Ts² â; Ts â]
  // ff_normal_estimate: Kalman 상태 z = [d_x, d_y, η] (몸체) 와 공분산. η 몫은 리셋해도 남긴다
  double rho_ = 1.0;
  Eigen::Vector3d z_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Pz_ = Eigen::Matrix3d::Zero();
  bool rho_have_ = false;            // 직전 주기의 c_n, n̂_c 가 측정 회귀에 쓸 만한가
  double rho_cn_prev_ = 0.0;
  Eigen::Vector2d rho_n_prev_ = Eigen::Vector2d::Zero();
  void joint_kalman_update(const TVec6 & d_body, double theta);
  bool us_clipped_ = false;          // 직전 주기 U_s 를 target_projection 이 잘랐다 (= 기준이 상한 밖) — 포화 중 멈춤에 쓴다
  TVec4 tau_bar_override_ = TVec4::Constant(-1.0);
  TVec4 slip_scale_ = TVec4::Ones();                    // 2026-11-23 η_s (다음 step 에 쓸 값)
  TVec4 slip_scale_used_ = TVec4::Ones();               // 직전 step 이 쓴 η_s (포화 중 멈춤 판단은 tau_prev_ 와 같은 상자로)
  Eigen::Matrix<double, 4, Eigen::Dynamic> tb_steps_;   // 2026-11-23 지평 스텝별 상한 (하중 이동)
  TVec4 n_now_ = TVec4::Zero();                         // 스텝 0 의 바퀴별 수직력
  TMat34 F_ = TMat34::Zero();                           // force_map (가중 배분용)
  bool ground_ = false;                              // cap_on_ground 이 실제로 켜졌다 (limit_mode friction)
  Eigen::Matrix4d P_ground_ = Eigen::Matrix4d::Identity();
  TorqueStepStats stats_;

  // 상수부
  Eigen::MatrixXd H_const_;                          // Dᵀ W̄_Δ D + R̄_U + q_null N
  Eigen::Matrix<double, 4, 3> F_pinv_;               // ((T⁻¹)ᵀ)⁺ = Fᵀ (F Fᵀ)⁻¹ (식-14)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_gen_;   // j ≥ 1 의 차분 행 (DAQP 는 row-major)

  // 작업 버퍼
  Eigen::MatrixXd Phi_, WPhi_, H_;
  Eigen::VectorXd e0_, g_, T_, lam_, lb_, ub_, Us_;
};

}  // namespace amr_mpc_cpp
