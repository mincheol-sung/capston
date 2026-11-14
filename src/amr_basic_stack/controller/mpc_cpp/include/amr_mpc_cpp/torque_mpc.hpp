// 토크 입력 MPC (구조도 v3, 『메카넘 토크 MPC 1판 설계』 식-1~22).  amr_mpc/torque/model.py 와 같은 모델이다.
// ---------------------------------------------------------------
// ROS 에 의존하지 않는다 (노드는 torque_mpc_node.cpp, 시험은 test/test_torque_mpc.cpp).
//
// 상태 (관성 좌표, map):  x = [x, y, θ, ẋ, ẏ, θ̇]          입력: τ = [FL, FR, RL, RR] 바퀴 순토크 [N·m]
//   Ẍ = G(θ) τ + w,   G(θ) = M⁻¹ R(θ) (T⁻¹)ᵀ,   w = M⁻¹ [μ ω_z S Ẋ_xy; 0]                    (식-6)
//   M = diag(m+μ, m+μ, I_z+μk²),  μ = 4J_w/r²   (use_wheel_inertia 가 꺼지면 μ = 0 → w = 0)
// 이산화 (ZOH, 정확):  x(k+1) = A x(k) + B(k) τ(k) + E w(k) [+ d̂]                               (식-8)
//   A = [I, Ts·I; 0, I],  B(k) = [Ts²/2·G(θ_r(k)); Ts·G(θ_r(k))],  E = [Ts²/2·I; Ts·I]
// 결정변수는 절대 토크 T = [τ_0 … τ_{Nc−1}] (4Nc).  Nc 이후는 τ_{Nc−1} 유지.
// 비용 (식-16):  Σ_{j=1}^{Np} ‖x_j − x_r,j‖²_W + Σ_{j<Nc} w_Δ‖τ_j − τ_{j−1}‖² + q_null Σ_{j<Nc} (nᵀτ_j)²
//   토크 자체에는 벌점이 없다 — 선회 중 구심 토크가 유지되어도 비용 0.
// 제약 (식-19~21):  |τ_i,j| ≤ τ̄_i,   |τ_i,j − τ_i,j−1| ≤ Δτ̄_i
//   DAQP 에는  변수 bound(±τ̄, j=0 은 변화율과 교집합)  +  일반 제약 (j ≥ 1 의 차분 행) 으로 넘긴다.
//   j=0 에서 상한이 변화율보다 우선한다 (torque_mpc_todo.md: 상한이 한 스텝에 Δτ̄ 보다 크게 내려가도 해가 있게):
//     lo = clamp(τ_prev − Δτ̄, −τ̄, τ̄),  hi = clamp(τ_prev + Δτ̄, −τ̄, τ̄)
// 외란 추정치 (식-9~11):  d̂(k) = x̂(k) − A x̂(k−1) − B(θ̂(k−1)) τ(k−1) − E ŵ(k−1)
//   항상 계산해 d_hat() 으로 내주고(로그), dist_enable 일 때만 예측에 더한다.
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
  double J_w = 9.7e-4;               // [kg·m²]
  bool use_wheel_inertia = false;
  double tau_rated = 1.3;            // τ_lim [N·m]
  double tau_fric_max = 0.0;         // [N·m]
  double mu_hat = 0.3;
  double c_s = 0.8;
  double gravity = 9.81;

  double k() const {return 0.5 * (wheel_base + robot_width);}
  double mu() const {return use_wheel_inertia ? 4.0 * J_w / (wheel_radius * wheel_radius) : 0.0;}
  double m_eff() const {return mass + mu();}
  double I_eff() const {return inertia_z + mu() * k() * k();}
  // (T⁻¹)ᵀ : 바퀴 토크 → 몸체 [F_x, F_y, M_z]
  TMat34 force_map() const;
  // G(θ) = M⁻¹ R(θ) (T⁻¹)ᵀ
  TMat34 G(double theta) const;
  // w = M⁻¹ [μ ω_z S Ẋ_xy; 0]   (Xdot = [ẋ, ẏ, θ̇])
  TVec3 w(const TVec3 & Xdot) const;
  double tau_motor_limit() const {return tau_rated - tau_fric_max;}                       // 비교군 B
  double tau_friction_limit() const;                                                      // τ_μ, 식-21
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
  double w_delta = 0.01;
  double q_null = 0.01;
  TorqueLimitMode limit_mode = TorqueLimitMode::MotorOnly;
  // 구조체 기본값은 1판 방식(reference) — 1판 수치 예 시험이 이 값에 묶여 있다.
  // 노드는 torque_mpc.yaml 의 linearization.mode 를 읽고, 그 기본값도 reference 다 (2026-11-14).
  TorqueLinearization linearization = TorqueLinearization::Reference;
  double rate_fraction = 1.0 / 3.0;
  bool constrained = true;           // false 면 QP 대신 H T = −g (자기검사용, 1판 09절)
  bool dist_enable = false;
  double dist_alpha = 1.0;
  int daqp_iter_limit = 1000;
};

struct TorqueStepStats
{
  int iters = 0;
  int active = 0;          // 활성 제약 수
  int fallback = 0;        // QP 실패 → 무제약 해를 상한으로 깎았다
  int exitflag = 0;
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
  TVec4 step(const TVec6 & x_hat, const std::vector<TorqueRefPoint> & ref);

  // 상한을 밖에서 줄 때 (MTTE 등 바퀴별 온라인 상한용 자리). 음수면 모드에서 계산한 값
  void set_tau_bar_override(const TVec4 & tau_bar) {tau_bar_override_ = tau_bar;}

  TVec4 tau_bar() const;                          // 이번에 쓰는 토크 상한 (바퀴별)
  double dtau_bar_of(double tau_bar) const {return tu_.rate_fraction * tau_bar;}
  const TVec6 & d_hat() const {return d_hat_;}    // 필터 후 외란 추정치
  const TVec4 & tau_prev() const {return tau_prev_;}
  const TorqueStepStats & stats() const {return stats_;}
  const TorqueModel & model() const {return md_;}
  const TorqueTuning & tuning() const {return tu_;}
  const Eigen::VectorXd & solution() const {return T_;}        // T* 전체 (4Nc)
  static TVec4 null_dir() {return TVec4(0.5, 0.5, -0.5, -0.5);}

private:
  void update_disturbance(const TVec6 & x_hat);
  bool solve_qp(const TVec4 & tau_bar);

  TorqueModel md_;
  TorqueTuning tu_;
  int nU_ = 0;

  TVec6 x_prev_ = TVec6::Zero();
  TVec4 tau_prev_ = TVec4::Zero();
  bool has_prev_ = false;
  TorqueLinearModel lin_prev_;       // current_state: 직전 주기에 쓴 이산 모델 (외란 추정치용)
  bool has_lin_ = false;
  TVec6 d_hat_ = TVec6::Zero();
  TVec4 tau_bar_override_ = TVec4::Constant(-1.0);
  TorqueStepStats stats_;

  // 상수부
  Eigen::MatrixXd H_const_;                          // Dᵀ W̄_Δ D + q_null N
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_gen_;   // j ≥ 1 의 차분 행 (DAQP 는 row-major)

  // 작업 버퍼
  Eigen::MatrixXd Phi_, WPhi_, H_;
  Eigen::VectorXd e0_, g_, T_, lam_, lb_, ub_;
};

}  // namespace amr_mpc_cpp
