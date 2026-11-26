// 토크 입력 MPC — 식의 근거는 torque_mpc.hpp 머리주석과 『메카넘 토크 MPC 1판 설계』.
#include "amr_mpc_cpp/torque_mpc.hpp"

extern "C" {
#include "api.h"
}

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace amr_mpc_cpp
{

namespace
{
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
constexpr double kJointPd0 = 0.09;   // 결합 Kalman 의 d_x·d_y 초기 분산 [(m/s²)²] (σ 0.3 — 실차 8자 정상상태 몫 0.09~0.26 을 덮는다)
}  // namespace

TorqueLimitMode parse_limit_mode(const std::string & s)
{
  if (s == "motor_only") {return TorqueLimitMode::MotorOnly;}
  if (s == "friction") {return TorqueLimitMode::Friction;}
  if (s == "friction_mtte") {
    throw std::invalid_argument("torque_limit.mode=friction_mtte 는 아직 구현되지 않았다 (motor_only | friction)");
  }
  throw std::invalid_argument("torque_limit.mode=" + s + " — motor_only | friction 중 하나여야 한다");
}

// ── 모델 ────────────────────────────────────────────────────────
TMat34 TorqueModel::force_map() const
{
  const double kk = k();
  TMat34 F;
  F << 1.0, 1.0, 1.0, 1.0,        // F_x
    -1.0, 1.0, 1.0, -1.0,         // F_y
    -kk, kk, -kk, kk;             // M_z
  F /= wheel_radius;
  if (drive_yaw != 0.0) {         // 2026-11-23 바퀴 힘 방향도 굴러가는 축과 같이 돈다 (속도 사상 R(δ)T 의 쌍대). M_z 행은 그대로
    const double c = std::cos(drive_yaw), s = std::sin(drive_yaw);
    const Eigen::Matrix<double, 1, 4> fx = F.row(0), fy = F.row(1);
    F.row(0) = c * fx - s * fy;
    F.row(1) = s * fx + c * fy;
  }
  return F;
}

TVec3 TorqueModel::correct_wheel_velocity(const TVec3 & v, double wz) const
{
  const double ac = std::clamp(std::hypot(v(0), v(1)) * wz, -slip_ac_max, slip_ac_max);
  const double a = drive_yaw + slip_coef * ac;
  if (a == 0.0) {return v;}
  const double c = std::cos(a), s = std::sin(a);
  return TVec3(c * v(0) - s * v(1), s * v(0) + c * v(1), v(2));
}

TMat34 TorqueModel::G(double theta) const
{
  const TMat34 F = force_map();
  const double c = std::cos(theta), s = std::sin(theta);
  TMat34 g;
  g.row(0) = (c * F.row(0) - s * F.row(1)) / m_eff();
  g.row(1) = (s * F.row(0) + c * F.row(1)) / m_eff();
  g.row(2) = F.row(2) / I_eff();
  return g;
}

TVec3 TorqueModel::w_value(const TVec3 & Xdot) const
{
  const double m = mu();
  // 식-6:  w = (m_w / m_eq) θ̇ [−ẏ, ẋ, 0]   (S Ẋ_xy = [−ẏ, ẋ])
  return TVec3(-m * Xdot(2) * Xdot(1) / m_eff(), m * Xdot(2) * Xdot(0) / m_eff(), 0.0);
}

TVec3 TorqueModel::roller_force(const TVec3 & v, double F_rr, double s0) const
{
  if (!(F_rr > 0.0)) {return TVec3::Zero();}
  const double lx = 0.5 * wheel_base, r2 = std::sqrt(2.0);
  const double sf = r2 * (v(1) + lx * v(2)), sr = r2 * (v(1) - lx * v(2));    // 앞·뒤 롤러 상대 속도 (머리주석 "롤러 구름 저항")
  const double gf = sf / std::sqrt(sf * sf + s0 * s0), gr = sr / std::sqrt(sr * sr + s0 * s0);   // ∂|s|/∂s
  // Q = −F_rr Σ ∂|s_i|/∂v,  ∂s_f/∂v = √2 [0, 1, l_x], ∂s_r/∂v = √2 [0, 1, −l_x], 앞·뒤 바퀴가 두 개씩
  return TVec3(0.0, -2.0 * r2 * F_rr * (gf + gr), -2.0 * r2 * F_rr * lx * (gf - gr));
}

TVec3 TorqueModel::w(const TVec3 & Xdot) const
{
  if (!use_w) {return TVec3::Zero();}             // s_w = 0: 더하지 않는다 (지시서 0.3)
  return w_value(Xdot);
}

double TorqueModel::tau_friction_limit() const
{
  return wheel_radius / std::sqrt(2.0) * mu_hat * mass * gravity / 4.0 * c_s;
}

Eigen::Matrix4d TorqueModel::ground_map() const
{
  const TMat34 F = force_map();
  const Eigen::Vector3d Minv(1.0 / m_eff(), 1.0 / m_eff(), 1.0 / I_eff());
  return Eigen::Matrix4d::Identity() - J_w * F.transpose() * Minv.asDiagonal() * F;
}

TVec4 TorqueModel::tau_friction_limits() const
{
  // 식-21 을 바퀴마다: τ_μ,i = (r/√2)·μ̂_i·N_i·c_s,  N_i = 몫_i·m g  (몫 1/4 이면 mg/4 — 위와 같다) (2026-11-19 (10))
  // μ̂_i = mu_hat_wheels(i) (> 0 일 때), 아니면 mu_hat. 2026-11-22: 바퀴별 μ̂ 도 N_i 기준 — 하중 몫을 곱한다
  //   (11-19 (11) ~ 11-21 은 mg/4 기준이라 하중 몫을 안 곱했다)
  const double unit = wheel_radius / std::sqrt(2.0) * mass * gravity * c_s;
  TVec4 out;
  for (int i = 0; i < 4; ++i) {
    out(i) = unit * (mu_hat_wheels(i) > 0.0 ? mu_hat_wheels(i) : mu_hat) * wheel_load_share(i);
  }
  return out;
}

TVec4 TorqueModel::drivetrain_friction(const TVec4 & omega, double smooth) const
{
  // 2026-11-24 (hpp 머리주석 "마찰 상한 옮김"): torque_output.friction_torque 의 운동 마찰 항과 같은 식
  TVec4 out;
  for (int i = 0; i < 4; ++i) {out(i) = fric_coulomb(i) * std::tanh(omega(i) / smooth) + fric_viscous(i) * omega(i);}
  return out;
}

TVec4 TorqueModel::wheel_normal_forces(double ax, double ay, double h, double min_frac) const
{
  // 머리주석 (hpp) 의 식. h ≤ 0 이면 정지 하중 그대로. 합 Σ N_i = m g 는 가속도와 상관없이 보존된다 (자르기 전)
  const TVec4 n0 = wheel_load_share * mass * gravity;
  if (!(h > 0.0)) {return n0;}
  const TVec4 sx(1.0, 1.0, -1.0, -1.0), sy(1.0, -1.0, 1.0, -1.0);
  const TVec4 n = n0 - sx * (mass * ax * h / (2.0 * wheel_base)) - sy * (mass * ay * h / (2.0 * robot_width));
  return n.cwiseMax(min_frac * n0);
}

// ── 현재 상태 기준 선형화 ───────────────────────────────────────
TorqueLinearization parse_linearization(const std::string & s)
{
  if (s == "reference") {return TorqueLinearization::Reference;}
  if (s == "current_state") {return TorqueLinearization::CurrentState;}
  throw std::invalid_argument("linearization.mode=" + s + " — reference | current_state 중 하나여야 한다");
}

TVec6 torque_dynamics(const TorqueModel & md, const TVec6 & x, const TVec4 & tau)
{
  TVec6 f;
  f.head<3>() = x.tail<3>();
  f.tail<3>() = md.G(x(2)) * tau + md.w(x.tail<3>());
  return f;
}

TorqueLinearModel linearize_continuous(const TorqueModel & md, const TVec6 & x0, const TVec4 & tau0)
{
  TorqueLinearModel lin;
  const double c = std::cos(x0(2)), s = std::sin(x0(2));
  const double xd = x0(3), yd = x0(4), thd = x0(5);
  lin.A.block<3, 3>(0, 3).setIdentity();
  // ∂(G τ₀)/∂θ = M⁻¹ (dR/dθ) (T⁻¹)ᵀ τ₀  — 방향이 틀어지면 같은 토크가 내는 map 가속도가 돌아간다
  const TVec3 F0 = md.force_map() * tau0;
  lin.A(3, 2) = (-s * F0(0) - c * F0(1)) / md.m_eff();
  lin.A(4, 2) = (c * F0(0) - s * F0(1)) / md.m_eff();
  // ∂w/∂Ẋ,  w = a [−θ̇ ẏ, θ̇ ẋ, 0],  a = μ / m_eff   (use_w 가 꺼지면 w 가 없으니 야코비안도 없다)
  if (md.use_w) {
    const double a = md.mu() / md.m_eff();
    lin.A(3, 4) = -a * thd;
    lin.A(3, 5) = -a * yd;
    lin.A(4, 3) = a * thd;
    lin.A(4, 5) = a * xd;
  }
  lin.B.bottomRows<3>() = md.G(x0(2));
  lin.c = torque_dynamics(md, x0, tau0) - lin.A * x0 - lin.B * tau0;
  return lin;
}

TorqueLinearModel discretize_zoh(const TorqueLinearModel & cont, double Ts)
{
  // exp([[A, B, c], [0, 0, 0]] Ts) = [[A_d, B_d, c_d], [0, I]].  스케일링·제곱 + 테일러 18항
  using Mat11 = Eigen::Matrix<double, 11, 11>;
  Mat11 M = Mat11::Zero();
  M.block<6, 6>(0, 0) = cont.A;
  M.block<6, 4>(0, 6) = cont.B;
  M.block<6, 1>(0, 10) = cont.c;
  M *= Ts;
  const double norm = M.cwiseAbs().colwise().sum().maxCoeff();
  int squarings = 0;
  if (norm > 0.5) {squarings = std::max(0, static_cast<int>(std::ceil(std::log2(norm / 0.5))));}
  const Mat11 X = M / std::pow(2.0, squarings);
  Mat11 E = Mat11::Identity(), term = Mat11::Identity();
  for (int i = 1; i <= 18; ++i) {
    term = term * X / static_cast<double>(i);
    E += term;
  }
  for (int i = 0; i < squarings; ++i) {E = (E * E).eval();}
  TorqueLinearModel d;
  d.A = E.block<6, 6>(0, 0);
  d.B = E.block<6, 4>(0, 6);
  d.c = E.block<6, 1>(0, 10);
  return d;
}

// ── MPC ─────────────────────────────────────────────────────────
TorqueMPC::TorqueMPC(const TorqueModel & model, const TorqueTuning & tuning)
: md_(model), tu_(tuning)
{
  if (tu_.Np < 1 || tu_.Nc < 1 || tu_.Nc > tu_.Np) {throw std::invalid_argument("1 ≤ Nc ≤ Np 여야 한다");}
  if (!(tu_.Ts > 0.0)) {throw std::invalid_argument("Ts > 0 이어야 한다");}
  if (!(md_.mass > 0.0 && md_.inertia_z > 0.0 && md_.wheel_radius > 0.0)) {
    throw std::invalid_argument("mass, inertia_z, wheel_radius 는 양수여야 한다");
  }
  if (!(tu_.dist_alpha > 0.0 && tu_.dist_alpha <= 1.0)) {throw std::invalid_argument("0 < disturbance.alpha ≤ 1");}
  if (!(tu_.rate_fraction > 0.0)) {throw std::invalid_argument("rate_fraction > 0 이어야 한다");}
  if (md_.tau_motor_limit() <= 0.0) {throw std::invalid_argument("τ_lim − τ_fric,max ≤ 0 — 모터 여유가 없다");}
  if (!(tu_.w_delta >= 0.0 && tu_.r_u >= 0.0 && tu_.q_null >= 0.0)) {
    throw std::invalid_argument("w_delta, r_u, q_null ≥ 0 이어야 한다");
  }
  if (!(tu_.w_delta + tu_.r_u > 0.0)) {throw std::invalid_argument("w_delta + r_u > 0 이어야 H 가 양의 정부호다");}

  nU_ = 4 * tu_.Nc;
  ground_ = tu_.cap_on_ground && tu_.limit_mode == TorqueLimitMode::Friction;
  P_ground_ = md_.ground_map();
  const int mg_rate = 4 * (tu_.Nc - 1);
  const int mg = mg_rate + (ground_ ? 4 * tu_.Nc : 0);       // 2026-11-22 바닥 토크 행 (cap_on_ground)

  // H_const = Dᵀ W̄_Δ D + R̄_U + q_null N.   DᵀD 는 블록 삼중대각: 대각 2I (마지막 I), 이웃 −I
  H_const_ = Eigen::MatrixXd::Zero(nU_, nU_);
  const Eigen::Matrix4d NN = null_dir() * null_dir().transpose();
  for (int j = 0; j < tu_.Nc; ++j) {
    const double diag = (j == tu_.Nc - 1) ? 1.0 : 2.0;
    H_const_.block<4, 4>(4 * j, 4 * j) =
      (tu_.w_delta * diag + tu_.r_u) * Eigen::Matrix4d::Identity() + tu_.q_null * NN;
    if (j + 1 < tu_.Nc) {
      H_const_.block<4, 4>(4 * j, 4 * (j + 1)) = -tu_.w_delta * Eigen::Matrix4d::Identity();
      H_const_.block<4, 4>(4 * (j + 1), 4 * j) = -tu_.w_delta * Eigen::Matrix4d::Identity();
    }
  }
  // 일반 제약: j ≥ 1 의 τ_j − τ_{j−1}
  A_gen_.setZero(mg, nU_);
  for (int j = 1; j < tu_.Nc; ++j) {
    for (int i = 0; i < 4; ++i) {
      A_gen_(4 * (j - 1) + i, 4 * j + i) = 1.0;
      A_gen_(4 * (j - 1) + i, 4 * (j - 1) + i) = -1.0;
    }
  }
  if (ground_) {                  // 행 mg_rate + 4j + i : (P τ_j)_i
    for (int j = 0; j < tu_.Nc; ++j) {A_gen_.block<4, 4>(mg_rate + 4 * j, 4 * j) = P_ground_;}
  }

  {
    const TMat34 F = md_.force_map();
    F_pinv_ = F.transpose() * (F * F.transpose()).inverse();      // 식-14 의 ((T⁻¹)ᵀ)⁺ — 행이 직교라 닫힌 형과 같다
    F_ = F;
  }
  if (!(tu_.load_com_height >= 0.0 && tu_.load_min_frac > 0.0 && tu_.load_min_frac <= 1.0)) {
    throw std::invalid_argument("load_transfer: com_height ≥ 0, 0 < min_frac ≤ 1 이어야 한다");
  }
  if (!(tu_.slip_alloc_floor > 0.0 && tu_.slip_alloc_floor <= 1.0)) {
    throw std::invalid_argument("slip_alloc_floor 는 0 < 값 ≤ 1 (가중 배분에서 η_s 아래 한계)");
  }
  tb_steps_.setZero(4, tu_.Nc);
  // 2026-11-24 마찰 상한 옮김·마찰 모델 (머리주석). 옮김은 friction 모드에서만 실제로 켠다
  if (tu_.cap_friction_shift || tu_.use_fric_model) {
    if (!(tu_.cap_friction_scale >= 0.0 && tu_.cap_friction_scale <= 1.5 && tu_.cap_friction_smooth > 0.0)) {
      throw std::invalid_argument("cap_friction_shift·use_fric_model: 0 ≤ scale ≤ 1.5, smooth > 0 이어야 한다");
    }
    if ((md_.fric_coulomb.array() < 0.0).any() || (md_.fric_viscous.array() < 0.0).any() ||
      !md_.fric_coulomb.allFinite() || !md_.fric_viscous.allFinite())
    {
      throw std::invalid_argument("cap_friction_shift·use_fric_model: 구동계 마찰 계수 (coulomb·viscous) 는 0 이상의 유한한 값이어야 한다");
    }
    if (!(tu_.fric_rolling >= 0.0 && tu_.fric_rolling <= 0.02)) {     // 2026-11-24 구름 손실 (잰 값 최대 7 mN·m 의 3 배까지 [구현 선택])
      throw std::invalid_argument("fric_rolling: 0 ≤ 값 ≤ 0.02 N·m 이어야 한다 (구름 손실 — 마찰 모델에 더하는 네 바퀴 공통 토크)");
    }
  }
  fshift_ = tu_.cap_friction_shift && tu_.limit_mode == TorqueLimitMode::Friction;
  if (!(tu_.roller_rr >= 0.0 && tu_.roller_rr <= 2.0) || !(tu_.roller_s0 > 0.0)) {     // 2026-11-25 롤러 구름 저항 (잰 값 0.88 N 의 약 2 배까지 [구현 선택])
    throw std::invalid_argument("roller_rr: 0 ≤ 값 ≤ 2 N, roller_s0 > 0 이어야 한다 (롤러 하나당 구름 저항)");
  }
  fs_steps_.setZero(4, tu_.Nc);
  fmodel_ = tu_.use_fric_model;
  ff_steps_.setZero(4, tu_.Np);
  if (tu_.dist_kalman) {         // 축별 정상상태 Kalman 이득 (머리주석 "외란 관측기 이득")
    if (!((tu_.dist_kq.array() > 0.0).all() && (tu_.dist_kr_pos.array() > 0.0).all() && (tu_.dist_kr_vel.array() > 0.0).all())) {
      throw std::invalid_argument("dist_kalman: kalman_q·r_pos·r_vel 은 모두 양수여야 한다");
    }
    for (int ax = 0; ax < 3; ++ax) {
      K_kal_.row(ax) = steady_kalman_gain(tu_.Ts, tu_.dist_kq(ax), tu_.dist_kr_pos(ax), tu_.dist_kr_vel(ax));
    }
  }
  if (!(tu_.ff_normal_min > 0.0 && tu_.ff_normal_min <= tu_.ff_normal_max && tu_.ff_normal_q >= 0.0 && tu_.ff_normal_P0 > 0.0)) {
    throw std::invalid_argument("ff_normal: 0 < min ≤ max, q ≥ 0, P0 > 0 이어야 한다");
  }
  if (tu_.ff_normal_estimate && !(tu_.dist_kalman && tu_.dist_body_frame)) {
    throw std::invalid_argument("ff_normal_estimate 는 dist_kalman (observer kalman) 과 dist_body_frame (frame body) 이 켜져 있어야 한다");
  }
  rho_ = std::clamp(tu_.ff_normal_scale, tu_.ff_normal_min, tu_.ff_normal_max);   // 추정 시작값
  z_ << 0.0, 0.0, 1.0 - rho_;
  Pz_ = Eigen::Vector3d(kJointPd0, kJointPd0, tu_.ff_normal_P0).asDiagonal();
  Us_.setZero(nU_);
  Phi_.setZero(6 * tu_.Np, nU_);
  WPhi_.setZero(6 * tu_.Np, nU_);
  H_.setZero(nU_, nU_);
  e0_.setZero(6 * tu_.Np);
  g_.setZero(nU_);
  T_.setZero(nU_);
  lam_.setZero(nU_ + mg);
  lb_.setZero(nU_ + mg);
  ub_.setZero(nU_ + mg);
}

void TorqueMPC::reset()
{
  has_prev_ = false;
  has_lin_ = false;
  x_prev_.setZero();
  tau_prev_.setZero();
  fs_used_.setZero();
  d_hat_.setZero();
  a_hat_.setZero();
  rho_have_ = false;             // d 몫은 지우고 η̂ 와 그 분산은 남긴다 (머리주석)
  z_.head<2>().setZero();
  {
    const double pe = Pz_(2, 2);
    Pz_ = Eigen::Vector3d(kJointPd0, kJointPd0, pe).asDiagonal();
  }
  T_.setZero();
  stats_ = TorqueStepStats{};
}

TVec4 TorqueMPC::tau_bar() const
{
  TVec4 out = TVec4::Constant(md_.tau_motor_limit());
  if (tu_.limit_mode == TorqueLimitMode::Friction) {out = out.cwiseMin(md_.tau_friction_limits());}   // 바퀴별 (2026-11-19)
  for (int i = 0; i < 4; ++i) {
    if (tau_bar_override_(i) >= 0.0) {out(i) = std::min(tau_bar_override_(i), md_.tau_motor_limit());}
  }
  return out;
}

void TorqueMPC::update_disturbance(const TVec6 & x_hat, double dt_state)
{
  if (!has_prev_) {
    d_hat_.setZero();
    return;
  }
  if (tu_.ff_normal_estimate) {  // 결합 Kalman 예측 단계: random walk 분산을 더한다 (갱신을 건너뛰는 주기도). η 분산은 10 P0 에서 멈춘다
    const double Ts = tu_.Ts;
    Pz_.diagonal() += Ts * Eigen::Vector3d(tu_.dist_kq(0) * tu_.dist_kq(0), tu_.dist_kq(1) * tu_.dist_kq(1), tu_.ff_normal_q * tu_.ff_normal_q);
    const double pmax = 10.0 * tu_.ff_normal_P0;
    if (Pz_(2, 2) > pmax) {
      const double f = std::sqrt(pmax / Pz_(2, 2));
      Pz_.row(2) *= f;
      Pz_.col(2) *= f;
    }
  }
  // 식-9: 한 스텝 전 상태·토크로 예측한 값과 지금 추정값의 차
  TVec6 pred;
  TVec4 tau_res = tau_prev_;     // 잔차 모델에 넣는 토크. 2026-11-24 마찰 모델이면 바퀴에 남는 몫 τ − τ̂_b (추정 상태의 바퀴 속도로 계산)
  if (fmodel_) {tau_res -= friction_at_state(x_prev_);}
  if (tu_.linearization == TorqueLinearization::CurrentState && has_lin_) {
    // 직전 주기에 MPC 가 쓴 바로 그 모델 (x̂(k−1), τ(k−2) 에서 선형화) 의 한 스텝 예측
    pred = lin_prev_.A * x_prev_ + lin_prev_.B * tau_res + lin_prev_.c;
  } else {
    const double Ts = tu_.Ts;
    const TVec3 Xp = x_prev_.head<3>(), Vp = x_prev_.tail<3>();
    const TVec3 a = md_.G(Xp(2)) * tau_res + md_.w(Vp);
    pred.head<3>() = Xp + Ts * Vp + 0.5 * Ts * Ts * a;
    pred.tail<3>() = Vp + Ts * a;
  }
  if (tu_.roller_rr > 0.0) {      // 2026-11-25 롤러 구름 저항: 한 스텝 예측에도 넣는다 (안 넣으면 d̂ 가 같은 저항을 한 번 더 배운다)
    const double th = x_prev_(2), c = std::cos(th), s = std::sin(th);
    const TVec3 vb(c * x_prev_(3) + s * x_prev_(4), -s * x_prev_(3) + c * x_prev_(4), x_prev_(5));
    const TVec3 a_rr = roller_accel(vb, th);
    pred.head<3>() += 0.5 * tu_.Ts * tu_.Ts * a_rr;
    pred.tail<3>() += tu_.Ts * a_rr;
  }
  TVec6 d = x_hat - pred;
  d(2) = wrap(d(2));
  // 갱신 건너뛰기 (지시서 4절·7.4절): 상태 간격이 Ts 에서 벗어나면 잔차가 −Ts·ẋ 꼴로 틀리고, SLAM 보정 계단은 위치 잔차로 들어와
  // 예측 구간 끝에서 j 배로 커진다. 이 주기는 d̂_f 를 그대로 둔다 (x̂_prev 는 step 끝에서 갱신).
  if (tu_.dist_dt_tol > 0.0 && dt_state > 0.0 && std::abs(dt_state - tu_.Ts) > tu_.dist_dt_tol) {
    stats_.d_skipped = 1;
    return;
  }
  if (tu_.dist_skip_xy > 0.0 && std::max(std::abs(d(0)), std::abs(d(1))) > tu_.dist_skip_xy) {
    stats_.d_skipped = 2;
    return;
  }
  // 포화 중 갱신 멈춤 (2026-11-21, 머리주석): 직전에 낸 τ 가 상한에 붙었거나 목표 입력이 상한 밖이면 잔차가 "못 낸 힘" 을 담는다
  if (tu_.dist_freeze_on_sat) {
    const TVec4 tb = tau_bar().cwiseProduct(slip_scale_used_);   // 2026-11-23 η_s: tau_prev_ 를 낸 상자 (η_s 1 이면 예전 그대로)
    TVec4 tp = ground_ ? TVec4(P_ground_ * tau_prev_) : tau_prev_;                // 바닥 토크 상한이면 바닥 몫으로 견준다
    TVec4 us0 = Us_.size() >= 4 ? (ground_ ? TVec4(P_ground_ * Us_.head<4>()) : TVec4(Us_.head<4>())) : TVec4::Zero();
    if (fshift_) {     // 2026-11-24 마찰 상한 옮김: 상자가 f 만큼 옮겨져 있으니 τ − f (바닥 쪽 몫) 로 견준다
      const TVec4 fs = ground_ ? TVec4(P_ground_ * fs_used_) : fs_used_;
      tp -= fs;
      us0 -= fs;
    }
    const bool sat = (tp.cwiseAbs().array() >= 0.98 * tb.array()).any() || us_clipped_ ||
      (Us_.size() >= 4 && (us0.cwiseAbs().array() > tb.array()).any());
    if (sat) {
      stats_.d_skipped = 3;
      return;
    }
  }
  if (tu_.dist_accel_only && !tu_.dist_kalman) {     // 가속도 외란 모델 (머리주석, 2026-11-18): 위치 잔차를 속도 잔차에서 만든다
    d.head<3>() = 0.5 * tu_.Ts * d.tail<3>();
  }
  if (tu_.dist_body_frame) {     // 몸체 좌표 (2026-11-21): x·y 성분을 R(θ̂)ᵀ 로
    const double c = std::cos(x_hat(2)), s = std::sin(x_hat(2));
    for (int k : {0, 3}) {
      const double ex = d(k), ey = d(k + 1);
      d(k) = c * ex + s * ey;
      d(k + 1) = -s * ex + c * ey;
    }
  }
  const double Ts = tu_.Ts;
  if (tu_.dist_kalman) {         // 정상상태 Kalman 이득 (머리주석 "외란 관측기 이득"): 축마다 â += K [r_p − ½Ts² â ; r_v − Ts â]
    if (tu_.ff_normal_estimate) {  // x·y 는 η 와 같이 (머리주석 "구심 feedforward 계수")
      joint_kalman_update(d, x_hat(2));
      a_hat_.head<2>() = z_.head<2>();
    }
    for (int ax = tu_.ff_normal_estimate ? 2 : 0; ax < 3; ++ax) {
      const double ip = d(ax) - 0.5 * Ts * Ts * a_hat_(ax), iv = d(ax + 3) - Ts * a_hat_(ax);
      a_hat_(ax) += K_kal_(ax, 0) * ip + K_kal_(ax, 1) * iv;
    }
    d_hat_ << 0.5 * Ts * Ts * a_hat_, Ts * a_hat_;
  } else {
    d_hat_ = (1.0 - tu_.dist_alpha) * d_hat_ + tu_.dist_alpha * d;     // 식-11
  }
  // 상한 (2026-11-21): 가속도 크기로 자른다. 위치 성분도 같은 비율로 (accel_only 면 ½Ts·속도 꼴이 유지된다)
  if (tu_.dist_acc_max > 0.0) {
    const double n = std::hypot(d_hat_(3), d_hat_(4)) / Ts;
    if (n > tu_.dist_acc_max) {
      const double f = tu_.dist_acc_max / n;
      d_hat_(0) *= f; d_hat_(1) *= f; d_hat_(3) *= f; d_hat_(4) *= f;
    }
  }
  if (tu_.dist_yaw_acc_max > 0.0) {
    const double n = std::abs(d_hat_(5)) / Ts;
    if (n > tu_.dist_yaw_acc_max) {
      const double f = tu_.dist_yaw_acc_max / n;
      d_hat_(2) *= f; d_hat_(5) *= f;
    }
  }
  if (tu_.dist_kalman) {a_hat_ = d_hat_.tail<3>() / Ts;}   // 상한에 잘렸으면 Kalman 상태도 같이
  if (tu_.ff_normal_estimate) {z_.head<2>() = a_hat_.head<2>();}
}

void TorqueMPC::joint_kalman_update(const TVec6 & d, double theta)
{
  // 머리주석 "구심 feedforward 계수": z = [d_x, d_y, η] (몸체),  측정 [r_p,x; r_p,y; r_v,x; r_v,y] (몸체 잔차) = H z + v
  const double Ts = tu_.Ts, hp = 0.5 * Ts * Ts;
  Eigen::Vector2d g = Eigen::Vector2d::Zero();       // g = (c_n/m_eq) R(θ̂)ᵀ n̂_c — 잔차를 돌린 것과 같은 θ̂
  if (rho_have_) {
    const double c = std::cos(theta), s = std::sin(theta);
    g << c * rho_n_prev_(0) + s * rho_n_prev_(1), -s * rho_n_prev_(0) + c * rho_n_prev_(1);
    g *= rho_cn_prev_ / md_.m_eff();
  }
  Eigen::Matrix<double, 4, 3> H;
  H << hp, 0.0, hp * g(0),
    0.0, hp, hp * g(1),
    Ts, 0.0, Ts * g(0),
    0.0, Ts, Ts * g(1);
  const Eigen::Vector4d y(d(0), d(1), d(3), d(4));
  const Eigen::Vector4d rd(tu_.dist_kr_pos(0) * tu_.dist_kr_pos(0), tu_.dist_kr_pos(1) * tu_.dist_kr_pos(1),
    tu_.dist_kr_vel(0) * tu_.dist_kr_vel(0), tu_.dist_kr_vel(1) * tu_.dist_kr_vel(1));
  const Eigen::Matrix4d Rm = rd.asDiagonal();
  const Eigen::Matrix4d S = H * Pz_ * H.transpose() + Rm;
  const Eigen::Matrix<double, 3, 4> K = (S.ldlt().solve(H * Pz_)).transpose();     // P Hᵀ S⁻¹ (S, P 대칭)
  z_ += K * (y - H * z_);
  const Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity() - K * H;
  Pz_ = IKH * Pz_ * IKH.transpose() + K * Rm * K.transpose();          // Joseph form
  z_(2) = std::clamp(z_(2), 1.0 - tu_.ff_normal_max, 1.0 - tu_.ff_normal_min);
  rho_ = 1.0 - z_(2);
  const double gn = g.norm();
  stats_.rho_updated = gn > 0.0 ? 1 : 0;
  stats_.rho_g = gn;
  stats_.rho_r = gn > 0.0 ? (d(3) * g(0) + d(4) * g(1)) / (gn * Ts) : 0.0;
}

Eigen::RowVector2d TorqueMPC::steady_kalman_gain(double Ts, double q_rate, double r_pos, double r_vel)
{
  // 상태 a (스칼라 random walk, 스텝 분산 q = q_rate²·Ts), 측정 H = [½Ts²; Ts], R = diag(r_pos², r_vel²)
  const double q = q_rate * q_rate * Ts;
  const Eigen::Vector2d H(0.5 * Ts * Ts, Ts);
  const Eigen::Matrix2d R = Eigen::Vector2d(r_pos * r_pos, r_vel * r_vel).asDiagonal();
  double P = q;
  Eigen::RowVector2d K = Eigen::RowVector2d::Zero();
  for (int it = 0; it < 200000; ++it) {
    const double Pm = P + q;                                    // 예측
    const Eigen::Matrix2d S = H * Pm * H.transpose() + R;
    const Eigen::RowVector2d Kn = Pm * H.transpose() * S.inverse();
    const double Pn = (1.0 - Kn.dot(H)) * Pm;                    // 갱신
    const bool done = std::abs(Pn - P) <= 1e-14 * std::max(P, 1e-300);
    P = Pn;
    K = Kn;
    if (done) {break;}
  }
  return K;
}

TVec6 TorqueMPC::d_hat_world(double theta) const
{
  if (!tu_.dist_body_frame) {return d_hat_;}
  TVec6 out = d_hat_;
  const double c = std::cos(theta), s = std::sin(theta);
  for (int k : {0, 3}) {
    out(k) = c * d_hat_(k) - s * d_hat_(k + 1);
    out(k + 1) = s * d_hat_(k) + c * d_hat_(k + 1);
  }
  return out;
}

TVec4 TorqueMPC::friction_at_state(const TVec6 & x) const
{
  // 관성 좌표 속도 → 몸체 속도 R(θ)ᵀ → 무슬립 바퀴 속도 T⁻¹ → 마찰 토크 (지평에서 쓰는 것과 같은 배율·ω_s)
  const double c = std::cos(x(2)), s = std::sin(x(2));
  const TVec3 vb(c * x(3) + s * x(4), -s * x(3) + c * x(4), x(5));
  return model_friction(md_.wheel_nominal_speeds(vb));
}

TVec3 TorqueMPC::roller_accel(const TVec3 & v_body, double theta) const
{
  if (!(tu_.roller_rr > 0.0)) {return TVec3::Zero();}
  const TVec3 q = md_.roller_force(v_body, tu_.roller_rr, tu_.roller_s0);
  const double c = std::cos(theta), s = std::sin(theta);
  return TVec3((c * q(0) - s * q(1)) / md_.m_eff(), (s * q(0) + c * q(1)) / md_.m_eff(), q(2) / md_.I_eff());
}

TVec4 TorqueMPC::model_friction(const TVec4 & omega) const
{
  // 모델 (예측·목표 입력·잔차) 에 넣는 마찰: 배율을 곱한 구동계 마찰 + 구름 손실 (2026-11-24, 머리주석). 상한 옮김은 앞 항만 쓴다
  TVec4 out = tu_.cap_friction_scale * md_.drivetrain_friction(omega, tu_.cap_friction_smooth);
  if (tu_.fric_rolling != 0.0) {
    for (int i = 0; i < 4; ++i) {out(i) += tu_.fric_rolling * std::tanh(omega(i) / tu_.cap_friction_smooth);}
  }
  return out;
}

void TorqueMPC::var_box(int i, int j, double & lo, double & hi) const
{
  // 바퀴 i, 지평 스텝 j 의 바퀴 토크 변수 상자 (j=0 의 변화율 교집합 전)
  const double tmot = md_.tau_motor_limit();
  const double box = ground_ ? tmot : tb_steps_(i, j);     // 바닥 토크 상한이면 바퀴 토크 자체는 모터 한계까지 (마찰 상한은 일반 제약)
  lo = -box;
  hi = box;
  if (fshift_ && !ground_) {     // 2026-11-24 마찰 상한 옮김 (머리주석): f 만큼 옮기고 모터 한계 안으로
    const double f = fs_steps_(i, j);
    lo = std::max(-box + f, -tmot);
    hi = std::min(box + f, tmot);
    if (lo > hi) {lo = hi = std::clamp(f, -tmot, tmot);}   // |f| 가 모터 한계 + 상자보다 클 때만 (실제 값으로는 안 생긴다)
  }
}

bool TorqueMPC::solve_qp(const TVec4 & tau_bar)
{
  const int mg = static_cast<int>(A_gen_.rows());
  const int mg_rate = 4 * (tu_.Nc - 1);
  for (int j = 0; j < tu_.Nc; ++j) {
    TVec4 pf = TVec4::Zero();     // 바닥 토크 행을 옮기는 양 (P f)_i — 마찰 상한 옮김 + cap_on ground 일 때만 쓴다
    if (fshift_ && ground_) {pf = P_ground_ * fs_steps_.col(j);}
    for (int i = 0; i < 4; ++i) {
      const double tb = tb_steps_(i, j), dtb = dtau_bar_of(tau_bar(i));   // 상자는 스텝별 (하중 이동), 변화율은 정지 상한 기준
      double lo, hi;
      var_box(i, j, lo, hi);
      if (j == 0) {     // 첫 스텝의 변화율은 단일 변수라 bound 에 합친다. 상한이 우선 (머리주석)
        const double blo = lo, bhi = hi;
        lo = std::clamp(tau_prev_(i) - dtb, blo, bhi);
        hi = std::clamp(tau_prev_(i) + dtb, blo, bhi);
      } else {
        lb_(nU_ + 4 * (j - 1) + i) = -dtb;
        ub_(nU_ + 4 * (j - 1) + i) = dtb;
      }
      lb_(4 * j + i) = lo;
      ub_(4 * j + i) = hi;
      if (ground_) {
        lb_(nU_ + mg_rate + 4 * j + i) = -tb;
        ub_(nU_ + mg_rate + 4 * j + i) = tb;
        if (fshift_) {     // 2026-11-24: −τ̄ + (P f)_i ≤ (P τ)_i ≤ τ̄ + (P f)_i
          lb_(nU_ + mg_rate + 4 * j + i) += pf(i);
          ub_(nU_ + mg_rate + 4 * j + i) += pf(i);
        }
      }
    }
  }
  DAQPProblem qp{};
  qp.n = nU_;
  qp.m = nU_ + mg;
  qp.ms = nU_;
  qp.H = H_.data();               // 대칭이라 저장 순서 무관.  J = TᵀHT + 2gᵀT 의 최소점 = ½TᵀHT + gᵀT 의 최소점
  qp.f = g_.data();
  qp.A = mg > 0 ? A_gen_.data() : nullptr;
  qp.bupper = ub_.data();
  qp.blower = lb_.data();
  qp.sense = nullptr;
  DAQPSettings settings;
  daqp_default_settings(&settings);
  settings.iter_limit = tu_.daqp_iter_limit;
  DAQPResult res{};
  res.x = T_.data();
  res.lam = lam_.data();
  daqp_quadprog(&res, &qp, &settings);
  stats_.exitflag = res.exitflag;
  stats_.iters = res.iter;
  stats_.active = static_cast<int>((lam_.array().abs() > 1e-12).count());
  return res.exitflag >= 1;
}

TVec4 TorqueMPC::step(
  const TVec6 & x_hat, const std::vector<TorqueRefPoint> & ref, const std::vector<TVec3> * acc_ref, double dt_state)
{
  const int Np = tu_.Np, Nc = tu_.Nc;
  if (static_cast<int>(ref.size()) < Np + 1) {throw std::invalid_argument("기준 지평은 Np+1 점이 필요하다");}
  const double Ts = tu_.Ts;
  stats_ = TorqueStepStats{};

  update_disturbance(x_hat, dt_state);
  // 지평 스텝 i 에서 쓰는 d̂ (월드 좌표): 몸체 저장이면 그 스텝의 기준 방향 θ_r,i 로 돌린다 (2026-11-21)
  auto d_at = [this](double th) -> TVec6 {return tu_.dist_enable ? d_hat_world(th) : TVec6::Zero();};
  const Eigen::Matrix3d Mdiag = TVec3(md_.m_eff(), md_.m_eff(), md_.I_eff()).asDiagonal();
  const TVec4 tb_now = tau_bar();
  // 2026-11-23 슬립 계수 η_s (머리주석): 상자에만 곱한다 (변화율은 tb_now 기준). 1 이면 예전 그대로
  const TVec4 eta_s = slip_scale_;
  const bool slip_w = tu_.slip_alloc && (eta_s.array() < 1.0 - 1e-9).any();
  const TVec4 eta_w = eta_s.cwiseMax(tu_.slip_alloc_floor);
  // 2026-11-23 하중 이동 (TorqueTuning load_*): 기본은 열마다 tb_now, 켜면 아래 지평 루프에서 스텝마다 바꾼다
  tb_steps_.resize(4, Nc);
  tb_steps_.colwise() = tb_now.cwiseProduct(eta_s);
  const bool load_on = tu_.load_com_height > 0.0;
  const TVec4 n0 = md_.wheel_normal_forces(0.0, 0.0, 0.0, 1.0);
  const TVec4 tf0 = md_.tau_friction_limits();
  const bool cap_dyn = load_on && tu_.load_dynamic_cap && tu_.limit_mode == TorqueLimitMode::Friction &&
    !(tau_bar_override_.array() >= 0.0).any();
  n_now_ = n0;

  rho_have_ = false;              // 이번 주기 i = 0 에서 다시 채운다 (다음 주기 결합 Kalman 측정 회귀)
  us_clipped_ = false;
  const double rho_use = ff_normal_rho();

  // current_state: 이번 주기의 모델을 x̂(k), τ(k−1) 에서 한 번 만든다 (머리주석 "선형화 방식")
  const bool current = tu_.linearization == TorqueLinearization::CurrentState;
  TorqueLinearModel lin;
  Eigen::Matrix<double, 6, Eigen::Dynamic> Phi_j;
  if (current) {
    lin = discretize_zoh(linearize_continuous(md_, x_hat, tau_prev_), Ts);
    Phi_j = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, nU_);
  }

  // 지평을 따라: 영입력 응답 x_free (w, d̂ 포함), 기준 상태 x_r, 토크 응답 Φ
  TVec6 xf = x_hat;
  Eigen::Matrix<double, 3, Eigen::Dynamic> P = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, nU_);
  Eigen::Matrix<double, 3, Eigen::Dynamic> V = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, nU_);
  // 식-15: 첫 점을 θ̂ 에 가장 가까운 가지로, 이후는 앞 점과 연속
  double th_r = x_hat(2) + wrap(ref[0].first(2) - x_hat(2));
  for (int i = 0; i < Np; ++i) {
    const int cidx = std::min(i, Nc - 1);
    const TVec6 d_use = d_at(th_r);
    const TVec3 a_d = d_use.tail<3>() / Ts;          // 식-14 의 â_d
    // 기준 속도·가속도 (지평 모든 스텝 — ρ 외란을 예측에도 넣으려고)
    const TVec3 & u_i = ref[i].second;
    const double c = std::cos(th_r), s = std::sin(th_r);
    const TVec3 Xdot_r(u_i(0) * c - u_i(1) * s, u_i(0) * s + u_i(1) * c, u_i(2));
    TVec3 acc;
    if (acc_ref && static_cast<int>(acc_ref->size()) > i) {
      acc = (*acc_ref)[i];
    } else {
      // 지평 점에서: p̈ = d/dt (R(θ)u) = R(θ)(u̇ + ω S u),  S u = [−u_y, u_x, 0]
      const int i0 = std::max(i - 1, 0), i1 = i + 1;
      const TVec3 du = (ref[i1].second - ref[i0].second) / ((i1 - i0) * Ts);
      const TVec3 body(du(0) - u_i(2) * u_i(1), du(1) + u_i(2) * u_i(0), du(2));
      acc << body(0) * c - body(1) * s, body(0) * s + body(1) * c, body(2);
    }
    // 구심 feedforward 계수 ρ (머리주석): 곡률 중심 쪽 구심 몫 c_n = n̂_c·M(p̈_r − w) 중 (1 − ρ) 만큼을 "모델에 없는 안쪽 힘" 으로 본다 →
    //   가속도 외란 a_ρ = (1 − ρ) c_n n̂_c / m_eff 를 U_s (빼기) 와 예측 (더하기) 에 같이 — offset-free 의 조건 (외란이 예측 모델에도 들어간다)
    TVec3 a_rho = TVec3::Zero();
    const double sp_r = std::hypot(Xdot_r(0), Xdot_r(1));
    if (sp_r > 1e-3 && std::abs(u_i(2)) > 1e-3) {
      const double sg = u_i(2) > 0.0 ? 1.0 : -1.0;
      const Eigen::Vector2d n_c(-sg * Xdot_r(1) / sp_r, sg * Xdot_r(0) / sp_r);
      const TVec3 cent = Mdiag * (acc - md_.w(Xdot_r));
      const double c_n = n_c.dot(cent.head<2>());
      a_rho.head<2>() = (1.0 - rho_use) * c_n / md_.m_eff() * n_c;
      if (i == 0 && std::abs(u_i(2)) > 0.3 && sp_r > 0.2 && std::abs(c_n) > tu_.ff_normal_x_min) {
        rho_have_ = true;
        rho_cn_prev_ = c_n;
        rho_n_prev_ = n_c;
      }
    }
    // 2026-11-25 롤러 구름 저항 (머리주석): 기준 몸체 속도의 저항 가속도. a_ρ 와 같이 U_s 에서 빼고 예측에 더한다. 0 이면 0
    const TVec3 a_rr = roller_accel(u_i, th_r);
    // 2026-11-24 마찰 상한 옮김·마찰 모델 (머리주석): 스텝 i 의 기준 바퀴 속도에서 구동계 마찰 τ_b,i. 둘 다 꺼지면 계산하지 않는다
    TVec4 tau_b = TVec4::Zero();
    if (fmodel_ || (fshift_ && i < Nc)) {
      tau_b = tu_.cap_friction_scale * md_.drivetrain_friction(md_.wheel_nominal_speeds(u_i), tu_.cap_friction_smooth);
    }
    // 2026-11-24 구름 손실 (머리주석): 모델 쪽 마찰 τ_m = τ_b + τ_roll·tanh(ω_r/ω_s). 상한 옮김 (fs_steps_) 은 τ_b 그대로 둔다
    const TVec4 tau_m = fmodel_ && tu_.fric_rolling != 0.0 ? model_friction(md_.wheel_nominal_speeds(u_i)) : tau_b;
    if (fmodel_) {ff_steps_.col(i) = tau_m;}
    if (i < Nc) {
      if (fshift_) {fs_steps_.col(i) = tau_b;}
      // 식-14 목표 입력: U_s,i = F⁺ R(θ_r,i)ᵀ M (p̈_r,i − w_i − â_d − a_ρ)
      const TVec3 rhs = Mdiag * (acc - md_.w(Xdot_r) - a_d - a_rho - a_rr);
      const TVec3 rb(c * rhs(0) + s * rhs(1), -s * rhs(0) + c * rhs(1), rhs(2));   // R(θ_r)ᵀ
      TVec4 cw = TVec4::Ones();          // 배분 가중 c_i (가중이 없으면 최소 노름 F⁺)
      bool weighted = false;
      if (load_on) {
        // 하중 이동: 기준 관성 가속도의 몸체 성분 → N_i,j. 상한 τ̄ ∝ μ̂_i N_i 를 N_i,j/N_i,0 배, 배분 가중 c_i = 그 마찰 한계
        const TVec4 nj = md_.wheel_normal_forces(c * acc(0) + s * acc(1), -s * acc(0) + c * acc(1), tu_.load_com_height, tu_.load_min_frac);
        const TVec4 cap = tf0.cwiseProduct(nj.cwiseQuotient(n0));
        if (i == 0) {n_now_ = nj;}
        if (cap_dyn) {tb_steps_.col(i) = cap.cwiseMin(md_.tau_motor_limit()).cwiseProduct(eta_s);}
        if (tu_.load_weighted_alloc) {cw = cap; weighted = true;}
      }
      if (slip_w) {cw = cw.cwiseProduct(eta_w); weighted = true;}   // 2026-11-23 η_s: 미끄러지는 바퀴 몫을 나머지로
      if (weighted) {     // min Σ (τ_q / c_q)²  s.t. F τ = rb  →  τ = W⁻¹Fᵀ (F W⁻¹ Fᵀ)⁻¹ rb,  W⁻¹ = diag(c²)
        const Eigen::Matrix<double, 4, 3> WiFt = cw.cwiseProduct(cw).asDiagonal() * F_.transpose();
        Us_.segment<4>(4 * i) = WiFt * (F_ * WiFt).inverse() * rb;
      } else {
        Us_.segment<4>(4 * i) = F_pinv_ * rb;
      }
      if (fmodel_) {Us_.segment<4>(4 * i) += tau_m;}     // 2026-11-24 마찰 모델: 바퀴마다 자기 마찰만큼 더 낸다 (머리주석)
      if (tu_.target_projection && fshift_) {   // 2026-11-24 마찰 상한 옮김: 옮긴 상자에 사영한다
        if (ground_) {     // P (U_s − f) 가 ±τ̄ 안에 들도록 f 쪽으로 같은 비율로 줄인다 (f = 0 이면 아래 ground 분기와 같다)
          const TVec4 fs = fs_steps_.col(i);
          const TVec4 du = Us_.segment<4>(4 * i) - fs;
          const TVec4 ug = P_ground_ * du;
          double sc = 1.0;
          for (int q = 0; q < 4; ++q) {if (std::abs(ug(q)) > tb_steps_(q, i)) {sc = std::min(sc, tb_steps_(q, i) / std::abs(ug(q)));}}
          if (sc < 1.0) {Us_.segment<4>(4 * i) = fs + sc * du; if (i == 0) {us_clipped_ = true;}}
        } else {
          for (int q = 0; q < 4; ++q) {
            double blo, bhi;
            var_box(q, i, blo, bhi);
            const double u = Us_(4 * i + q), uc = std::clamp(u, blo, bhi);
            if (i == 0 && uc != u) {us_clipped_ = true;}
            Us_(4 * i + q) = uc;
          }
        }
      } else if (tu_.target_projection && ground_) {   // 바닥 토크 상한이면 P·U_s 가 상자 안에 들도록 같은 비율로 줄인다
        const TVec4 ug = P_ground_ * Us_.segment<4>(4 * i);
        double sc = 1.0;
        for (int q = 0; q < 4; ++q) {if (std::abs(ug(q)) > tb_steps_(q, i)) {sc = std::min(sc, tb_steps_(q, i) / std::abs(ug(q)));}}
        if (sc < 1.0) {Us_.segment<4>(4 * i) *= sc; if (i == 0) {us_clipped_ = true;}}
      } else if (tu_.target_projection) {   // 제약 있는 목표 입력 (머리주석): 바퀴별 [−τ̄, τ̄] 사영
        for (int q = 0; q < 4; ++q) {
          const double u = Us_(4 * i + q), uc = std::clamp(u, -tb_steps_(q, i), tb_steps_(q, i));
          if (i == 0 && uc != u) {us_clipped_ = true;}
          Us_(4 * i + q) = uc;
        }
      }
    }
    TVec6 d_rho;
    d_rho << 0.5 * Ts * Ts * (a_rho + a_rr), Ts * (a_rho + a_rr);      // ρ 외란·롤러 저항의 한 스텝 몫 (가속도 외란 꼴)
    if (current) {
      // x_{i+1} = A_d x_i + B_d τ + c_d + d̂ + d_ρ   — 지평 전체에 같은 (A_d, B_d, c_d)
      xf = lin.A * xf + lin.c + d_use + d_rho;
      if (fmodel_) {xf -= lin.B * tau_m;}                // 2026-11-24 마찰 모델: 바퀴에 남는 토크는 τ − τ_b
      Phi_j = (lin.A * Phi_j).eval();
      Phi_j.block<6, 4>(0, 4 * cidx) += lin.B;
      Phi_.block(6 * i, 0, 6, nU_) = Phi_j;
    } else {
      // 스텝 i 의 기준 방향·속도로 B_i, w_i (식-14)
      const TVec3 w_i = md_.w(Xdot_r);
      const TMat34 G_i = md_.G(th_r);

      // x_{i+1} = A x_i + B_i τ + E w_i + d̂ + d_ρ
      xf.head<3>() += Ts * xf.tail<3>() + 0.5 * Ts * Ts * w_i;
      xf.tail<3>() += Ts * w_i;
      xf += d_use + d_rho;
      if (fmodel_) {                                     // 2026-11-24 마찰 모델: −B_i τ_b,i = −[½Ts² G_i τ_b; Ts G_i τ_b]
        const TVec3 af = G_i * tau_m;
        xf.head<3>() -= 0.5 * Ts * Ts * af;
        xf.tail<3>() -= Ts * af;
      }
      P += Ts * V;
      P.block<3, 4>(0, 4 * cidx) += 0.5 * Ts * Ts * G_i;
      V.block<3, 4>(0, 4 * cidx) += Ts * G_i;
      Phi_.block(6 * i, 0, 3, nU_) = P;
      Phi_.block(6 * i + 3, 0, 3, nU_) = V;
    }

    // 기준 상태 x_r,{i+1}
    const double th_next = th_r + wrap(ref[i + 1].first(2) - ref[i].first(2));
    const TVec3 & u_n = ref[i + 1].second;
    const double cn = std::cos(th_next), sn = std::sin(th_next);
    TVec6 xr;
    xr << ref[i + 1].first(0), ref[i + 1].first(1), th_next,
      u_n(0) * cn - u_n(1) * sn, u_n(0) * sn + u_n(1) * cn, u_n(2);
    e0_.segment<6>(6 * i) = xf - xr;
    th_r = th_next;
  }

  // 식-18: H = ΦᵀW̄Φ + 상수,  g = ΦᵀW̄ e0 − DᵀW̄_Δ (e_1 ⊗ τ_prev)
  for (int i = 0; i < Np; ++i) {
    WPhi_.block(6 * i, 0, 6, nU_) = tu_.W.asDiagonal() * Phi_.block(6 * i, 0, 6, nU_);
  }
  H_.noalias() = Phi_.transpose() * WPhi_;
  H_ += H_const_;
  g_.noalias() = WPhi_.transpose() * e0_;
  g_.head<4>() -= tu_.w_delta * tau_prev_;
  g_ -= tu_.r_u * Us_;                              // 식-18 의 −R̄_U 𝐔_s

  const TVec4 tb = tau_bar();
  if (!tu_.constrained) {
    T_ = H_.ldlt().solve(-g_);
  } else if (!solve_qp(tb)) {
    T_ = H_.ldlt().solve(-g_);          // QP 실패 → 무제약 해를 상한으로 깎는다
    stats_.fallback = 1;
    for (int i = 0; i < 4; ++i) {
      const double dtb = dtau_bar_of(tb(i)), b0 = tb_steps_(i, 0);
      double blo = -b0, bhi = b0;
      if (fshift_) {     // 2026-11-24 마찰 상한 옮김: 깎는 상자도 f 만큼 옮긴다 (cap_on ground 도 여기서는 바퀴 토크 기준 근사)
        const double f = fs_steps_(i, 0), tmot = md_.tau_motor_limit();
        blo = std::max(-b0 + f, -tmot);
        bhi = std::min(b0 + f, tmot);
        if (blo > bhi) {blo = bhi = std::clamp(f, -tmot, tmot);}
      }
      T_(i) = std::clamp(
        T_(i), std::clamp(tau_prev_(i) - dtb, blo, bhi), std::clamp(tau_prev_(i) + dtb, blo, bhi));
    }
  }
  const TVec4 tau = T_.head<4>();
  x_prev_ = x_hat;
  tau_prev_ = tau;
  slip_scale_used_ = eta_s;
  if (fshift_) {fs_used_ = fs_steps_.col(0);}
  has_prev_ = true;
  lin_prev_ = lin;
  has_lin_ = current;
  return tau;
}

// ── 슬립 계수 η_s (2026-11-23, 머리주석) ───────────────────────────
SlipEta::SlipEta(const SlipEtaParams & p)
: p_(p)
{
  if (!(p_.s > 0.0 && p_.lambda_c > 0.0 && p_.lambda_c < 2.0 && p_.fc > 0.0 && p_.omega_eps > 0.0)) {
    throw std::invalid_argument("slip_eta: s > 0, 0 < lambda_c < 2, fc > 0, omega_eps > 0 이어야 한다");
  }
  tau_ = 1.0 / (2.0 * M_PI * p_.fc);
}

TVec4 SlipEta::slip_ratio(const TVec4 & omega, const TVec4 & omega_nom, double omega_eps)
{
  TVec4 out;
  for (int i = 0; i < 4; ++i) {
    const double den = std::max({std::abs(omega(i)), std::abs(omega_nom(i)), omega_eps});
    out(i) = (omega(i) - omega_nom(i)) * (omega_nom(i) >= 0.0 ? 1.0 : -1.0) / den;
  }
  return out;
}

double SlipEta::eta_of(double lambda_f, double lambda_c, double s)
{
  return 1.0 / (1.0 + std::exp((std::abs(lambda_f) - lambda_c) / s));
}

const TVec4 & SlipEta::update(const TVec4 & omega, const TVec4 & omega_nom, const bool valid[4], double dt)
{
  if (!(dt > 0.0) || dt > 0.1) {dt = 0.01;}        // 첫 표본·stamp 이상: 공칭 100 Hz
  const double a = dt / (tau_ + dt);
  const TVec4 lam = slip_ratio(omega, omega_nom, p_.omega_eps);
  for (int i = 0; i < 4; ++i) {
    if (!valid[i] || !std::isfinite(lam(i))) {continue;}
    lam_(i) = lam(i);
    lam_f_(i) += a * (lam(i) - lam_f_(i));
    eta_(i) = eta_of(lam_f_(i), p_.lambda_c, p_.s);
  }
  return eta_;
}

void SlipEta::reset()
{
  lam_.setZero();
  lam_f_.setZero();
  eta_.setOnes();
}

}  // namespace amr_mpc_cpp
