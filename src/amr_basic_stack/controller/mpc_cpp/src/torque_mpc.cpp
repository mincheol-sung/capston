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
  return F / wheel_radius;
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

TVec3 TorqueModel::w(const TVec3 & Xdot) const
{
  const double m = mu();
  if (m == 0.0) {return TVec3::Zero();}
  // S Ẋ_xy = [−ẏ, ẋ]
  return TVec3(-m * Xdot(2) * Xdot(1) / m_eff(), m * Xdot(2) * Xdot(0) / m_eff(), 0.0);
}

double TorqueModel::tau_friction_limit() const
{
  return wheel_radius / std::sqrt(2.0) * mu_hat * mass * gravity / 4.0 * c_s;
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
  // ∂w/∂Ẋ,  w = a [−θ̇ ẏ, θ̇ ẋ, 0],  a = μ / m_eff   (바퀴 관성 끔이면 a = 0)
  const double a = md.mu() / md.m_eff();
  lin.A(3, 4) = -a * thd;
  lin.A(3, 5) = -a * yd;
  lin.A(4, 3) = a * thd;
  lin.A(4, 5) = a * xd;
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

  nU_ = 4 * tu_.Nc;
  const int mg = 4 * (tu_.Nc - 1);

  // H_const = Dᵀ W̄_Δ D + q_null N.   DᵀD 는 블록 삼중대각: 대각 2I (마지막 I), 이웃 −I
  H_const_ = Eigen::MatrixXd::Zero(nU_, nU_);
  const Eigen::Matrix4d NN = null_dir() * null_dir().transpose();
  for (int j = 0; j < tu_.Nc; ++j) {
    const double diag = (j == tu_.Nc - 1) ? 1.0 : 2.0;
    H_const_.block<4, 4>(4 * j, 4 * j) = tu_.w_delta * diag * Eigen::Matrix4d::Identity() + tu_.q_null * NN;
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
  d_hat_.setZero();
  T_.setZero();
  stats_ = TorqueStepStats{};
}

TVec4 TorqueMPC::tau_bar() const
{
  double base = md_.tau_motor_limit();
  if (tu_.limit_mode == TorqueLimitMode::Friction) {base = std::min(base, md_.tau_friction_limit());}
  TVec4 out = TVec4::Constant(base);
  for (int i = 0; i < 4; ++i) {
    if (tau_bar_override_(i) >= 0.0) {out(i) = std::min(tau_bar_override_(i), md_.tau_motor_limit());}
  }
  return out;
}

void TorqueMPC::update_disturbance(const TVec6 & x_hat)
{
  if (!has_prev_) {
    d_hat_.setZero();
    return;
  }
  // 식-9: 한 스텝 전 상태·토크로 예측한 값과 지금 추정값의 차
  TVec6 pred;
  if (tu_.linearization == TorqueLinearization::CurrentState && has_lin_) {
    // 직전 주기에 MPC 가 쓴 바로 그 모델 (x̂(k−1), τ(k−2) 에서 선형화) 의 한 스텝 예측
    pred = lin_prev_.A * x_prev_ + lin_prev_.B * tau_prev_ + lin_prev_.c;
  } else {
    const double Ts = tu_.Ts;
    const TVec3 Xp = x_prev_.head<3>(), Vp = x_prev_.tail<3>();
    const TVec3 a = md_.G(Xp(2)) * tau_prev_ + md_.w(Vp);
    pred.head<3>() = Xp + Ts * Vp + 0.5 * Ts * Ts * a;
    pred.tail<3>() = Vp + Ts * a;
  }
  TVec6 d = x_hat - pred;
  d(2) = wrap(d(2));
  d_hat_ = (1.0 - tu_.dist_alpha) * d_hat_ + tu_.dist_alpha * d;     // 식-11
}

bool TorqueMPC::solve_qp(const TVec4 & tau_bar)
{
  const int mg = static_cast<int>(A_gen_.rows());
  for (int j = 0; j < tu_.Nc; ++j) {
    for (int i = 0; i < 4; ++i) {
      const double tb = tau_bar(i), dtb = dtau_bar_of(tb);
      double lo = -tb, hi = tb;
      if (j == 0) {     // 첫 스텝의 변화율은 단일 변수라 bound 에 합친다. 상한이 우선 (머리주석)
        lo = std::clamp(tau_prev_(i) - dtb, -tb, tb);
        hi = std::clamp(tau_prev_(i) + dtb, -tb, tb);
      } else {
        lb_(nU_ + 4 * (j - 1) + i) = -dtb;
        ub_(nU_ + 4 * (j - 1) + i) = dtb;
      }
      lb_(4 * j + i) = lo;
      ub_(4 * j + i) = hi;
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

TVec4 TorqueMPC::step(const TVec6 & x_hat, const std::vector<TorqueRefPoint> & ref)
{
  const int Np = tu_.Np, Nc = tu_.Nc;
  if (static_cast<int>(ref.size()) < Np + 1) {throw std::invalid_argument("기준 지평은 Np+1 점이 필요하다");}
  const double Ts = tu_.Ts;
  stats_ = TorqueStepStats{};

  update_disturbance(x_hat);
  const TVec6 d_use = tu_.dist_enable ? d_hat_ : TVec6::Zero();

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
    if (current) {
      // x_{i+1} = A_d x_i + B_d τ + c_d + d̂   — 지평 전체에 같은 (A_d, B_d, c_d)
      xf = lin.A * xf + lin.c + d_use;
      Phi_j = (lin.A * Phi_j).eval();
      Phi_j.block<6, 4>(0, 4 * cidx) += lin.B;
      Phi_.block(6 * i, 0, 6, nU_) = Phi_j;
    } else {
      // 스텝 i 의 기준 방향·속도로 B_i, w_i (식-14)
      const TVec3 & u_i = ref[i].second;
      const double c = std::cos(th_r), s = std::sin(th_r);
      const TVec3 Xdot_r(u_i(0) * c - u_i(1) * s, u_i(0) * s + u_i(1) * c, u_i(2));
      const TVec3 w_i = md_.w(Xdot_r);
      const TMat34 G_i = md_.G(th_r);

      // x_{i+1} = A x_i + B_i τ + E w_i + d̂
      xf.head<3>() += Ts * xf.tail<3>() + 0.5 * Ts * Ts * w_i;
      xf.tail<3>() += Ts * w_i;
      xf += d_use;
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

  const TVec4 tb = tau_bar();
  if (!tu_.constrained) {
    T_ = H_.ldlt().solve(-g_);
  } else if (!solve_qp(tb)) {
    T_ = H_.ldlt().solve(-g_);          // QP 실패 → 무제약 해를 상한으로 깎는다
    stats_.fallback = 1;
    for (int i = 0; i < 4; ++i) {
      const double dtb = dtau_bar_of(tb(i));
      T_(i) = std::clamp(
        T_(i), std::clamp(tau_prev_(i) - dtb, -tb(i), tb(i)), std::clamp(tau_prev_(i) + dtb, -tb(i), tb(i)));
    }
  }
  const TVec4 tau = T_.head<4>();
  x_prev_ = x_hat;
  tau_prev_ = tau;
  has_prev_ = true;
  lin_prev_ = lin;
  has_lin_ = current;
  return tau;
}

}  // namespace amr_mpc_cpp
