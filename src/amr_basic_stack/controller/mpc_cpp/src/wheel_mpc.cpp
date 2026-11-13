// 구조 B 바퀴 MPC — 식의 근거는 wheel_mpc.hpp 머리주석과 amr_mpc/wheel/{model,mpc}.py.
#include "amr_mpc_cpp/wheel_mpc.hpp"

extern "C" {
#include "api.h"
}

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace amr_mpc_cpp
{

namespace
{
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
}  // namespace

Mecanum mecanum_matrices(double wheel_radius, double wheel_base, double robot_width)
{
  Mecanum m;
  m.k = (wheel_base + robot_width) / 2.0;
  m.T_inv << 1.0, -1.0, -m.k,    // FL
    1.0, 1.0, m.k,               // FR
    1.0, 1.0, -m.k,              // RL
    1.0, -1.0, m.k;              // RR
  m.T_inv /= wheel_radius;
  // T⁻¹ 는 열 rank 3 이라 pinv = (T⁻ᵀT⁻¹)⁻¹T⁻ᵀ
  m.T_fwd = (m.T_inv.transpose() * m.T_inv).inverse() * m.T_inv.transpose();
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 4>> svd(m.T_fwd, Eigen::ComputeFullV);
  m.null_dir = svd.matrixV().col(3);
  return m;
}

void kinematic_jacobians(double theta_r, double vxr, double vyr, double Ts, Mat3 & A_k, Mat3 & B_k3)
{
  const double c = std::cos(theta_r), s = std::sin(theta_r);
  const double Xr_dot = vxr * c - vyr * s, Yr_dot = vxr * s + vyr * c;
  A_k.setIdentity();
  A_k(0, 2) = -Ts * Yr_dot;
  A_k(1, 2) = Ts * Xr_dot;
  B_k3 << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  B_k3 *= Ts;
}

Vec3 body_kinematics(const Vec3 & X, const Vec3 & v)
{
  const double c = std::cos(X(2)), s = std::sin(X(2));
  return Vec3(v(0) * c - v(1) * s, v(0) * s + v(1) * c, v(2));
}

Vec4 clip_wheels(const Vec4 & w, double w_max)
{
  const double m = w.cwiseAbs().maxCoeff();
  return m > w_max ? Vec4(w * (w_max / m)) : w;
}

Vec3 extrapolate_pose(const Vec3 & X, const Vec3 & v_body, double dt)
{
  const double phi = v_body(2) * dt;
  Eigen::Matrix2d V;                    // ∫_0^1 R(sφ) ds
  if (std::abs(phi) < 1e-9) {
    V.setIdentity();
  } else {
    V << std::sin(phi) / phi, -(1.0 - std::cos(phi)) / phi, (1.0 - std::cos(phi)) / phi, std::sin(phi) / phi;
  }
  const Eigen::Vector2d d = V * (v_body.head<2>() * dt);   // 시작 몸체 프레임에서의 변위
  const double c = std::cos(X(2)), s = std::sin(X(2));
  return Vec3(X(0) + c * d(0) - s * d(1), X(1) + s * d(0) + c * d(1), wrap(X(2) + phi));
}

// ---------------- 관측기 ----------------
VelocityObserver::VelocityObserver(const Vec4 & a, const Vec4 & b, int d, double K)
: a_(a), b_(b), d_(d), K_(K) {}

void VelocityObserver::reset()
{
  has_vhat_ = false;
  u_hist_.clear();
}

void VelocityObserver::push(const Vec4 & u)
{
  u_hist_.push_back(u);
  while (static_cast<int>(u_hist_.size()) > d_ + 1) {u_hist_.pop_front();}
}

Vec4 VelocityObserver::update(const Vec4 & v_meas, bool fresh)
{
  // 기동 구간: 지연만큼의 명령 이력이 없으면 측정을 그대로 믿는다 (observer.py 와 같다)
  if (!has_vhat_ || static_cast<int>(u_hist_.size()) < d_ + 1) {
    vhat_ = v_meas;
    has_vhat_ = true;
    return vhat_;
  }
  vhat_ = a_.cwiseProduct(vhat_) + b_.cwiseProduct(u_hist_.front());   // front = u(k−1−d)
  if (fresh) {vhat_ += K_ * (v_meas - vhat_);}
  return vhat_;
}

// ---------------- MPC ----------------
WheelMPC::WheelMPC(const WheelModel & model, const WheelTuning & tuning)
: md_(model), tu_(tuning)
{
  if (tu_.Ts <= 0.0 || tu_.Np < 1 || tu_.Nc < 1 || tu_.Nc > tu_.Np) {
    throw std::invalid_argument("Ts > 0, 1 ≤ Nc ≤ Np 이어야 한다");
  }
  const double dr = md_.theta / tu_.Ts;
  d_ = static_cast<int>(std::lround(dr));
  if (md_.theta < 0.0 || std::abs(dr - d_) > 1e-9) {
    throw std::invalid_argument(
            "theta/Ts = " + std::to_string(dr) + " 가 0 이상의 정수가 아니다 (theta 를 Ts 의 정수배로)");
  }
  mec_ = mecanum_matrices(md_.wheel_radius, md_.wheel_base, md_.robot_width);
  for (int i = 0; i < 4; ++i) {
    a_(i) = std::exp(-tu_.Ts / md_.tau(i));
    b_(i) = md_.k_dc(i) * (1.0 - a_(i));
  }
  dw_max_ = md_.alpha_max * tu_.Ts;
  n_m_ = 3 + 4 + 4 * d_;
  n_ = n_m_ + p_;
  nU_ = nu_ * tu_.Nc;
  obs_ = VelocityObserver(a_, b_, d_, tu_.k_obs);
  build_constant_parts();
  reset();
}

void WheelMPC::reset()
{
  has_xm_prev_ = false;
  obs_.reset();
  cmd_hist_.assign(d_, Vec4::Zero());
  stats_ = StepStats{};
}

void WheelMPC::plant(double theta_r, double vxr, double vyr, MatX & A_m, MatX & B_m, MatX & C_m) const
{
  Mat3 A_k, B_k3;
  kinematic_jacobians(theta_r, vxr, vyr, tu_.Ts, A_k, B_k3);
  A_m = MatX::Zero(n_m_, n_m_);
  B_m = MatX::Zero(n_m_, nu_);
  C_m = MatX::Zero(p_, n_m_);
  A_m.block<3, 3>(0, 0) = A_k;
  A_m.block<3, 4>(0, 3) = B_k3 * mec_.T_fwd;
  A_m.block<4, 4>(3, 3) = a_.asDiagonal();
  if (d_ == 0) {
    B_m.block<4, 4>(3, 0) = b_.asDiagonal();
  } else {
    A_m.block<4, 4>(3, 7 + 4 * (d_ - 1)) = b_.asDiagonal();          // ω_e ← b′ z_d
    for (int j = 1; j < d_; ++j) {
      A_m.block<4, 4>(7 + 4 * j, 7 + 4 * (j - 1)).setIdentity();      // z_{j+1} ← z_j
    }
    B_m.block<4, 4>(7, 0).setIdentity();                              // z_1 ← ω_e,cmd
  }
  C_m.block<3, 3>(0, 0).setIdentity();
}

void WheelMPC::augment(double theta_r, double vxr, double vyr, MatX & A, MatX & B, MatX & C) const
{
  MatX A_m, B_m, C_m;
  plant(theta_r, vxr, vyr, A_m, B_m, C_m);
  A = MatX::Zero(n_, n_);
  B = MatX::Zero(n_, nu_);
  C = MatX::Zero(p_, n_);
  A.topLeftCorner(n_m_, n_m_) = A_m;
  A.bottomLeftCorner(p_, n_m_) = C_m * A_m;
  A.bottomRightCorner(p_, p_).setIdentity();
  B.topRows(n_m_) = B_m;
  B.bottomRows(p_) = C_m * B_m;
  C.rightCols(p_).setIdentity();
}

void WheelMPC::build_constant_parts()
{
  const int Np = tu_.Np, Nc = tu_.Nc;
  G_.resize(n_, n_ + nU_);
  tmpX_.resize(3, n_ + nU_);
  F_free_.resize(p_ * Np, n_);
  Phi_.resize(p_ * Np, nU_);
  Ph_.resize(p_ * Np, nU_);

  qbar_.resize(p_ * Np);
  for (int i = 0; i < Np; ++i) {qbar_.segment<3>(3 * i) << tu_.q_pos, tu_.q_pos, tu_.q_yaw;}
  qsqrt_ = qbar_.cwiseSqrt();

  // null-space 벌점: N_j = nᵀ S_{min(j,Nc−1)},  S_j 는 Σ_{l≤j} Δω_l 을 뽑는다
  const Vec4 n_dir = mec_.null_dir / mec_.null_dir.norm();
  MatX N = MatX::Zero(Np, nU_);
  for (int j = 0; j < Np; ++j) {
    const int jj = std::min(j, Nc - 1);
    for (int l = 0; l <= jj; ++l) {N.block<1, 4>(j, 4 * l) = n_dir.transpose();}
  }
  E_const_ = 2.0 * tu_.r_dw * MatX::Identity(nU_, nU_) + 2.0 * tu_.q_null * (N.transpose() * N);
  F_null_ = -2.0 * tu_.q_null * (N.transpose() * VecX::Ones(Np));

  // j ≥ 1 바퀴속도 box: ω_i(k+j) = ω_prev,i − Σ_{l≤j} η_{l,i}
  A_gen_.setZero(4 * (Nc - 1), nU_);
  for (int j = 1; j < Nc; ++j) {
    for (int i = 0; i < 4; ++i) {
      for (int l = 0; l <= j; ++l) {A_gen_(4 * (j - 1) + i, 4 * l + i) = 1.0;}
    }
  }
  E_.resize(nU_, nU_);
  F_.resize(nU_);
  eta_.resize(nU_);
  lam_.resize(nU_ + A_gen_.rows());
  lb_.resize(nU_ + A_gen_.rows());
  ub_.resize(nU_ + A_gen_.rows());
}

// G 의 왼쪽 ncols 열에 증강 A 를 곱한다. A 의 0 이 아닌 블록만 쓴다 (dense 26×26 곱 대비 O(n) 배 적다):
//   X ← A_k X + B_k W,   W ← a∘W + b∘Z_d (d=0 이면 a∘W),   Z_j ← Z_{j−1}, Z_1 ← 0,   Y ← Y + (새 X)
// 마지막 식은 증강 y 행이 [C_m A_m, I] 이고 C_m A_m = A_m 의 위 3행 = [A_k, B_k, 0] 이기 때문이다.
void WheelMPC::apply_A(const Mat3 & A_k, const Eigen::Matrix<double, 3, 4> & B_k, int ncols)
{
  auto G = G_.leftCols(ncols);
  auto X = tmpX_.leftCols(ncols);
  X.noalias() = A_k * G.topRows<3>();
  X.noalias() += B_k * G.middleRows<4>(3);
  if (d_ == 0) {
    G.middleRows<4>(3) = a_.asDiagonal() * G.middleRows<4>(3);
  } else {
    G.middleRows<4>(3) = a_.asDiagonal() * G.middleRows<4>(3) + b_.asDiagonal() * G.middleRows<4>(7 + 4 * (d_ - 1));
    for (int j = d_ - 1; j >= 1; --j) {G.middleRows<4>(7 + 4 * j) = G.middleRows<4>(7 + 4 * (j - 1));}
    G.middleRows<4>(7).setZero();
  }
  G.topRows<3>() = X;
  G.bottomRows<3>() += X;
}

// LTV 누적곱 (Python _F_Phi 와 같은 결과).  C = [0 I] 라 C·(·) 는 마지막 p 행.
// G 의 왼쪽 n 열이 A_i⋯A_0 (→ F_free), 그 뒤 4 열씩이 Φ 의 열 블록 cols_j.
// cols_j 는 j 번째 스텝에 B 가 더해지기 전까지 0 이라 곱하지 않는다.
void WheelMPC::predict(const std::vector<RefPoint> & ref, MatX & F_free, MatX & Phi)
{
  const int Np = tu_.Np, Nc = tu_.Nc;
  if (ref.empty()) {throw std::invalid_argument("빈 지평 기준");}
  G_.setZero();
  G_.leftCols(n_).setIdentity();
  F_free.resize(p_ * Np, n_);
  Phi.resize(p_ * Np, nU_);
  Mat3 A_k, B_k3;
  for (int i = 0; i < Np; ++i) {
    const RefPoint & r = ref[std::min<size_t>(i, ref.size() - 1)];
    kinematic_jacobians(r(0), r(1), r(2), tu_.Ts, A_k, B_k3);
    const Eigen::Matrix<double, 3, 4> B_k = B_k3 * mec_.T_fwd;
    apply_A(A_k, B_k, n_ + nu_ * std::min(i, Nc));
    // Nc 이후에도 마지막 증분을 매 스텝 다시 적용한다 (u 가 램프). Wang Ch.1.3 의 Δu = 0 과 다른 의도적 이탈이고
    // 근거(명령 부호반전 40.9 → 30.2 %)는 amr_mpc/body/mpc.py:202. Python 판과 같게 둔다.
    // 주의: null 벌점 N_j = nᵀS_{min(j,Nc−1)} 은 Nc 이후 Δω = 0 을 가정한다 — Python 판의 같은 불일치를 그대로 옮겼다.
    const int c = n_ + nu_ * std::min(i, Nc - 1);
    if (d_ == 0) {
      G_.block<4, 4>(3, c).diagonal() += b_;          // B_m = [0; b′]
    } else {
      G_.block<4, 4>(7, c).diagonal().array() += 1.0; // B_m = [0; 0; I; 0 …]  (z_1 ← ω_e,cmd)
    }
    F_free.middleRows(p_ * i, p_) = G_.block(n_m_, 0, p_, n_);
    Phi.middleRows(p_ * i, p_) = G_.block(n_m_, n_, p_, nU_);
  }
}

bool WheelMPC::solve_qp(const MatX & E, const VecX & F, const Vec4 & w_prev, VecX & eta)
{
  const int mg = static_cast<int>(A_gen_.rows());
  const double W = md_.w_max, dw = dw_max_;
  for (int l = 0; l < tu_.Nc; ++l) {
    for (int i = 0; i < 4; ++i) {
      double lo = -dw, hi = dw;
      if (l == 0) {               // j = 0 바퀴속도 box 는 단일 변수라 bound 에 합친다
        lo = std::max(lo, w_prev(i) - W);
        hi = std::min(hi, w_prev(i) + W);
      }
      lb_(4 * l + i) = lo;
      ub_(4 * l + i) = hi;
    }
  }
  for (int j = 1; j < tu_.Nc; ++j) {
    for (int i = 0; i < 4; ++i) {
      lb_(nU_ + 4 * (j - 1) + i) = w_prev(i) - W;
      ub_(nU_ + 4 * (j - 1) + i) = w_prev(i) + W;
    }
  }
  DAQPProblem qp{};
  qp.n = nU_;
  qp.m = nU_ + mg;
  qp.ms = nU_;
  qp.H = const_cast<double *>(E.data());          // 대칭이라 저장 순서 무관
  qp.f = const_cast<double *>(F.data());
  qp.A = mg > 0 ? A_gen_.data() : nullptr;
  qp.bupper = ub_.data();
  qp.blower = lb_.data();
  qp.sense = nullptr;
  DAQPSettings settings;
  daqp_default_settings(&settings);
  settings.iter_limit = tu_.daqp_iter_limit;
  DAQPResult res{};
  res.x = eta.data();
  res.lam = lam_.data();
  daqp_quadprog(&res, &qp, &settings);
  stats_.exitflag = res.exitflag;
  stats_.iters = res.iter;
  stats_.active = static_cast<int>((lam_.array().abs() > 1e-12).count());
  return res.exitflag >= 1;
}

Vec4 WheelMPC::step(
  const Vec3 & X, const Vec4 & w_meas, const Vec4 & w_prev,
  const std::vector<RefPoint> & ref, const Vec3 & Xr, const Vec3 & ur, bool w_fresh)
{
  const Vec4 w_r = mec_.T_inv * ur;
  Vec3 Xe = Xr - X;
  Xe(2) = wrap(Xe(2));
  // 측정을 그대로 쓰면 아래 Δx_m 이 그걸 차분해 잡음을 √2 배로 키운다 (observer.py 머리주석)
  const Vec4 we = w_r - obs_.update(w_meas, w_fresh);
  VecX xm(n_m_);
  xm << Xe, we;
  for (int j = 1; j <= d_; ++j) {
    const Vec4 & u_past = (j == 1) ? w_prev : cmd_hist_[j - 1];     // z_j = ω_r − ω_cmd(k−j)
    xm.segment<4>(3 + 4 * j) = w_r - u_past;
  }
  if (!has_xm_prev_) {
    xm_prev_ = xm;
    has_xm_prev_ = true;
  }
  VecX x(n_);
  x << xm - xm_prev_, Xe;
  xm_prev_ = xm;

  predict(ref, F_free_, Phi_);
  // E = 2ΦᵀQ̄Φ + 2R̄ + E_null.  대칭이라 √Q̄Φ 의 rankUpdate 로 아래 삼각만 만들고 위를 채운다
  Ph_.noalias() = qsqrt_.asDiagonal() * Phi_;
  E_.setZero();
  E_.selfadjointView<Eigen::Lower>().rankUpdate(Ph_.transpose(), 2.0);
  E_.triangularView<Eigen::StrictlyUpper>() = E_.transpose();
  E_ += E_const_;
  F_.noalias() = 2.0 * (Ph_.transpose() * qsqrt_.cwiseProduct(F_free_ * x));
  F_ += F_null_ * (mec_.null_dir.normalized().dot(w_prev));

  stats_.fallback = 0;
  if (!solve_qp(E_, F_, w_prev, eta_)) {
    eta_ = E_.ldlt().solve(-F_);        // QP 실패 → 무제약 해를 바퀴 한계로 깎는다
    stats_.fallback = 1;
  }
  const Vec4 w = w_prev - eta_.head<4>();
  const Vec4 w_c = clip_wheels(w, md_.w_max);
  // np.allclose(w, w_c) 와 같은 판정 (rtol 1e-5, atol 1e-8)
  if (((w - w_c).cwiseAbs().array() > 1e-8 + 1e-5 * w_c.cwiseAbs().array()).any()) {stats_.fallback = 1;}
  obs_.push(w_c);
  if (d_ > 0) {
    cmd_hist_.push_front(w_c);
    cmd_hist_.pop_back();
  }
  return w_c;
}

}  // namespace amr_mpc_cpp
