#include "amr_mpc_cpp/reference.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace amr_mpc_cpp
{

double wrap_angle(double a) {return std::atan2(std::sin(a), std::cos(a));}

ReferenceHorizon::ReferenceHorizon(double t0, double dt, const std::vector<Row> & rows)
: t0_(t0), dt_(dt)
{
  if (rows.empty() || !(dt > 0.0)) {throw std::invalid_argument("빈 기준열 또는 dt<=0");}
  const size_t T = rows.size();
  times_.resize(T);
  X_.resize(T);
  U_.resize(T);
  for (size_t i = 0; i < T; ++i) {
    times_[i] = t0 + dt * static_cast<double>(i + 1);
    X_[i] << rows[i][0], rows[i][1], rows[i][2];
    U_[i] << rows[i][3], rows[i][4], rows[i][5];
  }
  // np.unwrap(period 2π, discont π) 와 같은 규칙
  double correction = 0.0;
  double prev_raw = X_[0](2);
  for (size_t i = 1; i < T; ++i) {
    const double raw = X_[i](2);
    const double dd = raw - prev_raw;
    double ddmod = std::fmod(dd + M_PI, 2.0 * M_PI);
    if (ddmod < 0.0) {ddmod += 2.0 * M_PI;}
    ddmod -= M_PI;
    if (ddmod == -M_PI && dd > 0.0) {ddmod = M_PI;}
    if (std::abs(dd) >= M_PI) {correction += ddmod - dd;}
    prev_raw = raw;
    X_[i](2) = raw + correction;
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ReferenceHorizon::at(double t) const
{
  Eigen::Vector3d Xr, ur;
  if (t <= times_.front()) {
    // 뒤로 외삽: 상수 twist (−u0) 를 dtb 동안 정확 적분.  V = ∫_0^1 R(sφ) ds
    const Eigen::Vector3d & X0 = X_.front();
    const Eigen::Vector3d & u0 = U_.front();
    const double dtb = times_.front() - t;
    const double phi = -u0(2) * dtb;
    Eigen::Matrix2d V;
    if (std::abs(phi) < 1e-9) {
      V.setIdentity();
    } else {
      V << std::sin(phi) / phi, -(1.0 - std::cos(phi)) / phi,
        (1.0 - std::cos(phi)) / phi, std::sin(phi) / phi;
    }
    const double c = std::cos(X0(2)), s = std::sin(X0(2));
    const Eigen::Vector2d d_body = V * (-u0.head<2>() * dtb);
    Xr << X0(0) + c * d_body(0) - s * d_body(1), X0(1) + s * d_body(0) + c * d_body(1), X0(2) + phi;
    ur = u0;
  } else if (t >= times_.back()) {
    Xr = X_.back();
    ur.setZero();
  } else {
    // searchsorted(side="right") − 1
    const size_t i = static_cast<size_t>(std::upper_bound(times_.begin(), times_.end(), t) - times_.begin()) - 1;
    const double w = (t - times_[i]) / dt_;
    Xr = (1.0 - w) * X_[i] + w * X_[i + 1];
    ur = (1.0 - w) * U_[i] + w * U_[i + 1];
  }
  Xr(2) = wrap_angle(Xr(2));
  return {Xr, ur};
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ReferenceHorizon::horizon(double t, double Ts, int Np) const
{
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> out;
  out.reserve(Np);
  for (int i = 0; i < Np; ++i) {out.push_back(at(t + i * Ts));}
  return out;
}

bool ReferenceHorizon::is_hold() const
{
  double umax = 0.0;
  Eigen::Vector3d lo = X_.front(), hi = X_.front();
  for (size_t i = 0; i < X_.size(); ++i) {
    umax = std::max(umax, U_[i].cwiseAbs().maxCoeff());
    lo = lo.cwiseMin(X_[i]);
    hi = hi.cwiseMax(X_[i]);
  }
  return umax < 1e-9 && (hi - lo).maxCoeff() < 1e-6;
}

HoldLatch::HoldLatch(double enter_xy, double enter_yaw, double exit_xy, double exit_yaw)
: enter_xy_(enter_xy), enter_yaw_(enter_yaw), exit_xy_(exit_xy), exit_yaw_(exit_yaw)
{
  if (!(0 < enter_xy && enter_xy < exit_xy && 0 < enter_yaw && enter_yaw < exit_yaw)) {
    throw std::invalid_argument(
            "hold 문턱은 0 < enter < exit 여야 한다: xy " + std::to_string(enter_xy) + "/" + std::to_string(exit_xy) +
            ", yaw " + std::to_string(enter_yaw) + "/" + std::to_string(exit_yaw));
  }
}

bool HoldLatch::update(bool is_hold, const Eigen::Vector3d & Xe, bool stopped)
{
  if (!is_hold) {
    idle_ = false;
    return false;
  }
  const double d = std::hypot(Xe(0), Xe(1)), a = std::abs(wrap_angle(Xe(2)));
  if (idle_) {
    if (d > exit_xy_ || a > exit_yaw_) {idle_ = false;}
  } else if (stopped && d < enter_xy_ && a < enter_yaw_) {
    idle_ = true;
  }
  return idle_;
}

}  // namespace amr_mpc_cpp
