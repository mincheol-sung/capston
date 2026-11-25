// 이상 플랜트 (test_torque_mpc.cpp 의 Plant) 8자 R 0.3 에서 (Np, Nc) 하나를 돌린다. 인자: Np Nc 장면(0 = 외란 없음, 1 = 실차 외란 + d̂, 2 = 실차 외란 · d̂ 끔) v
// 빌드 (WS 에서): M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 analysis/torque_mpc_sim/2026-11-23_horizon/grid.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/grid
// 토크 MPC 단위 시험 — 『메카넘 토크 MPC 1판 설계』의 수치 예와 09절 자기검사, 11절 통과시험의 축소판.
#include "amr_mpc_cpp/torque_mpc.hpp"


#include <cmath>
#include <cstdio>
#include <vector>

using namespace amr_mpc_cpp;

namespace
{
// 연속 모델을 RK4 로 적분하는 시험용 플랜트. force_ext: 모델에 없는 관성 좌표 힘 [N, N, N·m]
struct Plant
{
  TorqueModel md;
  TVec6 x = TVec6::Zero();
  TVec3 force_ext = TVec3::Zero();

  TVec6 f(const TVec6 & s, const TVec4 & tau) const
  {
    TVec6 d;
    d.head<3>() = s.tail<3>();
    TVec3 a = md.G(s(2)) * tau + md.w(s.tail<3>());
    a(0) += force_ext(0) / md.m_eff();
    a(1) += force_ext(1) / md.m_eff();
    a(2) += force_ext(2) / md.I_eff();
    d.tail<3>() = a;
    return d;
  }
  void step(const TVec4 & tau, double Ts, int sub = 10)
  {
    const double h = Ts / sub;
    for (int i = 0; i < sub; ++i) {
      const TVec6 k1 = f(x, tau), k2 = f(x + 0.5 * h * k1, tau), k3 = f(x + 0.5 * h * k2, tau), k4 = f(x + h * k3, tau);
      x += h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
    }
  }
};

// 등속 원운동 기준: 반경 R, 속도 v, 시각 t 에 (포즈, 몸체 속도). 반시계, t=0 에 원점에서 +x 를 본다
TorqueRefPoint circle(double R, double v, double t)
{
  const double om = v / R, th = om * t;
  return {TVec3(R * std::sin(th), R * (1.0 - std::cos(th)), th), TVec3(v, 0.0, om)};
}

std::vector<TorqueRefPoint> hold_ref(const TVec3 & X, int n)
{
  return std::vector<TorqueRefPoint>(n, {X, TVec3::Zero()});
}
}  // namespace
#include <chrono>
#include <cstdio>
#include <cstdlib>
static TorqueRefPoint eight(double R, double v, double t)
{
  const double om = v / R, T = 2.0 * M_PI / om, tt = std::fmod(t, 2.0 * T);
  if (tt < T) {return circle(R, v, tt);}
  const double ph = om * (tt - T);
  return {TVec3(R * std::sin(ph), -R * (1.0 - std::cos(ph)), 2.0 * M_PI - ph), TVec3(v, 0.0, -om)};
}
int main(int argc, char ** argv)
{
  const int Np = std::atoi(argv[1]), Nc = std::atoi(argv[2]), scen = std::atoi(argv[3]); const double v = std::atof(argv[4]);
  TorqueModel md; md.use_w = true;
  TorqueTuning tu; tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0; tu.Np = Np; tu.Nc = Nc;
  if (scen == 1) {   // yaml 의 d̂: 몸체 · α 0.01 · accel_only · 포화 중 멈춤 · 상한 (잡음 없는 플랜트라 skip_xy 끔)
    tu.dist_enable = true; tu.dist_body_frame = true; tu.dist_alpha = 0.01; tu.dist_accel_only = true; tu.dist_freeze_on_sat = true;
    tu.dist_acc_max = 0.5; tu.dist_yaw_acc_max = 2.0; tu.dist_skip_xy = -1.0;
  }
  TorqueMPC mpc(md, tu); Plant plant{md};
  const TVec4 tb(0.1445, 0.1184, 0.1444, 0.1859);              // C 상한 (2026-11-22 (4) 바퀴별 μ̂ · 하중 몫)
  mpc.set_tau_bar_override(tb);
  const double R = 0.3, om = v / R, T = 2.0 * M_PI / om, dxb = -0.155, dyb = 0.085;   // 실차 잔차 d_x · d_y (RESULTS 20 절)
  plant.x << 0, 0, 0, v, 0, om;
  const int N = static_cast<int>(5 * 2 * T / tu.Ts);
  double s_l = 0, s_r = 0, sq = 0, sqt = 0, sdt = 0, tmax = 0; int n_l = 0, n_r = 0, n = 0, nsat = 0;
  std::vector<double> ms; TVec4 tprev = TVec4::Zero();
  for (int k = 0; k < N; ++k) {
    const double t = k * tu.Ts, tt = std::fmod(t, 2.0 * T); const bool left = tt < T; const double cy = left ? R : -R, th = plant.x(2);
    if (scen >= 1) {plant.force_ext << md.m_eff() * (dxb * std::cos(th) - dyb * std::sin(th)), md.m_eff() * (dxb * std::sin(th) + dyb * std::cos(th)), 0.0;}
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= Np; ++j) {ref.push_back(eight(R, v, t + j * tu.Ts));}
    auto t0 = std::chrono::steady_clock::now();
    const TVec4 tau = mpc.step(plant.x, ref, nullptr, tu.Ts);
    ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    plant.step(tau, tu.Ts);
    if (t < 2 * 2 * T) {tprev = tau; continue;}                  // 앞 2 랩 버림
    const double e = R - std::hypot(plant.x(0), plant.x(1) - cy);   // + 안쪽
    const auto r = eight(R, v, t + tu.Ts);
    if (left) {s_l += e; ++n_l;} else {s_r += e; ++n_r;}
    sq += e * e; sqt += std::pow(plant.x(0) - r.first(0), 2) + std::pow(plant.x(1) - r.first(1), 2);
    sdt += (tau - tprev).squaredNorm() / 4.0; tprev = tau; tmax = std::max(tmax, tau.cwiseAbs().maxCoeff());
    nsat += (tau.cwiseAbs().array() >= 0.98 * tb.array()).any(); ++n;
  }
  std::sort(ms.begin(), ms.end()); double avg = 0; for (double x : ms) {avg += x;} avg /= ms.size();
  std::printf("%d %d %d %.2f  s_l %+.2f s_r %+.2f  pathRMS %.2f  timeRMS %.2f  dtau %.5f  tmax %.3f  sat %.1f  ms %.3f %.3f %.3f\n", Np, Nc, scen, v,
    1e3 * s_l / n_l, 1e3 * s_r / n_r, 1e3 * std::sqrt(sq / n), 1e3 * std::sqrt(sqt / n), std::sqrt(sdt / n), tmax, 100.0 * nsat / n,
    avg, ms[static_cast<size_t>(0.99 * ms.size())], ms.back());
}
