// ρ 결합 Kalman 수렴 속도 (q_xy × q_η) — 시뮬 플랜트 (test_torque_mpc.cpp 의 Plant), 8자 R 0.5·v 0.5, 안쪽 힘 β 0.3 + 몸체 왼쪽 0.09 m/s², 6 랩
// 빌드: M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 rho_convergence.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/rc && /tmp/rc
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
#include <cstdio>
static TorqueRefPoint eight(double R, double v, double t)
{
  const double om = v / R, T = 2.0 * M_PI / om, tt = std::fmod(t, 2.0 * T);
  if (tt < T) {return circle(R, v, tt);}
  const double ph = om * (tt - T);
  return {TVec3(R * std::sin(ph), -R * (1.0 - std::cos(ph)), 2.0 * M_PI - ph), TVec3(v, 0.0, -om)};
}
int main()
{
  for (double qxy : {0.3, 0.1, 0.05}) for (double qe : {0.03, 0.1}) {
    TorqueModel md; TorqueTuning tu;
    tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0; tu.dist_enable = true; tu.dist_kalman = true;
    tu.dist_body_frame = true; tu.dist_skip_xy = -1.0; tu.dist_acc_max = 0.5; tu.ff_normal_estimate = true;
    tu.dist_kq = TVec3(qxy, qxy, 0.8); tu.ff_normal_q = qe;
    TorqueMPC mpc(md, tu); Plant plant{md};
    const double R = 0.5, v = 0.5, om = v / R, T = 2.0 * M_PI / om, beta = 0.3, d_b = 0.09;
    plant.x << 0, 0, 0, v, 0, om;
    std::printf("q_xy %.2f  q_eta %.2f  rho(랩 끝) :", qxy, qe);
    double emax = 0; 
    const int N = static_cast<int>(6 * 2 * T / tu.Ts);
    for (int k = 0; k < N; ++k) {
      const double t = k * tu.Ts, tt = std::fmod(t, 2.0 * T); const bool left = tt < T;
      const double cy = left ? R : -R, dx = -plant.x(0), dy = cy - plant.x(1), dn = std::hypot(dx, dy);
      const double Fin = beta * md.m_eff() * v * v / R, Fd = md.m_eff() * d_b, th = plant.x(2);
      plant.force_ext << Fin * dx / dn - Fd * std::sin(th), Fin * dy / dn + Fd * std::cos(th), 0.0;
      std::vector<TorqueRefPoint> ref;
      for (int j = 0; j <= tu.Np; ++j) {ref.push_back(eight(R, v, t + j * tu.Ts));}
      plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);
      const double ph = std::fmod(tt, T) / T;
      if (t > 10 * T && ph > 0.25 && ph < 0.9) emax = std::max(emax, std::abs(std::hypot(plant.x(0), plant.x(1) - cy) - R));
      if ((k + 1) % static_cast<int>(2 * T / tu.Ts) == 0) std::printf(" %.3f", mpc.ff_normal_rho());
    }
    std::printf("   마지막 랩 원 위 |반경 어긋남| 최대 %.1f mm\n", emax * 1e3);
  }
}
