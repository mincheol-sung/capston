// 이상 플랜트 (test_torque_mpc.cpp 의 Plant = 제어기 모델 그대로, RK4). RESULTS_KO.md 참고
// 빌드 (WS 에서): M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 analysis/torque_mpc_sim/2026-11-21_bias_source/ideal/ideal_ss.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/ideal_ss && /tmp/ideal_ss
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
// 8자 R, 속도 v 일정. 원 위 위상 [p0, p1] 의 e_c (+ 안쪽) 평균, 왼원·오른원
static TorqueRefPoint eight(double R, double v, double t)
{
  const double om = v / R, T = 2.0 * M_PI / om, tt = std::fmod(t, 2.0 * T);
  if (tt < T) {return circle(R, v, tt);}
  const double ph = om * (tt - T);
  return {TVec3(R * std::sin(ph), -R * (1.0 - std::cos(ph)), 2.0 * M_PI - ph), TVec3(v, 0.0, -om)};
}

int main()
{
  // 이상 플랜트 8자 R 0.3 v 0.61: 원 후반 (0.5~0.95) 의 기준 좌표 e_n, e_t 와 MPC 몸체 앞쪽 힘 — 시뮬 B5 와 같은 잣대. Nc 5 / 25, 진행 끌림 0 / −0.03 m/s²
  for (int nc : {5, 25}) for (double dt_ : {0.0, -0.03}) {
    TorqueModel md; md.use_w = true;
    TorqueTuning tu; tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0; tu.Nc = nc;
    TorqueMPC mpc(md, tu); Plant plant{md};
    const double R = 0.3, v = 0.61, om = v / R, T = 2.0 * M_PI / om;
    plant.x << 0, 0, 0, v, 0, om;
    const TMat34 Fm = md.force_map();
    double sn = 0, st = 0, sf = 0; int n = 0;
    const int N = static_cast<int>(4 * 2 * T / tu.Ts);
    for (int k = 0; k < N; ++k) {
      const double t = k * tu.Ts, tt = std::fmod(t, 2.0 * T), th = plant.x(2);
      plant.force_ext << md.m_eff() * dt_ * std::cos(th), md.m_eff() * dt_ * std::sin(th), 0.0;
      std::vector<TorqueRefPoint> ref;
      for (int j = 0; j <= tu.Np; ++j) {ref.push_back(eight(R, v, t + j * tu.Ts));}
      const TVec4 tau = mpc.step(plant.x, ref, nullptr, tu.Ts);
      plant.step(tau, tu.Ts);
      if (t < 2 * 2 * T) continue;
      const double ph = std::fmod(tt, T) / T;
      if (!(ph > 0.5 && ph < 0.95)) continue;
      const auto r = eight(R, v, t + tu.Ts); const double c = std::cos(r.first(2)), s = std::sin(r.first(2)), sg = r.second(2) > 0 ? 1.0 : -1.0;
      const double ex = plant.x(0) - r.first(0), ey = plant.x(1) - r.first(1);
      st += c * ex + s * ey; sn += sg * (-s * ex + c * ey); sf += (Fm * tau)(0); ++n;
    }
    std::printf("Nc %2d  진행 끌림 %+.2f m/s²: 원 후반 e_n %+5.1f  e_t %+5.1f mm   MPC 앞쪽 힘 %+.3f N\n", nc, dt_, 1e3 * sn / n, 1e3 * st / n, sf / n);
  }
}
