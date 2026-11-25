// 이상 플랜트 (test_torque_mpc.cpp 의 Plant = 제어기 모델 그대로, RK4). RESULTS_KO.md 참고
// 빌드 (WS 에서): M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 analysis/torque_mpc_sim/2026-11-21_bias_source/ideal/nctime.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/nctime && /tmp/nctime
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

#include <chrono>
int main()
{
  // Nc 별 step() 시간 (8자 R 0.3 v 0.8, C 상한처럼 낮은 τ̄ 로 제약이 걸리게) — 평균·99 %·최대
  for (int nc : {5, 10, 15, 20, 25}) {
    TorqueModel md; md.use_w = true;
    TorqueTuning tu; tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0; tu.Nc = nc;
    TorqueMPC mpc(md, tu); Plant plant{md};
    mpc.set_tau_bar_override(TVec4(0.1230, 0.1253, 0.1298, 0.1187));
    const double R = 0.3, v = 0.8, om = v / R, T = 2.0 * M_PI / om;
    plant.x << 0, 0, 0, v, 0, om;
    std::vector<double> ms;
    for (int k = 0; k < static_cast<int>(2 * 2 * T / tu.Ts); ++k) {
      const double t = k * tu.Ts;
      std::vector<TorqueRefPoint> ref;
      for (int j = 0; j <= tu.Np; ++j) {ref.push_back(eight(R, v, t + j * tu.Ts));}
      auto t0 = std::chrono::steady_clock::now();
      const TVec4 tau = mpc.step(plant.x, ref, nullptr, tu.Ts);
      ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      plant.step(tau, tu.Ts);
    }
    std::sort(ms.begin(), ms.end());
    double avg = 0; for (double x : ms) avg += x; avg /= ms.size();
    std::printf("Nc %2d: step() 평균 %.2f ms  99%% %.2f ms  최대 %.2f ms\n", nc, avg, ms[static_cast<size_t>(0.99 * ms.size())], ms.back());
  }
}
