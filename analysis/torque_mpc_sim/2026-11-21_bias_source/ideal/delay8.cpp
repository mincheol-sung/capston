// 이상 플랜트 (test_torque_mpc.cpp 의 Plant = 제어기 모델 그대로, RK4). RESULTS_KO.md 참고
// 빌드 (WS 에서): M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 analysis/torque_mpc_sim/2026-11-21_bias_source/ideal/delay8.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/delay8 && /tmp/delay8
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

#include <deque>
int main()
{
  // 이상 플랜트에 입력 지연 (τ 를 n 스텝 늦게) · 상태 지연 (MPC 가 n 스텝 전 상태를 받음). 8자·원 하나, R 0.3 v 0.61, Nc 5, use_w 1
  for (int eightmode : {0, 1}) for (int kind : {0, 1}) for (int nd : {0, 1, 2, 4}) {
    if (nd == 0 && kind == 1) continue;
    TorqueModel md; md.use_w = true;
    TorqueTuning tu; tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0;
    TorqueMPC mpc(md, tu); Plant plant{md};
    const double R = 0.3, v = 0.61, om = v / R, T = 2.0 * M_PI / om;
    plant.x << 0, 0, 0, v, 0, om;
    std::deque<TVec4> tq(nd, TVec4::Zero()); std::deque<TVec6> xq(nd, plant.x);
    double s0 = 0, s1 = 0; int n0 = 0, n1 = 0;
    const int N = static_cast<int>(4 * 2 * T / tu.Ts);
    for (int k = 0; k < N; ++k) {
      const double t = k * tu.Ts, tt = std::fmod(t, 2.0 * T); const bool left = eightmode ? tt < T : true; const double cy = left ? R : -R;
      std::vector<TorqueRefPoint> ref;
      for (int j = 0; j <= tu.Np; ++j) {ref.push_back(eightmode ? eight(R, v, t + j * tu.Ts) : circle(R, v, t + j * tu.Ts));}
      TVec6 xm = plant.x;
      if (kind == 1) {xq.push_back(plant.x); xm = xq.front(); xq.pop_front();}
      TVec4 tau = mpc.step(xm, ref, nullptr, tu.Ts);
      if (kind == 0) {tq.push_back(tau); tau = tq.front(); tq.pop_front();}
      plant.step(tau, tu.Ts);
      if (t < 2 * 2 * T) continue;
      const double ph = std::fmod(tt, T) / T, e = R - std::hypot(plant.x(0), plant.x(1) - cy);
      s0 += e; ++n0; if (ph > 0.5 && ph < 0.95) {s1 += e; ++n1;}
    }
    std::printf("%s %s 지연 %d 스텝 (%2.0f ms):  e_c (+안쪽) 전체 %+6.1f mm   원 후반 (0.5~0.95) %+6.1f mm\n", eightmode ? "8자   " : "원 하나", kind ? "상태" : "입력",
      nd, nd * 10.0, 1e3 * s0 / n0, 1e3 * s1 / n1);
  }
}
