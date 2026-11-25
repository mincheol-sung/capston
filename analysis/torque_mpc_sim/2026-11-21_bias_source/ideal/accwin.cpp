// 이상 플랜트 (test_torque_mpc.cpp 의 Plant = 제어기 모델 그대로, RK4). RESULTS_KO.md 참고
// 빌드 (WS 에서): M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 analysis/torque_mpc_sim/2026-11-21_bias_source/ideal/accwin.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/accwin && /tmp/accwin
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

static TVec3 inertial_vel(const TorqueRefPoint & p)
{
  const double c = std::cos(p.first(2)), s = std::sin(p.first(2));
  return TVec3(p.second(0) * c - p.second(1) * s, p.second(0) * s + p.second(1) * c, p.second(2));
}
int main()
{
  // 기준 가속도: nullptr (지평 차분, 전환이 계단) vs 노드처럼 창 acc_dt 중앙 차분. 8자 R 0.3, Nc 5, use_w 1
  for (double adt : {0.0, 0.05, 0.1, 0.2}) for (double v : {0.43, 0.61, 0.80}) {
    TorqueModel md; md.use_w = true;
    TorqueTuning tu; tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0;
    TorqueMPC mpc(md, tu); Plant plant{md};
    const double R = 0.3, om = v / R, T = 2.0 * M_PI / om;
    plant.x << 0, 0, 0, v, 0, om;
    double s0 = 0, s1 = 0; int n0 = 0, n1 = 0; double bins[10] = {0}; int cnt[10] = {0};
    const int N = static_cast<int>(4 * 2 * T / tu.Ts);
    std::vector<TVec3> acc(tu.Nc);
    for (int k = 0; k < N; ++k) {
      const double t = k * tu.Ts, tt = std::fmod(t, 2.0 * T); const bool left = tt < T; const double cy = left ? R : -R;
      std::vector<TorqueRefPoint> ref;
      for (int j = 0; j <= tu.Np; ++j) {ref.push_back(eight(R, v, t + j * tu.Ts));}
      for (int j = 0; j < tu.Nc; ++j) {
        const double tj = t + j * tu.Ts;
        acc[j] = adt > 0 ? TVec3((inertial_vel(eight(R, v, tj + 0.5 * adt)) - inertial_vel(eight(R, v, tj - 0.5 * adt))) / adt) : TVec3::Zero();
      }
      plant.step(mpc.step(plant.x, ref, adt > 0 ? &acc : nullptr, tu.Ts), tu.Ts);
      if (t < 2 * 2 * T) continue;
      const double ph = std::fmod(tt, T) / T, e = R - std::hypot(plant.x(0), plant.x(1) - cy);
      s0 += e; ++n0; if (ph > 0.5 && ph < 0.95) {s1 += e; ++n1;}
      const int bi = std::min(9, static_cast<int>(ph * 10)); bins[bi] += e; ++cnt[bi];
    }
    std::printf("acc 창 %.2f s  v %.2f:  e_c (+안쪽) 전체 %+5.1f  원 후반 %+5.1f mm   위상:", adt, v, 1e3 * s0 / n0, 1e3 * s1 / n1);
    for (int b = 0; b < 10; ++b) {std::printf(" %+5.1f", 1e3 * bins[b] / cnt[b]);}
    std::printf("\n");
  }
}
