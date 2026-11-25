// 이상 플랜트 (test_torque_mpc.cpp 의 Plant = 제어기 모델 그대로, RK4). RESULTS_KO.md 참고
// 빌드 (WS 에서): M=src/amr_basic_stack/controller/mpc_cpp; g++ -O2 -std=c++17 -I$M/include -I/usr/include/eigen3 analysis/torque_mpc_sim/2026-11-21_bias_source/ideal/stiff.cpp build/amr_mpc_cpp/libamr_torque_mpc.a build/amr_mpc_cpp/libdaqp_vendored.a -o /tmp/stiff && /tmp/stiff
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

// 원 하나 (R 0.3, 반시계), 몸체 일정 외란 δ = [δ_t (몸체 x), δ_n (몸체 y = 안쪽)] [m/s²]. 정상상태 e_n (+ 안쪽), e_t (+ 앞)
static void run(double v, double dt_, double dn_, double & en, double & et)
{
  TorqueModel md; md.use_w = true;
  TorqueTuning tu; tu.W << 20, 20, 10, 2, 2, 1; tu.w_delta = 100.0; tu.r_u = 10.0;
  TorqueMPC mpc(md, tu); Plant plant{md};
  const double R = 0.3, om = v / R;
  plant.x << 0, 0, 0, v, 0, om;
  double sn = 0, st = 0; int n = 0;
  for (int k = 0; k < 1500; ++k) {
    const double t = k * tu.Ts, th = plant.x(2);
    plant.force_ext << md.m_eff() * (dt_ * std::cos(th) - dn_ * std::sin(th)), md.m_eff() * (dt_ * std::sin(th) + dn_ * std::cos(th)), 0.0;
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, t + j * tu.Ts));}
    plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);
    if (k >= 1000) {
      const auto r = circle(R, v, t + tu.Ts); const double c = std::cos(r.first(2)), s = std::sin(r.first(2));
      const double ex = plant.x(0) - r.first(0), ey = plant.x(1) - r.first(1);
      st += c * ex + s * ey; sn += -s * ex + c * ey; ++n;                 // 기준 몸체 좌표: x = 앞, y = 왼쪽 = 안쪽
    }
  }
  en = sn / n; et = st / n;
}
int main()
{
  for (double v : {0.30, 0.43, 0.61, 0.80}) {
    const double om = v / 0.3; double en0, et0, en1, et1, en2, et2;
    run(v, 0.0, 0.0, en0, et0); run(v, -0.05, 0.0, en1, et1); run(v, 0.0, 0.05, en2, et2);
    // 외란 몫만 (0 외란 판 빼기). 진행 끌림 δ_t: e_n = −b δ_t/(a²+b²), e_t = a δ_t/(a²+b²);  안쪽 힘 δ_n: e_n = a δ_n/(a²+b²), e_t = b δ_n/(a²+b²)
    const double En1 = en1 - en0, Et1 = et1 - et0, En2 = en2 - en0, Et2 = et2 - et0;
    // 법선 힘 판에서 a, b 역산: a/(a²+b²) = En2/δ_n, b/(a²+b²) = Et2/δ_n
    const double p = En2 / 0.05, q = Et2 / 0.05, den = p * p + q * q, a = p / den, b = q / den;
    std::printf("v %.2f ω %.2f ω² %.2f | 끌림 δ_t −0.05: Δe_n %+6.1f Δe_t %+6.1f mm | 안쪽 δ_n +0.05: Δe_n %+6.1f Δe_t %+6.1f mm | 역산 a = k_p−ω² %+.2f → k_p %.2f, b = k_dω %.2f → k_d %.2f | 끌림 판 예측 Δe_n %+6.1f Δe_t %+6.1f\n",
      v, om, om * om, 1e3 * En1, 1e3 * Et1, 1e3 * En2, 1e3 * Et2, a, a + om * om, b, b / om,
      1e3 * (-b * -0.05 / (a * a + b * b)), 1e3 * (a * -0.05 / (a * a + b * b)));
  }
}
