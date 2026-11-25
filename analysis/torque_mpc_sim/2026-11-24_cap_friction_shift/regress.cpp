// 수정 전·후 회귀 비교용 드라이버 (2026-11-24). 예전 헤더에 있는 것만 쓴다 — 예전·새 라이브러리 양쪽에 그대로 링크된다.
//   출력: 판 이름, 스텝, τ×4 (%.17g). 두 빌드의 출력을 수치로 견준다 (compare.py).
#include "amr_mpc_cpp/torque_mpc.hpp"

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace amr_mpc_cpp;

static const double kPi = 3.14159265358979323846;

// 기준: 월드 x 축 위를 v(t) = sin(2π t / 2.4) 로 왕복 (최대 가속 2.62 m/s² — C 상한 밖, 속도 부호가 바뀐다), 헤딩 0.3 sin(1.3 t)
static TorqueRefPoint ref_at(double t)
{
  const double w = 2.0 * kPi / 2.4;
  const double vx = std::sin(w * t), x = (1.0 - std::cos(w * t)) / w;
  const double th = 0.3 * std::sin(1.3 * t), thd = 0.39 * std::cos(1.3 * t);
  return {TVec3(x, 0.0, th), TVec3(vx * std::cos(th), -vx * std::sin(th), thd)};
}

static TorqueModel real_model()
{
  TorqueModel md;
  md.wheel_radius = 0.04; md.wheel_base = 0.1724; md.robot_width = 0.2140;
  md.mass = 4.56; md.inertia_z = 0.06; md.J_w = 8.9e-4; md.use_w = true;
  md.tau_rated = 1.3; md.tau_fric_max = 0.1; md.mu_hat = 0.3;
  md.wheel_load_share = TVec4(0.2388, 0.2392, 0.2608, 0.2612);
  md.mu_hat_wheels = TVec4(0.598, 0.489, 0.547, 0.703);
  return md;
}

static TorqueTuning real_tuning()
{
  TorqueTuning tu;
  tu.Np = 75; tu.Nc = 40;
  tu.W << 20.0, 20.0, 10.0, 2.0, 2.0, 1.0;
  tu.w_delta = 0.01 / (0.01 * 0.01); tu.r_u = 10.0;
  tu.rate_fraction = 0.3333;
  tu.limit_mode = TorqueLimitMode::Friction;
  return tu;
}

static void run(const std::string & name, const TorqueModel & md, const TorqueTuning & tu,
  const std::function<void(TorqueMPC &, int)> & pre = nullptr)
{
  TorqueMPC mpc(md, tu);
  const TVec4 coul(0.0174, 0.0206, 0.0155, 0.0157), visc(0.001066, 0.001158, 0.001112, 0.001177);
  TVec6 x;
  x << 0.02, -0.015, 0.03, 0.0, 0.0, 0.0;
  unsigned lcg = 12345u;
  auto noise = [&lcg]() {lcg = lcg * 1664525u + 1013904223u; return (static_cast<double>(lcg >> 8) / 16777216.0 - 0.5);};
  for (int k = 0; k < 400; ++k) {
    const double t = k * tu.Ts;
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= tu.Np; ++j) {ref.push_back(ref_at(t + j * tu.Ts));}
    if (pre) {pre(mpc, k);}
    TVec6 xm = x;                                   // 측정 잡음 (결정적)
    xm(0) += 2e-3 * noise(); xm(1) += 2e-3 * noise(); xm(3) += 3e-2 * noise(); xm(4) += 3e-2 * noise(); xm(5) += 5e-2 * noise();
    const TVec4 tau = mpc.step(xm, ref);
    std::printf("%s %d %.17g %.17g %.17g %.17g %d %d\n", name.c_str(), k, tau(0), tau(1), tau(2), tau(3),
      mpc.stats().active, mpc.stats().fallback);
    // 플랜트: 모델 + 구동계 마찰 (보상 안 된 판) — 몸체 속도에서 바퀴 속도, 마찰 토크만큼 덜 전달
    const double c = std::cos(x(2)), s = std::sin(x(2));
    const TVec3 vb(c * x(3) + s * x(4), -s * x(3) + c * x(4), x(5));
    const TVec4 om = md.wheel_nominal_speeds(vb);
    TVec4 f;
    for (int i = 0; i < 4; ++i) {f(i) = coul(i) * std::tanh(om(i) / 0.5) + visc(i) * om(i);}
    const TVec6 dx = torque_dynamics(md, x, tau - f);
    x += tu.Ts * dx;
  }
}

int main()
{
  const TorqueModel md = real_model();
  {TorqueTuning tu = real_tuning(); tu.limit_mode = TorqueLimitMode::MotorOnly; run("motor_only", md, tu);}
  {TorqueTuning tu = real_tuning(); run("friction_total", md, tu);}
  {TorqueTuning tu = real_tuning(); tu.cap_on_ground = true; run("friction_ground", md, tu);}
  {
    TorqueTuning tu = real_tuning();
    tu.dist_enable = true; tu.dist_alpha = 0.01; tu.dist_accel_only = true; tu.dist_body_frame = true;
    tu.dist_freeze_on_sat = true; tu.dist_acc_max = 0.5; tu.dist_yaw_acc_max = 2.0;
    run("friction_total_dhat", md, tu);
    tu.cap_on_ground = true; tu.target_projection = true;
    run("friction_ground_dhat_proj", md, tu);
    tu.dist_kalman = true;
    run("friction_ground_dhat_kalman", md, tu);
  }
  {TorqueTuning tu = real_tuning(); tu.load_com_height = 0.105; run("friction_total_load", md, tu);}
  {TorqueTuning tu = real_tuning(); tu.target_projection = true; run("friction_total_proj", md, tu);}
  {
    TorqueTuning tu = real_tuning();
    run("friction_total_slip", md, tu, [](TorqueMPC & m, int k) {
        const double e = 0.5 + 0.5 * std::cos(0.07 * k);
        m.set_slip_scale(TVec4(e, 1.0, 1.0 - 0.5 * e, (k / 50) % 2 ? 0.0 : 1.0));
      });
  }
  {TorqueTuning tu = real_tuning(); tu.linearization = TorqueLinearization::CurrentState; run("friction_current_state", md, tu);}
  return 0;
}
