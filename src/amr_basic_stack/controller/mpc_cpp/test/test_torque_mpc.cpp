// 토크 MPC 단위 시험 — 『메카넘 토크 MPC 1판 설계』의 수치 예와 09절 자기검사, 11절 통과시험의 축소판.
#include "amr_mpc_cpp/torque_mpc.hpp"

#include <gtest/gtest.h>

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

TEST(TorqueModel, ForceMapNullSpaceAndNumbers)
{
  TorqueModel md;
  EXPECT_LT((md.force_map() * TorqueMPC::null_dir()).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(md.mu(), 2.425, 1e-3);                    // 1판 식-3: m_w 는 늘 M 에 (w 항·외란 추정치 지시서 2026-11-16)
  EXPECT_NEAR(md.I_eff() - md.inertia_z, 0.0954, 1e-4);
  EXPECT_NEAR(md.tau_friction_limit(), 0.0666, 5e-4);   // 1판 식-21 수치 예 (4 kg, μ̂ 0.3)
  // 네 바퀴 같은 토크 → 전방 가속만
  const TVec3 a = md.G(0.0) * TVec4::Constant(0.1);
  EXPECT_NEAR(a(0), 0.4 / (0.04 * 6.425), 1e-9);
  EXPECT_NEAR(a(1), 0.0, 1e-12);
  EXPECT_NEAR(a(2), 0.0, 1e-12);
  // θ = 90° 면 "앞으로" 가 map 의 +y
  const TVec3 a90 = md.G(M_PI / 2) * TVec4::Constant(0.1);
  EXPECT_NEAR(a90(0), 0.0, 1e-12);
  EXPECT_GT(a90(1), 0.0);
  // 1판 식-14 수치 예: θ_r = 90°, u_r = [1, 0, 0.417] → w_xy = [−0.157, 0]  (w 항 스위치와 상관없는 값 — 켜고 끄기는 WTermSwitch)
  const TVec3 w = md.w_value(TVec3(0.0, 1.0, 0.417));
  EXPECT_NEAR(w(0), -0.157, 1e-3);
  EXPECT_NEAR(w(1), 0.0, 1e-12);
}

TEST(TorqueModel, LimitModeParsing)
{
  EXPECT_EQ(parse_limit_mode("motor_only"), TorqueLimitMode::MotorOnly);
  EXPECT_EQ(parse_limit_mode("friction"), TorqueLimitMode::Friction);
  EXPECT_THROW(parse_limit_mode("friction_mtte"), std::invalid_argument);
  EXPECT_THROW(parse_limit_mode("nope"), std::invalid_argument);
}

// 지시서 U8 (1판 07절·발표 20쪽 손 계산): Np = Nc = 1, 제약 없음, 정지, 1 cm 앞, Q = diag(100,100,100,1,1,1),
// r_Δ = r_U = 0.001 → 0.0241 N·m × 4 (네 바퀴 같음, 영공간 성분 0). 기준이 정지라 U_s = 0
TEST(TorqueMPC, HandExampleOneStep)
{
  TorqueModel md;                          // 예제가 m_eff = 6.425 kg 기준 (m_w 는 늘 M 에)
  TorqueTuning tu;
  tu.Np = 1; tu.Nc = 1; tu.constrained = false;
  tu.W << 100, 100, 100, 1, 1, 1;
  tu.w_delta = 1e-3; tu.r_u = 1e-3; tu.q_null = 0.0;
  TorqueMPC mpc(md, tu);
  std::vector<TorqueRefPoint> ref = {{TVec3::Zero(), TVec3::Zero()}, {TVec3(0.01, 0, 0), TVec3::Zero()}};
  const TVec4 tau = mpc.step(TVec6::Zero(), ref);
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(tau(i), 0.0241, 1e-4);}
  EXPECT_NEAR(TorqueMPC::null_dir().dot(tau), 0.0, 1e-12);
  EXPECT_LT(mpc.target_input().cwiseAbs().maxCoeff(), 1e-12);
}

// 지시서 U10 / w 항 지시서 W3: 목표 입력 — 4 kg, 1 m/s, R 2.4 m 등속 선회, θ_r = 90°, 해석 가속도, nᵀU_s = 0
//   use_w 켬: m_eq (p̈ − w) = (m + m_w)·v²/R − m_w·v²/R = m v²/R → ±0.01667 N·m
//   use_w 끔: m_eq p̈ = 6.425 × 0.4167 = 2.677 N → ±0.02677 N·m   (부호 [−, +, +, −])
class TargetInput : public ::testing::TestWithParam<bool> {};
TEST_P(TargetInput, SteadyTurnMinimumNorm)
{
  TorqueModel md;
  md.use_w = GetParam();
  TorqueTuning tu;
  tu.constrained = false;
  TorqueMPC mpc(md, tu);
  const double R = 2.4, v = 1.0, om = v / R, th = M_PI / 2;
  // θ = 90° 에서 앞(+x 몸체) = map +y, 원 중심은 몸체 왼쪽 = map −x
  std::vector<TorqueRefPoint> ref;
  for (int j = 0; j <= tu.Np; ++j) {ref.push_back({TVec3(0, 0, th + om * j * tu.Ts), TVec3(v, 0, om)});}
  std::vector<TVec3> acc(tu.Nc, TVec3(-v * v / R, 0.0, 0.0));   // 해석 가속도 (관성 좌표, 중심 쪽)
  TVec6 x;
  x << 0, 0, th, 0, v, om;
  mpc.step(x, ref, &acc);
  const TVec4 Us = mpc.target_input().head<4>();
  const double want = 0.04 / 4.0 * (md.use_w ? md.mass : md.m_eff()) * v * v / R;   // 0.01667 / 0.02677
  EXPECT_NEAR(want, md.use_w ? 0.016667 : 0.026771, 1e-5);
  EXPECT_NEAR(Us(0), -want, 1e-4);
  EXPECT_NEAR(Us(1), want, 1e-4);
  EXPECT_NEAR(Us(2), want, 1e-4);
  EXPECT_NEAR(Us(3), -want, 1e-4);
  EXPECT_NEAR(TorqueMPC::null_dir().dot(Us), 0.0, 1e-12);
  // 지평 점에서 계산한 p̈ (acc_ref 없이) 도 같은 값 — 등속 선회에서는 ω S u 항이 정확히 구심 가속도
  TorqueMPC mpc2(md, tu);
  mpc2.step(x, ref);
  EXPECT_LT((mpc2.target_input().head<4>() - Us).cwiseAbs().maxCoeff(), 1e-6);
}
INSTANTIATE_TEST_SUITE_P(WTerm, TargetInput, ::testing::Values(false, true));

// 2026-11-18 velocity_source wheels: 바퀴 기구학 T ω 가 force_map 의 T⁻¹ 와 짝 — v → ω = T⁻¹ v → T ω = v,
//   전진 0.5 m/s 면 네 바퀴 모두 +12.5 rad/s (WheelStates 부호 규약: 전진 시 ω > 0)
TEST(TorqueModel, WheelKinematicsInvertsForceMap)
{
  TorqueModel md;
  const TMat34 Ft = md.force_map();             // (T⁻¹)ᵀ
  for (const TVec3 & v : {TVec3(0.5, 0, 0), TVec3(0, 0.3, 0), TVec3(0, 0, 1.2), TVec3(0.7, -0.2, 1.4)}) {
    const TVec4 omega = Ft.transpose() * v;     // T⁻¹ v
    EXPECT_LT((md.wheel_body_velocity(omega) - v).cwiseAbs().maxCoeff(), 1e-12);
  }
  const TVec4 fwd = md.force_map().transpose() * TVec3(0.5, 0, 0);
  EXPECT_NEAR(fwd.minCoeff(), 12.5, 1e-9);
  EXPECT_NEAR(fwd.maxCoeff(), 12.5, 1e-9);
}

// w 항 지시서 W1: w (식-6) — 4 kg, θ_r = 90°, ṗ_r = [0, 1, 0.417] → [−0.1574, 0, 0] m/s² (원 중심 쪽 = 서쪽).
//   use_w 가 꺼지면 w() 는 더하지 않는 0, 실제 값은 w_value() (기록·잔차 저장용)
TEST(TorqueModel, WTermSwitch)
{
  TorqueModel md;                                  // 4 kg, J_w 9.7e-4 (지시서 수치) → m_w 2.425, m_eq 6.425
  const TVec3 v(0.0, 1.0, 0.417);
  EXPECT_LT(md.w(v).cwiseAbs().maxCoeff(), 1e-15);  // 기본 끔
  EXPECT_NEAR(md.w_value(v)(0), -2.425 / 6.425 * 0.417, 1e-12);
  EXPECT_NEAR(md.w_value(v)(0), -0.1574, 1e-4);
  md.use_w = true;
  EXPECT_NEAR(md.w(v)(0), -0.1574, 1e-4);
  EXPECT_NEAR(md.w(v)(1), 0.0, 1e-12);
  EXPECT_NEAR(md.w(v)(2), 0.0, 1e-12);
  EXPECT_NEAR(md.m_eff(), 6.425, 1e-3);             // M 은 use_w 와 상관없이 m_eq
}

// 지시서 U11: U10 상태에서 X̂ = X_r,0, U_{-1} = U_s,0 이면 한 주기 해 U* ≈ U_s,0
TEST(TorqueMPC, OnReferenceSolutionEqualsTargetInput)
{
  TorqueModel md;
  TorqueTuning tu;
  TorqueMPC mpc(md, tu);
  Plant plant{md};
  const double R = 2.4, v = 1.0;
  plant.x << 0, 0, 0, v, 0, v / R;
  TVec4 tau = TVec4::Zero();
  for (int k = 0; k < 300; ++k) {               // 기준 위에서 몇 초 돌아 U_{-1} 이 U_s 로 들어가게
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, k * tu.Ts + j * tu.Ts));}
    tau = mpc.step(plant.x, ref);
    plant.step(tau, tu.Ts);
  }
  EXPECT_LT((tau - mpc.target_input().head<4>()).cwiseAbs().maxCoeff(), 1e-4);
}

// 지시서 4절·7.4절: 상태 간격이 Ts 에서 벗어나거나 위치 잔차가 계단이면 외란 추정치 갱신을 건너뛴다
TEST(TorqueMPC, DisturbanceSkipRules)
{
  TorqueTuning tu;
  tu.dist_enable = true;
  TorqueMPC mpc(TorqueModel{}, tu);
  const TVec3 X(0, 0, 0);
  TVec6 x = TVec6::Zero();
  mpc.step(x, hold_ref(X, 51));
  mpc.step(x, hold_ref(X, 51), nullptr, tu.Ts);
  EXPECT_EQ(mpc.stats().d_skipped, 0);
  mpc.step(x, hold_ref(X, 51), nullptr, 2.5 * tu.Ts);         // 상태 하나를 놓쳤다
  EXPECT_EQ(mpc.stats().d_skipped, 1);
  TVec6 jump = x;
  jump(0) = 0.01;                                              // SLAM 보정 1 cm 계단
  mpc.step(jump, hold_ref(X, 51), nullptr, tu.Ts);
  EXPECT_EQ(mpc.stats().d_skipped, 2);
  EXPECT_LT(mpc.d_hat().head<2>().cwiseAbs().maxCoeff(), 1e-3);   // 계단이 d̂ 로 들어가지 않았다
}

// 09절 자기검사 2: 정지 상태에서 기준이 현재 포즈면 T* = 0
TEST(TorqueMPC, RestAtReferenceGivesZeroTorque)
{
  TorqueMPC mpc(TorqueModel{}, TorqueTuning{});
  const TVec3 X(0.3, -0.2, 1.0);
  TVec6 x = TVec6::Zero();
  x.head<3>() = X;
  const TVec4 tau = mpc.step(x, hold_ref(X, 51));
  EXPECT_LT(mpc.solution().cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT(tau.cwiseAbs().maxCoeff(), 1e-9);
}

// 식-15: 접힌 기준각 (θ̂ = 3.0, θ_r = −3.1) 이 2π 점프를 만들지 않는다
TEST(TorqueMPC, YawUnwrap)
{
  TorqueTuning tu;
  TorqueMPC a(TorqueModel{}, tu), b(TorqueModel{}, tu);
  TVec6 x = TVec6::Zero();
  x(2) = 3.0;
  const TVec4 ta = a.step(x, hold_ref(TVec3(0, 0, -3.1), 51));
  const TVec4 tb = b.step(x, hold_ref(TVec3(0, 0, 3.0 + 0.18318530718), 51));
  EXPECT_LT((ta - tb).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_GT(ta(1), 0.0);          // 반시계로 0.18 rad 더 돌아야 한다: [−, +, −, +]
  EXPECT_LT(ta(0), 0.0);
}

// 통과시험 1: 등속 선회 1 m/s, 반경 2.4 m, 4 kg → 최소 노름 분배 [−0.0167, 0.0167, 0.0167, −0.0167], 오차 0
class SteadyTurn : public ::testing::TestWithParam<bool> {};

TEST_P(SteadyTurn, MinimumNormTorqueAndZeroError)
{
  TorqueModel md;
  md.use_w = GetParam();
  TorqueTuning tu;
  TorqueMPC mpc(md, tu);
  Plant plant{md};
  const double R = 2.4, v = 1.0;
  plant.x << 0, 0, 0, v, 0, v / R;        // 기준 위에서 출발
  TVec4 tau = TVec4::Zero();
  double emax = 0.0;
  for (int k = 0; k < 400; ++k) {
    const double t = k * tu.Ts;
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, t + j * tu.Ts));}
    tau = mpc.step(plant.x, ref);
    plant.step(tau, tu.Ts);
    if (k > 200) {
      const auto r = circle(R, v, t + tu.Ts);
      emax = std::max(emax, (plant.x.head<2>() - r.first.head<2>()).norm());
    }
  }
  // use_w 켬: m ω v = 1.67 N (몸체 질량 4 kg — m_eq p̈ − m_w ω v), 끔: m_eq ω v = 2.68 N (w 항·외란 추정치 지시서 W3)
  const double f = (md.use_w ? md.mass : md.m_eff()) * (v * v / R);
  const double want = 0.04 / 4.0 * f;      // ± r/4 · F_y
  EXPECT_NEAR(tau(0), -want, 0.02 * want);
  EXPECT_NEAR(tau(1), want, 0.02 * want);
  EXPECT_NEAR(tau(2), want, 0.02 * want);
  EXPECT_NEAR(tau(3), -want, 0.02 * want);
  EXPECT_NEAR(TorqueMPC::null_dir().dot(tau), 0.0, 1e-6);
  EXPECT_LT(emax, 1e-3);                   // 등속 선회 오차 1 mm 미만
  EXPECT_EQ(mpc.stats().fallback, 0);
}
INSTANTIATE_TEST_SUITE_P(WTerm, SteadyTurn, ::testing::Values(false, true));

// 바퀴별 하중 몫 (2026-11-19): friction 모드에서 τ̄_i = (r/√2)·μ̂·(몫_i·m g)·c_s. 몫 1/4 이면 예전 식-21 과 같다.
//   토크·변화율이 바퀴마다 자기 상한 안에 있고, 가벼운 바퀴(FL)는 자기 상한에 붙는다. motor_only 에서는 안 쓰인다
TEST(TorqueMPC, WheelLoadShare)
{
  TorqueModel md;
  md.mu_hat = 0.4;
  const TVec4 even = md.tau_friction_limits();
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(even(i), md.tau_friction_limit(), 1e-12);}   // 기본 1/4 = mg/4
  md.wheel_load_share = TVec4(0.2388, 0.2392, 0.2608, 0.2612);                        // 2026-11-18 저울
  const double per_kg = md.wheel_radius / std::sqrt(2.0) * md.mu_hat * md.mass * md.gravity * md.c_s;
  const TVec4 tf = md.tau_friction_limits();
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(tf(i), per_kg * md.wheel_load_share(i), 1e-12);}
  EXPECT_NEAR(tf.mean(), md.tau_friction_limit(), 1e-12);      // 몫 합 1 → 평균은 그대로
  EXPECT_LT(tf(0), tf(2));                                    // 앞(가벼움) < 뒤
  TorqueTuning tu;
  tu.limit_mode = TorqueLimitMode::Friction;
  TorqueMPC mpc(md, tu);
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(mpc.tau_bar()(i), std::min(tf(i), md.tau_motor_limit()), 1e-12);}
  Plant plant{md};
  TVec4 prev = TVec4::Zero();
  bool fl_saturated = false;
  for (int k = 0; k < 100; ++k) {
    const TVec4 tau = mpc.step(plant.x, hold_ref(TVec3(1.0, 0, 0), 51));     // 1 m 앞: 급가속 요구
    for (int i = 0; i < 4; ++i) {
      EXPECT_LE(std::abs(tau(i)), tf(i) + 1e-9) << "k=" << k << " i=" << i;
      EXPECT_LE(std::abs(tau(i) - prev(i)), tf(i) / 3.0 + 1e-9) << "k=" << k << " i=" << i;
    }
    EXPECT_EQ(mpc.stats().fallback, 0);
    if (std::abs(std::abs(tau(0)) - tf(0)) < 1e-9) {fl_saturated = true;}
    plant.step(tau, tu.Ts);
    prev = tau;
  }
  EXPECT_TRUE(fl_saturated);
  TorqueTuning tub;                             // motor_only: 하중 몫을 안 쓴다
  TorqueMPC mpcb(md, tub);
  EXPECT_NEAR(mpcb.tau_bar()(0), md.tau_motor_limit(), 1e-12);
  EXPECT_NEAR(mpcb.tau_bar()(3), md.tau_motor_limit(), 1e-12);
}

// 바퀴별 μ̂ (2026-11-19 (11), 2026-11-22 N_i 기준): 값이 있는 바퀴는 (r/√2)·μ̂_i·N_i·c_s — mu_hat 자리만 바뀌고 하중 몫은 곱한다.
//   0 이하인 바퀴는 mu_hat × 하중 몫
TEST(TorqueMPC, MuHatWheelsUseLoadShare)
{
  TorqueModel md;
  md.mu_hat = 0.4;
  md.wheel_load_share = TVec4(0.2388, 0.2392, 0.2608, 0.2612);
  md.mu_hat_wheels = TVec4(0.598, 0.489, -1.0, 0.703);
  const double unit = md.wheel_radius / std::sqrt(2.0) * md.mass * md.gravity * md.c_s;   // μ̂ 1, 하중 몫 1 일 때
  const TVec4 tf = md.tau_friction_limits();
  EXPECT_NEAR(tf(0), 0.598 * 0.2388 * unit, 1e-12);
  EXPECT_NEAR(tf(1), 0.489 * 0.2392 * unit, 1e-12);
  EXPECT_NEAR(tf(2), 0.4 * 0.2608 * unit, 1e-12);             // RL: mu_hat × 하중 몫
  EXPECT_NEAR(tf(3), 0.703 * 0.2612 * unit, 1e-12);
  // 하중 몫이 1/4 이면 11-21 까지의 mg/4 기준 식 (r/√2)·μ̂_i·(mg/4)·c_s 와 같다
  md.wheel_load_share = TVec4::Constant(0.25);
  EXPECT_NEAR(md.tau_friction_limits()(0), 0.598 * unit / 4.0, 1e-12);
  md.wheel_load_share = TVec4(0.2388, 0.2392, 0.2608, 0.2612);
  TorqueTuning tu;
  tu.limit_mode = TorqueLimitMode::Friction;
  TorqueMPC mpc(md, tu);
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(mpc.tau_bar()(i), tf(i), 1e-12);}
  Plant plant{md};
  TVec4 prev = TVec4::Zero();
  for (int k = 0; k < 60; ++k) {
    const TVec4 tau = mpc.step(plant.x, hold_ref(TVec3(1.0, 0, 0), 51));
    for (int i = 0; i < 4; ++i) {EXPECT_LE(std::abs(tau(i)), tf(i) + 1e-9) << "k=" << k << " i=" << i;}
    EXPECT_EQ(mpc.stats().fallback, 0);
    plant.step(tau, tu.Ts);
    prev = tau;
  }
}

// 통과시험 3 (제약 부분): 비교군 C 상한에서 토크가 τ̄ 에 붙고, 변화율이 Δτ̄ 를 넘지 않는다
TEST(TorqueMPC, FrictionLimitAndRateAreRespected)
{
  TorqueModel md;
  TorqueTuning tu;
  tu.limit_mode = TorqueLimitMode::Friction;
  TorqueMPC mpc(md, tu);
  const double tb = md.tau_friction_limit(), dtb = tb / 3.0;
  EXPECT_NEAR(mpc.tau_bar()(0), tb, 1e-12);
  Plant plant{md};
  TVec4 prev = TVec4::Zero();
  double tmax = 0.0;
  bool saturated = false;
  for (int k = 0; k < 100; ++k) {
    const TVec4 tau = mpc.step(plant.x, hold_ref(TVec3(1.0, 0, 0), 51));     // 1 m 앞: 급가속 요구
    EXPECT_LE((tau - prev).cwiseAbs().maxCoeff(), dtb + 1e-9) << "k=" << k;
    EXPECT_LE(mpc.solution().cwiseAbs().maxCoeff(), tb + 1e-9);
    EXPECT_EQ(mpc.stats().fallback, 0);
    tmax = std::max(tmax, tau.cwiseAbs().maxCoeff());
    if (std::abs(tau(0) - tb) < 1e-9) {saturated = true; EXPECT_GT(mpc.stats().active, 0);}
    plant.step(tau, tu.Ts);
    prev = tau;
  }
  EXPECT_TRUE(saturated);
  EXPECT_NEAR(tmax, tb, 1e-9);
  // 비교군 B 는 같은 상황에서 훨씬 큰 토크를 쓴다
  TorqueTuning tub;
  TorqueMPC mpcb(md, tub);
  Plant pb{md};
  double tmax_b = 0.0;
  for (int k = 0; k < 20; ++k) {
    const TVec4 tau = mpcb.step(pb.x, hold_ref(TVec3(1.0, 0, 0), 51));
    tmax_b = std::max(tmax_b, tau.cwiseAbs().maxCoeff());
    pb.step(tau, tub.Ts);
  }
  EXPECT_GT(tmax_b, 3.0 * tb);
  EXPECT_LE(tmax_b, md.tau_motor_limit() + 1e-9);
}

// 상한이 한 스텝에 Δτ̄ 보다 크게 내려가도 (MTTE 등) 해가 있고 상한이 우선한다
TEST(TorqueMPC, LimitDropBeyondRateStaysFeasible)
{
  TorqueModel md;
  TorqueMPC mpc(md, TorqueTuning{});
  Plant plant{md};
  TVec4 tau = TVec4::Zero();
  for (int k = 0; k < 10; ++k) {
    tau = mpc.step(plant.x, hold_ref(TVec3(2.0, 0, 0), 51));
    plant.step(tau, 0.01);
  }
  ASSERT_GT(tau(0), 0.3);                               // 모터 한계 쪽으로 크게 올라가 있다
  mpc.set_tau_bar_override(TVec4::Constant(0.05));      // 갑자기 0.05 로
  tau = mpc.step(plant.x, hold_ref(TVec3(2.0, 0, 0), 51));
  EXPECT_LE(tau.cwiseAbs().maxCoeff(), 0.05 + 1e-9);
  EXPECT_NEAR(tau(0), 0.05, 1e-6);
}

// 식-9 수치 예: 모델에 없는 1 N → d̂ = [7.8e-6, 0, 0, 0.00156, 0, 0]  (m_eff 6.425 kg)
TEST(TorqueMPC, DisturbanceResidualMatchesDesignExample)
{
  TorqueModel md;                         // m_eff 6.425 kg (m_w 는 늘 M 에)
  TorqueMPC mpc(md, TorqueTuning{});      // d̂ 는 꺼져 있어도 계산은 한다
  Plant plant{md};
  plant.force_ext << 1.0, 0, 0;
  plant.x << 0, 0, 0, 0.5, 0, 0;
  const auto ref = hold_ref(TVec3::Zero(), 51);
  const TVec4 t0 = mpc.step(plant.x, ref);
  plant.step(t0, 0.01);
  mpc.step(plant.x, ref);
  EXPECT_NEAR(mpc.d_hat()(0), 7.8e-6, 2e-7);
  EXPECT_NEAR(mpc.d_hat()(3), 0.00156, 2e-5);
  EXPECT_LT(std::abs(mpc.d_hat()(1)) + std::abs(mpc.d_hat()(2)) + std::abs(mpc.d_hat()(4)), 1e-9);
}

// 통과시험 2(b) 축소판: 모르는 일정한 힘 아래 등속 직진. d̂ 를 켜면 정상상태 오차가 사라지고, 끄면 남는다
TEST(TorqueMPC, DisturbanceEstimateRemovesSteadyStateOffset)
{
  auto run = [](bool enable) {
      TorqueModel md;
      TorqueTuning tu;
      tu.dist_enable = enable;
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      plant.force_ext << -1.0, 0, 0;         // 구름 저항 같은 일정한 힘
      const double v = 0.5;
      plant.x << 0, 0, 0, v, 0, 0;
      double err = 0.0;
      for (int k = 0; k < 500; ++k) {
        std::vector<TorqueRefPoint> ref;
        for (int j = 0; j <= tu.Np; ++j) {ref.push_back({TVec3(v * (k + j) * tu.Ts, 0, 0), TVec3(v, 0, 0)});}
        plant.step(mpc.step(plant.x, ref), tu.Ts);
        err = v * (k + 1) * tu.Ts - plant.x(0);
      }
      return err;
    };
  const double off = run(false), on = run(true);
  // 노미널 MPC 는 뒤처진 채로 남는다. 다만 위치 가중이 커서 1 N 에 0.37 mm 정도다 (2026-11-11 실측)
  EXPECT_GT(off, 2e-4);
  EXPECT_LT(off, 1e-3);
  EXPECT_LT(std::abs(on), 0.05 * off);       // d̂: 오프셋 제거 (1/20 미만)
  std::printf("            정상상태 오차  d̂ 끔 %.3f mm   d̂ 켬 %.4f mm\n", off * 1e3, on * 1e3);
}

// 2026-11-18 가속도 외란 모델 (dist_accel_only): 진짜 힘이면 식-9 와 같은 d̂, 위치만 튄 잔차는 d̂ 로 안 들어가고, 오프셋도 없앤다
TEST(TorqueMPC, DisturbanceAccelOnly)
{
  TorqueModel md;
  {   // (a) 모델에 없는 1 N — 식-9 수치 예와 같은 값 (위치 성분 ½Ts·0.00156 = 7.8e-6)
    TorqueTuning tu;
    tu.dist_accel_only = true;
    TorqueMPC mpc(md, tu);
    Plant plant{md};
    plant.force_ext << 1.0, 0, 0;
    plant.x << 0, 0, 0, 0.5, 0, 0;
    const auto ref = hold_ref(TVec3::Zero(), 51);
    plant.step(mpc.step(plant.x, ref), 0.01);
    mpc.step(plant.x, ref);
    EXPECT_NEAR(mpc.d_hat()(0), 7.8e-6, 2e-7);
    EXPECT_NEAR(mpc.d_hat()(3), 0.00156, 2e-5);
  }
  for (bool accel : {false, true}) {   // (b) 속도 그대로 위치만 0.5 mm (skip_xy 1 mm 아래): 식-9 는 d̂_x 로, 가속도 모델은 0
    TorqueTuning tu;
    tu.dist_accel_only = accel;
    TorqueMPC mpc(md, tu);
    TVec6 x = TVec6::Zero();
    mpc.step(x, hold_ref(TVec3::Zero(), 51));
    x(0) = 5e-4;
    mpc.step(x, hold_ref(TVec3::Zero(), 51), nullptr, tu.Ts);
    EXPECT_EQ(mpc.stats().d_skipped, 0);
    EXPECT_NEAR(mpc.d_hat()(0), accel ? 0.0 : 5e-4, 1e-9);
  }
  // (c) 모르는 일정한 힘 아래 등속 직진: 가속도 모델로도 오프셋이 사라진다
  auto run = [&md](bool enable) {
      TorqueTuning tu;
      tu.dist_enable = enable;
      tu.dist_accel_only = true;
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      plant.force_ext << -1.0, 0, 0;
      const double v = 0.5;
      plant.x << 0, 0, 0, v, 0, 0;
      double err = 0.0;
      for (int k = 0; k < 500; ++k) {
        std::vector<TorqueRefPoint> ref;
        for (int j = 0; j <= tu.Np; ++j) {ref.push_back({TVec3(v * (k + j) * tu.Ts, 0, 0), TVec3(v, 0, 0)});}
        plant.step(mpc.step(plant.x, ref), tu.Ts);
        err = v * (k + 1) * tu.Ts - plant.x(0);
      }
      return err;
    };
  const double off = run(false), on = run(true);
  EXPECT_GT(off, 2e-4);
  EXPECT_LT(std::abs(on), 0.05 * off);
  std::printf("            가속도 외란 모델  d̂ 끔 %.3f mm   d̂ 켬 %.4f mm\n", off * 1e3, on * 1e3);
}

TEST(TorqueMPC, RejectsBadConfig)
{
  TorqueTuning tu;
  tu.Nc = 60;
  EXPECT_THROW(TorqueMPC(TorqueModel{}, tu), std::invalid_argument);
  TorqueModel md;
  md.tau_fric_max = 2.0;                     // 모터 정격보다 큰 마찰 여유
  EXPECT_THROW(TorqueMPC(md, TorqueTuning{}), std::invalid_argument);
  TorqueMPC ok(TorqueModel{}, TorqueTuning{});
  EXPECT_THROW(ok.step(TVec6::Zero(), hold_ref(TVec3::Zero(), 10)), std::invalid_argument);
}

// ───────────── 현재 상태 기준 선형화 (linearization = current_state) ─────────────
// torque_mpc.hpp 머리주석 "선형화 방식". 인수인계 문서(2026-11-14 jjm) 2절의 시험 9 종.
namespace
{
TorqueTuning current_tuning()
{
  TorqueTuning tu;
  tu.linearization = TorqueLinearization::CurrentState;
  return tu;
}

// 등속 원운동 기준열 (지평 Np+1 점)
std::vector<TorqueRefPoint> circle_ref(double R, double v, double t, const TorqueTuning & tu)
{
  std::vector<TorqueRefPoint> ref;
  for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, t + j * tu.Ts));}
  return ref;
}
}  // namespace

// 1. 문자열 처리
TEST(TorqueLinearizationModel, Parsing)
{
  EXPECT_EQ(parse_linearization("reference"), TorqueLinearization::Reference);
  EXPECT_EQ(parse_linearization("current_state"), TorqueLinearization::CurrentState);
  EXPECT_THROW(parse_linearization("current"), std::invalid_argument);
  EXPECT_THROW(parse_linearization(""), std::invalid_argument);
  EXPECT_EQ(TorqueTuning{}.linearization, TorqueLinearization::Reference);   // 구조체 기본값은 1판 방식
}

// 2. 야코비안이 torque_dynamics 의 중앙 유한차분과 같다 (w 항 켬, 움직이는 상태, τ₀ ≠ 0)
TEST(TorqueLinearizationModel, JacobianMatchesFiniteDifference)
{
  TorqueModel md;
  md.use_w = true;
  TVec6 x0;
  x0 << 0.3, -0.2, 0.7, 0.4, -0.25, 0.8;
  TVec4 tau0;
  tau0 << 0.05, -0.02, 0.08, 0.01;
  const auto lin = linearize_continuous(md, x0, tau0);
  const double h = 1e-6;
  for (int j = 0; j < 6; ++j) {
    TVec6 e = TVec6::Zero();
    e(j) = h;
    const TVec6 col = (torque_dynamics(md, x0 + e, tau0) - torque_dynamics(md, x0 - e, tau0)) / (2 * h);
    EXPECT_LT((lin.A.col(j) - col).cwiseAbs().maxCoeff(), 1e-7) << "A col " << j;
  }
  for (int j = 0; j < 4; ++j) {
    TVec4 e = TVec4::Zero();
    e(j) = h;
    const TVec6 col = (torque_dynamics(md, x0, tau0 + e) - torque_dynamics(md, x0, tau0 - e)) / (2 * h);
    EXPECT_LT((lin.B.col(j) - col).cwiseAbs().maxCoeff(), 1e-7) << "B col " << j;
  }
  // 선형화 점에서 값이 같다 (상수항 c)
  EXPECT_LT((lin.A * x0 + lin.B * tau0 + lin.c - torque_dynamics(md, x0, tau0)).cwiseAbs().maxCoeff(), 1e-12);
}

// 3. θ̇ = 0, τ₀ = 0, 바퀴 관성 끔이면 ZOH 결과가 1판 식-8 과 같다
TEST(TorqueLinearizationModel, ZohReducesToDesignEq8)
{
  TorqueModel md;
  const double Ts = 0.01;
  TVec6 x0;
  x0 << 1.0, 2.0, 0.6, 0.3, -0.1, 0.0;
  const auto d = discretize_zoh(linearize_continuous(md, x0, TVec4::Zero()), Ts);
  TMat6 A = TMat6::Identity();
  A.block<3, 3>(0, 3) = Ts * Eigen::Matrix3d::Identity();
  TMat64 B;
  B.topRows<3>() = 0.5 * Ts * Ts * md.G(x0(2));
  B.bottomRows<3>() = Ts * md.G(x0(2));
  EXPECT_LT((d.A - A).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((d.B - B).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT(d.c.cwiseAbs().maxCoeff(), 1e-12);
}

// 4. 이산 모델의 한 스텝 예측이 RK4 플랜트와 맞는다 (선형화 점에서 출발 → 오차는 Ts² 차수보다 작다)
TEST(TorqueLinearizationModel, OneStepMatchesRk4Plant)
{
  for (bool inertia : {false, true}) {          // w 항 끔·켬
    TorqueModel md;
    md.use_w = inertia;
    Plant plant{md};
    plant.x << 0.2, 0.1, 1.2, 0.6, 0.3, 1.5;
    TVec4 tau;
    tau << 0.1, -0.05, 0.12, 0.02;
    const auto d = discretize_zoh(linearize_continuous(md, plant.x, tau), 0.01);
    const TVec6 pred = d.A * plant.x + d.B * tau + d.c;
    plant.step(tau, 0.01, 50);
    EXPECT_LT((pred - plant.x).head<3>().cwiseAbs().maxCoeff(), 1e-6) << "inertia " << inertia;
    EXPECT_LT((pred - plant.x).tail<3>().cwiseAbs().maxCoeff(), 1e-4) << "inertia " << inertia;
  }
}

// 5. 정지·기준 = 현재 포즈 → 토크 0, 그리고 yaw 언랩 (기존 두 시험의 current_state 판)
TEST(TorqueLinearizationMPC, RestAtReferenceAndYawUnwrap)
{
  {
    TorqueMPC mpc(TorqueModel{}, current_tuning());
    const TVec3 X(0.3, -0.2, 1.0);
    TVec6 x = TVec6::Zero();
    x.head<3>() = X;
    const TVec4 tau = mpc.step(x, hold_ref(X, 51));
    EXPECT_LT(tau.cwiseAbs().maxCoeff(), 1e-9);
  }
  TorqueMPC a(TorqueModel{}, current_tuning()), b(TorqueModel{}, current_tuning());
  TVec6 x = TVec6::Zero();
  x(2) = 3.0;
  const TVec4 ta = a.step(x, hold_ref(TVec3(0, 0, -3.1), 51));
  const TVec4 tb = b.step(x, hold_ref(TVec3(0, 0, 3.0 + 0.18318530718), 51));
  EXPECT_LT((ta - tb).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_GT(ta(1), 0.0);
  EXPECT_LT(ta(0), 0.0);
}

// 6. 등속 선회 통과시험 (1 m/s, R 2.4 m) 을 current_state 로 — 최소 노름 분배, 오차 1 mm 미만
class SteadyTurnCurrent : public ::testing::TestWithParam<bool> {};

TEST_P(SteadyTurnCurrent, MinimumNormTorqueAndZeroError)
{
  TorqueModel md;
  md.use_w = GetParam();
  const TorqueTuning tu = current_tuning();
  TorqueMPC mpc(md, tu);
  Plant plant{md};
  const double R = 2.4, v = 1.0;
  plant.x << 0, 0, 0, v, 0, v / R;
  TVec4 tau = TVec4::Zero();
  double emax = 0.0;
  for (int k = 0; k < 400; ++k) {
    const double t = k * tu.Ts;
    tau = mpc.step(plant.x, circle_ref(R, v, t, tu));
    plant.step(tau, tu.Ts);
    if (k > 200) {
      emax = std::max(emax, (plant.x.head<2>() - circle(R, v, t + tu.Ts).first.head<2>()).norm());
    }
  }
  const double want = 0.04 / 4.0 * (md.use_w ? md.mass : md.m_eff()) * (v * v / R);   // 0.01667 / 0.02677
  EXPECT_NEAR(tau(0), -want, 0.02 * want);
  EXPECT_NEAR(tau(1), want, 0.02 * want);
  EXPECT_NEAR(tau(2), want, 0.02 * want);
  EXPECT_NEAR(tau(3), -want, 0.02 * want);
  EXPECT_NEAR(TorqueMPC::null_dir().dot(tau), 0.0, 1e-6);
  EXPECT_LT(emax, 1e-3);
  EXPECT_EQ(mpc.stats().fallback, 0);
  // 9a. 플랜트 = 모델이면 외란 추정치가 0 근처 (이 모델의 한 스텝 잔차)
  EXPECT_LT(mpc.d_hat().cwiseAbs().maxCoeff(), 1e-5);
}
INSTANTIATE_TEST_SUITE_P(WTerm, SteadyTurnCurrent, ::testing::Values(false, true));

// 7. 직선·기준 위·τ_prev = 0 (정지, θ̇ = 0) 이면 두 방식의 첫 토크가 같다
TEST(TorqueLinearizationMPC, MatchesReferenceOnStraightLineFromRest)
{
  TorqueMPC ref_mpc(TorqueModel{}, TorqueTuning{}), cur_mpc(TorqueModel{}, current_tuning());
  TVec6 x = TVec6::Zero();
  x(2) = 0.4;
  std::vector<TorqueRefPoint> ref;
  for (int j = 0; j <= 50; ++j) {
    const double s = 0.002 * j * j * 0.01;   // 같은 방향으로 천천히 출발
    ref.push_back({TVec3(s * std::cos(0.4), s * std::sin(0.4), 0.4), TVec3::Zero()});
  }
  const TVec4 tr = ref_mpc.step(x, ref), tc = cur_mpc.step(x, ref);
  EXPECT_GT(tr.cwiseAbs().maxCoeff(), 1e-4);
  EXPECT_LT((tr - tc).cwiseAbs().maxCoeff(), 1e-9);
}

// 8. 초기 방향 오차가 큰 폐루프 (30°·40°): 두 방식 모두 수렴하는지, 추종 오차를 비교해 찍는다
TEST(TorqueLinearizationMPC, LargeInitialHeadingErrorClosedLoop)
{
  auto run = [](TorqueLinearization mode, double th0, double & e_rms, double & e_final) {
      TorqueModel md;
      TorqueTuning tu;
      tu.linearization = mode;
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      const double R = 1.0, v = 0.5;
      plant.x << 0, 0, th0, 0, 0, 0;          // 정지, 방향만 틀어져 있다
      double s2 = 0.0;
      int n = 0, fb = 0;
      for (int k = 0; k < 600; ++k) {
        const double t = k * tu.Ts;
        plant.step(mpc.step(plant.x, circle_ref(R, v, t, tu)), tu.Ts);
        fb += mpc.stats().fallback;
        const auto r = circle(R, v, t + tu.Ts);
        const double e = (plant.x.head<2>() - r.first.head<2>()).norm();
        s2 += e * e; ++n;
        e_final = e;
      }
      e_rms = std::sqrt(s2 / n);
      return fb;
    };
  for (double deg : {30.0, 40.0}) {
    const double th0 = deg * M_PI / 180.0;
    double r_rms, r_fin, c_rms, c_fin;
    const int fb_r = run(TorqueLinearization::Reference, th0, r_rms, r_fin);
    const int fb_c = run(TorqueLinearization::CurrentState, th0, c_rms, c_fin);
    std::printf("            초기 방향 오차 %2.0f°  reference RMS %.2f mm 끝 %.3f mm | current_state RMS %.2f mm 끝 %.3f mm\n",
      deg, r_rms * 1e3, r_fin * 1e3, c_rms * 1e3, c_fin * 1e3);
    EXPECT_EQ(fb_r, 0);
    EXPECT_EQ(fb_c, 0);
    EXPECT_LT(r_fin, 1e-3);                  // 둘 다 6 s 안에 기준으로 돌아온다
    EXPECT_LT(c_fin, 1e-3);
  }
}

// 9b. 모르는 일정한 힘 아래 등속 직진: current_state 에서도 d̂ 가 정상상태 오차를 없앤다
TEST(TorqueLinearizationMPC, DisturbanceEstimateRemovesOffset)
{
  auto run = [](bool enable) {
      TorqueModel md;
      TorqueTuning tu = current_tuning();
      tu.dist_enable = enable;
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      plant.force_ext << -1.0, 0, 0;
      const double v = 0.5;
      plant.x << 0, 0, 0, v, 0, 0;
      double err = 0.0;
      for (int k = 0; k < 500; ++k) {
        std::vector<TorqueRefPoint> ref;
        for (int j = 0; j <= tu.Np; ++j) {ref.push_back({TVec3(v * (k + j) * tu.Ts, 0, 0), TVec3(v, 0, 0)});}
        plant.step(mpc.step(plant.x, ref), tu.Ts);
        err = v * (k + 1) * tu.Ts - plant.x(0);
      }
      return err;
    };
  const double off = run(false), on = run(true);
  EXPECT_GT(off, 2e-4);
  EXPECT_LT(std::abs(on), 0.05 * off);
  std::printf("            [current_state] 정상상태 오차  d̂ 끔 %.3f mm   d̂ 켬 %.4f mm\n", off * 1e3, on * 1e3);
}

// 2026-11-21 몸체 좌표 외란 추정치: 원을 도는 동안 몸체 기준으로 일정한 옆 힘 (실차 8자의 원 안쪽 어긋남 꼴).
//   월드 좌표 d̂ 는 그 힘이 ω 로 도는 벡터라 느린 α 에서 지워지고, 몸체 좌표 d̂ 는 일정한 값으로 잡아 정상상태 옆 오차를 없앤다.
namespace
{
struct CircleRun
{
  double radial_mean = 0.0;   // 마지막 3 s 의 (|p − c| − R) 평균 [m] (+ 바깥)
  TVec6 d_last = TVec6::Zero();
};

CircleRun run_circle_body_force(bool enable, bool body, double alpha)
{
  TorqueModel md;
  TorqueTuning tu;
  tu.W << 20, 20, 10, 2, 2, 1;          // 지금 실차 Q (2026-11-21), R_Δ 100, R_U 10
  tu.w_delta = 100.0;
  tu.r_u = 10.0;
  tu.dist_enable = enable;
  tu.dist_alpha = alpha;
  tu.dist_accel_only = true;
  tu.dist_body_frame = body;
  tu.dist_skip_xy = -1.0;
  TorqueMPC mpc(md, tu);
  Plant plant{md};
  const double R = 0.5, v = 0.5, om = v / R;
  const TVec3 F_body(0.0, -2.0, 0.0);   // 몸체 기준 오른쪽 (원 바깥) 2 N
  plant.x << 0, 0, 0, v, 0, om;
  CircleRun out;
  int n = 0;
  const int steps = 900;                // 9 s (한 바퀴 6.3 s)
  for (int k = 0; k < steps; ++k) {
    const double th = plant.x(2), c = std::cos(th), s = std::sin(th);
    plant.force_ext << c * F_body(0) - s * F_body(1), s * F_body(0) + c * F_body(1), 0.0;
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, (k + j) * tu.Ts));}
    plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);
    if (k >= steps - 300) {
      out.radial_mean += std::hypot(plant.x(0), plant.x(1) - R) - R;
      ++n;
    }
  }
  out.radial_mean /= n;
  out.d_last = mpc.d_hat();
  return out;
}
}  // namespace

TEST(TorqueMPC, DisturbanceBodyFrameOnCircle)
{
  const auto off = run_circle_body_force(false, false, 0.01);
  const auto world = run_circle_body_force(true, false, 0.01);
  const auto body = run_circle_body_force(true, true, 0.01);
  std::printf("            원 위 몸체 옆 힘 2 N: 반경 어긋남  d̂ 끔 %+.1f mm  월드 α0.01 %+.1f mm  몸체 α0.01 %+.1f mm\n",
    off.radial_mean * 1e3, world.radial_mean * 1e3, body.radial_mean * 1e3);
  EXPECT_GT(off.radial_mean, 0.01);                                  // 끄면 바깥으로 1 cm 넘게 남는다
  EXPECT_LT(std::abs(body.radial_mean), 0.2 * off.radial_mean);       // 몸체 좌표: 1/5 아래
  EXPECT_LT(std::abs(body.radial_mean), 0.5 * std::abs(world.radial_mean));   // 월드 좌표 (같은 α) 보다 절반 아래
  // 몸체 좌표 d̂ 는 그 힘을 일정한 값으로 잡는다: d̂_ẏ / Ts ≈ F_y / m_eff
  TorqueModel md;
  EXPECT_NEAR(body.d_last(4) / 0.01, (-2.0) / md.m_eff(), 0.3 * std::abs((-2.0) / md.m_eff()));
}

// 포화 중 갱신 멈춤: 상한이 작아 τ 가 붙어 있는 동안은 d̂ 가 그대로다 (d_skipped 3). 끄면 갱신된다
TEST(TorqueMPC, DisturbanceFreezeOnSaturation)
{
  for (bool freeze : {true, false}) {
    TorqueModel md;
    TorqueTuning tu;
    tu.dist_freeze_on_sat = freeze;
    tu.dist_alpha = 1.0;
    tu.dist_skip_xy = -1.0;
    TorqueMPC mpc(md, tu);
    mpc.set_tau_bar_override(TVec4::Constant(0.01));
    Plant plant{md};
    plant.force_ext << 3.0, 0, 0;
    const auto ref = hold_ref(TVec3(0.5, 0, 0), 51);    // 0.5 m 앞 — 상한에 붙는다
    // 처음 몇 스텝은 변화율 제한 (Δτ̄ = τ̄/3) 으로 아직 상한 아래라 d̂ 가 갱신된다 — 상한에 붙은 뒤를 본다
    for (int k = 0; k < 6; ++k) {plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);}
    const TVec6 d0 = mpc.d_hat();
    plant.force_ext << -3.0, 2.0, 0;                       // 외란이 바뀌어도
    for (int k = 0; k < 5; ++k) {plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);}
    if (freeze) {
      EXPECT_EQ(mpc.stats().d_skipped, 3);
      EXPECT_LT((mpc.d_hat() - d0).cwiseAbs().maxCoeff(), 1e-15);   // 포화 중에는 그대로
    } else {
      EXPECT_EQ(mpc.stats().d_skipped, 0);
      EXPECT_GT((mpc.d_hat() - d0).cwiseAbs().maxCoeff(), 1e-4);    // 끄면 새 외란을 따라간다
    }
  }
}

// 상한: |â_d,xy| = |d̂_ṗ,xy| / Ts ≤ acc_max, |α_d| ≤ yaw_acc_max
TEST(TorqueMPC, DisturbanceAccelClamp)
{
  TorqueModel md;
  TorqueTuning tu;
  tu.dist_alpha = 1.0;
  tu.dist_accel_only = true;
  tu.dist_skip_xy = -1.0;
  tu.dist_acc_max = 0.5;
  tu.dist_yaw_acc_max = 2.0;
  TorqueMPC mpc(md, tu);
  Plant plant{md};
  plant.force_ext << 50.0, -30.0, 5.0;                  // 7.8·4.7 m/s², 52 rad/s² — 상한보다 훨씬 크다
  const auto ref = hold_ref(TVec3::Zero(), 51);
  for (int k = 0; k < 3; ++k) {plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);}
  EXPECT_LE(std::hypot(mpc.d_hat()(3), mpc.d_hat()(4)) / tu.Ts, 0.5 + 1e-9);
  EXPECT_LE(std::abs(mpc.d_hat()(5)) / tu.Ts, 2.0 + 1e-9);
  EXPECT_GT(std::hypot(mpc.d_hat()(3), mpc.d_hat()(4)) / tu.Ts, 0.49);   // 잘렸지 0 은 아니다
  // accel_only 꼴 (위치 = ½Ts·속도) 이 유지된다
  EXPECT_NEAR(mpc.d_hat()(0), 0.5 * tu.Ts * mpc.d_hat()(3), 1e-12);
}

// 2026-11-21 정상상태 Kalman 이득: 위치 잡음이 매우 크면 위치 무게 → 0 이고, 속도 측정만의 스칼라 식 α²/(1−α) = q Ts²/σ_v² 와 같다
TEST(TorqueMPC, DisturbanceKalmanGainMatchesScalarFormula)
{
  const double Ts = 0.01, qr = 0.3, sv = 0.029;
  const Eigen::RowVector2d K = TorqueMPC::steady_kalman_gain(Ts, qr, 1e6, sv);
  const double alpha = K(1) * Ts;                       // 등가 α = K_vel · H_v
  const double lhs = alpha * alpha / (1.0 - alpha), rhs = qr * qr * Ts * Ts * Ts / (sv * sv);
  EXPECT_LT(std::abs(K(0)) * 0.5 * Ts * Ts, 1e-9);
  EXPECT_NEAR(lhs, rhs, 1e-6 * rhs);
  // 실차 잡음 (위치 1.6 mm, 속도 29 mm/s) 이면 위치 몫은 속도 몫의 1 % 아래 (실차 잔차 212502, RESULTS 19 절)
  const Eigen::RowVector2d K2 = TorqueMPC::steady_kalman_gain(Ts, qr, 1.6e-3, sv);
  EXPECT_LT(K2(0) * 0.5 * Ts * Ts, 0.01 * K2(1) * Ts);
  std::printf("            Kalman: q %.2f σ_v %.3f → 등가 α %.4f (시정수 %.2f s)\n", qr, sv, alpha, Ts / alpha);
}

// Kalman 관측기 (몸체 좌표) 도 원 위 몸체 옆 힘의 정상상태 어긋남을 없앤다
TEST(TorqueMPC, DisturbanceKalmanBodyFrameOnCircle)
{
  auto run = [](bool enable) {
      TorqueModel md;
      TorqueTuning tu;
      tu.W << 20, 20, 10, 2, 2, 1;
      tu.w_delta = 100.0;
      tu.r_u = 10.0;
      tu.dist_enable = enable;
      tu.dist_kalman = true;
      tu.dist_body_frame = true;
      tu.dist_skip_xy = -1.0;
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      const double R = 0.5, v = 0.5;
      plant.x << 0, 0, 0, v, 0, v / R;
      double rad = 0.0; int n = 0;
      for (int k = 0; k < 900; ++k) {
        const double th = plant.x(2);
        plant.force_ext << 2.0 * std::sin(th), -2.0 * std::cos(th), 0.0;   // 몸체 오른쪽 2 N
        std::vector<TorqueRefPoint> ref;
        for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, (k + j) * tu.Ts));}
        plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);
        if (k >= 600) {rad += std::hypot(plant.x(0), plant.x(1) - R) - R; ++n;}
      }
      return rad / n;
    };
  const double off = run(false), on = run(true);
  std::printf("            Kalman 몸체 d̂: 반경 어긋남 끔 %+.1f mm  켬 %+.1f mm\n", off * 1e3, on * 1e3);
  EXPECT_GT(off, 0.01);
  EXPECT_LT(std::abs(on), 0.2 * off);
}

// 제약 있는 목표 입력: 기준이 상한 밖이면 U_s 를 [−τ̄, τ̄] 로 자른다
TEST(TorqueMPC, TargetProjectionClipsToBounds)
{
  for (bool proj : {false, true}) {
    TorqueModel md;
    TorqueTuning tu;
    tu.target_projection = proj;
    TorqueMPC mpc(md, tu);
    mpc.set_tau_bar_override(TVec4::Constant(0.05));
    std::vector<TorqueRefPoint> ref;
    for (int j = 0; j <= tu.Np; ++j) {           // 2 m/s² 가속 기준 — 필요한 토크 6.425·2·0.04/4 = 0.129 N·m > 0.05
      const double t = j * tu.Ts;
      ref.push_back({TVec3(t * t, 0, 0), TVec3(2.0 * t, 0, 0)});
    }
    std::vector<TVec3> acc(tu.Nc, TVec3(2.0, 0, 0));
    mpc.step(TVec6::Zero(), ref, &acc);
    const double umax = mpc.target_input().cwiseAbs().maxCoeff();
    if (proj) {EXPECT_LE(umax, 0.05 + 1e-12);} else {EXPECT_GT(umax, 0.1);}
  }
}


// 공통 설정: 실차 yaml 의 Q·R 과 몸체·Kalman d̂ (잡음 없는 플랜트라 skip_xy 끔)
static TorqueTuning ff_tuning(bool dist, bool est)
{
  TorqueTuning tu;
  tu.W << 20, 20, 10, 2, 2, 1;
  tu.w_delta = 100.0;
  tu.r_u = 10.0;
  tu.dist_enable = dist;
  tu.dist_kalman = true;
  tu.dist_body_frame = true;
  tu.dist_skip_xy = -1.0;
  tu.dist_acc_max = 0.5;
  tu.ff_normal_estimate = est;
  return tu;
}

// 원 하나 (반시계, R 0.5, v 0.5) 에 구심의 30 % 안쪽 힘. 한 원에서는 n̂_c 가 몸체 +y 로 고정이라 d_y 와 η 가 갈리지 않는다 —
//   합 d_y + η g (몸체 +y 쪽 모델 밖 가속도) 가 참값 β v²/R = 0.15 m/s² 로 가고 반경 오프셋이 없어지면 된다
TEST(TorqueMPC, FeedforwardNormalEstimateOnCircle)
{
  struct Out {double rad, rho, sum;};
  auto run = [](bool dist, bool est, double beta) {
      TorqueModel md;
      const TorqueTuning tu = ff_tuning(dist, est);
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      const double R = 0.5, v = 0.5;
      plant.x << 0, 0, 0, v, 0, v / R;
      double rad = 0.0, sum = 0.0; int n = 0;
      for (int k = 0; k < 2000; ++k) {
        const double dx = -plant.x(0), dy = R - plant.x(1), dn = std::hypot(dx, dy);
        const double Fin = beta * md.m_eff() * v * v / R;                    // 원 중심 쪽 모델 밖 힘
        plant.force_ext << Fin * dx / dn, Fin * dy / dn, 0.0;
        std::vector<TorqueRefPoint> ref;
        for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, (k + j) * tu.Ts));}
        plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);
        if (k >= 1500) {
          rad += std::hypot(plant.x(0), plant.x(1) - R) - R;
          sum += mpc.d_hat()(4) / tu.Ts + (1.0 - mpc.ff_normal_rho()) * mpc.stats().rho_g;
          ++n;
        }
      }
      return Out{rad / n, mpc.ff_normal_rho(), sum / n};
    };
  const Out off = run(false, false, 0.3), est_only = run(false, true, 0.3), both = run(true, true, 0.3), clean = run(true, true, 0.0);
  for (auto [nm, o] : {std::pair{"보상 없음     ", off}, {"ρ 추정만 씀  ", est_only}, {"d̂ + ρ 추정   ", both}, {"힘 없음 d̂+ρ  ", clean}}) {
    std::printf("            원 하나 %s: 반경 어긋남 %+.1f mm   ρ̂ %.3f   d̂_y + η̂ g %+.3f m/s²\n", nm, o.rad * 1e3, o.rho, o.sum);
  }
  EXPECT_LT(off.rad, -0.005);                                  // 안쪽 (반경이 작다) 으로 5 mm 넘게
  EXPECT_LT(std::abs(both.rad), 0.1 * std::abs(off.rad));
  EXPECT_NEAR(both.sum, 0.15, 0.02);
  EXPECT_DOUBLE_EQ(off.rho, 1.0);
  EXPECT_NEAR(clean.rho, 1.0, 0.05);                           // 모델이 맞으면 ρ̂ 가 1 에서 안 벗어난다
  EXPECT_LT(std::abs(clean.rad), 0.002);
}

// 8자 (원 두 개, 왼원 반시계 → 오른원 시계, 원점에서 +x): 시각 t 의 (포즈, 몸체 속도). 방향은 랩마다 0 → 2π → 0
static TorqueRefPoint eight(double R, double v, double t)
{
  const double om = v / R, T = 2.0 * M_PI / om, tt = std::fmod(t, 2.0 * T);
  if (tt < T) {return circle(R, v, tt);}
  const double ph = om * (tt - T);
  return {TVec3(R * std::sin(ph), -R * (1.0 - std::cos(ph)), 2.0 * M_PI - ph), TVec3(v, 0.0, -om)};
}

// 8자에서 d̂ 와 ρ 가 나눠 갖는지: 실차 8자에서 분해한 두 몫을 그대로 넣는다 (RESULTS 19: 몸체 y 잔차 왼원 +0.18, 오른원 0 = d ± s)
//   s = 구심에 비례하는 안쪽 힘 (β 0.3 → ρ 참값 0.7, R 0.5·v 0.5 에서 0.15 m/s²),  d = 몸체 왼쪽 일정 힘 0.09 m/s²
TEST(TorqueMPC, FeedforwardNormalEstimateOnEight)
{
  struct Out {double left, right, rho, dl, dr;};
  auto run = [](bool dist, bool est, double beta, double d_b) {
      TorqueModel md;
      const TorqueTuning tu = ff_tuning(dist, est);
      TorqueMPC mpc(md, tu);
      Plant plant{md};
      const double R = 0.5, v = 0.5, om = v / R, T = 2.0 * M_PI / om;
      plant.x << 0, 0, 0, v, 0, om;
      const int N = static_cast<int>(6 * 2 * T / tu.Ts);            // 6 랩
      double sl = 0, sr = 0, al = 0, ar = 0; int nl = 0, nr = 0;
      for (int k = 0; k < N; ++k) {
        const double t = k * tu.Ts, tt = std::fmod(t, 2.0 * T);
        const bool left = tt < T;
        const double cy = left ? R : -R, dx = -plant.x(0), dy = cy - plant.x(1), dn = std::hypot(dx, dy);
        const double Fin = beta * md.m_eff() * v * v / R, Fd = md.m_eff() * d_b, th = plant.x(2);
        plant.force_ext << Fin * dx / dn - Fd * std::sin(th), Fin * dy / dn + Fd * std::cos(th), 0.0;
        std::vector<TorqueRefPoint> ref;
        for (int j = 0; j <= tu.Np; ++j) {ref.push_back(eight(R, v, t + j * tu.Ts));}
        plant.step(mpc.step(plant.x, ref, nullptr, tu.Ts), tu.Ts);
        const double ph = std::fmod(tt, T) / T;
        if (t > 8 * T && ph > 0.25 && ph < 0.9) {                       // 마지막 2 랩, 원 진입 과도 뺀 구간
          const double e = std::hypot(plant.x(0), plant.x(1) - cy) - R;   // + = 바깥
          const double ay = mpc.d_hat()(4) / tu.Ts;
          if (left) {sl += e; al += ay; ++nl;} else {sr += e; ar += ay; ++nr;}
        }
      }
      return Out{sl / nl, sr / nr, mpc.ff_normal_rho(), al / nl, ar / nr};
    };
  const Out a = run(false, false, 0.3, 0.09), b = run(true, false, 0.3, 0.09), c = run(false, true, 0.3, 0.09),
    d = run(true, true, 0.3, 0.09), e = run(true, true, 0.0, 0.0);
  for (auto [nm, o] : {std::pair{"보상 없음   ", a}, {"d̂ 만       ", b}, {"ρ 추정만 씀", c}, {"d̂ + ρ 추정 ", d}, {"힘 없음     ", e}}) {
    std::printf("            8자 %s: 반경 어긋남 왼원 %+.1f mm  오른원 %+.1f mm   ρ̂ %.3f   몸체 â_d,y 왼원 %+.3f 오른원 %+.3f m/s²\n",
      nm, o.left * 1e3, o.right * 1e3, o.rho, o.dl, o.dr);
  }
  EXPECT_NEAR(d.rho, 0.7, 0.1);                               // 구심 몫은 ρ 가
  EXPECT_NEAR(d.dl, 0.09, 0.04);                              // 몸체 일정 몫은 d̂ 가 — 두 원에서 같은 값
  EXPECT_NEAR(d.dr, 0.09, 0.04);
  EXPECT_LT(std::abs(d.left) + std::abs(d.right), 0.5 * (std::abs(b.left) + std::abs(b.right)));
  EXPECT_NEAR(e.rho, 1.0, 0.05);
}

// 2026-11-22 바닥 토크 상한 (TorqueTuning::cap_on_ground, TorqueModel::ground_map)
TEST(TorqueModel, GroundMapForwardShareAndSymmetry)
{
  TorqueModel md;
  md.mass = 4.56;
  md.J_w = 8.9e-4;
  const Eigen::Matrix4d P = md.ground_map();
  EXPECT_TRUE(P.isApprox(P.transpose(), 1e-12));
  // 네 바퀴 같은 토크 (앞) 의 바닥 몫 = 1 − m_w/m_eff (닫힌 꼴) = 1 − 2.225/6.785
  const TVec4 ones = TVec4::Ones();
  const TVec4 g = P * ones;
  EXPECT_NEAR(g(0), 1.0 - md.mu() / md.m_eff(), 1e-12);
  EXPECT_NEAR(g(0), 0.672, 1e-3);
  // 바닥 토크로 만든 힘 = 몸체를 가속하는 힘: F τ_g = M_b M_eff⁻¹ F τ  (유한차분 아님, 식 대조)
  const TVec4 tau(0.1, -0.05, 0.08, 0.02);
  const TMat34 F = md.force_map();
  const TVec3 a = TVec3(1.0 / md.m_eff(), 1.0 / md.m_eff(), 1.0 / md.I_eff()).asDiagonal() * (F * tau);
  const TVec3 Fb = F * (P * tau);
  EXPECT_NEAR(Fb(0), md.mass * a(0), 1e-9);
  EXPECT_NEAR(Fb(1), md.mass * a(1), 1e-9);
  EXPECT_NEAR(Fb(2), md.inertia_z * a(2), 1e-9);
}

TEST(TorqueMPC, GroundCapBoundsGroundTorqueAndAllowsMoreWheelTorque)
{
  TorqueModel md;
  md.mass = 4.56;
  md.J_w = 8.9e-4;
  TorqueTuning tu;
  tu.limit_mode = TorqueLimitMode::Friction;
  tu.cap_on_ground = true;
  TorqueMPC mpc(md, tu);
  ASSERT_TRUE(mpc.cap_on_ground());
  const double tb = md.tau_friction_limit();
  const Eigen::Matrix4d P = md.ground_map();
  Plant plant{md};
  double tmax = 0.0, gmax = 0.0;
  for (int k = 0; k < 100; ++k) {
    const TVec4 tau = mpc.step(plant.x, hold_ref(TVec3(1.0, 0, 0), 51));     // 1 m 앞: 급가속 요구
    EXPECT_EQ(mpc.stats().fallback, 0);
    for (int j = 0; j < tu.Nc; ++j) {
      const TVec4 tj = mpc.solution().segment<4>(4 * j);
      EXPECT_LE((P * tj).cwiseAbs().maxCoeff(), tb + 1e-7) << "k=" << k << " j=" << j;
      EXPECT_LE(tj.cwiseAbs().maxCoeff(), md.tau_motor_limit() + 1e-9);
    }
    tmax = std::max(tmax, tau.cwiseAbs().maxCoeff());
    gmax = std::max(gmax, (P * tau).cwiseAbs().maxCoeff());
    plant.step(tau, tu.Ts);
  }
  EXPECT_NEAR(gmax, tb, 1e-6);                 // 바닥 몫이 상한에 붙는다
  EXPECT_GT(tmax, 1.4 * tb);                   // 바퀴 토크는 τ̄ 를 넘는다 (앞 고른 토크면 τ̄/0.672 ≈ 1.49 τ̄)
  // limit_mode motor_only 면 cap_on_ground 를 켜도 안 쓰인다
  TorqueTuning tub;
  tub.cap_on_ground = true;
  TorqueMPC mpcb(md, tub);
  EXPECT_FALSE(mpcb.cap_on_ground());
}

// 2026-11-23 구동계 방향 보정 (torque_plant.yaml drive). δ0 는 force_map 의 xy 행만 R(δ0) 로 돌리고 (M_z 행·영공간 그대로),
//   그래서 G(θ) 의 xy 행 = 보정 없는 G(θ + δ0) 의 xy 행. 기본 0 이면 예전과 같은 값.
TEST(TorqueModel, DriveYawRotatesForceMapXYRowsOnly)
{
  TorqueModel nom, md;
  md.drive_yaw = 2.4 * M_PI / 180.0;
  const TMat34 F0 = nom.force_map(), F = md.force_map();
  const double c = std::cos(md.drive_yaw), s = std::sin(md.drive_yaw);
  EXPECT_LT((F.row(0) - (c * F0.row(0) - s * F0.row(1))).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((F.row(1) - (s * F0.row(0) + c * F0.row(1))).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((F.row(2) - F0.row(2)).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((F * TorqueMPC::null_dir()).cwiseAbs().maxCoeff(), 1e-12);
  for (double th : {0.0, 0.7, -2.1}) {
    const TMat34 G = md.G(th), Gn = nom.G(th + md.drive_yaw);
    EXPECT_LT((G.topRows<2>() - Gn.topRows<2>()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((G.row(2) - nom.G(th).row(2)).cwiseAbs().maxCoeff(), 1e-12);
  }
  // 앞으로 1 N 을 내려고 고른 토크 (보정 없는 모델의 최소 노름) 는 실제로 옆 sin δ0 = 4.2 % 를 낸다 — 보정 모델의 F⁺ 는 정확히 1 N
  const TVec4 tau0 = F0.transpose() * (F0 * F0.transpose()).inverse() * TVec3(1, 0, 0);
  const TVec3 f_real = F * tau0;
  EXPECT_NEAR(f_real(1), std::sin(md.drive_yaw), 1e-12);
  const TVec4 tau1 = F.transpose() * (F * F.transpose()).inverse() * TVec3(1, 0, 0);
  EXPECT_LT(((F * tau1) - TVec3(1, 0, 0)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(TorqueModel, CorrectWheelVelocity)
{
  TorqueModel md;
  const TVec3 v(0.6, 0.0, 0.3);
  EXPECT_EQ(md.correct_wheel_velocity(v, 1.8), v);                       // 기본 0: 그대로
  md.drive_yaw = 2.4 * M_PI / 180.0;
  TVec3 out = md.correct_wheel_velocity(v, 0.0);
  EXPECT_NEAR(out(0), 0.6 * std::cos(md.drive_yaw), 1e-12);
  EXPECT_NEAR(out(1), 0.6 * std::sin(md.drive_yaw), 1e-12);
  EXPECT_DOUBLE_EQ(out(2), 0.3);                                         // ω_z 성분은 안 건드림
  md.slip_coef = -2.5 * M_PI / 180.0;
  out = md.correct_wheel_velocity(TVec3(0.9, 0.0, 0.0), 1.8);            // P1 최고속 CCW: a_c = 1.62 → δ = 2.4 − 4.05 = −1.65°
  EXPECT_NEAR(std::atan2(out(1), out(0)), (2.4 - 2.5 * 1.62) * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(std::hypot(out(0), out(1)), 0.9, 1e-12);                   // 크기는 그대로 (회전만)
  out = md.correct_wheel_velocity(TVec3(1.0, 0.0, 0.0), -3.0);           // a_c −3 → slip_ac_max 2 로 자름 → δ = 2.4 + 5.0°
  EXPECT_NEAR(std::atan2(out(1), out(0)), (2.4 + 2.5 * 2.0) * M_PI / 180.0, 1e-12);
}

// 2026-11-23 하중 이동: 수직력 식의 부호·보존·아래 한계
TEST(TorqueModel, WheelNormalForcesLoadTransfer)
{
  TorqueModel md;
  md.mass = 4.56; md.wheel_base = 0.1724; md.robot_width = 0.214; md.gravity = 9.81;
  md.wheel_load_share = TVec4(0.2388, 0.2392, 0.2608, 0.2612);
  const TVec4 n0 = md.wheel_load_share * md.mass * md.gravity;
  EXPECT_LT((md.wheel_normal_forces(1.0, 2.0, 0.0, 0.2) - n0).cwiseAbs().maxCoeff(), 1e-12);   // h = 0 → 정지
  const double h = 0.105;
  const TVec4 n = md.wheel_normal_forces(1.5, 2.0, h, 0.0);       // 앞으로 가속 + 왼쪽 구심 (왼쪽 선회)
  EXPECT_NEAR(n.sum(), md.mass * md.gravity, 1e-9);                // 합 보존
  const double dx = md.mass * 1.5 * h / (2 * md.wheel_base), dy = md.mass * 2.0 * h / (2 * md.robot_width);
  EXPECT_NEAR(n(0), n0(0) - dx - dy, 1e-9);                        // FL: 앞·안쪽 → 가장 가볍다
  EXPECT_NEAR(n(3), n0(3) + dx + dy, 1e-9);                        // RR: 뒤·바깥 → 가장 무겁다
  EXPECT_NEAR(n(1), n0(1) - dx + dy, 1e-9);
  EXPECT_NEAR(n(2), n0(2) + dx - dy, 1e-9);
  const TVec4 nc = md.wheel_normal_forces(10.0, 10.0, h, 0.2);     // 아주 센 가속 → 아래 한계 0.2 × 정지
  EXPECT_NEAR(nc(0), 0.2 * n0(0), 1e-12);
}

// 2026-11-23 하중 이동: 왼쪽 원 위에서 목표 입력이 같은 몸체 힘을 내되 (F U_s 같음) 가벼운 안쪽 바퀴에 덜 싣고, friction 상한이 안쪽만 준다
TEST(TorqueMPC, LoadTransferWeightsTargetInputAndCaps)
{
  TorqueModel md;
  md.mass = 4.56; md.wheel_base = 0.1724; md.robot_width = 0.214;
  md.mu_hat = 0.5;
  TorqueTuning tu;
  tu.limit_mode = TorqueLimitMode::Friction;
  const double R = 0.4, v = 1.0;                                    // 구심 2.5 m/s²
  std::vector<TorqueRefPoint> ref;
  for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, j * tu.Ts));}
  TVec6 x;
  x << 0, 0, 0, v, 0, v / R;
  TorqueMPC off(md, tu);
  off.step(x, ref);
  tu.load_com_height = 0.105;
  TorqueMPC on(md, tu);
  on.step(x, ref);
  const TMat34 F = md.force_map();
  const TVec4 u0 = off.target_input().head<4>(), u1 = on.target_input().head<4>();
  EXPECT_LT((F * u0 - F * u1).cwiseAbs().maxCoeff(), 1e-9);         // 같은 몸체 힘
  // 왼쪽 선회 → 왼쪽 (FL, RL) 이 가볍다: 가중 배분은 왼쪽 |τ| 를 줄이고 오른쪽을 늘린다
  EXPECT_LT(std::abs(u1(0)), std::abs(u0(0)));
  EXPECT_LT(std::abs(u1(2)), std::abs(u0(2)));
  EXPECT_GT(std::abs(u1(1)), std::abs(u0(1)));
  EXPECT_GT(std::abs(u1(3)), std::abs(u0(3)));
  const TVec4 n = on.normal_forces_now();
  EXPECT_LT(n(0), n(1));                                            // FL < FR (안쪽 < 바깥)
  const auto & cap = on.tau_bar_steps();
  EXPECT_LT(cap(0, 0), off.tau_bar()(0));                           // 안쪽 상한은 줄고
  EXPECT_GT(cap(1, 0), off.tau_bar()(1));                           // 바깥은 는다
  EXPECT_LT((off.tau_bar_steps().col(0) - off.tau_bar()).cwiseAbs().maxCoeff(), 1e-12);   // 끄면 정지 상한 그대로
}

// ── 2026-11-23 슬립 계수 η_s (torque_mpc.hpp 머리주석) ──────────────
// 10·15·20 %: s = 0.05/ln19 이면 η 0.95 at 0.10, 0.5 at 0.15, 0.05 at 0.20 (부호 무관)
TEST(SlipEta, CurvePoints)
{
  const double s = 0.05 / std::log(19.0);
  EXPECT_NEAR(SlipEta::eta_of(0.10, 0.15, s), 0.95, 1e-12);
  EXPECT_NEAR(SlipEta::eta_of(0.15, 0.15, s), 0.5, 1e-12);
  EXPECT_NEAR(SlipEta::eta_of(0.20, 0.15, s), 0.05, 1e-12);
  EXPECT_NEAR(SlipEta::eta_of(-0.20, 0.15, s), 0.05, 1e-12);
  EXPECT_NEAR(SlipEta::eta_of(0.0, 0.15, s), 1.0, 1e-3);
  EXPECT_LT(SlipEta::eta_of(1.0, 0.15, s), 1e-20);
}

// slip_logger.slip_ratio 와 같은 값 (구동 +, 제동 −, 역회전 −2, 음의 방향, ω_ε 아래)
TEST(SlipEta, SlipRatioMatchesLogger)
{
  const TVec4 l = SlipEta::slip_ratio(TVec4(12, 0, -10, -12), TVec4(10, 10, 10, -10), 2.5);
  EXPECT_NEAR(l(0), 2.0 / 12.0, 1e-12);
  EXPECT_NEAR(l(1), -1.0, 1e-12);
  EXPECT_NEAR(l(2), -2.0, 1e-12);
  EXPECT_NEAR(l(3), 2.0 / 12.0, 1e-12);
  EXPECT_NEAR(SlipEta::slip_ratio(TVec4(0.5, 0, 0, 0), TVec4::Zero(), 2.5)(0), 0.2, 1e-12);   // 분모 ω_ε
}

// 1 차 LPF: λ 계단 0.5 에 y_n = 0.5 (1 − (1 − α)^n), α = Δt/(τ + Δt). valid 아닌 바퀴는 그대로
TEST(SlipEta, LowPassStepAndInvalidHold)
{
  SlipEta se(SlipEtaParams{});
  const double tau = 1.0 / (2.0 * M_PI * 10.0), a = 0.01 / (tau + 0.01);
  const bool all[4] = {true, true, true, true};
  const bool fr_bad[4] = {true, false, true, true};
  const TVec4 om = TVec4::Constant(20.0), nom = TVec4::Constant(10.0);   // λ = 10/20 = 0.5
  for (int n = 1; n <= 5; ++n) {
    se.update(om, nom, n <= 2 ? all : fr_bad, 0.01);
    EXPECT_NEAR(se.lambda_f()(0), 0.5 * (1.0 - std::pow(1.0 - a, n)), 1e-12) << n;
  }
  EXPECT_NEAR(se.lambda_f()(1), 0.5 * (1.0 - std::pow(1.0 - a, 2)), 1e-12);   // 3 번째부터 멈춤
  EXPECT_NEAR(se.eta()(0), SlipEta::eta_of(se.lambda_f()(0), 0.15, 0.017), 1e-12);
  se.reset();
  EXPECT_EQ(se.eta(), TVec4::Ones());
  EXPECT_THROW(SlipEta(SlipEtaParams{0.15, 0.0}), std::invalid_argument);
}

// ω_nom = T⁻¹ v 는 wheel_body_velocity (T) 의 역이고 Python TorquePlant.Tinv (k/r = 4.83) 와 같다
TEST(TorqueModel, WheelNominalSpeedsInvertKinematics)
{
  TorqueModel md;
  md.wheel_base = 0.1724; md.robot_width = 0.214;
  const TVec3 v(0.7, -0.3, 1.2);
  EXPECT_LT((md.wheel_body_velocity(md.wheel_nominal_speeds(v)) - v).cwiseAbs().maxCoeff(), 1e-12);
  const TVec4 w = md.wheel_nominal_speeds(TVec3(0, 0, 1));
  EXPECT_NEAR(w(0), -4.83, 1e-12);
  EXPECT_NEAR(w(1), 4.83, 1e-12);
  EXPECT_NEAR(w(2), -4.83, 1e-12);
  EXPECT_NEAR(w(3), 4.83, 1e-12);
}

// η_s = 1 이면 예전과 같은 해, η_s,FL = 0 이면 그 주기 |τ_FL| = 0 (변화율보다 상자 우선, QP 성공) 이고
// U_s 는 같은 몸체 힘을 나머지 세 바퀴로 (가중 배분)
TEST(TorqueMPC, SlipScaleCapsWheelAndShiftsTarget)
{
  TorqueModel md;
  md.mass = 4.56; md.wheel_base = 0.1724; md.robot_width = 0.214;
  md.mu_hat = 1.0;
  TorqueTuning tu;
  tu.limit_mode = TorqueLimitMode::Friction;
  const double R = 0.6, v = 0.8;
  std::vector<TorqueRefPoint> ref;
  for (int j = 0; j <= tu.Np; ++j) {ref.push_back(circle(R, v, j * tu.Ts));}
  TVec6 x;
  x << 0.01, -0.01, 0.0, 0.5, 0.0, v / R;          // 기준보다 느리다 → 가속 토크
  TorqueMPC a(md, tu), b(md, tu);
  b.set_slip_scale(TVec4::Ones());
  for (int k = 0; k < 5; ++k) {
    const TVec4 ta = a.step(x, ref), tb = b.step(x, ref);
    EXPECT_LT((ta - tb).cwiseAbs().maxCoeff(), 1e-12);
  }
  const TVec4 before = b.tau_prev();
  ASSERT_GT(std::abs(before(0)), 0.05);
  const TVec4 us_a = a.target_input().head<4>();
  b.set_slip_scale(TVec4(0.0, 1.0, 1.0, 1.0));
  const TVec4 t1 = b.step(x, ref);
  a.step(x, ref);
  EXPECT_EQ(b.stats().fallback, 0);
  EXPECT_LE(std::abs(t1(0)), 1e-9);
  EXPECT_LE(b.tau_bar_steps().col(0)(0), 1e-12);
  EXPECT_NEAR(b.tau_bar_steps().col(0)(1), b.tau_bar()(1), 1e-12);
  const TMat34 F = md.force_map();
  const TVec4 us_b = b.target_input().head<4>();
  EXPECT_LT((F * us_a - F * us_b).cwiseAbs().maxCoeff(), 1e-9);    // 같은 몸체 힘
  EXPECT_LT(std::abs(us_b(0)), 0.01 * std::abs(us_a(0)) + 1e-9);   // FL 몫은 floor² 만큼만
}
