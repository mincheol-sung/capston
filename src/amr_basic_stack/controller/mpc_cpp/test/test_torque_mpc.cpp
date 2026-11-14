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
  EXPECT_DOUBLE_EQ(md.mu(), 0.0);                       // 기본: 바퀴 관성 끔
  md.use_wheel_inertia = true;
  EXPECT_NEAR(md.mu(), 2.425, 1e-3);                    // 1판 식-3
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
  // 1판 식-14 수치 예: θ_r = 90°, u_r = [1, 0, 0.417] → w_xy = [−0.157, 0]
  const TVec3 w = md.w(TVec3(0.0, 1.0, 0.417));
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

// 1판 07절의 손 계산 예: Np = Nc = 1, 정지, 1 cm 앞 → 0.0275 N·m
TEST(TorqueMPC, HandExampleOneStep)
{
  TorqueModel md;
  md.use_wheel_inertia = true;             // 예제가 m_eff = 6.425 kg 기준
  TorqueTuning tu;
  tu.Np = 1; tu.Nc = 1; tu.constrained = false;
  tu.W << 100, 100, 100, 1, 1, 1;
  tu.w_delta = 1e-3; tu.q_null = 1e-2;
  TorqueMPC mpc(md, tu);
  std::vector<TorqueRefPoint> ref = {{TVec3::Zero(), TVec3::Zero()}, {TVec3(0.01, 0, 0), TVec3::Zero()}};
  const TVec4 tau = mpc.step(TVec6::Zero(), ref);
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(tau(i), 0.0275, 2e-4);}
  EXPECT_NEAR(TorqueMPC::null_dir().dot(tau), 0.0, 1e-12);
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
  md.use_wheel_inertia = GetParam();
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
  const double f = 4.0 * (v * v / R);      // m ω v = 1.67 N (몸체 질량 4 kg — 바퀴 관성과 무관)
  const double want = 0.04 / 4.0 * f;      // ± r/4 · F_y
  EXPECT_NEAR(tau(0), -want, 0.02 * want);
  EXPECT_NEAR(tau(1), want, 0.02 * want);
  EXPECT_NEAR(tau(2), want, 0.02 * want);
  EXPECT_NEAR(tau(3), -want, 0.02 * want);
  EXPECT_NEAR(TorqueMPC::null_dir().dot(tau), 0.0, 1e-6);
  EXPECT_LT(emax, 1e-3);                   // 등속 선회 오차 1 mm 미만
  EXPECT_EQ(mpc.stats().fallback, 0);
}
INSTANTIATE_TEST_SUITE_P(WheelInertia, SteadyTurn, ::testing::Values(false, true));

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
  TorqueModel md;
  md.use_wheel_inertia = true;
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

// 2. 야코비안이 torque_dynamics 의 중앙 유한차분과 같다 (바퀴 관성 켬, 움직이는 상태, τ₀ ≠ 0)
TEST(TorqueLinearizationModel, JacobianMatchesFiniteDifference)
{
  TorqueModel md;
  md.use_wheel_inertia = true;
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
  for (bool inertia : {false, true}) {
    TorqueModel md;
    md.use_wheel_inertia = inertia;
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
  md.use_wheel_inertia = GetParam();
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
  const double want = 0.04 / 4.0 * 4.0 * (v * v / R);
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
INSTANTIATE_TEST_SUITE_P(WheelInertia, SteadyTurnCurrent, ::testing::Values(false, true));

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
