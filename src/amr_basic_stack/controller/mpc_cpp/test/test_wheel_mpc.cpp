// amr_mpc_cpp 단위 시험.  Python 판과의 수치 일치는 tools/make_parity_fixture.py + wheel_mpc_replay 가 따로 본다.
#include "amr_mpc_cpp/reference.hpp"
#include "amr_mpc_cpp/wheel_mpc.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <random>
#include <vector>

using namespace amr_mpc_cpp;

namespace
{
WheelMPC make(double Ts, int Np, int Nc, double theta)
{
  WheelModel md;
  md.theta = theta;
  WheelTuning tu;
  tu.Ts = Ts; tu.Np = Np; tu.Nc = Nc;
  return WheelMPC(md, tu);
}
}  // namespace

TEST(Mecanum, InverseAndNullSpace)
{
  const Mecanum m = mecanum_matrices(0.04, 0.1897, 0.207);
  EXPECT_LT((m.T_fwd * m.T_inv - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((m.T_fwd * m.null_dir).cwiseAbs().maxCoeff(), 1e-12);
  // n = ±[1, 1, −1, −1]/2 (앞 두 바퀴 vs 뒤 두 바퀴)
  const Vec4 n = m.null_dir * (m.null_dir(0) > 0 ? 1.0 : -1.0);
  EXPECT_LT((n - Vec4(0.5, 0.5, -0.5, -0.5)).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(Model, RejectsNonIntegerDelay)
{
  EXPECT_THROW(make(0.01, 80, 20, 0.045), std::invalid_argument);
  EXPECT_NO_THROW(make(0.01, 80, 20, 0.040));
  EXPECT_NO_THROW(make(0.01, 80, 20, 0.0));
}

// d 단 shift register 가 ω_e(k) = a ω_e(k−1) + b u(k−1−d) 를 정확히 낸다, 지연 블록이 nilpotent 다
TEST(Model, DelayChainMatchesExplicitRecursion)
{
  for (int d : {0, 1, 2, 4, 8}) {
    WheelMPC m = make(0.01, 80, 20, 0.01 * d);
    ASSERT_EQ(m.d(), d);
    MatX A_m, B_m, C_m;
    m.plant(0.3, 0.5, 0.1, A_m, B_m, C_m);
    ASSERT_EQ(A_m.rows(), 7 + 4 * d);
    std::mt19937 rng(d);
    std::normal_distribution<double> nd;
    VecX xm = VecX::Zero(A_m.rows());
    Vec4 we = Vec4::Zero();
    std::vector<Vec4> u_hist;
    double worst = 0.0;
    for (int k = 0; k < 60; ++k) {
      const Vec4 u(nd(rng), nd(rng), nd(rng), nd(rng));
      u_hist.push_back(u);
      xm = A_m * xm + B_m * u;
      const int kd = static_cast<int>(u_hist.size()) - 1 - d;      // ω_e(k+1) = a ω_e(k) + b u(k−d)
      we = m.a().cwiseProduct(we) + m.b().cwiseProduct(kd >= 0 ? u_hist[kd] : Vec4::Zero());
      worst = std::max(worst, (xm.segment<4>(3) - we).cwiseAbs().maxCoeff());
    }
    EXPECT_LT(worst, 1e-12) << "d=" << d;
    if (d > 0) {
      const MatX Sz = A_m.bottomRightCorner(4 * d, 4 * d);
      MatX P = MatX::Identity(4 * d, 4 * d);
      for (int i = 0; i < d; ++i) {P = Sz * P;}
      EXPECT_EQ(P.cwiseAbs().maxCoeff(), 0.0) << "S^d ≠ 0, d=" << d;
    }
  }
}

// Wang 증강계 (Δx_m, y) 가 원 플랜트와 같은 출력을 낸다 (python model._check [5] 와 같은 시험)
TEST(Model, AugmentedMatchesPlant)
{
  for (int d : {0, 1, 4}) {
    WheelMPC m = make(0.01, 80, 20, 0.01 * d);
    MatX A_m, B_m, C_m, A, B, C;
    m.plant(0.3, 0.5, 0.0, A_m, B_m, C_m);
    m.augment(0.3, 0.5, 0.0, A, B, C);
    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, 0.5);
    VecX xm_prev = VecX::NullaryExpr(A_m.rows(), [&]() {return 0.1 * nd(rng);});
    Vec4 ue_prev(nd(rng), nd(rng), nd(rng), nd(rng));
    VecX xm = A_m * xm_prev + B_m * ue_prev;
    VecX x(A.rows());
    x << xm - xm_prev, C_m * xm;
    double worst = 0.0;
    for (int k = 0; k < 40; ++k) {
      const Vec4 ue(nd(rng), nd(rng), nd(rng), nd(rng));
      const Vec4 du = ue - ue_prev;
      ue_prev = ue;
      xm = A_m * xm + B_m * ue;
      x = A * x + B * du;
      worst = std::max(worst, (C_m * xm - C * x).cwiseAbs().maxCoeff());
    }
    EXPECT_LT(worst, 1e-10) << "d=" << d;
  }
}

// predict() 의 (F, Φ) 가 증강계를 직접 굴린 출력과 같다 — LTV 누적곱·제어지평 처리 검사.
// Nc 이후에는 마지막 증분 Δu_{Nc−1} 을 매 스텝 다시 적용한다 (교재의 Δu = 0 과 다르다 — body/mpc.py:202 의 의도적 이탈)
TEST(Model, PredictionMatchesRollout)
{
  WheelMPC m = make(0.01, 30, 6, 0.04);
  std::vector<RefPoint> ref;
  for (int i = 0; i < 30; ++i) {ref.emplace_back(0.2 + 0.01 * i, 0.4, 0.05);}
  MatX F, Phi;
  m.predict(ref, F, Phi);
  std::mt19937 rng(3);
  std::normal_distribution<double> nd;
  const int n = m.n_m() + 3;
  VecX x0 = VecX::NullaryExpr(n, [&]() {return nd(rng);});
  VecX dU = VecX::NullaryExpr(4 * 6, [&]() {return nd(rng);});
  const VecX Y = F * x0 + Phi * dU;
  VecX x = x0;
  double worst = 0.0;
  for (int i = 0; i < 30; ++i) {
    MatX A, B, C;
    m.augment(ref[i](0), ref[i](1), ref[i](2), A, B, C);
    const Vec4 du = dU.segment<4>(4 * std::min(i, 5));
    x = A * x + B * du;
    worst = std::max(worst, (C * x - Y.segment<3>(3 * i)).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(worst, 1e-9);
}

// 관측기: K=1 은 측정 그대로, 무잡음이면 K 와 무관하게 플랜트를 정확히 재현 (d=4)
TEST(Observer, PassThroughAndExactReproduction)
{
  const Vec4 a = Vec4::Constant(std::exp(-0.01 / 0.1)), b = Vec4::Constant(1.0 - std::exp(-0.01 / 0.1));
  const int d = 4;
  {
    VelocityObserver ob(a, b, d, 1.0);
    for (int k = 0; k < 20; ++k) {
      const Vec4 vm = Vec4::Constant(std::sin(k));
      EXPECT_LT((ob.update(vm) - vm).cwiseAbs().maxCoeff(), 1e-15);
      ob.push(Vec4::Constant(std::cos(k)));
    }
  }
  for (double K : {0.0, 0.04, 1.0}) {
    VelocityObserver ob(a, b, d, K);
    Vec4 v = Vec4::Zero();
    std::deque<Vec4> hist(d, Vec4::Zero());
    double worst = 0.0;
    for (int k = 0; k < 300; ++k) {
      const Vec4 u(0.3 * std::sin(0.05 * k), 0.1, -0.2, 0.05 * k);
      worst = std::max(worst, (ob.update(v) - v).cwiseAbs().maxCoeff());
      ob.push(u);
      hist.push_back(u);
      v = a.cwiseProduct(v) + b.cwiseProduct(hist.front());
      hist.pop_front();
    }
    EXPECT_LT(worst, 1e-9) << "K=" << K;
  }
}

// 100 Hz, d=4 폐루프: python demo() 와 같은 판정 (4 s 이후 오차 < 2 cm, 바퀴 한계 안), QP 실패 없음
TEST(ClosedLoop, Circle100HzWithDelay)
{
  WheelMPC m = make(0.01, 80, 20, 0.04);
  const double Ts = 0.01, V = 0.5, R = 1.0, om = V / R;
  const int dp = 4;
  Vec3 X(0.0, -0.15, 0.0);
  Vec4 w = Vec4::Zero(), w_prev = Vec4::Zero();
  std::deque<Vec4> hist(dp, Vec4::Zero());
  std::vector<RefPoint> ref(80);
  double e_late = 0.0, wmax = 0.0;
  int bad = 0;
  for (int k = 0; k < 1600; ++k) {
    const double t = k * Ts;
    for (int i = 0; i < 80; ++i) {ref[i] = Vec3(om * (t + i * Ts), V, 0.0);}
    const Vec3 Xr(R * std::sin(om * t), R * (1 - std::cos(om * t)), om * t), ur(V, 0.0, om);
    const Vec4 wc = m.step(X, w, w_prev, ref, Xr, ur);
    bad += m.stats().exitflag < 1;
    wmax = std::max(wmax, wc.cwiseAbs().maxCoeff());
    w_prev = wc;
    hist.push_back(wc);
    w = m.a().cwiseProduct(w) + m.b().cwiseProduct(hist.front());
    hist.pop_front();
    X += Ts * body_kinematics(X, m.mecanum().T_fwd * w);
    const double tn = t + Ts;
    if (k >= 400) {e_late = std::max(e_late, std::hypot(X(0) - R * std::sin(om * tn), X(1) - R * (1 - std::cos(om * tn))));}
  }
  EXPECT_LT(e_late, 0.02);
  EXPECT_LE(wmax, m.w_max() + 1e-9);
  EXPECT_EQ(bad, 0);
}

// 바퀴 한계·가속 한계가 걸리는 상황에서도 명령이 제약 안이다
TEST(Constraints, RespectedUnderSaturation)
{
  WheelMPC m = make(0.01, 80, 20, 0.04);
  std::vector<RefPoint> ref(80, RefPoint(0.0, 1.1, 0.0));
  Vec4 w_prev = Vec4::Zero();
  for (int k = 0; k < 200; ++k) {
    const Vec4 wc = m.step(Vec3(-2.0, 1.0, 0.8), w_prev, w_prev, ref, Vec3::Zero(), Vec3(1.1, 0.0, 0.0));
    EXPECT_LE(wc.cwiseAbs().maxCoeff(), m.w_max() + 1e-9);
    EXPECT_LE((wc - w_prev).cwiseAbs().maxCoeff(), m.dw_max() + 1e-6) << "k=" << k;
    EXPECT_GE(m.stats().exitflag, 1);
    w_prev = wc;
  }
}

TEST(Reference, HoldLatchHysteresis)       // test_reference_horizon.py::test_hold_latch_hysteresis 와 같은 열
{
  const double deg = M_PI / 180.0;
  HoldLatch L(0.01, 2 * deg, 0.03, 5 * deg);
  EXPECT_FALSE(L.update(false, Vec3(0, 0, 0)));
  EXPECT_FALSE(L.update(true, Vec3(0.02, 0, 0)));
  EXPECT_TRUE(L.update(true, Vec3(0.005, 0, 1 * deg)));
  EXPECT_TRUE(L.update(true, Vec3(0.02, 0, 0)));
  EXPECT_FALSE(L.update(true, Vec3(0.035, 0, 0)));
  EXPECT_FALSE(L.update(true, Vec3(0.02, 0, 0)));
  EXPECT_TRUE(L.update(true, Vec3(0, 0, 0)));
  EXPECT_FALSE(L.update(true, Vec3(0, 0, 6 * deg)));
  EXPECT_FALSE(L.update(true, Vec3(0, 0, 0), false));
  EXPECT_TRUE(L.update(true, Vec3(0, 0, 0), true));
  EXPECT_TRUE(L.update(true, Vec3(0, 0, 0), false));
  EXPECT_TRUE(L.update(true, Vec3(0, 0, -1 * deg)));
  EXPECT_FALSE(L.update(false, Vec3(0, 0, 0)));
  EXPECT_THROW(HoldLatch(0.03, 2 * deg, 0.01, 5 * deg), std::invalid_argument);
}

TEST(Reference, BackwardExtrapolationExactOnCircle)
{
  // 상수 twist 원호: 첫 샘플에서 뒤로 정확 적분하면 원 위의 점과 일치 (직선 외삽이면 수 mm 오차)
  const double V = 0.5, R = 1.0, om = V / R, dt = 0.2;
  std::vector<ReferenceHorizon::Row> rows;
  for (int i = 1; i <= 20; ++i) {
    const double th = om * i * dt;
    rows.push_back({R * std::sin(th), R * (1 - std::cos(th)), th, V, 0.0, om});
  }
  ReferenceHorizon h(0.0, dt, rows);
  const auto [Xr, ur] = h.at(0.05);
  const double th = om * 0.05;
  EXPECT_LT(std::hypot(Xr(0) - R * std::sin(th), Xr(1) - R * (1 - std::cos(th))), 1e-9);
  EXPECT_LT(std::abs(wrap_angle(Xr(2) - th)), 1e-9);
  EXPECT_THROW(ReferenceHorizon(0.0, 0.0, rows), std::invalid_argument);
  EXPECT_THROW(ReferenceHorizon(0.0, 0.2, {}), std::invalid_argument);
}

// 포즈 외삽: 상수 twist 원호를 정확히 따라간다 (wheel_mpc_node 의 50 Hz 포즈 → 100 Hz 틱 보정)
TEST(Pose, ExtrapolationExactOnArc)
{
  const double V = 0.5, R = 1.0, om = V / R;
  for (double t0 : {0.0, 1.3, 4.0}) {
    const Vec3 X0(R * std::sin(om * t0), R * (1 - std::cos(om * t0)), wrap_angle(om * t0));
    for (double dt : {0.0, 0.01, 0.02, 0.3}) {
      const Vec3 X = extrapolate_pose(X0, Vec3(V, 0.0, om), dt);
      const double t = t0 + dt;
      EXPECT_LT(std::hypot(X(0) - R * std::sin(om * t), X(1) - R * (1 - std::cos(om * t))), 1e-12);
      EXPECT_LT(std::abs(wrap_angle(X(2) - om * t)), 1e-12);
    }
  }
  // 회전 없는 경우 (φ = 0 분기): 몸체 프레임 횡이동
  const Vec3 X = extrapolate_pose(Vec3(1.0, 2.0, M_PI / 2), Vec3(0.0, 0.4, 0.0), 0.5);
  EXPECT_LT((X - Vec3(0.8, 2.0, M_PI / 2)).cwiseAbs().maxCoeff(), 1e-12);
}
