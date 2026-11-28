// Q 후보 걸러내기 (2026-11-27): 무제약 MPC 이득 (amr_mpc_cpp::unconstrained_gain — 실제 풀이와 1e-8 로 같음, gtest UnconstrainedGainMatchesSolver)
// 을 공칭 모델 (θ = 0, M_eff⁻¹ F, ZOH Ts) 에 닫아 본다. 입력 지연 d 틱 (0·2) 은 지연 버퍼 상태로 붙인다.
//   δτ_k = −K_x e_k + K_τ δτ_{k−1}   (e = x − x_r,  δτ = τ − U_s)
// 지표: 닫힌 고리 극 (가장 느린 시정수 · 최대 |극|), 일정 외란 가속도 0.1 m/s² 의 정상 위치 오프셋, 원을 돌 때처럼 1.6 rad/s 로 도는
//       외란 (몸체에 붙은 외란이 map 에서 도는 것) 에 대한 위치 오차 진폭, 이득 크기 (오차 1 cm · 1 cm/s · 1° · 1 °/s 에 바퀴 토크 몇 mN·m).
#include <amr_mpc_cpp/torque_mpc.hpp>
#include <Eigen/Eigenvalues>
#include <complex>
#include <cstdio>
#include <vector>
using namespace amr_mpc_cpp;

int main()
{
  TorqueModel md;
  md.wheel_radius = 0.04; md.wheel_base = 0.1724; md.robot_width = 0.2140; md.mass = 4.56;
  md.inertia_z = 0.03; md.J_w = 8.9e-4; md.use_w = true; md.drive_yaw = 3.1 * M_PI / 180.0;
  struct C {const char * name; double w[6];};
  std::vector<C> cands = {
    {"기준 20,20,10,2,2,1", {20, 20, 10, 2, 2, 1}},
    {"위치x2 40,40,10,2,2,1", {40, 40, 10, 2, 2, 1}},
    {"위치x4 80,80,10,2,2,1", {80, 80, 10, 2, 2, 1}},
    {"속도x0.5 20,20,10,1,1,1", {20, 20, 10, 1, 1, 1}},
    {"위치x2속도x0.5 40,40,10,1,1,1", {40, 40, 10, 1, 1, 1}},
    {"속도x2 20,20,10,4,4,1", {20, 20, 10, 4, 4, 1}},
    {"헤딩x2 20,20,20,2,2,1", {20, 20, 20, 2, 2, 1}},
    {"요속x0.25 20,20,10,2,2,0.25", {20, 20, 10, 2, 2, 0.25}},
    {"요속x4 20,20,10,2,2,4", {20, 20, 10, 2, 2, 4}},
  };
  const double Ts = 0.01;
  const Eigen::Matrix<double, 3, 4> F = md.force_map();
  const Eigen::Vector3d Minv(1.0 / md.m_eff(), 1.0 / md.m_eff(), 1.0 / md.I_eff());
  const Eigen::Matrix<double, 3, 4> G = Minv.asDiagonal() * F;            // 가속도 = G τ  (map, θ = 0)
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
  A.block<3, 3>(0, 3) = Ts * Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 4> B;
  B.topRows<3>() = 0.5 * Ts * Ts * G; B.bottomRows<3>() = Ts * G;
  Eigen::Matrix<double, 6, 3> Bd;                                           // 외란 가속도 입력
  Bd.topRows<3>() = 0.5 * Ts * Ts * Eigen::Matrix3d::Identity(); Bd.bottomRows<3>() = Ts * Eigen::Matrix3d::Identity();

  std::printf("%-30s | %-34s | %-34s | %-22s | %s\n", "후보 W", "지연 0: 느린 극 시정수 병진/요 · 최대|극|", "지연 2틱: 최대|극| · 감쇠 최소",
              "외란 0.1 m/s²: 정상 / 1.6 rad/s", "이득: 1 cm · 1 cm/s · 1° · 1 °/s 당 바퀴 토크 [mN·m]");
  for (const auto & c : cands) {
    TorqueTuning tu;
    tu.Ts = Ts; tu.Np = 75; tu.Nc = 40; tu.w_delta = 100.0; tu.r_u = 10.0; tu.q_null = 0.0; tu.constrained = false;
    for (int i = 0; i < 6; ++i) {tu.W(i) = c.w[i];}
    const auto K = unconstrained_gain(md, tu);
    auto closed = [&](int d) {
      const int n = 6 + 4 * (d + 1);                                       // [e, δτ_{k−1}, 지연 버퍼 d 개]
      Eigen::MatrixXd Acl = Eigen::MatrixXd::Zero(n, n), Bcl = Eigen::MatrixXd::Zero(n, 3);
      // δτ_k = −K_x e + K_τ δτ_{k−1}  (행 6..9 = 새 δτ_k 로 갱신), 실제 가해지는 입력 = d 틱 전 것
      Acl.block(6, 0, 4, 6) = -K.Kx; Acl.block(6, 6, 4, 4) = K.Ktau;
      for (int j = 0; j < d; ++j) {Acl.block(10 + 4 * j, 6 + 4 * j, 4, 4) = Eigen::Matrix4d::Identity();}
      const int applied = 6 + 4 * d;
      Acl.block(0, 0, 6, 6) = A;
      if (d == 0) {Acl.block(0, 0, 6, 6) += -B * K.Kx; Acl.block(0, 6, 6, 4) = B * K.Ktau;}
      else {Acl.block(0, applied, 6, 4) = B;}
      Bcl.topRows(6) = Bd;
      return std::make_pair(Acl, Bcl);
    };
    auto poles = [&](const Eigen::MatrixXd & M) {Eigen::EigenSolver<Eigen::MatrixXd> es(M); return es.eigenvalues();};
    const auto [A0, B0] = closed(0);
    const auto p0 = poles(A0);
    double slow_tr = 0, slow_yaw = 0, maxp0 = 0;
    {
      Eigen::EigenSolver<Eigen::MatrixXd> es(A0);
      for (int i = 0; i < p0.size(); ++i) {
        const double m = std::abs(p0(i)); maxp0 = std::max(maxp0, m);
        if (m < 1e-6 || m >= 1.0) {continue;}
        const double tc = -Ts / std::log(m);
        const auto v = es.eigenvectors().col(i);
        const double yaw_share = std::abs(v(2)) + std::abs(v(5)), tr_share = std::abs(v(0)) + std::abs(v(1)) + std::abs(v(3)) + std::abs(v(4));
        if (yaw_share > tr_share) {slow_yaw = std::max(slow_yaw, tc);} else {slow_tr = std::max(slow_tr, tc);}
      }
    }
    const auto [A2, B2] = closed(2);
    const auto p2 = poles(A2);
    double maxp2 = 0, minz = 1e9;
    for (int i = 0; i < p2.size(); ++i) {
      maxp2 = std::max(maxp2, std::abs(p2(i)));
      if (std::abs(p2(i)) > 1e-6) {
        const std::complex<double> s = std::log(p2(i)) / Ts;
        if (std::abs(s.imag()) > 1e-6) {minz = std::min(minz, -s.real() / std::abs(s));}
      }
    }
    // 외란 → 위치 오차: 정상 (z = 1), 도는 외란 (z = e^{jωTs}, ω 1.6 rad/s)
    auto pos_gain = [&](const Eigen::MatrixXd & Acl, const Eigen::MatrixXd & Bcl, double w) {
      const std::complex<double> z = std::polar(1.0, w * Ts);
      const Eigen::MatrixXcd Mz = z * Eigen::MatrixXcd::Identity(Acl.rows(), Acl.cols()) - Acl.cast<std::complex<double>>();
      const Eigen::MatrixXcd X = Mz.fullPivLu().solve(Bcl.cast<std::complex<double>>());
      return std::abs(X(0, 0));                                             // x 외란 → x 위치 [m per m/s²]
    };
    const double ss = 0.1 * pos_gain(A0, B0, 0.0) * 100, rot = 0.1 * pos_gain(A0, B0, 1.6) * 100;
    const double kp = K.Kx.col(0).norm() * 0.01 * 1e3, kv = K.Kx.col(3).norm() * 0.01 * 1e3, kth = K.Kx.col(2).norm() * (M_PI / 180) * 1e3,
                 kw = K.Kx.col(5).norm() * (M_PI / 180) * 1e3;
    std::printf("%-30s | %6.0f ms / %6.0f ms · %.4f        | %.4f · ζ %.2f              | %5.2f cm / %5.2f cm      | %5.1f · %5.1f · %5.1f · %5.1f\n",
                c.name, 1e3 * slow_tr, 1e3 * slow_yaw, maxp0, maxp2, minz, ss, rot, kp, kv, kth, kw);
  }
  return 0;
}
