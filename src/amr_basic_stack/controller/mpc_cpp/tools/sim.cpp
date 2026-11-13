// 폐루프 시뮬 + step 시간 측정.  amr_mpc/wheel/mpc.py demo() 와 같은 원궤적에 측정잡음과 50 Hz 측정 갱신을 더했다.
//   wheel_mpc_sim [key=value ...]
//   키: Ts Np Nc theta(모델) plant_theta speed radius T meas_hz noise ex ey eyaw(deg) q_pos q_yaw r_dw q_null k_obs seed
//       pose_hz  포즈 측정 갱신 주기 (실차 /odometry/filtered 50 Hz).  기본은 매 틱
//       extrap   1 이면 묵은 포즈를 측정 시각의 몸체 twist 로 현재 틱까지 외삽 (wheel_mpc_node 와 같은 처리)
//       fresh    1 이면 새 바퀴 측정이 있는 틱에만 관측기 보정 (wheel_mpc_node 와 같은 처리)
//       model_tau model_k   모델 τ·이득 (네 바퀴 같은 값. 0 이면 WheelModel 기본)
//       plant_tau plant_k   실제 플랜트 τ·이득 (0 이면 모델과 같음) — 모델 오차 시험
// 플랜트: 바퀴별 FOPDT (τ, K 는 모델과 같음, θ 는 plant_theta) → 순기구학 → 포즈 Euler 적분.
// 종료코드 0 = 4 s 이후 위치오차 최대 < 2 cm 이고 바퀴 한계 안 (python demo() 와 같은 판정).
#include "amr_mpc_cpp/wheel_mpc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace amr_mpc_cpp;

int main(int argc, char ** argv)
{
  std::map<std::string, double> P = {
    {"Ts", 0.01}, {"Np", 80}, {"Nc", 20}, {"theta", 0.04}, {"plant_theta", 0.04},
    {"speed", 0.5}, {"radius", 1.0}, {"T", 16.0}, {"meas_hz", 50.0}, {"noise", 0.05},
    {"ex", 0.0}, {"ey", -0.35}, {"eyaw", -23.0}, {"seed", 0},
    {"q_pos", 1.5}, {"q_yaw", 0.125}, {"r_dw", 0.0064}, {"q_null", 0.000025}, {"k_obs", 0.04},
    {"pose_hz", 0.0}, {"extrap", 0}, {"fresh", 0},
    {"model_tau", 0.0}, {"model_k", 0.0}, {"plant_tau", 0.0}, {"plant_k", 0.0}};
  for (int i = 1; i < argc; ++i) {
    const char * eq = std::strchr(argv[i], '=');
    if (!eq || !P.count(std::string(argv[i], eq - argv[i]))) {
      std::fprintf(stderr, "알 수 없는 인자 %s\n", argv[i]);
      return 2;
    }
    P[std::string(argv[i], eq - argv[i])] = std::atof(eq + 1);
  }
  WheelModel md;
  md.theta = P["theta"];
  if (P["model_tau"] > 0) {md.tau = Vec4::Constant(P["model_tau"]);}
  if (P["model_k"] > 0) {md.k_dc = Vec4::Constant(P["model_k"]);}
  WheelTuning tu;
  tu.Ts = P["Ts"]; tu.Np = static_cast<int>(P["Np"]); tu.Nc = static_cast<int>(P["Nc"]);
  tu.q_pos = P["q_pos"]; tu.q_yaw = P["q_yaw"]; tu.r_dw = P["r_dw"]; tu.q_null = P["q_null"]; tu.k_obs = P["k_obs"];
  WheelMPC m(md, tu);
  const double Ts = tu.Ts;
  const int dp = static_cast<int>(std::lround(P["plant_theta"] / Ts));
  Vec4 a = m.a(), b = m.b();
  if (P["plant_tau"] > 0 || P["plant_k"] > 0) {
    for (int i = 0; i < 4; ++i) {
      const double tau = P["plant_tau"] > 0 ? P["plant_tau"] : md.tau(i);
      const double k = P["plant_k"] > 0 ? P["plant_k"] : md.k_dc(i);
      a(i) = std::exp(-Ts / tau);
      b(i) = k * (1.0 - a(i));
    }
  }
  const double V = P["speed"], R = P["radius"], om = V / R;
  auto rf = [&](double t, Vec3 & Xr, Vec3 & ur) {
      Xr << R * std::sin(om * t), R * (1 - std::cos(om * t)), om * t;
      ur << V, 0.0, om;
    };
  std::mt19937 rng(static_cast<unsigned>(P["seed"]));
  std::normal_distribution<double> nd(0.0, P["noise"]);
  Vec3 X(P["ex"], P["ey"], P["eyaw"] * M_PI / 180.0);
  Vec4 w = Vec4::Zero(), w_prev = Vec4::Zero(), w_meas = Vec4::Zero();
  std::deque<Vec4> hist(dp, Vec4::Zero());          // push 뒤 길이 dp+1 → front = u(k − dp)
  const int hold = std::max(1, static_cast<int>(std::lround(1.0 / (P["meas_hz"] * Ts))));
  const int hold_p = P["pose_hz"] > 0 ? std::max(1, static_cast<int>(std::lround(1.0 / (P["pose_hz"] * Ts)))) : 1;
  Vec3 X_meas = X, v_meas = Vec3::Zero();
  double pose_stamp = 0.0;
  int flips = 0, flip_n = 0;
  Vec4 dw_prev = Vec4::Zero();
  const int steps = static_cast<int>(P["T"] / Ts), n0 = static_cast<int>(4.0 / Ts);
  std::vector<double> err, t_ms;
  std::vector<RefPoint> ref(tu.Np);
  double wmax = 0.0, tv = 0.0;
  int fb = 0, itmax = 0, bad_exit = 0;
  Vec3 Xr, ur, Xi, ui;
  for (int k = 0; k < steps; ++k) {
    const double t = k * Ts;
    const bool w_new = (k % hold == 0);
    if (w_new) {for (int i = 0; i < 4; ++i) {w_meas(i) = w(i) + nd(rng);}}
    if (k % hold_p == 0) {X_meas = X; v_meas = m.mecanum().T_fwd * w; pose_stamp = t;}
    const Vec3 X_used = P["extrap"] > 0.5 ? extrapolate_pose(X_meas, v_meas, t - pose_stamp) : X_meas;
    rf(t, Xr, ur);
    for (int i = 0; i < tu.Np; ++i) {rf(t + i * Ts, Xi, ui); ref[i] = Vec3(Xi(2), ui(0), ui(1));}
    const auto t0 = std::chrono::steady_clock::now();
    const Vec4 wc = m.step(X_used, w_meas, w_prev, ref, Xr, ur, P["fresh"] > 0.5 ? w_new : true);
    t_ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    fb += m.stats().fallback;
    itmax = std::max(itmax, m.stats().iters);
    bad_exit += m.stats().exitflag < 1;
    tv += (wc - w_prev).cwiseAbs().sum();
    const Vec4 dw = wc - w_prev;
    if (k > 0) {for (int i = 0; i < 4; ++i) {flips += dw(i) * dw_prev(i) < 0; ++flip_n;}}
    dw_prev = dw;
    wmax = std::max(wmax, wc.cwiseAbs().maxCoeff());
    w_prev = wc;
    hist.push_back(wc);
    w = a.cwiseProduct(w) + b.cwiseProduct(hist.front());   // front = u(k − dp)
    hist.pop_front();
    X += Ts * body_kinematics(X, m.mecanum().T_fwd * w);
    X(2) = std::atan2(std::sin(X(2)), std::cos(X(2)));
    rf(t + Ts, Xr, ur);
    err.push_back(std::hypot(X(0) - Xr(0), X(1) - Xr(1)));
  }
  double s_tr = 0, s_ss = 0, e_ss = 0;
  int entry = -1;
  for (int k = 0; k < steps; ++k) {
    if (entry < 0 && err[k] < 0.01) {entry = k;}
    if (k < n0) {s_tr += err[k] * err[k];} else {s_ss += err[k] * err[k]; e_ss = std::max(e_ss, err[k]);}
  }
  std::vector<double> ts = t_ms;
  std::sort(ts.begin(), ts.end());
  const bool ok = e_ss < 0.02 && wmax <= m.w_max() + 1e-9 && std::isfinite(e_ss);
  std::printf(
    "Ts %.0f ms Np %d Nc %d  모델 θ %.0f ms (d=%d, 상태 %d+3) τ %.0f ms  플랜트 θ %.0f τ %.0f ms | 과도RMS %.2f cm  정상RMS %.3f cm  "
    "정상max %.3f cm  1cm진입 %.2f s  명령TV %.1f  Δω부호반전 %.0f %%  |ω|max %.1f  fallback %d  DAQP 실패 %d  반복max %d | "
    "step p50 %.3f  p99 %.3f  max %.3f ms  → %s\n",
    Ts * 1e3, tu.Np, tu.Nc, md.theta * 1e3, m.d(), m.n_m(), md.tau(0) * 1e3, P["plant_theta"] * 1e3,
    (P["plant_tau"] > 0 ? P["plant_tau"] : md.tau(0)) * 1e3,
    std::sqrt(s_tr / n0) * 100, std::sqrt(s_ss / (steps - n0)) * 100, e_ss * 100, entry * Ts,
    tv / P["T"] / 4.0, 100.0 * flips / std::max(1, flip_n), wmax, fb, bad_exit, itmax, ts[ts.size() / 2], ts[ts.size() * 99 / 100], ts.back(),
    ok ? "OK" : "✗");
  return ok ? 0 : 1;
}
