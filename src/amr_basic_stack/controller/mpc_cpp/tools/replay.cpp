// Python 판과의 일치 검증. 형식은 tools/make_parity_fixture.py 머리주석.
//   wheel_mpc_replay mpc <file>    WheelMPC.step 출력 비교
//   wheel_mpc_replay ref <file>    ReferenceHorizon.at / is_hold, HoldLatch.update 비교
// 종료코드 0 = 허용오차 안.
#include "amr_mpc_cpp/reference.hpp"
#include "amr_mpc_cpp/wheel_mpc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace amr_mpc_cpp;

namespace
{
struct Reader
{
  std::ifstream f;
  explicit Reader(const char * p)
  : f(p, std::ios::binary) {if (!f) {throw std::runtime_error(std::string("열 수 없다: ") + p);}}
  double d()
  {
    double v;
    if (!f.read(reinterpret_cast<char *>(&v), 8)) {throw std::runtime_error("파일이 짧다");}
    return v;
  }
  int i() {return static_cast<int>(std::lround(d()));}
  Vec3 v3() {double a = d(), b = d(), c = d(); return Vec3(a, b, c);}
  Vec4 v4() {double a = d(), b = d(), c = d(), e = d(); return Vec4(a, b, c, e);}
};

int replay_mpc(const char * path)
{
  Reader r(path);
  WheelModel md;
  WheelTuning tu;
  tu.Ts = r.d(); tu.Np = r.i(); tu.Nc = r.i(); md.theta = r.d();
  tu.q_pos = r.d(); tu.q_yaw = r.d(); tu.r_dw = r.d(); tu.q_null = r.d(); tu.k_obs = r.d();
  md.tau = r.v4(); md.k_dc = r.v4();
  md.wheel_radius = r.d(); md.wheel_base = r.d(); md.robot_width = r.d(); md.w_max = r.d(); md.alpha_max = r.d();
  const int N = r.i();
  WheelMPC m(md, tu);
  double worst = 0.0;
  int worst_k = -1, n_bad = 0, fb_py = 0, fb_cpp = 0;
  std::vector<double> t_ms;
  std::vector<RefPoint> ref(tu.Np);
  for (int k = 0; k < N; ++k) {
    const bool reset = r.d() > 0.5;
    const Vec3 X = r.v3();
    const Vec4 w_meas = r.v4(), w_prev = r.v4();
    const Vec3 Xr = r.v3(), ur = r.v3();
    for (int i = 0; i < tu.Np; ++i) {ref[i] = r.v3();}
    const Vec4 wc_py = r.v4();
    fb_py += r.i();
    if (reset) {m.reset();}
    const auto t0 = std::chrono::steady_clock::now();
    const Vec4 wc = m.step(X, w_meas, w_prev, ref, Xr, ur);
    t_ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    fb_cpp += m.stats().fallback;
    const double e = (wc - wc_py).cwiseAbs().maxCoeff();
    if (e > 1e-8) {++n_bad;}
    if (e > worst) {worst = e; worst_k = k;}
  }
  std::sort(t_ms.begin(), t_ms.end());
  std::printf(
    "%s: Ts %.0f ms Np %d Nc %d d %d | %d 스텝  max|ω_cpp − ω_py| %.2e rad/s (스텝 %d)  1e-8 초과 %d  "
    "fallback py %d / cpp %d | step p50 %.3f  max %.3f ms\n",
    path, tu.Ts * 1e3, tu.Np, tu.Nc, m.d(), N, worst, worst_k, n_bad, fb_py, fb_cpp, t_ms[t_ms.size() / 2], t_ms.back());
  return worst < 1e-6 ? 0 : 1;
}

int replay_ref(const char * path)
{
  Reader r(path);
  const int n_cases = r.i();
  double worst = 0.0;
  int n_q = 0, hold_bad = 0;
  for (int c = 0; c < n_cases; ++c) {
    const int T = r.i();
    const double dt = r.d(), t0 = r.d();
    std::vector<ReferenceHorizon::Row> rows(T);
    for (auto & row : rows) {for (auto & v : row) {v = r.d();}}
    const bool hold_py = r.d() > 0.5;
    const int nt = r.i();
    ReferenceHorizon h(t0, dt, rows);
    hold_bad += (h.is_hold() != hold_py);
    for (int q = 0; q < nt; ++q) {
      const double t = r.d();
      const Vec3 Xr_py = r.v3(), ur_py = r.v3();
      const auto [Xr, ur] = h.at(t);
      Vec3 dX = Xr - Xr_py;
      dX(2) = wrap_angle(dX(2));
      worst = std::max({worst, dX.cwiseAbs().maxCoeff(), (ur - ur_py).cwiseAbs().maxCoeff()});
      ++n_q;
    }
  }
  const double exy = r.d(), eyaw = r.d(), xxy = r.d(), xyaw = r.d();
  const int n = r.i();
  HoldLatch L(exy, eyaw, xxy, xyaw);
  int latch_bad = 0;
  for (int k = 0; k < n; ++k) {
    const bool ih = r.d() > 0.5;
    const Vec3 Xe = r.v3();
    const bool st = r.d() > 0.5, out_py = r.d() > 0.5;
    latch_bad += (L.update(ih, Xe, st) != out_py);
  }
  std::printf(
    "%s: 기준열 %d 개, 질의 %d 개  max|Δ| %.2e   is_hold 불일치 %d   HoldLatch %d 스텝 중 불일치 %d\n",
    path, n_cases, n_q, worst, hold_bad, n, latch_bad);
  return (worst < 1e-9 && hold_bad == 0 && latch_bad == 0) ? 0 : 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "사용: %s mpc|ref <file> [<file> ...]\n", argv[0]);
    return 2;
  }
  int rc = 0;
  try {
    for (int i = 2; i < argc; ++i) {
      rc |= (std::strcmp(argv[1], "mpc") == 0) ? replay_mpc(argv[i]) : replay_ref(argv[i]);
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "오류: %s\n", e.what());
    return 2;
  }
  return rc;
}
