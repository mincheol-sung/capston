// 플래너 기준열 → MPC 시간축 보간과 hold latch.  amr_mpc/core/reference.py 의 이식이고 규칙·근거는 그 파일 주석.
//   references[i] 의 목표 시각 = header.stamp + (i+1)·dt
//   t < t_0      : 첫 샘플의 몸체속도로 뒤로 외삽 (상수 twist 의 정확 적분, SE(2) 지수사상)
//   t_0 ≤ t ≤ t_end : 선형 보간 (yaw 는 unwrap 뒤)
//   t > t_end    : 끝 포즈 유지, 속도 0
#pragma once

#include <Eigen/Dense>

#include <array>
#include <utility>
#include <vector>

namespace amr_mpc_cpp
{

double wrap_angle(double a);

class ReferenceHorizon
{
public:
  using Row = std::array<double, 6>;   // x, y, yaw, vx, vy, yaw_rate

  // dt ≤ 0 이거나 rows 가 비면 std::invalid_argument
  ReferenceHorizon(double t0, double dt, const std::vector<Row> & rows);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> at(double t) const;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> horizon(double t, double Ts, int Np) const;
  bool is_hold() const;   // 속도 전부 0 이고 포즈가 한 점 (플래너의 정지 기준열)

  double t0() const {return t0_;}
  double dt() const {return dt_;}
  int size() const {return static_cast<int>(X_.size());}

private:
  double t0_, dt_;
  std::vector<double> times_;
  std::vector<Eigen::Vector3d> X_, U_;
};

class HoldLatch
{
public:
  HoldLatch(double enter_xy, double enter_yaw, double exit_xy, double exit_yaw);
  // true 면 이번 틱은 명령 0 + MPC 리셋. stopped: 측정 바퀴속도가 정지 문턱 아래인가
  bool update(bool is_hold, const Eigen::Vector3d & Xe, bool stopped = true);
  bool idle() const {return idle_;}

private:
  double enter_xy_, enter_yaw_, exit_xy_, exit_yaw_;
  bool idle_ = false;
};

}  // namespace amr_mpc_cpp
