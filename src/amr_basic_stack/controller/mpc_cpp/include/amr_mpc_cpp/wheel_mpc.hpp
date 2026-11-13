// 구조 B — 바퀴 각속도 입력 MPC 의 C++ 이식.  amr_mpc/wheel/{model,mpc}.py + core/observer.py 와 같은 식이다.
// ---------------------------------------------------------------
// ROS 에 의존하지 않는다 (노드는 wheel_mpc_node.cpp, 시험은 test/).
//
// Python 판과 다른 점은 두 가지뿐이다.
//   1. dead time d = θ/Ts 를 임의의 정수로 받는다 (Python 은 d ≤ 1).  z_1 … z_d 의 shift register:
//        z_1(k+1) = ω_e,cmd(k),   z_{j+1}(k+1) = z_j(k),   ω_e(k+1) = a′ ω_e(k) + b′ z_d(k)
//      지연 블록은 nilpotent (S^d = 0) 라 근사가 아니라 정확한 표현이다. 100 Hz, θ = 40 ms 이면 d = 4.
//   2. QP 를 DAQP (Arnström 2022, dual active-set) 로 푼다.  Python 의 M η ≤ γ 를 두 갈래로 나눈다:
//        Δω rate 제약과 j=0 의 바퀴속도 box  → 변수별 bound  lb ≤ η ≤ ub
//        j ≥ 1 의 바퀴속도 box               → 양쪽 제약     ω_prev − W ≤ S_j η ≤ ω_prev + W
//      해는 같다 (행 정규화와 분할은 feasible set 을 바꾸지 않는다). 2026-10-22 폐루프 QP 1600 개에서
//      Python qpactive 해와 최대 8e-14.
//
// 오차 관례 (r − y):  X_e = X_r − X,  ω_e = ω_r − ω,  ω_r = T⁻¹ u_r,  z_j = ω_r − ω_cmd(k−j),
//                    ω_cmd = ω_prev − Δω_e(k)
// x_m = [X_e (3); ω_e (4); z_1 … z_d (4d)],   Wang 증강 x = [Δx_m; X_e]
// ---------------------------------------------------------------
#pragma once

#include <Eigen/Dense>

#include <deque>
#include <vector>

namespace amr_mpc_cpp
{

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using MatX = Eigen::MatrixXd;
using VecX = Eigen::VectorXd;

// plant_wheel.yaml 의 model / geometry / limits 절
struct WheelModel
{
  Vec4 tau{0.101, 0.098, 0.102, 0.093};   // [s]  FL FR RL RR
  Vec4 k_dc{1.010, 1.010, 1.011, 1.010};  // [-]
  double theta = 0.040;                   // [s]  dead time. Ts 의 정수배여야 한다
  double wheel_radius = 0.04;
  double wheel_base = 0.1897;
  double robot_width = 0.207;
  double w_max = 30.0;                    // [rad/s]
  double alpha_max = 112.5;               // [rad/s²]  → |Δω| ≤ alpha_max·Ts
};

// plant_wheel.yaml 의 tuning 절 (100 Hz 는 tuning_100hz)
struct WheelTuning
{
  double Ts = 0.01;
  int Np = 80;
  int Nc = 20;
  double q_pos = 1.5;
  double q_yaw = 0.125;
  double r_dw = 0.0064;
  double q_null = 0.000025;
  double k_obs = 0.04;
  int daqp_iter_limit = 1000;
};

struct Mecanum
{
  Eigen::Matrix<double, 4, 3> T_inv;   // ω = T⁻¹ v   (드라이버 twist_to_wheels 와 같은 식)
  Eigen::Matrix<double, 3, 4> T_fwd;   // v = T ω     (pinv)
  Vec4 null_dir;                       // T_fwd n = 0, ‖n‖ = 1   ([1,1,−1,−1]/2 의 ± )
  double k = 0.0;                      // (wheel_base + width)/2
};

Mecanum mecanum_matrices(double wheel_radius, double wheel_base, double robot_width);

// reference 주위 linearization 의 ZOH (A_k, B_k3).  A_c nilpotent 라 A_k = I + Ts A_c 가 정확하다.
void kinematic_jacobians(double theta_r, double vxr, double vyr, double Ts, Mat3 & A_k, Mat3 & B_k3);

// 몸체 kinematics Ẋ = R(θ) v  (시뮬·시험용)
Vec3 body_kinematics(const Vec3 & X, const Vec3 & v);

// 바퀴별 한계. 드라이버 limit_and_apply_inversion 과 같은 비례 축소 (방향 유지)
Vec4 clip_wheels(const Vec4 & w, double w_max);

// 포즈 외삽: 몸체 twist v 를 dt 동안 상수로 두고 정확 적분 (SE(2) 지수사상).
// 포즈 원천(EKF 50 Hz)보다 제어가 빠를 때 측정 시각 → 현재 틱으로 옮긴다 (wheel_mpc_node 머리주석)
Vec3 extrapolate_pose(const Vec3 & X, const Vec3 & v_body, double dt);

// FOPDT 속도 관측기 (core/observer.py).  v̂(k) = a v̂(k−1) + b u(k−1−d),  v̂ ← v̂ + K (v_meas − v̂)
class VelocityObserver
{
public:
  VelocityObserver() = default;
  VelocityObserver(const Vec4 & a, const Vec4 & b, int d, double K);
  void reset();
  void push(const Vec4 & u);            // 이번 스텝에 실제로 낸 명령 (step 의 return 직전)
  // step 의 맨 앞. fresh=false (지난 틱 이후 새 측정 없음) 이면 예측만 하고 보정을 건너뛴다 —
  // 같은 측정을 두 번 innovation 으로 넣으면 묵은 값 쪽으로 끌린다 (측정 50 Hz, 제어 100 Hz)
  Vec4 update(const Vec4 & v_meas, bool fresh = true);

private:
  Vec4 a_ = Vec4::Zero(), b_ = Vec4::Zero();
  int d_ = 0;
  double K_ = 1.0;
  std::deque<Vec4> u_hist_;             // [u(k−1−d), …, u(k−1)],  최대 d+1
  Vec4 vhat_ = Vec4::Zero();
  bool has_vhat_ = false;
};

struct StepStats
{
  int iters = 0;
  int active = 0;
  int fallback = 0;        // 해를 바퀴 한계로 다시 깎았거나 QP 가 실패해 무제약 해를 썼다
  int exitflag = 0;        // DAQP exitflag (≥ 1 이면 최적)
};

// 한 스텝 기준: (θ_r, v_xr, v_yr)
using RefPoint = Vec3;

class WheelMPC
{
public:
  WheelMPC(const WheelModel & model, const WheelTuning & tuning);

  void reset();

  // 현재 포즈 X, 측정 바퀴속도 w_meas, 직전 바퀴 명령 w_prev, 지평 기준 ref[Np],
  // 현재 기준 포즈 Xr, 현재 기준 몸체속도 ur  →  다음 바퀴 명령
  // w_fresh: w_meas 가 지난 step 이후 새로 들어온 측정인가 (관측기 보정 여부)
  Vec4 step(
    const Vec3 & X, const Vec4 & w_meas, const Vec4 & w_prev,
    const std::vector<RefPoint> & ref, const Vec3 & Xr, const Vec3 & ur, bool w_fresh = true);

  // ---- 모델 (시험·검증용으로 공개) ----
  void plant(double theta_r, double vxr, double vyr, MatX & A_m, MatX & B_m, MatX & C_m) const;
  void augment(double theta_r, double vxr, double vyr, MatX & A, MatX & B, MatX & C) const;
  void predict(const std::vector<RefPoint> & ref, MatX & F_free, MatX & Phi);   // Wang 의 (F, Φ)

  int d() const {return d_;}
  int n_m() const {return n_m_;}
  double Ts() const {return tu_.Ts;}
  int Np() const {return tu_.Np;}
  int Nc() const {return tu_.Nc;}
  double w_max() const {return md_.w_max;}
  double dw_max() const {return dw_max_;}
  const Vec4 & a() const {return a_;}
  const Vec4 & b() const {return b_;}
  const Mecanum & mecanum() const {return mec_;}
  const StepStats & stats() const {return stats_;}

private:
  void build_constant_parts();
  void apply_A(const Mat3 & A_k, const Eigen::Matrix<double, 3, 4> & B_k, int ncols);
  bool solve_qp(const MatX & E, const VecX & F, const Vec4 & w_prev, VecX & eta);

  WheelModel md_;
  WheelTuning tu_;
  Mecanum mec_;
  Vec4 a_, b_;
  int d_ = 0, n_m_ = 0, n_ = 0, p_ = 3, nu_ = 4, nU_ = 0;
  double dw_max_ = 0.0;

  VelocityObserver obs_;
  std::deque<Vec4> cmd_hist_;           // [ω_cmd(k−1), ω_cmd(k−2), …],  d 개
  VecX xm_prev_;
  bool has_xm_prev_ = false;
  StepStats stats_;

  // 상수부
  VecX qbar_;                           // Q̄ 대각 (3Np)
  MatX E_const_;                        // 2R̄ + E_null
  VecX F_null_;                         // × nᵀω_prev
  // j ≥ 1 의 바퀴속도 box 행 S_j,i.  DAQP 는 row-major 를 받는다
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_gen_;

  VecX qsqrt_;                          // √Q̄ — E 를 rankUpdate 로 만든다

  // 매 스텝 작업 버퍼.  G = [A_{i}…A_0 | cols_0 … cols_{Nc−1}]  (n × (n + 4Nc)), predict() 머리주석
  MatX G_, tmpX_, F_free_, Phi_, Ph_, E_;
  VecX F_, eta_, lam_, lb_, ub_;
};

}  // namespace amr_mpc_cpp
