// 토크 입력 MPC 노드 (구조도 v3 의 torque_mpc_node).  핵심 계산은 torque_mpc.hpp, 식은 『메카넘 토크 MPC 1판 설계』.
// ---------------------------------------------------------------
//   reference_topic   PlanarReferenceArray   기준열 (figure8_reference_node, 10 Hz, 0.05 s × 80 — torque_track.yaml)
//   state_topic       nav_msgs/Odometry      상태 추정값 x̂.  pose = map 좌표 [x, y, yaw],  twist = 몸체 [v_x, v_y, ω_z]
//      ↓ ReferenceHorizon 보간 (Ts 격자, Np+1 점) → TorqueMPC (DAQP)
//   cmd_topic         WheelTorqueCommand     MPC 순토크 τ* → torque_output → 게이트 → /wheel_torque_cmd
//   /torque_mpc/status, /torque_mpc/reference_horizon
//
// 실행 방식: 타이머가 아니라 **상태 메시지가 올 때마다** 한 번 푼다 (추정기 100 Hz = 제어 100 Hz).
//   추정값의 나이가 틱마다 달라지지 않고, 명령의 header.stamp 가 그 추정값의 시각 그대로라
//   ak40_can_node 가 종단 지연을 잴 수 있다 (WheelTorqueCommand.msg 주석).
//
// 파라미터 출처 (amr_mpc 패키지 config/)
//   torque_plant.yaml : 로봇·노면 값 (질량, 관성, 모터 한계, μ̂, 바퀴 관성 스위치)
//   torque_mpc.yaml   : 제어기 설정 (비교군 limit_mode, 외란 추정치 스위치, 튜닝)
//   두 파일 값은 ROS 파라미터의 기본값이다 → launch 인자나 --ros-args -p limit_mode:=friction 으로 실행마다 덮어쓴다.
//   payload_kg 는 "제어기가 믿는" 적재다. 시뮬 플랜트의 payload_kg 와 따로 줄 수 있다 (통과시험 2(b): 질량 오차).
//
// 무장: armed_topic(기본 /mpc/armed, 게이트가 latched 로 낸다)이 false 인 동안은 풀지 않고 DAMPING + reset.
//   게이트가 막혀 있는데 MPC 만 계속 풀면, 서 있는 로봇과 기준의 오차로 토크가 쌓였다가 무장 순간 한꺼번에
//   나간다 (2026-11-11). armed_topic 을 빈 문자열로 주면 이 검사를 끈다 (게이트 없이 단독 시험할 때만).
//   무장한 뒤에도 "무장 이후에 받은 기준열" 이 올 때까지는 풀지 않는다. 그 전 기준열은 정지 중의 묵은 기준이라
//   (figure8 은 재무장 때 로봇 위치로 기준을 다시 맞춘다) 그걸로 풀면 오차 수 cm 를 상한 토크로 메우려 든다.
// 정지: 추종 중이 아닐 때는 언제나 MODE_DAMPING 을 낸다 — 미무장, 기준·상태 끊김, hold 진입, 종료.
//   토크 0 은 관성 주행이라 정지가 아니다. 그때 MPC 를 reset() 한다 (명령 이력이 끊기면 d̂ 가 뜻을 잃는다, 1판 09절).
// 좌표계: 기준열 header.frame_id 와 상태 header.frame_id 가 둘 다 있고 다르면 풀지 않는다 (R5 리뷰 2026-11-13).
//   예: 기준은 odom, 상태는 map 이면 둘 사이 오프셋만큼 로봇을 끌고 간다.
// ---------------------------------------------------------------
#include "amr_mpc_cpp/reference.hpp"
#include "amr_mpc_cpp/torque_mpc.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <amr_interfaces/msg/planar_reference_array.hpp>
#include <amr_interfaces/msg/wheel_torque_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using amr_interfaces::msg::WheelTorqueCommand;
using amr_mpc_cpp::HoldLatch;
using amr_mpc_cpp::ReferenceHorizon;
using amr_mpc_cpp::TorqueModel;
using amr_mpc_cpp::TorqueMPC;
using amr_mpc_cpp::TorqueTuning;
using amr_mpc_cpp::TVec3;
using amr_mpc_cpp::TVec4;
using amr_mpc_cpp::TVec6;

namespace
{
std::atomic<bool> g_stop{false};
void on_sigint(int) {g_stop = true;}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// amr_mpc/core/config.py 와 같은 규칙: 환경변수 AMR_MPC_<FILE> → 설치본 share/amr_mpc/config/
std::string config_file(const char * env_name, const std::string & filename)
{
  if (const char * env = std::getenv(env_name)) {return env;}
  return ament_index_cpp::get_package_share_directory("amr_mpc") + "/config/" + filename;
}
}  // namespace

class TorqueMpcNode : public rclcpp::Node
{
public:
  TorqueMpcNode()
  : Node("torque_mpc_node")
  {
    const std::string plant_file = declare_parameter<std::string>(
      "plant_file", config_file("AMR_MPC_TORQUE_PLANT_YAML", "torque_plant.yaml"));
    const std::string mpc_file = declare_parameter<std::string>(
      "mpc_file", config_file("AMR_MPC_TORQUE_MPC_YAML", "torque_mpc.yaml"));
    const YAML::Node yp = YAML::LoadFile(plant_file);
    const YAML::Node ym = YAML::LoadFile(mpc_file);

    TorqueModel md;
    md.wheel_radius = yp["geometry"]["wheel_radius_m"].as<double>();
    md.wheel_base = yp["geometry"]["wheel_base_m"].as<double>();
    md.robot_width = yp["geometry"]["robot_width_m"].as<double>();
    const double payload = declare_parameter<double>("payload_kg", yp["body"]["payload_kg"].as<double>());
    md.mass = yp["body"]["mass_kg"].as<double>() + payload;
    md.inertia_z = declare_parameter<double>("inertia_z", yp["body"]["inertia_z_kgm2"].as<double>());
    md.J_w = yp["wheel"]["inertia_J_w_kgm2"].as<double>();
    md.use_wheel_inertia = declare_parameter<bool>("use_wheel_inertia", yp["model"]["use_wheel_inertia"].as<bool>());
    md.tau_rated = yp["motor"]["tau_rated_Nm"].as<double>();
    md.tau_fric_max = yp["friction"]["tau_fric_max_Nm"].as<double>();
    md.mu_hat = declare_parameter<double>("mu_hat", yp["surface"]["mu_hat"].as<double>());
    md.c_s = yp["surface"]["c_s"].as<double>();
    md.gravity = yp["model"]["gravity"].as<double>();

    TorqueTuning tu;
    const YAML::Node yt = ym["tuning"];
    tu.Ts = declare_parameter<double>("Ts", yt["Ts"].as<double>());
    tu.Np = declare_parameter<int>("Np", yt["Np"].as<int>());
    tu.Nc = declare_parameter<int>("Nc", yt["Nc"].as<int>());
    const std::vector<double> W = declare_parameter<std::vector<double>>("W", yt["W"].as<std::vector<double>>());
    if (W.size() != 6) {throw std::invalid_argument("tuning.W 는 6개여야 한다 [x, y, θ, ẋ, ẏ, θ̇]");}
    for (int i = 0; i < 6; ++i) {tu.W(i) = W[i];}
    tu.w_delta = declare_parameter<double>("w_delta", yt["w_delta"].as<double>());
    tu.q_null = declare_parameter<double>("q_null", yt["q_null"].as<double>());
    tu.daqp_iter_limit = declare_parameter<int>("daqp_iter_limit", yt["daqp_iter_limit"].as<int>(1000));
    limit_mode_ = declare_parameter<std::string>("limit_mode", ym["torque_limit"]["mode"].as<std::string>());
    tu.limit_mode = amr_mpc_cpp::parse_limit_mode(limit_mode_);
    tu.rate_fraction = declare_parameter<double>("rate_fraction", ym["torque_limit"]["rate_fraction"].as<double>());
    tu.dist_enable = declare_parameter<bool>("disturbance_enable", ym["disturbance"]["enable"].as<bool>());
    tu.dist_alpha = declare_parameter<double>("disturbance_alpha", ym["disturbance"]["alpha"].as<double>());
    mpc_ = std::make_unique<TorqueMPC>(md, tu);

    const auto ref_topic = declare_parameter<std::string>("reference_topic", "/dra_mppi/reference_trajectory");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/state_estimator/odom");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/torque_mpc/cmd_raw");
    const auto armed_topic = declare_parameter<std::string>("armed_topic", "/mpc/armed");
    state_timeout_ = declare_parameter<double>("state_timeout", ym["safety"]["state_timeout"].as<double>());
    ref_timeout_ = declare_parameter<double>("ref_timeout", ym["safety"]["ref_timeout"].as<double>());
    debug_ = declare_parameter<bool>("publish_debug", true);
    const auto log_prefix = declare_parameter<std::string>("log_prefix", "");
    const YAML::Node yh = ym["hold"];
    const double deg = M_PI / 180.0;
    latch_.emplace(
      declare_parameter<double>("hold_enter_xy", yh["enter_xy"].as<double>()),
      declare_parameter<double>("hold_enter_yaw_deg", yh["enter_yaw_deg"].as<double>()) * deg,
      declare_parameter<double>("hold_exit_xy", yh["exit_xy"].as<double>()),
      declare_parameter<double>("hold_exit_yaw_deg", yh["exit_yaw_deg"].as<double>()) * deg);
    hold_stopped_speed_ = declare_parameter<double>("hold_stopped_speed", yh["stopped_speed"].as<double>());
    if (!log_prefix.empty()) {
      char buf[32];
      const std::time_t now = std::time(nullptr);
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
      csv_path_ = log_prefix + "_" + buf + ".csv";
    }

    pub_ = create_publisher<WheelTorqueCommand>(cmd_topic, 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/torque_mpc/reference_horizon", 1);
    pub_status_ = create_publisher<std_msgs::msg::Float64MultiArray>("/torque_mpc/status", 10);
    sub_state_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic, 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_state(*m);});
    sub_ref_ = create_subscription<amr_interfaces::msg::PlanarReferenceArray>(
      ref_topic, 5, [this](amr_interfaces::msg::PlanarReferenceArray::ConstSharedPtr m) {on_ref(*m);});
    if (armed_topic.empty()) {
      armed_ = true;
      RCLCPP_WARN(get_logger(), "armed_topic 이 비어 있다 — 게이트 무장 여부를 보지 않고 바로 푼다 (단독 시험 전용)");
    } else {
      sub_armed_ = create_subscription<std_msgs::msg::Bool>(
        armed_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data != armed_) {
            armed_ = m->data;
            armed_since_ = now_s();
            RCLCPP_INFO(get_logger(), "%s", armed_ ? "게이트 무장 — 추종 가능" : "게이트 정지 — DAMPING, MPC 리셋");
          }
        });
    }
    watchdog_ = rclcpp::create_timer(this, get_clock(), 50ms, [this]() {on_watchdog();});

    const TVec4 tb = mpc_->tau_bar();
    RCLCPP_INFO(
      get_logger(),
      "토크 MPC (C++/DAQP).  기준 %s  상태 %s → %s\n"
      "  Ts %.0f ms  Np %d  Nc %d   m %.2f kg  I_z %.3f  바퀴관성 %s (μ %.3f kg)\n"
      "  상한 모드 %s: τ̄ %.4f N·m, Δτ̄ %.4f N·m/스텝  (모터 %.3f, 마찰 τ_μ %.4f @ μ̂ %.2f)   외란 추정치 %s (α %.2f)",
      ref_topic.c_str(), state_topic.c_str(), cmd_topic.c_str(), tu.Ts * 1e3, tu.Np, tu.Nc, md.mass, md.inertia_z,
      md.use_wheel_inertia ? "켬" : "끔", md.mu(), limit_mode_.c_str(), tb(0), mpc_->dtau_bar_of(tb(0)),
      md.tau_motor_limit(), md.tau_friction_limit(), md.mu_hat, tu.dist_enable ? "켬" : "끔", tu.dist_alpha);
  }

  void finish()
  {
    if (done_) {return;}
    done_ = true;
    publish_damping();
    if (csv_path_.empty() || rows_.empty()) {return;}
    std::ofstream f(csv_path_);
    f.precision(15);
    f << "t,x,y,th,xd,yd,thd,xr,yr,thr,ex,ey,eth,tau_fl,tau_fr,tau_rl,tau_rr,tau_bar,active,qp_iters,fallback,"
      "step_ms,ref_age,d_x,d_y,d_th,d_xd,d_yd,d_thd\n";
    double se = 0.0, emax = 0.0, step_sum = 0.0, step_max = 0.0;
    for (const auto & r : rows_) {
      for (size_t i = 0; i < r.size(); ++i) {f << (i ? "," : "") << r[i];}
      f << "\n";
      const double e = std::hypot(r[10], r[11]);
      se += e * e;
      emax = std::max(emax, e);
      step_sum += r[21];
      step_max = std::max(step_max, r[21]);
    }
    const double n = static_cast<double>(rows_.size());
    std::printf(
      "\n[INFO] 저장: %zu행 → %s\n       위치오차 RMS %.2f cm  최대 %.2f cm   step 평균 %.3f / 최대 %.3f ms\n",
      rows_.size(), csv_path_.c_str(), std::sqrt(se / n) * 100, emax * 100, step_sum / n, step_max);
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  void publish_torque(const TVec4 & tau, const builtin_interfaces::msg::Time & stamp)
  {
    WheelTorqueCommand m;
    m.header.stamp = stamp;                 // 이 명령을 계산한 상태 추정값의 시각
    m.header.frame_id = "base_footprint";
    m.mode = WheelTorqueCommand::MODE_TORQUE;
    for (int i = 0; i < 4; ++i) {m.torque[i] = tau(i);}
    pub_->publish(m);
  }

  void publish_damping()
  {
    WheelTorqueCommand m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = "base_footprint";
    m.mode = WheelTorqueCommand::MODE_DAMPING;
    pub_->publish(m);
  }

  void on_ref(const amr_interfaces::msg::PlanarReferenceArray & m)
  {
    if (m.references.empty()) {return;}
    std::vector<ReferenceHorizon::Row> rows;
    rows.reserve(m.references.size());
    for (const auto & r : m.references) {rows.push_back({r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate});}
    try {
      ref_.emplace(rclcpp::Time(m.header.stamp).seconds(), m.dt, rows);
    } catch (const std::invalid_argument & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "기준열 무시: %s", e.what());
      return;
    }
    ref_rx_ = now_s();
    ref_frame_ = m.header.frame_id;
  }

  void stop(const char * reason)
  {
    publish_damping();
    if (active_) {RCLCPP_INFO(get_logger(), "정지(DAMPING): %s", reason);}
    active_ = false;
    mpc_->reset();
  }

  // 상태가 끊기면 on_state 가 불리지 않으므로 여기서 정지 명령을 계속 낸다
  void on_watchdog()
  {
    const double now = now_s();
    if (last_state_rx_ <= 0.0 || now - last_state_rx_ > state_timeout_) {
      stop("상태 추정값 없음");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "상태 추정값 대기/끊김");
    }
  }

  void on_state(const nav_msgs::msg::Odometry & m)
  {
    const double now = now_s();
    const double st = rclcpp::Time(m.header.stamp).seconds();
    const double t_state = st > 0.0 ? st : now;         // stamp 가 비어 있으면 수신 시각
    if (last_state_stamp_ > 0.0) {
      const double dt = t_state - last_state_stamp_;
      if (std::abs(dt - mpc_->tuning().Ts) > 0.5 * mpc_->tuning().Ts) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "상태 주기 %.1f ms ≠ Ts %.1f ms — 모델 이산화와 어긋난다",
          dt * 1e3, mpc_->tuning().Ts * 1e3);
      }
    }
    last_state_stamp_ = t_state;
    last_state_rx_ = now;

    if (!ref_frame_.empty() && !m.header.frame_id.empty() && ref_frame_ != m.header.frame_id) {
      stop("기준·상태 좌표계 다름");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "기준열 좌표계 '%s' ≠ 상태 좌표계 '%s' — 풀지 않는다 (figure8 frame_id 확인)",
        ref_frame_.c_str(), m.header.frame_id.c_str());
      return;
    }
    const auto & p = m.pose.pose;
    const double yaw = yaw_of(p.orientation);
    const TVec3 vb(m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z);
    const double c = std::cos(yaw), s = std::sin(yaw);
    TVec6 x;
    x << p.position.x, p.position.y, yaw, vb(0) * c - vb(1) * s, vb(0) * s + vb(1) * c, vb(2);   // 식-1: Ẋ = R(θ) v

    if (!armed_) {
      stop("게이트 미무장");
      return;
    }
    if (!ref_) {
      stop("기준 없음");
      return;
    }
    if (ref_rx_ < armed_since_) {
      stop("무장 뒤 새 기준열 대기");
      return;
    }
    if (now - ref_rx_ > ref_timeout_) {
      stop("기준열 끊김");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "기준열 %.2f s 없음", now - ref_rx_);
      return;
    }
    if (!active_) {
      active_ = true;
      mpc_->reset();
      RCLCPP_INFO(get_logger(), "추종 시작. 기준열 %d×%.2f s", ref_->size(), ref_->dt());
    }
    const int Np = mpc_->tuning().Np;
    const auto hz = ref_->horizon(t_state, mpc_->tuning().Ts, Np + 1);
    const TVec3 Xr = hz[0].first;
    TVec3 Xe = Xr - x.head<3>();
    Xe(2) = amr_mpc_cpp::wrap_angle(Xe(2));
    const bool stopped = std::hypot(vb(0), vb(1)) < hold_stopped_speed_ &&
      std::abs(vb(2)) * mpc_->model().k() < hold_stopped_speed_;
    const bool was_idle = latch_->idle();
    if (latch_->update(ref_->is_hold(), Xe, stopped)) {
      if (!was_idle) {
        RCLCPP_INFO(
          get_logger(), "hold 진입: 오차 %.1f mm, %.2f° — DAMPING, MPC 리셋",
          std::hypot(Xe(0), Xe(1)) * 1e3, Xe(2) * 180.0 / M_PI);
        mpc_->reset();
      }
      publish_damping();
      if (debug_) {publish_status(now, Xe, TVec4::Zero(), 0.0, t_state - ref_->t0());}
      return;
    }
    if (was_idle) {
      RCLCPP_INFO(
        get_logger(), "hold 해제: 오차 %.1f mm, %.2f° — 추종 재개",
        std::hypot(Xe(0), Xe(1)) * 1e3, Xe(2) * 180.0 / M_PI);
    }

    const auto t_c = std::chrono::steady_clock::now();
    const TVec4 tau = mpc_->step(x, hz);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_c).count();
    publish_torque(tau, m.header.stamp);

    const auto & stt = mpc_->stats();
    if (stt.fallback) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "QP 실패(exitflag %d) — 무제약 해를 상한으로 깎았다", stt.exitflag);
    }
    const double ref_age = t_state - ref_->t0();
    const TVec6 & d = mpc_->d_hat();
    if (!csv_path_.empty()) {
      rows_.push_back(
        {t_state, x(0), x(1), x(2), x(3), x(4), x(5), Xr(0), Xr(1), Xr(2), Xe(0), Xe(1), Xe(2),
          tau(0), tau(1), tau(2), tau(3), mpc_->tau_bar().maxCoeff(), static_cast<double>(stt.active),
          static_cast<double>(stt.iters), static_cast<double>(stt.fallback), ms, ref_age,
          d(0), d(1), d(2), d(3), d(4), d(5)});
    }
    if (debug_) {
      publish_status(now, Xe, tau, ms, ref_age);
      nav_msgs::msg::Path path;
      path.header.frame_id = ref_frame_.empty() ? "map" : ref_frame_;
      path.header.stamp = get_clock()->now();
      path.poses.reserve(hz.size());
      for (const auto & h : hz) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = h.first(0);
        ps.pose.position.y = h.first(1);
        ps.pose.orientation.z = std::sin(h.first(2) / 2);
        ps.pose.orientation.w = std::cos(h.first(2) / 2);
        path.poses.push_back(ps);
      }
      pub_path_->publish(path);
    }
  }

  // [t, ex, ey, eθ, τ_FL, τ_FR, τ_RL, τ_RR, τ̄, 활성 제약 수, QP 반복, fallback, step_ms, ref_age, d̂ ×6]
  void publish_status(double now, const TVec3 & Xe, const TVec4 & tau, double ms, double ref_age)
  {
    const auto & stt = mpc_->stats();
    const TVec6 & d = mpc_->d_hat();
    std_msgs::msg::Float64MultiArray m;
    m.data = {now, Xe(0), Xe(1), Xe(2), tau(0), tau(1), tau(2), tau(3), mpc_->tau_bar().maxCoeff(),
      static_cast<double>(stt.active), static_cast<double>(stt.iters), static_cast<double>(stt.fallback), ms, ref_age,
      d(0), d(1), d(2), d(3), d(4), d(5)};
    pub_status_->publish(m);
  }

  std::unique_ptr<TorqueMPC> mpc_;
  std::optional<HoldLatch> latch_;
  std::optional<ReferenceHorizon> ref_;
  std::string limit_mode_, ref_frame_, csv_path_;
  double last_state_rx_ = 0.0, last_state_stamp_ = 0.0, ref_rx_ = 0.0, armed_since_ = 0.0;
  double state_timeout_ = 0.3, ref_timeout_ = 0.5, hold_stopped_speed_ = 0.04;
  bool debug_ = true, active_ = false, done_ = false, armed_ = false;
  std::vector<std::vector<double>> rows_;

  rclcpp::Publisher<WheelTorqueCommand>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_state_;
  rclcpp::Subscription<amr_interfaces::msg::PlanarReferenceArray>::SharedPtr sub_ref_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_armed_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  {
    std::shared_ptr<TorqueMpcNode> node;
    try {
      node = std::make_shared<TorqueMpcNode>();
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[torque_mpc_node] 기동 실패: %s\n", e.what());
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && !g_stop) {exec.spin_once(50ms);}
    node->finish();
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
