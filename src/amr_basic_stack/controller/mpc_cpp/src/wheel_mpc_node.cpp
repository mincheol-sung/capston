// 구조 B — 바퀴 각속도 입력 MPC 노드 (C++).  amr_mpc/nodes/wheel_tracking.py 와 토픽·안전·hold 규칙이 같다.
// ---------------------------------------------------------------
//   /dra_mppi/reference_trajectory   PlanarReferenceArray
//   odom_topic                       pose X = [x, y, yaw]   (실주행은 /odometry/filtered — wheel.yaml 주석)
//   /measured_wheel_speeds           Float32MultiArray [FL, FR, RL, RR] rad/s
//      ↓ ReferenceHorizon 보간 → WheelMPC (d 단 지연, DAQP)
//   /wheel_cmd                       Float32MultiArray [FL, FR, RL, RR] rad/s
//   /wheel_mpc/reference_horizon, /wheel_mpc/status
//
// 파라미터 출처
//   물리값 (model / geometry / limits / safety / hold) : plant_file (기본 amr_mpc 의 plant_wheel.yaml)
//   모델 절은 model_section (기본 model_100hz — 1차 근사, 실제 바퀴 속도 기준. 없으면 model)
//   튜닝 (Ts, Np, Nc, 가중치, k_obs)                  : 같은 파일의 tuning_section 절 (기본 tuning_100hz)
//   위 값은 ROS 파라미터의 기본값으로만 쓴다 → --ros-args -p Nc:=10 처럼 실행마다 덮어쓸 수 있다.
//   theta 도 파라미터다 (기본 model.theta). Ts 의 정수배가 아니면 기동 시 거부한다.
// 측정 주기 < 제어 주기 처리 (2026-10-22): 실차 /odometry/filtered 50 Hz, /measured_wheel_speeds 48 Hz 인데 제어는 100 Hz.
//   증분형 x = [Δx_m; X_e] 는 매 틱 Δx_m 을 차분하므로, 묵은 포즈를 그대로 쓰면 "한 틱 정지 / 다음 틱 두 배 이동" 이
//   번갈아 보여 명령이 50 Hz 로 떤다. 시뮬(tools/sim pose_hz=50): 4 s 이후 RMS 4.45 cm, 명령 총변동 98 (매 틱 포즈면 0.001 cm, 3.1).
//   pose_extrapolation: 포즈를 메시지 stamp 부터 현재 틱까지 메시지의 몸체 twist 로 외삽한다 (SE(2) 정확 적분)
//                       → 0.001 cm, 3.3 으로 회복.
//   바퀴속도: 새 메시지가 없는 틱에는 관측기가 예측만 하고 보정하지 않는다 (같은 측정을 두 번 넣지 않는다).
// 안전: reference / odom / wheel 끊김 → 0 출력 + 리셋.  /cmd_vel 발행자가 있으면 경고.
// 종료: SIGINT 를 직접 받아 0 명령을 낸 뒤 CSV 를 쓰고 내려간다 (rclcpp 기본 핸들러는 발행 전에 컨텍스트를 닫는다).
// ---------------------------------------------------------------
#include "amr_mpc_cpp/reference.hpp"
#include "amr_mpc_cpp/wheel_mpc.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <amr_interfaces/msg/planar_reference_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
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
using amr_mpc_cpp::HoldLatch;
using amr_mpc_cpp::ReferenceHorizon;
using amr_mpc_cpp::Vec3;
using amr_mpc_cpp::Vec4;
using amr_mpc_cpp::WheelModel;
using amr_mpc_cpp::WheelMPC;
using amr_mpc_cpp::WheelTuning;

namespace
{
std::atomic<bool> g_stop{false};
void on_sigint(int) {g_stop = true;}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string default_plant_file()
{
  if (const char * env = std::getenv("AMR_MPC_PLANT_WHEEL_YAML")) {return env;}   // amr_mpc/core/config.py 와 같은 규칙
  return ament_index_cpp::get_package_share_directory("amr_mpc") + "/config/plant_wheel.yaml";
}

Vec4 wheel_map(const YAML::Node & n)
{
  return Vec4(n["FL"].as<double>(), n["FR"].as<double>(), n["RL"].as<double>(), n["RR"].as<double>());
}
}  // namespace

class WheelMpcNode : public rclcpp::Node
{
public:
  WheelMpcNode()
  : Node("wheel_mpc_node")
  {
    const std::string plant_file = declare_parameter<std::string>("plant_file", default_plant_file());
    const std::string section = declare_parameter<std::string>("tuning_section", "tuning_100hz");
    std::string model_section = declare_parameter<std::string>("model_section", "model_100hz");
    const YAML::Node y = YAML::LoadFile(plant_file);
    if (!y[model_section]) {
      RCLCPP_WARN(get_logger(), "%s 에 %s 절이 없어 model 을 쓴다", plant_file.c_str(), model_section.c_str());
      model_section = "model";
    }
    const YAML::Node ym = y[model_section], yg = y["geometry"], yl = y["limits"], ys = y["safety"], yh = y["hold"];
    const YAML::Node yt = y[section];
    if (!yt) {throw std::runtime_error(plant_file + " 에 " + section + " 절이 없다");}

    WheelModel md;
    md.tau = wheel_map(ym["tau"]);
    md.k_dc = wheel_map(ym["k_dc"]);
    md.theta = declare_parameter<double>("theta", ym["theta"].as<double>());
    md.wheel_radius = yg["wheel_radius_m"].as<double>();
    md.wheel_base = yg["wheel_base_m"].as<double>();
    md.robot_width = yg["robot_width_m"].as<double>();
    md.w_max = yl["max_wheel_rad_per_sec"].as<double>();
    md.alpha_max = yl["max_wheel_accel"].as<double>();

    WheelTuning tu;
    tu.Ts = declare_parameter<double>("Ts", yt["Ts"].as<double>());
    tu.Np = declare_parameter<int>("Np", yt["Np"].as<int>());
    tu.Nc = declare_parameter<int>("Nc", yt["Nc"].as<int>());
    tu.q_pos = declare_parameter<double>("q_pos", yt["q_pos"].as<double>());
    tu.q_yaw = declare_parameter<double>("q_yaw", yt["q_yaw"].as<double>());
    tu.r_dw = declare_parameter<double>("r_dw", yt["r_dw"].as<double>());
    tu.q_null = declare_parameter<double>("q_null", yt["q_null"].as<double>());
    tu.k_obs = declare_parameter<double>("k_obs", yt["k_obs"].as<double>());
    tu.daqp_iter_limit = declare_parameter<int>("daqp_iter_limit", yt["daqp_iter_limit"].as<int>(1000));
    mpc_ = std::make_unique<WheelMPC>(md, tu);

    const auto ref_topic = declare_parameter<std::string>("reference_topic", "/dra_mppi/reference_trajectory");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto wheel_topic = declare_parameter<std::string>("wheel_topic", "/measured_wheel_speeds");
    const auto cmd_topic = declare_parameter<std::string>("wheel_cmd_topic", "/wheel_cmd");
    ref_timeout_ = declare_parameter<double>("ref_timeout", ys["ref_timeout"].as<double>());
    odom_timeout_ = declare_parameter<double>("odom_timeout", ys["odom_timeout"].as<double>());
    wheel_timeout_ = declare_parameter<double>("wheel_timeout", ys["wheel_timeout"].as<double>(0.3));
    budget_ = declare_parameter<double>("load_budget", yl["load_budget"].as<double>());
    debug_ = declare_parameter<bool>("publish_debug", true);
    pose_extrapolation_ = declare_parameter<bool>("pose_extrapolation", true);
    const auto log_prefix = declare_parameter<std::string>("log_prefix", "");
    const double deg = M_PI / 180.0;
    latch_.emplace(
      declare_parameter<double>("hold_enter_xy", yh["enter_xy"].as<double>()),
      declare_parameter<double>("hold_enter_yaw_deg", yh["enter_yaw_deg"].as<double>()) * deg,
      declare_parameter<double>("hold_exit_xy", yh["exit_xy"].as<double>()),
      declare_parameter<double>("hold_exit_yaw_deg", yh["exit_yaw_deg"].as<double>()) * deg);
    hold_stopped_wheel_ = declare_parameter<double>("hold_stopped_wheel", yh["stopped_wheel"].as<double>());
    if (!log_prefix.empty()) {
      char buf[32];
      const std::time_t now = std::time(nullptr);
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
      csv_path_ = log_prefix + "_" + buf + ".csv";
    }

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(cmd_topic, 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/wheel_mpc/reference_horizon", 1);
    pub_status_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_mpc/status", 10);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto & p = m->pose.pose;
        X_ = Vec3(p.position.x, p.position.y, yaw_of(p.orientation));
        v_odom_ = Vec3(m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.angular.z);
        last_odom_ = now_s();
        const double st = rclcpp::Time(m->header.stamp).seconds();
        odom_stamp_ = st > 0.0 ? st : last_odom_;          // stamp 가 비어 있으면 수신 시각
      });
    sub_wheel_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      wheel_topic, 20, [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) {
        if (m->data.size() != 4) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "measured_wheel_speeds 길이 %zu ≠ 4", m->data.size());
          return;
        }
        w_ = Vec4(m->data[0], m->data[1], m->data[2], m->data[3]);
        last_wheel_ = now_s();
        w_fresh_ = true;
      });
    sub_ref_ = create_subscription<amr_interfaces::msg::PlanarReferenceArray>(
      ref_topic, 5, [this](amr_interfaces::msg::PlanarReferenceArray::ConstSharedPtr m) {on_ref(*m);});
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(tu.Ts), [this]() {on_tick();});
    competition_timer_ = rclcpp::create_timer(this, get_clock(), 2s, [this]() {check_competition();});
    RCLCPP_INFO(
      get_logger(),
      "구조 B (바퀴 입력, C++/DAQP).  기준 %s  포즈 %s  바퀴 %s → %s   Ts %.0f ms  Np %d  Nc %d  θ %.0f ms (d=%d)  "
      "|ω| ≤ %.0f rad/s  [%s : %s, %s]",
      ref_topic.c_str(), odom_topic.c_str(), wheel_topic.c_str(), cmd_topic.c_str(), tu.Ts * 1e3, tu.Np, tu.Nc,
      md.theta * 1e3, mpc_->d(), md.w_max, plant_file.c_str(), model_section.c_str(), section.c_str());
  }

  void finish()
  {
    if (done_) {return;}
    done_ = true;
    publish_cmd(Vec4::Zero());
    if (csv_path_.empty() || rows_.empty()) {return;}
    std::ofstream f(csv_path_);
    f.precision(15);   // t 가 1.8e9 s 라 기본 6 자리면 모든 행이 같은 시각으로 잘린다
    f << "t,x,y,th,xr,yr,thr,xe,ye,the,w_fl,w_fr,w_rl,w_rr,wc_fl,wc_fr,wc_rl,wc_rr,load,qp_iters,fallback,step_ms,ref_age\n";
    double se = 0.0, emax = 0.0, step_sum = 0.0, step_max = 0.0;
    for (const auto & r : rows_) {
      for (size_t i = 0; i < r.size(); ++i) {f << (i ? "," : "") << r[i];}
      f << "\n";
      const double e = std::hypot(r[7], r[8]);
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

  void publish_cmd(const Vec4 & w)
  {
    std_msgs::msg::Float32MultiArray m;
    m.data = {static_cast<float>(w(0)), static_cast<float>(w(1)), static_cast<float>(w(2)), static_cast<float>(w(3))};
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

  void check_competition()
  {
    const size_t n = count_publishers("/cmd_vel");
    if (n > 0 && active_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000, "/cmd_vel 발행자 %zu개 — 드라이버가 두 명령을 받는다. wheel_cmd 와 경쟁", n);
    }
  }

  void stop(const char * reason)
  {
    publish_cmd(Vec4::Zero());
    if (active_) {RCLCPP_INFO(get_logger(), "정지: %s", reason);}
    active_ = false;
    w_prev_.setZero();
    mpc_->reset();
  }

  void on_tick()
  {
    const double now = now_s();
    if (!X_ || now - last_odom_ > odom_timeout_) {
      stop("odom 없음");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom 대기/끊김");
      return;
    }
    if (!w_ || now - last_wheel_ > wheel_timeout_) {
      stop("wheel 없음");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "measured_wheel_speeds 대기/끊김");
      return;
    }
    if (!ref_) {
      stop("기준 없음");
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
      w_prev_.setZero();
      RCLCPP_INFO(get_logger(), "추종 시작. 기준열 %d×%.2f s", ref_->size(), ref_->dt());
    }
    // 포즈를 현재 틱으로 외삽 (머리주석). 나이는 [0, odom_timeout] 로 자른다 — 그 이상이면 위에서 이미 정지했다
    const double age = std::clamp(now - odom_stamp_, 0.0, odom_timeout_);
    const Vec3 X = pose_extrapolation_ ? amr_mpc_cpp::extrapolate_pose(*X_, v_odom_, age) : *X_;
    const auto hz = ref_->horizon(now, mpc_->Ts(), mpc_->Np());
    const Vec3 Xr = hz[0].first, ur = hz[0].second;
    Vec3 Xe = Xr - X;
    Xe(2) = amr_mpc_cpp::wrap_angle(Xe(2));
    const bool was_idle = latch_->idle();
    if (latch_->update(ref_->is_hold(), Xe, w_->cwiseAbs().maxCoeff() < hold_stopped_wheel_)) {
      if (!was_idle) {
        RCLCPP_INFO(
          get_logger(), "hold 진입: 오차 %.1f mm, %.2f° — 명령 0, MPC 리셋",
          std::hypot(Xe(0), Xe(1)) * 1e3, Xe(2) * 180.0 / M_PI);
        mpc_->reset();
        w_prev_.setZero();
      }
      publish_cmd(Vec4::Zero());
      if (debug_) {publish_status({now, Xe(0), Xe(1), Xe(2), 0.0, 0.0, 0.0, 0.0, now - ref_->t0()});}
      return;
    }
    if (was_idle) {
      RCLCPP_INFO(
        get_logger(), "hold 해제: 오차 %.1f mm, %.2f° — 추종 재개",
        std::hypot(Xe(0), Xe(1)) * 1e3, Xe(2) * 180.0 / M_PI);
    }
    ref_pts_.resize(hz.size());
    for (size_t i = 0; i < hz.size(); ++i) {ref_pts_[i] = Vec3(hz[i].first(2), hz[i].second(0), hz[i].second(1));}
    const auto t_c = std::chrono::steady_clock::now();
    const Vec4 wc = mpc_->step(X, *w_, w_prev_, ref_pts_, Xr, ur, w_fresh_);
    w_fresh_ = false;
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_c).count();
    w_prev_ = wc;
    publish_cmd(wc);
    const double load = (mpc_->mecanum().T_inv * ur).cwiseAbs().maxCoeff() / mpc_->w_max();
    if (load > budget_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "기준 바퀴속도 %.0f %% > 예산 %.0f %%", load * 100, budget_ * 100);
    }
    const auto & st = mpc_->stats();
    const double wc_load = wc.cwiseAbs().maxCoeff() / mpc_->w_max();
    const double ref_age = now - ref_->t0();
    if (!csv_path_.empty()) {
      rows_.push_back(
        {now, X(0), X(1), X(2), Xr(0), Xr(1), Xr(2), Xe(0), Xe(1), Xe(2),
          (*w_)(0), (*w_)(1), (*w_)(2), (*w_)(3), wc(0), wc(1), wc(2), wc(3), wc_load,
          static_cast<double>(st.iters), static_cast<double>(st.fallback), ms, ref_age});
    }
    if (debug_) {
      publish_status({now, Xe(0), Xe(1), Xe(2), wc_load, static_cast<double>(st.iters),
          static_cast<double>(st.fallback), ms, ref_age});
      nav_msgs::msg::Path path;
      path.header.frame_id = ref_frame_.empty() ? "odom" : ref_frame_;
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

  void publish_status(std::vector<double> v)
  {
    std_msgs::msg::Float64MultiArray m;
    m.data = std::move(v);
    pub_status_->publish(m);
  }

  std::unique_ptr<WheelMPC> mpc_;
  std::optional<HoldLatch> latch_;
  std::optional<ReferenceHorizon> ref_;
  std::optional<Vec3> X_;
  std::optional<Vec4> w_;
  Vec3 v_odom_ = Vec3::Zero();
  double odom_stamp_ = 0.0;
  bool w_fresh_ = false, pose_extrapolation_ = true;
  Vec4 w_prev_ = Vec4::Zero();
  std::vector<Vec3> ref_pts_;
  double last_odom_ = 0.0, last_wheel_ = 0.0, ref_rx_ = 0.0;
  double ref_timeout_ = 0.5, odom_timeout_ = 0.3, wheel_timeout_ = 0.3, budget_ = 0.65, hold_stopped_wheel_ = 1.0;
  bool debug_ = true, active_ = false, done_ = false;
  std::string ref_frame_, csv_path_;
  std::vector<std::array<double, 23>> rows_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wheel_;
  rclcpp::Subscription<amr_interfaces::msg::PlanarReferenceArray>::SharedPtr sub_ref_;
  rclcpp::TimerBase::SharedPtr timer_, competition_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  int rc = 0;
  {
    std::shared_ptr<WheelMpcNode> node;
    try {
      node = std::make_shared<WheelMpcNode>();
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[wheel_mpc_node] 기동 실패: %s\n", e.what());
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && !g_stop) {exec.spin_once(50ms);}
    node->finish();
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return rc;
}
