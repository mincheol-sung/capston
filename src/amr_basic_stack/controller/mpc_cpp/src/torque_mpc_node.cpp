// 토크 입력 MPC 노드 (구조도 v3 의 torque_mpc_node).  핵심 계산은 torque_mpc.hpp, 식은 『메카넘 토크 MPC 1판 설계』.
// ---------------------------------------------------------------
//   reference_topic   PlanarReferenceArray   기준열 (figure8_reference_node, 10 Hz, 0.05 s × 80 — torque_track.yaml)
//   state_topic       nav_msgs/Odometry      상태 추정값 x̂.  pose = map 좌표 [x, y, yaw],  twist = 몸체 [v_x, v_y, ω_z]
//   /wheel_states     WheelStates            늘 받는다 (2026-11-23 슬립 계수 η_s — 켜지 않아도 λ̃·η_s 를 /torque_mpc/slip 에 기록).
//                                            velocity_source:=wheels 면 (2026-11-18) 몸체 v_x·v_y 를 바퀴 기구학 T ω 로 바꾼다.
//                                            ω_z 는 추정기 값(자이로) 그대로. 실차 bag 에서 추정기 v_x 가 바퀴보다 90~120 ms 늦었다.
//                                            바퀴 값이 wheel_timeout 보다 오래되면 그 주기는 추정기 속도로 (로그).
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
#include <cstdio>
#include <string>
#include <amr_interfaces/msg/planar_reference_array.hpp>
#include <amr_interfaces/msg/wheel_states.hpp>
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
    md.J_w = yp["wheel"]["inertia_J_w_kgm2"].as<double>();          // m_w = 4J_w/r² 는 M 에 늘 들어간다 (지시서 2026-11-16)
    // w 항 스위치 (식-6, 지시서 1절 use_w). 제어기 설정이라 torque_mpc.yaml model_terms 에 둔다 — torque_plant.yaml 의
    // model.use_wheel_inertia 는 이제 시뮬 플랜트 물리만 정한다
    md.use_w = declare_parameter<bool>("use_w", ym["model_terms"] ? ym["model_terms"]["use_w"].as<bool>(false) : false);
    md.tau_rated = yp["motor"]["tau_rated_Nm"].as<double>();
    md.tau_fric_max = yp["friction"]["tau_fric_max_Nm"].as<double>();
    md.mu_hat = declare_parameter<double>("mu_hat", yp["surface"]["mu_hat"].as<double>());
    // 바퀴별 수직력 몫 [FL, FR, RL, RR] (2026-11-19, torque_plant.yaml body.wheel_load_share — 정지 저울 측정).
    //   비우면 1/4 씩. 합이 1 ± 5 % 밖이거나 0.1~0.4 밖이면 기동 실패, 안이면 합 1 로 맞춘다
    const std::vector<double> share = declare_parameter<std::vector<double>>(
      "wheel_load_share", yp["body"]["wheel_load_share"] ? yp["body"]["wheel_load_share"].as<std::vector<double>>() :
      std::vector<double>{});
    if (!share.empty()) {
      if (share.size() != 4) {throw std::invalid_argument("wheel_load_share 는 4 개 [FL, FR, RL, RR] 이거나 비어 있어야 한다");}
      double sum = 0.0;
      for (int i = 0; i < 4; ++i) {
        if (!(share[i] >= 0.1 && share[i] <= 0.4)) {throw std::invalid_argument("wheel_load_share 값은 0.1 ~ 0.4");}
        sum += share[i];
      }
      if (std::abs(sum - 1.0) > 0.05) {throw std::invalid_argument("wheel_load_share 합이 1 ± 0.05 밖");}
      for (int i = 0; i < 4; ++i) {md.wheel_load_share(i) = share[i] / sum;}
    }
    // 바퀴별 μ̂ (2026-11-19 (11), torque_plant.yaml surface.mu_hat_wheels). 2026-11-22 부터 N_i 기준 — 그 바퀴는 mu_hat 대신
    //   이 값에 하중 몫을 곱한다. 비우거나 0 이하(launch mu_hat_wheels:=off → −1) 인 바퀴는 mu_hat × 하중 몫
    const std::vector<double> muw = declare_parameter<std::vector<double>>(
      "mu_hat_wheels", yp["surface"]["mu_hat_wheels"] ? yp["surface"]["mu_hat_wheels"].as<std::vector<double>>() :
      std::vector<double>{});
    if (!muw.empty()) {
      if (muw.size() != 4) {throw std::invalid_argument("mu_hat_wheels 는 4 개 [FL, FR, RL, RR] 이거나 비어 있어야 한다");}
      for (int i = 0; i < 4; ++i) {
        if (muw[i] > 2.0) {throw std::invalid_argument("mu_hat_wheels 값은 2 이하 (0 이하 = 그 바퀴는 mu_hat × 하중 몫)");}
        md.mu_hat_wheels(i) = muw[i];
      }
    }
    md.c_s = yp["surface"]["c_s"].as<double>();
    md.gravity = yp["model"]["gravity"].as<double>();
    {   // 2026-11-23 구동계 방향 보정 (torque_plant.yaml drive, 기본 0). launch drive_yaw_deg·drive_slip_coef 로 덮어쓴다
      const YAML::Node yd = yp["drive"];
      const double yaw_deg = declare_parameter<double>("drive_yaw_offset_deg", yd ? yd["yaw_offset_deg"].as<double>(0.0) : 0.0);
      const double coef_deg = declare_parameter<double>(
        "drive_slip_coef_deg", yd ? yd["slip_coef_deg_per_mps2"].as<double>(0.0) : 0.0);
      md.slip_ac_max = declare_parameter<double>("drive_slip_ac_max", yd ? yd["slip_ac_max_mps2"].as<double>(2.0) : 2.0);
      if (!(std::abs(yaw_deg) <= 10.0) || !(std::abs(coef_deg) <= 10.0) || !(md.slip_ac_max >= 0.0 && md.slip_ac_max <= 5.0)) {
        throw std::invalid_argument("drive: |yaw_offset_deg| ≤ 10, |slip_coef_deg| ≤ 10 °/(m/s²), 0 ≤ slip_ac_max ≤ 5 m/s²");
      }
      md.drive_yaw = yaw_deg * M_PI / 180.0;
      md.slip_coef = coef_deg * M_PI / 180.0;
    }

    TorqueTuning tu;
    const YAML::Node yt = ym["tuning"];
    tu.Ts = declare_parameter<double>("Ts", yt["Ts"].as<double>());
    tu.Np = declare_parameter<int>("Np", yt["Np"].as<int>());
    tu.Nc = declare_parameter<int>("Nc", yt["Nc"].as<int>());
    const std::vector<double> W = declare_parameter<std::vector<double>>("W", yt["W"].as<std::vector<double>>());
    if (W.size() != 6) {throw std::invalid_argument("tuning.W 는 6개여야 한다 [x, y, θ, ẋ, ẏ, θ̇]");}
    for (int i = 0; i < 6; ++i) {tu.W(i) = W[i];}
    // 입력 변화 벌점 (2026-11-18 교수님 지시: Δτ 를 시간 미분으로). w_dudt = r_τ̇ [(N·m/s)⁻²] 로 ‖Δτ/Ts‖² 에 건다
    //   → r_τ̇ ‖Δτ/Ts‖² = (r_τ̇/Ts²) ‖Δτ‖² 이므로 코어(식-16·18)에는 스텝 단위 r_Δ = r_τ̇/Ts² 로 넘긴다 (코어·1판 수치 시험은 그대로).
    //   예전 스텝 단위 w_delta [(N·m)⁻²] 를 주면(launch mpc_w_delta, 옛 yaml) 그 값이 우선한다.
    const double w_dudt = declare_parameter<double>("w_dudt", yt["w_dudt"] ? yt["w_dudt"].as<double>() : -1.0);
    const double w_step = declare_parameter<double>("w_delta", yt["w_delta"] ? yt["w_delta"].as<double>() : -1.0);
    if (w_step >= 0.0) {
      tu.w_delta = w_step;
      w_delta_src_ = "w_delta (스텝 단위) 직접";
    } else if (w_dudt >= 0.0) {
      tu.w_delta = w_dudt / (tu.Ts * tu.Ts);
      w_delta_src_ = "w_dudt " + std::to_string(w_dudt) + " (N·m/s)⁻² / Ts²";
    } else {
      throw std::invalid_argument("tuning.w_dudt (≥ 0) 또는 w_delta 가 있어야 한다");
    }
    tu.q_null = declare_parameter<double>("q_null", yt["q_null"].as<double>(0.0));
    tu.r_u = declare_parameter<double>("r_u", yt["r_u"].as<double>(0.0));
    acc_dt_ = declare_parameter<double>("acc_dt", yt["acc_dt"].as<double>(0.2));     // 기준 가속도 중앙 차분 창 (지시서 3.9)
    // 2026-11-20: 창이 기준열 첫 행 앞에 걸치면 민다 (reference.hpp acc_window_start). false = 예전 동작 (비교 시험용)
    acc_window_shift_ = declare_parameter<bool>("acc_window_shift", yt["acc_window_shift"] ? yt["acc_window_shift"].as<bool>() : true);
    if (!(acc_dt_ > 0.0)) {throw std::invalid_argument("acc_dt > 0 이어야 한다");}
    tu.daqp_iter_limit = declare_parameter<int>("daqp_iter_limit", yt["daqp_iter_limit"].as<int>(1000));
    limit_mode_ = declare_parameter<std::string>("limit_mode", ym["torque_limit"]["mode"].as<std::string>());
    tu.limit_mode = amr_mpc_cpp::parse_limit_mode(limit_mode_);
    // 선형화 방식 (torque_mpc.hpp 머리주석). YAML 에 절이 없으면 reference (1판 방식, 2026-11-14 기본 유지 결정)
    linearization_ = declare_parameter<std::string>(
      "linearization", ym["linearization"] ? ym["linearization"]["mode"].as<std::string>("reference") : "reference");
    tu.linearization = amr_mpc_cpp::parse_linearization(linearization_);
    tu.rate_fraction = declare_parameter<double>("rate_fraction", ym["torque_limit"]["rate_fraction"].as<double>());
    // 2026-11-22 마찰 상한을 거는 곳: total = 바퀴 토크 (1판) | ground = 바닥 토크 P τ (TorqueModel::ground_map)
    cap_on_ = declare_parameter<std::string>("cap_on", ym["torque_limit"]["cap_on"].as<std::string>("total"));
    if (cap_on_ != "total" && cap_on_ != "ground") {throw std::invalid_argument("cap_on 은 total | ground — " + cap_on_);}
    tu.cap_on_ground = cap_on_ == "ground";
    tu.dist_enable = declare_parameter<bool>("disturbance_enable", ym["disturbance"]["enable"].as<bool>());
    tu.dist_alpha = declare_parameter<double>("disturbance_alpha", ym["disturbance"]["alpha"].as<double>());
    tu.dist_dt_tol = declare_parameter<double>("disturbance_dt_tol", ym["disturbance"]["dt_tol"].as<double>(0.002));
    tu.dist_skip_xy = declare_parameter<double>("disturbance_skip_xy", ym["disturbance"]["skip_xy"].as<double>(1e-3));
    tu.dist_accel_only = declare_parameter<bool>("disturbance_accel_only", ym["disturbance"]["accel_only"].as<bool>(false));
    {   // 2026-11-21: 몸체 좌표·포화 중 멈춤·상한 (torque_mpc.hpp 머리주석). yaml 에 없으면 예전 동작
      const std::string fr = declare_parameter<std::string>("disturbance_frame", ym["disturbance"]["frame"].as<std::string>("world"));
      if (fr != "world" && fr != "body") {throw std::invalid_argument("disturbance_frame 은 world | body (받은 값 " + fr + ")");}
      tu.dist_body_frame = fr == "body";
      tu.dist_freeze_on_sat = declare_parameter<bool>(
        "disturbance_freeze_on_saturation", ym["disturbance"]["freeze_on_saturation"].as<bool>(false));
      tu.dist_acc_max = declare_parameter<double>("disturbance_acc_max", ym["disturbance"]["acc_max"].as<double>(-1.0));
      tu.dist_yaw_acc_max = declare_parameter<double>("disturbance_yaw_acc_max", ym["disturbance"]["yaw_acc_max"].as<double>(-1.0));
      // 2026-11-21 정상상태 Kalman 이득 (torque_mpc.hpp 머리주석 "외란 관측기 이득"). observer: lpf (α) | kalman
      const std::string ob = declare_parameter<std::string>("disturbance_observer", ym["disturbance"]["observer"].as<std::string>("lpf"));
      if (ob != "lpf" && ob != "kalman") {throw std::invalid_argument("disturbance_observer 는 lpf | kalman (받은 값 " + ob + ")");}
      tu.dist_kalman = ob == "kalman";
      auto vec3 = [&](const char * name, const char * key, const TVec3 & def) {
          std::vector<double> d0(def.data(), def.data() + 3);
          const auto yn = ym["disturbance"][key];
          if (yn && yn.IsSequence() && yn.size() == 3) {d0 = yn.as<std::vector<double>>();}
          const auto v = declare_parameter<std::vector<double>>(name, d0);
          if (v.size() != 3) {throw std::invalid_argument(std::string(name) + " 은 3 개 [x, y, θ]");}
          return TVec3(v[0], v[1], v[2]);
        };
      tu.dist_kq = vec3("disturbance_kalman_q", "kalman_q", tu.dist_kq);
      tu.dist_kr_pos = vec3("disturbance_kalman_r_pos", "kalman_r_pos", tu.dist_kr_pos);
      tu.dist_kr_vel = vec3("disturbance_kalman_r_vel", "kalman_r_vel", tu.dist_kr_vel);
    }
    {   // 2026-11-23 하중 이동 (torque_mpc.hpp TorqueTuning load_*). yaml load_transfer 절이 없거나 com_height 0 이면 끔
      const auto yl = ym["load_transfer"];
      tu.load_com_height = declare_parameter<double>("load_com_height", yl ? yl["com_height_m"].as<double>(0.0) : 0.0);
      tu.load_min_frac = declare_parameter<double>("load_min_frac", yl ? yl["min_frac"].as<double>(0.2) : 0.2);
      tu.load_weighted_alloc = declare_parameter<bool>("load_weighted_alloc", yl ? yl["weighted_alloc"].as<bool>(true) : true);
      tu.load_dynamic_cap = declare_parameter<bool>("load_dynamic_cap", yl ? yl["dynamic_cap"].as<bool>(true) : true);
    }
    {   // 2026-11-21 제약 있는 목표 입력·구심 feedforward 계수 ρ (torque_mpc.hpp 머리주석). yaml 에 없으면 예전 동작
      const auto yf = ym["feedforward"];
      tu.target_projection = declare_parameter<bool>("target_projection", yf ? yf["target_projection"].as<bool>(false) : false);
      tu.ff_normal_scale = declare_parameter<double>("ff_normal_scale", yf ? yf["normal_scale"].as<double>(1.0) : 1.0);
      tu.ff_normal_estimate = declare_parameter<bool>("ff_normal_estimate", yf ? yf["normal_estimate"].as<bool>(false) : false);
      tu.ff_normal_q = declare_parameter<double>("ff_normal_q", yf ? yf["normal_q"].as<double>(0.03) : 0.03);
      tu.ff_normal_min = declare_parameter<double>("ff_normal_min", yf ? yf["normal_min"].as<double>(0.4) : 0.4);
      tu.ff_normal_max = declare_parameter<double>("ff_normal_max", yf ? yf["normal_max"].as<double>(1.2) : 1.2);
      tu.ff_normal_P0 = declare_parameter<double>("ff_normal_P0", yf ? yf["normal_P0"].as<double>(0.04) : 0.04);
      tu.ff_normal_x_min = declare_parameter<double>("ff_normal_x_min", yf ? yf["normal_x_min"].as<double>(0.3) : 0.3);
    }
    amr_mpc_cpp::SlipEtaParams sp;
    {   // 2026-11-23 슬립 계수 η_s (torque_mpc.hpp 머리주석). yaml slip_eta 절이 없으면 끔. 꺼도 λ̃·η_s 는 계산해 /torque_mpc/slip 에 낸다
      const auto ye = ym["slip_eta"];
      slip_on_ = declare_parameter<bool>("slip_eta_enable", ye ? ye["enable"].as<bool>(false) : false);
      sp.lambda_c = declare_parameter<double>("slip_eta_lambda_c", ye ? ye["lambda_c"].as<double>(sp.lambda_c) : sp.lambda_c);
      sp.s = declare_parameter<double>("slip_eta_s", ye ? ye["s"].as<double>(sp.s) : sp.s);
      sp.fc = declare_parameter<double>("slip_eta_fc", ye ? ye["fc_hz"].as<double>(sp.fc) : sp.fc);
      sp.omega_eps = declare_parameter<double>("slip_eta_omega_eps", ye ? ye["omega_eps"].as<double>(sp.omega_eps) : sp.omega_eps);
      tu.slip_alloc = declare_parameter<bool>("slip_eta_alloc", ye ? ye["weighted_alloc"].as<bool>(true) : true);
      tu.slip_alloc_floor = declare_parameter<double>("slip_eta_alloc_floor", ye ? ye["alloc_floor"].as<double>(0.05) : 0.05);
    }
    slip_.emplace(sp);
    mpc_ = std::make_unique<TorqueMPC>(md, tu);

    const auto ref_topic = declare_parameter<std::string>("reference_topic", "/dra_mppi/reference_trajectory");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/state_estimator/odom");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/torque_mpc/cmd_raw");
    const auto armed_topic = declare_parameter<std::string>("armed_topic", "/mpc/armed");
    // 속도 상태 출처 (2026-11-18): estimator | wheels.  yaml state_input 절이 없으면 estimator
    const YAML::Node ys = ym["state_input"];
    velocity_source_ = declare_parameter<std::string>(
      "velocity_source", ys ? ys["velocity_source"].as<std::string>("estimator") : "estimator");
    if (velocity_source_ != "estimator" && velocity_source_ != "wheels") {
      throw std::invalid_argument("velocity_source 는 estimator | wheels — " + velocity_source_);
    }
    wheel_timeout_ = declare_parameter<double>("wheel_timeout", ys ? ys["wheel_timeout"].as<double>(0.05) : 0.05);
    const auto wheel_topic = declare_parameter<std::string>("wheel_states_topic", "/wheel_states");
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
    pub_adapt_ = create_publisher<std_msgs::msg::Float64MultiArray>("/torque_mpc/adapt", 10);
    pub_load_ = create_publisher<std_msgs::msg::Float64MultiArray>("/torque_mpc/load", 10);
    pub_slip_ = create_publisher<std_msgs::msg::Float64MultiArray>("/torque_mpc/slip", 10);
    sub_state_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic, 20, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_state(*m);});
    sub_ref_ = create_subscription<amr_interfaces::msg::PlanarReferenceArray>(
      ref_topic, 5, [this](amr_interfaces::msg::PlanarReferenceArray::ConstSharedPtr m) {on_ref(*m);});
    // 바퀴 상태: velocity_source wheels 의 몸체 속도 + 슬립 계수 η_s (2026-11-23 — η_s 를 꺼도 기록용으로 늘 받는다)
    sub_wheels_ = create_subscription<amr_interfaces::msg::WheelStates>(
      wheel_topic, 20, [this](amr_interfaces::msg::WheelStates::ConstSharedPtr m) {on_wheels(*m);});
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
      "  Ts %.0f ms  Np %d  Nc %d   m %.2f kg  I_z %.3f  바퀴 등가 질량 m_w %.3f kg (M 에 늘)  w 항 %s\n"
      "  상한 모드 %s: τ̄ %.4f N·m, Δτ̄ %.4f N·m/스텝  (모터 %.3f, 마찰 τ_μ %.4f @ μ̂ %.2f)   외란 추정치 %s (α %.3f%s, %s 좌표%s, 상한 %.2f m/s² · %.2f rad/s²)",
      ref_topic.c_str(), state_topic.c_str(), cmd_topic.c_str(), tu.Ts * 1e3, tu.Np, tu.Nc, md.mass, md.inertia_z,
      md.mu(), md.use_w ? "켬" : "끔", limit_mode_.c_str(), tb(0), mpc_->dtau_bar_of(tb(0)),
      md.tau_motor_limit(), md.tau_friction_limit(), md.mu_hat, tu.dist_enable ? "켬" : "끔", tu.dist_alpha,
      tu.dist_accel_only ? ", 가속도 외란 모델" : "", tu.dist_body_frame ? "몸체" : "월드",
      tu.dist_freeze_on_sat ? ", 포화 중 멈춤" : "", tu.dist_acc_max, tu.dist_yaw_acc_max);
    if (mpc_->cap_on_ground()) {
      const Eigen::Matrix4d P = md.ground_map();
      const double fwd = P.row(0).sum();                 // 네 바퀴 같은 토크로 앞으로 밀 때 바퀴 0 의 바닥 몫
      RCLCPP_INFO(
        get_logger(), "  마찰 상한을 바닥 토크에 건다 (cap_on ground): |(P τ)_i| ≤ τ̄_i, |τ_i| ≤ %.3f (모터).  앞 고른 토크의 바닥 몫 %.3f, P 대각 [%.3f %.3f %.3f %.3f]",
        md.tau_motor_limit(), fwd, P(0, 0), P(1, 1), P(2, 2), P(3, 3));
    } else if (cap_on_ == "ground") {
      RCLCPP_WARN(get_logger(), "  cap_on ground 은 limit_mode friction 에서만 — 지금은 바퀴 토크 상한 그대로");
    }
    if (tu.dist_kalman) {
      const auto & K = mpc_->dist_kalman_gain();
      RCLCPP_INFO(
        get_logger(), "  외란 관측기: 정상상태 Kalman — 축 x·y·θ  [K_pos, K_vel] = [%.3g, %.4g] · [%.3g, %.4g] · [%.3g, %.4g]"
        "  (등가 α = K·H: %.4f · %.4f · %.4f)", K(0, 0), K(0, 1), K(1, 0), K(1, 1), K(2, 0), K(2, 1),
        K(0, 0) * 0.5 * tu.Ts * tu.Ts + K(0, 1) * tu.Ts, K(1, 0) * 0.5 * tu.Ts * tu.Ts + K(1, 1) * tu.Ts,
        K(2, 0) * 0.5 * tu.Ts * tu.Ts + K(2, 1) * tu.Ts);
    }
    RCLCPP_INFO(
      get_logger(), "  목표 입력 사영 %s   구심 feedforward 계수 ρ %s (시작 %.2f, η random walk q %.3f /√s, [%.2f, %.2f])",
      tu.target_projection ? "켬" : "끔", tu.ff_normal_estimate ? "d̂ 와 결합 Kalman 추정" : "고정", mpc_->ff_normal_rho(), tu.ff_normal_q,
      tu.ff_normal_min, tu.ff_normal_max);
    if (tu.load_com_height > 0.0) {
      RCLCPP_INFO(
        get_logger(), "  하중 이동 켬: 무게중심 높이 %.3f m (L %.4f, W %.4f), 배분 %s, 스텝별 상한 %s, N ≥ %.2f × 정지 — /torque_mpc/load",
        tu.load_com_height, md.wheel_base, md.robot_width, tu.load_weighted_alloc ? "마찰 사용률 가중" : "최소 노름",
        tu.load_dynamic_cap ? (limit_mode_ == "friction" ? "켬" : "friction 이 아니라 안 씀") : "끔", tu.load_min_frac);
    }
    if ((md.wheel_load_share.array() - 0.25).abs().maxCoeff() > 1e-9 || (md.mu_hat_wheels.array() > 0.0).any()) {
      const TVec4 tf = md.tau_friction_limits();
      std::string muw_s = "바퀴별 μ̂ [";
      for (int i = 0; i < 4; ++i) {
        char b[16];
        std::snprintf(b, sizeof(b), "%.3f", md.mu_hat_wheels(i));
        muw_s += (md.mu_hat_wheels(i) > 0.0 ? std::string(b) : std::string("-")) + (i < 3 ? " " : "]");
      }
      RCLCPP_INFO(
        get_logger(), "  %s (N_i 기준, - = μ̂ %.2f)  하중 몫 FL %.4f FR %.4f RL %.4f RR %.4f\n"
        "  → 마찰 τ_μ,i [%.4f %.4f %.4f %.4f] N·m, 쓰는 τ̄ [%.4f %.4f %.4f %.4f]%s",
        muw_s.c_str(), md.mu_hat, md.wheel_load_share(0), md.wheel_load_share(1), md.wheel_load_share(2), md.wheel_load_share(3),
        tf(0), tf(1), tf(2), tf(3), tb(0), tb(1), tb(2), tb(3),
        limit_mode_ == "friction" ? "" : "  (상한 모드가 friction 이 아니라 안 쓰인다)");
    }
    RCLCPP_INFO(
      get_logger(), "  가중: W [%g, %g, %g, %g, %g, %g]  r_Δ %g /스텝 (= %s)  r_U %g  q_null %g",
      tu.W(0), tu.W(1), tu.W(2), tu.W(3), tu.W(4), tu.W(5), tu.w_delta, w_delta_src_.c_str(), tu.r_u, tu.q_null);
    RCLCPP_INFO(
      get_logger(), "  속도 상태: %s", velocity_source_ == "wheels" ?
      ("v_x·v_y = 바퀴 기구학 T ω (" + wheel_topic + ", 묵음 " + std::to_string(int(wheel_timeout_ * 1e3)) +
      " ms 넘으면 추정기), ω_z = 추정기").c_str() : "추정기 그대로");
    if (md.drive_yaw != 0.0 || md.slip_coef != 0.0) {
      RCLCPP_INFO(
        get_logger(), "  구동계 방향 보정: δ0 %+.2f° (force_map xy 행·엔코더 속도), 선회 미끄럼 %+.2f °/(m/s²) × a_c (|a_c| ≤ %.1f, 엔코더 속도만)%s",
        md.drive_yaw * 180.0 / M_PI, md.slip_coef * 180.0 / M_PI, md.slip_ac_max,
        velocity_source_ == "wheels" ? "" : "  — velocity_source 가 wheels 가 아니라 엔코더 속도 보정은 안 쓰인다");
    }
    {
      const auto & p = slip_->params();
      const double l19 = std::log(19.0) * p.s;
      RCLCPP_INFO(
        get_logger(), "  슬립 계수 η_s %s: λ̃ = %.0f Hz LPF(λ, ω_ε %.1f rad/s), η_s = 1/(1+exp((|λ̃| − %.3f)/%.4f)) — 0.95 at %.3f, 0.5 at %.3f, "
        "0.05 at %.3f%s → /torque_mpc/slip", slip_on_ ? "켬 (상한 τ̄_i × η_s,i)" : "끔 (계산·기록만)", p.fc, p.omega_eps,
        p.lambda_c, p.s, p.lambda_c - l19, p.lambda_c, p.lambda_c + l19,
        slip_on_ ? (tu.slip_alloc ? ", U_s 도 η_s 가중 배분" : ", U_s 배분은 그대로") : "");
      if (slip_on_ && limit_mode_ == "friction" && ((md.mu_hat_wheels.array() > 0.0).any() || md.mu_hat < 0.99)) {
        const std::string which = (md.mu_hat_wheels.array() > 0.0).any() ? std::string("바퀴별") : std::to_string(md.mu_hat);
        RCLCPP_WARN(
          get_logger(), "  η_s 를 켰는데 friction 상한이 μ̂ %s 그대로다 — 회의 결정은 μ_max 1.0 으로 풀고 η_s 를 곱하는 것 "
          "(launch mu_hat_wheels:=off mu_hat:=1.0)", which.c_str());
      }
    }
    RCLCPP_INFO(
      get_logger(), "  선형화 %s (%s)", linearization_.c_str(),
      tu.linearization == amr_mpc_cpp::TorqueLinearization::CurrentState ?
      "매 주기 현재 추정 상태 x̂(k), τ(k−1) 에서 야코비안" : "1판: 지평의 기준 방향 θ_r 을 대입");
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
      "step_ms,ref_age,d_x,d_y,d_th,d_xd,d_yd,d_thd,eta_fl,eta_fr,eta_rl,eta_rr\n";   // eta_*: 상한에 곱한 η_s (끄면 1)
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

  // /wheel_states: velocity_source wheels 면 몸체 속도 T ω, 그리고 슬립 계수 η_s 갱신 (2026-11-23).
  //   λ 는 slip_logger 와 같게 — 이 바퀴 메시지와 가장 최근 추정기 몸체 속도 (est_vb_, 바퀴로 바꾸기 전). 응답 없음·에러 바퀴는 λ̃ 를 그대로 둔다
  void on_wheels(const amr_interfaces::msg::WheelStates & m)
  {
    const TVec4 om(m.velocity[0], m.velocity[1], m.velocity[2], m.velocity[3]);
    const double now = now_s();
    if (velocity_source_ == "wheels") {
      wheel_v_ = mpc_->model().wheel_body_velocity(om);
      wheel_rx_ = now;
    }
    if (est_rx_ <= 0.0) {return;}
    const double st = rclcpp::Time(m.header.stamp).seconds();
    const double t = st > 0.0 ? st : now;
    bool valid[4];
    for (int i = 0; i < 4; ++i) {
      valid[i] = (m.fresh.size() != 4 || m.fresh[i]) && (m.error.size() != 4 || m.error[i] == 0) && std::isfinite(om(i));
    }
    slip_->update(om, mpc_->model().wheel_nominal_speeds(est_vb_), valid, slip_t_ > 0.0 ? t - slip_t_ : -1.0);
    slip_t_ = t;
    slip_rx_ = now;
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
    double dt_state = -1.0;
    if (last_state_stamp_ > 0.0) {
      const double dt = t_state - last_state_stamp_;
      dt_state = dt;
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
    TVec3 vb(m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z);
    est_vb_ = vb;                            // 슬립 계수 η_s 의 기준 속도 (추정기 그대로, 2026-11-23)
    est_rx_ = now;
    if (velocity_source_ == "wheels") {      // 몸체 v_x·v_y 만 바퀴 기구학으로 (ω_z 는 추정기 = 자이로)
      if (wheel_rx_ > 0.0 && now - wheel_rx_ <= wheel_timeout_) {
        // 2026-11-23 구동계 방향 보정: R(δ0 + c·a_c), a_c = |v|·ω_z (ω_z = 추정기 = 자이로). drive 값이 0 이면 그대로
        const TVec3 wc = mpc_->model().correct_wheel_velocity(wheel_v_, vb(2));
        vb(0) = wc(0);
        vb(1) = wc(1);
      } else {
        ++wheel_fallback_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "바퀴 상태 %.0f ms 묵음 — 이 주기는 추정기 속도 (누적 %ld 회)",
          wheel_rx_ > 0.0 ? (now - wheel_rx_) * 1e3 : -1.0, wheel_fallback_);
      }
    }
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
      if (debug_) {publish_status(now, Xe, TVec4::Zero(), 0.0, t_state - ref_->t0(), true, stopped, TVec4::Zero());}
      return;
    }
    if (was_idle) {
      RCLCPP_INFO(
        get_logger(), "hold 해제: 오차 %.1f mm, %.2f° — 추종 재개",
        std::hypot(Xe(0), Xe(1)) * 1e3, Xe(2) * 180.0 / M_PI);
    }

    const auto t_c = std::chrono::steady_clock::now();
    // 기준 가속도 p̈_r,j (관성 좌표): 기준 속도의 중앙 차분, 창 acc_dt (지시서 3.9 — 0.2 s 계단을 사다리꼴로 편다)
    //   2026-11-20: 창 아래끝이 기준열 첫 행보다 앞이면 첫 행으로 민다 (reference.hpp acc_window_start 주석).
    //   예전에는 t − h 가 늘 첫 행 앞(상수 외삽)이라 앞먹임이 필요량의 50~100 %, 평균 75 % 였다.
    acc_ref_.resize(mpc_->tuning().Nc);
    {
      const double Ts = mpc_->tuning().Ts, h = 0.5 * acc_dt_;
      const double t_lo = acc_window_shift_ ? amr_mpc_cpp::acc_window_start(t_state, h, ref_->t0() + ref_->dt()) : t_state - h;
      const auto lo = ref_->horizon(t_lo, Ts, mpc_->tuning().Nc);
      const auto hi = ref_->horizon(t_lo + acc_dt_, Ts, mpc_->tuning().Nc);
      auto inertial = [](const amr_mpc_cpp::TorqueRefPoint & p) {
        const double c = std::cos(p.first(2)), s = std::sin(p.first(2));
        const TVec3 & u = p.second;
        return TVec3(u(0) * c - u(1) * s, u(0) * s + u(1) * c, u(2));
      };
      for (int j = 0; j < mpc_->tuning().Nc; ++j) {acc_ref_[j] = (inertial(hi[j]) - inertial(lo[j])) / acc_dt_;}
    }
    if (slip_on_) {          // 2026-11-23 슬립 계수 η_s: 바퀴 값이 묵으면 깎지 않는다 (η_s 1)
      const bool fresh = slip_rx_ > 0.0 && now - slip_rx_ <= wheel_timeout_;
      mpc_->set_slip_scale(fresh ? slip_->eta() : TVec4::Ones());
      if (!fresh) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "η_s: 바퀴 상태 %.0f ms 묵음 — 이 주기는 η_s 1",
          slip_rx_ > 0.0 ? (now - slip_rx_) * 1e3 : -1.0);
      }
    }
    const TVec4 tau = mpc_->step(x, hz, &acc_ref_, dt_state);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_c).count();
    publish_torque(tau, m.header.stamp);

    const auto & stt = mpc_->stats();
    if (stt.fallback) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "QP 실패(exitflag %d) — 무제약 해를 상한으로 깎았다", stt.exitflag);
    }
    const double ref_age = t_state - ref_->t0();
    const TVec6 & d = mpc_->d_hat();
    if (!csv_path_.empty()) {
      const TVec4 & es = mpc_->slip_scale();
      rows_.push_back(
        {t_state, x(0), x(1), x(2), x(3), x(4), x(5), Xr(0), Xr(1), Xr(2), Xe(0), Xe(1), Xe(2),
          tau(0), tau(1), tau(2), tau(3), mpc_->tau_bar().maxCoeff(), static_cast<double>(stt.active),
          static_cast<double>(stt.iters), static_cast<double>(stt.fallback), ms, ref_age,
          d(0), d(1), d(2), d(3), d(4), d(5), es(0), es(1), es(2), es(3)});
    }
    if (debug_) {
      TVec4 us = TVec4::Zero();                         // 이번 주기 목표 입력 U_s,0 (식-14, 지평 첫 스텝)
      if (mpc_->target_input().size() >= 4) {us = mpc_->target_input().head<4>();}
      publish_status(now, Xe, tau, ms, ref_age, false, stopped, us);
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

  // [t, ex, ey, eθ, τ_FL, τ_FR, τ_RL, τ_RR, τ̄, 활성 제약 수, QP 반복, fallback, step_ms, ref_age, d̂ ×6,      (0~19)
  //  hold, stopped, U_s FL, FR, RL, RR]                                                                        (20~25)
  //  2026-11-16 뒤 6 칸 추가 (앞 20 칸 순서 그대로):
  //    hold    1 = hold 래치로 DAMPING (이 주기는 풀지 않았다, τ·U_s 는 0) / 0 = 추종
  //    stopped 1 = 몸체 속도·회전이 hold_stopped_speed 아래 (hold 진입 조건 중 하나, 추정기 속도 기준)
  //    U_s     이번 주기 목표 입력 U_s,0 (식-14) — 모델이 맞으면 기준을 따라가는 데 드는 토크
  void publish_status(double now, const TVec3 & Xe, const TVec4 & tau, double ms, double ref_age,
    bool hold, bool stopped, const TVec4 & us)
  {
    const auto & stt = mpc_->stats();
    const TVec6 & d = mpc_->d_hat();
    std_msgs::msg::Float64MultiArray m;
    m.data = {now, Xe(0), Xe(1), Xe(2), tau(0), tau(1), tau(2), tau(3), mpc_->tau_bar().maxCoeff(),
      static_cast<double>(stt.active), static_cast<double>(stt.iters), static_cast<double>(stt.fallback), ms, ref_age,
      d(0), d(1), d(2), d(3), d(4), d(5),
      hold ? 1.0 : 0.0, stopped ? 1.0 : 0.0, us(0), us(1), us(2), us(3)};
    pub_status_->publish(m);
    // 2026-11-21 /torque_mpc/adapt = [t, ρ (지금 쓰는 값), η̂ 분산, η 측정 0/1, |g| = c_n/m_eq, 잔차 가속도 ĝ 성분, d̂ 갱신 건너뜀 코드]
    //   — status 26 칸은 그대로 둔다
    std_msgs::msg::Float64MultiArray a;
    a.data = {now, mpc_->ff_normal_rho(), mpc_->ff_normal_P(), static_cast<double>(stt.rho_updated), stt.rho_g, stt.rho_r,
      static_cast<double>(stt.d_skipped)};
    pub_adapt_->publish(a);
    // 2026-11-23 /torque_mpc/load = [t, 스텝 0 의 수직력 N×4 [N], 스텝 0 의 상한 τ̄×4] (하중 이동 끄면 정지 값)
    std_msgs::msg::Float64MultiArray l;
    const TVec4 & nn = mpc_->normal_forces_now();
    const auto & tbs = mpc_->tau_bar_steps();
    l.data = {now, nn(0), nn(1), nn(2), nn(3), tbs(0, 0), tbs(1, 0), tbs(2, 0), tbs(3, 0)};
    pub_load_->publish(l);
    // 2026-11-23 /torque_mpc/slip = [t, 켬 0/1, λ×4, λ̃×4, 계산한 η_s×4, 상한에 곱한 η_s×4] (바퀴 FL FR RL RR).
    //   끄면 "계산한 η_s" 는 켰을 때 곱했을 값, "곱한 η_s" 는 1 — 켬/끔 판을 같은 지표로 견준다
    std_msgs::msg::Float64MultiArray sl;
    const TVec4 & lr = slip_->lambda(), & lf = slip_->lambda_f(), & ec = slip_->eta(), & ea = mpc_->slip_scale();
    sl.data = {now, slip_on_ ? 1.0 : 0.0, lr(0), lr(1), lr(2), lr(3), lf(0), lf(1), lf(2), lf(3),
      ec(0), ec(1), ec(2), ec(3), ea(0), ea(1), ea(2), ea(3)};
    pub_slip_->publish(sl);
  }

  std::unique_ptr<TorqueMPC> mpc_;
  std::optional<HoldLatch> latch_;
  std::optional<ReferenceHorizon> ref_;
  std::string limit_mode_, linearization_, ref_frame_, csv_path_;
  std::string w_delta_src_;   // r_Δ 를 어디서 정했나 (기동 로그)
  std::string cap_on_ = "total";
  std::string velocity_source_ = "estimator";
  double wheel_timeout_ = 0.05, wheel_rx_ = 0.0;
  TVec3 wheel_v_ = TVec3::Zero();
  long wheel_fallback_ = 0;
  rclcpp::Subscription<amr_interfaces::msg::WheelStates>::SharedPtr sub_wheels_;
  // 2026-11-23 슬립 계수 η_s
  std::optional<amr_mpc_cpp::SlipEta> slip_;
  bool slip_on_ = false;
  TVec3 est_vb_ = TVec3::Zero();
  double est_rx_ = 0.0, slip_rx_ = 0.0, slip_t_ = 0.0;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_slip_;
  double last_state_rx_ = 0.0, last_state_stamp_ = 0.0, ref_rx_ = 0.0, armed_since_ = 0.0;
  double state_timeout_ = 0.3, ref_timeout_ = 0.5, hold_stopped_speed_ = 0.04;
  double acc_dt_ = 0.2;
  bool acc_window_shift_ = true;
  std::vector<TVec3> acc_ref_;
  bool debug_ = true, active_ = false, done_ = false, armed_ = false;
  std::vector<std::vector<double>> rows_;

  rclcpp::Publisher<WheelTorqueCommand>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_adapt_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_load_;
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
