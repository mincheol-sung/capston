#!/usr/bin/env python3
"""실차를 터미널 하나로 — 센서 · 측위 · CAN+추정기(+MPC) · 표시 노드 · 측위 맞추기 (2026-11-16).

예전에는 터미널 다섯 개(amrcan·amrsens·amrloc·amrbase/amrmpc·amrviz)로 띄웠다. 이 launch 는 그 launch 들을 그대로 include 한다:
  ① amr_bringup/sensors.launch.py                     라이다 2 개 + D455 IMU
  ② amr_perception_bringup/cartographer.launch.py      mode:=localization map:=<map> lua_basename:=<lua>
  ③ amr_mpc/torque_experiment.launch.py plant:=real    ak40_can_node(CAN 자동 준비) · IMU 회전 · robot_state_publisher · 추정기
                                                       use_mpc:=true 면 기준·MPC·게이트도 — 이 터미널에서 a = 무장, s/스페이스 = 정지
  ④ amr_mpc/torque_viz.launch.py rviz:=false           map → base_footprint TF, 지나간 경로 (RViz 는 랩 PC 에서)
  (origin_mode 기본 robot — 무장한 자리가 출발점)
  ⑤ (auto_localize) localize_delay 초 뒤 tools/check_localization.py --apply 한 번 — "맞음" 이 로그에 나와야 한다
  ⑥ (bag:=true 또는 bag:=<이름>, 폴더는 bags/<날짜>/<시분초>[_이름] 자동) tools/record_all.sh — 모든 토픽 녹화 (영상·점군 제외) + run_info/ (파라미터·git 상태·launch 인자)

사용 (NUC, SSH 터미널 하나): amr_aliases.sh 의 amrall 이 환경(도메인 81, AMR_PEERS → ROS_STATIC_PEERS)과 sudo -v 를 맞춘다
  export AMR_PEERS=192.168.0.209     # 랩 PC 로 볼 때만
  amrall                                                     # MPC 없이 (basic_motion_test 는 다른 터미널에서 amrmove)
  amrall use_mpc:=true shape:=line seg_distance:=0.4 seg_dir_deg:=45 speed:=0.10 limit_mode:=friction mu_hat:=0.55
  amrall use_mpc:=true controller:=pure_pursuit shape:=circles8 width:=2.0 yaw_mode:=tangent speed:=0.3   # 비교군 A
    (퓨어 퍼슛 + 바퀴별 PI, 게인 config/torque_baseline_a.yaml. v_y = 0 이라 yaw_mode:=tangent 로 — fixed(게걸음)는 못 따라간다.
     limit_mode·disturbance·mu_hat·use_w 같은 MPC 인자는 A 에서 안 쓴다. 2026-11-16 전에는 controller 가 안 넘어가 늘 MPC 가 떴다)
※ 카메라 노드(realsense2_camera_node)가 죽으면(2026-11-15 한 번) IMU 가 없어 측위가 위치를 안 낸다 → Ctrl+C 하고 다시.
※ use_mpc:=true 이면 게이트가 무장 전에도 DAMPING 을 50 Hz 로 낸다 → basic_motion_test(amrmove)와 같이 쓰지 말 것.
※ torque_output 의 정지 마찰 보상이 기본으로 켜진다 (plant:=real, 2026-11-16). 보상 없이 비교하려면 stiction_comp:=false
※ MPC 모델에는 바퀴 등가 질량 m_w = 4J_w/r² (≈ 2.2 kg) 이 늘 들어간다 (1판 식-3·5, w 항·외란 추정치 지시서 2026-11-16).
  w 항(식-6)은 use_w:=true, 외란 추정치는 disturbance:=true 로 켠다 — 둘 다 기본 끔. yaw_mode:=fixed 면 θ̇ = 0 이라 w = 0.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# torque_experiment.launch.py 로 그대로 넘기는 인자 (비우면 넘기지 않는다 → 그쪽 기본값·YAML)
_PASS = ["use_mpc", "controller", "wheel_odom", "log_prefix", "limit_mode", "mu_hat", "mu_hat_wheels", "wheel_load_share", "disturbance", "stiction_comp", "use_w",
         "origin_mode", "shape", "crossing_blend_m", "load_budget", "spin_rate",
         "width", "height", "yaw_mode", "align_s",
         "seg_distance", "seg_dir_deg", "seg_angle_deg", "seg_yaw_rate", "seg_out_and_back", "seg_pause_s", "speed",
         "square_side", "square_corner_speed", "square_v_max", "square_yaw_blend",
         "accel", "laps", "hold_s", "max_track_error",
         "ramp_dir", "ramp_tau_max", "ramp_rate", "ramp_distance", "ramp_stop_distance", "ramp_inertia_ff", "ramp_log_dir",
         "step_dir", "step_tau", "step_distance", "step_max_time", "step_inertia_ff", "step_log_dir", "mpc_W", "mpc_w_delta", "mpc_w_dudt", "mpc_r_u", "mpc_q_null", "mpc_dist_alpha", "mpc_dist_accel_only", "velocity_source", "est_pose_period", "pp_speed_source", "pp_kp", "pp_ki",
         "est_mode", "trip_on_pose_jump"]


# sensors.launch.py 로 넘기는 인자 (이름 바꿔서). 비우면 넘기지 않는다
_SENSOR_PASS = {"imu_accel_fps": "depth_camera_accel_fps", "imu_gyro_fps": "depth_camera_gyro_fps",
                "imu_unite": "depth_camera_unite_imu_method"}


def _truthy(v):
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def _inc(pkg, name, args=None):
    # scoped: include 한 launch 의 인자가 밖으로 새지 않게 (2026-11-15).
    # forwarding=False: 이 launch 의 빈 인자(max_track_error 등 "")가 안으로 들어가 그쪽 기본값(1.5)을 덮지 않게 —
    #   2026-11-16 첫 실행 "could not convert string to float: ''". 넘길 값은 launch_arguments 로만 준다.
    src = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), "launch", name))
    return GroupAction([IncludeLaunchDescription(src, launch_arguments=(args or {}).items())], scoped=True, forwarding=False)


def _nodes(context):
    g = lambda n: LaunchConfiguration(n).perform(context).strip()
    out = [LogInfo(msg=f"[real_all] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(없음)')}  "
                       f"peers={os.environ.get('ROS_STATIC_PEERS', '(없음, NUC 만)')}")]
    # ⑥ bag:=true | <이름> — 모든 토픽 녹화 (tools/record_all.sh: 영상·점군만 뺌, 20 s 뒤 파라미터·git 상태를 run_info/ 에). 2026-11-16
    #    폴더 이름은 record_all.sh 가 정한다: true → bags/<날짜>/<시분초>, 이름 → bags/<날짜>/<시분초>_<이름>,
    #    / 가 있으면 bags/<그대로>. false·빈 값이면 녹화 안 함. Ctrl+C 로 같이 닫힌다.
    if g("bag") and g("bag").lower() not in ("false", "0", "no", "off"):
        ws = os.path.realpath(os.path.join(get_package_share_directory("amr_mpc"), "..", "..", "..", ".."))
        rec = os.path.join(ws, "src", "amr_basic_stack", "controller", "mpc", "tools", "record_all.sh")
        info = "real_all.launch.py " + " ".join(f"{k}:={v}" for k, v in sorted(context.launch_configurations.items()) if v)
        if os.path.isfile(rec):
            out.append(ExecuteProcess(cmd=["bash", rec, g("bag"), info], output="screen", name="bag_record"))
        else:
            out.append(LogInfo(msg=f"[real_all] {rec} 없음 — 녹화하지 않는다"))
    if _truthy(g("sensors")):
        # D455 IMU 설정 (2026-11-19, 가속도계 지연 시험): 비우면 sensors.launch.py 기본값
        sens = {dst: g(src) for src, dst in _SENSOR_PASS.items() if g(src)}
        out.append(_inc("amr_bringup", "sensors.launch.py", sens))
    if _truthy(g("localization")):
        out.append(_inc("amr_perception_bringup", "cartographer.launch.py",
                        {"mode": "localization", "map": g("map"), "lua_basename": g("lua")}))
    torque = {"plant": "real", "use_rviz": "false"}
    for k in _PASS:
        if g(k):
            torque[k] = g(k)
    torque.setdefault("use_mpc", "false")
    # 2026-11-19: 스캔 하나씩 매칭하는 lua (localization_torque_acc1_2d.lua) 면 포즈가 20 Hz — 추정기 게인 설계 주기도 0.05 s
    if "acc1" in g("lua") and "est_pose_period" not in torque:
        torque["est_pose_period"] = "0.05"
        out.append(LogInfo(msg=f"[real_all] lua {g('lua')} → 추정기 est_pose_period 0.05 s (포즈 20 Hz)"))
    # 실차 5-3·8자는 무장한 자리가 출발점 (amr8 과 같다). torque_track.yaml 기본 fixed 는 지도 (0, 0) 이라 0.30 m 밖이면
    # "출발 거부" — 2026-11-16 첫 실행에서 a 를 눌러도 안 움직였다. 지도 고정으로 쓰려면 origin_mode:=fixed
    torque.setdefault("origin_mode", "robot")
    out.append(_inc("amr_mpc", "torque_experiment.launch.py", torque))
    if _truthy(g("viz")):
        out.append(_inc("amr_mpc", "torque_viz.launch.py", {"rviz": "false"}))
    if _truthy(g("auto_localize")):
        # 설치본 share/amr_mpc → 작업공간 → 소스의 tools (check_localization.py 는 설치하지 않는다)
        ws = os.path.realpath(os.path.join(get_package_share_directory("amr_mpc"), "..", "..", "..", ".."))
        tool = os.path.join(ws, "src", "amr_basic_stack", "controller", "mpc", "tools", "check_localization.py")
        if os.path.isfile(tool):
            out.append(TimerAction(period=float(g("localize_delay")), actions=[
                LogInfo(msg="[real_all] 측위 맞추기 (check_localization.py --apply)"),
                ExecuteProcess(cmd=["/usr/bin/python3", tool, "--apply"], output="screen", name="check_localization")]))
        else:
            out.append(LogInfo(msg=f"[real_all] {tool} 없음 — 측위는 다른 터미널에서 amrcheck"))
    return out


def generate_launch_description():
    A = DeclareLaunchArgument
    return LaunchDescription([
        A("sensors", default_value="true", description="① 센서 같이 띄우기"),
        A("localization", default_value="true", description="② 측위 같이 띄우기"),
        A("map", default_value="smoc_lab"),
        A("lua", default_value="localization_torque_2d.lua"),
        A("viz", default_value="true", description="④ 표시 노드 (RViz 는 띄우지 않는다)"),
        A("auto_localize", default_value="true", description="⑤ 시작 뒤 check_localization.py --apply 한 번"),
        A("localize_delay", default_value="15.0", description="⑤ 까지 기다리는 시간 [s] (지도 불러오기·IMU 준비)"),
        A("bag", default_value="", description="⑥ 모든 토픽 녹화. true → bags/<날짜>/<시분초>, 이름 → bags/<날짜>/<시분초>_<이름> (비우면 녹화 안 함)"),
        *[A(k, default_value="", description="torque_experiment.launch.py 로 넘긴다 (비우면 그쪽 기본값)") for k in _PASS],
        A("imu_accel_fps", default_value="", description="① D455 가속도계 Hz (100|200|400, 비우면 장치 기본)"),
        A("imu_gyro_fps", default_value="", description="① D455 자이로 Hz (200|400, 비우면 장치 기본)"),
        A("imu_unite", default_value="", description="① gyro+accel 결합 0|1 copy|2 linear_interpolation (비우면 2)"),
        OpaqueFunction(function=_nodes),
    ])
