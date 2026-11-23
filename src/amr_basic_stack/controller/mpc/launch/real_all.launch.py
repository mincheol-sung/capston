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
  amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 yaw_mode:=tangent speed:=0.3   # 비교군 A, 룩어헤드 고정 0.30 m
  amrall use_mpc:=true controller:=adpp ...  # 어댑티브 PP: 같은 PP, 룩어헤드만 속도 비례 clamp(0.5 s·|v_ref|, 0.15, 0.60) m (2026-11-23)
    (퓨어 퍼슛 + 바퀴별 PI, 게인 config/torque_baseline_a.yaml. 예전 이름 controller:=pure_pursuit = pp.
     v_y = 0 이라 yaw_mode:=tangent 로 — fixed(게걸음)는 못 따라간다.
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
                            RegisterEventHandler,
                            OpaqueFunction, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# torque_experiment.launch.py 로 그대로 넘기는 인자 (비우면 넘기지 않는다 → 그쪽 기본값·YAML)
_PASS = ["use_mpc", "controller", "wheel_odom", "log_prefix", "limit_mode", "mu_hat", "mu_hat_wheels", "wheel_load_share", "disturbance", "stiction_comp", "friction_comp", "use_w",
         "origin_mode", "shape", "crossing_blend_m", "load_budget", "spin_rate", "speed_mod_amp", "speed_mod_cycles", "speed_mod_phase_deg", "speed_mod_shape", "lap_speeds",
         "width", "height", "yaw_mode", "align_s",
         "seg_distance", "seg_dir_deg", "seg_angle_deg", "seg_yaw_rate", "seg_out_and_back", "seg_pause_s", "speed",
         "square_side", "square_corner_speed", "square_v_max", "square_yaw_blend", "square_corner_mode", "square_rot_s", "square_dwell_s",
         "accel", "max_ff_ratio", "laps", "hold_s", "max_track_error",
         "ramp_dir", "ramp_tau_max", "ramp_rate", "ramp_distance", "ramp_stop_distance", "ramp_inertia_ff", "ramp_log_dir",
         "step_dir", "step_tau", "step_distance", "step_max_time", "step_inertia_ff", "step_log_dir", "mpc_W", "mpc_com_height", "mpc_load_weighted_alloc", "mpc_load_dynamic_cap", "mpc_w_delta", "mpc_w_dudt", "mpc_r_u", "mpc_Nc", "mpc_Np", "mpc_q_null", "mpc_dist_alpha", "mpc_dist_accel_only", "mpc_dist_frame", "mpc_dist_freeze", "mpc_dist_acc_max", "mpc_dist_yaw_acc_max", "mpc_dist_observer", "mpc_target_projection", "mpc_cap_on", "mpc_ff_normal_scale", "mpc_ff_normal_estimate", "mpc_ff_normal_q", "velocity_source", "est_pose_period", "pp_speed_source", "pp_kp", "pp_ki",
         "pp_lookahead_min", "pp_lookahead_max", "est_mode", "trip_on_pose_jump", "slip_guard", "est_gains", "est_noise",
         "drive_yaw_deg", "drive_slip_coef", "max_com_height", "path_follow_lead"]                      # 2026-11-23 구동계 방향 보정 (torque_plant.yaml drive)


# sensors.launch.py 로 넘기는 인자 (이름 바꿔서). 비우면 넘기지 않는다
_SENSOR_PASS = {"imu_accel_fps": "depth_camera_accel_fps", "imu_gyro_fps": "depth_camera_gyro_fps",
                "imu_unite": "depth_camera_unite_imu_method"}


def _leftover_nodes():
    """앞서 비정상 종료한 launch 가 남긴 고아 노드 (2026-11-23): 부모가 PID 1 (또는 systemd 하위 수거자) 이고, 이 작업공간 install/ 이나
    /opt/ros/ 의 실행 파일이며, ROS_DOMAIN_ID 가 지금과 같은 프로세스. [(pid, 명령줄)] — ros2 daemon 은 뺀다."""
    dom = os.environ.get("ROS_DOMAIN_ID", "0")
    install = os.path.realpath(os.path.join(get_package_share_directory("amr_mpc"), "..", "..", ".."))
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            if ppid != 1:
                with open(f"/proc/{ppid}/comm") as f:
                    if f.read().strip() not in ("systemd", "init"):
                        continue
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            if not (install in cmd or "/opt/ros/" in cmd) or "_ros2_daemon" in cmd or "ros2 launch" in cmd:
                continue
            with open(f"/proc/{pid}/environ", "rb") as f:
                env = dict(e.split(b"=", 1) for e in f.read().split(b"\0") if b"=" in e)
            if env.get(b"ROS_DOMAIN_ID", b"0").decode() != dom:
                continue
            found.append((int(pid), cmd))
        except (OSError, ValueError, IndexError):
            continue
    return sorted(found)


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
    # ⓪ 고아 노드 검사 (2026-11-23): 남아 있으면 스캔·초기 포즈·오도메트리가 두 번씩 나가 측위가 망가진다 (11-23 11:39 판).
    #    아무것도 띄우기 전에 멈춘다 — 여기서 예외가 나면 남는 프로세스가 없다.
    left = _leftover_nodes()
    if left:
        lines = "\n".join(f"    {pid}  {cmd[:110]}" for pid, cmd in left)
        raise RuntimeError(f"[real_all] 앞서 비정상 종료한 launch 의 노드 {len(left)} 개가 남아 있다 (부모 PID 1, ROS_DOMAIN_ID "
                           f"{os.environ.get('ROS_DOMAIN_ID', '0')}). 이대로 띄우면 스캔·포즈가 두 번씩 나간다.\n{lines}\n"
                           f"  정리한 뒤 다시: kill -INT {' '.join(str(p) for p, _ in left)}")
    out = [LogInfo(msg=f"[real_all] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(없음)')}  "
                       f"peers={os.environ.get('ROS_STATIC_PEERS', '(없음, NUC 만)')}")]
    # ① torque_experiment 를 **맨 앞에** 둔다 (2026-11-23). 그 안의 OpaqueFunction 이 CAN 준비 실패·인자 검사에서 예외를 던지면
    #    launch 는 "Caught exception" 으로 끝나는데, 그 전에 방문한 프로세스 (센서·cartographer 묶음·bag) 는 종료 신호 없이
    #    PPID 1 고아로 남는다 — 11-23 11:36 CAN 실패 두 번이 스캔 필터·병합기·pose_to_odom·초기 포즈 노드 12 개를 남겨, 다음 판에서
    #    스캔이 두 번씩 들어가 cartographer "Dropped ~1725 earlier points" 35 회/s · SLAM 포즈 3.6 m 오감. 먼저 방문하면 예외가 나도
    #    아직 아무 프로세스도 안 떴다. (bag 은 수 ms 늦게 시작 — 녹화에는 영향 없음)
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
    if _truthy(g("viz")):
        out.append(_inc("amr_mpc", "torque_viz.launch.py", {"rviz": "false"}))
    if _truthy(g("auto_localize")):
        # 설치본 share/amr_mpc → 작업공간 → 소스의 tools (check_localization.py 는 설치하지 않는다)
        ws = os.path.realpath(os.path.join(get_package_share_directory("amr_mpc"), "..", "..", "..", ".."))
        tool = os.path.join(ws, "src", "amr_basic_stack", "controller", "mpc", "tools", "check_localization.py")
        if os.path.isfile(tool):
            check = ExecuteProcess(cmd=["/usr/bin/python3", tool, "--apply"], output="screen", name="check_localization")
            out.append(TimerAction(period=float(g("localize_delay")), actions=[
                LogInfo(msg="[real_all] 측위 맞추기 (check_localization.py --apply) — 끝나면 큰 배너가 나온다. 그 전에 a 누르지 말 것"),
                check]))
            # 2026-11-20: 끝났다는 줄이 Cartographer 로그에 묻혀 그 전에 무장했다가 측위 보정(1.6 m 튐)에 fault_monitor 가 섰다
            #   → 종료 코드 (check_localization.py: 0 맞음, 1 측위 결과 없음, 2 찾았지만 안 넣음, 3 지도 어디에도 안 맞음, 4 넣었는데 아직 안 맞음,
            #     5 넣었는데 새 trajectory 가 안 생김 — 2026-11-23 부터 --apply 는 보정 (없으면 최대 30 s) 뒤에 끝난다)
            #   로 눈에 띄는 배너를 찍는다
            out.append(RegisterEventHandler(OnProcessExit(target_action=check, on_exit=_localize_banner)))
        else:
            out.append(LogInfo(msg=f"[real_all] {tool} 없음 — 측위는 다른 터미널에서 amrcheck"))
    return out


def _localize_banner(event, context):
    """check_localization 종료 코드 → 배너 (ANSI 색: 맞음 초록, 그 밖 빨강)."""
    code = event.returncode
    why = {1: "측위 결과(/tracked_pose·스캔)를 못 받음", 2: "위치를 찾았지만 넣지 않음", 3: "지도 어디에도 안 맞음 — 위치를 손으로",
           4: "/initialpose 를 넣었는데 아직 안 맞음",
           5: "/initialpose 뒤 새 trajectory 가 안 생김 — 측위 맞추기가 안 먹었다"}.get(code, f"종료 코드 {code}")
    if code == 0:
        line, color = "측위 맞추기 끝: 맞음 — 이제 'a' 로 무장해도 된다", "\033[1;30;42m"
    else:
        line, color = f"측위 맞추기 실패: {why} — 무장하지 말 것 (위 check_localization 출력 확인)", "\033[1;37;41m"
    bar = "█" * 72
    return [LogInfo(msg=f"\n{color}{bar}\033[0m\n{color}██  {line}\033[0m\n{color}{bar}\033[0m")]


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
