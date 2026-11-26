#!/usr/bin/env python3
"""바닥 토크 스텝 시험 — 실차 한 터미널 (2026-11-17). torque_ramp.launch.py 와 같은 스택, 램프 대신 일정 토크.

  real_all.launch.py use_mpc:=true controller:=step  (센서 · 측위 · CAN · 추정기 · torque_output · 게이트 · fault_monitor · bag)
    torque_step_node: 'a' 누른 순간부터 바퀴–바닥 토크 τ_g = tau 를 일정하게 (+ 바퀴 관성 ff, torque_output 마찰 보상).
    출발점에서 distance 만큼 가면 DAMPING + 무장 해제.

사용 (NUC):  amrstep dir:=fwd  tau:=0.3 distance:=2.0 bag:=step_fwd
             amrstep dir:=left tau:=0.3 distance:=2.0 bag:=step_left
             amrstep dir:=ccw  tau:=0.1 angle:=90 bag:=step_ccw      (2026-11-25 제자리 회전: 회전각으로 끊음, distance = 벗어남 한도)
  tau·distance 는 꼭 준다. 'a' = 한 판, 끝나면 로봇을 되돌려 놓고 다시 'a'. 's'/스페이스 = 즉시 정지.
  판마다 CSV: analysis/torque_mpc_real/<날짜>_step/step_<dir>_<tau>Nm_<시분초>_run<n>.csv
  bag: bag:=<이름> → bags/<날짜>/<시분초>_<이름>
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_ARGS = {  # 이 launch 인자 → real_all 인자, 기본값
    "dir": ("step_dir", "fwd"), "tau": ("step_tau", ""), "distance": ("step_distance", ""), "angle": ("step_angle_deg", ""),
    "max_time": ("step_max_time", "5.0"), "inertia_ff": ("step_inertia_ff", "true"), "log_dir": ("step_log_dir", ""),
    "bag": ("bag", ""), "stiction_comp": ("stiction_comp", ""),
    # 2026-11-24: 주행 launch 의 마찰 보상 기본이 끔으로 바뀌었다. 이 시험은 "바닥 토크 τ_g 를 준다" 가 정의라 보상을 켠 채로 둔다 (끄려면 friction_comp:=false)
    "friction_comp": ("friction_comp", "true"),
}


def _include(context):
    share = get_package_share_directory("amr_mpc")
    ws = os.path.realpath(os.path.join(share, "..", "..", "..", ".."))
    args = {"use_mpc": "true", "controller": "step"}
    for k, (dst, _) in _ARGS.items():
        v = LaunchConfiguration(k).perform(context).strip()
        if v:
            args[dst] = v
    if args.get("step_dir") not in ("fwd", "back", "left", "right", "ccw", "cw"):
        raise RuntimeError(f"dir:={args.get('step_dir')!r} — fwd, back, left, right, ccw, cw")
    if args["step_dir"] in ("ccw", "cw"):
        if "step_tau" not in args or "step_angle_deg" not in args:
            raise RuntimeError("회전은 tau:=<N·m> 와 angle:=<°> 를 꼭 준다. 예) amrstep dir:=ccw tau:=0.1 angle:=90")
    elif "step_tau" not in args or "step_distance" not in args:
        raise RuntimeError("tau:=<N·m> 와 distance:=<m> 를 꼭 준다. 예) amrstep dir:=fwd tau:=0.3 distance:=2.0")
    args.setdefault("step_log_dir",
                    os.path.join(ws, "analysis", "torque_mpc_real", time.strftime("%Y-%m-%d") + "_step"))
    src = PythonLaunchDescriptionSource(os.path.join(share, "launch", "real_all.launch.py"))
    return [IncludeLaunchDescription(src, launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=d) for k, (_, d) in _ARGS.items()],
        OpaqueFunction(function=_include),
    ])
