#!/usr/bin/env python3
"""바닥 토크 램프 시험 — 실차 한 터미널 (2026-11-17). amrall use_mpc:=true 와 같은 스택에서 MPC 만 램프로 바꾼다.

  real_all.launch.py use_mpc:=true controller:=ramp  (센서 · 측위 · CAN · 추정기 · torque_output · 게이트 · fault_monitor · bag)
    torque_ramp_node: 바퀴–바닥 토크 τ_g 를 0 → ramp_tau_max 까지 올리고, 바퀴 관성 feedforward 를 더해 /torque_mpc/cmd_raw 로.
    마찰 보상(coulomb·viscous·static)은 torque_output 이 amrall 과 똑같이 더한다.
    기울기는 mu_hat 과 ramp_distance(기본 3 m)로 자동 — 로그 첫 줄 "torque_ramp:" 에 값이 나온다.

사용 (NUC):  amrramp dir:=fwd mu_hat:=0.6 bag:=true          직진
             amrramp dir:=left mu_hat:=0.6 bag:=ramp_left     왼쪽 90° (게걸음)
             dir:=back · dir:=right (2026-11-20) — fwd↔back, left↔right 를 번갈아 돌면 되돌려 놓지 않아도 출발점 근처를 오간다
  준비 로그 뒤 'a' = 한 판. 끝나면 스스로 무장 해제 → 로봇을 되돌려 놓고 다시 'a'. 's'/스페이스 = 즉시 정지.
  판마다 CSV: analysis/torque_mpc_real/<날짜>_ramp/ramp_<dir>_<시분초>_run<n>.csv  (ramp_log_dir:= 로 바꿈)
  bag: bag:=true → bags/<날짜>/<시분초>, bag:=<이름> → bags/<날짜>/<시분초>_<이름> (real_all ⑥, record_all.sh)
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_ARGS = {  # 이 launch 인자 → real_all 인자, 기본값
    "dir": ("ramp_dir", "fwd"), "mu_hat": ("mu_hat", "0.6"), "tau_max": ("ramp_tau_max", "1.0"),
    "distance": ("ramp_distance", "3.0"), "rate": ("ramp_rate", ""), "stop_distance": ("ramp_stop_distance", ""),
    "inertia_ff": ("ramp_inertia_ff", "true"), "log_dir": ("ramp_log_dir", ""), "bag": ("bag", ""),
    "stiction_comp": ("stiction_comp", ""),
}


def _include(context):
    share = get_package_share_directory("amr_mpc")
    ws = os.path.realpath(os.path.join(share, "..", "..", "..", ".."))
    args = {"use_mpc": "true", "controller": "ramp"}
    for k, (dst, _) in _ARGS.items():
        v = LaunchConfiguration(k).perform(context).strip()
        if v:
            args[dst] = v
    if args.get("ramp_dir") not in ("fwd", "back", "left", "right"):          # 2026-11-20 back·right 추가
        raise RuntimeError(f"dir:={args.get('ramp_dir')!r} — fwd | back | left | right")
    args.setdefault("ramp_log_dir",
                    os.path.join(ws, "analysis", "torque_mpc_real", time.strftime("%Y-%m-%d") + "_ramp"))
    src = PythonLaunchDescriptionSource(os.path.join(share, "launch", "real_all.launch.py"))
    return [IncludeLaunchDescription(src, launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=d) for k, (_, d) in _ARGS.items()],
        OpaqueFunction(function=_include),
    ])
