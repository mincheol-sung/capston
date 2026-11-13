#!/usr/bin/python3
"""시뮬 플랜트 시연 주행 — 개루프 토크로 정해진 움직임을 하고 멈춘다 (MPC 없음).

토크를 계속 주면 끝없이 가속하므로, 움직임마다 "가속 T초 → 같은 크기로 감속 T초"를 준다.
마찰이 없는 모델이라 정확히 목표 변위에서 선다. 움직임 사이에는 DAMPING 으로 잠깐 세운다.

  시뮬 launch 를 먼저 띄운 뒤:
    /usr/bin/python3 tools/torque_demo_drive.py            # 한 번
    /usr/bin/python3 tools/torque_demo_drive.py --loops 3  # 세 번 반복

토크 분배는 (T⁻¹)ᵀ 의 최소 노름 해(1판 기본 식-35와 같은 꼴)이고, 질량·관성은 torque_plant.yaml 에서 읽는다.
시뮬 플랜트를 payload_kg / use_wheel_inertia 로 바꿔 띄웠으면 같은 인자를 여기에도 줘야 정확히 선다.
"""
import argparse
import math
import time

import numpy as np
import rclpy

from amr_interfaces.msg import WheelTorqueCommand
from amr_mpc.core import config
from amr_mpc.torque.model import TorquePlant

# (이름, 몸체 좌표 변위 [전방 m, 좌측 m, 반시계 rad], 가속 시간 T [s])
MOVES = [
    ("앞으로 1 m", [1.0, 0.0, 0.0], 1.2),
    ("왼쪽으로 게걸음 0.6 m", [0.0, 0.6, 0.0], 1.0),
    ("제자리 반시계 90°", [0.0, 0.0, math.pi / 2], 1.0),
    ("대각선 (앞·오른쪽) 0.8 m", [0.8 / math.sqrt(2), -0.8 / math.sqrt(2), 0.0], 1.2),
    ("제자리 시계 90°", [0.0, 0.0, -math.pi / 2], 1.0),
    ("뒤로 1 m", [-1.0, 0.0, 0.0], 1.2),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--payload_kg", type=float, default=None)
    ap.add_argument("--use_wheel_inertia", action="store_true")
    ap.add_argument("--pause", type=float, default=0.8, help="움직임 사이 DAMPING 시간 [s]")
    args = ap.parse_args()

    override = {"use_wheel_inertia": args.use_wheel_inertia}
    if args.payload_kg is not None:
        override["mass"] = float(config.section("body", "torque_plant.yaml")["mass_kg"]) + args.payload_kg
    p = TorquePlant.from_yaml(**override)
    force_pinv = np.linalg.pinv(p.Tinv.T)       # 몸체 [F_x, F_y, M_z] → 바퀴 토크 (최소 노름)

    rclpy.init()
    node = rclpy.create_node("torque_demo_drive")
    pub = node.create_publisher(WheelTorqueCommand, "/wheel_torque_cmd", 10)

    def send(mode, tau, sec):
        msg = WheelTorqueCommand()
        msg.mode = mode
        msg.torque = [float(x) for x in tau]
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            time.sleep(0.01)

    time.sleep(0.5)   # 구독 연결 대기
    print(f"m = {p.mass:.2f} kg, I_z = {p.inertia_z:.3f} kg·m², 바퀴관성 {'켬' if p.use_wheel_inertia else '끔'}")
    try:
        for lap in range(args.loops):
            for name, disp, T in MOVES:
                a_body = np.array(disp) / T ** 2            # 가속 T + 감속 T 로 변위 disp
                tau = force_pinv @ (p.M_eff @ a_body)
                peak_v = np.abs(np.array(disp)) / T
                print(f"[{lap + 1}] {name:<22} τ = {np.round(tau, 4)} N·m   최고속도 {np.round(peak_v, 2)}")
                send(WheelTorqueCommand.MODE_TORQUE, tau, T)
                send(WheelTorqueCommand.MODE_TORQUE, -tau, T)
                send(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], args.pause)
    except KeyboardInterrupt:
        pass
    finally:
        send(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.3)
        node.destroy_node()
        rclpy.shutdown()
    print("끝")


if __name__ == "__main__":
    main()
