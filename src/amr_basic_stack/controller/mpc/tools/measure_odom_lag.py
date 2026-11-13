#!/usr/bin/env python3
# /cmd_vel 이 각 odom 토픽의 실측 속도에 나타나기까지 걸리는 지연을 잰다.
# ---------------------------------------------------------------
# 왜 필요한가: mpc.py 의 예측모델은 model.py 의 FOPDT(τ, K, θ=40 ms) 를 쓰고, 그 θ 는
#   sys_id/wp2_step_logger.py 가 /odom 으로 식별한 값이다 (d = θ/Ts = 1, 정확히 한 스텝).
#   /odometry/filtered 는 EKF 를 한 단 더 끼우므로 θ 가 커진다. 그 차이가 모델에 없으면
#   MPC 가 과보정 → 포화 → QP 반복 상한 → 발산한다.
#
# 쓰는 법 (로봇이 움직이는 동안. 텔레옵이든 8자 주행이든):
#   ros2 run amr_mpc ... 대신 직접:  python3 tools/measure_odom_lag.py
#   Ctrl+C 로 종료하면 토픽별 지연을 낸다.
#
# 판정:  /odom 이 40~60 ms 근처여야 정상 (θ=40 ms + 샘플링).
#        /odometry/filtered 가 그보다 80 ms 이상 크면 MPC 를 거기에 물리면 안 된다.
# ---------------------------------------------------------------
import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

TOPICS = ["/odom", "/odometry/filtered"]
GRID = 0.02          # 재표본 간격 [s]
MAX_LAG = 0.40       # 훑어볼 최대 지연 [s]


class LagMeter(Node):
    def __init__(self):
        super().__init__("measure_odom_lag")
        self.cmd = []                                   # (t, vx, vy, wz)
        self.odo = {t: [] for t in TOPICS}
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd, 50)
        for t in TOPICS:
            self.create_subscription(Odometry, t, lambda m, k=t: self.cb_odom(m, k), 50)
        self.get_logger().info(f"기록 중 — 로봇을 움직여라 (10 s 이상). 토픽 {', '.join(TOPICS)}. Ctrl+C 로 결과.")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_cmd(self, m):
        self.cmd.append((self.now(), m.linear.x, m.linear.y, m.angular.z))

    def cb_odom(self, m, key):
        t = m.twist.twist
        self.odo[key].append((self.now(), t.linear.x, t.linear.y, t.angular.z))

    def report(self):
        if len(self.cmd) < 50:
            print(f"\n[ERROR] /cmd_vel 표본 {len(self.cmd)} 개 — 로봇이 움직이는 동안 다시 재라."); return
        C = np.array(self.cmd)
        t0, t1 = C[0, 0], C[-1, 0]
        g = np.arange(t0, t1, GRID)
        print(f"\n기록 {t1-t0:.1f} s,  /cmd_vel {len(C)} 표본\n")
        print(f"{'토픽':>22} {'표본':>6} {'축':>4} {'지연':>7} {'상관':>6} {'이득':>6}")
        for key in TOPICS:
            if len(self.odo[key]) < 50:
                print(f"{key:>22} {len(self.odo[key]):6d}   — 표본 부족 (토픽이 없거나 안 뜸)"); continue
            O = np.array(self.odo[key])
            for ax, name in ((1, "vx"), (2, "vy"), (3, "wz")):
                c = np.interp(g, C[:, 0], C[:, ax])
                if np.std(c) < 1e-3:
                    continue                                     # 이 축은 명령이 없었다
                best = (0.0, -2.0)
                for lag in np.arange(0.0, MAX_LAG, GRID):
                    o = np.interp(g + lag, O[:, 0], O[:, ax])
                    r = np.corrcoef(c, o)[0, 1]
                    if r > best[1]:
                        best = (lag, r)
                o = np.interp(g + best[0], O[:, 0], O[:, ax])
                gain = float(np.polyfit(c, o, 1)[0])
                print(f"{key:>22} {len(O):6d} {name:>4} {best[0]*1000:5.0f}ms {best[1]:6.3f} {gain:6.2f}")
        print("\n판정: model.py 의 θ=40 ms 는 /odom 기준이다.")
        print("      MPC 의 odom_topic 은 지연이 가장 작은 토픽이어야 한다.")
        print("      /odometry/filtered 가 /odom 보다 80 ms 이상 느리면 MPC 를 거기 물리지 말 것.")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LagMeter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.report()
        try: node.destroy_node()
        except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
