#!/usr/bin/env python3
# /odometry/filtered 의 잡음을 '모양'까지 잰다.  WP4 MPC 의 Δx_m 이 무엇을 보는지 확인용.
# ---------------------------------------------------------------
# Wang Ch.1.2 증강모델은 x = [Δx_m ; y] 를 쓴다. Δx_m 은 차분이라 포즈 추정의
# 상수 bias 는 0 이 되고 (경로가 밀릴 뿐 떨리지 않는다), 틱간 무상관 성분만
# σ√2/Ts 의 가짜 속도로 증폭된다. 그래서 σ 하나로는 부족하고 스펙트럼이 필요하다.
#
# 폐루프 시뮬 기준 (MPPI 10 Hz → MPC 25 Hz, 직선+R1.5m 선회):
#     상수 bias 5 cm      → 명령의 2 Hz 이상 전력  0.8 %   (무교란과 같음)
#     백색 jitter 1 mm    →                      12.7 %
#     백색 jitter 3 mm    →                      53.1 %
#     랜덤워크 3 mm/tick  →                      42.6 %
# 즉 판단 기준은 σ 가 아니라 아래 '차분 σ' 와 '2 Hz 이상 비율' 이다.
#
# 사용: ROS2 소싱 후, 로봇을 완전히 정지시킨 뒤 (cmd_vel 0, 모터 정지)
#       python3 measure_odom_noise.py                  # /odometry/filtered 30 s
#       python3 measure_odom_noise.py --sec 60 --topic /odom      # 원천 비교용
#
# /odom 과 /odometry/filtered 를 둘 다 재면 EKF 가 실제로 눌러주는 양을 알 수 있다.
# ---------------------------------------------------------------
import argparse, math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

TS_MPC = 0.04                      # mpc.yaml 의 Ts. 가짜 속도 환산에 쓴다


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomNoise(Node):
    def __init__(self, topic, sec):
        super().__init__("measure_odom_noise")
        self.sec, self.rows, self.t0 = sec, [], None
        self.create_subscription(Odometry, topic, self.cb, 50)
        self.get_logger().info(f"{topic} 기록 {sec:.0f} s — 로봇이 완전히 정지해 있어야 합니다")

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        p, w = m.pose.pose, m.twist.twist
        self.rows.append([t - self.t0, p.position.x, p.position.y, yaw_of(p.orientation),
                          w.linear.x, w.linear.y, w.angular.z])
        if t - self.t0 >= self.sec:
            raise SystemExit(0)


def report(R):
    t = R[:, 0]
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    print(f"\n표본 {len(R)} 개, 평균 주기 {dt*1000:.1f} ms ({fs:.1f} Hz), 길이 {t[-1]:.1f} s\n")
    names = ["x [mm]", "y [mm]", "yaw [mdeg]", "vx [mm/s]", "vy [mm/s]", "wz [mdeg/s]"]
    scale = [1e3, 1e3, 1e3 * 180 / math.pi, 1e3, 1e3, 1e3 * 180 / math.pi]
    print(f"{'채널':>12} | {'σ (원신호)':>12} | {'σ (차분)':>12} | {'가짜속도 σd/Ts':>16} | {'차분 2Hz↑':>9}")
    print("-" * 76)
    for i in range(6):
        x = R[:, i + 1] * scale[i]
        x = x - np.polyval(np.polyfit(t, x, 1), t)      # 선형 추세(드리프트) 제거 후 잔차
        d = np.diff(x)                                  # ★ MPC 의 Δx_m 이 실제로 보는 신호
        X = np.abs(np.fft.rfft((d - d.mean()) * np.hanning(len(d)))) ** 2
        f = np.fft.rfftfreq(len(d), dt)
        # 차분이 사실상 0 이면 (완전한 상수·bias) 스펙트럼 비율은 부동소수 먼지라 의미가 없다
        hi = (X[f >= 2.0].sum() / X.sum() * 100) if np.std(d) > 1e-9 * max(np.std(x), 1.0) else float("nan")
        fake = f"{np.std(d)/1e3/TS_MPC:12.3f} m/s" if i < 2 else " " * 16
        hs = "     —" if np.isnan(hi) else f"{hi:8.1f}%"
        print(f"{names[i]:>12} | {np.std(x):12.3f} | {np.std(d):12.3f} | {fake} | {hs}")
    print("\n판정 (위치 x·y 기준):")
    for i in (0, 1):
        x = R[:, i + 1]; x = x - np.polyval(np.polyfit(t, x, 1), t)
        sd = np.std(np.diff(x))
        v = sd / TS_MPC
        verdict = ("양호 — MPC 명령이 매끄러울 것" if v < 0.02 else
                   "주의 — 명령에 고주파가 섞인다" if v < 0.06 else
                   "나쁨 — Δx_m 이 주행속도에 맞먹는 가짜 속도를 읽는다")
        print(f"  {'xy'[i]}: 차분 σ {sd*1e3:.2f} mm → 가짜 속도 {v:.3f} m/s   {verdict}")
    print("  (주행속도 ~0.6 m/s 대비 몇 % 인지가 핵심. 상수 bias·느린 드리프트는 여기 안 잡히고, 잡혀도 무해하다)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/odometry/filtered")
    ap.add_argument("--sec", type=float, default=30.0)
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = OdomNoise(a.topic, a.sec)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    R = np.array(node.rows, float)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if len(R) < 32:
        print(f"[ERROR] 표본이 {len(R)} 개뿐입니다 — 토픽이 나오는지 확인하세요")
        raise SystemExit(1)
    report(R)


if __name__ == "__main__":
    main()
