#!/usr/bin/python3
"""Floor test of mecanum kinematics: forward, left strafe, CCW rotation (open-loop torque, no MPC).

Each motion: operator presses Enter -> torque pattern for --time s -> DAMPING.
Pattern = minimum-norm wheel torques for a unit body force/moment, from the model's T^-1,
scaled so max |tau| = --amp. Prints measured body motion and per-wheel speed check.

Needs: ak40_can_node + state_estimator running (torque_experiment.launch.py plant:=real use_mpc:=false),
and nothing else publishing /wheel_torque_cmd.

  python3 tools/basic_motion_test.py                                  # 직진 → 왼쪽 옆걸음 → 반시계 회전
  python3 tools/basic_motion_test.py --only rotate --repeat 3 --time 1.0   # 회전만 3 번 (0.5 s), 회전 중심 맞춤 출력

Rotation also fits /tracked_pose to p = c + R(θ)·r: r = body position of the SLAM tracked point relative to the
true rotation centre (compare with torque_estimator.yaml imu_offset / pose_offset). 2026-11-14 첫 실차 결과는
amr/notes_amr/real_robot_checklist.md 5단계 머리 기록.
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.torque.model import TorquePlant

WHEELS = ("FL", "FR", "RL", "RR")
MOTIONS = [  # name, body wrench direction [Fx, Fy, Mz], expected body motion axis index, time scale
    ("직진 (앞으로)", np.array([1.0, 0.0, 0.0]), 0, 1.0),
    ("옆걸음 (왼쪽으로)", np.array([0.0, 1.0, 0.0]), 1, 1.0),
    ("제자리 회전 (반시계)", np.array([0.0, 0.0, 1.0]), 2, 0.5),   # 회전은 관성이 작아 절반 시간
]


class Tester(Node):
    def __init__(self):
        super().__init__("basic_motion_test")
        self.pub = self.create_publisher(WheelTorqueCommand, "/wheel_torque_cmd", 10)
        self.lock = threading.Lock()
        self.odom = []    # (t, vx, vy, wz, x, y, yaw)
        self.wheels = []  # (t, w[4], err[4])
        self.create_subscription(Odometry, "/state_estimator/odom", self.on_odom, 50)
        self.create_subscription(WheelStates, "/wheel_states", self.on_wheels, 50)
        self.slam = []    # (t, x, y, yaw) of Cartographer tracked point
        self.create_subscription(PoseStamped, "/tracked_pose", self.on_slam, 50)

    def on_slam(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        with self.lock:
            self.slam.append((time.time(), m.pose.position.x, m.pose.position.y, yaw))

    def fit_rotation(self, t0, t1):
        """Pure rotation: p = c + R(θ) r  → least squares for c (map) and r (body offset of tracked point)."""
        with self.lock:
            s = [x for x in self.slam if t0 <= x[0] <= t1]
        if len(s) < 6:
            return None
        th = np.unwrap([x[3] for x in s])
        if th.max() - th.min() < math.radians(15):
            return None
        A, b = [], []
        for (_, px, py, _), t in zip(s, th):
            c, sn = math.cos(t), math.sin(t)
            A.append([1, 0, c, -sn]); b.append(px)
            A.append([0, 1, sn, c]); b.append(py)
        A, b = np.array(A), np.array(b)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        resid = np.sqrt(np.mean((A @ sol - b) ** 2))
        return sol[2], sol[3], resid, math.degrees(th.max() - th.min()), len(s)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        t = m.twist.twist
        with self.lock:
            self.odom.append((time.time(), t.linear.x, t.linear.y, t.angular.z,
                              m.pose.pose.position.x, m.pose.pose.position.y, yaw))

    def on_wheels(self, m):
        with self.lock:
            self.wheels.append((time.time(), list(m.velocity), list(m.error)))

    def send(self, mode, tau):
        msg = WheelTorqueCommand()
        msg.mode = mode
        msg.torque = [float(x) for x in tau]
        self.pub.publish(msg)

    def hold(self, mode, tau, sec, rate=50.0):
        end = time.time() + sec
        while time.time() < end:
            self.send(mode, tau)
            time.sleep(1.0 / rate)

    def window(self, t0, t1):
        with self.lock:
            od = [o for o in self.odom if t0 <= o[0] <= t1]
            wh = [w for w in self.wheels if t0 <= w[0] <= t1]
        return od, wh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--amp", type=float, default=0.10, help="max |wheel torque| [N·m]")
    ap.add_argument("--time", type=float, default=0.7, help="torque duration [s]")
    ap.add_argument("--only", choices=["forward", "strafe", "rotate"], default=None, help="한 동작만 (반복 가능)")
    ap.add_argument("--repeat", type=int, default=1, help="--only 동작 반복 횟수")
    a = ap.parse_args()
    motions = MOTIONS
    if a.only:
        motions = [MOTIONS[{"forward": 0, "strafe": 1, "rotate": 2}[a.only]]] * max(1, a.repeat)
    if not (0.0 < a.amp <= 0.2 and 0.0 < a.time <= 1.5):
        sys.exit("amp ≤ 0.2 N·m, time ≤ 1.5 s 로 제한")

    p = TorquePlant.from_yaml()
    Tinv = p.Tinv
    pinv = np.linalg.pinv(Tinv.T)  # body wrench -> min-norm wheel torques
    rclpy.init()
    node = Tester()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)

    print(f"모델 T⁻¹ (행 FL FR RL RR, 열 vx vy ω) × r:\n{np.round(Tinv * p.r, 3)}")
    try:
        for name, wrench, axis, tscale in motions:
            tau = pinv @ wrench
            tau = tau / np.max(np.abs(tau)) * a.amp
            dur = a.time * tscale
            print(f"\n=== {name}: 토크 [FL FR RL RR] = {np.round(tau, 3)} N·m, {dur:.2f} s")
            s = input("    로봇 주변 확인 후 Enter = 실행, q = 끝: ").strip().lower()
            if s == "q":
                break
            node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.5)
            with node.lock:
                fresh_odom = bool(node.odom) and time.time() - node.odom[-1][0] < 0.3
                fresh_wheels = bool(node.wheels) and time.time() - node.wheels[-1][0] < 0.3
            if not (fresh_odom and fresh_wheels):
                print(f"    ★ 실행 안 함: 상태 추정 {'OK' if fresh_odom else '없음'}, 바퀴 상태 {'OK' if fresh_wheels else '없음'}")
                continue
            node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.3)
            t0 = time.time()
            node.hold(WheelTorqueCommand.MODE_TORQUE, tau, dur)
            t1 = time.time()
            node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 1.5)
            t2 = time.time()
            od, wh = node.window(t0, t2)
            od_drive, wh_drive = node.window(t0 + 0.2, t1 + 0.2)
            if len(od) < 5 or len(wh) < 5:
                print("    ★ 상태 추정·바퀴 데이터가 부족함 (노드 확인)")
                continue
            # 출발 몸체 좌표로 적분 (회전이 크게 돌아도 접히지 않게 ω 를 적분)
            dfx = dfy = th = 0.0
            for (ta, vxa, vya, wza, *_), (tb, *_rest) in zip(od[:-1], od[1:]):
                dt = tb - ta
                dfx += (vxa * math.cos(th) - vya * math.sin(th)) * dt
                dfy += (vxa * math.sin(th) + vya * math.cos(th)) * dt
                th += wza * dt
            dth = math.degrees(th)
            print(f"    실제 이동 (출발 시 몸체 기준): 앞 {dfx*100:+.1f} cm, 왼쪽 {dfy*100:+.1f} cm, 회전 {dth:+.1f}°")
            v = np.array([[o[1], o[2], o[3]] for o in od_drive]).mean(axis=0) if od_drive else np.zeros(3)
            w = np.array([x[1] for x in wh_drive]).mean(axis=0) if wh_drive else np.zeros(4)
            errs = sorted({e for x in wh for e in x[2] if e})
            w_model = Tinv @ v
            print(f"    추정 몸체 속도 (토크 구간 평균): vx {v[0]:+.3f} m/s, vy {v[1]:+.3f} m/s, ω {v[2]:+.3f} rad/s")
            print("    바퀴 속도 [rad/s]   실제      모델(T⁻¹·v)   토크부호  실제부호")
            for i, wn in enumerate(WHEELS):
                print(f"      {wn}           {w[i]:+7.2f}    {w_model[i]:+7.2f}        {'+' if tau[i] > 0 else '-'}        {'+' if w[i] > 0 else '-'}")
            main_ok = (axis < 2 and abs([dfx, dfy][axis]) > 0.02 and [dfx, dfy][axis] > 0
                       and abs([dfx, dfy][1 - axis]) < 0.5 * abs([dfx, dfy][axis])) or \
                      (axis == 2 and dth > 3 and math.hypot(dfx, dfy) < 0.05)
            sign_ok = all((w[i] > 0) == (tau[i] > 0) for i in range(4) if abs(w[i]) > 0.5)
            print(f"    판정: 의도한 방향 {'맞음' if main_ok else '★ 다름'}, 바퀴 회전 부호 {'토크와 같음' if sign_ok else '★ 다름'}"
                  + (f", 모터 에러 {errs}" if errs else ""))
            if axis == 2:
                f = node.fit_rotation(t0 - 0.3, t2 + 0.5)
                if f:
                    rx, ry, res, ang, n = f
                    print(f"    [회전 중심 맞춤] SLAM 기준점의 몸체 위치 (실제 회전 중심 기준): x {rx*1000:+.1f} mm, y {ry*1000:+.1f} mm"
                          f"  (잔차 {res*1000:.2f} mm, 회전 {ang:.0f}°, 포즈 {n} 개)")
                else:
                    print("    [회전 중심 맞춤] 데이터 부족 (회전 15° 미만 또는 포즈 6 개 미만)")
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.5)
        print("DAMPING 보냄. 끝.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
