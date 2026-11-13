#!/usr/bin/python3
"""비교군 A/B/C 한 판 기록 — 시뮬 참값으로 추종 오차·토크를 잰다 (6단계 비교용, 로깅 전용).

제어기마다 자기 status 의 오차는 "추정 상태" 기준이라 서로 비교하기 어렵다. 이 도구는 셋 모두에 같은 잣대를 댄다:
  추종 오차 = |기준열(지금 시각) − /sim/ground_truth|   (기준열은 figure8 이 낸 것 그대로, core/reference.py 로 보간)
  토크      = /wheel_torque_cmd (게이트를 지난 최종 명령 — 플랜트가 실제로 받은 것)
기준 속도가 0 이 아닌 구간("주행 중")과 무장 구간 전체를 따로 낸다. 기준열이 end_after 초 끊기면(8자 완주 + 유지 끝) 끝낸다.

  launch 를 띄운 터미널과 같은 ROS_DOMAIN_ID 에서:
    /usr/bin/python3 tools/compare_run.py --label A --timeout 90
  마지막 줄에 JSON 요약을 찍는다.
"""
import argparse
import json
import math
import signal
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from amr_interfaces.msg import PlanarReferenceArray, WheelTorqueCommand
from amr_mpc.core.reference import ReferenceHorizon, wrap


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, end_after):
        super().__init__("compare_run_recorder")
        self.end_after = end_after
        self.ref, self.ref_rx, self.armed, self.fault = None, None, False, ""
        self.err, self.err_yaw, self.moving, self.tau, self.tau_t = [], [], [], [], []
        self.n_damping = 0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PlanarReferenceArray, "/dra_mppi/reference_trajectory", self.on_ref, 10)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_truth, 50)
        self.create_subscription(WheelTorqueCommand, "/wheel_torque_cmd", self.on_cmd, 50)
        self.create_subscription(Bool, "/mpc/armed", lambda m: setattr(self, "armed", bool(m.data)), latched)
        self.create_subscription(String, "/fault_monitor/fault", lambda m: setattr(self, "fault", m.data), latched)

    def on_ref(self, m):
        if not m.references:
            return
        rows = [(r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for r in m.references]
        self.ref = ReferenceHorizon.from_rows(stamp_s(m.header), m.dt, rows)
        self.ref_rx = time.monotonic()

    def on_truth(self, m):
        if not self.armed or self.ref is None:
            return
        t = stamp_s(m.header)
        Xr, ur = self.ref.at(t)
        p = m.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z), 1 - 2 * p.orientation.z ** 2)
        self.err.append(math.hypot(Xr[0] - p.position.x, Xr[1] - p.position.y))
        self.err_yaw.append(abs(wrap(Xr[2] - yaw)))
        self.moving.append(math.hypot(ur[0], ur[1]) > 0.02)

    def on_cmd(self, m):
        if not self.armed:
            return
        if m.mode == WheelTorqueCommand.MODE_DAMPING:
            self.n_damping += 1
            return
        self.tau.append(list(m.torque))
        self.tau_t.append(stamp_s(m.header))

    def finished(self):
        return self.ref_rx is not None and time.monotonic() - self.ref_rx > self.end_after

    def summary(self, label):
        e, ey, mv = np.array(self.err), np.array(self.err_yaw), np.array(self.moving, bool)
        tau = np.array(self.tau).reshape(-1, 4)
        out = {"label": label, "samples": int(len(e)), "fault": self.fault}
        if len(e):
            out.update(rms_mm=1e3 * float(np.sqrt(np.mean(e ** 2))), max_mm=1e3 * float(e.max()),
                       yaw_rms_deg=math.degrees(float(np.sqrt(np.mean(ey ** 2)))))
        if mv.any():
            out.update(moving_rms_mm=1e3 * float(np.sqrt(np.mean(e[mv] ** 2))), moving_max_mm=1e3 * float(e[mv].max()))
        if len(tau):
            out.update(tau_max=float(np.abs(tau).max()), tau_rms=float(np.sqrt(np.mean(tau ** 2))),
                       dtau_rms=float(np.sqrt(np.mean(np.diff(tau, axis=0) ** 2))) if len(tau) > 1 else 0.0,
                       n_torque=int(len(tau)))
        out["n_damping_while_armed"] = self.n_damping
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--end_after", type=float, default=1.5, help="기준열이 이만큼 [s] 끊기면 끝")
    a = ap.parse_args()
    rclpy.init()
    node = Recorder(a.end_after)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["flag"] and not node.finished() and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    s = node.summary(a.label)
    s["timed_out"] = time.monotonic() - t0 >= a.timeout
    print(json.dumps(s, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
