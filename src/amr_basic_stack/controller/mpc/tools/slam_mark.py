#!/usr/bin/python3
"""5-1 라이다 오도메트리 확인: Enter 를 누를 때마다 SLAM 위치를 찍고, 앞 표시에서 얼마나 움직였는지 보여 준다 (2026-11-14).

로봇을 손으로 밀어(모터 끔 — 토크 스택 없이 센서·측위만) 바닥 테이프의 아는 거리·각도에 맞춘 뒤 Enter.
/tracked_pose 는 SLAM 추적점(IMU 자리)이라 바퀴 축 중심으로 옮겨서 잰다 (torque_estimator.yaml imu_offset).
표시마다 0.5 s 평균을 쓴다 (정지 잡음 줄이기).

  export ROS_DOMAIN_ID=81 && unset ROS_STATIC_PEERS
  /usr/bin/python3 tools/slam_mark.py --csv ~/capston/capston/analysis/torque_mpc_real/<날짜>_lidar/marks
  Enter = 표시, 숫자 입력 후 Enter = 그 표시에 줄자 값 메모(예: 1.000), r = 기준을 지금 위치로, q = 끝
"""
import argparse
import math
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from amr_mpc.core import config


class Marker(Node):
    def __init__(self, off):
        super().__init__("slam_mark")
        self.off = off
        self.lock = threading.Lock()
        self.buf = []
        self.create_subscription(PoseStamped, "/tracked_pose", self.on_pose, 20)

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        x = m.pose.position.x - (c * self.off[0] - s * self.off[1])
        y = m.pose.position.y - (s * self.off[0] + c * self.off[1])
        with self.lock:
            self.buf.append((time.time(), x, y, yaw))
            self.buf = self.buf[-200:]

    def mean(self, sec=0.5):
        with self.lock:
            b = [p for p in self.buf if p[0] >= time.time() - sec]
        if not b:
            return None
        a = np.array(b)
        yaw = math.atan2(np.sin(a[:, 3]).mean(), np.cos(a[:, 3]).mean())
        return a[:, 1].mean(), a[:, 2].mean(), yaw, len(b), 1000 * np.ptp(a[:, 1]), 1000 * np.ptp(a[:, 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="", help="표시를 남길 파일 앞부분 (경로) → <앞>.csv")
    a = ap.parse_args()
    est = config.section("estimator", "torque_estimator.yaml")
    off = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))
    rclpy.init()
    n = Marker(off)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    time.sleep(1.0)
    f = None
    if a.csv:
        import os
        path = os.path.expanduser(a.csv) + ".csv"
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        f = open(path, "w")
        f.write("mark,t,x,y,yaw_deg,d_fwd_m,d_left_m,dist_m,dyaw_deg,tape_m\n")
    print(f"바퀴 축 중심 = SLAM 추적점 − R(yaw)·{off} m. Enter 로 표시 (첫 표시가 기준)")
    ref = prev = None
    k = 0
    try:
        while True:
            s = input("> ").strip().lower()
            if s == "q":
                break
            m = n.mean()
            if m is None:
                print("  /tracked_pose 가 0.5 s 동안 없음 — 측위·ROS_DOMAIN_ID 확인")
                continue
            x, y, yaw, cnt, jx, jy = m
            if s == "r" or ref is None:
                ref = prev = (x, y, yaw)
                print(f"  기준: ({x:+.3f}, {y:+.3f}) m, {math.degrees(yaw):+.1f}°  (0.5 s 동안 {cnt} 개, 흔들림 {jx:.1f}·{jy:.1f} mm)")
                continue
            k += 1
            c, sn = math.cos(prev[2]), math.sin(prev[2])
            dx, dy = x - prev[0], y - prev[1]
            fwd, left = c * dx + sn * dy, -sn * dx + c * dy
            dyaw = math.degrees(math.atan2(math.sin(yaw - prev[2]), math.cos(yaw - prev[2])))
            cr, sr = math.cos(ref[2]), math.sin(ref[2])
            rdx, rdy = x - ref[0], y - ref[1]
            tape = s if s.replace(".", "", 1).replace("-", "", 1).isdigit() else ""
            print(f"  [{k}] 앞 표시에서: 앞 {fwd*100:+.1f} cm, 왼쪽 {left*100:+.1f} cm, 거리 {math.hypot(dx, dy)*100:.1f} cm, "
                  f"회전 {dyaw:+.1f}°   | 기준에서: 앞 {(cr*rdx+sr*rdy)*100:+.1f}, 왼쪽 {(-sr*rdx+cr*rdy)*100:+.1f} cm, "
                  f"{math.degrees(math.atan2(math.sin(yaw-ref[2]), math.cos(yaw-ref[2]))):+.1f}°"
                  + (f"   (줄자 {float(tape)*100:.1f} cm → 차이 {(math.hypot(dx, dy)-float(tape))*100:+.1f} cm)" if tape else ""))
            if f:
                f.write(f"{k},{time.time():.3f},{x:.4f},{y:.4f},{math.degrees(yaw):.2f},{fwd:.4f},{left:.4f},"
                        f"{math.hypot(dx, dy):.4f},{dyaw:.2f},{tape}\n")
                f.flush()
            prev = (x, y, yaw)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if f:
            f.close()
        ex.shutdown()              # spin 스레드를 먼저 멈춘다 (끝낼 때 core dump 방지, 2026-11-15)
        th.join(timeout=2.0)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
