#!/usr/bin/python3
"""bag 재생 결과(replay.sh) 를 바퀴·IMU·기록 SLAM·줄자와 비교한다 (2026-11-15).

  # 토크 시험 bag (basic_motion_test): 토크마다 2.5 s 뒤 앞 이동과 선 뒤 최대 넘침
  /usr/bin/python3 tools/slam_replay/compare.py --bag <bag> V0 V2 ...
  # 손으로 밀기 bag (slam_mark.py): 표시마다 앞 표시에서의 거리·회전을 줄자와
  /usr/bin/python3 tools/slam_replay/compare.py --bag <bag> --marks <marks.csv> V0 V2 ...
라벨은 replay.sh --label 과 같게 (결과 폴더 --out, 기본 ~/slam_replay_out).
위치는 모두 바퀴 축 중심(추적점 − R(yaw)·imu_offset, torque_estimator.yaml)으로 옮겨 비교한다.
"""
import argparse
import csv
import math
import os

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

from amr_mpc.core import config
from amr_mpc.torque.model import TorquePlant


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_bag(path, topics):
    rd = SequentialReader()
    rd.open(StorageOptions(uri=path, storage_id="mcap"), ConverterOptions("", ""))
    rd.set_filter(StorageFilter(topics=topics))
    types = {m.name: m.type for m in rd.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while rd.has_next():
        tp, data, ts = rd.read_next()
        out[tp].append((ts * 1e-9, deserialize_message(data, get_message(types[tp]))))
    return out


def base_pose(a, off):
    """[stamp, x, y, qz, qw] 열 → (t, bx, by, yaw)."""
    yaw = np.unwrap(2 * np.arctan2(a[:, 3], a[:, 4]))
    c, s = np.cos(yaw), np.sin(yaw)
    return a[:, 0], a[:, 1] - (c * off[0] - s * off[1]), a[:, 2] - (s * off[0] + c * off[1]), yaw


def load_replay(out, label):
    a = np.genfromtxt(os.path.join(out, label, "pose.csv"), delimiter=",", skip_header=1, ndmin=2)
    if a.size == 0:
        return None
    return a[:, [0, 1, 2, 5, 6]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--marks", default="")
    ap.add_argument("--out", default=os.path.expanduser("~/slam_replay_out"))
    ap.add_argument("labels", nargs="*")
    a = ap.parse_args()
    est = config.section("estimator", "torque_estimator.yaml")
    off = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))
    B = read_bag(a.bag, ["/tracked_pose", "/wheel_states", "/imu/base_link", "/wheel_torque_cmd"])
    rec = np.array([[stamp(m.header), m.pose.position.x, m.pose.position.y, m.pose.orientation.z, m.pose.orientation.w]
                    for _, m in B["/tracked_pose"]])
    srcs = {"기록 SLAM": rec}
    for l in a.labels:
        r = load_replay(a.out, l)
        if r is None:
            print(f"{l}: 포즈 0 개 — carto.log 확인")
            continue
        srcs[l] = r
    if a.marks:
        marks = list(csv.DictReader(open(os.path.expanduser(a.marks))))
        tm = [float(m["t"]) for m in marks]
        tape = [m["tape_m"] for m in marks]

        def at(src, t):
            T, X, Y, Yaw = base_pose(src, off)
            m = (T >= t - 0.5) & (T <= t)
            if m.sum() == 0:
                return None
            return X[m].mean(), Y[m].mean(), math.atan2(np.sin(Yaw[m]).mean(), np.cos(Yaw[m]).mean())
        names = list(srcs)
        print("표시 | 줄자 [cm] | " + " | ".join(f"{n} 거리·회전" for n in names))
        prev = {n: at(srcs[n], tm[0] - 5.0) for n in names}   # 기준 표시 직전
        for k, t in enumerate(tm):
            cells = []
            for n in names:
                p = at(srcs[n], t)
                q = prev[n]
                if p is None or q is None:
                    cells.append("   —   ")
                else:
                    d = math.hypot(p[0] - q[0], p[1] - q[1])
                    dy = math.degrees(math.atan2(math.sin(p[2] - q[2]), math.cos(p[2] - q[2])))
                    cells.append(f"{100*d:6.1f} {dy:+6.1f}°")
                prev[n] = p
            tp = f"{100*float(tape[k]):6.1f}" if tape[k] else "   —  "
            print(f"[{k+1:2d}] | {tp} | " + " | ".join(cells))
        return
    # 토크 시험: /wheel_torque_cmd 의 토크 시작마다
    cmd = [(t, m.mode, m.torque[0]) for t, m in B["/wheel_torque_cmd"]]
    ev = [(cmd[i][0], cmd[i][2]) for i in range(1, len(cmd)) if cmd[i][1] == 0 and cmd[i - 1][1] != 0]
    p = TorquePlant.from_yaml()
    fk = np.linalg.pinv(p.Tinv)
    W = np.array([[stamp(m.header)] + list(m.velocity) for _, m in B["/wheel_states"]])
    vk = np.array([(fk @ w[1:])[0] for w in W])
    xk = np.concatenate([[0.0], np.cumsum(vk[:-1] * np.diff(W[:, 0]))])
    IM = np.array([(stamp(m.header), m.linear_acceleration.x) for _, m in B["/imu/base_link"]])
    print("#  토크 | 바퀴 | IMU | " + " | ".join(f"{n} 끝(넘침)" for n in srcs))
    for n_, (t0, amp) in enumerate(ev, 1):
        pre = (IM[:, 0] > t0 - 0.8) & (IM[:, 0] < t0 - 0.05)
        b = IM[pre, 1].mean()
        I = IM[(IM[:, 0] >= t0) & (IM[:, 0] <= t0 + 1.2)]
        v = x = 0.0
        for i in range(len(I) - 1):
            dt = I[i + 1, 0] - I[i, 0]
            acc = I[i, 1] - b
            x += v * dt + 0.5 * acc * dt * dt
            v += acc * dt
        w = np.interp(t0 + 2.5, W[:, 0], xk) - np.interp(t0, W[:, 0], xk)
        cells = []
        for n, src in srcs.items():
            T, X, Y, Yaw = base_pose(src, off)
            th0 = np.interp(t0, T, Yaw)
            f = np.cos(th0) * (X - np.interp(t0, T, X)) + np.sin(th0) * (Y - np.interp(t0, T, Y))
            mm = (T > t0 + 0.9) & (T < t0 + 3.0)
            over = (f[mm].max() if amp > 0 else f[mm].min()) if mm.any() else float("nan")
            cells.append(f"{100*np.interp(t0+2.5, T, f):+6.1f} ({100*over:+6.1f})")
        print(f"{n_} {amp:+.2f} | {100*w:+6.1f} | {100*x:+6.1f} | " + " | ".join(cells))


if __name__ == "__main__":
    main()
