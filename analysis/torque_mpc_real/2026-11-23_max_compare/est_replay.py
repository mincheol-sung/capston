#!/usr/bin/python3
"""상태 추정기 (amr_mpc/torque/estimator.py PlanarStateEstimator, kf) 를 bag 의 /imu/base_link · /tracked_pose 로 다시 돌려
SLAM 계단 (121319 1.04 s, 81 mm) 에 대한 후보 처리 비교. 게인 (K_p·K_v·K_b) 은 그대로 — 큰 잔차가 왔을 때의 처리만 바꾼다.
  V0 지금        : |e| ≤ 15 cm 면 K_p·e, K_v·e, K_b·e 를 한 번에
  V1 잔차 자르기  : e 를 |e| ≤ c (3 cm) 로 잘라 같은 게인 (계단을 여러 번에 나눠 따라감 — innovation clipping)
  V2 위치만       : |e| > c 면 K_p·e 만 (K_v·e·K_b·e 안 함 — 속도 상태를 계단으로 차지 않음)
  V3 문턱 5 cm   : |e| > 5 cm 면 버림 (연속 5 번이면 그 포즈로 재초기화 — 지금 규칙)
  V4 V1+V2       : |e| > c 면 K_p·(c 로 자른 e) 만 — 위치를 천천히 (0.1 s 에 0.711·3 cm), 속도·바이어스는 안 참
노드처럼 stamped 포즈 (새 stamp 만). 출력은 IMU 마다 state() (로봇 중심)."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core import config
from amr_mpc.torque.estimator import PlanarStateEstimator
from load import load, ref_at
B = "/home/smoc/capston/capston/bags/2026-11-23"
CLIP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03


class Variant(PlanarStateEstimator):
    def __init__(self, cfg, kind):
        self.kind = kind
        if kind == "V3":
            cfg = dict(cfg, jump_xy=0.05)
        super().__init__(cfg)

    def _apply(self, tm, dp, dv, db, dpsi, dbg):
        if getattr(self, "_e", None) is not None and self.kind in ("V1", "V2", "V4"):
            e = self._e; n = math.hypot(*e)
            if n > CLIP:
                Kp, Kv, Kb = self.K_xy
                if self.kind == "V1":
                    es = e * (CLIP / n); dp, dv, db = Kp * es, Kv * es, Kb * es
                elif self.kind == "V4":                  # V1 + V2: 위치만, 그것도 K_p·(3 cm 로 자른 e) — 0.1 s 에 2.1 cm 씩 따라감
                    dp, dv, db = Kp * e * (CLIP / n), np.zeros(2), np.zeros(2)
                else:
                    dv, db = np.zeros(2), np.zeros(2)
        super()._apply(tm, dp, dv, db, dpsi, dbg)

    def pose(self, tm, x, y, yaw):
        self._e = None
        if self.pose_ready and not (self.t is not None and self.t - tm > self.max_pose_age):
            p_m, _, _ = self._hist_at(tm); self._e = self._to_imu_point(x, y, yaw) - p_m
        return super().pose(tm, x, y, yaw)


def replay(bag, kind, cfg):
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}; rd.set_filter(rosbag2_py.StorageFilter(topics=["/imu/base_link", "/tracked_pose"]))
    est = Variant(cfg, kind); out = []; last = -1.0; jumps = 0
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    while rd.has_next():
        tp, data, tb = rd.read_next(); m = deserialize_message(data, get_message(types[tp]))
        if tp == "/imu/base_link":
            if est.imu(st(m.header), m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z) and est.ready_to_publish():
                out.append(est.state())
        else:
            t = st(m.header)
            if t <= last:
                continue
            last = t; r = est.pose(t, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)); jumps += r in ("jump", "reset")
    return np.array(out), jumps


if __name__ == "__main__":
    bag = sys.argv[1] if len(sys.argv) > 1 else "121319"
    cfg = dict(config.section("estimator", "torque_estimator.yaml"))
    d = load(f"{B}/{bag}"); D = d["D"]; sl, E = d["slam"], d["E"]; a0, a1 = d["arm"]
    ts = np.arange(a0, a0 + 60, 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts])
    tr0 = ts[np.argmax(sp > 0.02)]; tr1 = ts[len(sp) - 1 - np.argmax(sp[::-1] > 0.02)]
    Xr0, _ = ref_at(D, tr0 + 0.5); n0 = np.array([-math.sin(Xr0[2]), math.cos(Xr0[2])])
    i = np.argmin(np.abs(sl[:, 0] - (tr0 + 0.5))); base = sl[i, 1:3]
    res = {}
    for kind in ("V0", "V1", "V2", "V3", "V4"):
        o, nj = replay(bag, kind, cfg); res[kind] = o
        run = (o[:, 0] >= tr0) & (o[:, 0] <= tr1)
        es = np.stack([np.interp(o[run, 0], sl[:, 0], sl[:, 1]), np.interp(o[run, 0], sl[:, 0], sl[:, 2])], 1)
        dev = np.linalg.norm(o[run, 1:3] - es, axis=1)
        step = np.linalg.norm(np.diff(o[run, 1:3], axis=0), axis=1) - np.hypot(o[run, 4], o[run, 5])[1:] * np.diff(o[run, 0])
        win = (o[:, 0] > tr0 + 1.0) & (o[:, 0] < tr0 + 1.6); lat = (o[win, 1:3] - base) @ n0
        msg = f"{kind}: 추정기 − SLAM RMS {1e3 * np.sqrt(np.mean(dev ** 2)):.0f} mm (최대 {1e3 * dev.max():.0f}), 10 ms 위치 계단 최대 {1e3 * step.max():.0f} mm, 튐 처리 {nj} 번"
        if bag == "121319":
            msg += f", 1.0~1.6 s 옆 위치 최대 {1e3 * lat.max():+.0f} mm (SLAM {1e3 * ((sl[(sl[:,0]>tr0+1.0)&(sl[:,0]<tr0+1.6),1:3]-base)@n0).max():+.0f})"
        print(msg)
    # V0 가 실제 노드 출력과 같은지
    o = res["V0"]; run = (o[:, 0] >= tr0) & (o[:, 0] <= tr1)
    dx = np.hypot(np.interp(o[run, 0], E[:, 0], E[:, 1]) - o[run, 1], np.interp(o[run, 0], E[:, 0], E[:, 2]) - o[run, 2])
    print(f"[검산] V0 재생 − 실제 /state_estimator/odom 위치 차 중앙 {1e3 * np.median(dx):.1f} mm, 최대 {1e3 * dx.max():.1f} mm")
    np.savez(f"est_replay_{bag}.npz", **res, tr0=tr0, base=base, n0=n0)
