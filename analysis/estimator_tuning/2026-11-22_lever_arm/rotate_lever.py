#!/usr/bin/python3
"""제자리 스핀 판 (shape:=rotate seg_out_and_back:=true) 으로 SLAM 추적점의 실제 위치 재기 (2026-11-22).

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 analysis/estimator_tuning/2026-11-22_lever_arm/rotate_lever.py bags/2026-11-22/<시분초> [...]

원리: 추정기의 로봇 중심 속도 v_c = v_imu − ω × r_yaml. 실제 r = r_yaml + δ 면 v_c 에 ω × δ = (−ω δ_y, ω δ_x) 가 섞인다.
바퀴 기구학 v_w = (r/4)Σ(±ω_i) 는 운동학 중심의 속도. 그래서 제자리 회전에서
  Δv_x = v_c,x − v_w,x = −δ_y ω + b_x,    Δv_y = v_c,y − v_w,y = δ_x ω + b_y
양방향 (out_and_back: + 로 돌고 − 로 돌아옴) 이라 ω 부호가 바뀌어 일정한 치우침 b 와 δ 가 갈린다.
창: 기준 각속도가 순항값의 90 % 이상인 정상 회전 구간 (가감속 빼기). 불확실성: 0.5 s 블록 부트스트랩.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

R_YAML = np.array([0.1093, 0.017])          # torque_estimator.yaml imu_offset = 실행 TF camera_gyro_frame (2026-11-21 bag 확인)
TOP = ["/wheel_states", "/state_estimator/odom", "/dra_mppi/reference_trajectory", "/tracked_pose", "/slip_logger/slip"]


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    have = {t.name for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOP if t in have]))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in TOP}
    while r.has_next():
        n, d, t = r.read_next()
        D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    return D


def samples(b):
    D = read(b)
    W = np.array([[st(m.header), *m.velocity] for _, m in D["/wheel_states"]])
    E = np.array([[st(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, m in D["/state_estimator/odom"]])
    t = W[:, 0]
    keep = (t > E[0, 0]) & (t < E[-1, 0])
    w = W[keep, 1:5]; t = t[keep]
    vw = np.stack([0.01 * w.sum(1), 0.01 * (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3])], 1)      # r/4 = 0.01
    par = w[:, 0] + w[:, 1] - w[:, 2] - w[:, 3]                                                # T 의 null space: 무슬립이면 0
    ve = np.stack([np.interp(t, E[:, 0], E[:, i]) for i in (1, 2, 3)], 1)
    if D["/dra_mppi/reference_trajectory"]:
        R = np.array(sorted((st(m.header) + m.dt, r.yaw_rate, math.hypot(r.vx, r.vy)) for _, m in D["/dra_mppi/reference_trajectory"]
                            if m.references for r in m.references[:1]))
        wr = np.interp(t, R[:, 0], R[:, 1], left=0, right=0); vr = np.interp(t, R[:, 0], R[:, 2], left=0, right=0)
        cruise = np.percentile(np.abs(wr), 95)
        win = (np.abs(wr) >= 0.9 * cruise) & (vr < 0.02) & (cruise > 0.2)
        src = f"기준 순항 |ω_r| {cruise:.2f} rad/s 의 90 % 이상"
    else:                                                     # 기준열이 없는 bag (점검용): 추정 ω 로
        win = np.abs(ve[:, 2]) > 0.5
        src = "기준열 없음 → |ω_gyro| > 0.5"
    return t[win], vw[win], ve[win], par[win], src


def fit(dvx, dvy, wz):
    A = np.stack([wz, np.ones_like(wz)], 1)
    kx = np.linalg.lstsq(A, dvx, rcond=None)[0]; ky = np.linalg.lstsq(A, dvy, rcond=None)[0]
    return np.array([ky[0], -kx[0], kx[1], ky[1]])            # δ_x, δ_y, b_x, b_y


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    allx = []
    for b in sys.argv[1:]:
        t, vw, ve, par, src = samples(b)
        if len(t) < 100:
            print(f"== {b}: 정상 회전 구간이 {len(t)/100:.1f} s 뿐 — shape:=rotate 판인지 확인 ({src})")
            continue
        dv = ve[:, :2] - vw
        wz = ve[:, 2]
        pos, neg = wz > 0, wz < 0
        f = fit(dv[:, 0], dv[:, 1], wz)
        # 블록 부트스트랩 (0.5 s = 50 표본)
        blk = [np.arange(i, min(i + 50, len(t))) for i in range(0, len(t), 50)]
        bs = []
        for _ in range(500):
            idx = np.concatenate([blk[j] for j in rng.integers(0, len(blk), len(blk))])
            bs.append(fit(dv[idx, 0], dv[idx, 1], wz[idx]))
        se = np.std(bs, axis=0)
        print(f"== {b}  ({src})")
        print(f"   정상 회전 표본 {len(t)} 개 = {len(t)/100:.1f} s (+ 방향 {pos.sum()/100:.1f} s, − 방향 {neg.sum()/100:.1f} s), |ω| 평균 {np.abs(wz).mean():.2f} rad/s")
        for lab, m in (("+ 방향", pos), ("− 방향", neg)):
            if m.sum() > 10:
                print(f"   {lab}: Δv 평균 ({dv[m,0].mean()*1e3:+.1f}, {dv[m,1].mean()*1e3:+.1f}) mm/s, Δv RMS {np.sqrt(np.mean(np.sum(dv[m]**2,1)))*1e3:.1f} mm/s, "
                      f"바퀴 parity |c| 중앙값 {np.median(np.abs(par[m])):.2f} rad/s")
        print(f"   δ = ({f[0]*1e3:+.1f} ± {se[0]*1e3:.1f}, {f[1]*1e3:+.1f} ± {se[1]*1e3:.1f}) mm → 실제 r = ({R_YAML[0]+f[0]:.4f}, {R_YAML[1]+f[1]:.4f}) m"
              f"   (일정 치우침 b = ({f[2]*1e3:+.1f}, {f[3]*1e3:+.1f}) mm/s)")
        if pos.sum() < 100 or neg.sum() < 100:
            print("   ! 한 방향이 1 s 보다 짧다 — δ 와 b 가 잘 안 갈린다 (seg_out_and_back:=true 인지 확인)")
        allx.append(f)
    if len(allx) > 1:
        A_ = np.array(allx)
        print(f"\n판 {len(A_)} 개: δ_x {A_[:,0].mean()*1e3:+.1f} (판 사이 σ {A_[:,0].std(ddof=1)*1e3:.1f}), δ_y {A_[:,1].mean()*1e3:+.1f} (σ {A_[:,1].std(ddof=1)*1e3:.1f}) mm")
