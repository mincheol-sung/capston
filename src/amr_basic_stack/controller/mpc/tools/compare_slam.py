#!/usr/bin/python3
"""비교군 A(퓨어 퍼슛)·B·C(토크 MPC) 실차 bag 을 같은 잣대로 — SLAM 기준 추종 오차 (2026-11-19, 발표 비교용).

analyze_run.py 는 /torque_mpc/status(추정기 기준)를 읽어 퓨어 퍼슛 bag 을 못 본다. 이 도구는 제어기와 상관없이
모든 판에 있는 토픽만 쓴다:
  /dra_mppi/reference_trajectory  기준열 (figure8_reference_node) — core/reference.py 로 SLAM 포즈 시각에 보간
  /tracked_pose                   Cartographer 포즈. **D455 IMU 점의 포즈**라 torque_estimator.yaml imu_offset 으로 로봇 중심으로 옮긴다
                                  (안 옮기면 tangent 주행에서 진행 방향 +98 mm 가 생긴다 — 2026-11-18 분석)
  /mpc/armed                      무장 구간 (게이트, A·B·C 같은 규칙)
  /wheel_torque_cmd               게이트를 지난 최종 토크 (DAMPING 제외)
  /wheel_states                   바퀴 기구학 속도 T ω — 슬립 지표
지표 (주행 = 기준 속도 > 0.02 m/s, 무장 구간)
  위치 오차 RMS·최대, 경로 좌표 e_t(진행 방향 + 앞)·e_c(곡률 중심 쪽 + 안) RMS·평균, 교차점 창(ω_r 부호 바뀜 −0.5~+1.5 s)과 나머지,
  요 오차 RMS·최대, 토크 |τ| 최대·틱간 Δτ RMS, 슬립 = 바퀴 기구학 v − SLAM 중심 속도 (몸체 x·y, RMS)
※ SLAM 시각은 자이로·바퀴와 −50~+50 ms 로 맞는다 (2026-11-18 상호상관). 추정기 속도는 0.1 s 늦어 슬립 기준으로 쓰지 않는다.
※ SLAM 도 측정이라 참값은 아니다 (보정 잔차 RMS 1.4~2.2 cm). 판끼리 비교는 같은 날·같은 지도에서.
※ 슬립 지표는 SLAM 포즈 차분 잡음이 섞여 0.1 m/s 근처가 바닥이다 (2026-11-18·17 판 모두 0.10~0.12) — 판끼리 상대 비교만.

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 tools/compare_slam.py <bag 폴더> [<bag 폴더> ...] [--csv 표.csv]
"""
import argparse
import csv
import math
import os
import re

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "config")
TOPICS = ["/dra_mppi/reference_trajectory", "/tracked_pose", "/mpc/armed", "/wheel_torque_cmd", "/wheel_states"]


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def imu_to_center(x, y, yaw, off):
    """IMU 점 포즈 → 로봇 중심. p_c = p_imu − R(ψ) r_imu."""
    c, s = np.cos(yaw), np.sin(yaw)
    return x - (c * off[0] - s * off[1]), y - (s * off[0] + c * off[1])


def path_errors(x, y, yaw, xr, yr, thr, vx, vy, om):
    """기준 대비 오차를 경로 좌표로. 기준 속도 [vx, vy] 는 몸체(기준 헤딩), om 은 기준 요레이트.
    반환: e_t (진행 방향, + 앞), e_c (곡률 중심 쪽, + 안), |e|, 요 오차 [rad], 기준 속력."""
    c, s = np.cos(thr), np.sin(thr)
    vw = np.stack([c * vx - s * vy, s * vx + c * vy], 1)
    sp = np.linalg.norm(vw, axis=1)
    tt = vw / np.maximum(sp, 1e-9)[:, None]
    nn = np.stack([-tt[:, 1], tt[:, 0]], 1)
    e = np.stack([x - xr, y - yr], 1)
    et = (e * tt).sum(1)
    ec = (e * nn).sum(1) * np.sign(om)
    return et, ec, np.hypot(e[:, 0], e[:, 1]), wrap(yaw - thr), sp


def crossing_mask(t, om, moving, before=0.5, after=1.5):
    """주행 중 ω_r 부호가 바뀐 시각 −before ~ +after 창 (8자 교차점). 반환: (창 마스크, 교차 수)."""
    sw = np.flatnonzero((np.sign(om[1:]) != np.sign(om[:-1])) & moving[1:] & moving[:-1]) + 1
    win = np.zeros(len(t), bool)
    for k in sw:
        win |= (t >= t[k] - before) & (t <= t[k] + after)
    return win & moving, len(sw)


def armed_mask(t, events):
    """/mpc/armed (수신 시각, 값) 사건 목록 → t 가 무장 구간 안인지."""
    on = np.zeros(len(t), bool)
    ev = sorted(events)
    for i, (tb, v) in enumerate(ev):
        if v:
            t1 = next((tb2 for tb2, v2 in ev[i + 1:] if not v2), np.inf)
            on |= (t >= tb) & (t <= t1)
    return on


def rms(v, m):
    return float(np.sqrt(np.mean(v[m] ** 2))) if m.any() else float("nan")


def run_label(bag):
    """run_info 로 어떤 판인지: 제어기, 속도, μ̂, w, 속도 상태."""
    args = ""
    p = os.path.join(bag, "run_info", "args.txt")
    if os.path.isfile(p):
        args = open(p).read()
    get = lambda k: (re.search(rf"\b{k}:=(\S+)", args) or [None, ""])[1]
    ctrl = get("controller") or "mpc"
    lab = "A 퓨어퍼슛" if ctrl == "pure_pursuit" else ("MPC " + (get("limit_mode") or "") + f" μ̂{get('mu_hat') or '?'}"
                                                       + (" w" if get("use_w") in ("true", "True", "1") else "")
                                                       + (" 바퀴v" if get("velocity_source") == "wheels" else ""))
    return f"{lab} v{get('speed') or '?'} {get('shape') or ''}{get('width') or ''}"


def load(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import sys
    sys.path.insert(0, os.path.join(HERE, ".."))
    from amr_mpc.core.reference import ReferenceHorizon

    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in TOPICS if w in types]))
    D = {"ref": [], "pose": [], "armed": [], "tau": [], "wheel": []}
    while r.has_next():
        tp, data, tb = r.read_next()
        m = deserialize_message(data, get_message(types[tp]))
        if tp == "/dra_mppi/reference_trajectory" and m.references:
            D["ref"].append((st(m.header), ReferenceHorizon.from_rows(
                st(m.header), m.dt, [(q.x, q.y, q.yaw, q.vx, q.vy, q.yaw_rate) for q in m.references])))
        elif tp == "/tracked_pose":
            D["pose"].append((st(m.header), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
        elif tp == "/mpc/armed":
            D["armed"].append((tb * 1e-9, bool(m.data)))
        elif tp == "/wheel_torque_cmd":
            D["tau"].append((st(m.header), int(m.mode), *m.torque))
        elif tp == "/wheel_states":
            D["wheel"].append((st(m.header), *m.velocity))
    return D


def analyse(bag, off, r_w, k):
    D = load(bag)
    if not D["ref"] or not D["pose"] or not any(v for _, v in D["armed"]):
        raise RuntimeError(f"기준 {len(D['ref'])}·포즈 {len(D['pose'])}·무장 {sum(v for _, v in D['armed'])} — 추종 판이 아니다")
    rt = np.array([a for a, _ in D["ref"]])
    P = np.array(D["pose"])
    xc, yc = imu_to_center(P[:, 1], P[:, 2], P[:, 3], off)
    rows = []
    for t, x, y, th in zip(P[:, 0], xc, yc, P[:, 3]):
        i = np.searchsorted(rt, t, side="right") - 1
        if i < 0:
            continue
        Xr, ur = D["ref"][i][1].at(t)
        rows.append([t, x, y, th, *Xr, *ur])
    if not rows:
        raise RuntimeError("기준과 포즈가 겹치는 시간이 없다")
    a = np.array(rows)
    t, x, y, th, xr, yr, thr, vx, vy, om = a.T
    et, ec, en, ey, sp = path_errors(x, y, th, xr, yr, thr, vx, vy, om)
    mv = armed_mask(t, D["armed"]) & (sp > 0.02)
    if not mv.any():
        raise RuntimeError("무장 중 주행 구간이 없다")
    win, ncross = crossing_mask(t, om, mv)
    rest = mv & ~win
    out = dict(dur=float(t[mv][-1] - t[mv][0]), v_ref=float(sp[mv].max()),
               rms=rms(en, mv), max=float(en[mv].max()), et=rms(et, mv), et_mean=float(et[mv].mean()),
               ec=rms(ec, mv), ec_mean=float(ec[mv].mean()), cross=rms(en, win), rest=rms(en, rest), ncross=ncross,
               yaw=math.degrees(rms(ey, mv)), yaw_max=math.degrees(float(np.abs(ey[mv]).max())))
    # 토크 (무장 주행 구간 시각 범위, DAMPING 제외)
    t0, t1 = t[mv][0], t[mv][-1]
    if D["tau"]:
        T = np.array(D["tau"])
        T = T[(T[:, 0] >= t0) & (T[:, 0] <= t1) & (T[:, 1] != 1)]      # WheelTorqueCommand.MODE_DAMPING = 1
        if len(T) > 1:
            out.update(tau_max=float(np.abs(T[:, 2:6]).max()), dtau=float(np.sqrt(np.mean(np.diff(T[:, 2:6], axis=0) ** 2))))
    # 슬립: 바퀴 기구학 몸체 속도 − SLAM 중심 속도 (±0.1 s 중앙 차분, 몸체 축)
    if D["wheel"]:
        W = np.array(D["wheel"])
        q = r_w / 4.0
        wv = np.stack([q * W[:, 1:5].sum(1), q * (-W[:, 1] + W[:, 2] + W[:, 3] - W[:, 4])], 1)
        ts, h = P[:, 0], 0.1
        sv = []
        for arr in (xc, yc):
            sv.append((np.interp(ts + h, ts, arr) - np.interp(ts - h, ts, arr)) / (2 * h))
        c, s = np.cos(P[:, 3]), np.sin(P[:, 3])
        bx, by = c * sv[0] + s * sv[1], -s * sv[0] + c * sv[1]
        g = (ts >= t0 + h) & (ts <= t1 - h)
        if g.sum() > 5:
            dx = np.interp(ts[g], W[:, 0], wv[:, 0]) - bx[g]
            dy = np.interp(ts[g], W[:, 0], wv[:, 1]) - by[g]
            out.update(slip_x=float(np.sqrt(np.mean(dx ** 2))), slip_y=float(np.sqrt(np.mean(dy ** 2))))
    return out


COLS = [("dur", "주행 s", "{:.1f}"), ("v_ref", "v_r", "{:.2f}"), ("rms", "RMS mm", "{:.1f}", 1e3), ("max", "최대 mm", "{:.1f}", 1e3),
        ("et", "e_t RMS", "{:.1f}", 1e3), ("et_mean", "e_t 평균", "{:+.1f}", 1e3), ("ec", "e_c RMS", "{:.1f}", 1e3),
        ("ec_mean", "e_c 평균", "{:+.1f}", 1e3), ("cross", "교차 창", "{:.1f}", 1e3), ("rest", "나머지", "{:.1f}", 1e3),
        ("yaw", "요 RMS°", "{:.2f}"), ("yaw_max", "요 최대°", "{:.1f}"), ("tau_max", "|τ| 최대", "{:.3f}"),
        ("dtau", "Δτ RMS", "{:.4f}"), ("slip_x", "슬립 x", "{:.3f}"), ("slip_y", "슬립 y", "{:.3f}")]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--csv", default="", help="표를 CSV 로도 (발표용)")
    a = ap.parse_args()
    plant = yaml.safe_load(open(os.path.join(CFG, "torque_plant.yaml")))
    est = yaml.safe_load(open(os.path.join(CFG, "torque_estimator.yaml")))["estimator"]
    off = (float(est.get("imu_offset_x", 0.0)), float(est.get("imu_offset_y", 0.0)))
    geo = plant["geometry"]
    r_w, k = geo["wheel_radius_m"], 0.5 * (geo["wheel_base_m"] + geo["robot_width_m"])
    print(f"SLAM 기준 (IMU 점 → 중심 {off[0]:.3f}, {off[1]:.3f} m). 단위: mm·°·N·m·m/s. 슬립 = 바퀴 기구학 − SLAM 속도 RMS")
    head = ["bag", "판"] + [c[1] for c in COLS]
    print(" | ".join(head))
    table = []
    for bag in a.bags:
        bag = bag.rstrip("/")
        name = os.path.relpath(bag, os.path.join(bag, "..", ".."))
        try:
            d = analyse(bag, off, r_w, k)
        except Exception as ex:
            print(f"{name} | {run_label(bag)} | 건너뜀: {ex}")
            continue
        cells = []
        for c in COLS:
            v = d.get(c[0], float("nan"))
            cells.append(c[2].format(v * (c[3] if len(c) > 3 else 1)) if v == v else "—")
        print(" | ".join([name, run_label(bag)] + cells))
        table.append([name, run_label(bag)] + cells)
    if a.csv and table:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(head)
            w.writerows(table)
        print(f"CSV: {a.csv}")


if __name__ == "__main__":
    main()
