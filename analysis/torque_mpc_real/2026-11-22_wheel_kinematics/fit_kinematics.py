#!/usr/bin/python3
"""바퀴 기구학 유효값 맞추기 — SLAM 포즈 기준 (2026-11-22).

ω_i = c_x,i v_x + c_y,i v_y + c_ω,i ω  (몸체, 로봇 중심). 공칭 (torque_plant.yaml r 0.04, k 0.1932): c_x = 25, c_y = ∓25, c_ω = ∓4.83
  FL (+, −, −), FR (+, +, +), RL (+, +, −), RR (+, −, +)
기준: SLAM 중심 (c = p − R(ψ) r_imu) 의 중앙 차분 속도 (앞뒤 포즈, 0.2 s → 몸체) 와 SLAM yaw 중앙 차분 ω. 바퀴 ω 는 같은 0.2 s 창 평균.
정속 표본만: |가속| < 0.3 m/s², |각가속| < 0.6 rad/s², 움직임 (|v| > 0.1 또는 |ω| > 0.3). v_y 는 11-20 뒤 판에서 거의 0 → c_y 는 공칭으로 두고 c_x, c_ω 만.
큰 잔차 (3·MAD 밖) 는 되풀이해 빼서 슬립 표본에 덜 끌리게.
  nice -n 19 ionice -c3 /usr/bin/python3 analysis/torque_mpc_real/2026-11-22_wheel_kinematics/fit_kinematics.py
"""
import glob, json, math, os, re
import numpy as np, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

R_IMU = np.array([0.1093, 0.017]); RW, K = 0.04, 0.1932
SX = np.array([1, 1, 1, 1.]); SY = np.array([-1, 1, 1, -1.]); SW = np.array([-1, 1, -1, 1.])
HERE = os.path.dirname(os.path.abspath(__file__))
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
WH = ["FL", "FR", "RL", "RR"]


def bag_samples(d):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=d, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/wheel_states", "/state_estimator/odom"]))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    P, W, E = [], [], []
    while r.has_next():
        n, dd, _ = r.read_next(); m = deserialize_message(dd, ty[n])
        if n == "/tracked_pose":
            q = m.pose.orientation
            P.append((st(m.header), m.pose.position.x, m.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        elif n == "/wheel_states":
            if all(m.fresh):
                W.append((st(m.header), *m.velocity))
        else:
            E.append((st(m.header), m.twist.twist.angular.z))
    if len(P) < 50 or len(W) < 500:
        return None
    P = np.array(sorted(set(P))); W = np.array(W); E = np.array(E) if E else None
    psi = np.unwrap(P[:, 3]); c, s = np.cos(psi), np.sin(psi)
    C = P[:, 1:3] - np.stack([c * R_IMU[0] - s * R_IMU[1], s * R_IMU[0] + c * R_IMU[1]], 1)
    t = P[:, 0]; dt = t[2:] - t[:-2]
    ok = (dt > 0.15) & (dt < 0.3)                                  # 포즈가 끊기지 않은 곳만
    vw = (C[2:] - C[:-2]) / dt[:, None]; wz = (psi[2:] - psi[:-2]) / dt
    tm, cm, sm = t[1:-1], np.cos(psi[1:-1]), np.sin(psi[1:-1])
    vx, vy = cm * vw[:, 0] + sm * vw[:, 1], -sm * vw[:, 0] + cm * vw[:, 1]
    # 가속: 이웃 속도 차
    ax = np.gradient(np.hypot(vx, vy), tm); al = np.gradient(wz, tm)
    # 바퀴 ω: 같은 창 [t_{k-1}, t_{k+1}] 평균
    cs = np.vstack([np.zeros(4), np.cumsum(W[:, 1:5] * np.gradient(W[:, 0])[:, None], 0)])
    tW = np.r_[W[0, 0], W[:, 0]]
    def win_mean(a, b):
        return (np.array([np.interp(b, tW, cs[:, i]) - np.interp(a, tW, cs[:, i]) for i in range(4)]).T) / (b - a)[:, None]
    wm = win_mean(t[:-2], t[2:])
    inW = (t[:-2] > W[0, 0]) & (t[2:] < W[-1, 0])
    gyro = np.interp(tm, E[:, 0], E[:, 1]) if E is not None and len(E) else np.full(len(tm), np.nan)
    mov = (np.hypot(vx, vy) > 0.1) | (np.abs(wz) > 0.3)
    sel = ok & inW & mov & (np.abs(ax) < 0.3) & (np.abs(al) < 0.6)
    return np.column_stack([vx, vy, wz, wm, gyro])[sel]


def robust_fit(X, y, it=5):
    m = np.ones(len(y), bool)
    for _ in range(it):
        c, *_ = np.linalg.lstsq(X[m], y[m], rcond=None)
        r = y - X @ c; mad = np.median(np.abs(r[m] - np.median(r[m]))) * 1.4826
        m = np.abs(r) < 3 * max(mad, 1e-6)
    return c, m


if __name__ == "__main__":
    groups = {}
    for meta in sorted(glob.glob("bags/2026-11-2[0-2]/*/metadata.yaml")):
        d = os.path.dirname(meta)
        try:
            args = open(os.path.join(d, "run_info", "args.txt")).read().splitlines()[3]
        except (OSError, IndexError):
            continue
        shape = (re.findall(r"shape:=(\w+)", args) or ["?"])[0]
        try:
            S = bag_samples(d)
        except Exception as e:
            print("skip", d, e); continue
        if S is None or len(S) < 20:
            continue
        groups.setdefault(shape, []).append(S)
        print(f"{d} {shape}: 정속 표본 {len(S)}", flush=True)
    res = {}
    print("\n| 묶음 | 표본 | 바퀴 | c_x (공칭 25) → 유효 r mm | c_ω (공칭 ∓4.83) → 유효 k mm | 쓴 표본 % |")
    print("|---|---|---|---|---|---|")
    for g in list(groups) + ["전체"]:
        S = np.vstack(groups[g] if g != "전체" else [s for v in groups.values() for s in v])
        vx, vy, wz, Wm = S[:, 0], S[:, 1], S[:, 2], S[:, 3:7]
        for i in range(4):
            y = Wm[:, i] - SY[i] * vy / RW                          # v_y 몫은 공칭으로 빼고
            X = np.stack([vx, wz], 1) if g != "line" else vx[:, None]
            if g == "rotate":
                X = wz[:, None]
            c, m = robust_fit(X, y)
            cx = c[0] if g != "rotate" else np.nan
            cw = c[-1] if g != "line" else np.nan
            res.setdefault(g, {})[WH[i]] = (cx, cw)
            r_eff = 1e3 / cx if np.isfinite(cx) else np.nan
            k_eff = 1e3 * abs(cw) / cx if np.isfinite(cx) and np.isfinite(cw) else (1e3 * abs(cw) * RW if np.isfinite(cw) else np.nan)
            print(f"| {g} | {len(S)} | {WH[i]} | {cx:.2f} → {r_eff:.2f} | {cw:+.3f} → {k_eff:.1f} | {100*m.mean():.0f} |")
    # 자이로 배율 확인: SLAM yaw rate vs 추정기 ω (자이로)
    S = np.vstack([s for v in groups.values() for s in v]); ok = np.isfinite(S[:, 7]) & (np.abs(S[:, 2]) > 0.3)
    print(f"\n자이로 배율: 추정 ω / SLAM yaw rate 기울기 {np.polyfit(S[ok, 2], S[ok, 7], 1)[0]:.4f} (표본 {ok.sum()})")
    json.dump({g: {w: list(map(float, v)) for w, v in d.items()} for g, d in res.items()}, open(os.path.join(HERE, "fit.json"), "w"), indent=1)
