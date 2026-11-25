# 슬립률 λ_i = (ω_i − ω_nom,i)·sgn(ω_nom,i) / max(|ω_i|, |ω_nom,i|, ω_ε)  (slip_logger.slip_ratio 와 같은 식, ω_ε 2.5 rad/s)
#   ω_nom = Fᵀ v,  v = [v_x, v_y] SLAM (/tracked_pose 를 base_footprint 로 옮겨 header.stamp 로 ±0.1 s 중앙 차분) + ω_z IMU 자이로 (/imu/base_link)
#   — slip_logger 는 v = 추정기 v̂. SLAM 은 수신이 stamp 보다 0.11 s 늦어 stamp 로 맞춘다. 바퀴도 header.stamp.
#   몸체 속력 < 0.05 m/s 는 뺌 (slip_logger min_speed). 같은 ±0.1 s 창의 바닥 토크 비율 r 로 나눔.
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze as A
from amr_mpc.nodes.slip_logger import slip_ratio

def imu(b):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name == "/imu/base_link"}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(ty))); out = []
    while r.has_next():
        n, d, t = r.read_next(); m = deserialize_message(d, ty[n])
        out.append([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m.angular_velocity.z])
    return np.array(out)


def run(b):
    raw = A.read(b)
    import yaml
    prm = next(iter(yaml.safe_load(open(os.path.join(b, "run_info", "params", "figure8_reference_node.yaml"))).values()))["ros__parameters"]
    z = A.Zigzag(n=prm["zigzag_n"], length=prm["zigzag_length"], angle_deg=prm["zigzag_angle_deg"], accel=prm["accel"], v_max=1.55,
                 profile=prm["zigzag_profile"], rot_s=prm["zigzag_rot_s"], dwell_s=prm["zigzag_dwell_s"], align_s=prm["align_s"],
                 reverse=prm["zigzag_reverse"], corner_mode=prm["zigzag_corner"], corner_speed=prm["zigzag_corner_speed"], turn_m=prm["zigzag_turn_m"])
    pr = np.array([[t, m.data[1]] for t, m in raw["/figure8/progress"]])
    w0 = np.interp(0.0, pr[:, 1], pr[:, 0]); w1 = np.interp(min(z.T_path, pr[:, 1].max()), pr[:, 1], pr[:, 0])
    st = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    S = np.array([[st(m), *A.slam_base(m)] for t, m in raw["/tracked_pose"] if w0 - 0.7 <= t <= w1 + 0.7])
    W = np.array([[st(m), *m.velocity] for t, m in raw["/wheel_states"] if w0 - 0.5 <= t <= w1 + 0.5])
    G = imu(b)
    Tq = np.array([[t, *m.torque] for t, m in raw["/torque_output/cmd_raw"] if w0 - 0.5 <= t <= w1 + 0.5])
    om = np.stack([np.interp(Tq[:, 0], W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    g = (np.abs((Tq[:, 1:] - (A.FC * np.tanh(om / 0.5) + A.FV * om)) @ A.P_GROUND.T) / A.C["tau_bar"]).max(1)
    LG = np.array([[t, *m.data[1:5]] for t, m in raw["/slip_logger/slip"] if w0 <= t <= w1])
    h = 0.1; yawu = np.unwrap(S[:, 3]); rows = []
    for t in np.arange(w0 + h, w1 - h, 0.02):
        xa, xb = (np.array([np.interp(t + s, S[:, 0], S[:, k]) for k in (1, 2)]) for s in (-h, h))
        ya, yb, yaw = (np.interp(t + s, S[:, 0], yawu) for s in (-h, h, 0.0))
        vw = (xb - xa) / (2 * h); c, s_ = np.cos(yaw), np.sin(yaw)
        mi = (G[:, 0] >= t - h) & (G[:, 0] <= t + h)
        v = np.array([c * vw[0] + s_ * vw[1], -s_ * vw[0] + c * vw[1], G[mi, 1].mean()])
        if np.hypot(v[0], v[1]) < 0.05:
            continue
        m = (W[:, 0] >= t - h) & (W[:, 0] <= t + h)
        wn = A.C["F"].T @ v
        lam = slip_ratio(W[m, 1:].mean(0), wn, 2.5)
        sv = A.C["r"] * (W[m, 1:].mean(0) - wn)                          # 바퀴 둘레 미끄럼 속도 r(ω − ω_nom) [m/s]
        r = g[(Tq[:, 0] >= t - h) & (Tq[:, 0] <= t + h)].max()
        lg = np.nanmean(LG[(LG[:, 0] >= t - h) & (LG[:, 0] <= t + h), 1:], 0)
        rows.append([r, *lam, *lg, *sv, *np.abs(wn)])
    X = np.array(rows); r = X[:, 0]; L = X[:, 1:5]; Lg = X[:, 5:9]; SV = X[:, 9:13]; WN = X[:, 13:17]
    def stat(M, sel):
        a = np.abs(M[sel]); a = a[np.isfinite(a)]
        return None if not len(a) else dict(n=int(sel.sum()), med=float(np.median(a)), rms=float(np.sqrt(np.mean(a ** 2))),
                                            gt01=float(np.mean(a > 0.1)), gt02=float(np.mean(a > 0.2)), p95=float(np.percentile(a, 95)))
    bins = {"all": np.ones(len(r), bool), "r<0.8": r < 0.8, "0.8~1.25": (r >= 0.8) & (r < 1.25), "r≥1.25": r >= 1.25}
    sv_stat = lambda sel: None if not sel.any() else dict(rms=float(np.sqrt(np.mean(SV[sel] ** 2))), p95=float(np.percentile(np.abs(SV[sel]), 95)),
                                                         gt005=float(np.mean(np.abs(SV[sel]).max(1) > 0.05)), gt01=float(np.mean(np.abs(SV[sel]).max(1) > 0.1)))
    near0 = float(np.mean((WN < 2.5).any(1)))                            # 한 바퀴라도 |ω_nom| < ω_ε (λ 분모가 ω_ε 로 막히는 구간)
    return dict(name=os.path.basename(b), slipvel={k: sv_stat(s_) for k, s_ in bins.items()}, near_zero_wheel=near0, slam={k: stat(L, s) for k, s in bins.items()}, logger={k: stat(Lg, s) for k, s in bins.items()},
                per_wheel_rms=[float(np.sqrt(np.nanmean(L[:, i] ** 2))) for i in range(4)],
                mean_signed=[float(np.nanmean(L[:, i])) for i in range(4)])

if __name__ == "__main__":
    for b in sys.argv[1:]:
        print(json.dumps(run(b), ensure_ascii=False))
