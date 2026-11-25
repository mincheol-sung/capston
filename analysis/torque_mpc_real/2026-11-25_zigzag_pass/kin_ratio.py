# 바퀴로 본 몸체 속도 v_w = pinv(Fᵀ)ω 와 SLAM 속도 v_s 의 성분별 비 (원점 지나는 최소제곱 v_s ≈ k·v_w), 바닥 토크 비율 r < 0.8 구간만
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze as A, slip_check as SC
def run(b):
    raw = A.read(b)
    import yaml
    prm = next(iter(yaml.safe_load(open(os.path.join(b, "run_info", "params", "figure8_reference_node.yaml"))).values()))["ros__parameters"]
    z = A.Zigzag(n=prm["zigzag_n"], length=prm["zigzag_length"], angle_deg=prm["zigzag_angle_deg"], accel=prm["accel"], v_max=1.55,
                 profile=prm["zigzag_profile"], rot_s=prm["zigzag_rot_s"], dwell_s=prm["zigzag_dwell_s"], align_s=prm["align_s"],
                 reverse=prm["zigzag_reverse"], corner_mode=prm["zigzag_corner"], corner_speed=prm["zigzag_corner_speed"], turn_m=prm["zigzag_turn_m"])
    pr = np.array([[t, m.data[1]] for t, m in raw["/figure8/progress"]])
    w0 = np.interp(0.0, pr[:, 1], pr[:, 0]); w1 = np.interp(min(z.T_path, pr[:, 1].max()), pr[:, 1], pr[:, 0])
    S = np.array([[t, *A.slam_base(m)] for t, m in raw["/tracked_pose"] if w0 - 0.5 <= t <= w1 + 0.5])
    W = np.array([[t, *m.velocity] for t, m in raw["/wheel_states"] if w0 - 0.5 <= t <= w1 + 0.5])
    Tq = np.array([[t, *m.torque] for t, m in raw["/torque_output/cmd_raw"] if w0 - 0.5 <= t <= w1 + 0.5])
    om = np.stack([np.interp(Tq[:, 0], W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    g = (np.abs((Tq[:, 1:] - (A.FC * np.tanh(om / 0.5) + A.FV * om)) @ A.P_GROUND.T) / A.C["tau_bar"]).max(1)
    Fi = np.linalg.pinv(A.C["F"].T); h = 0.1; yawu = np.unwrap(S[:, 3]); R = []
    for t in np.arange(w0 + h, w1 - h, 0.02):
        xa, xb = (np.array([np.interp(t + s, S[:, 0], S[:, k]) for k in (1, 2)]) for s in (-h, h))
        ya, yb, yaw = (np.interp(t + s, S[:, 0], yawu) for s in (-h, h, 0.0))
        vw = (xb - xa) / (2 * h); c, s_ = np.cos(yaw), np.sin(yaw)
        vs = [c * vw[0] + s_ * vw[1], -s_ * vw[0] + c * vw[1], (yb - ya) / (2 * h)]
        m = (W[:, 0] >= t - h) & (W[:, 0] <= t + h)
        R.append([g[(Tq[:, 0] >= t - h) & (Tq[:, 0] <= t + h)].max(), *vs, *(Fi @ W[m, 1:].mean(0))])
    R = np.array(R); lo = R[:, 0] < 0.8
    k = [float(np.sum(R[lo, 1 + i] * R[lo, 4 + i]) / np.sum(R[lo, 4 + i] ** 2)) for i in range(3)]
    res = [float(np.sqrt(np.mean((R[lo, 1 + i] - k[i] * R[lo, 4 + i]) ** 2))) for i in range(3)]
    return dict(name=os.path.basename(b), k_vx=k[0], k_vy=k[1], k_w=k[2], res=res, n=int(lo.sum()),
                w_abs_max=float(np.abs(R[lo, 3]).max()), vy_abs_max=float(np.abs(R[lo, 2]).max()))
if __name__ == "__main__":
    for b in sys.argv[1:]:
        print(json.dumps(run(b), ensure_ascii=False))
