# 바퀴 속도로 본 몸체 속도 v_w = pinv(Fᵀ)·ω (메카넘 운동학, 무슬립 가정) 와 SLAM (/tracked_pose, 바퀴와 독립) 미분 속도의 차 = 미끄러짐 속도.
# SLAM 은 ±0.1 s 중앙 차분 (잡음 줄임), 바퀴 속도도 같은 창 평균. 분석 창은 analyze.py 와 같다 (기준 t ∈ [0, T_path], 주행만).
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze as A

def run(b):
    raw = A.read(b)
    import yaml
    prm = next(iter(yaml.safe_load(open(os.path.join(b, "run_info", "params", "figure8_reference_node.yaml"))).values()))["ros__parameters"]
    z = A.Zigzag(n=prm["zigzag_n"], length=prm["zigzag_length"], angle_deg=prm["zigzag_angle_deg"], accel=prm["accel"], v_max=1.55,
                 profile=prm["zigzag_profile"], rot_s=prm["zigzag_rot_s"], dwell_s=prm["zigzag_dwell_s"], align_s=prm["align_s"],
                 reverse=prm["zigzag_reverse"], corner_mode=prm["zigzag_corner"], corner_speed=prm["zigzag_corner_speed"], turn_m=prm["zigzag_turn_m"])
    pr = np.array([[t, m.data[1]] for t, m in raw["/figure8/progress"]])
    w0 = np.interp(0.0, pr[:, 1], pr[:, 0]); w1 = np.interp(min(z.T_path, pr[:, 1].max()), pr[:, 1], pr[:, 0])
    S = np.array([[t, *A.slam_base(m)] for t, m in raw["/tracked_pose"] if w0 - 0.5 <= t <= w1 + 0.5])   # IMU 점 → base_footprint
    W = np.array([[t, *m.velocity] for t, m in raw["/wheel_states"] if w0 - 0.5 <= t <= w1 + 0.5])
    Fi = np.linalg.pinv(A.C["F"].T)                                     # ω = Fᵀ v → v = pinv(Fᵀ) ω
    tt = np.arange(w0, w1, 0.02); h = 0.1
    out = []
    for t in tt:
        xa = [np.interp(t - h, S[:, 0], S[:, k]) for k in (1, 2)]; xb = [np.interp(t + h, S[:, 0], S[:, k]) for k in (1, 2)]
        yaw = np.interp(t, S[:, 0], np.unwrap(S[:, 3])); ya, yb = (np.interp(t + s, S[:, 0], np.unwrap(S[:, 3])) for s in (-h, h))
        vw = np.array([(xb[0] - xa[0]) / (2 * h), (xb[1] - xa[1]) / (2 * h)])
        c, s = np.cos(yaw), np.sin(yaw)
        vs = np.array([c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1], (yb - ya) / (2 * h)])
        m = (W[:, 0] >= t - h) & (W[:, 0] <= t + h)
        vwh = Fi @ W[m, 1:].mean(0)
        out.append([vs[0], vs[1], vs[2], *vwh])
    o = np.array(out)
    Tq = np.array([[t, *m.torque] for t, m in raw["/torque_output/cmd_raw"] if w0 - 0.5 <= t <= w1 + 0.5])
    om = np.stack([np.interp(Tq[:, 0], W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    g = np.abs((Tq[:, 1:] - (A.FC * np.tanh(om / 0.5) + A.FV * om)) @ A.P_GROUND.T) / A.C["tau_bar"]
    gmax = g.max(1)
    rr = np.array([gmax[(Tq[:, 0] >= t - h) & (Tq[:, 0] <= t + h)].max() for t in tt])     # 같은 ±0.1 s 창의 최대 비율
    dv = np.hypot(o[:, 3] - o[:, 0], o[:, 4] - o[:, 1]); dw = o[:, 5] - o[:, 2]
    sp = np.hypot(o[:, 0], o[:, 1])
    bins = {"r<0.8": rr < 0.8, "0.8≤r<1.25": (rr >= 0.8) & (rr < 1.25), "r≥1.25": rr >= 1.25}
    cond = {k: (float(np.sqrt(np.mean(dv[m] ** 2))) if m.any() else None, float(np.sqrt(np.mean(dw[m] ** 2))) if m.any() else None,
                float(np.mean(m)), float(sp[m].mean()) if m.any() else None) for k, m in bins.items()}
    return dict(name=os.path.basename(b), cond=cond, slip_v_rms=float(np.sqrt(np.mean(dv ** 2))), slip_v_p95=float(np.percentile(dv, 95)), slip_v_max=float(dv.max()),
                slip_w_rms=float(np.sqrt(np.mean(dw ** 2))), slip_w_max=float(np.abs(dw).max()), v_mean=float(sp.mean()),
                bias_vx=float(np.mean(o[:, 3] - o[:, 0])), ratio_vx=float(np.sum(o[:, 3] * o[:, 0]) / np.sum(o[:, 0] ** 2)))

if __name__ == "__main__":
    for b in sys.argv[1:]:
        print(json.dumps(run(b), ensure_ascii=False))
