# 2026-11-25 지그재그 pass (cubic spiral turn, turn_m 0.5) — MPC vs PP·adpp 추종·슬립·토크.
# 분석 창 = 기준 시각 t (/figure8/progress data[1]) 가 [0, T_total] 인 동안 (주행 + 끝 정렬). 그 뒤 (유지·로봇 듦) 는 뺀다.
#   python3 analysis/torque_mpc_real/2026-11-25_zigzag_pass/analyze.py bags/2026-11-25/140937 ...
import os, sys, math, json, numpy as np, yaml
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "src", "amr_basic_stack", "controller", "mpc"))
from amr_mpc.core.zigzag import Zigzag
from amr_mpc.core import fig8_limits
CFG = os.path.join(HERE, "..", "..", "..", "src", "amr_basic_stack", "controller", "mpc", "config")
C = fig8_limits.plant_constants(CFG)                                   # 오늘 yaml (μ̂ 0.33, c_s 0.8, 바퀴별 μ 끔)
_PL = yaml.safe_load(open(os.path.join(CFG, "torque_plant.yaml")))
JW = float(_PL["wheel"]["inertia_J_w_kgm2"])
FC = np.array(_PL["friction"]["coulomb_Nm"], float); FV = np.array(_PL["friction"]["viscous_Nms_per_rad"], float)
# 바닥 토크 사상 P = I − J_w Fᵀ M_eff⁻¹ F (torque_mpc.cpp ground_map), 구동계 마찰 f(ω) = c·tanh(ω/0.5) + v·ω (drivetrain_friction)
P_GROUND = np.eye(4) - JW * C["F"].T @ np.diag([1 / C["me"], 1 / C["me"], 1 / C["Ie"]]) @ C["F"]
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ("/figure8/progress", "/mpc/armed", "/state_estimator/odom", "/tracked_pose", "/figure8/path", "/torque_mpc/status",
          "/pure_pursuit/status", "/slip_logger/slip", "/torque_output/cmd_raw", "/torque_mpc/load", "/torque_mpc/cap",
          "/fault_monitor/fault", "/wheel_states")


def read(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in TOPICS}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    out = {k: [] for k in TOPICS}
    while r.has_next():
        n, d, t = r.read_next(); out[n].append((t * 1e-9, deserialize_message(d, types[n])))
    return out


def seg_dist(P, path):
    """점 P (n,2) 에서 꺾은선 path (m,2) 까지 최단 거리 (선분 사영)."""
    A, B = path[:-1], path[1:]; AB = B - A; L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    AP = P[:, None, :] - A[None]
    s = np.clip((AP * AB[None]).sum(2) / L2[None], 0.0, 1.0)
    Q = A[None] + s[..., None] * AB[None]
    return np.sqrt(((P[:, None, :] - Q) ** 2).sum(2)).min(1)


R_IMU = np.array([0.109, 0.017])     # /tracked_pose = Cartographer tracking_frame (카메라 IMU) — base_footprint 기준 위치 (추정기 기동 로그)


def slam_base(m):
    """/tracked_pose → base_footprint [x, y, ψ]: p_base = p_slam − R(ψ)·r_IMU."""
    q = m.pose.orientation; psi = 2.0 * math.atan2(q.z, q.w)
    c, s = math.cos(psi), math.sin(psi)
    return (m.pose.position.x - (c * R_IMU[0] - s * R_IMU[1]), m.pose.position.y - (s * R_IMU[0] + c * R_IMU[1]), psi)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def analyze(b):
    name = os.path.basename(b.rstrip("/"))
    prm = yaml.safe_load(open(os.path.join(b, "run_info", "params", "figure8_reference_node.yaml")))
    prm = next(iter(prm.values()))["ros__parameters"]
    args = open(os.path.join(b, "run_info", "args.txt")).read()
    ctrl = "adpp" if "controller:=adpp" in args else "pp" if "controller:=pp" in args else "mpc"
    la = [a.split(":=")[1] for a in args.split() if a.startswith("pp_lookahead_m")]
    z = Zigzag(n=prm["zigzag_n"], length=prm["zigzag_length"], angle_deg=prm["zigzag_angle_deg"], accel=prm["accel"],
               v_max=prm["speed"] if prm["speed"] > 0 else 1.55, profile=prm["zigzag_profile"], rot_s=prm["zigzag_rot_s"],
               dwell_s=prm["zigzag_dwell_s"], yaw_mode=prm["yaw_mode"], align_s=prm["align_s"], reverse=prm["zigzag_reverse"],
               corner_mode=prm["zigzag_corner"], corner_speed=prm["zigzag_corner_speed"], turn_m=prm["zigzag_turn_m"])
    raw = read(b)
    pr = np.array([[t, m.data[1]] for t, m in raw["/figure8/progress"]])
    if not len(pr) or pr[:, 1].max() < 0.5:
        return dict(name=name, ctrl=ctrl, note="주행 안 함")
    w0 = np.interp(0.0, pr[:, 1], pr[:, 0])
    reached = pr[:, 1].max() >= z.T_total
    w1 = np.interp(z.T_total, pr[:, 1], pr[:, 0]) if reached else pr[pr[:, 1].argmax(), 0]
    tref = lambda tw: np.interp(tw, pr[:, 0], pr[:, 1])
    turn_of = lambda tr: np.array([any(s0 <= x < s0 + T for s0, T, k, _ in z.seg if k == "turn") for x in tr])
    drive = lambda tr: tr <= z.T_path
    st_name = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    St = np.array([[t, *m.data[1:4]] for t, m in raw[st_name] if w0 <= t <= w1])
    trS = tref(St[:, 0]); dS = drive(trS); tS = turn_of(trS)
    exy = np.hypot(St[:, 1], St[:, 2]); eth = np.degrees(np.abs(np.arctan2(np.sin(St[:, 3]), np.cos(St[:, 3]))))
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in [m for t, m in raw["/figure8/path"] if t <= w1][-1].poses])
    Od = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if w0 <= t <= w1])
    Sl = np.array([[t, *slam_base(m)] for t, m in raw["/tracked_pose"] if w0 <= t <= w1])
    trO = tref(Od[:, 0]); dO = drive(trO); tO = turn_of(trO)
    dmin = seg_dist(Od[:, 1:], path); dslam = seg_dist(Sl[:, 1:3], path); trL = tref(Sl[:, 0])
    # 슬립 λ (slip_logger: 추정기 v̂ 기준, 잡음 바닥 ~0.1)
    lam = np.array([[t, *m.data[1:5]] for t, m in raw["/slip_logger/slip"] if w0 <= t <= w1])
    trl = tref(lam[:, 0]); lam_d = lam[drive(trl)][:, 1:]
    lam_abs = np.nanmax(np.abs(lam_d), 1)
    # 토크 (torque_output 출력 = 모터 축, 두 제어기 같은 자리) vs 마찰 상한 τ̄ (MPC 판의 /torque_mpc/load 스텝 0)
    Tq = np.array([[t, *m.torque] for t, m in raw["/torque_output/cmd_raw"] if w0 <= t <= w1])
    tau_bar = np.array([m.data[5:9] for t, m in raw["/torque_mpc/load"]]).mean(0) if raw["/torque_mpc/load"] else None
    cap = np.array([[t, *m.data[2:6], *m.data[6:14]] for t, m in raw["/torque_mpc/cap"] if w0 <= t <= w1]) if raw["/torque_mpc/cap"] else None
    # 바닥 토크 / 마찰 상한: |(P (τ − f(ω)))_i| / τ̄_i — MPC (cap_on ground) 의 제약식 그대로, PP 토크에도 같은 식
    Wv = np.array([[t, *m.velocity] for t, m in raw["/wheel_states"] if w0 - 0.1 <= t <= w1 + 0.1])
    om = np.stack([np.interp(Tq[:, 0], Wv[:, 0], Wv[:, 1 + i]) for i in range(4)], 1)
    f = FC * np.tanh(om / 0.5) + FV * om
    tg = (Tq[:, 1:] - f) @ P_GROUND.T
    ratio = np.abs(tg) / C["tau_bar"]
    rmax = ratio.max(1); trq = tref(Tq[:, 0]); dq = drive(trq)
    fault = [(round(t - w0, 2), getattr(m, "data", m)) for t, m in raw["/fault_monitor/fault"] if t >= w0 - 1 and t <= w1 + 1]
    # 분석 창 뒤 (유지·듦) 의 위치 변화 — 창으로 뺀 부분이 있었는지 보이려고
    Oa = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y] for t, m in raw["/state_estimator/odom"] if t > w1])
    after = float(np.hypot(*(Oa[:, 1:] - Od[-1, 1:]).T).max()) if len(Oa) else 0.0
    return dict(name=name, ctrl=ctrl, la="/".join(la) if la else ("0.30" if ctrl == "pp" else "0.15~0.60" if ctrl == "adpp" else "-"),
                n=z.n, vc=z.vc, T_path=z.T_path, T_total=z.T_total, reached=bool(reached), dur=float(w1 - w0),
                e_rms=rms(exy[dS]), e_max=float(exy[dS].max()), e_turn_rms=rms(exy[dS & tS]), e_str_rms=rms(exy[dS & ~tS]),
                th_rms=rms(eth[dS]), th_max=float(eth[dS].max()),
                d_rms=rms(dmin[dO]), d_max=float(dmin[dO].max()), d_turn_rms=rms(dmin[dO & tO]), d_str_rms=rms(dmin[dO & ~tO]),
                dslam_rms=rms(dslam[drive(trL)]), dslam_max=float(dslam[drive(trL)].max()),
                lam_rms=rms(lam_d[np.isfinite(lam_d)]), lam_gt01=float(np.mean(lam_abs > 0.1)), lam_gt03=float(np.mean(lam_abs > 0.3)),
                tau_bar=None if tau_bar is None else tau_bar.tolist(),
                tq_max=np.abs(Tq[:, 1:]).max(0).tolist(), tq_absmax=float(np.abs(Tq[:, 1:]).max()),
                cap_hi_mean=None if cap is None else np.abs(cap[:, 9:13]).mean(0).tolist(),
                cap_shift_max=None if cap is None else np.abs(cap[:, 1:5]).max(0).tolist(),
                at_box=None if cap is None else float(np.mean(np.any(np.isclose(np.interp(cap[:, 0], Tq[:, 0], Tq[:, 1])[:, None] * 0 + cap[:, 5:13], 0) & False, 1))),
                tau_bar_static=C["tau_bar"].tolist(), g_ratio_max=float(rmax[dq].max()), g_ratio_p95=float(np.percentile(rmax[dq], 95)),
                g_over1=float(np.mean(rmax[dq] > 1.0)), g_over15=float(np.mean(rmax[dq] > 1.5)),
                g_over1_turn=float(np.mean(rmax[dq & turn_of(trq)] > 1.0)), g_over1_str=float(np.mean(rmax[dq & ~turn_of(trq)] > 1.0)),
                fault=fault, after_window_move=after,
                _ts=dict(t=(trq[dq]).tolist(), r=rmax[dq].tolist(), te=trS[dS].tolist(), e=exy[dS].tolist()),
                _traj=dict(path=path.tolist(), od=Od[:, 1:].tolist(), slam=Sl[:, 1:3].tolist(), Oa=Oa[:, 1:].tolist() if len(Oa) else []))


if __name__ == "__main__":
    res = []
    for b in sys.argv[1:]:
        try:
            r = analyze(b)
        except Exception as e:
            r = dict(name=os.path.basename(b.rstrip("/")), note=f"오류 {type(e).__name__}: {e}")
        res.append(r)
        print(json.dumps({k: v for k, v in r.items() if not k.startswith("_")}, ensure_ascii=False, default=float))
    json.dump(res, open(os.path.join(HERE, "results.json"), "w"), ensure_ascii=False, default=float)
