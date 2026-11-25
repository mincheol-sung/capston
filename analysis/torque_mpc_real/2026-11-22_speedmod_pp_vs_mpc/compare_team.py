#!/usr/bin/python3
"""새벽 팀원 판 (A 002822·003041·003451, C 002707·002931·003146) 과 밤 판 (A 212837, C 212703) 을 analyze.py 와 같은 잣대로.

  cd ~/capston/capston && source install/setup.bash
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-22_speedmod_pp_vs_mpc/compare_team.py

팀원 C 3 판만 인자가 셋 더 붙었다: disturbance:=true slip_guard:=true velocity_source:=wheels (run_info/args.txt).
추가 열: τ̄ (status 8 중앙값), d̂ (status 14~19) 크기, 슬립 가드 g < 1 시간 %·최소 g, 기준 속력 프로파일이 밤 판과 같은지.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze as A  # noqa: E402

BAGS = "bags/2026-11-22"
GROUPS = [("A 새벽", ["002822", "003041", "003451"]), ("C 새벽 (+dist·guard·wheels)", ["002707", "002931", "003146"]),
          ("A 밤", ["212837"]), ("C 밤", ["212703"])]
ROWS = [  # (표 이름, 키, 배율, 자릿수)
    (r"위치 \|e\| RMS [cm]", "exy_rms", 100, 1), (r"위치 \|e\| 최대 [cm]", "exy_max", 100, 1),
    (r"SLAM 기준 \|e\| RMS [cm]", "exy_slam_rms", 100, 1), ("경로 거리 RMS [cm]", "dpath_rms", 100, 1),
    ("along 평균 (+ 앞섬) [cm]", "along_mean", 100, 1), ("cross RMS [cm]", "cross_rms", 100, 1),
    ("원 위 안쪽 치우침 [cm]", "inside_mean_turn", 100, 1),
    ("헤딩 RMS [°]", "eth_rms", 1, 1), ("헤딩 최대 [°]", "eth_max", 1, 1), ("교차점 헤딩 RMS [°]", "eth_rms_near", 1, 1),
    ("ω 오차 RMS [rad/s]", "w_err_rms", 1, 3),
    ("속력 오차 RMS [m/s]", "v_err_rms", 1, 3), ("속력 오차 평균 [m/s]", "v_err_mean", 1, 3), ("속력 이득", "v_gain", 1, 2),
    ("가속도 오차 RMS IMU [m/s²]", "a_imu_err_rms", 1, 3), ("가속도 이득 IMU", "a_imu_gain", 1, 2),
    ("가속도 지연 IMU [ms]", "a_imu_lag", 1000, 0),
    ("v_y RMS [m/s]", "vy_rms", 1, 3), ("v_y 떨림 RMS [m/s]", "jit_vy", 1, 3), ("ω_z 떨림 RMS [rad/s]", "jit_w", 1, 3),
    ("큰 슬립 %", "big_pct", 1, 1), ("출발 2 s 뒤 큰 슬립 %", "big_late", 1, 1), ("0.5 s 평균 뒤 큰 슬립 %", "big_avg_pct", 1, 1),
    ("λ RMS", "lam_rms", 1, 3), ("헛돎 몫", "spin_share", 100, 0),
    (r"\|τ\| 95 % [N·m]", "tau_p95", 1, 3), ("Δτ RMS [N·m/주기]", "dtau_rms", 1, 4),
]


def extra(bag, s_ref):
    raw = A.read(bag)
    arm = [(t, bool(m.data)) for t, m in raw["/mpc/armed"]]
    s0 = next(t for t, on in arm if on)
    out = {}
    St = np.array([m.data for t, m in raw["/torque_mpc/status"] if m.data[0] >= s0]) if raw["/torque_mpc/status"] else None
    if St is not None:
        out["τ̄"] = f"{np.median(St[:, 8]):.3f}"
        d = St[:, 14:20]
        out["|d̂| 중앙 (상태 6)"] = " ".join(f"{v:.3f}" for v in np.median(np.abs(d), axis=0))
    return out


def guard(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray, Bool
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    names = {t.name for t in r.get_all_topics_and_types()}
    if "/torque_output/slip_guard" not in names:
        return None
    r.set_filter(rosbag2_py.StorageFilter(topics=["/torque_output/slip_guard", "/mpc/armed"]))
    G, arm = [], []
    while r.has_next():
        n, d, t = r.read_next()
        if n == "/mpc/armed":
            arm.append((t * 1e-9, deserialize_message(d, Bool).data))
        else:
            G.append(deserialize_message(d, Float64MultiArray).data)
    s0 = next(t for t, on in arm if on)
    s1 = next((t for t, on in arm if not on and t > s0), 1e18)
    G = np.array([g for g in G if s0 <= g[0] <= s1])
    g = G[:, 5:9]
    return f"g<1 {100 * np.mean(np.any(g < 0.999, axis=1)):.1f} %, 최소 g {g.min():.2f}, 바퀴별 g<1 % " + \
        " ".join(f"{100 * np.mean(g[:, i] < 0.999):.1f}" for i in range(4))


def main():
    res, vrefs = {}, {}
    for _, bags in GROUPS:
        for b in bags:
            r, s = A.analyze(os.path.join(BAGS, b))
            res[b] = r
            vrefs[b] = s["vref"]
    # 기준 속력 프로파일 같은가 (주행 시작 기준 같은 격자)
    base = vrefs["212703"]
    print("기준 속력 프로파일 |Δv_ref| 최대 (212703 대비, 짧은 쪽 길이까지):",
          {b: round(float(np.max(np.abs(v[:min(len(v), len(base))] - base[:min(len(v), len(base))]))), 4) for b, v in vrefs.items()})
    hdr = "| 지표 | " + " | ".join(f"{g} ({'·'.join(bs)})" for g, bs in GROUPS) + " |"
    print(hdr); print("|---|" + "---|" * len(GROUPS))
    for name, key, sc, nd in ROWS:
        cells = []
        for _, bs in GROUPS:
            v = [res[b][key] * sc for b in bs]
            cells.append(f"{min(v):.{nd}f}" if len(v) == 1 else f"{min(v):.{nd}f} – {max(v):.{nd}f}")
        print(f"| {name} | " + " | ".join(cells) + " |")
    print()
    for _, bs in GROUPS:
        for b in bs:
            e = extra(os.path.join(BAGS, b), None)
            gd = guard(os.path.join(BAGS, b))
            print(b, res[b]["ctrl"], f"주행 {res[b]['move_s']:.1f}s", e, ("슬립가드 " + gd) if gd else "")


if __name__ == "__main__":
    main()
