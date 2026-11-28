# 2026-11-27 꺾은선 통과 (가속 1.5 · 최고 1.5 · friction 상한) 의 모든 MPC 판을 실제 노드 파라미터로 묶어 속도 출처 바퀴 vs 추정기 비교
#   묶음은 run_info/params/torque_mpc_node.yaml 로 확인한 설정 (velocity_source · roller_rr · inertia_z · drive_yaw_offset_deg/slip_coef) 이 같은 판끼리.
#   지표: analyze.py (지난 비교와 같은 잣대) + summary.extra (슬립 10 Hz LPF · 회전 중 헤딩 · 출발 회전) + 2 m 직선 끝 옆 밀림 (SLAM).
#   판 수가 7 대 7 인 묶음은 Mann–Whitney U (양쪽, 정확 검정) p 값을 같이 낸다 — 판 사이 흔들림을 넘는 차이인지.
#   source install/setup.bash 뒤  /usr/bin/python3 read_bags.py <bag…> (npz 를 이 폴더에) → /usr/bin/python3 summary_all.py
import io, contextlib, json, os, sys
import numpy as np
from scipy.stats import mannwhitneyu
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import analyze as AZ
with contextlib.redirect_stdout(io.StringIO()):
    import summary as SM                     # extra() 만 쓴다 (모듈 실행 때 찍는 표는 버린다)
B = "/home/smoc/capston/capston/bags/"
GROUPS = {
    "지금 기본 · 바퀴": ["2026-11-26/183158_dy_b_r1", "2026-11-26/183407_dy_b_r2", "2026-11-26/183629_dy_b_r3", "2026-11-26/191922_s2pl_mpc_r1",
                     "2026-11-27/132307_vs_w_r1", "2026-11-27/132601_vs_w_r2", "2026-11-27/132755_vs_w_r3"],
    "지금 기본 · 추정기": ["2026-11-26/191808_s2pl_mpc_r1", "2026-11-26/201259_s2pl_mpc_r1", "2026-11-26/201604_s2pl_mpc_r2", "2026-11-26/201944_s2pl_mpc_r3",
                      "2026-11-27/132405_vs_e_r1", "2026-11-27/132659_vs_e_r2", "2026-11-27/132852_vs_e_r3"],
    "옛 설정 · 바퀴": ["2026-11-25/154715_sq2p_mpcw_r1", "2026-11-25/155359_sq2p_mpcw_r2", "2026-11-25/155804_sq2p_mpcw_r3",
                    "2026-11-26/174101_rr_off_r1", "2026-11-26/174256_rr_off_r2", "2026-11-26/174505_rr_off_r3"],
    "옛 설정 · 추정기": ["2026-11-25/162408_sq2p_mpc_r1", "2026-11-25/162830_sq2p_mpc_r2"],
}
# SLAM 중심 위치 (plot_paths.py 의 slam_read) — 날짜마다 BAGS 를 바꿔 부른다
SRC = os.path.join(HERE, "..", "2026-11-25_compare_paths", "plot_paths.py"); src = open(SRC).read()
ns = {"__file__": SRC}; exec(compile(src[:src.index("\nfor sc in SCEN:")], SRC, "exec"), ns); ns["HERE"] = HERE
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi


def straight_drift(path, lab):
    ns["BAGS"] = B + path.split("/")[0] + "/"
    T, _, _, t_arm = ns["slam_read"](lab)
    A = np.load(os.path.join(HERE, lab + ".npz")); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]
    od = A["od"]; i0 = np.searchsorted(od[:, 0], t_on); x0, y0, a0 = od[i0, 1], od[i0, 2], od[i0, 3]
    c, s = np.cos(a0), np.sin(a0)
    X = c * (T[:, 1] - x0) + s * (T[:, 2] - y0); Y = -s * (T[:, 1] - x0) + c * (T[:, 2] - y0); H = wrap(T[:, 3] - a0)
    a = (np.abs(X - 1.0) < 0.2) & (Y > -0.7) & (Y < 0.8) & (np.abs(wrap(H - np.pi / 2)) < 0.5)
    b = (np.abs(Y) < 0.2) & (X > 0.2) & (X < 1.8) & (np.abs(wrap(H - np.pi)) < 0.5)
    return -(X[a][np.argmax(Y[a])] - 1.0) * 100, -(Y[b][np.argmin(X[b])]) * 100


res = {}
for g, paths in GROUPS.items():
    res[g] = []
    for p in paths:
        lab = p.split("/")[1]
        r, _ = AZ.analyze(B + p)
        da, db = straight_drift(p, lab)
        res[g].append(dict(r, **SM.extra(lab), driftA=da, driftB=db, lab=lab))
json.dump({k: [{kk: vv for kk, vv in d.items() if isinstance(vv, (int, float, str))} for d in v] for k, v in res.items()},
          open(os.path.join(HERE, "summary_all.json"), "w"), ensure_ascii=False, indent=1)
ROWS = [("위치 오차 RMS cm", "exy_rms", 100), ("SLAM 기준 위치 RMS cm", "exy_slam_rms", 100), ("위치 오차 최대 cm", "exy_max", 100),
        ("경로까지 거리 RMS cm", "dpath_rms", 100), ("경로까지 거리 최대 cm", "dpath_max", 100),
        ("헤딩 오차 RMS °", "eth_rms", 1), ("헤딩 오차 최대 °", "eth_max", 1), ("회전 중 헤딩 오차 RMS °", "rot_eth_rms", 1), ("회전 중 헤딩 오차 최대 °", "rot_eth_max", 1),
        ("출발 회전 끝 헤딩 오차 °", "start_rot_err", 1), ("2 m 직선 A 끝 왼쪽 밀림 cm", "driftA", 1), ("2 m 직선 B 끝 왼쪽 밀림 cm", "driftB", 1),
        ("속력 오차 RMS m/s", "v_err_rms", 1), ("토크 변화 RMS mN·m/주기", "dtau_rms", 1000),
        ("슬립 > 25 % 시간 %", "slip25", 1), ("슬립 > 50 % 시간 %", "slip50", 1)]
fmt = lambda a: (f"{np.mean(a):.2f} ({np.min(a):.2f}~{np.max(a):.2f})" if abs(np.mean(a)) < 1 else f"{np.mean(a):.1f} ({np.min(a):.1f}~{np.max(a):.1f})")
G = list(GROUPS)
print("| 지표 (판 평균, 최소~최대) | " + " | ".join(f"{g} ({len(GROUPS[g])})" for g in G) + " | 지금 기본: 바퀴 vs 추정기 p |")
print("|---|" + "---|" * (len(G) + 1))
for name, key, sc in ROWS:
    vals = {g: np.array([d[key] * sc for d in res[g] if key in d and d[key] == d[key]]) for g in G}
    p = mannwhitneyu(vals[G[0]], vals[G[1]], alternative="two-sided", method="exact").pvalue if len(vals[G[0]]) and len(vals[G[1]]) else np.nan
    print(f"| {name} | " + " | ".join(fmt(vals[g]) if len(vals[g]) else "—" for g in G) + f" | {p:.3f} |")
