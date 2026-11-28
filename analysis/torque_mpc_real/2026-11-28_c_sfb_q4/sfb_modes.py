"""friction 모드에서 원 (반지름 0.5 · 1.0 m/s) 위치–슬립을 둘 다 잡을 수단이 있는지 — 두 가지를 잰다.
1) friction 상한 붙음: |τ_MPC,i| ≥ 0.95 τ̄_i 인 시간 % (τ̄ = (r/√2)·μ̂·몫_i·m g·c_s = [0.0798, 0.0799, 0.0871, 0.0872] N·m, 정적 값 — dynamic_cap 의 하중 이동은 안 넣음).
   motor_only 판은 같은 τ̄ 대비 |τ| 분포 (상한이 없었다면 MPC 가 얼마를 썼나).
2) 바퀴 공간 mode 분해: J 의 세 열 (v_x·v_y·ω_z) 과 null n = [1, 1, −1, −1] 은 서로 직교 → 단위 벡터
   F = [1,1,1,1]/2 (앞), L = [−1,1,1,−1]/2 (옆 — 접선 헤딩 원에서는 원 중심 쪽), Y = [−1,1,−1,1]/2 (요), N = [1,1,−1,−1]/2 (몸체를 안 움직이는 앞·뒤 쌍).
   e_ω = ω − ω_nom (/slip_logger: /wheel_states − ω_nom), Δτ (/torque_mpc/slip_fb 6~9) 를 각 mode 에 사영한 RMS. 창 = 최고 속력 구간 (기준 > 0.9 최고).
nice -n 19 /usr/bin/python3 sfb_modes.py → sfb_modes_out.txt"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import seq_s1 as S  # noqa: E402
import seq_s1_cause as C  # noqa: E402
from bag_runs import runs  # noqa: E402

TBAR = np.array([0.0798, 0.0799, 0.0871, 0.0872])
# 바닥 토크 P(τ − f) — torque_mpc.cpp ground_map: P = I − J_w Fᵀ M_eff⁻¹ F, F = force_map (drive_yaw 3.1° 로 앞·옆 행 회전), f = /torque_mpc/cap 2~5 열
_r, _k, _Jw, _m, _Iz, _d = 0.04, 0.1932, 8.9e-4, 4.56, 0.03, np.radians(3.1)
_mu = 4 * _Jw / _r ** 2
_F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-_k, _k, -_k, _k]], float) / _r
_F[:2] = np.array([[np.cos(_d), -np.sin(_d)], [np.sin(_d), np.cos(_d)]]) @ _F[:2]
PG = np.eye(4) - _Jw * _F.T @ np.diag([1 / (_m + _mu), 1 / (_m + _mu), 1 / (_Iz + _mu * _k ** 2)]) @ _F


def read_cap(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    have = {t.name: t.type for t in r.get_all_topics_and_types()}
    if "/torque_mpc/cap" not in have:
        return None
    r.set_filter(rosbag2_py.StorageFilter(topics=["/torque_mpc/cap"])); ty = get_message(have["/torque_mpc/cap"]); out = []
    while r.has_next():
        _, data, _ = r.read_next(); out.append(list(deserialize_message(data, ty).data[:6]))
    return np.array(out)
MODES = {"앞 F": np.array([1, 1, 1, 1]) / 2, "옆 L": np.array([-1, 1, 1, -1]) / 2, "요 Y": np.array([-1, 1, -1, 1]) / 2, "null N": np.array([1, 1, -1, -1]) / 2}
CASES = [("friction · 끔", "2026-11-27", ["165139_sfbw_lqr025_r3"]),
         ("friction · 0.25", "2026-11-27", ["164829_sfbw_lqr025_r1", "164956_sfbw_lqr025_r2"]),
         ("friction · 1.0", "2026-11-27", ["192519_yarr_mpc", "193234_yarr_mpc", "193851_yarr_mpc"]),
         ("friction · 1.5", "2026-11-27", ["192404_yarr_mpc", "193135_yarr_mpc", "194433_yarr_mpc"]),
         ("motor_only · 끔", "2026-11-27", ["162411_sfb_on_r2"]),
         ("motor_only · 1.3~1.5", "2026-11-27", ["175559_circle1", "175853_circle1", "173707_circle1", "175358_circle1"])]


def one(D, F, t_on, t_off, CAP=None):
    m = S.metrics(D, t_on, t_off); ts = m["_ts"] + t_on; vref = m["_vref"]
    Sl = D["slip"]; W = D["ws"]
    wkey = {round(t * 1e6): i for i, t in enumerate(W[:, 0])}
    idx = np.array([wkey.get(round(t * 1e6), -1) for t in Sl[:, 0]]); ok = (idx >= 0) & np.isfinite(Sl[:, 5:9]).all(1)
    tv = Sl[ok, 0]; ew = W[idx[ok], 1:5] - Sl[ok, 5:9]
    kv = (tv >= t_on) & (tv <= t_off) & (np.interp(tv, ts, vref) > 0.9 * vref.max())
    res = dict(pos=m["pos"], sv=m["sv"], m50=m["m50"])
    res["ew"] = {k: S.R_W * 100 * S.rms(ew[kv] @ v) for k, v in MODES.items()}          # cm/s (r × mode 성분)
    if F is not None:
        kf = (F[:, 0] >= t_on) & (F[:, 0] <= t_off) & (np.interp(F[:, 0], ts, vref) > 0.9 * vref.max())
        tm = F[kf, 10:14]; dt = F[kf, 6:10]; on = np.nanmean(F[kf, 1]) > 0.5
        fsh = np.column_stack([np.interp(F[kf, 0], CAP[:, 0], CAP[:, 2 + i]) for i in range(4)]) if CAP is not None else 0.0
        tm = (tm - fsh) @ PG.T                                           # 바닥 토크 τ_g = P(τ − f)
        res["sat"] = 100 * float(np.mean((np.abs(tm) >= 0.95 * TBAR).any(1)))
        res["ratio95"] = float(np.percentile((np.abs(tm) / TBAR).max(1), 95)); res["ratio50"] = float(np.median((np.abs(tm) / TBAR).max(1)))
        res["dt"] = {k: 1e3 * S.rms(dt @ v) for k, v in MODES.items()} if on else None
    else:
        M = D["mot"]; km = (M[:, 0] >= t_on) & (M[:, 0] <= t_off) & (np.interp(M[:, 0], ts, vref) > 0.9 * vref.max()); tm = M[km, 1:5]
        res["sat"] = 100 * float(np.mean((np.abs(tm) >= 0.95 * TBAR).any(1)))
        res["ratio95"] = float(np.percentile((np.abs(tm) / TBAR).max(1), 95)); res["ratio50"] = float(np.median((np.abs(tm) / TBAR).max(1))); res["dt"] = None
    return res


def show(P, lab, R):
    a = lambda f: np.mean([f(r) for r in R])
    P(f"| {lab} | {len(R)} | {a(lambda r: r['pos']):.1f} | {a(lambda r: r['sv']):.1f} | {a(lambda r: r['sat']):.0f} | {a(lambda r: r['ratio50']):.2f} / {a(lambda r: r['ratio95']):.2f} | "
      + " · ".join(f"{a(lambda r, k=k: r['ew'][k]):.1f}" for k in MODES) + " | "
      + (" · ".join(f"{a(lambda r, k=k: r['dt'][k]):.1f}" for k in MODES) if all(r["dt"] for r in R) else "-") + " |")


if __name__ == "__main__":
    out = open(os.path.join(HERE, "sfb_modes_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    P("최고 속력 구간. MPC 판: 바닥 토크 τ_g = P(τ_MPC − f) 를 정적 τ̄ 와 견줌 (dynamic_cap 하중 이동 안 넣음). 상한 붙음 = 한 바퀴라도 |τ_g| ≥ 0.95 τ̄ 인 시간 %. |τ_g|/τ̄ = 네 바퀴 중 최대의 중앙 / 95 백분위. PP 줄은 모터 토크 그대로 (참고).")
    P("| 조건 (limit · 되먹임 배율) | 판 | 위치 cm | 미끄럼 속도 cm/s | 상한 붙음 % | 최대 |τ|/τ̄ 중앙 / 95 % | 미끄럼 mode RMS cm/s 앞 · 옆 · 요 · null | Δτ mode RMS mN·m 앞 · 옆 · 요 · null |")
    P("|---|---|---|---|---|---|---|---|")
    for lab, day, names in CASES:
        R = []
        for n in names:
            p = S.find(day, n); rr = runs(p)[0]; t_on, t_off = rr["armed"][0]
            R.append(one(S.read(p), C.read_sfb(p), t_on, t_off, read_cap(p)))
        show(P, lab, R)
    RS = runs(S.SEQ); D = S.read(S.SEQ); F = C.read_sfb(S.SEQ)
    CP = read_cap(S.SEQ)
    R = [one(D, F, *r["armed"][0], CP) for r in RS if r["controller"] == "mpc"]
    show(P, "friction · 1.0 · W 80 (11-28 seq, blend 0.6)", R)
    R = [one(D, None, *r["armed"][0]) for r in RS if r["controller"] == "pp"]
    show(P, "PP (11-28 seq)", R)
    out.close()
