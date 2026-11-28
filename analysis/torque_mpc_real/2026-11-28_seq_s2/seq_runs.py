"""2026-11-28 amrseq (controller:=all) bag — 판마다 추종·슬립 지표, 제어기별 비교.
source install/setup.bash 뒤 (nice·ionice 권장 — 실차가 도는 중이면)
  /usr/bin/python3 seq_runs.py read <bag 폴더 또는 .mcap> <캐시.npz>       bag 을 한 번 읽어 필요한 토픽만 캐시 (녹화 중인 bag 도 — 끝까지 읽힌 데까지)
  /usr/bin/python3 seq_runs.py report <캐시.npz> <판 a-b> [<판 c-d> ...]   판 범위마다 판별 표 + 제어기별 평균 (복귀 다리 kind=return 은 뺀다)
지표 정의는 ../2026-11-28_poly_stress/poly_run.py 와 같다:
  창 = 그 판의 무장 구간 · 기준 속력 > 0.05 m/s. 추정기 오차 = 기준 − /state_estimator/odom. 진행 + = 뒤처짐, 옆 + = 로봇이 기준 왼쪽.
  SLAM 오차 = 기준 − /tracked_pose (IMU 점 → 로봇 중심, imu_offset 0.1093·0.017 m).
  슬립 λ = /slip_logger/slip, 10 Hz 1차 Butterworth filtfilt. 큰 슬립 = 네 바퀴 중 최대 |λ̃| > 0.5 인 시간 %. 헛돎 몫 = 그 순간 가장 큰 바퀴 λ > 0.
  꼭짓점 통과 속력 = 기준 속력 극소 (< CORNER_MIN) 시각의 SLAM 중심 0.1 s 차분 속력. zigzag pass 기준 꼭짓점 0.3 m/s → CORNER_MIN 0.45.
판·제어기·다리는 tools/bag_runs.py split_runs (/figure8/run_config · /figure8/status · /mpc/armed · /ctrl/active · /seq/status)."""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "..", "..", "..", "src", "amr_basic_stack", "controller", "mpc", "tools")
sys.path.insert(0, TOOLS)
JSON_T = ["/figure8/run_config", "/figure8/status", "/torque_mpc/config", "/ctrl/active", "/seq/status"]
WANT = JSON_T + ["/mpc/armed", "/dra_mppi/reference_trajectory", "/state_estimator/odom", "/slip_logger/slip", "/tracked_pose",
                 "/fault_monitor/fault"]
REF_SHIFT, IMU_OFF, CORNER_MIN = 0.05, (0.1093, 0.017), 0.45
WH = ["FL", "FR", "RL", "RR"]


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rms(a):
    a = np.asarray(a, float)
    return float(np.sqrt(np.mean(np.square(a)))) if a.size else float("nan")


def cmd_read(src, out):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if os.path.isdir(src):
        mc = [f for f in os.listdir(src) if f.endswith(".mcap")]
        uri = src if os.path.exists(os.path.join(src, "metadata.yaml")) else os.path.join(src, sorted(mc)[0])
    else:
        uri = src
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    have = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in WANT if t in have]))
    types = {n: get_message(have[n]) for n in WANT if n in have}
    ev, ref, od, slip, sl, flt = [], {}, [], [], [], []
    n = 0
    try:
        while r.has_next():
            name, data, ns = r.read_next()
            tb = ns * 1e-9
            m = deserialize_message(data, types[name])
            n += 1
            if name in JSON_T:
                ev.append((tb, name, json.loads(m.data)))
            elif name == "/mpc/armed":
                ev.append((tb, name, bool(m.data)))
            elif name == "/dra_mppi/reference_trajectory":
                t0 = stamp(m.header)
                for j, p in enumerate(m.references):
                    if j * m.dt > 0.15:
                        break
                    ref[round((t0 + j * m.dt + REF_SHIFT) * 1000)] = (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate)
            elif name == "/state_estimator/odom":
                p, tw = m.pose.pose.position, m.twist.twist
                od.append([stamp(m.header), tb, p.x, p.y, yaw_of(m.pose.pose.orientation), tw.linear.x, tw.linear.y, tw.angular.z])
            elif name == "/slip_logger/slip":
                slip.append(list(m.data[:5]))
            elif name == "/tracked_pose":
                sl.append([stamp(m.header), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)])
            else:
                flt.append((tb, m.data))
    except Exception as e:                                # 녹화 중인 bag 의 끝 (덜 쓴 chunk)
        print(f"읽기 멈춤 ({n} 개 뒤): {e}")
    kt = np.array(sorted(ref))
    np.savez_compressed(out, tr=kt / 1000.0, R=np.array([ref[k] for k in kt]), od=np.array(od), slip=np.array(slip), sl=np.array(sl),
                        ev=np.array(json.dumps(ev, ensure_ascii=False)), flt=np.array(json.dumps(flt, ensure_ascii=False)),
                        src=np.array(os.path.abspath(src)))
    print(f"{out}: 메시지 {n} 개, odom {len(od)}, 기준 {len(kt)}, 슬립 {len(slip)}, SLAM {len(sl)}, 사건 {len(ev)}")


def load(cache):
    from bag_runs import split_runs
    D = dict(np.load(cache, allow_pickle=False))
    ev = [tuple(e) for e in json.loads(str(D["ev"]))]
    t_start = min(e[0] for e in ev) if ev else 0.0
    t_end = float(D["od"][-1, 1]) if len(D["od"]) else t_start
    D["runs"] = split_runs(sorted(ev, key=lambda e: e[0]), t_start, t_end)
    D["flt"] = json.loads(str(D["flt"]))
    return D


def metrics(D, run):
    from scipy.signal import butter, filtfilt
    tr, R, od = D["tr"], D["R"], D["od"]
    sp = run["armed"]
    if not sp:
        return None
    t_on, t_off = sp[0][0], min(sp[-1][1], run["t1"])
    # od 는 [stamp, 수신, x, y, yaw, vx, vy, wz]. 창은 수신 시각 축 (무장 구간과 같은 축), 오차는 stamp 축 기준과 맞춘다
    k = (od[:, 1] >= t_on - 0.05) & (od[:, 1] <= t_off) & (od[:, 0] >= tr[0]) & (od[:, 0] <= tr[-1])
    X = od[k]
    if len(X) < 50:
        return None
    t = X[:, 0]
    xr, yr, ub, vb, wr = (np.interp(t, tr, R[:, i]) for i in (0, 1, 3, 4, 5))
    thr = np.interp(t, tr, np.unwrap(R[:, 2])); th = X[:, 4]
    vxr = np.cos(thr) * ub - np.sin(thr) * vb; vyr = np.sin(thr) * ub + np.cos(thr) * vb
    E = np.column_stack([xr - X[:, 2], yr - X[:, 3], wrap(thr - th)])
    vref = np.hypot(vxr, vyr); win = vref > 0.05
    if win.sum() < 20:
        return None
    tx, ty = vxr / np.maximum(vref, 1e-6), vyr / np.maximum(vref, 1e-6)
    along = E[:, 0] * tx + E[:, 1] * ty; cross = -(E[:, 0] * -ty + E[:, 1] * tx)
    epos = np.hypot(E[:, 0], E[:, 1])
    spd = np.hypot(X[:, 5], X[:, 6])
    t_mv0, t_mv1 = t[win][0], t[win][-1]
    # SLAM 중심
    SL = D["sl"]; c, s = np.cos(SL[:, 3]), np.sin(SL[:, 3])
    sx = SL[:, 1] - (c * IMU_OFF[0] - s * IMU_OFF[1]); sy = SL[:, 2] - (s * IMU_OFF[0] + c * IMU_OFF[1])
    ks = (SL[:, 0] >= t_mv0) & (SL[:, 0] <= t_mv1)
    es = np.hypot(np.interp(SL[ks, 0], tr, R[:, 0]) - sx[ks], np.interp(SL[ks, 0], tr, R[:, 1]) - sy[ks])
    # 슬립
    S = D["slip"]; ksl = (S[:, 0] >= t_on - 1.0) & (S[:, 0] <= t_off + 1.0); S = S[ksl]
    m50 = m25 = spin = big_corner = np.nan; p50 = [np.nan] * 4
    if len(S) > 30:
        L = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0)
        Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(S[:, 0])))), L, axis=0)
        sp_l = np.interp(S[:, 0], t, spd)
        ws = (S[:, 0] >= t_mv0) & (S[:, 0] <= t_mv1) & (sp_l > 0.05)
        A4 = np.abs(Lf[ws]); imax = A4.argmax(1); big = A4.max(1) > 0.5
        sgn = np.sign(Lf[ws][np.arange(len(imax)), imax])
        corner = np.interp(S[ws, 0], t, vref) < CORNER_MIN + 0.1
        m50, m25 = 100 * float(big.mean()), 100 * float(np.mean(A4.max(1) > 0.25))
        p50 = list(100 * (A4 > 0.5).mean(0))
        spin = 100 * float(np.mean(sgn[big] > 0)) if big.any() else np.nan
        big_corner = 100 * float(np.mean(corner[big])) if big.any() else np.nan
    # 꼭짓점 통과 속력 (기준 속력 극소)
    kr = (tr >= t_mv0) & (tr <= t_mv1); trr = tr[kr]; vr_g = np.hypot(R[kr, 3], R[kr, 4]); tc = []
    for i in range(1, len(vr_g) - 1):
        if vr_g[i] < CORNER_MIN and vr_g[i] <= vr_g[i - 1] and vr_g[i] <= vr_g[i + 1] and vr_g[i] > 0.05 and (not tc or trr[i] - tc[-1] > 0.4):
            tc.append(trr[i])
    vc_s, vc_r = [], []
    for x in tc:
        i0, i1 = np.searchsorted(SL[:, 0], x - 0.05), min(np.searchsorted(SL[:, 0], x + 0.05), len(SL) - 1)
        if i1 > i0:
            vc_s.append(math.hypot(sx[i1] - sx[i0], sy[i1] - sy[i0]) / max(SL[i1, 0] - SL[i0, 0], 1e-3))
            vc_r.append(float(np.interp(x, trr, vr_g)))
    rc = run["ref"] or {}
    return dict(run=run["run"], ctrl=run.get("controller"), seq=run.get("seq") or {}, end=run["end"],
                shape=rc.get("shape"), summary=rc.get("summary", ""), relevant=rc.get("relevant", {}),
                dur=float(t_mv1 - t_mv0), pos=100 * rms(epos[win]), pos_max=100 * float(epos[win].max()),
                cross=100 * rms(cross[win]), along=100 * float(np.mean(along[win])), along_r=100 * rms(along[win]),
                along_max=100 * float(along[win].max()), head=float(np.degrees(rms(E[win, 2]))),
                slam=100 * rms(es), slam_max=100 * float(es.max()) if es.size else np.nan,
                m50=m50, m25=m25, p50=p50, spin=spin, big_corner=big_corner,
                vcorner=float(np.mean(vc_s)) if vc_s else np.nan, vcorner_ref=float(np.mean(vc_r)) if vc_r else np.nan, n_corner=len(vc_s),
                vmax=float(spd[win].max()), vref_max=float(vref[win].max()),
                faults=[f for tb, f in D["flt"] if t_on - 0.5 <= tb <= t_off + 1.0])


def welch(a, b):
    from scipy.stats import ttest_ind
    a, b = np.asarray(a, float), np.asarray(b, float)
    a, b = a[np.isfinite(a)], b[np.isfinite(b)]
    if len(a) < 2 or len(b) < 2:
        return np.nan
    return float(ttest_ind(a, b, equal_var=False).pvalue)


KEYS = [("pos", "위치 RMS 추정기 [cm]"), ("pos_max", "위치 최대 [cm]"), ("slam", "위치 RMS SLAM [cm]"), ("cross", "옆 RMS [cm]"),
        ("along", "진행 평균 (+뒤처짐) [cm]"), ("along_max", "진행 최대 [cm]"), ("head", "헤딩 RMS [°]"), ("m50", "큰 슬립 >0.5 [%]"),
        ("m25", "중간 슬립 >0.25 [%]"), ("spin", "큰 슬립 중 헛돎 [%]"), ("vcorner", "꼭짓점 통과 속력 SLAM [m/s]"), ("vmax", "최고 속력 [m/s]")]


def cmd_report(cache, ranges):
    D = load(cache)
    out = {}
    for rg in ranges:
        a, b = (int(x) for x in rg.split("-"))
        rows = [m for r in D["runs"] if a <= r["run"] <= b for m in [metrics(D, r)] if m]
        data = [m for m in rows if (m["seq"].get("kind") or "data") == "data" and m["shape"] != "rotate"]
        ret = [m for m in rows if m not in data]
        print(f"\n━━ 판 {a}~{b}: 판 {len(rows)} 개 (data {len(data)} · 복귀 {len(ret)})")
        shapes = sorted({m["summary"] for m in data})
        for s_ in shapes:
            print(f"   기준: {s_}")
        print(f"{'판':>3} {'제어기':5} {'다리':>3} {'방향':3} {'위치':>5} {'최대':>5} {'SLAM':>5} {'옆':>5} {'진행':>6} {'헤딩°':>5} {'큰슬립%':>6} {'헛돎%':>5} {'꼭짓점':>6} {'최고':>5}  끝")
        for m in data:
            q = m["seq"]
            print(f"{m['run']:3d} {m['ctrl'] or '?':5} {q.get('i', ''):>3} {q.get('dir') or '':3} {m['pos']:5.2f} {m['pos_max']:5.1f} {m['slam']:5.2f} {m['cross']:5.2f} "
                  f"{m['along']:+6.2f} {m['head']:5.2f} {m['m50']:6.1f} {m['spin']:5.0f} {m['vcorner']:6.2f} {m['vmax']:5.2f}  {m['end'][:6]}"
                  + (f"  fault {m['faults']}" if m["faults"] else ""))
        if ret:
            print(f"   복귀 회전 {len(ret)} 번: 위치 RMS 평균 {np.mean([m['pos'] for m in ret]):.2f} cm, 헤딩 RMS 평균 {np.mean([m['head'] for m in ret]):.2f}°")
        by = {c: [m for m in data if m["ctrl"] == c] for c in ("mpc", "pp", "adpp")}
        print(f"\n   제어기별 평균 ± 표준편차 (판 수) — p = Welch (mpc 대비)")
        print(f"   {'지표':28} " + " ".join(f"{c + f' ({len(by[c])})':>18}" for c in by) + f" {'p pp':>7} {'p adpp':>7}")
        summ = {}
        for k, lab in KEYS:
            vals = {c: np.array([m[k] for m in by[c]], float) for c in by}
            cell = lambda v: f"{np.nanmean(v):8.2f} ± {np.nanstd(v, ddof=1) if np.isfinite(v).sum() > 1 else 0:5.2f}" if np.isfinite(v).any() else f"{'-':>16}"
            print(f"   {lab:28} " + " ".join(f"{cell(vals[c]):>18}" for c in by)
                  + f" {welch(vals['mpc'], vals['pp']):7.3f} {welch(vals['mpc'], vals['adpp']):7.3f}")
            summ[k] = {c: [float(x) for x in vals[c]] for c in by}
        if any(m["seq"].get("dir") for m in data):
            print("\n   방향별 위치 RMS 평균 (fwd / rev):")
            for c in by:
                f = [m["pos"] for m in by[c] if m["seq"].get("dir") == "fwd"]; r = [m["pos"] for m in by[c] if m["seq"].get("dir") == "rev"]
                print(f"   {c:5} fwd {np.mean(f) if f else np.nan:5.2f} ({len(f)}) · rev {np.mean(r) if r else np.nan:5.2f} ({len(r)})")
        out[rg] = dict(rows=[{k: v for k, v in m.items()} for m in data], returns=len(ret), summary=summ)
    js = os.path.join(HERE, "report_" + "_".join(ranges) + ".json")
    with open(js, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1, default=str)
    print(f"\nJSON: {js}")


if __name__ == "__main__":
    if sys.argv[1] == "read":
        cmd_read(sys.argv[2], sys.argv[3])
    else:
        cmd_report(sys.argv[2], sys.argv[3:])
