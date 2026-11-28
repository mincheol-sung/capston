"""2026-11-28 183517_seq_sfbq4 — S1 원 8 자 (circles8 width 2.0 = 반지름 0.5 m · 1.0 m/s · 가속 1.0 · blend 0.6 · 1 바퀴) 세 제어기 자동 순서 분석.
launch 하나 = bag 하나에 판 여러 개 → tools/bag_runs.runs() 의 무장 구간으로 판마다 자른다 (제어기 = r["controller"]).
비교: 11-26 18:47~19:04 같은 경로 조건 판 (MPC = W ×1 · 되먹임 끔, 롤러 0.88 · I_z 0.03 · 구동 방향 3.1° 는 오늘과 같음).

지표 창: 무장 중 · 기준 속력 > 0.05 m/s.
  위치 = 기준 − SLAM 로봇 중심 (/tracked_pose 는 IMU 점 → imu_offset (0.1093, 0.017) m 만큼 옮김). 진행 + = 로봇이 뒤처짐, 안쪽 + = 원 안쪽.
  출발 오차 = 무장 순간 SLAM 중심과 기준의 거리 (앞 판이 멈춘 자리에서 출발하므로 판마다 다름).
  교차점 창 = 기준 요 속도 부호가 바뀌는 시각 ±0.3 s.
  큰 슬립 % = /slip_logger/slip λ (10 Hz LPF) 네 바퀴 중 최대 |λ| > 0.5 인 시간 % (창 = 무장 · 추정 속력 > 0.05). 헛돎 = λ > 0.
  미끄럼 속도 = r (ω − ω_nom), ω = /wheel_states, ω_nom = slip 6~9 열 — 네 바퀴 중 최대의 RMS [cm/s] (circ_slipvel.py 와 같은 정의, 제어기와 무관).
  모터 온도 = /wheel_states temperature 네 바퀴 평균 (무장 구간 중앙값). 명령 떨림 = /wheel_torque_cmd 한 틱 차분 RMS [mN·m].
source install/setup.bash 뒤 /usr/bin/python3 seq_s1.py → 표 출력 + seq_s1_out.txt + 그림 seq_s1_*.png"""
import json
import math
import os
import re
import sys
import warnings

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy import stats
from scipy.signal import butter, filtfilt

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
from bag_runs import runs  # noqa: E402

B = "/home/smoc/capston/capston/bags/"
SEQ = B + "2026-11-28/183517_seq_sfbq4"
OLD = {"MPC": ["184758_s2_mpc_r1", "184937_s2_mpc_r2", "185209_s2_mpc_r3", "185937_s2_mpc_r4", "190248_s2_mpc_r5", "190352_s2_mpc_r6"],
       "PP": ["185321_s2_pp_r1", "185437_s2_pp_r2", "185532_s2_pp_r3"], "ADPP": ["185636_s2_adpp_r1", "185736_s2_adpp_r2", "185834_s2_adpp_r3"]}
WANT = ["/dra_mppi/reference_trajectory", "/state_estimator/odom", "/mpc/armed", "/slip_logger/slip", "/tracked_pose", "/wheel_states",
        "/wheel_torque_cmd", "/seq/status", "/fault_monitor/fault"]
REF_SHIFT = +0.05
R_W = 0.04
IMU_OFF = (0.1093, 0.017)
WH = ["FL", "FR", "RL", "RR"]


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rms(a):
    a = np.asarray(a)
    return float(np.sqrt(np.mean(np.square(a)))) if a.size else np.nan


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    have = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in WANT if t in have]))
    ty = {n: get_message(have[n]) for n in WANT if n in have}
    ref, od, arm, slip, sl, ws, mot, seq, flt = {}, [], [], [], [], [], [], [], []
    while r.has_next():
        n, data, ns = r.read_next()
        m = deserialize_message(data, ty[n]); tb = ns * 1e-9
        if n == "/dra_mppi/reference_trajectory":
            t0 = stamp(m.header)
            for j, p in enumerate(m.references):
                if j * m.dt > 0.15:
                    break
                ref[round((t0 + j * m.dt + REF_SHIFT) * 1000)] = (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate)
        elif n == "/state_estimator/odom":
            p, tw = m.pose.pose.position, m.twist.twist
            od.append([stamp(m.header), p.x, p.y, yaw_of(m.pose.pose.orientation), tw.linear.x, tw.linear.y, tw.angular.z])
        elif n == "/mpc/armed":
            arm.append([tb, float(m.data)])
        elif n == "/slip_logger/slip":
            slip.append(list(m.data[:9]))
        elif n == "/tracked_pose":
            sl.append([stamp(m.header), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)])
        elif n == "/wheel_states":
            ws.append([stamp(m.header), *m.velocity, *m.temperature])
        elif n == "/wheel_torque_cmd":
            if len(m.torque) == 4:
                mot.append([tb, *m.torque])
        elif n == "/seq/status":
            seq.append((tb, json.loads(m.data)))
        else:
            flt.append((tb, m.data))
    kt = np.array(sorted(ref))
    return dict(tr=kt / 1000.0, R=np.array([ref[k] for k in kt]), od=np.array(od), arm=np.array(arm), slip=np.array(slip), sl=np.array(sl),
                ws=np.array(ws), mot=np.array(mot), seq=seq, flt=flt)


def metrics(D, t_on, t_off):
    tr_all, R_all = D["tr"], D["R"]
    k = (tr_all >= t_on - 0.5) & (tr_all <= t_off + 0.5); tr, R = tr_all[k], R_all[k]
    th_ref = np.unwrap(R[:, 2])
    # SLAM 로봇 중심
    SL = D["sl"]; c, s = np.cos(SL[:, 3]), np.sin(SL[:, 3])
    sx = SL[:, 1] - (c * IMU_OFF[0] - s * IMU_OFF[1]); sy = SL[:, 2] - (s * IMU_OFF[0] + c * IMU_OFF[1])
    ks = (SL[:, 0] >= max(tr[0], t_on)) & (SL[:, 0] <= min(tr[-1], t_off))
    ts = SL[ks, 0]; sx, sy, syaw = sx[ks], sy[ks], SL[ks, 3]
    xr, yr, ub, vb, wr = (np.interp(ts, tr, R[:, i]) for i in (0, 1, 3, 4, 5))
    thr = np.interp(ts, tr, th_ref)
    vxr = np.cos(thr) * ub - np.sin(thr) * vb; vyr = np.sin(thr) * ub + np.cos(thr) * vb        # 기준 vx·vy 는 몸체 좌표
    vref = np.hypot(vxr, vyr); win = vref > 0.05
    ex, ey = xr - sx, yr - sy; epos = np.hypot(ex, ey)
    tx, ty = vxr / np.maximum(vref, 1e-6), vyr / np.maximum(vref, 1e-6)
    along = ex * tx + ey * ty; cross = -(ex * -ty + ey * tx); inside = cross * np.sign(wr)
    head = wrap(thr - syaw)
    wsg = np.sign(R[:, 5]); flip = tr[1:][(wsg[1:] * wsg[:-1] < 0)]
    near_x = np.zeros_like(ts, bool)
    for f in flip:
        near_x |= np.abs(ts - f) < 0.3
    start_err = float(np.hypot(R[np.searchsorted(tr, t_on), 0] - np.interp(t_on, SL[:, 0], SL[:, 1] - (np.cos(SL[:, 3]) * IMU_OFF[0] - np.sin(SL[:, 3]) * IMU_OFF[1])),
                               R[np.searchsorted(tr, t_on), 1] - np.interp(t_on, SL[:, 0], SL[:, 2] - (np.sin(SL[:, 3]) * IMU_OFF[0] + np.cos(SL[:, 3]) * IMU_OFF[1]))))
    # 추정기 속력
    od = D["od"]; ko = (od[:, 0] >= t_on) & (od[:, 0] <= t_off)
    vest = np.hypot(od[ko, 4], od[ko, 5]); vref_o = np.interp(od[ko, 0], ts, vref)
    vtop = vest[vref_o > 0.9 * vref.max()]
    # 슬립 λ
    S = D["slip"]; tsl = S[:, 0]; L = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(tsl)))), L, axis=0)
    sp = np.hypot(np.interp(tsl, od[:, 0], od[:, 4]), np.interp(tsl, od[:, 0], od[:, 5]))
    wl = (tsl >= t_on) & (tsl <= t_off) & (sp > 0.05)
    A4 = np.abs(Lf[wl]); imax = A4.argmax(1); big = A4.max(1) > 0.5
    sgn = np.sign(Lf[wl][np.arange(len(imax)), imax])
    nx_l = np.zeros(wl.sum(), bool)
    for f in flip:
        nx_l |= np.abs(tsl[wl] - f) < 0.3
    # 미끄럼 속도 (circ_slipvel.py)
    W = D["ws"]; wkey = {round(t * 1e6): i for i, t in enumerate(W[:, 0])}
    idx = np.array([wkey.get(round(t * 1e6), -1) for t in tsl])
    ok = (idx >= 0) & np.isfinite(S[:, 5:9]).all(1)
    tv = tsl[ok]; ev = R_W * (W[idx[ok], 1:5] - S[ok, 5:9]); sg = np.sign(S[ok, 5:9])
    ev = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(tv)))), ev, axis=0)
    wv = (tv >= t_on) & (tv <= t_off) & (np.interp(tv, ts, vref) > 0.05)
    a = np.abs(ev[wv]); iv = a.argmax(1); mx = a.max(1)
    spin_v = (ev[wv] * sg[wv])[np.arange(len(iv)), iv] > 0
    # 온도·명령
    kw = (W[:, 0] >= t_on) & (W[:, 0] <= t_off)
    temp = float(np.median(W[kw, 5:9].mean(1))); temp_max = float(np.median(W[kw, 5:9].max(1)))
    M = D["mot"]; km = (M[:, 0] >= t_on) & (M[:, 0] <= t_off)
    lap = (ts >= ts[win][0]) & win
    return dict(t_on=t_on, armed=t_off - t_on, start=100 * start_err,
                pos=100 * rms(epos[win]), pos_max=100 * float(epos[win].max()), along=100 * float(np.mean(along[win])), along_r=100 * rms(along[win]),
                cross_r=100 * rms(cross[win]), inside=100 * float(np.mean(inside[win])), head=float(np.degrees(rms(head[win]))),
                head_max=float(np.degrees(np.abs(head[win]).max())),
                pos_x=100 * rms(epos[win & near_x]), pos_nx=100 * rms(epos[win & ~near_x]),
                pos_late=100 * rms(epos[win & (ts > ts[win][0] + 1.0)]),
                vtop=float(np.mean(vtop)) if vtop.size else np.nan, vref_max=float(vref.max()),
                m50=100 * float(big.mean()), spin=100 * float(np.mean(sgn[big] > 0)) if big.any() else np.nan,
                big_x=100 * float(np.mean(nx_l[big])) if big.any() else np.nan, x_share=100 * float(nx_l.mean()),
                lrms=np.sqrt(np.mean(Lf[wl] ** 2, 0)), p50=100 * (A4 > 0.5).mean(0),
                sv=100 * float(np.sqrt(np.mean(mx ** 2))), sv20=100 * float((mx > 0.20).mean()), sv_spin=100 * float(spin_v[mx > 0.10].mean()) if (mx > 0.10).any() else np.nan,
                sv_w=100 * np.sqrt(np.mean(ev[wv] ** 2, 0)),
                temp=temp, temp_max=temp_max, tau_max=float(np.abs(M[km, 1:5]).max()), dtick=1e3 * rms(np.diff(M[km, 1:5], axis=0)),
                _ts=ts - t_on, _win=win, _sx=sx, _sy=sy, _epos=epos, _along=along, _inside=inside, _R=R, _trR=tr, _vref=vref,
                _flip=flip - t_on, _tl=tsl[wl] - t_on, _Lf=Lf[wl], _tv=tv[wv] - t_on, _ev=ev[wv], _to=od[ko, 0] - t_on, _vest=vest)


def old_params(path):
    f = os.path.join(path, "run_info", "params", "torque_mpc_node.yaml")
    if not os.path.exists(f):
        return "-", "-"
    pr = open(f).read()
    W = re.search(r"\n\s*W:\s*\n((?:\s*-\s*\S+\s*\n){6})", pr)
    W = ",".join(f"{float(x):g}" for x in re.findall(r"-\s*(\S+)", W.group(1))) if W else "?"
    en = re.search(r"slip_fb_enable:\s*(\S+)", pr)
    return W, en.group(1) if en else "?"


def find(day, name):
    for d in os.listdir(B + day):
        if d.startswith(name) and os.path.isdir(os.path.join(B + day, d)):
            return os.path.join(B + day, d)


if __name__ == "__main__":
    out = open(os.path.join(HERE, "seq_s1_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    RS = runs(SEQ); D = read(SEQ)
    P(f"== {SEQ}: 판 {len(RS)} 개, fault 메시지 {len(D['flt'])} 개")
    for tb, v in D["seq"][-4:]:
        P(f"  /seq/status 끝쪽 {tb - D['arm'][0, 0]:.1f} s: {json.dumps(v, ensure_ascii=False)[:220]}")
    rows = []
    for r in RS:
        (t_on, t_off), = r["armed"]
        m = metrics(D, t_on, t_off); m.update(ctrl=r["controller"].upper(), rnd=r["seq"]["round"], run=r["run"], day="11-28"); rows.append(m)
    for day, cs in [("2026-11-26", OLD)]:
        for c, names in cs.items():
            for n in names:
                p = find(day, n); (rr,) = runs(p); (t_on, t_off), = rr["armed"][:1] or [(None, None)]
                Dd = read(p); m = metrics(Dd, t_on, t_off); Wp, en = old_params(p)
                m.update(ctrl=c, rnd=n.split("_r")[-1], run=n[:6], day="11-26", W=Wp, sfb=en); rows.append(m)

    P("\n| 날짜 | 판 | 제어기 | 회차 | 모터 °C | 출발 오차 cm | 위치 RMS cm | 최대 | 진행 평균 (+뒤) | 안쪽 평균 | 옆 RMS | 헤딩 RMS ° | 교차 ±0.3 s | 나머지 | 최고 속력 m/s | 큰 슬립 % | 헛돎 % | 미끄럼 속도 cm/s | >0.2 m/s % | |τ| 최대 | 떨림 mN·m |")
    P("|" + "---|" * 21)
    for m in rows:
        P(f"| {m['day']} | {m['run']} | {m['ctrl']} | {m['rnd']} | {m['temp']:.1f} | {m['start']:.1f} | {m['pos']:.1f} | {m['pos_max']:.1f} | {m['along']:+.1f} | {m['inside']:+.1f} | "
          f"{m['cross_r']:.1f} | {m['head']:.1f} | {m['pos_x']:.1f} | {m['pos_nx']:.1f} | {m['vtop']:.2f} | {m['m50']:.1f} | {m['spin']:.0f} | {m['sv']:.1f} | {m['sv20']:.1f} | "
          f"{m['tau_max']:.3f} | {m['dtick']:.1f} |")

    KEYS = [("pos", "위치 RMS cm"), ("pos_max", "위치 최대 cm"), ("along", "진행 평균 cm (+뒤처짐)"), ("inside", "안쪽 평균 cm"), ("cross_r", "옆 RMS cm"),
            ("head", "헤딩 RMS °"), ("vtop", "최고 구간 속력 m/s"), ("m50", "큰 슬립 %"), ("spin", "큰 슬립 중 헛돎 %"), ("sv", "미끄럼 속도 RMS cm/s"),
            ("sv20", "미끄럼 > 0.2 m/s %"), ("dtick", "명령 떨림 mN·m"), ("start", "출발 오차 cm"), ("temp", "모터 온도 °C")]
    for day in ("11-28", "11-26"):
        P(f"\n## {day} 제어기별 평균 (최소~최대)")
        P("| 지표 | " + " | ".join(f"{c} (n {sum(1 for m in rows if m['day'] == day and m['ctrl'] == c)})" for c in ("MPC", "PP", "ADPP")) + " |")
        P("|---|---|---|---|")
        for k, lab in KEYS:
            cells = []
            for c in ("MPC", "PP", "ADPP"):
                v = np.array([m[k] for m in rows if m["day"] == day and m["ctrl"] == c])
                cells.append(f"{v.mean():.2f} ({v.min():.2f}~{v.max():.2f})" if len(v) else "-")
            P(f"| {lab} | " + " | ".join(cells) + " |")
    P("\n## 11-28 두 제어기씩 (Welch t · Mann-Whitney 양측 p, n 5 대 5 — MW 최소 p 0.008)")
    for k, lab in KEYS[:12]:
        g = {c: np.array([m[k] for m in rows if m["day"] == "11-28" and m["ctrl"] == c]) for c in ("MPC", "PP", "ADPP")}
        s = []
        for a_, b_ in (("MPC", "PP"), ("MPC", "ADPP"), ("PP", "ADPP")):
            t = stats.ttest_ind(g[a_], g[b_], equal_var=False).pvalue; u = stats.mannwhitneyu(g[a_], g[b_], alternative="two-sided").pvalue
            s.append(f"{a_}−{b_} {g[a_].mean() - g[b_].mean():+.2f} (t {t:.3f} · MW {u:.3f})")
        P(f"- {lab}: " + " · ".join(s))
    P("\n## 같은 제어기 11-28 대 11-26 (Welch t)")
    for k, lab in KEYS[:12]:
        s = []
        for c in ("MPC", "PP", "ADPP"):
            a_ = np.array([m[k] for m in rows if m["day"] == "11-28" and m["ctrl"] == c]); b_ = np.array([m[k] for m in rows if m["day"] == "11-26" and m["ctrl"] == c])
            s.append(f"{c} {b_.mean():.2f} → {a_.mean():.2f} (p {stats.ttest_ind(a_, b_, equal_var=False).pvalue:.3f})")
        P(f"- {lab}: " + " · ".join(s))
    P("\n## 11-28 회차 흐름 (회차 1→5) · 출발 오차·온도와 위치의 상관")
    for c in ("MPC", "PP", "ADPP"):
        g = [m for m in rows if m["day"] == "11-28" and m["ctrl"] == c]
        pos = np.array([m["pos"] for m in g]); st = np.array([m["start"] for m in g]); tp = np.array([m["temp"] for m in g]); rn = np.arange(1, len(g) + 1)
        P(f"- {c}: 위치 " + " → ".join(f"{x:.1f}" for x in pos) + " cm · 출발 오차 " + " → ".join(f"{x:.1f}" for x in st)
          + " · 온도 " + " → ".join(f"{x:.1f}" for x in tp)
          + f" · r(회차, 위치) {np.corrcoef(rn, pos)[0, 1]:+.2f} · r(출발, 위치) {np.corrcoef(st, pos)[0, 1]:+.2f} · 첫 1 s 뺀 위치 " + " → ".join(f"{m['pos_late']:.1f}" for m in g))
    P("\n## 11-26 MPC 판 설정 (params dump)")
    for m in rows:
        if m["day"] == "11-26" and m["ctrl"] == "MPC":
            P(f"- {m['run']}: W [{m['W']}] · slip_fb_enable {m['sfb']}")
    P("\n## 바퀴별 (11-28, 제어기 평균) — 큰 슬립 % · 미끄럼 속도 RMS cm/s")
    for c in ("MPC", "PP", "ADPP"):
        g = [m for m in rows if m["day"] == "11-28" and m["ctrl"] == c]
        p50 = np.mean([m["p50"] for m in g], 0); svw = np.mean([m["sv_w"] for m in g], 0)
        P(f"- {c}: " + " · ".join(f"{w} {a:.1f} % / {b:.1f}" for w, a, b in zip(WH, p50, svw)) + f" · 큰 슬립의 교차점 몫 {np.mean([m['big_x'] for m in g]):.0f} % (시간 몫 {np.mean([m['x_share'] for m in g]):.0f} %)")
    out.close()
    import pickle
    pickle.dump([{k: v for k, v in m.items()} for m in rows], open(os.path.join(os.environ.get("SEQ_PKL_DIR", HERE), "seq_s1_rows.pkl"), "wb"))
