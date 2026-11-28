"""2026-11-28 꺾은선 스트레스 판 (polyline · accel 1.5 · square_v_max 1.5 · pass · friction) 분석 — 추종 오차·슬립·경로 그림.
source install/setup.bash 뒤  /usr/bin/python3 poly_run.py <대상 bag> [비교 bag ...]
  지표 창: 무장 중 · 기준 속력 > 0.05 m/s (2026-11-27_q_tuning/q_laps.py 와 같은 정의).
  추정기 오차 = 기준 − /state_estimator/odom (map). 진행 + = 로봇이 뒤처짐, 옆 + = 로봇이 기준 왼쪽 (q_laps 부호 그대로).
  SLAM 오차 = 기준 − /tracked_pose 를 로봇 중심으로 옮긴 것 (imu_to_center, imu_offset 0.1093·0.017 m — tracked_pose 는 IMU 점).
  슬립 λ = /slip_logger/slip [t, λ_FL, λ_FR, λ_RL, λ_RR] (λ > 0 헛돎, < 0 끌림), 10 Hz 1차 Butterworth filtfilt, 창 = 무장 · 추정 속력 > 0.05.
    큰 슬립 = 네 바퀴 중 최대 |λ̃| > 0.5 인 시간 %. 헛돎/끌림 몫 = 큰 슬립 순간 가장 큰 바퀴의 부호.
    꼭짓점 구간 = 기준 속력 < 0.3 m/s (꼭짓점 통과 0.1 m/s 앞뒤 가감속), 나머지 = 직선.
  꼭짓점 통과 속력 = 움직이는 구간 안 기준 속력 극소 (< 0.15 m/s, 0.4 s 안 하나) 시각의 추정기 속력과 SLAM 중심 0.1 s 중심 차분 속력.
  미끄럼 속도 = /torque_mpc/slip_fb 맨 뒤 4 열 e_ω × r (0.04 m) 의 RMS — 그 토픽이 없는 옛 판은 '-'.
그림: <대상>_path.png (경로 · 큰 슬립 위치), <대상>_time.png (오차·속도·λ̃ 시간축), compare.png (비교 판 막대)."""
import math
import os
import re
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
BAGS = [p.rstrip("/") for p in sys.argv[1:]]
WANT = ["/dra_mppi/reference_trajectory", "/state_estimator/odom", "/mpc/armed", "/slip_logger/slip", "/torque_mpc/slip_fb", "/tracked_pose",
        "/wheel_torque_cmd", "/fault_monitor/fault"]
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
    types = {n: get_message(have[n]) for n in WANT if n in have}
    ref, od, arm, slip, sfb, sl, mot, flt = {}, [], [], [], [], [], [], []
    while r.has_next():
        name, data, ns = r.read_next()
        m = deserialize_message(data, types[name]); tb = ns * 1e-9
        if name == "/dra_mppi/reference_trajectory":
            t0 = stamp(m.header)
            for j, p in enumerate(m.references):
                if j * m.dt > 0.15:
                    break
                ref[round((t0 + j * m.dt + REF_SHIFT) * 1000)] = (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate)
        elif name == "/state_estimator/odom":
            p, tw = m.pose.pose.position, m.twist.twist
            od.append([stamp(m.header), p.x, p.y, yaw_of(m.pose.pose.orientation), tw.linear.x, tw.linear.y, tw.angular.z])
        elif name == "/mpc/armed":
            arm.append([tb, float(m.data)])
        elif name == "/slip_logger/slip":
            slip.append(list(m.data[:5]))
        elif name == "/torque_mpc/slip_fb":
            d = list(m.data); sfb.append(d[:22] + [math.nan] * (22 - len(d)))
        elif name == "/tracked_pose":
            sl.append([stamp(m.header), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)])
        elif name == "/wheel_torque_cmd":
            if len(m.torque) == 4:
                mot.append([tb, *m.torque])
        else:
            flt.append((tb, m.data))
    kt = np.array(sorted(ref))
    return dict(tr=kt / 1000.0, R=np.array([ref[k] for k in kt]), od=np.array(od), arm=np.array(arm), slip=np.array(slip),
                sfb=np.array(sfb) if sfb else None, sl=np.array(sl), mot=np.array(mot), flt=flt)


def args_of(path):
    a = open(os.path.join(path, "run_info", "args.txt")).read()
    pr = ""
    pd = os.path.join(path, "run_info", "params", "torque_mpc_node.yaml")
    if os.path.exists(pd):
        pr = open(pd).read()
    g = lambda k, d="": (re.search(rf"\b{k}:=(\S+)", a) or [None, d])[1]
    W = re.search(r"\n\s*W:\s*\n((?:\s*-\s*\S+\s*\n){6})", pr)
    W = ",".join(f"{float(x):g}" for x in re.findall(r"-\s*(\S+)", W.group(1))) if W else "?"
    en = re.search(r"slip_fb_enable:\s*(\S+)", pr); sc = re.search(r"slip_fb_scale:\s*(\S+)", pr)
    vs = re.search(r"velocity_source:\s*(\S+)", pr)
    sfb = "없음 (코드 전)" if not en else (f"lqr {float(sc.group(1)):g}" if en.group(1) == "true" else "끔")
    return dict(W=W, sfb=sfb, vs=vs.group(1) if vs else "?", accel=g("accel"), vmax=g("square_v_max"))


def analyze(path):
    name = os.path.basename(path); A = args_of(path); D = read(path)
    tr, R, od, arm = D["tr"], D["R"], D["od"], D["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]; t_off = td[td > t_on][0] if (td > t_on).any() else od[-1, 0]
    t = od[:, 0]; msk = (t >= tr[0]) & (t <= tr[-1]) & (t >= t_on - 0.05); t, X = t[msk], od[msk]
    xr, yr_, ub, vb, wr = (np.interp(t, tr, R[:, i]) for i in (0, 1, 3, 4, 5))
    th_r = np.interp(t, tr, np.unwrap(R[:, 2])); th = X[:, 3]
    vxr = np.cos(th_r) * ub - np.sin(th_r) * vb; vyr = np.sin(th_r) * ub + np.cos(th_r) * vb          # 기준 vx·vy 는 몸체 좌표
    vx = np.cos(th) * X[:, 4] - np.sin(th) * X[:, 5]; vy = np.sin(th) * X[:, 4] + np.cos(th) * X[:, 5]
    E = np.column_stack([xr - X[:, 1], yr_ - X[:, 2], wrap(th_r - th), vxr - vx, vyr - vy, wr - X[:, 6]])
    vref = np.hypot(vxr, vyr); win = (vref > 0.05) & (t <= t_off)
    tx, ty = vxr / np.maximum(vref, 1e-6), vyr / np.maximum(vref, 1e-6)
    along = E[:, 0] * tx + E[:, 1] * ty; cross = -(E[:, 0] * -ty + E[:, 1] * tx)
    epos = np.hypot(E[:, 0], E[:, 1])
    # SLAM (로봇 중심)
    SL = D["sl"]; c, s = np.cos(SL[:, 3]), np.sin(SL[:, 3])
    sx = SL[:, 1] - (c * IMU_OFF[0] - s * IMU_OFF[1]); sy = SL[:, 2] - (s * IMU_OFF[0] + c * IMU_OFF[1])
    ks = (SL[:, 0] >= tr[0]) & (SL[:, 0] <= tr[-1]) & (SL[:, 0] >= t_on) & (SL[:, 0] <= t_off)
    ts_ = SL[ks, 0]; vref_s = np.interp(ts_, t, vref); kk = vref_s > 0.05
    es = np.hypot(np.interp(ts_, tr, R[:, 0]) - sx[ks], np.interp(ts_, tr, R[:, 1]) - sy[ks])
    # 슬립
    S = D["slip"]; tsl = S[:, 0]; L = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(tsl)))), L, axis=0)
    sp = np.hypot(np.interp(tsl, od[:, 0], od[:, 4]), np.interp(tsl, od[:, 0], od[:, 5]))
    ws = (tsl >= t_on) & (tsl <= t_off) & (sp > 0.05)
    A4 = np.abs(Lf[ws]); imax = A4.argmax(1); big = A4.max(1) > 0.5
    sgn = np.sign(Lf[ws][np.arange(len(imax)), imax])
    vref_l = np.interp(tsl[ws], t, vref); corner = vref_l < 0.3
    F = D["sfb"]
    if F is not None and np.isfinite(F[:, 18]).any():
        tf = F[:, 0]; wf = (tf >= t_on) & (tf <= t_off) & (np.interp(tf, od[:, 0], np.hypot(od[:, 4], od[:, 5])) > 0.05)
        vslip = 100 * R_W * rms(F[wf, 18:22]); dtau = 1e3 * rms(F[wf, 6:10])
    else:
        vslip = dtau = np.nan
    M = D["mot"]; t_mv = t[win]; wm = (M[:, 0] >= t_mv[0]) & (M[:, 0] <= t_mv[-1])
    v_off = float(np.interp(t_off, tr, np.hypot(R[:, 3], R[:, 4]))); done = v_off < 0.05
    vr_g = np.hypot(R[:, 3], R[:, 4]); mv = np.where((tr >= t_on) & (vr_g > 0.3))[0]; tc = []
    for i in range(mv[0], mv[-1]):
        if vr_g[i] < 0.15 and vr_g[i] <= vr_g[i - 1] and vr_g[i] <= vr_g[i + 1] and (not tc or tr[i] - tc[-1] > 0.4):
            tc.append(tr[i])
    tsl_all = SL[:, 0]; vc_s = []
    for x in tc:
        i0, i1 = np.searchsorted(tsl_all, x - 0.05), np.searchsorted(tsl_all, x + 0.05)
        vc_s.append(np.hypot(sx[i1] - sx[i0], sy[i1] - sy[i0]) / max(tsl_all[i1] - tsl_all[i0], 1e-3))
    vc_e = [float(np.interp(x, od[:, 0], np.hypot(od[:, 4], od[:, 5]))) for x in tc]
    res = dict(name=name, A=A, done=done, armed=t_off - t_on, dur=float(win.sum() * np.median(np.diff(t))),
               pos=100 * rms(epos[win]), pos_max=100 * float(epos[win].max()), cross_r=100 * rms(cross[win]), along=100 * float(np.mean(along[win])),
               along_r=100 * rms(along[win]), head=float(np.degrees(rms(E[win, 2]))), head_max=float(np.degrees(np.abs(E[win, 2]).max())),
               vel=100 * rms(np.hypot(E[win, 3], E[win, 4])),
               slam_pos=100 * rms(es[kk]), slam_max=100 * float(es[kk].max()),
               m50=100 * float(big.mean()), m25=100 * float(np.mean(A4.max(1) > 0.25)), p50=100 * (A4 > 0.5).mean(0), lrms=np.sqrt(np.mean(Lf[ws] ** 2, 0)),
               spin=100 * float(np.mean(sgn[big] > 0)) if big.any() else np.nan,
               big_corner=100 * float(np.mean(corner[big])) if big.any() else np.nan,
               m50_corner=100 * float(big[corner].mean()) if corner.any() else np.nan, m50_line=100 * float(big[~corner].mean()) if (~corner).any() else np.nan,
               corner_share=100 * float(corner.mean()), vc_e=vc_e, vc_s=vc_s, vcorner=float(np.mean(vc_s)), vslip=vslip, dtau=dtau, dtick=1e3 * rms(np.diff(M[wm, 1:5], axis=0)),
               vmax=float(np.hypot(X[:, 4], X[:, 5])[win].max()), faults=[f for tb, f in D["flt"] if tb >= t_on - 1],
               # 그림용
               _t=t - t_on, _win=win, _xr=xr, _yr=yr_, _X=X, _epos=epos, _cross=cross, _along=along, _vref=vref, _ts=ts_ - t_on, _es=es, _sx=sx[ks], _sy=sy[ks],
               _tl=tsl - t_on, _Lf=Lf, _ws=ws, _R=R, _tr=tr - t_on, _big_t=tsl[ws][big] - t_on, _t_off=t_off - t_on)
    return res


def show(r):
    A = r["A"]
    print(f"{r['name']}: W {A['W']} · 슬립 되먹임 {A['sfb']} · 속도 출처 {A['vs']} · accel {A['accel']} · v_max {A['vmax']} · 무장 {r['armed']:.1f} s · 끝까지 {'예' if r['done'] else '아니오'}"
          f" · fault {r['faults'] or '없음'}")
    print(f"  추정기 기준: 위치 RMS {r['pos']:.2f} cm (최대 {r['pos_max']:.1f}) · 옆 RMS {r['cross_r']:.2f} · 진행 평균 {r['along']:+.2f} (RMS {r['along_r']:.2f})"
          f" · 헤딩 RMS {r['head']:.2f}° (최대 {r['head_max']:.1f}) · 속도 RMS {r['vel']:.1f} cm/s · 최고 속력 {r['vmax']:.2f} m/s")
    print(f"  SLAM 중심 기준: 위치 RMS {r['slam_pos']:.2f} cm (최대 {r['slam_max']:.1f})")
    print("  꼭짓점 통과 속력 (기준 0.1 m/s): 추정기 " + " ".join(f"{x:.2f}" for x in r['vc_e']) + " · SLAM " + " ".join(f"{x:.2f}" for x in r['vc_s'])
          + f" → SLAM 평균 {r['vcorner']:.2f} m/s")
    print(f"  슬립: 큰 슬립 (>0.5) {r['m50']:.1f} % · 중간 (>0.25) {r['m25']:.1f} % · 바퀴별 >0.5 " + " ".join(f"{w} {p:.1f}" for w, p in zip(WH, r['p50']))
          + " · λ̃ RMS " + " ".join(f"{w} {x:.2f}" for w, x in zip(WH, r['lrms'])))
    print(f"        큰 슬립 중 헛돎 {r['spin']:.0f} % / 끌림 {100 - r['spin']:.0f} % · 큰 슬립의 {r['big_corner']:.0f} % 가 꼭짓점 구간 (시간 몫 {r['corner_share']:.0f} %)"
          f" · 구간 안 큰 슬립 꼭짓점 {r['m50_corner']:.1f} % / 직선 {r['m50_line']:.1f} %")
    print(f"        미끄럼 속도 RMS {r['vslip']:.2f} cm/s · 되먹임 보정 토크 RMS {r['dtau']:.1f} mN·m · 모터 명령 떨림 {r['dtick']:.1f} mN·m")


# ── 글꼴·색
fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
C_REF, C_EST, C_SLAM, C_BIG = "#8c959f", "#0969da", "#1a7f37", "#cf222e"
WC = ["#0969da", "#bf8700", "#8250df", "#cf222e"]
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})

res = []
for p in BAGS:
    r = analyze(p); show(r); res.append(r)

T = res[0]; tag = T["name"].split("_")[0]
# 1) 경로
fig, axs = plt.subplots(1, 2, figsize=(13, 6.2), gridspec_kw=dict(width_ratios=[1.25, 1]))
ax = axs[0]; w = T["_win"]
ax.plot(T["_R"][:, 0], T["_R"][:, 1], color=C_REF, lw=6, alpha=0.35, solid_capstyle="round", label="기준 경로")
ax.plot(T["_X"][w, 1], T["_X"][w, 2], color=C_EST, lw=1.4, label="추정기")
ax.plot(T["_sx"], T["_sy"], color=C_SLAM, lw=1.0, ls="--", label="SLAM (로봇 중심)")
bt = T["_big_t"]
if len(bt):
    bx = np.interp(bt, T["_t"], T["_X"][:, 1]); by = np.interp(bt, T["_t"], T["_X"][:, 2])
    ax.scatter(bx, by, s=9, color=C_BIG, zorder=5, label=f"큰 슬립 (네 바퀴 중 최대 |λ| > 0.5) {T['m50']:.0f} %")
ax.plot(T["_R"][0, 0], T["_R"][0, 1], "o", color=INK, ms=6); ax.annotate("출발·끝", (T["_R"][0, 0], T["_R"][0, 1]), xytext=(8, -14), textcoords="offset points", color=INK2, fontsize=9)
ax.set_aspect("equal"); ax.grid(color=GRID, lw=0.5); ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m] (map)")
ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.09), fontsize=8.5, frameon=False, ncol=2)
ax.set_title(f"{T['name']} 경로 — 위치 RMS 추정기 {T['pos']:.1f} cm · SLAM {T['slam_pos']:.1f} cm", loc="left", fontsize=10.5, color=INK)
# 오른쪽: 경로를 오차 크기로 칠함
ax = axs[1]
sc = ax.scatter(T["_X"][w, 1], T["_X"][w, 2], c=100 * T["_epos"][w], s=4, cmap="viridis", vmin=0, vmax=max(8, np.percentile(100 * T["_epos"][w], 99)))
ax.plot(T["_R"][:, 0], T["_R"][:, 1], color=C_REF, lw=0.8)
cb = fig.colorbar(sc, ax=ax, shrink=0.8); cb.set_label("위치 오차 (추정기) [cm]")
ax.set_aspect("equal"); ax.grid(color=GRID, lw=0.5); ax.set_xlabel("x [m]")
ax.set_title("오차 크기로 칠한 추정기 경로", loc="left", fontsize=10.5, color=INK)
A = T["A"]
fig.text(0.01, 0.995, f"polyline R1,L1,L2,R1,R1,R2 · accel {A['accel']} m/s² · v_max {A['vmax']} m/s · 꼭짓점 통과 0.1 m/s · friction · W [{A['W']}] · 슬립 되먹임 {A['sfb']}",
         fontsize=10, color=INK2, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(os.path.join(HERE, f"{tag}_path.png"), dpi=130); plt.close(fig)

# 2) 시간축
fig, axs = plt.subplots(4, 1, figsize=(12, 9.5), sharex=True)
tt = T["_t"]; w = T["_win"]
corner = (T["_vref"] < 0.3) & w
for ax in axs:
    ax.fill_between(tt, 0, 1, where=corner, transform=ax.get_xaxis_transform(), color="#eaeef2", step="mid", lw=0)
    ax.grid(color=GRID, lw=0.5)
ax = axs[0]
ax.plot(tt[w], T["_vref"][w], color=C_REF, lw=2, label="기준 속력")
ax.plot(tt[w], np.hypot(T["_X"][w, 4], T["_X"][w, 5]), color=C_EST, lw=1, label="추정기 속력")
ax.set_ylabel("속력 [m/s]"); ax.legend(fontsize=8.5, frameon=False, loc="upper right", ncol=2)
ax.set_title(f"{T['name']} — 회색 띠 = 꼭짓점 구간 (기준 속력 < 0.3 m/s)", loc="left", fontsize=10.5, color=INK)
ax = axs[1]
ax.plot(tt[w], 100 * T["_epos"][w], color=C_EST, lw=1.2, label=f"추정기 |오차| (RMS {T['pos']:.1f})")
ax.plot(T["_ts"], 100 * T["_es"], color=C_SLAM, lw=1.0, ls="--", label=f"SLAM 중심 |오차| (RMS {T['slam_pos']:.1f})")
ax.set_ylabel("위치 오차 [cm]"); ax.legend(fontsize=8.5, frameon=False, loc="upper right", ncol=2)
ax = axs[2]
ax.plot(tt[w], 100 * T["_cross"][w], color="#8250df", lw=1.1, label="옆 (+ 로봇이 기준 왼쪽)")
ax.plot(tt[w], 100 * T["_along"][w], color="#bf8700", lw=1.1, label="진행 (+ 뒤처짐)")
ax.axhline(0, color=INK2, lw=0.6); ax.set_ylabel("경로 좌표 오차 [cm]"); ax.legend(fontsize=8.5, frameon=False, loc="upper right", ncol=2)
ax = axs[3]; tl = T["_tl"]; wsl = T["_ws"]
for i in range(4):
    ax.plot(tl[wsl], T["_Lf"][wsl, i], color=WC[i], lw=0.9, label=f"{WH[i]} (>0.5 {T['p50'][i]:.0f} %)")
for y in (0.5, -0.5):
    ax.axhline(y, color=C_BIG, lw=0.7, ls=":")
ax.axhline(0, color=INK2, lw=0.5)
ax.set_ylim(-2.05, 2.05); ax.set_ylabel("λ, 10 Hz LPF (+ 헛돎, − 끌림)"); ax.set_xlabel("무장 뒤 시간 [s]")
ax.legend(fontsize=8.5, frameon=False, loc="upper right", ncol=4)
ax.set_xlim(tt[w][0] - 0.5, tt[w][-1] + 0.5)
fig.tight_layout(); fig.savefig(os.path.join(HERE, f"{tag}_time.png"), dpi=130); plt.close(fig)

# 3) 비교 막대
if len(res) > 1:
    keys = [("pos", "위치 RMS 추정기 [cm]"), ("slam_pos", "위치 RMS SLAM 중심 [cm]"), ("head", "헤딩 RMS [°]"), ("m50", "큰 슬립 (>0.5) [%]"),
            ("vslip", "미끄럼 속도 RMS [cm/s]"), ("vcorner", "꼭짓점 통과 속력 SLAM [m/s] (기준 0.1)")]
    fig, axs = plt.subplots(2, 3, figsize=(13, 6.5))
    labs = [f"{r['name'].split('_')[0]}\nW{r['A']['W'].split(',')[0]} · {r['A']['sfb'].replace(' (코드 전)', '')}" for r in res]
    cols = [C_BIG if i == 0 else C_EST for i in range(len(res))]
    for ax, (k, ttl) in zip(axs.ravel(), keys):
        v = [r[k] for r in res]
        ax.bar(range(len(res)), [0 if not np.isfinite(x) else x for x in v], color=cols, width=0.65)
        for i, x in enumerate(v):
            ax.text(i, (x if np.isfinite(x) else 0), "-" if not np.isfinite(x) else (f"{x:.2f}" if k == "vcorner" else f"{x:.1f}"), ha="center", va="bottom", fontsize=8, color=INK2)
        ax.set_xticks(range(len(res))); ax.set_xticklabels(labs, fontsize=7.5); ax.set_title(ttl, loc="left", fontsize=10, color=INK)
        ax.grid(axis="y", color=GRID, lw=0.5)
    fig.text(0.01, 0.995, "같은 경로 (polyline · accel 1.5 · v_max 1.5 · pass · friction · 속도 출처 estimator) — 빨강 = 오늘 판, 파랑 = 11-27", fontsize=10.5, color=INK, va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(os.path.join(HERE, "compare.png"), dpi=130); plt.close(fig)
print("그림:", ", ".join(os.path.join(HERE, f) for f in (f"{tag}_path.png", f"{tag}_time.png") + (("compare.png",) if len(res) > 1 else ())))
