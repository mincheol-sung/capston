# 확인 주행 (MPC, 상한 옮김·마찰 모델·외란 추정 켬) 한 판 읽기:  /usr/bin/python3 check_run.py <bag>  → 숫자 출력 + check_run.npz
import os, sys, math, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
np.set_printoptions(precision=4, suppress=True, linewidth=200)
path = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
want = {"/torque_mpc/status":"st", "/torque_mpc/cap":"cap", "/torque_mpc/cmd_raw":"mpc", "/torque_output/cmd_raw":"out", "/wheel_torque_cmd":"mot",
        "/wheel_states":"ws", "/state_estimator/odom":"od", "/tracked_pose":"sl", "/mpc/armed":"arm", "/rosout":"log", "/slip_logger/slip":"slip",
        "/dra_mppi/reference_trajectory":"ref"}
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
d = {v: [] for v in want.values()}
t0 = None
while r.has_next():
    name, data, ns = r.read_next()
    if name not in types: continue
    if t0 is None: t0 = ns
    t = (ns - t0) * 1e-9
    m = deserialize_message(data, types[name]); k = want[name]
    if k in ("st", "cap", "slip"): d[k].append([t, *m.data])
    elif k in ("mpc", "out", "mot"):
        tq = list(m.torque) if len(m.torque) == 4 else [math.nan]*4
        d[k].append([t, float(m.mode), *tq])
    elif k == "ws": d[k].append([t, *m.position, *m.velocity, *m.torque, *m.temperature])
    elif k == "od":
        q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        d[k].append([t, p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z])
    elif k == "sl": d[k].append([t, m.pose.position.x, m.pose.position.y])
    elif k == "arm": d[k].append([t, float(m.data)])
    elif k == "log":
        if m.level >= 30: d[k].append((t, m.level, m.name, m.msg))
    elif k == "ref":
        if m.references:
            p = m.references[0]; d[k].append([t, p.x, p.y, p.vx, p.vy, len(m.references)])
A = {k: (np.array(v, float) if k != "log" and v else v) for k, v in d.items()}
print("armed 이벤트:", A["arm"])
arm = A["arm"]; ta = arm[arm[:,1] > 0.5, 0]; td = arm[arm[:,1] < 0.5, 0]
t_on = ta[0] if len(ta) else None
t_off = td[td > t_on][0] if t_on is not None and (td > t_on).any() else A["st"][-1,0]
print(f"무장 구간 {t_on:.2f} ~ {t_off:.2f} s ({t_off-t_on:.2f} s)")
st = A["st"]; ms = (st[:,0] >= t_on) & (st[:,0] <= t_off); S = st[ms]
print(f"status 행 {len(st)}, 무장 중 {len(S)}, 열 수 {st.shape[1]-1}")
D = S[:, 1:]   # D[:, idx]
print(f"추종 오차 ex ey [mm] RMS {1e3*np.sqrt(np.mean(D[:,1]**2)):.1f} {1e3*np.sqrt(np.mean(D[:,2]**2)):.1f}  최대 {1e3*np.abs(D[:,1]).max():.1f} {1e3*np.abs(D[:,2]).max():.1f}   eθ [deg] RMS {np.degrees(np.sqrt(np.mean(D[:,3]**2))):.2f} 최대 {np.degrees(np.abs(D[:,3]).max()):.2f}")
print(f"위치 오차 크기 RMS {1e3*np.sqrt(np.mean(D[:,1]**2+D[:,2]**2)):.1f} mm 최대 {1e3*np.hypot(D[:,1],D[:,2]).max():.1f} mm")
print(f"활성 제약 수: 최대 {D[:,9].max():.0f}, >0 인 스텝 {100*np.mean(D[:,9]>0):.1f} %   반복 최대 {D[:,10].max():.0f}   fallback 스텝 {int((D[:,11]>0).sum())} / {len(D)}   계산 ms 평균 {D[:,12].mean():.2f} 최대 {D[:,12].max():.2f}   ref_age 최대 {D[:,13].max():.3f}")
print(f"τ̄ (status 8): {np.unique(np.round(D[:,8],4))[:5]}")
print(f"MPC τ [FL FR RL RR] 최소 {D[:,4:8].min(0)} 최대 {D[:,4:8].max(0)}")
print(f"d̂ (14–19) 끝 {D[-1,14:20]}  최대|.| {np.abs(D[:,14:20]).max(0)}")
if D.shape[1] >= 26:
    print(f"hold {np.unique(D[:,20])} stopped {np.unique(D[:,21])}")
cap = A["cap"]; mc = (cap[:,0] >= t_on) & (cap[:,0] <= t_off); C = cap[mc][:, 1:]
print(f"cap 행 {len(C)} 열 {C.shape[1]}: shift on {np.unique(C[:,1])}, model on {np.unique(C[:,14])}")
print(f"  상한 옮김량 f [FL FR RL RR] 최소 {C[:,2:6].min(0)} 최대 {C[:,2:6].max(0)}")
print(f"  아래 한계 lo 범위 {C[:,6:10].min(0)} ~ {C[:,6:10].max(0)}")
print(f"  위 한계  hi 범위 {C[:,10:14].min(0)} ~ {C[:,10:14].max(0)}")
print(f"  모델 마찰 τ_b 최소 {C[:,15:19].min(0)} 최대 {C[:,15:19].max(0)}")
n = min(len(C), len(D))
tau = D[:n,4:8]; lo = C[:n,6:10]; hi = C[:n,10:14]
print(f"  τ 가 위 한계에 닿은 스텝 % {100*np.mean(tau >= hi-1e-6, axis=0)}  아래 한계 % {100*np.mean(tau <= lo+1e-6, axis=0)}  한계 밖 % {100*np.mean((tau>hi+1e-6)|(tau<lo-1e-6), axis=0)}")
# 구간 나눔: 바퀴 속도로
ws = A["ws"]; mw = (ws[:,0] >= t_on) & (ws[:,0] <= t_off); W = ws[mw]
om = W[:,5:9]; omb = om.mean(1)
print(f"바퀴 속도 평균 최대 {omb.max():.2f} rad/s (0.2 m/s = 5.0), 온도 {np.nanmean(W[:,13:17],0)}")
cr = np.abs(omb - 5.0) < 0.5
if cr.any():
    idx = np.flatnonzero(cr); tc0, tc1 = W[idx[0],0], W[idx[-1],0]
    print(f"등속 (5±0.5 rad/s) 구간 {tc0:.2f}~{tc1:.2f} s ({tc1-tc0:.2f} s)")
    tc0 += 0.4
    sel = (W[:,0] >= tc0) & (W[:,0] <= tc1)
    mot = A["mot"]; mm = (mot[:,0] >= tc0) & (mot[:,0] <= tc1) & (mot[:,1] == 0)
    mp = A["mpc"]; mmp = (mp[:,0] >= tc0) & (mp[:,0] <= tc1) & (mp[:,1] == 0)
    out = A["out"]; mo = (out[:,0] >= tc0) & (out[:,0] <= tc1) & (out[:,1] == 0)
    sc = (S[:,0] >= tc0) & (S[:,0] <= tc1); cc = (cap[mc][:,0] >= tc0) & (cap[mc][:,0] <= tc1)
    print(f"  바퀴 속도 평균 {W[sel,5:9].mean(0)} 표준편차 {W[sel,5:9].std(0)}")
    print(f"  MPC 명령 평균      {mp[mmp,2:6].mean(0)}")
    print(f"  torque_output 평균 {out[mo,2:6].mean(0)}")
    print(f"  모터 명령 평균     {mot[mm,2:6].mean(0)}")
    print(f"  피드백 토크 평균   {W[sel,9:13].mean(0)}")
    print(f"  status τ 평균      {D[sc,4:8].mean(0)}   목표 입력 U_s 평균 {D[sc,22:26].mean(0) if D.shape[1]>=26 else None}")
    print(f"  cap f 평균 {C[cc,2:6].mean(0)}  τ_b 평균 {C[cc,15:19].mean(0)}  lo {C[cc,6:10].mean(0)} hi {C[cc,10:14].mean(0)}")
    tc = np.array([0.0174,0.0206,0.0155,0.0157]); bv = np.array([0.001066,0.001158,0.001112,0.001177])
    wm = W[sel,5:9].mean(0); lift = tc + bv*wm
    print(f"  띄운 상태 마찰 (설정, 이 속도) {lift}  → 모터 명령 − 그 값 = {mot[mm,2:6].mean(0)-lift}  (바닥 구름 손실 첫 점, 4 바퀴 평균 {1e3*(mot[mm,2:6].mean(0)-lift).mean():.1f} mN·m)")
    od = A["od"]; so = (od[:,0] >= tc0) & (od[:,0] <= tc1)
    print(f"  추정기 속도 평균 {np.hypot(od[so,4],od[so,5]).mean():.3f} m/s, 바퀴 속도×r {0.04*wm.mean():.3f}")
sl = A["sl"]; p0 = sl[(sl[:,0] > t_on-1.0) & (sl[:,0] < t_on), 1:3].mean(0); p1 = sl[sl[:,0] > t_off-0.3, 1:3][:5].mean(0)
print(f"SLAM 이동 거리 (무장 전 → 해제 무렵) {np.hypot(*(p1-p0)):.3f} m,  바퀴 굴린 거리 {0.04*(W[-1,1:5]-W[0,1:5])}")
od = A["od"]; o0 = od[(od[:,0] > t_on-0.5) & (od[:,0] < t_on)]; o1 = od[(od[:,0] > t_off-0.3) & (od[:,0] < t_off)]
print(f"추정기 yaw 변화 {np.degrees(o1[:,3].mean()-o0[:,3].mean()):.2f} deg, 옆 이동 (출발 방향 기준) ", end="")
dx, dy = o1[:,1].mean()-o0[:,1].mean(), o1[:,2].mean()-o0[:,2].mean(); y0 = o0[:,3].mean()
print(f"앞 {dx*math.cos(y0)+dy*math.sin(y0):.3f} m 옆 {-dx*math.sin(y0)+dy*math.cos(y0):.4f} m")
mot = A["mot"]; mm = (mot[:,0] >= t_on) & (mot[:,0] <= t_off) & (mot[:,1] == 0)
print(f"모터 명령 |τ| 최대 {np.nanmax(np.abs(mot[mm,2:6]),0)}  피드백 |τ| 최대 {np.abs(W[:,9:13]).max(0)}")
slp = A["slip"]; msl = (slp[:,0] >= t_on) & (slp[:,0] <= t_off); L = slp[msl][:, 2:6]
mov = np.abs(W[:,5:9].mean(1)) > 1.0
print(f"슬립 λ (slip_logger, 무장 중 {len(L)} 표본): |λ|>0.5 % {100*np.nanmean(np.abs(L)>0.5,0)}  |λ|>0.25 % {100*np.nanmean(np.abs(L)>0.25,0)}  RMS {np.sqrt(np.nanmean(L**2,0))}")
print("경고·오류 로그:")
seen = {}
for t, lv, nm, msg in A["log"]:
    key = (nm, msg[:60])
    seen.setdefault(key, [0, t, lv, msg]); seen[key][0] += 1
for (nm, _), (n, t, lv, msg) in seen.items():
    print(f"  [{t:6.2f}s ×{n}] {'WARN' if lv==30 else 'ERROR'} {nm}: {msg[:230]}")
np.savez(os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_run.npz"), **{k: v for k, v in A.items() if k != "log" and len(v)}, t_on=t_on, t_off=t_off)
