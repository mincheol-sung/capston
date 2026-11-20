"""토크 램프 CSV (11-17, torque_ramp) 에서 바퀴별 슬립 시작 — 엔코더만으로. 2026-11-19
무슬립이면 방향 패턴 s (fwd ++++, left −++−) 로 맞춘 바퀴 속도 s_i·ω_i 가 네 바퀴 같다.
한 바퀴가 나머지 셋의 중앙값보다 max(1 rad/s, 15 %) 넘게 빨라져 30 ms 이어지면 그 바퀴 슬립 시작.
그 순간 접지 토크 τ_c = |τ_fb| − (C_i + V_i|ω|) − J_w·|ω̇| (11-16 마찰표, J_w 8.9e-4) → μ_i = τ_c / ((r/√2)(mg/4)).
몸체 수준 슬립 (v_wheel_dir − v_est_dir, IMU) 시작 시각도 같이."""
import sys, csv, numpy as np
W = ("FL", "FR", "RL", "RR"); PAT = {"fwd": np.array([1, 1, 1, 1]), "left": np.array([-1, 1, 1, -1])}
C = np.array([0.0174, 0.0206, 0.0155, 0.0157]); V = np.array([0.001066, 0.001158, 0.001112, 0.001177]); Jw = 8.9e-4
den = 0.04 / np.sqrt(2) * 4.56 * 9.81 / 4
for p in sys.argv[1:]:
    rows = list(csv.DictReader(open(p))); d = "fwd" if "_fwd_" in p else "left"; s = PAT[d]
    t = np.array([float(x["t"]) for x in rows]); om = np.array([[float(x[f"omega_{w}"]) for w in W] for x in rows])
    fb = np.array([[float(x[f"tau_fb_{w}"]) for w in W] for x in rows]); st = np.array([[float(x[f"tau_star_{w}"]) for w in W] for x in rows])
    vw = np.array([float(x["v_wheel_dir"]) for x in rows]); ve = np.array([float(x["v_est_dir"]) for x in rows])
    ph = [x["phase"] for x in rows]
    n = om * s; od = np.gradient(om, t, axis=0)
    first = None
    for i in range(len(t)):
        if ph[i] != "ramp": continue
        for j in range(4):
            others = np.median(np.delete(n[i], j)); thr = max(1.0, 0.15 * abs(others))
            if n[i, j] - others > thr:
                k = i
                while k < len(t) and n[k, j] - np.median(np.delete(n[k], j)) > max(1.0, 0.15 * abs(np.median(np.delete(n[k], j)))): k += 1
                if t[min(k, len(t) - 1)] - t[i] >= 0.03: first = (i, j); break
        if first: break
    bs = np.where((vw - ve > 0.05 + 0.15 * np.abs(vw)) & np.array([q == "ramp" for q in ph]))[0]
    name = p.split("/")[-1]
    if not first: print(f"{name}: 바퀴 슬립 없음 (램프 끝 |τ_fb| {np.round(np.abs(fb[[i for i,q in enumerate(ph) if q=='ramp'][-1]]),3)})"); continue
    i, j = first; tc = np.abs(fb[i]) - (C + V * np.abs(om[i])) - Jw * np.abs(od[i])
    print(f"{name}: 먼저 미끄러진 바퀴 {W[j]} @ t {t[i]:.2f} s  |τ*| {np.abs(st[i,j]):.3f}  |τ_fb| {np.round(np.abs(fb[i]),3)}  ω {np.round(om[i],1)}  "
          f"접지 τ_c {np.round(tc,3)} → μ {np.round(tc/den,2)}  (몸체 슬립 판정 {('t %.2f s' % t[bs[0]]) if len(bs) else '없음'})")
