#!/usr/bin/python3
"""한 바퀴만 헛도는 순간 찾기 — SLAM 없이 엔코더 4 개 + 자이로.
  무슬립이면 y_i = r ω_i − c_i k ω_z = v_x + b_i v_y  (b = [−1, 1, 1, −1], c = [−1, 1, −1, 1], ω_z = 자이로).
  바퀴 i 를 빼고 나머지 셋으로 (v_x, v_y) 최소제곱 → 셋의 잔차 RMS < RES 이고 i 의 어긋남 |d_i| > DEV 가 HOLD 이어지면 i 가 헛돎.
  (모든 바퀴가 같이 미끄러지거나 몸체가 통째로 밀리면 안 잡힌다 — 한 바퀴씩만.)
  접지 토크 τ_c = |τ_fb| − (C_i + V_i|ω_i|) − J_w|ω̇_i| (시작 30 ms 전), μ_i = τ_c / ((r/√2)(m g/4)). 구동 = d_i 와 τ_fb 부호가 같음."""
import importlib.util, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("se", os.path.join(HERE, "slip_events.py")); se = importlib.util.module_from_spec(spec); spec.loader.exec_module(se)
r, k, C, V, Jw, UNIT, W = se.r, se.k, se.C, se.V, se.Jw, se.UNIT, se.W
b = np.array([-1, 1, 1, -1.0]); c = np.array([-1, 1, -1, 1.0])
DEV, RES, HOLD = 0.15, 0.04, 0.03          # [m/s] 바퀴 표면 속도 기준 (0.15 m/s = 3.75 rad/s)
out = []
for bag in sys.argv[1:]:
    ws, P, G, arm = se.load(bag.rstrip("/"))
    t = ws[:, 0]; om = ws[:, 1:5]; tfb = ws[:, 5:9]; fresh = (ws[:, 9:13] > 0.5).all(1)
    wz = np.interp(t, G[:, 0], G[:, 1]); y = r * om - c[None, :] * k * wz[:, None]
    dev = np.zeros_like(y); res = np.zeros_like(y)
    for i in range(4):
        o = [q for q in range(4) if q != i]; A = np.stack([np.ones(3), b[o]], 1); pinv = np.linalg.pinv(A)
        sol = (pinv @ y[:, o].T).T                                     # (N, 2) v_x, v_y
        fit = sol @ A.T; res[:, i] = np.sqrt(np.mean((y[:, o] - fit) ** 2, 1))
        dev[:, i] = y[:, i] - (sol[:, 0] + b[i] * sol[:, 1])
    ok = fresh & se.cs.armed_mask(t, arm)
    dom = np.gradient(om, t, axis=0); n0 = len(out)
    for i in range(4):
        flag = ok & (np.abs(dev[:, i]) > DEV) & (res[:, i] < RES) & (np.abs(dev[:, i]) == np.abs(dev).max(1))
        j, last = 0, -1e9
        while j < len(t):
            if flag[j] and t[j] > last + 0.5:
                e = j
                while e < len(t) and flag[e]:
                    e += 1
                if t[min(e, len(t) - 1)] - t[j] >= HOLD:
                    p = max(0, np.searchsorted(t, t[j] - 0.03))
                    tc = abs(tfb[p, i]) - (C[i] + V[i] * abs(om[p, i])) - Jw * abs(dom[p, i])
                    out.append(dict(bag=os.path.basename(bag.rstrip("/")), wheel=W[i], t=float(t[j]), dev_peak=float(np.abs(dev[j:e, i]).max()),
                                    dur=float(t[min(e, len(t) - 1)] - t[j]), tau_fb=float(abs(tfb[p, i])), tau_c=float(tc), mu=float(tc / UNIT),
                                    drive=bool(np.sign(dev[j, i]) == np.sign(tfb[p, i])), omega=float(om[p, i])))
                    last = t[min(e, len(t) - 1)]
                j = e
            else:
                j += 1
    print(f"{os.path.basename(bag.rstrip('/'))}: 한 바퀴 헛돎 {len(out) - n0} 건  (CAN 응답 다 온 표본 {100*fresh.mean():.0f} %)")
json.dump(out, open(os.path.join(HERE, "single_wheel_slip.json"), "w"), ensure_ascii=False, indent=1)
print("\nbag     바퀴 어긋남최대[m/s] 지속[s] |τ_fb| τ_c   μ     구동? ω")
for x in sorted(out, key=lambda x: (x["wheel"], -x["dev_peak"])):
    print(f"{x['bag']} {x['wheel']}  {x['dev_peak']:.2f}  {x['dur']:.2f}  {x['tau_fb']:.3f} {x['tau_c']:+.3f} {x['mu']:+.3f}  {'구동' if x['drive'] else '제동'} {x['omega']:+5.1f}")
print("\n바퀴별 (구동 헛돎만, μ > 0): 중앙값 [범위] 건수")
for w in W:
    mu = np.array([x["mu"] for x in out if x["wheel"] == w and x["drive"] and x["mu"] > 0])
    print(f"  {w}: " + (f"{np.median(mu):.3f} [{mu.min():.3f} ~ {mu.max():.3f}] {len(mu)} 건" if len(mu) else "없음"))
