# detect_offline.py 의 σ 에 규칙: 30 ms 이동평균 → 네 바퀴 중 σ 가 가장 큰 바퀴만 후보 → σ > 문턱이 지속 (min_on) 이면 "슬립".
import sys, numpy as np
sys.argv_saved = sys.argv; sys.argv = [sys.argv[0]]
exec(open("analysis/torque_mpc_real/2026-11-21_slip_guard/detect_offline.py").read().split('if __name__')[0])
sys.argv = sys.argv_saved
def rule(t, sig, th=float(__import__("os").environ.get("TH", 0.3)), win=3, min_on=int(__import__("os").environ.get("MINON", 3))):
    s = np.array([np.convolve(sig[:,i], np.ones(win)/win, "same") for i in range(4)]).T
    am = s.argmax(1); flag = np.zeros_like(s, bool)
    cand = s[np.arange(len(s)), am] > th
    run = 0
    for n in range(len(s)):
        run = run + 1 if cand[n] and (n == 0 or am[n] == am[n-1]) else (1 if cand[n] else 0)
        if run >= min_on: flag[n, am[n]] = True
    return flag
print("bag | 설명 | 주행 시간 | 슬립 판정 시간 % [FL FR RL RR] | 첫 판정")
for b, nm in [x.split(":") for x in sys.argv[1:]]:
    try:
        t, sig, E, T = run(b)
    except StopIteration:
        print(b, "무장 안 됨"); continue
    f = rule(t, sig)
    firsts = [(t[np.argmax(f[:,i])], i) for i in range(4) if f[:,i].any()]
    fi = min(firsts) if firsts else None
    print(f"{b} | {nm} | {T:.1f} s | {(100*f.mean(0)).round(1)} | " + (f"{'FL FR RL RR'.split()[fi[1]]} {fi[0]:+.2f} s" if fi else "없음"))
