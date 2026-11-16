import sys, numpy as np
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
from plot_slip import read_bag, arrays, armed_spans
S, W, O, A = arrays(read_bag(sys.argv[1]))
sp = armed_spans(A, S[-1, 0]); s0, s1 = sp[0]
v = np.interp(S[:, 0], O[:, 0], np.hypot(O[:, 1], O[:, 2]))
arm = (S[:, 0] >= s0) & (S[:, 0] <= s1)
print(f"arm 중 |v̂| 최대 {v[arm].max():.2f} m/s, 순항(t 6~13 s) 평균 {v[arm & (S[:,0]-s0>6) & (S[:,0]-s0<13)].mean():.2f} m/s")
print("|v̂| 구간 [m/s]   행수   RMS λ (FL FR RL RR)          |λ|>0.1 % (4 바퀴 합)")
for lo, hi in [(0.05, 0.3), (0.3, 0.6), (0.6, 0.9), (0.9, 1.2)]:
    m = arm & (v >= lo) & (v < hi)
    L = S[m, 1:5]
    rms = [np.sqrt(np.nanmean(L[:, i] ** 2)) for i in range(4)]
    print(f"  {lo:.2f}–{hi:.2f}   {m.sum():5d}   " + " ".join(f"{x:.2f}" for x in rms) + f"   {100*np.nanmean(np.abs(L) > 0.1):5.1f}")
