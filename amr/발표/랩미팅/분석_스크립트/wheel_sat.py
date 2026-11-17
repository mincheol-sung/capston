import sys, numpy as np
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
from plot_slip import read_bag, arrays, armed_spans
for b in sys.argv[1:]:
    S, W, O, A = arrays(read_bag(b)); s0, s1 = armed_spans(A, S[-1, 0])[0]
    m = (W[:, 0] >= s0) & (W[:, 0] <= s1); w = W[m, 1:5]
    flips = [int(np.sum(np.abs(np.diff(np.sign(w[np.abs(w[:, i]) > 20, i]))) > 0)) for i in range(4)]
    ms = (S[:, 0] >= s0) & (S[:, 0] <= s1)
    diff = np.abs(S[ms, 1:5] * 0)  # placeholder
    wn = S[ms, 5:9]
    print(f"{b.split('/')[-1]}: arm {s1-s0:.1f} s, |ω| 최대 " + " ".join(f"{np.max(np.abs(w[:, i])):.1f}" for i in range(4))
          + f" rad/s, |ω|>38 비율 {100*np.mean(np.abs(w) > 38):.1f} %, |ω_nom| 최대 {np.max(np.abs(wn)):.1f}, ±20 넘는 구간 부호 뒤집힘 {flips}")
