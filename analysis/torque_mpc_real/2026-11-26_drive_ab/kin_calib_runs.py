# kin_calib.py 의 창 계산을 판마다 — 직선 δ 가 판·묶음 (rr 끔 · rr 켬 · dy 끔 · dy 켬) 끼리 고른지
import os, glob, runpy, io, contextlib
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "kin_calib.py")).read()
head = src[:src.index("R = np.array(rows)")]
ns = {"__file__": os.path.join(HERE, "kin_calib.py")}
out = {}
exec(compile(head.replace("    for t0 in np.arange", "    out_lab = lab\n    for t0 in np.arange").replace("        rows.append([", "        rows.append([out_lab, "), "kc", "exec"), ns)
rows = ns["rows"]
labs = sorted({r[0] for r in rows})
for lab in labs:
    R = np.array([r[1:] for r in rows if r[0] == lab], float)
    d, ac, w = R[:, 0], R[:, 1], R[:, 3]
    st = (np.abs(d) < 15) & (w < 0.3); tl = (np.abs(d) < 15) & (w >= 0.3) & (ac > 0); trr = (np.abs(d) < 15) & (w >= 0.3) & (ac < 0)
    print(f"{lab}: 직선 δ 중앙 {np.median(d[st]):+.2f}° ({st.sum()})  왼쪽 선회 {np.median(d[tl]):+.2f}° ({tl.sum()})  오른쪽 선회 {np.median(d[trr]):+.2f}° ({trr.sum()})")
