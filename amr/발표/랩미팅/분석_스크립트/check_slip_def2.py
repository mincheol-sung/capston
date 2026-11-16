# 슬립 행의 t = WheelStates header.stamp → 같은 메시지의 ω 와 행의 ω_nom 으로 λ 식을 다시 계산
import sys, numpy as np
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools")
from plot_slip import read_bag
raw = read_bag(sys.argv[1])
wmap = {round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 6): np.array(m.velocity) for _, m in raw["/wheel_states"]}
rows = np.array([m.data[:9] for _, m in raw["/slip_logger/slip"]])
hit = [round(t, 6) in wmap for t in rows[:, 0]]
R = rows[np.array(hit)]
w = np.array([wmap[round(t, 6)] for t in R[:, 0]]); wn = R[:, 5:9]
lam = (w - wn) * np.where(wn >= 0, 1, -1) / np.maximum(np.maximum(np.abs(w), np.abs(wn)), 2.5)
d = np.abs(lam - R[:, 1:5]); f = np.isfinite(R[:, 1:5])
print(f"짝지은 행 {len(R)}/{len(rows)}, λ 식 재계산 차이 최대 {d[f].max():.1e}")
