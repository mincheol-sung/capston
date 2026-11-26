# 2026-11-25 옆걸음 단계 토크 실차 요약: 방향별 · 옆걸음 중 회전이 큰 회를 뺀 맞춤 · 회전 방향 (어느 쪽으로 도나)
#   /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_lat/summary.py
import glob, importlib.util, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.realpath(os.path.join(HERE, "..", "..", ".."))
spec = importlib.util.spec_from_file_location("fw", os.path.join(WS, "src/amr_basic_stack/controller/mpc/tools/floor_wheel_tests.py"))
fw = importlib.util.module_from_spec(spec); spec.loader.exec_module(fw)
pv, lg = fw.plant_values(), fw.lat_geometry()
R = []
for b in sorted(glob.glob(os.path.join(WS, "bags/2026-11-25/17[34]*_lat_*"))):
    for a in fw.analyze_lats(fw.load_bag(b), pv, lg):
        a["name"] = os.path.basename(b); R.append(a)
R = [a for a in R if abs(a["v_start"]) < 0.05 and np.isfinite(a["F_res"])]
print(f"서 있다가 시작한 회 {len(R)} 개")
print("| 토크 | 방향 | 회 | 옆 저항 N 중앙 (사분위) | 회전 ° 중앙 (부호 = 반시계 +) | |회전| > 15° 회 | 끝 바퀴 속도 FL FR RL RR 중앙 |")
for lv in (0.08, 0.10, 0.12):
    for sg, nm in ((1.0, "왼쪽"), (-1.0, "오른쪽")):
        G = [a for a in R if abs(abs(a["amp"]) - lv) < 0.004 and a["sg"] == sg]
        if not G: continue
        f = np.array([a["F_res"] for a in G]); d = np.array([a["dyaw"] for a in G])
        w = np.nanmedian([a["w_end"] for a in G], axis=0)
        print(f"| {lv:.2f} | {nm} | {len(G)} | {np.median(f):.2f} ({np.percentile(f, 25):.2f}~{np.percentile(f, 75):.2f}) | {np.median(d):+.1f} | {int(np.sum(np.abs(d) > 15))} | {' '.join(f'{x:.1f}' for x in w)} |")
for lim in (1e9, 15.0, 5.0):
    S = [a for a in R if abs(a["dyaw"]) <= lim]
    x, se, rms, n = fw.fit_lat_runs(S)
    fixed = np.median([a["F_res"] for a in S])
    print(f"|회전| ≤ {lim if lim < 1e8 else '전부'}: 회 {n}, 맞춤 m {x[0]:.2f} ± {se[0]:.2f}  F_c {x[1]:.2f} ± {se[1]:.2f} N, m_eff 고정 중앙 {fixed:.2f} N")
d = np.array([a["dyaw"] for a in R]); f = np.array([a["F_res"] for a in R])
print(f"옆 저항 ~ |회전| 상관 {np.corrcoef(np.abs(d), f)[0, 1]:+.2f}")
print(f"롤러 가설 역방향: F_c × l_x = {4.99 * lg['lx']:.3f} N·m (회전 시험 0.37~0.44)")
