# 2026-11-25 회전 단계 토크 실차 요약: 토크별 묶음 · 미분 창 길이에 따른 관성 (regressor 잡음 감쇠 확인) · 짧은 맞춤 비교
#   /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_rot/summary.py
import glob, importlib.util, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.realpath(os.path.join(HERE, "..", "..", ".."))
spec = importlib.util.spec_from_file_location("fw", os.path.join(WS, "src/amr_basic_stack/controller/mpc/tools/floor_wheel_tests.py"))
fw = importlib.util.module_from_spec(spec); spec.loader.exec_module(fw)
pv, rg = fw.plant_values(), fw.rot_geometry()
bags = sorted(b for b in glob.glob(os.path.join(WS, "bags/2026-11-25/*_rot_*")) if not b.endswith(".tmp")
              and os.path.basename(b)[:6] >= "171650")
runs = {os.path.basename(b): fw.load_bag(b) for b in bags}

def rows_for(sg_s=0.08):
    out = []
    for name, run in runs.items():
        wins = fw.step_windows(run); bias = fw.gyro_rest_bias(run)
        for k, w in enumerate(wins):
            a = fw.analyze_rot(run, pv, rg, w, bias=bias, sg_s=sg_s, t_next=wins[k + 1][0] if k + 1 < len(wins) else None)
            if a is not None:
                a["name"], a["k"] = name, k + 1
                out.append(a)
    return out

R = rows_for()
print(f"회 {len(R)} 개 (판 {len(runs)} 개)")
print("\n토크별 (중앙값, 시작 때 돌고 있던 회 |ψ̇| > 0.2 는 따로 셈)")
print("| 바퀴당 토크 | 방향 | 회 | 시작 때 돌던 회 | 온도 °C | 출발까지 s | 평균 가속 측정 · 모델 rad/s² (비) | 최고 회전 속도 | 자이로/바퀴 | 끊은 뒤 더 돈 각 ° |")
for lv in (0.06, 0.08, 0.10):
    for sg in (1.0, -1.0):
        G = [a for a in R if abs(abs(a["yaw_amp"]) - lv) < 0.004 and a["sg"] == sg]
        if not G: continue
        rest = [a for a in G if abs(a["w_start"]) < 0.2]
        md = lambda k, X=rest: np.nanmedian([a[k] for a in X])
        print(f"| {lv:.2f} | {'반시계' if sg > 0 else '시계'} | {len(G)} | {len(G) - len(rest)} | {md('temp') if False else np.nanmedian([np.nanmean(a['temp']) for a in rest]):.0f} | {md('breakaway'):.3f} | "
              f"{abs(md('alpha_fit')):.2f} · {abs(md('alpha_model')):.2f} ({md('alpha_fit') / md('alpha_model'):.2f}) | {md('w_peak'):.2f} | {md('eff'):.3f} | {abs(md('ang_total') - md('ang_torque')):.1f} |")

print("\n미분 창 길이별 표본 맞춤 (잡음 섞인 가속을 설명 변수로 쓰면 관성이 0 쪽으로 줄어든다 — errors-in-variables)")
for s in (0.04, 0.08, 0.15, 0.25):
    Rs = R if s == 0.08 else rows_for(s)
    x, se, rms, n = fw.fit_rot(Rs, "M"); x2, *_ = fw.fit_rot(Rs, "M", with_b=False)
    print(f"  창 {s:.2f} s: I_eff {x[0]:.4f}  M_c {x[1]:.3f}  B {x[2]:+.4f} | B 없이 I_eff {x2[0]:.4f} M_c {x2[1]:.3f}  (표본 {n})")
for lab, sel in (("전부", R), ("서 있다가 시작한 회만", [a for a in R if abs(a["w_start"]) < 0.2])):
    for wb in (True, False):
        f = fw.fit_rot_runs(sel, with_b=wb)
        x, se, rms, n = f
        print(f"  회마다 평균 ({lab}, {'B 있음' if wb else 'B 없이'}): I_eff {x[0]:.4f} ± {se[0]:.4f}  M_c {x[1]:.3f} ± {se[1]:.3f}"
              + (f"  B {x[2]:+.4f} ± {se[2]:.4f}" if wb else "") + f"  잔차 {rms:.3f}  회 {n}")
