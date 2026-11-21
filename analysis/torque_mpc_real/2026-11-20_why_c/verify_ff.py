#!/usr/bin/python3
"""앞먹임 톱니 검증: 직선 가속 구간 (기준 가속 일정 1.5, 헤딩 회전 없음) 에서
  측정 비율 ρ = |U_s| / U_s,필요 (U_s,필요 = (m+m_w)·a·r/4, 네 바퀴 같음) 과 status 의 ref_age = t_state − t_0 (기준열 첫 행 시각) 의 관계.
  가설: lo = t − acc_dt/2 가 t_0 보다 앞이면 첫 행 속도로 상수 외삽 → p̈ = (v(t+h) − v(t_0)) / (2h) = a (h + ref_age) / (2h) → ρ = 0.5 + ref_age / 0.2."""
import importlib.util, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", os.path.join(HERE, "why_c.py")); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
r, m, Jw = 0.04, 4.56, 8.9e-4; mw = 4 * Jw / r**2
for bag in ("190506", "171531"):
    S, E, P, A = w.load(f"{WS}/bags/2026-11-20/{bag}"); D = cs.load(f"{WS}/bags/2026-11-20/{bag}")
    t = S[:, 0]; us = S[:, 22:26]; age = S[:, 13]; hold = S[:, 20] > 0.5
    rt = np.array([a for a, _ in D["ref"]])
    # 참 기준 가속: 같은 기준열 메시지 안의 행들(외삽 아닌 구간)로 v(t+0.15) − v(t+0.05)
    a_true, om = [], []
    for x in t:
        i = max(0, np.searchsorted(rt, x, side="right") - 1); R = D["ref"][i][1]
        v1 = np.hypot(*R.at(x + 0.15)[1][:2]); v0 = np.hypot(*R.at(x + 0.05)[1][:2]); a_true.append((v1 - v0) / 0.1); om.append(abs(R.at(x)[1][2]))
    a_true, om = np.array(a_true), np.array(om)
    sel = cs.armed_mask(t, A) & ~hold & (a_true > 1.3) & (om < 0.05) & (np.abs(us).max(1) > 0.01)
    need = (m + mw) * a_true * r / 4
    rho = np.abs(us[sel]).mean(1) / need[sel]
    pred = 0.5 + np.clip(age[sel] - 0.05, 0, 0.1) / 0.2          # 첫 행 = 헤더 시각 + dt 0.05
    c = np.corrcoef(rho, age[sel])[0, 1]
    print(f"{bag}: 직선 가속 표본 {sel.sum()}, ref_age 범위 {age[sel].min()*1e3:.0f}~{age[sel].max()*1e3:.0f} ms (중앙 {np.median(age[sel])*1e3:.0f})")
    print(f"   측정 ρ 평균 {rho.mean():.2f} [{rho.min():.2f}~{rho.max():.2f}],  가설 0.5+(age−0.05)/0.2 평균 {pred.mean():.2f},  ρ 와 ref_age 상관 r = {c:.2f},  |ρ − 가설| 평균 {np.abs(rho-pred).mean():.3f}")
