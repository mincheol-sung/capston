#!/usr/bin/python3
"""B 판 랩 2 원 후반 (위상 0.5~0.95) 정상상태의 안쪽 힘: 모델이 필요하다는 m v_r|ω_r| (use_w 켬 → c_n = m v ω) vs U_s 의 안쪽 성분 vs 낸 τ 의 안쪽 성분
   vs 참값 궤적의 실제 m·a_n. 안쪽 = 몸체 y × sign(ω_r) (tangent yaw, 요 오차 2.5° 무시). 힘 [N]."""
import importlib.util, sys
import numpy as np
HERE = "/home/smoc/capston/capston/analysis/torque_mpc_sim/2026-11-21_bias_source"; WS = "/home/smoc/capston/capston"
spec = importlib.util.spec_from_file_location("cs", f"{WS}/src/amr_basic_stack/controller/mpc/tools/compare_slam.py")
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
r_w, k, m, mu_w = 0.04, 0.1932, 4.56, 4 * 8.9e-4 / 0.04 ** 2; m_eff = m + mu_w
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r_w
for lab in sys.argv[1:] or ["B4_r1", "B3_r1", "B0_r1"]:
    z = np.load(f"{HERE}/runs/{lab}.npz"); raw = z["raw"]; st = z["st"]; t = raw[:, 0]
    et, ec, en, ey, sp = cs.path_errors(*[raw[:, i] for i in range(1, 10)]); om = raw[:, 9]
    mv = sp > 0.02; T0 = t[np.argmax(mv)]; tr = t - T0
    sg = np.sign(om); sw = np.flatnonzero(np.abs(np.diff(sg)) > 0)
    ph = np.full(len(t), np.nan)
    for a, b in zip(sw[:-1], sw[1:]):
        ph[a:b] = (t[a:b] - t[a]) / (t[b] - t[a])
    sel = mv & (tr >= 9.0) & (tr < 15.2) & (ph > 0.5) & (ph < 0.95) & (np.abs(om) > 0.3)
    # 참값 실제 구심: 위치 2 차 차분 → 기준 법선 쪽
    dt = np.gradient(t); vx = np.gradient(raw[:, 1]) / dt; vy = np.gradient(raw[:, 2]) / dt
    ax = np.gradient(vx) / dt; ay = np.gradient(vy) / dt
    thr = raw[:, 6]; c, s = np.cos(thr), np.sin(thr)
    nx, ny = -s * np.sign(om), c * np.sign(om)                    # 기준 헤딩 법선, 곡률 중심 쪽 (v_y_ref ≈ 0)
    a_n = ax * nx + ay * ny; v_act = np.hypot(vx, vy)
    # status 를 raw 시각으로
    ts = st[:, 0]; tau = st[:, 4:8]; us = st[:, 22:26]
    j = np.clip(np.searchsorted(ts, t), 0, len(ts) - 1)
    yaw = raw[:, 3]
    def inward(T):
        Fb = (F @ T[j].T).T                                         # 몸체 힘
        return Fb[:, 1] * np.sign(om)                                # 몸체 y × 부호 (tangent yaw)
    need = m * raw[:, 7] * np.abs(om)                                # m v_r |ω_r|
    Fus, Ftau = inward(us), inward(tau)
    act = m * a_n                                                    # 바퀴가 낸 안쪽 힘 (w 몫 m_w a_n 은 바퀴 관성이 냄)
    print(f"== {lab}  랩 2 원 후반 {sel.sum()} 표본: e_c {1e3*ec[sel].mean():+.1f} mm, v_r {raw[sel,7].mean():.3f} v_실제 {v_act[sel].mean():.3f}, |ω_r| {np.abs(om[sel]).mean():.3f}")
    print(f"   모델 필요 m v_r|ω_r| {need[sel].mean():.3f} N   U_s 안쪽 {Fus[sel].mean():.3f} N ({Fus[sel].mean()/need[sel].mean():.3f} 배)   "
          f"낸 τ 안쪽 {Ftau[sel].mean():.3f} N ({Ftau[sel].mean()/need[sel].mean():.3f} 배)   참값 m·a_n {act[sel].mean():.3f} N ({act[sel].mean()/need[sel].mean():.3f} 배)")
    print(f"   참값 실제 반경 v²/a_n {np.mean(v_act[sel]**2/a_n[sel]):.4f} m (기준 0.3)   e_t {1e3*et[sel].mean():+.1f} mm   요 오차 {np.degrees(ey[sel].mean()):+.2f}°")
