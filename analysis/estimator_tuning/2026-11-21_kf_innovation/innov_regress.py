#!/usr/bin/python3
"""주행 innovation 에 결정적(색이 있는) 성분이 있나: 진행 방향 성분 ν_along 을 속도 |v| 에 회귀 → 기울기 = 포즈 stamp 와 IMU 시각의 어긋남 Δt
   (ν = z − ẑ⁻ 이고 SLAM 포즈가 실제보다 Δt 늦은 위치를 가리키면 ν_along ≈ −|v|·Δt). 옆 성분 ν_cross 는 |v|·ω (구심) 에 회귀.
   회귀로 설명되는 몫을 빼고 남은 innovation 의 RMS·ρ₁."""
import math, os, sys, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import replay_tune as RT
from amr_mpc.core import config
from amr_mpc.torque.estimator import PlanarStateEstimator
for b in sys.argv[1:]:
    ev, _ = RT.load(b); cfg = dict(config.section("estimator", "torque_estimator.yaml")); cfg["mode"] = "kf"
    est = PlanarStateEstimator(cfg, pose_period=0.1); last = -1e9; rows = []
    for _, kind, t, a, bb, c in ev:
        if kind == 0:
            est.imu(t, a, bb, c); continue
        if t <= last: continue
        last = t
        if est.pose_ready and est.t is not None:
            _, v, _ = est._hist_at(t); w = est.omega
        r = est.pose(t, a, bb, c)
        if r == "used":
            rows.append((t, *est.last_residual[:2], v[0], v[1], w))
    R = np.array(rows); sp = np.hypot(R[:, 3], R[:, 4]); m = sp > 0.2; R, sp = R[m], sp[m]
    u = R[:, 3:5] / sp[:, None]; e = R[:, 1:3]
    al = (e * u).sum(1); cr = e[:, 0] * (-u[:, 1]) + e[:, 1] * u[:, 0]
    A = np.stack([sp, np.ones_like(sp)], 1); ka, *_ = np.linalg.lstsq(A, al, rcond=None)
    B = np.stack([sp * R[:, 5], np.ones_like(sp)], 1); kc, *_ = np.linalg.lstsq(B, cr, rcond=None)
    ra, rc = al - A @ ka, cr - B @ kc
    rho = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
    r2 = lambda y, res: 1 - np.var(res) / np.var(y)
    print(f"{os.path.basename(b)} (|v| > 0.2, {m.sum()} 개): ν_along = {ka[0]*1e3:+.1f} ms × |v| {ka[1]*1e3:+.2f} mm (R² {r2(al,ra):.2f}), "
          f"ν_cross = {kc[0]*1e3:+.1f} mm/(m²/s²) × |v|ω {kc[1]*1e3:+.2f} mm (R² {r2(cr,rc):.2f})")
    print(f"   along RMS {np.sqrt(np.mean(al**2))*1e3:.2f} → 회귀 뺀 뒤 {np.sqrt(np.mean(ra**2))*1e3:.2f} mm, ρ₁ {rho(al):+.2f} → {rho(ra):+.2f} | "
          f"cross RMS {np.sqrt(np.mean(cr**2))*1e3:.2f} → {np.sqrt(np.mean(rc**2))*1e3:.2f} mm, ρ₁ {rho(cr):+.2f} → {rho(rc):+.2f}")
