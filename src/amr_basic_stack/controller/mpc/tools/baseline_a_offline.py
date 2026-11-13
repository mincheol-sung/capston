#!/usr/bin/python3
"""비교군 A (퓨어 퍼슛 + 바퀴별 PI) 오프라인 폐루프 시뮬 — ROS 없이 게인을 빨리 훑어보는 도구.

ROS 시뮬(torque_experiment.launch.py controller:=pure_pursuit)과 같은 계산부(amr_mpc/baseline)·같은 플랜트 식
(torque/model.py TorquePlant / SlipPlant)·같은 8자(config/torque_track.yaml)를 쓴다. 다른 점:
  - 상태 추정기 없음: 퓨어 퍼슛은 참값 포즈(+ --pose_noise 로 흰 잡음)를 본다
  - 명령은 한 주기 늦게 들어간다 (ROS 경로의 PI → torque_output → 게이트 → 플랜트 한 틱 지연 흉내)

  /usr/bin/python3 tools/baseline_a_offline.py                       # 기본 게인 (config/torque_baseline_a.yaml)
  /usr/bin/python3 tools/baseline_a_offline.py --mu 0.3 --kp 0.08 --ki 0.6
  /usr/bin/python3 tools/baseline_a_offline.py --sweep                # PI 게인 격자
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from amr_mpc.baseline.pure_pursuit import PurePursuitConfig, pure_pursuit   # noqa: E402
from amr_mpc.baseline.wheel_pi import WheelPI, wheel_targets               # noqa: E402
from amr_mpc.core import config                                            # noqa: E402
from amr_mpc.core.figure8 import Figure8                                   # noqa: E402
from amr_mpc.core.reference import ReferenceHorizon                        # noqa: E402
from amr_mpc.torque.model import SlipPlant, TorquePlant                    # noqa: E402


def run(kp, ki, mu=0.0, pp=None, pose_noise=0.0, Ts=0.01, seed=0, verbose=False):
    track = config.load("torque_track.yaml")["track"]
    mpc = config.load("torque_mpc.yaml")
    path = Figure8(width=track["width"], height=track["height"], speed=track["speed"], accel=track["accel"],
                   laps=track["laps"], yaw_mode=track["yaw_mode"], align_s=track["align_s"],
                   lap_speeds=track["lap_speeds"] or None)
    p = TorquePlant.from_yaml()
    tau_bar = p.tau_motor_limit()
    pi = WheelPI(kp, ki, tau_bar, tau_bar * float(mpc["torque_limit"]["rate_fraction"]) / Ts)
    pp = pp or PurePursuitConfig.from_dict(config.load("torque_baseline_a.yaml")["pure_pursuit"])
    slip = SlipPlant(p, mu) if mu > 0 else None
    sub = max(10, int(math.ceil(Ts / slip.stable_dt()))) if slip else 10
    rng = np.random.default_rng(seed)

    X, Xd = np.zeros(3), np.zeros(3)
    w = np.zeros(4)
    tau_now = np.zeros(4)                   # 이번 주기에 플랜트가 받는 토크 (직전 주기 계산)
    T_end = path.T_total + 1.0
    ref, rows = None, []
    n = int(T_end / Ts)
    for k in range(n):
        t = k * Ts
        if k % 10 == 0:                     # 10 Hz 기준열, 첫 행 = 지금 (figure8_reference wait_for_arm 규약)
            R = [np.concatenate(path.at(t + i * 0.2)) for i in range(20)]
            ref = ReferenceHorizon.from_rows(t - 0.2, 0.2, R)
        omega = w if slip else p.wheel_speeds(X, Xd)
        pose = X + (rng.normal(0, pose_noise, 3) * [1, 1, 0.3] if pose_noise > 0 else 0.0)
        v, wz, info = pure_pursuit(pp, ref.at, t, pose)
        tau_next = pi.step(wheel_targets(p.Tinv, [v, 0.0, wz]), omega, Ts)
        Xr, _ = path.at(t)
        e = math.hypot(*(Xr[:2] - X[:2]))
        lam = np.zeros(4)
        if slip:
            om_nom = slip.contact_speed(X, Xd)
            lam = (w - om_nom) * np.where(om_nom >= 0, 1, -1) / np.maximum(np.maximum(abs(w), abs(om_nom)), 0.5)
        rows.append([t, e, abs(Xr[2] - X[2]), *tau_now, *lam, np.linalg.norm(Xd[:2])])
        for _ in range(sub):
            if slip:
                X, Xd, w = slip.step(X, Xd, w, tau_now, Ts / sub)
            else:
                X, Xd = p.step(X, Xd, tau_now, Ts / sub)
        tau_now = tau_next
        if e > 0.5:
            if verbose:
                print(f"  이탈 t={t:.2f}s")
            break
    A = np.array(rows)
    moving = A[:, -1] > 0.05
    out = {
        "rms_mm": 1e3 * float(np.sqrt(np.mean(A[:, 1] ** 2))), "max_mm": 1e3 * float(A[:, 1].max()),
        "tau_max": float(np.abs(A[:, 3:7]).max()),
        "dtau_rms": float(np.sqrt(np.mean(np.diff(A[:, 3:7], axis=0) ** 2))),
        "lam_rms": float(np.sqrt(np.mean(A[moving, 7:11] ** 2))) if slip and moving.any() else 0.0,
        "lam_max": float(np.abs(A[moving, 7:11]).max()) if slip and moving.any() else 0.0,
        "done": bool(A[-1, 0] >= T_end - 2 * Ts),
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--mu", type=float, default=0.0)
    ap.add_argument("--pose_noise", type=float, default=0.0, help="퓨어 퍼슛이 보는 포즈 잡음 σ [m] (yaw 는 0.3 배 rad)")
    ap.add_argument("--lookahead_time", type=float, default=None)
    ap.add_argument("--k_along", type=float, default=None)
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    cfg = config.load("torque_baseline_a.yaml")
    kp = a.kp if a.kp is not None else cfg["wheel_pi"]["kp"]
    ki = a.ki if a.ki is not None else cfg["wheel_pi"]["ki"]
    pp = PurePursuitConfig.from_dict(cfg["pure_pursuit"])
    if a.lookahead_time is not None:
        pp.lookahead_time = a.lookahead_time
    if a.k_along is not None:
        pp.k_along = a.k_along
    if a.sweep:
        print(f"μ={a.mu}  잡음 {a.pose_noise}")
        print(" kp     ki  | RMS mm  max mm | τmax   Δτ RMS | λ RMS  λ max | 완주")
        for kpi in (0.02, 0.04, 0.08, 0.15):
            for kii in (0.0, kpi * 4, kpi * 10, kpi * 25):
                r = run(kpi, kii, a.mu, pp, a.pose_noise)
                print(f"{kpi:5.2f} {kii:5.2f} | {r['rms_mm']:6.1f} {r['max_mm']:6.1f} | {r['tau_max']:.3f} "
                      f"{r['dtau_rms']:.4f} | {r['lam_rms']:.3f} {r['lam_max']:.3f} | {r['done']}")
        return
    r = run(kp, ki, a.mu, pp, a.pose_noise, verbose=True)
    print(f"kp {kp} ki {ki} μ {a.mu}: " + "  ".join(f"{k} {v:.4g}" if isinstance(v, float) else f"{k} {v}"
                                                  for k, v in r.items()))


if __name__ == "__main__":
    main()
