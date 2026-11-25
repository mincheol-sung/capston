#!/usr/bin/python3
"""후보 잡음 값 하나를 지금 yaml 과 나란히 전체 bag 에 재생: 주행 |ν| 중앙값·RMS·ρ₁·NIS(모델 S 기준 절대 일관성), 정지 innovation 도.
   후보 = xy 비율 ×3, yaw 비율 ×4 를 실측에 가까운 절대값으로:  σ_a 0.14 (주행 PSD 0.09~0.13), σ_p 2 mm (정지 SLAM 1.1~1.6 mm),
   σ_b 0.04 (바이어스 시정수 σ_a/σ_b 3.5 s 유지), σ_g 0.0064 (그대로), σ_ψ 0.05° (yaml 주석의 정지 실측), σ_bg 0.0005 (그대로)."""
import json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import replay_tune as RT
from amr_mpc.core import config
from amr_mpc.torque.estimator import design_gains
base = dict(config.section("estimator", "torque_estimator.yaml")); base["mode"] = "kf"
cand = dict(base, accel_noise_density=0.14, accel_bias_walk=0.04, pose_noise_xy=0.002, pose_noise_yaw_deg=0.05)
J = json.load(open(os.path.join(os.path.dirname(__file__), "all_bags_hi.json")))
keep = sorted(k for k, v in J.items() if v["res"].get("1.0", {}).get("n", 0) >= 30 and v.get("check_mm", 9) <= 1.0)
def S(c):
    return RT.S_ss(0.1, c["accel_noise_density"], c["accel_bias_walk"], c["pose_noise_xy"], c["gyro_noise_density"], c["gyro_bias_walk"], math.radians(c["pose_noise_yaw_deg"]))
agg = {"지금": [], "후보": []}
for k in keep:
    ev, _ = RT.load(os.path.join(sys.argv[1], k))
    for lab, c in (("지금", base), ("후보", cand)):
        R, _ = RT.replay(ev, c); u = R[:, 5] > 0
        agg[lab].append((R[(R[:, 4] > 0.05) & u], R[(R[:, 4] < 0.005) & u]))
rho = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
print(f"{len(keep)} bag")
print("| 설정 | K_xy | K_yaw | 주행 \\|ν_xy\\| 중앙값 mm | 주행 ν_xy RMS mm | ρ₁ xy (bag 중앙값) | NIS xy | 주행 ν_ψ RMS ° | ρ₁ yaw | NIS yaw | 정지 ν_xy RMS mm | 정지 NIS xy |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|")
for lab, c in (("지금", base), ("후보", cand)):
    Kx, Ky = design_gains(0.1, c["accel_noise_density"], c["accel_bias_walk"], c["pose_noise_xy"], c["gyro_noise_density"], c["gyro_bias_walk"], math.radians(c["pose_noise_yaw_deg"]))
    Sx, Sy = S(c)
    mv = np.concatenate([a for a, _ in agg[lab]]); st = np.concatenate([b for _, b in agg[lab]])
    exy = mv[:, 1:3]; mag = np.hypot(exy[:, 0], exy[:, 1])
    print(f"| {lab} | [{Kx[0]:.3f}, {Kx[1]:.2f}, {Kx[2]:.3f}] | [{Ky[0]:.3f}, {Ky[1]:.4f}] | {np.median(mag)*1e3:.2f} | {math.sqrt(np.mean(exy**2))*1e3:.2f} | "
          f"{np.median([0.5*(rho(a[:,1])+rho(a[:,2])) for a,_ in agg[lab]]):+.2f} | {np.mean(exy**2)/Sx:.2f} | {math.degrees(math.sqrt(np.mean(mv[:,3]**2))):.3f} | "
          f"{np.median([rho(a[:,3]) for a,_ in agg[lab]]):+.2f} | {np.mean(mv[:,3]**2)/Sy:.2f} | {math.sqrt(np.mean(st[:,1:3]**2))*1e3:.2f} | {np.mean(st[:,1:3]**2)/Sx:.2f} |")
