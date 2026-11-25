#!/usr/bin/python3
"""오늘 bag 전부로 kf 잡음 배율 점검 (2026-11-21). σ_a·σ_g 를 같은 배율로 (xy 게인은 σ_a/σ_p 에만, yaw 는 σ_g/σ_ψ 에만 달려 서로 안 섞인다).
   bag 마다: 지금 yaml 로 재생 → 기록된 /state_estimator/status 와 검산, 배율 0.5/1/2/3/4 의 주행 구간 innovation RMS·ρ₁.
   실차 스택이 같은 NUC 에서 돌 수 있으니 nice 19 + ionice idle 로, 한 프로세스로 돌린다.
     nice -n 19 ionice -c3 /usr/bin/python3 analysis/estimator_tuning/2026-11-21_kf_innovation/all_bags.py bags/2026-11-21
"""
import glob, json, math, os, re, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import replay_tune as RT
from amr_mpc.core import config

MUL = tuple(float(x) for x in os.environ.get("MUL", "0.5,1,2,3,4").split(","))
root = sys.argv[1]
base = dict(config.section("estimator", "torque_estimator.yaml")); base["mode"] = "kf"
na0, ng0 = base["accel_noise_density"], base["gyro_noise_density"]
out_json = os.path.join(os.path.dirname(__file__), os.environ.get("OUT", "all_bags.json"))
done = json.load(open(out_json)) if os.path.exists(out_json) else {}
now = time.time()
for b in sorted(glob.glob(os.path.join(root, "*", "metadata.yaml"))):
    d = os.path.dirname(b); name = os.path.basename(d)
    if name in done or now - os.path.getmtime(b) < 120:            # 이미 함 / 방금 쓰인 bag (녹화 중일 수 있음) 건너뜀
        continue
    import yaml
    meta = yaml.safe_load(open(b))["rosbag2_bagfile_information"]
    cnt = {t["topic_metadata"]["name"]: t["message_count"] for t in meta["topics_with_message_count"]}
    if cnt.get("/tracked_pose", 0) < 100 or cnt.get("/imu/base_link", 0) < 1000:
        continue
    args = ""
    try:
        args = open(os.path.join(d, "run_info", "args.txt")).read().splitlines()[3]
    except (OSError, IndexError):
        pass
    ctrl = "A" if "controller:=pure_pursuit" in args else ("B" if "limit_mode:=motor_only" in args else "C")
    estm = (re.findall(r"est_mode:=(\w+)", args) or ["kf"])[0]
    t0 = time.time()
    ev, status = RT.load(d)
    rec = dict(ctrl=ctrl, est_mode=estm, shape=" ".join(sorted(set(re.findall(r"shape:=(\w+)", args)))), res={})
    for m in MUL:
        cfg = dict(base); cfg["accel_noise_density"] = na0 * m; cfg["gyro_noise_density"] = ng0 * m
        R, _ = RT.replay(ev, cfg)
        if m == 1.0 and len(status):
            j = np.searchsorted(R[:, 0], status[:, 0]).clip(1, len(R) - 1)
            j = np.where(np.abs(R[j - 1, 0] - status[:, 0]) < np.abs(R[j, 0] - status[:, 0]), j - 1, j)
            ok = np.abs(R[j, 0] - status[:, 0]) < 0.005
            rec["check_mm"] = float(np.abs(R[j[ok], 1:3] - status[ok, 1:3]).max() * 1e3) if ok.any() else float("nan")
        mv = (R[:, 4] > 0.05) & (R[:, 5] > 0)
        e = R[mv]
        if len(e) < 30:
            break
        rho = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
        rec["res"][str(m)] = dict(n=int(len(e)), sxy=float(np.mean(e[:, 1:3] ** 2)), syaw=float(np.mean(e[:, 3] ** 2)),
                                  rxy=0.5 * (rho(e[:, 1]) + rho(e[:, 2])), ryaw=rho(e[:, 3]))
    rec["sec"] = time.time() - t0
    done[name] = rec
    json.dump(done, open(out_json, "w"), indent=1)
    r1 = rec["res"].get("1.0")
    print(f"{name} {ctrl} {estm} 검산 {rec.get('check_mm', float('nan')):.3f} mm, 주행 포즈 {r1['n'] if r1 else 0}, {rec['sec']:.0f} s", flush=True)
