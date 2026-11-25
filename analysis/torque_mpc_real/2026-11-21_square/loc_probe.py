#!/usr/bin/python3
"""측위 검사 (읽기만, /initialpose 안 보냄) — Cartographer 가 지금 놓은 점 (/scan_matched_points2) 을 png 지도에 겹쳐 보고,
지금 포즈 둘레를 촘촘히 (회전 ±2° 0.1° 간격, 이동 ±0.10 m 1 cm 간격) 훑어 png 에 가장 잘 맞는 어긋남을 찾는다.
check_localization.py 와 같은 지도·같은 겹침 정의 (±5 cm = 1 칸 팽창, 기준 60 % — 2026-11-21 70 에서 바꿈). 회전은 추적점 (/tracked_pose) 둘레.
  python3 loc_probe.py [반복 수=3] [간격 s=8]"""
import importlib.util, math, sys, time
import numpy as np
import rclpy
from scipy.ndimage import binary_dilation

spec = importlib.util.spec_from_file_location("cl", "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools/check_localization.py")
cl = importlib.util.module_from_spec(spec); spec.loader.exec_module(cl)


def main():
    n_rep = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    gap = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    occ, res, ox, oy = cl.load_map(cl.DEFAULT_MAP); occ1 = binary_dilation(occ, iterations=1)
    rclpy.init(); n = cl.Probe()
    try:
        for k in range(n_rep):
            if not n.wait(10.0):
                print("10 s 동안 /scan_matched_points2·/tracked_pose 없음"); return 1
            p, (x0, y0, th0) = n.cloud, n.pose
            sc0, _ = cl.score(occ1, res, ox, oy, p)
            q = p - [x0, y0]
            best = (sc0, 0.0, 0.0, 0.0); prof = []
            for dth in np.radians(np.arange(-2.0, 2.001, 0.1)):
                r = cl.transform(q, 0, 0, dth)
                bt = (0.0, 0.0, 0.0)
                for dx in np.arange(-0.10, 0.1001, 0.01):
                    for dy in np.arange(-0.10, 0.1001, 0.01):
                        sc, _ = cl.score(occ1, res, ox, oy, r + [x0 + dx, y0 + dy])
                        if sc > bt[0]:
                            bt = (sc, dx, dy)
                prof.append((math.degrees(dth), bt[0]))
                if bt[0] > best[0]:
                    best = (bt[0], bt[1], bt[2], math.degrees(dth))
            pr = "  ".join(f"{d:+.1f}°:{100*s:.0f}" for d, s in prof[::4])
            print(f"[{k}] 지금 ({x0:+.3f}, {y0:+.3f}) m {math.degrees(th0):+.2f}°  점 {len(p)}  겹침 {100*sc0:.1f} %  → 가장 잘 맞는 어긋남 "
                  f"회전 {best[3]:+.1f}° 이동 ({100*best[1]:+.0f}, {100*best[2]:+.0f}) cm 에서 {100*best[0]:.1f} %")
            print(f"     회전별 최고 겹침 (이동 최적): {pr}")
            if k + 1 < n_rep:
                time.sleep(gap)
    finally:
        n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
