#!/usr/bin/env python3
"""bag 에서 localization 품질을 판정한다.  capture.sh 로 뜬 bag 을 그대로 넣으면 된다.

    python3 check_localization.py ~/2026amr_ws/bags/mpc_chain_20260909_151007

무엇을 보나 — map→odom TF 하나면 대부분 판정된다
---------------------------------------------------------------
map→odom 은 "odometry 가 여태 얼마나 틀렸는지"에 대한 localization 의 정정값이다.
따라서 이 변환의 거동이 곧 품질이다:

  · 로봇이 **정지**해 있으면 odometry 가 안 움직이므로 정정할 것도 없다.
    → map→odom 은 상수여야 한다. 여기서 떨리면 그건 전부 scan matching 잡음이다.
      이게 가장 깨끗한 측정이다 — 다른 어떤 오차원과도 섞이지 않는다.
  · **주행 중**에는 odometry 오차만큼 정정이 들어간다. 정상이면 주행거리에 비례해
    조금씩, 연속적으로. 불연속 점프는 scan matcher 가 다른 곳에 붙었다는 뜻이다.
  · 보정량/주행거리 [m/m] 가 요약 지표다. 1 m 가면서 2 m 를 정정하고 있으면
    측위가 주행을 따라가지 못하는 것이다.

이 스택에서 map→odom 을 내는 것은 cartographer 가 아니다 (localization_2d.lua 가
publish_to_tf=false). 경로는:
    cartographer → /tracked_pose → pose_to_odom_node(공분산 주입) → /tracked_pose/odom
                 → ekf_carto (robot_localization, world_frame=map) → map→odom TF
그래서 이 스크립트가 나쁘다고 하면 용의자가 셋이다 — cartographer scan matching,
pose_to_odom_node 의 cov_x/cov_y/cov_yaw, ekf_carto 설정.
어느 쪽인지 가르려면 /scan 과 /tracked_pose 가 bag 에 있어야 한다 (capture.sh --heavy).

2026-09-09 mpc_chain_20260909_151007 실측 (이 스크립트의 기준선):
    정지·수렴 후   샘플간 p99 0.13 cm,  5 cm 점프 0 회      ← 정상
    정지·수렴 전   위치 1σ 58~95 cm, yaw 1σ 4.6~7.4 deg     ← 전역측위 미수렴
    주행 중        2.56 m 가는 동안 누적 보정 39.5 m (15.4 m/m), 5 cm 점프 122 회
  즉 "정지하면 멀쩡하고 움직이면 무너진다".
---------------------------------------------------------------
"""
import sys
from pathlib import Path

import numpy as np

JUMP_M = 0.05        # 연속 샘플 간 이 값을 넘으면 점프로 센다. 50 Hz 에서 5 cm = 2.5 m/s
HP_WIN = 51          # 고주파 떨림을 뽑는 이동평균 창 [샘플]. 50 Hz 에서 약 1 s
STILL_V = 1e-6       # /cmd_vel L1 부하가 이 값 이하이면 정지로 본다
K_GEOM = (0.1897 + 0.207) / 2.0


def read_bag(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    mo, cmd, odom = [], [], []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t = ts * 1e-9
        if topic == "/tf":
            m = deserialize_message(data, get_message(types[topic]))
            for tr in m.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    q = tr.transform.rotation
                    mo.append((t, tr.transform.translation.x, tr.transform.translation.y,
                               np.arctan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z))))
        elif topic == "/cmd_vel":
            m = deserialize_message(data, get_message(types[topic]))
            cmd.append((t, abs(m.linear.x) + abs(m.linear.y) + K_GEOM * abs(m.angular.z)))
        elif topic == "/wheel_cmd":                      # 구조 B — 바퀴 각속도로 정지 판정
            m = deserialize_message(data, get_message(types[topic]))
            cmd.append((t, float(np.abs(m.data).max()) if len(m.data) else 0.0))
        elif topic == "/odometry/filtered":
            m = deserialize_message(data, get_message(types[topic]))
            odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
    return np.array(mo), np.array(cmd), np.array(odom)


def segments(t, cmd, moving):
    """cmd 로 정지/주행 구간을 나눈다. 전이 앞뒤 0.5 s 는 버린다 (판정이 섞이지 않게)."""
    if len(cmd) == 0:
        return [(t[0], t[-1], "전체")]
    state = np.interp(t, cmd[:, 0], (cmd[:, 1] > STILL_V).astype(float)) > 0.5
    if not moving:
        state = ~state
    out, i = [], 0
    while i < len(state):
        if not state[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(state) and state[j + 1]:
            j += 1
        if t[j] - t[i] > 3.0:
            out.append((t[i] + 0.5, t[j] - 0.5, ""))
        i = j + 1
    return out


def report(t, x, y, yaw, od, a, b, label):
    m = (t >= a) & (t <= b)
    if m.sum() < HP_WIN * 3:
        return None
    xs, ys, ws = x[m], y[m], np.unwrap(yaw[m])
    step = np.hypot(np.diff(xs), np.diff(ys))
    dist = 0.0
    if len(od):
        g = np.arange(a, b, 0.05)
        ox, oy = np.interp(g, od[:, 0], od[:, 1]), np.interp(g, od[:, 0], od[:, 2])
        dist = float(np.hypot(np.diff(ox), np.diff(oy)).sum())
    k = np.ones(HP_WIN) / HP_WIN                          # 1 s 이동평균 대비 편차 = 고주파 떨림
    hp = lambda v: (v - np.convolve(v, k, mode="same"))[HP_WIN:-HP_WIN]
    jit = np.hypot(hp(xs), hp(ys))
    ratio = step.sum() / dist if dist > 0.02 else float("nan")
    return dict(label=label, a=a, b=b, n=int(m.sum()), dist=dist,
                jit=jit.std() * 100, jit99=np.percentile(jit, 99) * 100,
                jyaw=np.degrees(hp(ws)).std(),
                p99=np.percentile(step, 99) * 100, mx=step.max() * 100,
                jumps=int((step > JUMP_M).sum()), cum=step.sum(), ratio=ratio,
                net=np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) * 100,
                nyaw=np.degrees(ws[-1] - ws[0]))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(2)
    uri = Path(sys.argv[1]).expanduser()
    mo, cmd, od = read_bag(uri)
    if len(mo) < 100:
        raise SystemExit("map→odom TF 가 bag 에 없습니다. localization 이 안 떴거나 /tf 를 안 담았습니다.")
    t0 = mo[0, 0]
    t, x, y, yaw = mo[:, 0] - t0, mo[:, 1], mo[:, 2], mo[:, 3]
    if len(cmd):
        cmd = cmd.copy(); cmd[:, 0] -= t0
    if len(od):
        od = od.copy(); od[:, 0] -= t0

    dt = np.diff(t)
    print(f"bag: {uri.name}")
    print(f"map→odom {len(t)} 샘플 / {t[-1]-t[0]:.0f} s = {len(t)/(t[-1]-t[0]):.1f} Hz"
          f"   (발행 간격 p99 {np.percentile(dt,99)*1000:.0f} ms, max {dt.max()*1000:.0f} ms)")

    rows = []
    for moving, tag in ((False, "정지"), (True, "주행")):
        for a, b, _ in segments(t, cmd, moving):
            r = report(t, x, y, yaw, od, a, b, tag)
            if r:
                rows.append(r)
    rows.sort(key=lambda r: r["a"])

    print()
    hdr = (f'{"구간":<20s} {"주행[m]":>8s} {"떨림 1σ":>9s} {"떨림 yaw":>9s} '
           f'{"보정 p99":>9s} {"최대":>9s} {"점프":>6s} {"보정/주행":>10s}')
    print(hdr)
    print(f'{"":<20s} {"":>8s} {"[cm]":>9s} {"[deg]":>9s} {"[cm]":>9s} {"[cm]":>9s} '
          f'{f">{JUMP_M*100:.0f}cm":>6s} {"[m/m]":>10s}')
    for r in rows:
        rr = "  —" if np.isnan(r["ratio"]) else f'{r["ratio"]:10.2f}'
        seg = f'{r["label"]} {r["a"]:.0f}~{r["b"]:.0f}s'      # 중첩 f-string 을 피한다 (3.12 미만 호환)
        print(f'{seg:<20s} {r["dist"]:8.2f} '
              f'{r["jit"]:9.2f} {r["jyaw"]:9.3f} {r["p99"]:9.2f} {r["mx"]:9.1f} '
              f'{r["jumps"]:6d} {rr}')

    still = [r for r in rows if r["label"] == "정지"]
    drive = [r for r in rows if r["label"] == "주행"]
    print("\n판정")
    if still:
        best = min(still, key=lambda r: r["jit"])
        worst = max(still, key=lambda r: r["jit"])
        print(f"  · 정지 중 떨림 최선 {best['jit']:.2f} cm / {best['jyaw']:.3f}° "
              f"({best['a']:.0f}~{best['b']:.0f}s),  최악 {worst['jit']:.2f} cm / {worst['jyaw']:.3f}°")
        if worst["jit"] > 5 * max(best["jit"], 1e-3):
            print("    → 구간에 따라 크게 다르다. 전역측위가 수렴하기 전 구간이 섞여 있다는 뜻이다.")
            print("      수렴 후 값만 품질로 읽어라. 수렴에 걸린 시간이 곧 bootstrap 비용이다.")
        if best["jit"] > 1.0:
            print("    → 수렴 후에도 정지 중 1 cm 넘게 떨린다. scan matching 이나 ekf_carto 문제다.")
    if drive:
        ratios = [r["ratio"] for r in drive if not np.isnan(r["ratio"])]
        jumps = sum(r["jumps"] for r in drive)
        if ratios:
            print(f"  · 주행 중 보정/주행 = {min(ratios):.2f} ~ {max(ratios):.2f} m/m,  점프 {jumps} 회")
            if max(ratios) > 1.0:
                print("    → 1 m 가면서 1 m 넘게 정정하고 있다. 측위가 주행을 못 따라간다.")
                print("      odometry prior(메카넘 슬립)와 scan matching 중 어느 쪽인지 보려면")
                print("      /scan 과 /tracked_pose 가 필요하다 — capture.sh --heavy 로 다시 떠라.")
    if still and drive:
        sj = min(r["jit"] for r in still)
        dj = min(r["jit"] for r in drive)
        if dj > 5 * max(sj, 1e-3):
            print(f"  · 정지 {sj:.2f} cm vs 주행 {dj:.2f} cm — 움직일 때만 무너진다.")
            print("    정지 중 정상이면 맵과 센서 자체는 멀쩡하다. 운동 중 처리가 문제다:")
            print("      motion distortion(스캔 왜곡), odometry prior 품질, ekf_carto 의 시간정렬.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
