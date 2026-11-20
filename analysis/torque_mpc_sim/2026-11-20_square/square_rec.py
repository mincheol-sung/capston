#!/usr/bin/python3
"""정사각형 시뮬 한 판 기록: tools/compare_run.py 의 Recorder 를 그대로 쓰고, 원시 시계열을 더 남겨 꼭짓점 지표를 낸다.
  꼭짓점 통과 거리 = 내부 꼭짓점 c_j 근처(±0.4 m)에서 참값 위치가 c_j 에 가장 가까웠던 거리 (0 이면 꼭짓점을 정확히 지남)
  윤곽 오차 = 참값 위치에서 정사각형 변까지 거리 (시간 지연과 무관, 모양만)."""
import importlib.util, json, math, signal, sys, time
import numpy as np
import rclpy

MPC = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("compare_run", f"{MPC}/tools/compare_run.py")
cr = importlib.util.module_from_spec(spec); spec.loader.exec_module(cr)


class Rec(cr.Recorder):
    def __init__(self, end_after):
        super().__init__(end_after)
        self.raw, self.traw = [], []

    def on_truth(self, m):
        super().on_truth(m)
        if not self.armed or self.ref is None:
            return
        t = cr.stamp_s(m.header)
        Xr, ur = self.ref.at(t)
        p = m.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z), 1 - 2 * p.orientation.z ** 2)
        self.raw.append([t, p.position.x, p.position.y, yaw, *Xr, *ur])


def corner_metrics(raw, side, origin=(0.0, 0.0, 0.0)):
    a = np.array(raw)
    x0, y0, a0 = origin
    c0, s0 = math.cos(a0), math.sin(a0)
    px = c0 * (a[:, 1] - x0) + s0 * (a[:, 2] - y0)
    py = -s0 * (a[:, 1] - x0) + c0 * (a[:, 2] - y0)
    moving = np.hypot(a[:, 7], a[:, 8]) > 0.02
    corners = [(side, 0.0), (side, side), (0.0, side)]          # 내부 꼭짓점 (출발·도착 꼭짓점 제외)
    near = []
    for cx, cy in corners:
        d = np.hypot(px - cx, py - cy)
        m = d < 0.4
        near.append(float(d[m].min()) if m.any() else float("nan"))
    # 윤곽 오차: 네 변(선분)까지 거리의 최소
    def seg(px, py, ax, ay, bx, by):
        ux, uy = bx - ax, by - ay
        t = np.clip(((px - ax) * ux + (py - ay) * uy) / (ux * ux + uy * uy), 0, 1)
        return np.hypot(px - ax - t * ux, py - ay - t * uy)
    P = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    dc = np.min([seg(px, py, *P[i], *P[i + 1]) for i in range(4)], axis=0)
    return dict(corner_pass_mm=[round(1e3 * v, 1) for v in near],
                contour_rms_mm=round(1e3 * float(np.sqrt(np.mean(dc[moving] ** 2))), 1),
                contour_max_mm=round(1e3 * float(dc[moving].max()), 1))


def main():
    label, side, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    rclpy.init()
    node = Rec(1.5)
    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(f=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not node.finished() and time.monotonic() - t0 < 120:
        rclpy.spin_once(node, timeout_sec=0.1)
    s = node.summary(label)
    if node.raw:
        s.update(corner_metrics(node.raw, side))
        np.savez(f"{out}/{label}.npz", raw=np.array(node.raw), tau=np.array(node.tau).reshape(-1, 4), tau_t=np.array(node.tau_t))
    print(json.dumps(s, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
