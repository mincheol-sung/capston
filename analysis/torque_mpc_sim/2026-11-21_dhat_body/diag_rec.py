#!/usr/bin/python3
"""진단용: slip_rec 요약 + 시계열 저장 (npz: raw=[t,x,y,yaw,Xr3,ur3], lam=[t, λ_true×4], tau, tau_t).  출력 npz = $DIAG_OUT/<라벨>.npz"""
import argparse, importlib.util, json, math, os, signal, time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

D = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("slip_rec", f"{D}/slip_rec.py")
sr = importlib.util.module_from_spec(spec); spec.loader.exec_module(sr)
cr = sr.cr


class Diag(sr.SlipRec):
    def __init__(self, end_after):
        super().__init__(end_after)
        self.raw, self.lam_t = [], []
        self.create_subscription(Float64MultiArray, "/slip_logger/slip",
                                 lambda m: self.lam_t.append([m.data[0], *m.data[9:13]]) if self.armed and len(m.data) >= 13 else None, 100)

    def on_truth(self, m):
        super().on_truth(m)
        if not self.armed or self.ref is None:
            return
        t = cr.stamp_s(m.header)
        Xr, ur = self.ref.at(t)
        p = m.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z), 1 - 2 * p.orientation.z ** 2)
        self.raw.append([t, p.position.x, p.position.y, yaw, *Xr, *ur])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run"); ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args()
    rclpy.init()
    node = Diag(1.5)
    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(f=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not node.finished() and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    s = node.summary(a.label)
    lam = np.array(node.lam, float).reshape(-1, 4); lam = lam[np.all(np.isfinite(lam), axis=1)]
    if len(lam):
        mx = np.abs(lam).max(axis=1)
        s.update(slip_frac10=float(np.mean(mx > 0.1)), slip_max=float(mx.max()), slip_rms=float(np.sqrt(np.mean(lam ** 2))))
    out = os.environ.get("DIAG_OUT", D)
    np.savez(f"{out}/{a.label}.npz", raw=np.array(node.raw), lam=np.array(node.lam_t),
             tau=np.array(node.tau).reshape(-1, 4), tau_t=np.array(node.tau_t))
    print(json.dumps(s, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
