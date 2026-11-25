#!/usr/bin/python3
"""compare_run.Recorder + 참값 슬립률 (/slip_logger/slip 의 참값 λ×4) 요약.
  slip_frac10 = 주행 중 max_i |λ_i| > 0.1 인 표본 비율, slip_max = max |λ|, slip_rms = 네 바퀴 λ RMS."""
import argparse, importlib.util, json, signal, time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

MPC = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc"
spec = importlib.util.spec_from_file_location("compare_run", f"{MPC}/tools/compare_run.py")
cr = importlib.util.module_from_spec(spec); spec.loader.exec_module(cr)


class SlipRec(cr.Recorder):
    def __init__(self, end_after):
        super().__init__(end_after)
        self.lam = []
        self.create_subscription(Float64MultiArray, "/slip_logger/slip", self.on_slip, 100)

    def on_slip(self, m):
        if self.armed and len(m.data) >= 13 and self.ref is not None:
            _, ur = self.ref.at(m.data[0])
            if np.hypot(ur[0], ur[1]) > 0.02:                       # 주행 중만
                self.lam.append(list(m.data[9:13]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run"); ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args()
    rclpy.init()
    node = SlipRec(1.5)
    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(f=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not node.finished() and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    s = node.summary(a.label)
    lam = np.array(node.lam, float).reshape(-1, 4)
    lam = lam[np.all(np.isfinite(lam), axis=1)]
    if len(lam):
        mx = np.abs(lam).max(axis=1)
        s.update(slip_n=int(len(lam)), slip_frac10=float(np.mean(mx > 0.1)), slip_max=float(mx.max()),
                 slip_rms=float(np.sqrt(np.mean(lam ** 2))))
    s["timed_out"] = time.monotonic() - t0 >= a.timeout
    print(json.dumps(s, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
