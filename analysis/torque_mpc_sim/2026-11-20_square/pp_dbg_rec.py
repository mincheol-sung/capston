#!/usr/bin/python3
"""PP 꼭짓점 토크 튐 조사: square_rec 기록 + /pure_pursuit/status, /wheel_pi/status 원시값."""
import importlib.util, json, signal, sys, time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

SP = "/tmp/claude-1000/-home-smoc-capston/69d56774-629f-4670-9d6f-69486a1cc5db/scratchpad"
spec = importlib.util.spec_from_file_location("square_rec", f"{SP}/square_rec.py")
sr = importlib.util.module_from_spec(spec); spec.loader.exec_module(sr)


class Dbg(sr.Rec):
    def __init__(self, end_after):
        super().__init__(end_after)
        self.pp, self.pi = [], []
        self.create_subscription(Float64MultiArray, "/pure_pursuit/status", lambda m: self.pp.append(list(m.data)), 100)
        self.create_subscription(Float64MultiArray, "/wheel_pi/status", lambda m: self.pi.append(list(m.data)), 200)


def main():
    label, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Dbg(1.5)
    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(f=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not node.finished() and time.monotonic() - t0 < 120:
        rclpy.spin_once(node, timeout_sec=0.05)
    s = node.summary(label)
    s.update(sr.corner_metrics(node.raw, 1.5))
    np.savez(f"{out}/{label}.npz", raw=np.array(node.raw), tau=np.array(node.tau).reshape(-1, 4),
             tau_t=np.array(node.tau_t), pp=np.array([r for r in node.pp if len(r) == 10]),
             pi=np.array([r for r in node.pi if len(r) == 14]))
    print(json.dumps(s, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
