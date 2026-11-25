#!/usr/bin/python3
"""diag_rec + /torque_mpc/status + /torque_mpc/adapt (ρ̂·η 분산·측정) 저장. npz 에 st·ad 추가 (2026-11-21 offset-free 시뮬)."""
import argparse, importlib.util, json, os, signal, time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray
D = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("diag_rec", f"{D}/diag_rec.py"); dr = importlib.util.module_from_spec(spec); spec.loader.exec_module(dr)
class FF(dr.Diag):
    def __init__(self, e):
        super().__init__(e); self.st = []; self.ad = []
        self.create_subscription(Float64MultiArray, "/torque_mpc/status", lambda m: self.st.append(list(m.data)) if len(m.data) == 26 else None, 100)
        self.create_subscription(Float64MultiArray, "/torque_mpc/adapt", lambda m: self.ad.append(list(m.data)) if len(m.data) == 7 else None, 100)
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--label", default="run"); ap.add_argument("--timeout", type=float, default=120.0); a = ap.parse_args()
    rclpy.init(); n = FF(1.5); stop = {"f": False}; signal.signal(signal.SIGINT, lambda *_: stop.update(f=True)); t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not n.finished() and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
    s = n.summary(a.label); lam = np.array(n.lam, float).reshape(-1, 4); lam = lam[np.all(np.isfinite(lam), axis=1)]
    if len(lam):
        mx = np.abs(lam).max(1); s.update(slip_frac10=float(np.mean(mx > 0.1)), slip_max=float(mx.max()))
    out = os.environ.get("DIAG_OUT", D)
    np.savez(f"{out}/{a.label}.npz", raw=np.array(n.raw), lam=np.array(n.lam_t), tau=np.array(n.tau).reshape(-1, 4), tau_t=np.array(n.tau_t), st=np.array(n.st), ad=np.array(n.ad))
    print(json.dumps(s, ensure_ascii=False), flush=True)
if __name__ == "__main__":
    main()
