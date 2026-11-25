import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri="dbg/nod/nod_0.mcap", storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
T = {"/wheel_states": WheelStates, "/torque_output/cmd_raw": WheelTorqueCommand, "/torque_mpc/cmd_raw": WheelTorqueCommand, "/wheel_torque_cmd": WheelTorqueCommand, "/sim/ground_truth": Odometry, "/torque_mpc/status": Float64MultiArray}
D = {k: [] for k in T}
while rd.has_next():
    try: top, d, ts = rd.read_next()
    except Exception: break
    if top not in T: continue
    m = deserialize_message(d, T[top]); t = ts*1e-9
    if top == "/wheel_states": D[top].append([t, *m.velocity])
    elif top == "/sim/ground_truth": D[top].append([t, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x])
    elif top == "/torque_mpc/status": D[top].append([t, *m.data[:8]])
    else: D[top].append([t, m.mode, *m.torque])
D = {k: np.array(v) for k, v in D.items()}
w = D["/wheel_states"]; g = np.diff(w[:,0]); i = np.argmax(g); print("max wheel gap", round(g.max(),3), "at", round(w[i,0]-w[0,0],3))
c = D["/wheel_torque_cmd"]; tq = c[c[:,1]==1]
print("modes", np.unique(c[:,1]), "first TORQUE t", round(tq[0,0]-w[0,0],3) if len(tq) else None)
t0 = w[i,0]
np.set_printoptions(linewidth=200, suppress=True)
for k in ["/torque_mpc/cmd_raw", "/torque_output/cmd_raw"]:
    a = D[k]; s = (a[:,0] > t0-0.45) & (a[:,0] < t0+0.05)
    b = a[s].copy(); b[:,0] -= t0; print(k, len(b)); print(np.round(b[::5], 4)[:12])
s = (w[:,0] > t0-0.45) & (w[:,0] < t0+0.4); b = w[s].copy(); b[:,0]-=t0; print("wheels"); print(np.round(b[::5], 3)[:20])
g2 = D["/sim/ground_truth"]; print("truth gap", round(np.diff(g2[:,0]).max(),3))
print("--- message rates around t0 (count per 50 ms) ---")
for k in ["/torque_mpc/cmd_raw", "/torque_output/cmd_raw", "/wheel_torque_cmd", "/wheel_states", "/torque_mpc/status"]:
    a = D[k][:, 0] - t0; edges = np.arange(-0.3, 0.55, 0.05)
    print(f"{k:24s}", np.histogram(a, edges)[0])
s = D["/torque_mpc/status"]; ss = s[(s[:,0] > t0-0.2) & (s[:,0] < t0+0.5)].copy(); ss[:,0] -= t0
print(np.round(ss[::3], 4)[:15])
