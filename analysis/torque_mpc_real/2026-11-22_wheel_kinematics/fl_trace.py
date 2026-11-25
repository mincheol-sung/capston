#!/usr/bin/python3
"""한 판에서 바퀴 토크가 거치는 단계를 나란히 (2026-11-22, FL 혼자 도는 원인 찾기).
  τ*  = /torque_mpc/cmd_raw (MPC 순토크),  τ_out = /torque_output/cmd_raw (= τ* + 마찰 보상 τ_fric(ω) [+ 정지 마찰 보상]),
  τ_cmd = /wheel_torque_cmd (게이트 뒤 최종, mode), τ_meas = /wheel_states.torque (모터 보고), ω = /wheel_states.velocity, error·fresh·온도."""
import sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
b = sys.argv[1]; dt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
T = ["/wheel_states", "/state_estimator/odom", "/torque_mpc/status", "/torque_mpc/cmd_raw", "/torque_output/cmd_raw", "/wheel_torque_cmd",
     "/dra_mppi/reference_trajectory"]
r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
D = {k: [] for k in T}
while r.has_next():
    n, d, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
S = np.array([[t, m.data[20] if len(m.data) > 20 else 0] for t, m in D["/torque_mpc/status"]]); act = S[S[:, 1] == 0, 0]; t0 = act[0]
E = np.array([[st(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, m in D["/state_estimator/odom"]])
W = np.array([[t, *m.velocity, *m.torque, *m.temperature, *m.error, *[float(f) for f in m.fresh]] for t, m in D["/wheel_states"]])
def cmd(topic):
    return np.array([[t, m.mode, *m.torque] for t, m in D[topic]])
Tm, To, Tc = cmd("/torque_mpc/cmd_raw"), cmd("/torque_output/cmd_raw"), cmd("/wheel_torque_cmd")
R = np.array(sorted((st(m.header) + m.dt, rr.vy) for _, m in D["/dra_mppi/reference_trajectory"] if m.references for rr in m.references[:1]))
def at(A, t, c):
    j = np.searchsorted(A[:, 0], t); j = min(max(j, 0), len(A) - 1); return A[j, c]
print(f"{b}: 무장 뒤 시각 기준. 에러 코드 0 이 아닌 표본 수 (FL FR RL RR): {[(W[:, 13+i] != 0).sum() for i in range(4)]}, fresh false: {[(W[:, 17+i] == 0).sum() for i in range(4)]}, "
      f"온도 최대 °C: {[float(W[:, 9+i].max()) for i in range(4)]}")
print("  t s | 기준vy 추정vy | ω FL FR RL RR | τ* FL RR | τ_out FL RR (보상 FL RR) | 최종 mode τ FL RR | 모터 보고 τ FL FR RL RR")
t_end = act[-1] + 2.0
for tt in np.arange(act[0] - t0 + 2.0, t_end - t0, dt):
    t = t0 + tt
    w = [at(W, t, 1 + i) for i in range(4)]; tm = [at(W, t, 5 + i) for i in range(4)]
    mf, mr = at(Tm, t, 2), at(Tm, t, 5); of, orr = at(To, t, 2), at(To, t, 5)
    print(f"{tt:5.2f} | {np.interp(t, R[:, 0], R[:, 1], left=0, right=0):+.2f} {np.interp(t, E[:, 0], E[:, 2]):+.2f} | " + " ".join(f"{x:+6.1f}" for x in w) +
          f" | {mf:+.3f} {mr:+.3f} | {of:+.3f} {orr:+.3f} ({of-mf:+.3f} {orr-mr:+.3f}) | {int(at(Tc, t, 1))} {at(Tc, t, 2):+.3f} {at(Tc, t, 5):+.3f} | " +
          " ".join(f"{x:+.3f}" for x in tm))
