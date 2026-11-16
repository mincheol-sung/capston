"""상태 추정기(amr_mpc/torque/estimator.py) — 가상 궤적으로 IMU·SLAM 을 만들어 참값과 비교한다."""
import math

import numpy as np
import pytest

from amr_mpc.torque.estimator import PlanarStateEstimator, design_gains, wrap

R_IMU = (0.098, -0.030)
CFG = dict(imu_offset_x=R_IMU[0], imu_offset_y=R_IMU[1], init_s=1.0, history_s=0.6, max_pose_age=0.4,
           jump_xy=0.15, jump_yaw_deg=15.0, jump_reset_count=5, omega_lpf_hz=0.0)


def truth(t):
    """t < 2 s 정지, 이후 속도가 변하는 원 주행 (반경 1.2 m, 접선 방향 헤딩). 반환 (p[2], psi)."""
    R, s = 1.2, max(0.0, t - 2.0)
    phi = 0.5 * s + 0.25 * (1.0 - math.cos(0.8 * s)) / 0.8 if s > 0 else 0.0   # φ̇ = 0.5 + 0.25 sin(0.8 s) → 0.3~0.9 m/s
    if s > 0 and s < 1.0:                       # 출발 1 s 는 부드럽게 (φ̇ 연속)
        k = s * s * (3 - 2 * s)
        phi = phi * k
    return np.array([R * math.sin(phi), R * (1.0 - math.cos(phi))]), phi


def simulate(est, T=14.0, imu_hz=200.0, pose_period=0.1, pose_delay=0.13, bias_a=(-0.15, 0.13), bias_g=0.002,
             r=R_IMU, noise=None, seed=0, pose_hook=None):
    """추정기를 돌리고 (t, 오차[x, y, psi, vx, vy, wz]) 배열을 돌려준다. 오차는 로봇 중심 기준, 속도는 몸체 좌표.
    IMU 와 SLAM 포즈는 실차처럼 로봇 중심에서 r 만큼 떨어진 점(tracking_frame)의 것이다."""
    rng = np.random.default_rng(seed)
    n = noise or dict(accel=0.0, gyro=0.0, xy=0.0, yaw=0.0)
    dt, h = 1.0 / imu_hz, 1e-4
    pending, out, next_pose = [], [], 0.0
    for k in range(int(T * imu_hz)):
        t = k * dt
        p0, y0 = truth(t - h); p1, y1 = truth(t); p2, y2 = truth(t + h)
        acc = (p2 - 2 * p1 + p0) / (h * h)
        w = (y2 - y0) / (2 * h)
        al = (y2 - 2 * y1 + y0) / (h * h)
        c, s = math.cos(y1), math.sin(y1)
        ab = np.array([c * acc[0] + s * acc[1], -s * acc[0] + c * acc[1]])
        # 진동 잡음은 움직일 때만 크다 (bag 실측: 정지 0.015 m/s²·0.003 rad/s, 주행 0.95·0.09)
        moving = t > 2.0
        sa, sg = (n["accel"], n["gyro"]) if moving else (min(n["accel"], 0.015), min(n["gyro"], 0.003))
        a_imu = ab + al * np.array([-r[1], r[0]]) - w * w * np.array(r) + np.array(bias_a) + rng.normal(0, sa, 2)
        if t >= next_pose:                                      # 스캔 시각의 포즈가 pose_delay 뒤에 도착
            pt = p1 + np.array([c * r[0] - s * r[1], s * r[0] + c * r[1]])                    # tracking_frame 위치
            pending.append((t + pose_delay, t, pt[0] + rng.normal(0, n["xy"]), pt[1] + rng.normal(0, n["xy"]),
                            y1 + rng.normal(0, n["yaw"])))
            next_pose += pose_period
        while pending and pending[0][0] <= t:
            _, tm, x, y, yaw = pending.pop(0)
            if pose_hook:
                x, y, yaw = pose_hook(tm, x, y, yaw)
            est.pose(tm, x, y, wrap(yaw))
        if est.imu(t, a_imu[0], a_imu[1], w + bias_g + rng.normal(0, sg)):
            _, x, y, psi, vx, vy, wz = est.state()
            vb = np.array([c * (p2 - p0)[0] + s * (p2 - p0)[1], -s * (p2 - p0)[0] + c * (p2 - p0)[1]]) / (2 * h)
            out.append((t, x - p1[0], y - p1[1], wrap(psi - y1), vx - vb[0], vy - vb[1], wz - w))
    return np.array(out)


def test_gain_design_signs_and_ranges():
    K_xy, K_yaw = design_gains(0.1, 0.07, 0.02, 0.003, 0.0064, 0.0005, math.radians(0.2))
    assert 0.0 < K_xy[0] < 1.0 and K_xy[1] > 0.0 and K_xy[2] < 0.0
    assert 0.0 < K_yaw[0] < 1.0 and K_yaw[1] < 0.0
    # 포즈 잡음을 키우면 포즈를 덜 믿는다
    K2, _ = design_gains(0.1, 0.07, 0.02, 0.03, 0.0064, 0.0005, math.radians(0.2))
    assert K2[0] < K_xy[0]


def test_static_init_finds_biases():
    est = PlanarStateEstimator(CFG)
    simulate(est, T=1.6)
    assert est.ready
    assert est.b_a == pytest.approx([-0.15, 0.13], abs=1e-6)
    assert est.b_g == pytest.approx(0.002, abs=1e-9)


def test_noise_free_tracks_truth_with_delayed_poses():
    # 포즈가 0.13 s 늦게 와도 (0.9 m/s 면 12 cm 어긋난 값) 그 시각의 추정과 비교하므로 오차가 없어야 한다
    est = PlanarStateEstimator(CFG)
    o = simulate(est)
    m = o[:, 0] > 4.0
    assert np.abs(o[m, 1:3]).max() < 1e-3            # 위치 1 mm
    assert np.abs(o[m, 3]).max() < math.radians(0.05)
    assert np.abs(o[m, 4:6]).max() < 5e-3            # 속도 5 mm/s
    assert est.jump_count == 0


def test_tracking_point_offset_matters():
    # /tracked_pose 를 로봇 중심의 포즈로 잘못 보면 (offset 0) 위치가 r 만큼, 속도가 ω·r 만큼 틀린다
    # — 2026-11-11 실제 bag 에서 제자리 회전 중 옆 속도 45 mm/s 로 나타난 오차
    on = simulate(PlanarStateEstimator(CFG))
    off = simulate(PlanarStateEstimator({**CFG, "imu_offset_x": 0.0, "imu_offset_y": 0.0}))
    m = on[:, 0] > 4.0
    assert np.hypot(off[m, 1], off[m, 2]).mean() == pytest.approx(math.hypot(*R_IMU), abs=0.01)
    assert np.abs(off[m, 4:6]).max() > 5.0 * np.abs(on[m, 4:6]).max()


def test_pose_reference_point_can_differ_from_imu():
    # 포즈 기준점이 로봇 중심이고 IMU 만 떨어져 있는 구성 (pose_offset = 0)
    est = PlanarStateEstimator({**CFG, "pose_offset_x": 0.0, "pose_offset_y": 0.0})
    t = 0.0
    for k in range(600):                         # 정지: 포즈는 중심 (1, 2), IMU 는 그보다 r 만큼 앞
        t += 0.005
        if k % 20 == 0:
            est.pose(t - 0.13, 1.0, 2.0, 0.0)
        est.imu(t, 0.0, 0.0, 0.0)
    _, x, y, psi, vx, vy, wz = est.state()
    assert (x, y) == pytest.approx((1.0, 2.0), abs=1e-6)
    assert est.imu_point_state()[:2] == pytest.approx((1.0 + R_IMU[0], 2.0 + R_IMU[1]), abs=1e-6)


def test_realistic_noise_levels():
    # 2026-11-11 bag 실측 수준: 가속도 σ 0.95 m/s², 자이로 0.09 rad/s (200 Hz, 주행 중), 포즈 3 mm / 0.2°
    est = PlanarStateEstimator(CFG)
    o = simulate(est, T=30.0, noise=dict(accel=0.95, gyro=0.09, xy=0.003, yaw=math.radians(0.2)), seed=3)
    m = o[:, 0] > 4.0
    rms = lambda c: float(np.sqrt(np.mean(o[m, c] ** 2)))
    print(f"\n  실측 수준 잡음: 위치 RMS {math.hypot(rms(1), rms(2))*1e3:.2f} mm, 방향 {math.degrees(rms(3)):.3f}°, "
          f"속도 {math.hypot(rms(4), rms(5))*1e3:.1f} mm/s, 각속도 {rms(6):.4f} rad/s")
    # 2026-11-11 실측: 위치 9.7 mm, 방향 0.19°, 속도 52 mm/s. 진동 잡음(0.95 m/s²)을 포즈 지연 0.13 s + 주기 0.1 s 동안
    # 적분하는 데서 온다 — 게인을 바꿔도 거의 그대로다. 회귀 감시용 문턱은 그보다 넉넉하게.
    assert math.hypot(rms(1), rms(2)) < 0.015
    assert rms(3) < math.radians(0.5)
    assert math.hypot(rms(4), rms(5)) < 0.08
    assert est.jump_count == 0


def test_bias_change_is_tracked():
    # 초기화 뒤 가속도계 바이어스가 0.1 m/s² 달라져도 (기울기 변화) 수렴한다
    est = PlanarStateEstimator(CFG)
    simulate(est, T=1.6)
    est.b_a = est.b_a + np.array([0.1, -0.1])
    o = simulate(est, T=1.0)                          # 같은 추정기를 이어 돌릴 수 없으므로 새로 검증
    est2 = PlanarStateEstimator(CFG)
    simulate(est2, T=1.6)
    est2.b_a = est2.b_a + np.array([0.1, -0.1])
    # 정지 상태에서 포즈만 받아도 바이어스가 돌아온다
    t = 1.6
    for k in range(int(10.0 * 200)):
        t += 0.005
        if k % 20 == 0:
            est2.pose(t - 0.13, R_IMU[0], R_IMU[1], 0.0)        # 정지: tracking_frame 은 중심에서 r 만큼 떨어진 곳
        est2.imu(t, -0.15, 0.13, 0.002)
    assert est2.b_a == pytest.approx([-0.15, 0.13], abs=0.01)
    assert abs(est2.v[0]) < 0.005 and abs(est2.v[1]) < 0.005
    assert o is not None


def test_pose_jump_is_ignored_then_followed():
    est = PlanarStateEstimator(CFG)
    simulate(est, T=1.6)
    t = 1.6

    def run(n_poses, x):
        nonlocal t
        res = []
        for k in range(n_poses * 20):
            t += 0.005
            if k % 20 == 0:
                res.append(est.pose(t - 0.13, R_IMU[0] + x, R_IMU[1], 0.0))   # 포즈는 tracking_frame 의 것
            est.imu(t, -0.15, 0.13, 0.002)
        return res

    assert run(3, 0.0) == ["used"] * 3
    assert run(1, 0.5) == ["jump"]                    # 한 번 튄 포즈는 버린다
    assert abs(est.state()[1]) < 1e-3                 # 로봇 중심은 그대로 원점
    assert run(1, 0.0) == ["used"]
    assert run(5, 0.5) == ["jump"] * 4 + ["reset"]    # 계속 그러면 SLAM 을 따라간다
    assert est.state()[1] == pytest.approx(0.5, abs=5e-3)
    assert est.jump_count == 6


def test_stale_pose_is_rejected():
    est = PlanarStateEstimator(CFG)
    simulate(est, T=1.6)
    assert est.pose(est.t - 1.0, 0.3, 0.0, 0.0) == "old"
    assert abs(est.state()[1]) < 1e-9


# ── 시뮬 센서(amr_mpc/torque/sim_sensors.py) + 추정기 ──────────────────────────────────
from amr_mpc.torque.sim_sensors import SimSensors

QUIET = dict(accel_noise_still=0.0, accel_noise_moving=0.0, gyro_noise_still=0.0, gyro_noise_moving=0.0,
             slam_noise_xy=0.0, slam_noise_yaw_deg=0.0)


def run_with_sim_sensors(sensor_cfg, pose_source, T=20.0, est_cfg=None, seed=0):
    """참 궤적(truth) → SimSensors → 추정기. 반환 (t, 오차[x, y, psi, vx, vy, wz]) — 로봇 중심 기준."""
    sens = SimSensors({"imu_offset_x": R_IMU[0], "imu_offset_y": R_IMU[1], **sensor_cfg}, seed=seed)
    est = PlanarStateEstimator({**CFG, **(est_cfg or {})})
    h, out, take = 1e-4, [], False
    for k in range(int(T * 200)):
        t = k * 0.005
        p0, y0 = truth(t - h); p1, y1 = truth(t); p2, y2 = truth(t + h)
        Xd = np.array([(p2 - p0)[0] / (2 * h), (p2 - p0)[1] / (2 * h), (y2 - y0) / (2 * h)])
        s = sens.step(t, np.array([p1[0], p1[1], y1]), Xd)
        if s["scan_matched"]:
            take = True
        for (ts, x, y, yaw) in s["poses"]:
            if pose_source == "stamped" or take or not est.pose_ready:
                est.pose(ts, x, y, yaw)
                take = False
        if s["imu"] and est.imu(t, s["imu"][0], s["imu"][1], s["imu"][3]):
            _, x, y, psi, vx, vy, wz = est.state()
            c, sn = math.cos(y1), math.sin(y1)
            out.append((t, x - p1[0], y - p1[1], wrap(psi - y1), vx - (c * Xd[0] + sn * Xd[1]), vy - (-sn * Xd[0] + c * Xd[1]), wz - Xd[2]))
    return np.array(out), est


def test_sim_sensors_scan_time_mode_noise_free():
    o, est = run_with_sim_sensors({**QUIET, "slam_mode": "scan_time"}, "stamped")
    m = o[:, 0] > 5.0
    assert np.hypot(o[m, 1], o[m, 2]).max() < 1e-3
    assert np.hypot(o[m, 4], o[m, 5]).max() < 5e-3
    assert est.b_a == pytest.approx([-0.15, 0.13], abs=2e-3)


def test_sim_sensors_extrapolated_mode_carries_extrapolation_error():
    # 외삽 스트림에서 고른 표본에는 Cartographer 가 0.13 s 를 일정 속도로 외삽한 오차가 들어 있다.
    # 잡음이 없어도 가속 구간에서 오차가 남는다 — scan_time 모드보다 나쁘다.
    a, _ = run_with_sim_sensors({**QUIET, "slam_mode": "scan_time"}, "stamped")
    b, _ = run_with_sim_sensors({**QUIET, "slam_mode": "extrapolated"}, "after_scan_match")
    ma, mb = a[:, 0] > 5.0, b[:, 0] > 5.0
    ea, eb = np.hypot(a[ma, 1], a[ma, 2]).max(), np.hypot(b[mb, 1], b[mb, 2]).max()
    print(f"\n  잡음 없음, 위치 오차 최대: scan_time {ea*1e3:.2f} mm   extrapolated {eb*1e3:.2f} mm")
    assert eb > 3.0 * ea
    assert eb < 0.03                                 # 그래도 발산하지 않는다


def test_sim_sensors_realistic_noise_both_modes():
    for mode, src in (("scan_time", "stamped"), ("extrapolated", "after_scan_match")):
        o, est = run_with_sim_sensors({"slam_mode": mode}, src, T=30.0, seed=5)
        m = o[:, 0] > 5.0
        pos = float(np.sqrt(np.mean(o[m, 1] ** 2 + o[m, 2] ** 2)))
        vel = float(np.sqrt(np.mean(o[m, 4] ** 2 + o[m, 5] ** 2)))
        print(f"\n  실측 수준 잡음 [{mode}]: 위치 RMS {pos*1e3:.1f} mm, 속도 RMS {vel*1e3:.0f} mm/s, 방향 {math.degrees(np.sqrt(np.mean(o[m, 3]**2))):.2f}°")
        assert pos < 0.02 and vel < 0.09 and est.jump_count == 0


def test_pose_dropout_stops_output():
    """R3 리뷰: SLAM 포즈가 끊기면 IMU 적분만으로 계속 내지 않고 max_dead_reckoning 뒤 출력을 멈춘다."""
    est = PlanarStateEstimator(dict(CFG, max_dead_reckoning=0.5))
    t = 0.0
    while t < 1.05:                                   # 정지 바이어스 초기화
        est.imu(t, 0.0, 0.0, 0.0); t += 0.005
    assert est.pose(t, 0.0, 0.0, 0.0) == "init"
    for _ in range(40):                               # 0.2 s 동안 IMU + 0.1 s 마다 포즈
        est.imu(t, 0.0, 0.0, 0.0); t += 0.005
        if abs((t * 10) % 1) < 1e-6:
            est.pose(t - 0.13, 0.0, 0.0, 0.0)
    est.pose(t, 0.0, 0.0, 0.0)
    assert est.ready_to_publish() and est.dead_reckoning_age() < 0.01
    for _ in range(80):                               # 포즈 없이 0.4 s — 아직 냄
        est.imu(t, 0.0, 0.0, 0.0); t += 0.005
    assert est.ready_to_publish()
    for _ in range(40):                               # 0.6 s 째 — 멈춤
        est.imu(t, 0.0, 0.0, 0.0); t += 0.005
    assert not est.ready_to_publish() and est.dead_reckoning_age() > 0.5
    assert est.pose(t - 0.13, 0.0, 0.0, 0.0) == "used"   # 포즈가 다시 오면 바로 재개
    assert est.ready_to_publish()
    est0 = PlanarStateEstimator(dict(CFG, max_dead_reckoning=0.0))   # 0 이면 검사 안 함
    assert est0.max_dead_reckoning == 0.0


# ── 실차 배달 방식(QoS) 점검 (2026-11-13) ─────────────────────────────────────────
#   실차 imu_to_base_node 는 /imu/base_link 를 BEST_EFFORT 로 낸다. 추정기가 RELIABLE 로 받으면 연결이 막혀
#   IMU 가 0 개가 된다. 시뮬이 RELIABLE 로 내던 동안은 이 잘못이 시뮬에서 안 드러났다.
def _deliveries(publisher_reliability):
    """주어진 배달 방식으로 IMU 를 내면 추정기가 몇 개 받는가."""
    import time

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu

    from amr_mpc.nodes.state_estimator import StateEstimatorNode
    topic = f"/qos_test_{publisher_reliability.name.lower()}/imu"      # 돌고 있는 다른 시뮬과 안 섞이게
    rclpy.init(args=["--ros-args", "-p", f"imu_topic:={topic}"])
    try:
        est = StateEstimatorNode()
        src = Node("qos_test_imu_source")
        pub = src.create_publisher(Imu, topic, QoSProfile(reliability=publisher_reliability,
                                                          history=HistoryPolicy.KEEP_LAST, depth=20))
        ex = SingleThreadedExecutor()
        ex.add_node(est)
        ex.add_node(src)
        end = time.monotonic() + 5.0
        while est.n_imu == 0 and time.monotonic() < end:
            m = Imu()
            m.header.stamp = src.get_clock().now().to_msg()
            pub.publish(m)
            ex.spin_once(timeout_sec=0.02)
        n = est.n_imu
        est.destroy_node()
        src.destroy_node()
        return n
    finally:
        rclpy.shutdown()


def test_estimator_receives_best_effort_imu_like_real_robot():
    from rclpy.qos import ReliabilityPolicy
    assert _deliveries(ReliabilityPolicy.BEST_EFFORT) > 0      # 실차 imu_to_base_node


def test_estimator_still_receives_reliable_imu():
    from rclpy.qos import ReliabilityPolicy
    assert _deliveries(ReliabilityPolicy.RELIABLE) > 0         # RELIABLE 로 내는 다른 발행자도 받는다


def test_sim_imu_is_published_best_effort_like_real_robot():
    """시뮬 IMU 를 실차와 같은 BEST_EFFORT 로 내야, RELIABLE 구독 실수를 시뮬에서도 잡는다."""
    import rclpy
    from rclpy.qos import ReliabilityPolicy

    from amr_mpc.nodes.sim_plant_torque import SimPlantTorqueNode
    rclpy.init(args=["--ros-args", "-p", "sensors:=true", "-p", "imu_topic:=/qos_test_sim/imu",
                     "-p", "pose_topic:=/qos_test_sim/pose", "-p", "wheel_states_topic:=/qos_test_sim/ws",
                     "-p", "truth_topic:=/qos_test_sim/truth", "-p", "cmd_topic:=/qos_test_sim/cmd"])
    try:
        n = SimPlantTorqueNode()
        assert n.pub_imu.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        n.destroy_node()
    finally:
        rclpy.shutdown()
