"""비교군 A 계산부 — 퓨어 퍼슛(amr_mpc/baseline/pure_pursuit.py)과 바퀴별 PI(wheel_pi.py), 노드 없이."""
import math

import numpy as np
import pytest

from amr_mpc.baseline.pure_pursuit import PurePursuitConfig, find_lookahead, lookahead_distance, pure_pursuit
from amr_mpc.baseline.wheel_pi import WheelPI, wheel_targets
from amr_mpc.core import config
from amr_mpc.core.reference import ReferenceHorizon
from amr_mpc.torque.model import SlipPlant, TorquePlant


def circle_ref(R=1.0, V=0.5):
    """반지름 R, 속도 V 의 반시계 원 (원점 출발, +x 방향), 시각 → (Xr, ur)."""
    def at(t):
        th = V * t / R
        return np.array([R * math.sin(th), R * (1 - math.cos(th)), th]), np.array([V, 0.0, V / R])
    return at


def line_ref(V=0.5):
    def at(t):
        return np.array([V * t, 0.0, 0.0]), np.array([V, 0.0, 0.0])
    return at


# ── 퓨어 퍼슛 ─────────────────────────────────────────────────

def test_lookahead_scales_with_speed_and_clamps():
    c = PurePursuitConfig(lookahead_time=0.5, lookahead_min=0.15, lookahead_max=0.6)
    assert lookahead_distance(0.5, c) == pytest.approx(0.25)
    assert lookahead_distance(-0.5, c) == pytest.approx(0.25)
    assert lookahead_distance(0.1, c) == pytest.approx(0.15)       # 아래 바닥
    assert lookahead_distance(3.0, c) == pytest.approx(0.6)        # 위 천장


def test_on_circle_gives_exact_curvature():
    """원 위를 제 시각에 달리고 있으면 κ = 1/R, v = V, ω = V/R (교과서 퓨어 퍼슛의 성질 그대로)."""
    R, V = 0.8, 0.5
    at = circle_ref(R, V)
    c = PurePursuitConfig()
    for t in (0.0, 1.3, 4.0):
        Xr, _ = at(t)
        v, w, info = pure_pursuit(c, at, t, Xr)
        assert info["mode"] == "pursuit"
        assert info["kappa"] == pytest.approx(1.0 / R, rel=1e-3)
        assert v == pytest.approx(V) and w == pytest.approx(V / R, rel=1e-3)
        assert np.hypot(*(info["target"] - Xr[:2])) == pytest.approx(lookahead_distance(V, c), rel=1e-3)


def test_lateral_offset_steers_back():
    at = line_ref()
    c = PurePursuitConfig()
    _, w_left, _ = pure_pursuit(c, at, 1.0, [0.5, 0.05, 0.0])     # 기준보다 왼쪽(+y) → 오른쪽으로(ω < 0)
    _, w_right, _ = pure_pursuit(c, at, 1.0, [0.5, -0.05, 0.0])
    assert w_left < 0 < w_right and w_left == pytest.approx(-w_right)


def test_time_lag_speeds_up_and_aims_at_reference_point():
    """뒤처지면 ① 진행 속도가 k_along·e 만큼 빨라지고 ② L_d 밖의 기준점을 바로 겨눈다."""
    at = line_ref(0.5)
    c = PurePursuitConfig(k_along=1.0)
    v, w, info = pure_pursuit(c, at, 2.0, [0.6, 0.0, 0.0])        # 기준점 x = 1.0, 로봇 0.4 m 뒤
    assert info["e_along"] == pytest.approx(0.4)
    assert v == pytest.approx(0.5 + 0.4)
    assert np.allclose(info["target"], [1.0, 0.0]) and info["lead_s"] == 0.0
    v0, _, _ = pure_pursuit(PurePursuitConfig(k_along=0.0), at, 2.0, [0.6, 0.0, 0.0])
    assert v0 == pytest.approx(0.5)                               # k_along = 0 이면 교과서처럼 v = v_ref


def test_ahead_waits_without_reversing():
    at = line_ref(0.5)
    v, _, info = pure_pursuit(PurePursuitConfig(k_along=1.0), at, 1.0, [1.2, 0.0, 0.0])   # 0.7 m 앞섬
    assert v == 0.0                                                # 후진하지 않고 선다
    assert np.hypot(*(info["target"] - [1.2, 0.0])) == pytest.approx(0.25, rel=1e-3)      # 원 밖 첫 점


def test_rotate_in_place_segment():
    def at(t):        # 제자리 정렬 회전: 위치 고정, ω_ref = 0.5
        return np.array([0.0, 0.0, 0.5 * t]), np.array([0.0, 0.0, 0.5])
    c = PurePursuitConfig(k_heading=2.0)
    v, w, info = pure_pursuit(c, at, 1.0, [0.0, 0.0, 0.4])
    assert info["mode"] == "rotate"
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(0.5 + 2.0 * 0.1)


def test_lookahead_stops_at_horizon_end():
    """경로가 멈추면(끝점 유지) 원 밖 점이 없다 → 마지막 점을 겨눈다 (예외·무한 루프 없음)."""
    rows = [(0.01 * i, 0.0, 0.0, 0.05, 0.0, 0.0) for i in range(1, 6)]
    ref = ReferenceHorizon.from_rows(0.0, 0.2, rows)
    p, lead = find_lookahead(ref.at, 0.2, [0.0, 0.0], 0.3, PurePursuitConfig(search_horizon=3.0))
    assert np.allclose(p, [0.05, 0.0]) and lead == pytest.approx(3.0)


def test_figure8_crossing_follows_time_not_nearest_branch():
    """8자 교차점에서 다른 가지가 더 가까워도 기준 시각의 가지를 따라간다 (가까운 점 탐색을 안 하므로)."""
    from amr_mpc.core.figure8 import Figure8
    f8 = Figure8(width=3.0, height=1.5, speed=0.5, align_s=1.5)
    at = lambda t: f8.at(t)
    t_cross = f8.T_align + f8.T_path / 2                          # 한 바퀴 중간 = 교차점을 다시 지나는 때
    Xr, _ = at(t_cross)
    assert np.hypot(*Xr[:2]) < 0.05
    v, w, info = pure_pursuit(PurePursuitConfig(), at, t_cross, Xr)
    ahead, _ = at(t_cross + 0.5)
    assert np.hypot(*(info["target"] - ahead[:2])) < 0.1


# ── 바퀴별 PI ─────────────────────────────────────────────────

def test_inverse_kinematics_matches_plant():
    p = TorquePlant.from_yaml()
    om = wheel_targets(p.Tinv, [0.5, 0.0, 0.0])
    assert np.allclose(om, 0.5 / p.r)                               # 전진: 네 바퀴 모두 v/r
    om = wheel_targets(p.Tinv, [0.0, 0.0, 1.0])
    assert np.allclose(om, np.array([-1, 1, -1, 1]) * p.k / p.r)   # 반시계 회전: 왼쪽(FL, RL) 뒤로


def test_pi_proportional_and_integral():
    pi = WheelPI(kp=0.1, ki=1.0, tau_max=10.0)
    u = pi.step(np.full(4, 2.0), np.zeros(4), 0.01)
    assert np.allclose(u, 0.1 * 2.0 + 1.0 * 2.0 * 0.01)
    u = pi.step(np.full(4, 2.0), np.zeros(4), 0.01)
    assert np.allclose(u, 0.2 + 0.04)


def test_pi_wheels_independent():
    pi = WheelPI(kp=0.1, ki=0.0, tau_max=10.0)
    u = pi.step([1.0, 0.0, -2.0, 0.5], np.zeros(4), 0.01)
    assert np.allclose(u, [0.1, 0.0, -0.2, 0.05])


def test_pi_amplitude_and_rate_limits():
    pi = WheelPI(kp=10.0, ki=0.0, tau_max=1.3, dtau_rate=43.33)     # B 와 같은 값: Δτ̄ = 1.3/3 per 10 ms
    u = pi.step(np.full(4, 5.0), np.zeros(4), 0.01)
    assert np.allclose(u, 0.4333, atol=1e-4)                         # 첫 틱은 변화율에 걸린다
    for _ in range(5):
        u = pi.step(np.full(4, 5.0), np.zeros(4), 0.01)
    assert np.allclose(u, 1.3)                                       # 크기 상한


def test_pi_anti_windup():
    """상한에 걸린 동안 적분이 쌓이지 않아, 오차 부호가 바뀌면 바로 내려온다."""
    pi = WheelPI(kp=0.01, ki=1.0, tau_max=0.1)
    for _ in range(500):                                             # 5 s 동안 큰 오차로 포화
        u = pi.step(np.full(4, 10.0), np.zeros(4), 0.01)
    assert np.allclose(u, 0.1)
    assert np.all(pi.ki * pi.integ <= 0.1 + 1e-12)
    u = pi.step(np.full(4, -1.0), np.zeros(4), 0.01)                # 오차 반전
    assert np.all(u < 0.1)
    naive = 0.01 * -1.0 + 1.0 * (10.0 * 5.0)                         # 와인드업이면 여기서도 +상한
    assert naive > 0.1


def test_pi_reset_clears_state():
    pi = WheelPI(kp=0.1, ki=1.0, tau_max=1.0, dtau_rate=10.0)
    pi.step(np.ones(4), np.zeros(4), 0.01)
    pi.reset()
    assert np.all(pi.integ == 0.0) and np.all(pi.u_prev == 0.0)


def test_pi_dt_is_clamped():
    pi = WheelPI(kp=0.0, ki=1.0, tau_max=10.0, dt_max=0.03)
    u = pi.step(np.ones(4), np.zeros(4), 5.0)                       # 메시지가 5 s 끊겼다 와도
    assert np.allclose(u, 0.03)


def test_pi_tracks_wheel_speed_on_plant():
    """설정 파일 게인으로 무슬립·슬립(μ 0.3) 플랜트에서 전진 0.3 m/s 계단을 한 틱 지연을 두고 따라간다 (발진 없음)."""
    p = TorquePlant.from_yaml()
    cfg = config.load("torque_baseline_a.yaml")["wheel_pi"]
    for mu in (0.0, 0.3):
        pi = WheelPI(cfg["kp"], cfg["ki"], p.tau_motor_limit(), p.tau_motor_limit() / 3 / 0.01)
        slip = SlipPlant(p, mu) if mu > 0 else None
        X, Xd, w, tau = np.zeros(3), np.zeros(3), np.zeros(4), np.zeros(4)
        ref = wheel_targets(p.Tinv, [0.3, 0.0, 0.0])
        hist = []
        for _ in range(300):
            om = w if slip else p.wheel_speeds(X, Xd)
            nxt = pi.step(ref, om, 0.01)
            for _ in range(20):
                if slip:
                    X, Xd, w = slip.step(X, Xd, w, tau, 0.0005)
                else:
                    X, Xd = p.step(X, Xd, tau, 0.0005)
            tau = nxt
            hist.append(om.copy())
        hist = np.array(hist)
        assert np.allclose(hist[-50:], ref, atol=0.05 * ref[0]), mu   # 3 s 뒤 5 % 안
        assert hist.max() < 1.3 * ref[0], mu                          # 넘침 30 % 아래


def test_pure_pursuit_node_refuses_frame_mismatch():
    """리뷰 R7: 기준열과 상태의 frame_id 가 다르면 명령을 끊는다 (torque_mpc_node R5 검사와 같게)."""
    from unittest.mock import Mock

    import rclpy
    from nav_msgs.msg import Odometry

    from amr_mpc.nodes.pure_pursuit import PurePursuitNode
    rclpy.init()
    try:
        n = PurePursuitNode()
        n.pub = Mock()
        n.pub_status, n.pub_la = Mock(), Mock()
        now = n.now_s()
        rows = [(0.5 * 0.05 * i, 0.0, 0.0, 0.5, 0.0, 0.0) for i in range(80)]
        n.ref = ReferenceHorizon.from_rows(now, 0.05, rows)
        n.ref_rx, n.armed, n.armed_since = now, True, now - 1.0

        def state(frame):
            m = Odometry()
            m.header.frame_id = frame
            m.header.stamp = n.get_clock().now().to_msg()
            m.pose.pose.orientation.w = 1.0
            return m

        n.ref_frame = "map"
        n.on_state(state("odom"))
        assert not n.pub.publish.called                     # 다르면 명령 없음 (wheel_pi 가 DAMPING)
        n.on_state(state("map"))
        assert n.pub.publish.called                         # 같으면 추종
        n.destroy_node()
    finally:
        rclpy.shutdown()
