"""torque_output 의 계산부 — 마찰 보상과 안전 클램프 (노드 없이)."""
import numpy as np
import pytest

from amr_mpc.nodes.torque_output import friction_torque, motor_command


def test_zero_friction_is_identity():
    tau, clipped = motor_command([0.01, -0.02, 0.03, 0.0], [10, -10, 5, 0], 0.0, 0.0, 0.5, 1.3)
    assert np.allclose(tau, [0.01, -0.02, 0.03, 0.0]) and not clipped


def test_friction_follows_wheel_direction():
    f = friction_torque([20.0, -20.0, 0.0, 0.01], coulomb=0.03, viscous=0.001, smooth=0.5)
    assert f[0] == pytest.approx(0.03 + 0.02, abs=1e-6)       # 충분히 빠르면 tanh → 1
    assert f[1] == pytest.approx(-0.05, abs=1e-6)
    assert f[2] == 0.0                                        # 정지에서는 보상 0
    assert 0.0 < f[3] < 0.001                                 # 0 근처는 부드럽게 잇는다 (부호 튐 없음)


def test_safety_clamp():
    tau, clipped = motor_command([2.0, -2.0, 0.5, 0.0], np.zeros(4), 0.0, 0.0, 0.5, 1.3)
    assert np.allclose(tau, [1.3, -1.3, 0.5, 0.0]) and clipped


def test_friction_reserve_warning():
    """리뷰 R1: 마찰 보상을 켜면 tau_fric_max 가 coulomb 이상이어야 한다 (MPC 상한에 몫을 남김)."""
    from amr_mpc.nodes.torque_output import friction_reserve_warning
    assert friction_reserve_warning(0.0, 0.0, 0.0) is None            # 기본값 (보상 없음)
    assert friction_reserve_warning(0.026, 0.0016, 0.0) is not None    # coulomb 만 넣고 상한 몫을 잊음
    assert friction_reserve_warning(0.026, 0.0016, 0.06) is None       # 몫을 남김


def test_nan_wheel_speed_drops_only_that_compensation():
    """리뷰 R6: 바퀴 속도의 NaN 은 그 바퀴 마찰 보상만 0 으로 (토크가 NaN 이 되지 않는다)."""
    tau, _ = motor_command([0.05] * 4, [float('nan'), 20.0, float('inf'), 0.0], 0.03, 0.0, 0.5, 1.3)
    assert np.all(np.isfinite(tau))
    assert tau[0] == pytest.approx(0.05) and tau[2] == pytest.approx(0.05)
    assert tau[1] == pytest.approx(0.08, abs=1e-6)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_non_finite_torque_becomes_damping(bad):
    """리뷰 R6: MPC 토크에 NaN·inf 가 섞이면 DAMPING 으로 바꿔 낸다.
    (그대로 두면 inf 는 클램프에서 최대 토크, NaN 은 CAN 다리에서 토크 0 = 관성 주행.)"""
    from unittest.mock import Mock

    import rclpy

    from amr_interfaces.msg import WheelTorqueCommand
    from amr_mpc.nodes.torque_output import TorqueOutputNode
    rclpy.init()
    try:
        n = TorqueOutputNode()
        n.pub = Mock()
        m = WheelTorqueCommand(mode=WheelTorqueCommand.MODE_TORQUE, torque=[0.1, bad, 0.1, 0.1])
        m.header.stamp.sec = 77
        n.on_cmd(m)
        out = n.pub.publish.call_args.args[0]
        assert out.mode == WheelTorqueCommand.MODE_DAMPING
        assert out.header.stamp.sec == 77
        n.on_cmd(WheelTorqueCommand(mode=WheelTorqueCommand.MODE_TORQUE, torque=[0.1] * 4))   # 정상이면 다시 토크
        assert n.pub.publish.call_args.args[0].mode == WheelTorqueCommand.MODE_TORQUE
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_individual_bench_friction_and_motor_order():
    """2026-11-16 장착 그대로 띄운 정상 상태 실측 (analysis/torque_mpc_real/2026-11-16_jw/ss_*.csv, friction_steady.py)."""
    from amr_mpc.torque.model import TorquePlant
    p = TorquePlant.from_yaml()
    tc = np.array([0.0174, 0.0206, 0.0155, 0.0157])
    bv = np.array([0.001066, 0.001158, 0.001112, 0.001177])
    assert p.fric_coulomb == pytest.approx(tc)
    assert p.fric_viscous == pytest.approx(bv)
    # Equal speeds still produce distinct compensation; reverse speed reverses each torque.
    expected = tc + 10 * bv
    got = friction_torque([10] * 4, p.fric_coulomb, p.fric_viscous, 0.5)
    assert got == pytest.approx(expected)
    assert friction_torque([-10] * 4, tc, bv, 0.5) == pytest.approx(-expected)
    assert friction_torque([0] * 4, tc, bv, 0.5) == pytest.approx(np.zeros(4))
    # 정격 속도의 마찰 최대 0.0655 (FR, 26 rad/s 위는 직선 외삽) — 보상은 × 0.9 = 0.059 로 tau_fric_max 0.1 안.
    rated_speed = 370 * 2 * np.pi / 60
    assert p.fric_max == pytest.approx(0.1)
    assert np.max(tc + bv * rated_speed) == pytest.approx(0.06547, abs=1e-4)
    assert 0.9 * np.max(tc + bv * rated_speed) < p.fric_max
    assert p.tau_motor_limit() == pytest.approx(1.2)


def test_compensation_scale_applies_to_output_only():
    """torque_output 은 실측 × compensation_scale 로 보상하고, 시뮬 플랜트(TorquePlant)는 실측 그대로 쓴다."""
    import rclpy
    from amr_mpc.torque.model import TorquePlant
    from amr_mpc.nodes.torque_output import TorqueOutputNode
    from amr_mpc.core import config
    scale = float(config.load("torque_plant.yaml")["friction"]["compensation_scale"])
    assert scale == pytest.approx(0.9)
    p = TorquePlant.from_yaml()
    rclpy.init()
    try:
        n = TorqueOutputNode()
        assert n.coulomb == pytest.approx(scale * p.fric_coulomb)
        assert n.viscous == pytest.approx(scale * p.fric_viscous)
        n.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('bad', [[0.01, 0.02], [0, 0, 0, np.nan], [0, -0.01, 0, 0]])
def test_rejects_invalid_wheel_friction(bad):
    from amr_mpc.torque.model import wheel_coefficients
    with pytest.raises(ValueError):
        wheel_coefficients(bad)


def test_restored_reserve_warns_only_when_operating_speed_needs_more():
    """여유 tau_fric_max 가 운용 속도의 마찰보다 작을 때만 경고. 2026-11-16 마찰이면 정격 속도에서도 0.1 안 (0.0655)."""
    from amr_mpc.torque.model import TorquePlant
    from amr_mpc.nodes.torque_output import friction_reserve_warning
    p = TorquePlant.from_yaml()
    rated = 370 * 2 * np.pi / 60
    assert friction_reserve_warning(p.fric_coulomb, p.fric_viscous, 0.1, rated) is None
    assert friction_reserve_warning(p.fric_coulomb, p.fric_viscous, 0.05, rated)        # 여유가 작으면 경고
    assert friction_reserve_warning(p.fric_coulomb, p.fric_viscous, 0.05, 10.0) is None


# ── 정지 마찰 보상 (2026-11-16) ───────────────────────────────────────────
def test_static_off_keeps_old_formula():
    """omega_ref 가 None 이거나 static 이 0 이면 예전 식 그대로."""
    w = [3.0, -0.2, 0.0, 12.0]
    old = friction_torque(w, coulomb=0.02, viscous=0.002, smooth=0.5)
    assert np.allclose(friction_torque(w, 0.02, 0.002, 0.5, omega_ref=None, static=0.08), old)
    assert np.allclose(friction_torque(w, 0.02, 0.002, 0.5, omega_ref=[5, 5, 5, 5], static=0.0), old)


def test_static_at_standstill_follows_reference_direction():
    """바퀴가 서 있으면(ω = 0) 운동 마찰 항은 0, 정지 항이 기준 바퀴속도 방향으로 F_s·tanh(ω_ref/ω_s)."""
    f = friction_torque([0, 0, 0, 0], 0.02, 0.002, 0.5, omega_ref=[3.0, -3.0, 0.0, 0.25], static=0.08)
    assert f[0] == pytest.approx(0.08 * np.tanh(6.0))          # 0.07999
    assert f[1] == pytest.approx(-0.08 * np.tanh(6.0))
    assert f[2] == 0.0                                          # 기준이 멈춰 있으면 밀지 않는다
    assert f[3] == pytest.approx(0.08 * np.tanh(0.5))           # 0 근처 기준은 부드럽게


def test_static_fades_once_wheel_turns():
    """|ω| = ω_st 이면 e⁻¹, 1 rad/s 면 e⁻⁴, 5 rad/s 면 사실상 0 — 바퀴가 돌면 예전 식으로 돌아간다.
    역회전 중(ω > 0, ω_ref < 0) 바퀴가 빠르게 돌면 정지 항이 측정 방향 보상을 뒤집지 않는다."""
    st, ws = 0.08, 0.5
    for w, factor in [(0.5, np.exp(-1.0)), (1.0, np.exp(-4.0))]:
        f = friction_torque([w] * 4, 0.0, 0.0, 0.5, omega_ref=[3.0] * 4, static=st, stribeck=ws)
        assert f[0] == pytest.approx(st * factor * np.tanh(6.0))
    f = friction_torque([5.0] * 4, 0.02, 0.002, 0.5, omega_ref=[-3.0] * 4, static=st, stribeck=ws)
    assert f[0] == pytest.approx(0.02 * np.tanh(10.0) + 0.002 * 5.0, abs=1e-12)


def test_motor_command_with_static_and_bag_case():
    """bag 2026-11-16/142534 의 붙은 순간: τ_net = −τ̄(−0.0759), ω = 0, ω_ref(FL) = −4 rad/s.
    예전 명령 −0.0759 (출발 토크 ≈ 0.09 못 넘음) → 보상 0.08×0.9 를 더해 −0.148."""
    tau, clipped = motor_command([-0.0759] * 4, [0.0] * 4, 0.9 * 0.0214, 0.9 * 0.002, 0.5, 1.3,
                                 omega_ref=[-4.0] * 4, static=0.9 * 0.08)
    assert tau[0] == pytest.approx(-0.0759 - 0.072 * np.tanh(8.0), abs=1e-9)
    assert tau[0] < -0.14 and not clipped
    # ω_ref 의 NaN 은 그 바퀴 정지 항만 0
    tau, _ = motor_command([0.0] * 4, [0.0] * 4, 0.0, 0.0, 0.5, 1.3, omega_ref=[np.nan, 3, 3, 3], static=0.08)
    assert tau[0] == 0.0 and tau[1] > 0.07


def test_reference_wheel_speed_timing():
    """references[i] 목표 시각 = stamp + (i+1)·dt. 사이는 선형 보간, 첫 목표 전이면 첫 행, 오래됐거나 지평 끝이면 0."""
    from amr_mpc.nodes.torque_output import reference_wheel_speed
    from amr_mpc.torque.model import TorquePlant
    Tinv = TorquePlant.from_yaml().Tinv
    rows = np.array([[0.1, 0.0, 0.0], [0.2, 0.0, 0.0], [0.0, 0.3, 0.0]])
    ref = (100.0, 0.05, rows)
    r = 0.04
    assert reference_wheel_speed(ref, 100.02, Tinv, 0.5) == pytest.approx(Tinv @ rows[0])          # 첫 목표(100.05) 전
    assert reference_wheel_speed(ref, 100.075, Tinv, 0.5) == pytest.approx(Tinv @ [0.15, 0, 0])   # 1·2 행 가운데
    assert reference_wheel_speed(ref, 100.075, Tinv, 0.5)[0] == pytest.approx(0.15 / r)
    w = reference_wheel_speed(ref, 100.15, Tinv, 0.5)                                            # 마지막 행: 게걸음 +y
    assert w == pytest.approx(np.array([-1, 1, 1, -1]) * 0.3 / r)                               # FL·RR 역회전
    assert np.all(reference_wheel_speed(ref, 100.16, Tinv, 0.5) == 0)                            # 지평 끝
    assert np.all(reference_wheel_speed((100.0, 0.05, np.tile(rows, (20, 1))), 100.6, Tinv, 0.5) == 0)  # 0.6 s 묵음
    assert np.all(reference_wheel_speed(None, 100.0, Tinv, 0.5) == 0)


@pytest.mark.parametrize("on", [True, False])
def test_node_static_from_yaml_times_scale(on):
    """노드: static = torque_plant.yaml friction.static_Nm × compensation_scale, stiction_comp:=false 면 0."""
    import rclpy
    from amr_mpc.core import config
    from amr_mpc.nodes.torque_output import TorqueOutputNode
    fric = config.load("torque_plant.yaml")["friction"]
    rclpy.init(args=["--ros-args", "-p", f"stiction_comp:={'true' if on else 'false'}"])
    try:
        n = TorqueOutputNode()
        expect = fric["compensation_scale"] * np.asarray(fric["static_Nm"], float) if on else np.zeros(4)
        assert n.static == pytest.approx(expect)
        assert n.has_static == on and (not on or np.all(n.static > 0))
        n.destroy_node()
    finally:
        rclpy.shutdown()
