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
