"""fault_monitor 의 판정 함수."""
from amr_mpc.nodes.fault_monitor import check_faults

CFG = dict(max_track_error=0.30, timeout=0.3, trip_on_pose_jump=True)


def ok_state(now):
    return dict(jump_count=2, jump_count_at_arm=2, track_error=0.01, motor_error=[0, 0, 0, 0],
                wheel_rx=now - 0.01, state_rx=now - 0.01)


def test_nothing_when_disarmed_or_healthy():
    assert check_faults(10.0, False, CFG, {}) == []
    assert check_faults(10.0, True, CFG, ok_state(10.0)) == []


def test_each_fault_is_detected():
    for key, val, word in (("jump_count", 3, "pose_jump"), ("track_error", 0.31, "track_error"),
                           ("motor_error", [0, 5, 0, 0], "motor_error"), ("wheel_rx", 9.5, "wheel_timeout"),
                           ("state_rx", None, "state_timeout")):
        st = ok_state(10.0)
        st[key] = val
        out = check_faults(10.0, True, CFG, st)
        assert len(out) == 1 and out[0].startswith(word), (key, out)


def test_pose_jump_can_be_disabled():
    st = ok_state(10.0); st["jump_count"] = 9
    assert check_faults(10.0, True, {**CFG, "trip_on_pose_jump": False}, st) == []


def test_state_timeout_can_be_disabled():
    # 텔레옵(teleop.launch.py)은 추정기 없이 돈다 — 추정기 끊김은 안 보고, 바퀴 끊김·모터 에러는 그대로 본다 (2026-11-23)
    cfg = {**CFG, "check_state_timeout": False}
    st = ok_state(10.0); st["state_rx"] = None
    assert check_faults(10.0, True, cfg, st) == []
    st["wheel_rx"] = 9.0
    out = check_faults(10.0, True, cfg, st)
    assert len(out) == 1 and out[0].startswith("wheel_timeout"), out
    st = ok_state(10.0); st["state_rx"] = None; st["motor_error"] = [0, 255, 0, 0]
    out = check_faults(10.0, True, cfg, st)
    assert len(out) == 1 and out[0].startswith("motor_error"), out


def test_teleop_mode_checks_motor_and_wheels_only(monkeypatch):
    """2026-11-28: 패드 텔레옵 중 (/teleop/armed) 에는 모터 에러·바퀴 끊김만 — 추종 오차·포즈 튐·추정기 끊김은 안 본다."""
    from unittest.mock import Mock
    import rclpy
    from amr_mpc.nodes.fault_monitor import FaultMonitorNode
    rclpy.init(args=['--ros-args', '-p', 'teleop_armed_topic:=/teleop/armed'])
    try:
        n = FaultMonitorNode()
        n.pub_arm, n.pub_fault = Mock(), Mock()
        now = n.now()
        n.teleop = True
        n.st = dict(jump_count=9, jump_count_at_arm=0, track_error=5.0, motor_error=[0] * 4, wheel_rx=now, state_rx=None)
        n.on_tick()
        assert not n.pub_arm.publish.called
        n.st["motor_error"] = [0, 255, 0, 0]
        n.on_tick()
        assert n.pub_arm.publish.call_args.args[0].data is False and n.teleop is False
        n.destroy_node()
    finally:
        rclpy.shutdown()
