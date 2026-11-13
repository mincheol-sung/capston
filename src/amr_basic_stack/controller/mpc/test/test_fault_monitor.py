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
