"""값 규약과 병합 규칙 테스트. ROS 없이 돈다."""

import numpy as np

import pytest

from traversability import encoding


def test_blocked_value_survives_consumer_thresholds():
    """100 은 소비자들의 판정식을 통과하고 1 은 통과하지 못한다."""
    # nav2 map_server 의 occupied_thresh 0.65, DistanceField 의 65 와 같은 경계.
    assert encoding.BLOCKED >= encoding.OCCUPIED_THRESHOLD
    assert 1 < encoding.OCCUPIED_THRESHOLD


def test_unknown_round_trips_through_uint8_255():
    """-1 은 바이트로 255 이며, 그 바이트를 int8 로 읽으면 다시 -1 이다."""
    wire = np.array([encoding.UNKNOWN], dtype=np.int8)
    assert wire.view(np.uint8).tolist() == [255]
    assert wire.view(np.uint8).view(np.int8).tolist() == [encoding.UNKNOWN]


def test_classify_base_map_trusting_free():
    base = np.array([[0, 50, 65, 100, -1]], dtype=np.int8)
    out = encoding.classify_base_map(base, trust_free=True)
    assert out.tolist() == [[
        encoding.FREE, encoding.FREE, encoding.BLOCKED,
        encoding.BLOCKED, encoding.UNKNOWN,
    ]]


def test_classify_base_map_without_trusting_free():
    """2단계 정책: 기준 맵의 free 는 통과 가능의 근거가 아니다."""
    base = np.array([[0, 50, 65, 100, -1]], dtype=np.int8)
    out = encoding.classify_base_map(base, trust_free=False)
    assert out.tolist() == [[
        encoding.UNKNOWN, encoding.UNKNOWN, encoding.BLOCKED,
        encoding.BLOCKED, encoding.UNKNOWN,
    ]]


def _blank(shape):
    memory = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
    observed_at = np.full(shape, -np.inf, dtype=np.float64)
    return memory, observed_at


def test_memory_holds_then_expires_per_cell():
    """관측이 끊긴 셀은 memory_seconds 뒤에 미확인으로 돌아간다."""
    shape = (1, 2)
    memory, observed_at = _blank(shape)
    observation = np.array([[encoding.BLOCKED, encoding.UNKNOWN]], dtype=np.int8)

    memory, observed_at = encoding.update_memory(
        memory, observed_at, observation, now=0.0, memory_seconds=5.0)
    assert memory.tolist() == [[encoding.BLOCKED, encoding.UNKNOWN]]

    nothing = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing, now=4.9, memory_seconds=5.0)
    assert memory[0, 0] == encoding.BLOCKED, '5 s 안에는 유지되어야 한다'

    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing, now=5.1, memory_seconds=5.0)
    assert memory[0, 0] == encoding.UNKNOWN, '5 s 를 넘기면 만료되어야 한다'


def test_memory_expiry_is_not_refreshed_by_carrying_itself_forward():
    """기억을 매 주기 다시 실어도 만료 시각이 밀리지 않는다.

    격자 전체에 타임스탬프를 하나만 두면 이 성질이 깨진다. 셀별 observed_at
    을 쓰는 이유가 이것이다.
    """
    shape = (1, 1)
    memory, observed_at = _blank(shape)
    memory, observed_at = encoding.update_memory(
        memory, observed_at,
        np.array([[encoding.BLOCKED]], dtype=np.int8),
        now=0.0, memory_seconds=5.0)

    nothing = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
    for now in np.arange(0.5, 5.0, 0.5):      # 2 Hz 로 아홉 주기 반복
        memory, observed_at = encoding.update_memory(
            memory, observed_at, nothing, now=float(now), memory_seconds=5.0)
    assert memory[0, 0] == encoding.BLOCKED

    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing, now=5.5, memory_seconds=5.0)
    assert memory[0, 0] == encoding.UNKNOWN


def test_compose_prefers_memory_over_base_map():
    memory = np.array([[encoding.BLOCKED, encoding.UNKNOWN]], dtype=np.int8)
    base = np.array([[encoding.FREE, encoding.FREE]], dtype=np.int8)
    assert encoding.compose(memory, base).tolist() == [[
        encoding.BLOCKED, encoding.FREE,
    ]]


def test_compose_never_turns_unobserved_space_into_free():
    """설계문서 §7.1 — 미확인을 빈 공간으로 변환하지 않는다."""
    memory = np.array([[encoding.UNKNOWN]], dtype=np.int8)
    base = np.array([[encoding.UNKNOWN]], dtype=np.int8)
    assert encoding.compose(memory, base).tolist() == [[encoding.UNKNOWN]]


def test_unknown_fill_flags_report_the_sources_used():
    memory = np.array([[encoding.BLOCKED, encoding.UNKNOWN]], dtype=np.int8)
    base = np.array([[encoding.FREE, encoding.FREE]], dtype=np.int8)
    composed = encoding.compose(memory, base)
    flags = encoding.unknown_fill_flags(memory, base, composed)
    # rclpy 의 uint8 필드는 numpy 스칼라를 거부한다. 반드시 Python int 여야 한다.
    assert type(flags) is int
    assert flags & encoding.UNKNOWN_FILL_PREVIOUS
    assert flags & encoding.UNKNOWN_FILL_BASE_MAP

    empty = np.array([[encoding.UNKNOWN, encoding.UNKNOWN]], dtype=np.int8)
    none_flags = encoding.unknown_fill_flags(empty, None,
                                             encoding.compose(empty, None))
    assert none_flags == encoding.UNKNOWN_FILL_NONE


@pytest.mark.parametrize(
    'unknown_is_obstacle, expected',
    [
        # global planner (설계문서 §4): 미확인은 통과 허용
        (False, [[False, True, False]]),
        # local planner: 미확인은 통과 불가
        (True, [[False, True, True]]),
    ],
)
def test_obstacle_mask_unknown_policy_differs_per_consumer(
        unknown_is_obstacle, expected):
    data = [encoding.FREE, encoding.BLOCKED, encoding.UNKNOWN]
    mask = encoding.to_obstacle_mask(data, (1, 3), unknown_is_obstacle)
    assert mask.tolist() == expected


def test_free_observation_overrides_an_occupied_base_map():
    """관측으로 해제한 셀은 기준 맵이 점유라고 해도 통과 가능이 된다.

    설계문서 §6 — "초기 기준 지도에 이동 가능한 팔레트가 이미 포함돼 있다면
    ... 재관측 후 계획용 지도에 별도 해제 규칙을 설계해야 한다."
    """
    memory = np.array([[encoding.FREE, encoding.UNKNOWN]], dtype=np.int8)
    base = np.array([[encoding.BLOCKED, encoding.BLOCKED]], dtype=np.int8)
    assert encoding.compose(memory, base).tolist() == [[
        encoding.FREE, encoding.BLOCKED,
    ]]


def test_free_memory_is_permanent_by_default():
    """FREE 는 free_memory_seconds <= 0 이면 만료되지 않는다.

    광선이 관통했다는 증거는 시간이 지나도 사라지지 않는다. 이것이 기준 맵의
    오래된 점유를 덮는 유일한 수단이다.
    """
    shape = (1, 1)
    memory, observed_at = _blank(shape)
    memory, observed_at = encoding.update_memory(
        memory, observed_at, np.array([[encoding.FREE]], dtype=np.int8),
        now=0.0, memory_seconds=5.0, free_memory_seconds=0.0)

    nothing = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing,
        now=100000.0, memory_seconds=5.0, free_memory_seconds=0.0)
    assert memory[0, 0] == encoding.FREE


def test_free_memory_expires_when_a_ttl_is_given():
    shape = (1, 1)
    memory, observed_at = _blank(shape)
    memory, observed_at = encoding.update_memory(
        memory, observed_at, np.array([[encoding.FREE]], dtype=np.int8),
        now=0.0, memory_seconds=5.0, free_memory_seconds=60.0)
    nothing = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing,
        now=59.0, memory_seconds=5.0, free_memory_seconds=60.0)
    assert memory[0, 0] == encoding.FREE
    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing,
        now=61.0, memory_seconds=5.0, free_memory_seconds=60.0)
    assert memory[0, 0] == encoding.UNKNOWN


def test_blocked_still_expires_while_free_persists():
    """같은 격자에서 BLOCKED 은 5 s 로 만료되고 FREE 는 남는다."""
    shape = (1, 2)
    memory, observed_at = _blank(shape)
    memory, observed_at = encoding.update_memory(
        memory, observed_at,
        np.array([[encoding.BLOCKED, encoding.FREE]], dtype=np.int8),
        now=0.0, memory_seconds=5.0, free_memory_seconds=0.0)

    nothing = np.full(shape, encoding.UNKNOWN, dtype=np.int8)
    memory, observed_at = encoding.update_memory(
        memory, observed_at, nothing,
        now=6.0, memory_seconds=5.0, free_memory_seconds=0.0)
    assert memory.tolist() == [[encoding.UNKNOWN, encoding.FREE]]


def test_obstacle_appears_then_is_removed_then_returns():
    """장애물 추가 → 해제 → 재추가 시퀀스가 격자에 그대로 반영된다.

    기준 맵은 그 셀을 점유로 알고 있다(초기 매핑 때 물체가 있었다).
    """
    shape = (1, 1)
    base = np.array([[encoding.BLOCKED]], dtype=np.int8)
    memory, observed_at = _blank(shape)
    blocked = np.array([[encoding.BLOCKED]], dtype=np.int8)
    free = np.array([[encoding.FREE]], dtype=np.int8)
    nothing = np.full(shape, encoding.UNKNOWN, dtype=np.int8)

    def step(observation, now):
        nonlocal memory, observed_at
        memory, observed_at = encoding.update_memory(
            memory, observed_at, observation, now,
            memory_seconds=5.0, free_memory_seconds=0.0)
        return int(encoding.compose(memory, base)[0, 0])

    # 1) 관측 전 — 기준 맵을 따른다
    assert step(nothing, 0.0) == encoding.BLOCKED
    # 2) 물체를 치우고 광선이 관통 — 해제된다
    assert step(free, 1.0) == encoding.FREE
    # 3) 로봇이 다른 곳을 봄 — 해제가 유지된다
    assert step(nothing, 500.0) == encoding.FREE
    # 4) 물체가 다시 놓임 — 즉시 막힌다
    assert step(blocked, 501.0) == encoding.BLOCKED
    # 5) 다시 치워지고 관측이 끊김 — BLOCKED 만료 후 기준 맵으로 되돌아간다
    assert step(nothing, 507.0) == encoding.BLOCKED
    # 6) 다시 관통 관측 — 또 해제된다
    assert step(free, 508.0) == encoding.FREE


# ── 센서별 근거 기억 (update_memory_sourced) ─────────────────────────────────
#
# 여기서 검증하는 것은 하나다: LiDAR 평면(0.2178 m) 광선이 그보다 낮은 물체를
# 지웠다고 판단하지 않는 것. 설계문서 §6 이 명시적으로 금지한 오류다.

def _empty(shape=(1, 1)):
    return np.zeros(shape, dtype=bool)


def _yes(shape=(1, 1)):
    return np.ones(shape, dtype=bool)


def _blank_sourced(shape=(1, 1)):
    return (
        np.full(shape, encoding.UNKNOWN, dtype=np.int8),
        np.zeros(shape, dtype=np.uint8),
        np.full(shape, -np.inf, dtype=np.float64),
    )


def test_lidar_ray_does_not_clear_a_depth_marked_cell():
    """낮은 팔레트: LiDAR 빔은 위로 지나가고 D455 만 본다.

    D455 가 시야에서 놓친 주기에도 LiDAR 관통만으로 해제되면 안 된다.
    """
    memory, source, seen = _blank_sourced()

    # 1) D455 가 보고 LiDAR 광선은 그 위를 관통한다 — hit 이 이긴다.
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _yes(), _yes(), _empty(),
        now=0.0, memory_seconds=0.0)
    assert memory[0, 0] == encoding.BLOCKED
    assert source[0, 0] == encoding.SOURCE_DEPTH

    # 2) 로봇이 회전해 팔레트가 D455 화각을 벗어난다. LiDAR 는 계속 관통한다.
    #    팔레트는 그 자리에 있으므로 BLOCKED 이 유지돼야 한다.
    for step in range(1, 20):
        memory, source, seen = encoding.update_memory_sourced(
            memory, source, seen,
            _empty(), _empty(), _yes(), _empty(),
            now=float(step), memory_seconds=0.0)
        assert memory[0, 0] == encoding.BLOCKED, step
        assert source[0, 0] == encoding.SOURCE_DEPTH, step


def test_depth_ray_clears_a_depth_marked_cell_when_the_object_is_gone():
    """돌아와서 다시 봤을 때 없으면 지워진다 — 사용자가 요구한 동작."""
    memory, source, seen = _blank_sourced()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _yes(), _empty(), _empty(),
        now=0.0, memory_seconds=0.0)
    assert memory[0, 0] == encoding.BLOCKED

    # 물체를 치웠다. 카메라 광선이 그 셀을 관통하고 depth hit 은 없다.
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _empty(), _empty(), _yes(),
        now=1.0, memory_seconds=0.0)
    assert source[0, 0] == encoding.SOURCE_NONE
    assert memory[0, 0] == encoding.FREE


def test_a_cell_held_by_both_sensors_needs_both_clearings():
    """두 센서가 함께 세운 셀은 한쪽 해제로 풀리지 않는다."""
    memory, source, seen = _blank_sourced()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _yes(), _yes(), _empty(), _empty(),
        now=0.0, memory_seconds=0.0)
    assert source[0, 0] == encoding.SOURCE_LIDAR | encoding.SOURCE_DEPTH

    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _empty(), _yes(), _empty(),
        now=1.0, memory_seconds=0.0)
    assert source[0, 0] == encoding.SOURCE_DEPTH
    assert memory[0, 0] == encoding.BLOCKED

    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _empty(), _empty(), _yes(),
        now=2.0, memory_seconds=0.0)
    assert source[0, 0] == encoding.SOURCE_NONE
    assert memory[0, 0] == encoding.FREE


def test_hit_beats_clear_within_the_same_cycle_for_each_sensor():
    """같은 주기에 겹치면 hit 이 이긴다. 있는 것을 없다고 하지 않는다."""
    memory, source, seen = _blank_sourced()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _yes(), _yes(), _yes(), _yes(),
        now=0.0, memory_seconds=0.0)
    assert source[0, 0] == encoding.SOURCE_LIDAR | encoding.SOURCE_DEPTH
    assert memory[0, 0] == encoding.BLOCKED


def test_zero_memory_seconds_means_no_blocked_expiry():
    """free_memory_seconds 와 같은 규약. 예전 update_memory 는 즉시 만료였다."""
    memory, source, seen = _blank_sourced()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _yes(), _empty(), _empty(), _empty(),
        now=0.0, memory_seconds=0.0)
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _empty(), _empty(), _empty(),
        now=1000.0, memory_seconds=0.0)
    assert memory[0, 0] == encoding.BLOCKED
    assert source[0, 0] == encoding.SOURCE_LIDAR


def test_blocked_expiry_also_drops_the_source_bits():
    """만료됐는데 근거가 남으면 다음 주기에 곧바로 다시 BLOCKED 을 주장한다."""
    memory, source, seen = _blank_sourced()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _yes(), _yes(), _empty(), _empty(),
        now=0.0, memory_seconds=5.0)
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _empty(), _empty(), _empty(),
        now=6.0, memory_seconds=5.0)
    assert memory[0, 0] == encoding.UNKNOWN
    assert source[0, 0] == encoding.SOURCE_NONE


def test_no_evidence_keeps_the_memory_unchanged():
    """관측이 끊긴 것을 해제 근거로 쓰지 않는다(§6)."""
    memory, source, seen = _blank_sourced()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _yes(), _empty(), _empty(),
        now=0.0, memory_seconds=0.0)
    before = memory.copy()
    memory, source, seen = encoding.update_memory_sourced(
        memory, source, seen,
        _empty(), _empty(), _empty(), _empty(),
        now=1.0, memory_seconds=0.0)
    assert memory.tolist() == before.tolist()


def test_inputs_are_not_mutated():
    """반환 배열이 새것이어야 노드가 이전 상태를 안전하게 비교할 수 있다."""
    memory, source, seen = _blank_sourced()
    m0, s0, t0 = memory.copy(), source.copy(), seen.copy()
    encoding.update_memory_sourced(
        memory, source, seen,
        _yes(), _yes(), _empty(), _empty(),
        now=0.0, memory_seconds=0.0)
    assert memory.tolist() == m0.tolist()
    assert source.tolist() == s0.tolist()
    assert seen.tolist() == t0.tolist()
