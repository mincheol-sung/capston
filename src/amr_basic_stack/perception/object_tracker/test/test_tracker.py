"""core.tracker 통합시험 — 여러 물체가 있는 프레임 열을 통째로 돌린다."""
import numpy as np

from object_tracker.core.config import TrackerConfig
from object_tracker.core.track import DYNAMIC, STATIC, TENTATIVE
from object_tracker.core.tracker import MultiObjectTracker

DT = 1.0 / 12.0
SIGMA_Z = 0.004
R = np.eye(2) * SIGMA_Z ** 2


def test_new_measurement_creates_tentative_track():
    trk = MultiObjectTracker()
    published = trk.step(0.0, [np.array([3.0, 0.0])], [R])
    assert published == []                      # TENTATIVE 는 발행 안 한다
    assert len(trk.tracks) == 1
    assert trk.tracks[0].lifecycle == TENTATIVE


def test_spurious_detection_is_dropped():
    """한 프레임만 나타난 검출은 트랙을 남기지 않는다."""
    trk = MultiObjectTracker()
    trk.step(0.0, [np.array([3.0, 0.0])], [R])
    trk.step(DT, [], [])
    assert trk.tracks == []


def test_static_target_tracked_as_static():
    rng = np.random.default_rng(0)
    trk = MultiObjectTracker()
    t = 0.0
    for _ in range(40):
        t += DT
        z = np.array([3.0, 0.0]) + rng.normal(0, SIGMA_Z, 2)
        out = trk.step(t, [z], [R])
    assert len(out) == 1
    assert out[0].motion == STATIC
    assert trk.next_id == 1                     # ID 를 새로 안 만들었다


def test_crossing_targets_keep_their_ids():
    """정지 물체를 지나가는 이동 물체. 겹치는 순간에도 ID 가 안 바뀐다.

    위치만 보면 두 트랙이 같은 점에 있지만 각자의 예측이 달라 Mahalanobis 가
    갈라 준다. 검출 순서는 프레임마다 섞어 순서 의존성을 배제한다.
    """
    rng = np.random.default_rng(7)
    trk = MultiObjectTracker()
    static_p = np.array([3.0, 0.0])
    moving_p = np.array([3.0, -2.0])
    moving_v = np.array([0.0, 0.8])
    t = 0.0
    for _ in range(60):
        t += DT
        moving_p = moving_p + moving_v * DT
        zs = [static_p + rng.normal(0, SIGMA_Z, 2),
              moving_p + rng.normal(0, SIGMA_Z, 2)]
        rng.shuffle(zs)
        out = trk.step(t, zs, [R, R])

    assert trk.next_id == 2, 'ID 가 추가로 생겼다면 교차에서 association 이 깨졌다'
    assert len(out) == 2
    motions = sorted(o.motion for o in out)
    assert motions == [STATIC, DYNAMIC]


def test_occluded_track_survives_short_gap():
    """짧은 가림에서는 coast 하고 같은 ID 로 되돌아온다."""
    rng = np.random.default_rng(2)
    cfg = TrackerConfig()
    trk = MultiObjectTracker(cfg)
    t = 0.0
    for _ in range(20):
        t += DT
        trk.step(t, [np.array([3.0, 0.0]) + rng.normal(0, SIGMA_Z, 2)], [R])
    track_id = trk.tracks[0].id

    for _ in range(4):                          # 4 프레임 = 0.33s < max_coast
        t += DT
        trk.step(t, [], [])
    assert len(trk.tracks) == 1

    t += DT
    out = trk.step(t, [np.array([3.0, 0.0])], [R])
    assert len(out) == 1
    assert out[0].id == track_id
    assert trk.next_id == 1


def test_long_occlusion_deletes_track():
    rng = np.random.default_rng(2)
    cfg = TrackerConfig()
    trk = MultiObjectTracker(cfg)
    t = 0.0
    for _ in range(20):
        t += DT
        trk.step(t, [np.array([3.0, 0.0]) + rng.normal(0, SIGMA_Z, 2)], [R])

    n_gap = int(cfg.max_coast / DT) + 2
    for _ in range(n_gap):
        t += DT
        trk.step(t, [], [])
    assert trk.tracks == []


def test_far_apart_measurement_starts_new_track_not_a_jump():
    """게이트 밖 검출은 기존 트랙을 끌어당기지 않고 새 트랙이 된다."""
    rng = np.random.default_rng(4)
    trk = MultiObjectTracker()
    t = 0.0
    for _ in range(20):
        t += DT
        trk.step(t, [np.array([3.0, 0.0]) + rng.normal(0, SIGMA_Z, 2)], [R])
    first = trk.tracks[0]

    t += DT
    trk.step(t, [np.array([3.0, 0.0]), np.array([-4.0, 5.0])], [R, R])
    assert len(trk.tracks) == 2
    assert first.kf.position[0] > 2.0           # 원래 트랙이 안 끌려갔다


def test_empty_frames_are_safe():
    trk = MultiObjectTracker()
    for k in range(5):
        assert trk.step(k * DT, [], []) == []
    assert trk.tracks == []
