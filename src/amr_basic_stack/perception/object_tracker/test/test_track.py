"""core.track 단위시험 — 생명주기 FSM, 운동상태 FSM, 클래스 latch."""
from dataclasses import dataclass

import numpy as np
import pytest

from object_tracker.core.config import TrackerConfig
from object_tracker.core.track import (CLASS_ID_LIDAR, CONFIRMED, DELETED,
                                       DYNAMIC, LOST, STATIC, TENTATIVE,
                                       TRACK_UNKNOWN_CLASS_NAME, Track,
                                       UNKNOWN)

DT = 1.0 / 12.0
SIGMA_Z = 0.004
R = np.eye(2) * SIGMA_Z ** 2


@dataclass
class FakeDetection:
    class_id: int
    class_name: str
    confidence: float


def run(track, speed, n, rng, cfg=None):
    """정지/등속 물체를 n 프레임 먹인다. 반환은 (t, 운동상태) 이력."""
    t = track.stamp
    p = track.kf.position.copy()
    hist = []
    for _ in range(n):
        t += DT
        p = p + np.array([speed, 0.0]) * DT
        track.predict(t)
        track.update(p + rng.normal(0, SIGMA_Z, 2), R, t)
        hist.append((t, track.motion))
    return hist


# ── 생명주기 ────────────────────────────────────────────────────────── #

def test_new_track_is_tentative():
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig())
    assert trk.lifecycle == TENTATIVE
    assert not trk.publishable


def test_tentative_is_deleted_on_first_miss():
    """확정 전 트랙을 끌면 다음 프레임의 진짜 검출을 가로챈다."""
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig())
    trk.mark_missed(DT)
    assert trk.lifecycle == DELETED


def test_confirms_after_n_confirm_hits():
    cfg = TrackerConfig()
    trk = Track(0, [0.0, 0.0], R, 0.0, cfg)
    t = 0.0
    for _ in range(cfg.n_confirm - 1):     # 생성이 1 hit 이므로 하나 적게
        t += DT
        trk.predict(t)
        trk.update([0.0, 0.0], R, t)
    assert trk.hits == cfg.n_confirm
    assert trk.lifecycle == CONFIRMED
    assert trk.publishable


def test_confirmed_goes_lost_then_recovers():
    cfg = TrackerConfig()
    trk = Track(0, [0.0, 0.0], R, 0.0, cfg)
    t = 0.0
    for _ in range(cfg.n_confirm):
        t += DT
        trk.predict(t)
        trk.update([0.0, 0.0], R, t)
    assert trk.lifecycle == CONFIRMED

    t += DT
    trk.predict(t)
    trk.mark_missed(t)
    assert trk.lifecycle == LOST
    assert trk.publishable          # coast 중에도 발행한다

    t += DT
    trk.predict(t)
    trk.update([0.0, 0.0], R, t)
    assert trk.lifecycle == CONFIRMED


def test_lost_track_is_deleted_after_max_coast():
    cfg = TrackerConfig()
    trk = Track(0, [0.0, 0.0], R, 0.0, cfg)
    t = 0.0
    for _ in range(cfg.n_confirm):
        t += DT
        trk.predict(t)
        trk.update([0.0, 0.0], R, t)
    t += cfg.max_coast + DT
    trk.predict(t)
    trk.mark_missed(t)
    assert trk.lifecycle == DELETED


def test_predict_ignores_non_positive_dt():
    """검출이 순서 뒤집혀 와도 상태를 되감지 않는다."""
    trk = Track(0, [0.0, 0.0], R, 1.0, TrackerConfig())
    x_before = trk.kf.x.copy()
    assert trk.predict(0.5) == 0.0
    assert np.allclose(trk.kf.x, x_before)
    assert trk.stamp == 1.0


def test_predict_integrates_entire_gap_in_bounded_steps():
    cfg = TrackerConfig()
    trk = Track(0, [0.0, 0.0], R, 0.0, cfg)
    reference = Track(0, [0.0, 0.0], R, 0.0, cfg)
    assert trk.predict(1.2) == pytest.approx(1.2)
    for stamp in (0.5, 1.0, 1.2):
        reference.predict(stamp)
    assert np.allclose(trk.kf.P, reference.kf.P)
    assert trk.stamp == 1.2


# ── 운동상태 ────────────────────────────────────────────────────────── #

def test_motion_is_unknown_while_tentative():
    """새 트랙의 P_vv 는 sigma_v0^2 로 열려 있어 chi^2 가 무조건 작다.

    그대로 판정하면 "속도를 아직 못 재서"가 "정지해 있어서"로 둔갑한다.
    """
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig())
    assert trk.motion == UNKNOWN
    trk.predict(DT)
    trk.update([0.0, 0.0], R, DT)          # hits=2, 아직 TENTATIVE
    assert trk.lifecycle == TENTATIVE
    assert trk.motion == UNKNOWN


def test_static_target_becomes_static():
    rng = np.random.default_rng(0)
    trk = Track(0, [3.0, 0.0], R, 0.0, TrackerConfig())
    run(trk, speed=0.0, n=30, rng=rng)
    assert trk.motion == STATIC
    assert not trk.is_dynamic


def test_walking_target_becomes_dynamic():
    rng = np.random.default_rng(0)
    trk = Track(0, [3.0, 0.0], R, 0.0, TrackerConfig())
    run(trk, speed=1.2, n=30, rng=rng)
    assert trk.motion == DYNAMIC
    assert trk.is_dynamic


def test_static_to_dynamic_and_back_keeps_same_track():
    """멈췄다 다시 걷는 물체가 하나의 트랙으로 상태만 바꾼다."""
    rng = np.random.default_rng(1)
    cfg = TrackerConfig()
    trk = Track(0, [3.0, 0.0], R, 0.0, cfg)
    run(trk, speed=0.0, n=30, rng=rng)
    assert trk.motion == STATIC
    run(trk, speed=1.2, n=30, rng=rng)
    assert trk.motion == DYNAMIC
    run(trk, speed=0.0, n=60, rng=rng)
    assert trk.motion == STATIC
    assert trk.id == 0


def test_deadband_holds_previous_state():
    """진입/이탈 문턱 사이에서는 상태를 유지한다. 이게 hysteresis 다."""
    cfg = TrackerConfig()
    trk = Track(0, [0.0, 0.0], R, 0.0, cfg)
    trk.lifecycle = CONFIRMED
    trk.motion = DYNAMIC

    trk.motion_classifier.state = DYNAMIC
    trk.estimator.mu = np.array([0.5, 0.2, 0.3])
    for _ in range(50):
        trk.stamp += DT
        trk._update_motion()
    assert trk.motion == DYNAMIC          # 불감대에서는 안 바뀐다


def test_dynamic_needs_fewer_frames_than_static():
    """움직이는 물체를 STATIC 이라 부르는 쪽이 위험한 오류다."""
    cfg = TrackerConfig()
    assert cfg.dynamic_confirm_time < cfg.static_confirm_time


# ── 클래스 latch ────────────────────────────────────────────────────── #

def test_camera_class_is_latched():
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig(person_confirm_hits=1),
                FakeDetection(0, 'person', 0.9))
    assert trk.class_id == 0
    assert trk.class_name == 'person'


def test_lidar_only_detection_does_not_erase_class():
    """6.5m 이상에서 카메라가 검출을 못 해도 클래스를 유지한다.

    C2: 6.05m 92.8%, 7.10m 0%. 라이다 단독 관측은 class_id 가 -1 이다.
    """
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig(person_confirm_hits=1),
                FakeDetection(0, 'person', 0.9))
    trk.predict(DT)
    trk.update([0.0, 0.0], R, DT,
               FakeDetection(CLASS_ID_LIDAR, 'lidar_cluster', 1.0))
    assert trk.class_name == 'person'
    assert trk.class_id == 0


def test_lidar_born_track_has_no_class():
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig(),
                FakeDetection(CLASS_ID_LIDAR, 'lidar_cluster', 1.0))
    assert trk.class_id == CLASS_ID_LIDAR
    assert trk.class_name == TRACK_UNKNOWN_CLASS_NAME


def test_camera_class_can_be_revised():
    trk = Track(0, [0.0, 0.0], R, 0.0, TrackerConfig(),
                FakeDetection(0, 'person', 0.5))
    trk.predict(DT)
    trk.update([0.0, 0.0], R, DT, FakeDetection(56, 'chair', 0.8))
    assert trk.class_name == 'chair'
