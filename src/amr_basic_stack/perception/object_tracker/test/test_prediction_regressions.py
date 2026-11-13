"""Regressions for ghost classes, gated assignment and recorded input timing."""
from types import SimpleNamespace

import numpy as np

from object_tracker.core.association import assign, assign_observations, cost_matrix
from object_tracker.core.config import TrackerConfig
from object_tracker.core.pending import PendingMeasurements
from object_tracker.core.track import CONFIRMED, DELETED, LOST, Track
from object_tracker.core.tracker import MultiObjectTracker

R = np.eye(2) * 0.01


def detection(class_id=0, name='person', source=0):
    """Construct a semantic or anonymous observation."""
    return SimpleNamespace(class_id=class_id, class_name=name,
                           confidence=0.9, source=source)


def test_forbidden_edge_cannot_displace_two_valid_pairs():
    """Use the feasible cross-assignment instead of dropping a row."""
    matched, rows, cols = assign(np.array([[1., 9.], [9., 10.]]), 9.21)
    assert matched == {0: 1, 1: 0}
    assert rows == cols == []


def test_rectangular_assignment_leaves_forbidden_rows_unmatched():
    """Dummy assignments handle invalid rows without stealing valid columns."""
    matched, rows, cols = assign(np.array([[np.inf], [2.], [np.nan]]), 9.21)
    assert matched == {1: 0}
    assert rows == [0, 2] and cols == []


def test_camera_has_priority_over_a_closer_anonymous_cluster():
    """Anonymous geometry cannot starve an eligible camera update."""
    ds = [detection(-1, 'lidar_cluster', 1), detection()]
    matched, _, unmatched = assign_observations(np.array([[0.1, 2.]]), 9.21, ds)
    assert matched == {0: 1}
    assert unmatched == [0]


def test_confident_incompatible_classes_are_not_matched():
    """Known class conflicts are blocked while anonymous updates remain valid."""
    trk = Track(0, [0, 0], R, 0, TrackerConfig(person_confirm_hits=1), detection())
    ds = [detection(56, 'chair'), detection(-1, 'lidar_cluster', 1)]
    cost = cost_matrix([trk], [np.zeros(2), np.zeros(2)], [R, R], ds)
    assert np.isinf(cost[0, 0]) and np.isfinite(cost[0, 1])


def test_one_camera_hit_then_lidar_hits_does_not_confirm_person():
    """Geometric confirmation is not repeated semantic confirmation."""
    tracker = MultiObjectTracker()
    tracker.step(0., [np.zeros(2)], [R], [detection()])
    for i in range(1, 15):
        out = tracker.step(i * 0.1, [np.zeros(2)], [R],
                           [detection(-1, 'lidar_cluster', 1)])
    assert len(out) == 1
    assert out[0].class_id == -1


def test_lidar_updates_do_not_renew_person_semantic_lifetime():
    """Retain geometry and short occlusion but eventually expire the class."""
    tracker = MultiObjectTracker(TrackerConfig(person_label_timeout=1.))
    for i in range(3):
        out = tracker.step(i * 0.1, [np.zeros(2)], [R], [detection()])
    assert out[0].class_name == 'person'
    track_id = out[0].id
    for i in range(3, 10):
        out = tracker.step(i * 0.1, [np.zeros(2)], [R],
                           [detection(-1, 'lidar_cluster', 1)])
    assert out[0].class_name == 'person'  # short occlusion is retained
    for i in range(10, 20):
        out = tracker.step(i * 0.1, [np.zeros(2)], [R],
                           [detection(-1, 'lidar_cluster', 1)])
    assert out[0].id == track_id and out[0].class_id == -1
    assert out[0].last_semantic_update == 0.2
    assert out[0].last_measurement_source == 1


def test_stationary_person_remains_while_camera_confirms_it():
    """A stationary real person must not disappear merely for being STATIC."""
    tracker = MultiObjectTracker(TrackerConfig(person_label_timeout=0.5))
    for i in range(40):
        out = tracker.step(i * 0.1, [np.zeros(2)], [R], [detection()])
    assert len(out) == 1 and out[0].class_name == 'person'
    assert out[0].motion == 1


def test_reversed_and_duplicate_inputs_do_not_mutate_tracks():
    """Old observations cannot update the current state or hit counter."""
    tracker = MultiObjectTracker()
    for stamp in (1., 1.1, 1.2):
        tracker.step(stamp, [np.zeros(2)], [R])
    state = tracker.tracks[0].kf.x.copy()
    hits = tracker.tracks[0].hits
    for stamp in (1.1, 1.2):
        tracker.step(stamp, [np.ones(2)], [R])
    assert tracker.tracks[0].hits == hits
    assert np.array_equal(tracker.tracks[0].kf.x, state)
    assert tracker.rejected_timestamps == 2


def test_long_input_outage_does_not_resurrect_old_id():
    """A long outage expires the old identity before association."""
    tracker = MultiObjectTracker()
    for stamp in (0., 0.1, 0.2):
        tracker.step(stamp, [np.zeros(2)], [R])
    old_id = tracker.tracks[0].id
    tracker.step(3., [np.zeros(2)], [R])
    assert len(tracker.tracks) == 1 and tracker.tracks[0].id != old_id


def _person_track(track_id, x, lifecycle, cfg, stamp=0.):
    track = Track(track_id, [x, 0.], R, stamp, cfg, detection())
    track.lifecycle = lifecycle
    return track


def test_active_track_has_priority_over_closer_lost_person():
    cfg = TrackerConfig(person_confirm_hits=1)
    tracker = MultiObjectTracker(cfg)
    active = _person_track(0, 0., CONFIRMED, cfg)
    lost = _person_track(1, .05, LOST, cfg)
    tracker.tracks = [active, lost]
    tracker._next_id = 2
    tracker._last_stamp = 0.

    tracker.step(.1, [np.array([.05, 0.])], [R], [detection()])

    assert active.last_update == .1
    assert lost.last_update == 0.
    assert tracker.next_id == 2


def test_recent_lost_person_reconnects_outside_normal_gate():
    cfg = TrackerConfig(
        gate_chi2=.5, person_reid_gate_chi2=16.,
        person_reid_max_distance=.6, person_confirm_hits=1)
    tracker = MultiObjectTracker(cfg)
    lost = _person_track(4, 0., LOST, cfg)
    tracker.tracks = [lost]
    tracker._next_id = 5
    tracker._last_stamp = 0.

    tracker.step(.1, [np.array([.3, 0.])], [R], [detection()])

    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].id == 4
    assert tracker.tracks[0].lifecycle == CONFIRMED
    assert tracker.next_id == 5


def test_ambiguous_lost_people_are_not_forced_to_reconnect():
    cfg = TrackerConfig(person_confirm_hits=1)
    tracker = MultiObjectTracker(cfg)
    tracker.tracks = [
        _person_track(0, -.1, LOST, cfg),
        _person_track(1, .1, LOST, cfg),
    ]
    tracker._next_id = 2
    tracker._last_stamp = 0.

    tracker.step(.1, [np.zeros(2)], [R], [detection()])

    assert tracker.next_id == 3
    assert {track.id for track in tracker.tracks} == {0, 1, 2}


def test_new_confirmed_person_supersedes_only_an_older_lost_owner():
    cfg = TrackerConfig(person_confirm_hits=1)
    tracker = MultiObjectTracker(cfg)
    old = _person_track(0, 0., LOST, cfg, stamp=0.)
    new = _person_track(1, .05, CONFIRMED, cfg, stamp=.2)
    tracker.tracks = [old, new]

    tracker._supersede_person_duplicates(.3, {1})

    assert old.lifecycle == DELETED and old.superseded_by == 1
    assert tracker.last_superseded == [(0, 1)]


def test_simultaneously_observed_people_are_never_superseded():
    cfg = TrackerConfig(person_confirm_hits=1)
    tracker = MultiObjectTracker(cfg)
    old = _person_track(0, 0., LOST, cfg, stamp=.2)
    new = _person_track(1, .05, CONFIRMED, cfg, stamp=.2)
    tracker.tracks = [old, new]

    tracker._supersede_person_duplicates(.3, {1})

    assert old.lifecycle == LOST and tracker.last_superseded == []


def test_tf_queue_waits_without_consuming_then_processes_in_order():
    """TF availability releases an ordered backlog without blocking callbacks."""
    queue = PendingMeasurements(max_wait=0.25)
    done = []
    queue.add(20, 'second', 0.)
    queue.add(10, 'first', 0.)
    queue.drain(0.1, lambda _: None, lambda m, tf: done.append(m))
    assert done == []
    queue.drain(0.2, lambda _: 'tf', lambda m, tf: done.append(m))
    assert done == ['first', 'second']
    queue.add(10, 'old', 0.21)
    assert queue.dropped['non_increasing'] == 1


def test_tf_queue_expiry_and_capacity_are_bounded():
    """Expired, duplicate and overflowing entries are counted and discarded."""
    queue = PendingMeasurements(max_wait=0.25, capacity=2)
    for stamp in (1, 2, 3):
        queue.add(stamp, stamp, 0.)
    assert queue.dropped['overflow'] == 1
    queue.drain(0.3, lambda _: None, lambda *_: None)
    assert queue.dropped['tf_expired'] == 2
    queue.add(4, 4, 0.4)
    queue.add(4, 4, 0.4)
    assert queue.dropped['non_increasing'] == 1
