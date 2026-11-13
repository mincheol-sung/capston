"""Behavioral coverage for semantic inputs, three models and measured timing."""
from types import SimpleNamespace

import numpy as np
import pytest

from object_tracker.core.config import TrackerConfig
from object_tracker.core.imm import ImmEstimator, transition_matrix
from object_tracker.core.measurements import (
    semantic_measurement, tracking_measurement)
from object_tracker.core.motion_models import StaticKalmanModel
from object_tracker.core.motion_state import MotionClassifier, STATIC, DYNAMIC, UNKNOWN


def detection(source=0, name='chair', covariance=None):
    return SimpleNamespace(
        source=source, class_name=name, class_id=56, confidence=.9,
        position=SimpleNamespace(x=1., y=2.),
        position_covariance=([.02, -.005, -.005, .01]
                             if covariance is None else covariance))


@pytest.mark.parametrize('name', ['person', 'chair', 'backpack', 'bicycle'])
@pytest.mark.parametrize('source', [0, 2])
def test_all_camera_semantic_classes_are_accepted(name, source):
    assert semantic_measurement(detection(source, name), .1) is not None


@pytest.mark.parametrize('source', [1, 3, 255])
def test_non_camera_sources_are_rejected(source):
    assert semantic_measurement(detection(source), .1) is None


@pytest.mark.parametrize('source', [0, 1, 2])
def test_all_object_sources_are_accepted_for_tracking(source):
    assert tracking_measurement(detection(source), .1) is not None


@pytest.mark.parametrize('source', [3, 255])
def test_unknown_sources_are_rejected_for_tracking(source):
    assert tracking_measurement(detection(source), .1) is None


@pytest.mark.parametrize('covariance', [
    [1., 2., 2., 1.], [1., 0., 0., -1.], [1., 1., 0., 1.],
    [np.nan, 0., 0., 1.], [np.inf, 0., 0., 1.], [1., 2.],
])
def test_invalid_covariance_is_rejected(covariance):
    assert semantic_measurement(detection(covariance=covariance), .1) is None


def test_zero_covariance_uses_existing_fallback_and_nan_position_is_rejected():
    obj = detection(covariance=[0.] * 4)
    assert np.allclose(semantic_measurement(obj, .1)[1], np.eye(2)*.01)
    obj.position.x = np.nan
    assert semantic_measurement(obj, .1) is None


def test_static_model_never_moves_with_mixed_in_velocity():
    model = StaticKalmanModel([1., 2., 3., 4., 1.], np.eye(5))
    model.predict(.1, .04, .1)
    assert np.array_equal(model.x[:2], [1., 2.])
    assert np.linalg.norm(model.x[2:]) < np.linalg.norm([3., 4., 1.])
    assert np.linalg.eigvalsh(model.P).min() > 0


def test_three_mode_transition_handles_irregular_time():
    def matrix(dt):
        return transition_matrix(dt, 3., 1., 3.)
    assert matrix(.1).shape == (3, 3)
    assert np.allclose(matrix(.07) @ matrix(.13), matrix(.2))
    assert np.allclose(matrix(.2).sum(axis=1), 1.)
    assert np.all(matrix(.2) >= 0.)


def test_unreachable_zero_weight_mode_stays_finite():
    cfg = TrackerConfig(cv_to_static=0., ct_to_static=0.)
    imm = ImmEstimator.from_components(
        np.zeros((3, 5)), np.tile(np.eye(5), (3, 1, 1)), [1., 0., 0.], cfg)
    imm.predict(.2)
    assert imm.mu[2] == 0.
    assert np.all(np.isfinite(imm.x)) and np.all(np.isfinite(imm.P))


def test_initial_mode_prior_does_not_decide_motion():
    cfg = TrackerConfig()
    imm = ImmEstimator([0., 0.], np.eye(2)*.01, cfg)
    classifier = MotionClassifier(cfg)
    for stamp in np.arange(0., 2., .1):
        assert classifier.update(stamp, imm.mu, True) == UNKNOWN


def test_confirmation_uses_time_and_hysteresis():
    classifier = MotionClassifier(TrackerConfig())
    for stamp in [0., .07, .20, .29]:
        assert classifier.update(stamp, [.8, .1, .1], True) == UNKNOWN
    assert classifier.update(.31, [.8, .1, .1], True) == DYNAMIC
    assert classifier.update(.4, [.5, .2, .3], True) == DYNAMIC
    assert classifier.update(.5, [.2, .2, .6], True) == UNKNOWN
    for stamp in [.6, .7, .8, .9, 1.0]:
        assert classifier.update(stamp, [.05, .05, .9], True) == UNKNOWN
    assert classifier.update(1.1, [.05, .05, .9], True) == STATIC


def test_missed_time_cannot_confirm_and_tentative_remains_unknown():
    classifier = MotionClassifier(TrackerConfig())
    assert classifier.update(0., [.8, .1, .1], True) == UNKNOWN
    assert classifier.update(1., [.8, .1, .1], True) == UNKNOWN
    assert classifier.update(1.2, [.8, .1, .1], False) == UNKNOWN
    assert classifier.update(1.3, [.8, .1, .1], True) == UNKNOWN
    classifier.reset_evidence()
    assert classifier.update(1.7, [.8, .1, .1], True) == UNKNOWN


def test_node_lidar_frame_updates_track_without_semantic_refresh():
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    from object_tracker.core.tracker import MultiObjectTracker
    from object_tracker.object_tracker_node import ObjectTrackerNode
    cfg = TrackerConfig()
    node = SimpleNamespace(
        cfg=cfg, tracker=MultiObjectTracker(cfg), publish_tentative=False,
        _filter_tracks=lambda tracks, stamp: tracks,
        _to_msg=lambda tracks, stamp: tracks,
        _to_modes_msg=lambda tracks, stamp: tracks,
        pub=SimpleNamespace(publish=lambda msg: None),
        observations_pub=SimpleNamespace(publish=lambda msg: None),
        modes_pub=SimpleNamespace(publish=lambda msg: None), marker_pub=None)
    def process(ns, objects):
        msg = SimpleNamespace(objects=objects, header=Header(
            stamp=Time(sec=1, nanosec=ns)))
        ObjectTrackerNode._process_detections(node, msg, (np.zeros(2), np.eye(2)))
    for ns in [0, 100000000, 200000000]:
        process(ns, [detection()])
    assert node.tracker.tracks[0].lifecycle == 1
    semantic_stamp = node.tracker.tracks[0].last_semantic_update
    process(300000000, [detection(source=1)])
    assert node.tracker.tracks[0].lifecycle == 1
    assert node.tracker.tracks[0].hits == 4
    assert node.tracker.tracks[0].last_semantic_update == semantic_stamp
    assert node.tracker.next_id == 1
