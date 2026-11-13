"""Reject mismatched epochs and replayed provenance before an IMM update."""
import copy
from types import SimpleNamespace as NS

from amr_interfaces.msg import DetectedObject, DetectedObjectArray, ObservationProvenance
from object_tracker.core.pending import PendingMeasurements
from object_tracker.object_tracker_node import ObjectTrackerNode
import pytest


def message(sec=1, ns=0):
    msg = DetectedObjectArray()
    msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec = 'base_link', sec, ns
    obj = DetectedObject()
    obj.header = copy.deepcopy(msg.header)
    obs = ObservationProvenance()
    obs.header.frame_id, obs.header.stamp.sec = 'laser_front', 1
    obs.observation_id = '/scan_front|laser_front|1000000000|3,4,5'
    obj.observations = [obs]
    msg.objects = [obj]
    return msg


def node():
    return NS(max_observation_age=.75, _observation_ids={}, pending=PendingMeasurements(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_500_000_000)),
              _drain_detections=lambda: None)


def test_tracker_rejects_coordinate_epoch_mismatch():
    n, msg = node(), message()
    msg.objects[0].header.stamp.nanosec = 1
    ObjectTrackerNode._on_detections(n, msg)
    assert n.pending.dropped['epoch_mismatch'] == 1
    assert not n.pending._heap


def test_tracker_rejects_replayed_observation_at_new_array_epoch():
    n = node()
    ObjectTrackerNode._on_detections(n, message())
    ObjectTrackerNode._on_detections(n, message(ns=100_000_000))
    assert len(n.pending._heap) == 1
    assert n.pending.dropped['duplicate_observation'] == 1


def test_tracker_rejects_stale_and_future_input():
    n = node()
    ObjectTrackerNode._on_detections(n, message(sec=0))
    ObjectTrackerNode._on_detections(n, message(sec=2))
    assert not n.pending._heap
    assert n.pending.dropped['stale_or_future'] == 2


def test_missing_frame_is_not_treated_as_odom():
    n, msg = node(), message()
    msg.header.frame_id = msg.objects[0].header.frame_id = ''
    ObjectTrackerNode._on_detections(n, msg)
    assert n.pending.dropped['missing_frame'] == 1 and not n.pending._heap


@pytest.mark.parametrize('frame,seconds', [('', 1), ('base_link', 0), ('odom', 0)])
def test_invalid_tf_identity_never_uses_identity_or_latest(frame, seconds):
    n = NS(track_frame='odom')  # Any TF access would fail this test.
    assert ObjectTrackerNode._lookup(n, frame, message(sec=seconds).header.stamp) is None


@pytest.mark.parametrize('capture_seconds', [0, .5])
def test_individual_capture_cannot_be_hidden_by_a_fresh_array(capture_seconds):
    n, msg = node(), message()
    obs = msg.objects[0].observations[0]
    obs.header.stamp.sec = int(capture_seconds)
    obs.header.stamp.nanosec = int((capture_seconds % 1)*1e9)
    assert not msg.observations
    ObjectTrackerNode._on_detections(n, msg)
    assert not n.pending._heap and n.pending.dropped['stale_or_future'] == 1
