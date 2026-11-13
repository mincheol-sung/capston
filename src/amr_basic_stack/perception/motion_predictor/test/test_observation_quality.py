"""Observation quality survives tracker, predictor and the ROS wire format."""
from types import SimpleNamespace

from amr_interfaces.msg import MultiModalObjectPredictionArray

from builtin_interfaces.msg import Time

from motion_predictor.core.predictor import ImmMotionPredictor
from motion_predictor.motion_predictor_node import MotionPredictorNode

import numpy as np

from object_tracker.core.config import TrackerConfig
from object_tracker.core.track import Track
from object_tracker.object_tracker_node import ObjectTrackerNode

from rclpy.serialization import deserialize_message, serialize_message


def test_lost_track_quality_survives_prediction_serialization():
    """Keep lifecycle and measurement evidence separate from motion weights."""
    cfg = TrackerConfig(person_confirm_hits=1)
    camera = SimpleNamespace(class_id=0, class_name='person', confidence=.9, source=0)
    track = Track(7, [1., 2.], np.eye(2) * .01, 1., cfg, camera)
    for stamp in (1.1, 1.2):
        track.predict(stamp)
        track.update([1., 2.], np.eye(2) * .01, stamp, camera)
    track.predict(1.5)
    track.mark_missed(1.5)
    tracker_node = SimpleNamespace(track_frame='odom', cfg=cfg)
    modes = ObjectTrackerNode._to_modes_msg(
        tracker_node, [track], Time(sec=1, nanosec=500000000))
    published = []
    predictor_node = SimpleNamespace(
        _components=MotionPredictorNode._components,
        predictor=ImmMotionPredictor(cfg, horizon=2.5),
        pub=SimpleNamespace(publish=published.append))
    MotionPredictorNode._on_tracks(predictor_node, modes)
    decoded = deserialize_message(serialize_message(published[0]),
                                  MultiModalObjectPredictionArray)
    obj = decoded.objects[0]
    assert obj.track_id == 7 and obj.class_name == 'person'
    assert obj.lifecycle_state == 2
    assert np.isclose(obj.measurement_age, .3)
    assert np.isclose(obj.semantic_age, .3)
    assert obj.last_measurement_source == 0
    assert np.isclose(obj.class_confidence, .9)
    assert obj.time_offsets[0] == 0.0
    assert [mode.model_type for mode in obj.modes] == [0, 1, 2]
    assert np.isclose(sum(mode.points[0].weight for mode in obj.modes), 1.)
    assert len(obj.modes[0].points) == 14
    assert np.isclose(obj.modes[0].points[0].mean_x,
                      modes.objects[0].modes[0].state[0])
