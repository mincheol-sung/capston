"""Exercise the semantic ROS wire chain without DDS or actuator commands."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('object_tracker')
pytest.importorskip('motion_predictor')

from amr_interfaces.msg import (
    DetectedObject, DetectedObjectArray, MultiModalObjectPredictionArray,
)
from builtin_interfaces.msg import Time
from dra_mppi.dra_mppi_node import DraMppiNode, filter_predictions_for_control
from dra_mppi.dynamic_risk import DynamicRiskConfig, dynamic_risk_cost
from motion_predictor.core.predictor import ImmMotionPredictor
from motion_predictor.motion_predictor_node import MotionPredictorNode
from object_tracker.core.config import TrackerConfig
from object_tracker.core.tracker import MultiObjectTracker
from object_tracker.object_tracker_node import ObjectTrackerNode
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time as RosTime


@pytest.mark.parametrize('speed,frames,expected_motion', [
    (0., 3, 0), (0., 50, 1), (.6, 50, 2),
])
def test_detection_to_collision_risk(speed, frames, expected_motion):
    cfg = TrackerConfig()
    modes_messages = []
    node = SimpleNamespace(
        cfg=cfg, tracker=MultiObjectTracker(cfg), track_frame='odom',
        publish_tentative=False, marker_pub=None,
        _filter_tracks=lambda tracks, stamp: tracks,
        pub=SimpleNamespace(publish=lambda message: None),
        modes_pub=SimpleNamespace(publish=modes_messages.append))
    node._to_msg = lambda tracks, stamp: ObjectTrackerNode._to_msg(node, tracks, stamp)
    node._to_modes_msg = lambda tracks, stamp: ObjectTrackerNode._to_modes_msg(
        node, tracks, stamp)
    for frame in range(frames):
        msg = DetectedObjectArray()
        msg.header.frame_id = 'odom'
        msg.header.stamp = Time(sec=1 + frame // 10, nanosec=(frame % 10)*100000000)
        for source, label, x, y in (
                (0, 'person', 1. + speed*frame*.1, 0.),
                (2, 'chair', 1., 3.), (1, '', 1., 6.)):
            obj = DetectedObject()
            obj.source, obj.class_name, obj.confidence = source, label, .9
            obj.class_id = 0 if label == 'person' else 56
            obj.position.x, obj.position.y = x, y
            obj.position_covariance = [.0001, 0., 0., .0001]
            msg.objects.append(obj)
        ObjectTrackerNode._process_detections(node, msg, (np.zeros(2), np.eye(2)))

    # Anonymous LiDAR geometry is retained by the tracking contract; the control
    # filter below must still use only the semantically confirmed person.
    assert len(node.tracker.tracks) == 3
    assert {obj.class_name for obj in modes_messages[-1].objects} == {'person', 'chair', ''}
    predictions = []
    predictor = SimpleNamespace(
        _components=MotionPredictorNode._components,
        predictor=ImmMotionPredictor(cfg, horizon=2.5),
        pub=SimpleNamespace(publish=predictions.append))
    MotionPredictorNode._on_tracks(predictor, modes_messages[-1])
    wire = deserialize_message(serialize_message(predictions[-1]),
                               MultiModalObjectPredictionArray)
    assert all(len(obj.modes) == 3 for obj in wire.objects)
    filtered = filter_predictions_for_control(
        wire, ['person'], max_semantic_age=1., max_measurement_age=.5)
    assert len(filtered.objects) == 1
    person = filtered.objects[0]
    assert person.motion_state == expected_motion
    assert [m.model_type for m in person.modes] == (
        [0, 1] if expected_motion == 2 else [2])

    control = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: RosTime.from_msg(filtered.header.stamp)),
        _prediction_message=filtered,
        _last_prediction_monotonic=time.monotonic(),
        prediction_timeout=.5, prediction_fail_safe=True, planning_frame='odom',
        prediction_class_names=['person'], prediction_max_semantic_age=1.,
        prediction_max_measurement_age=.5, prediction_require_confirmed=True,
        _lookup_2d=lambda *args: (0., 0., 0.),
        controller=SimpleNamespace(config=SimpleNamespace(
            horizon_steps=10, dt=.2,
            dynamic_risk=SimpleNamespace(obstacle_radius=.25))))
    aligned = DraMppiNode._aligned_predictions(control)
    assert aligned.means.shape == (1, 2, 10, 2)
    if expected_motion != 2:
        assert np.allclose(aligned.means[0, 0], aligned.means[0, 0, 0])
    else:
        assert aligned.means[0, 0, -1, 0] > aligned.means[0, 0, 0, 0]
    # A robot trajectory through the prediction must cost more than one 5m away.
    robot_positions = np.stack([aligned.means[0, 0], aligned.means[0, 0]+[0., 5.]])
    risk = dynamic_risk_cost(robot_positions, aligned,
                             DynamicRiskConfig(mc_samples=4096, seed=3))
    assert np.all(np.isfinite(risk.cost))
    assert risk.cost[0] > risk.cost[1]
