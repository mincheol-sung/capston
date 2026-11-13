"""RViz prediction marker conversion tests."""
from amr_interfaces.msg import (
    GaussianPrediction2D,
    MotionModePrediction,
    MultiModalObjectPrediction,
    MultiModalObjectPredictionArray,
)
from motion_predictor.prediction_markers_node import (
    _ellipse_segments,
    build_prediction_markers,
)
import numpy as np
from visualization_msgs.msg import Marker


def _point(weight, x, y, variance):
    prediction = GaussianPrediction2D()
    prediction.weight = weight
    prediction.mean_x = x
    prediction.mean_y = y
    prediction.covariance = [variance, 0.0, 0.0, variance]
    return prediction


def _message():
    message = MultiModalObjectPredictionArray()
    message.header.frame_id = 'odom'
    predicted = MultiModalObjectPrediction()
    predicted.track_id = 7
    predicted.class_id = 0
    predicted.class_name = 'person'
    predicted.motion_state = 2
    predicted.time_offsets = [0.2, 0.4]

    cv = MotionModePrediction()
    cv.model_type = MotionModePrediction.MODEL_CV
    cv.points = [_point(0.75, 1.0, 2.0, 0.01),
                 _point(0.75, 1.2, 2.0, 0.02)]
    ct = MotionModePrediction()
    ct.model_type = MotionModePrediction.MODEL_CT
    ct.points = [_point(0.25, 1.0, 2.0, 0.01),
                 _point(0.25, 1.15, 2.1, 0.03)]
    predicted.modes = [cv, ct]
    message.objects = [predicted]
    return message


def test_builder_emits_paths_means_covariances_and_label():
    markers = build_prediction_markers(_message(), ellipse_segments=12)

    assert markers.markers[0].action == Marker.DELETEALL
    assert len(markers.markers) == 8
    by_namespace = {marker.ns: marker for marker in markers.markers[1:]}
    assert len(by_namespace['prediction_cv_path'].points) == 2
    assert len(by_namespace['prediction_ct_means'].points) == 2
    assert len(by_namespace['prediction_cv_covariance'].points) == 48
    assert 'CV 75%' in by_namespace['prediction_labels'].text
    assert 'CT 25%' in by_namespace['prediction_labels'].text
    assert all(marker.header.frame_id == 'odom'
               for marker in markers.markers)


def test_covariance_ellipse_has_expected_axis_extents():
    vertices = _ellipse_segments(
        [10.0, 20.0], [4.0, 0.0, 0.0, 1.0],
        scale=1.0, segment_count=24, z=0.1)
    xy = np.array([[point.x, point.y] for point in vertices])

    assert np.isclose(np.min(xy[:, 0]), 8.0)
    assert np.isclose(np.max(xy[:, 0]), 12.0)
    assert np.isclose(np.min(xy[:, 1]), 19.0)
    assert np.isclose(np.max(xy[:, 1]), 21.0)


def test_empty_prediction_clears_stale_markers():
    message = MultiModalObjectPredictionArray()
    message.header.frame_id = 'odom'

    markers = build_prediction_markers(message)

    assert len(markers.markers) == 1
    assert markers.markers[0].action == Marker.DELETEALL
