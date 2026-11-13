"""Prediction class-whitelist and control-safety tests."""
import time
import copy
from types import SimpleNamespace

import pytest

from amr_interfaces.msg import (
    GaussianPrediction2D,
    MotionModePrediction,
    MultiModalObjectPrediction,
    MultiModalObjectPredictionArray,
)
from dra_mppi.dra_mppi_node import (
    DraMppiNode,
    PredictionChainUnavailable,
    filter_predictions_by_class,
    filter_predictions_for_control,
)


def _prediction(track_id, class_name):
    predicted = MultiModalObjectPrediction()
    predicted.track_id = track_id
    predicted.class_name = class_name
    return predicted


def _message():
    message = MultiModalObjectPredictionArray()
    message.header.frame_id = 'odom'
    message.objects = [
        _prediction(1, 'person'),
        _prediction(2, 'chair'),
        _prediction(3, 'TV'),
        _prediction(4, ''),
    ]
    return message


def test_person_whitelist_keeps_only_person_without_mutating_source():
    source = _message()

    filtered = filter_predictions_by_class(source, ['person'])

    assert [item.track_id for item in filtered.objects] == [1]
    assert len(source.objects) == 4
    assert filtered.header.frame_id == 'odom'


def test_class_matching_is_case_and_whitespace_insensitive():
    filtered = filter_predictions_by_class(_message(), ['  PeRsOn  ', 'tv'])

    assert [item.track_id for item in filtered.objects] == [1, 3]


def test_empty_or_wildcard_whitelist_accepts_all_classes():
    assert len(filter_predictions_by_class(_message(), []).objects) == 4
    assert len(filter_predictions_by_class(_message(), ['*']).objects) == 4


def _quality_prediction(track_id=1, class_name='person', lifecycle=1,
                        motion=2, measurement_age=0.0, semantic_age=0.0):
    predicted = _prediction(track_id, class_name)
    predicted.lifecycle_state = lifecycle
    predicted.motion_state = motion
    predicted.measurement_age = measurement_age
    predicted.semantic_age = semantic_age
    predicted.time_offsets = [0.0, 0.2, 0.4]
    for model, x_values in ((0, [0.8, 1.0, 1.2]),
                            (1, [0.9, 1.0, 1.1])):
        mode = MotionModePrediction()
        mode.model_type = model
        for x in x_values:
            point = GaussianPrediction2D()
            point.weight = 0.5
            point.mean_x = x
            point.mean_y = 2.0
            point.covariance = [0.01, 0.0, 0.0, 0.01]
            mode.points.append(point)
        predicted.modes.append(mode)
    return predicted


def _filter(message, age_offset=0.0):
    return filter_predictions_for_control(
        message,
        ['person'],
        max_semantic_age=1.0,
        max_measurement_age=0.5,
        age_offset=age_offset,
    )


def test_control_filter_rejects_non_person_lost_and_stale_tracks():
    message = MultiModalObjectPredictionArray()
    message.objects = [
        _quality_prediction(1),
        _quality_prediction(2, class_name='chair'),
        _quality_prediction(3, lifecycle=2),
        _quality_prediction(4, semantic_age=1.1),
        _quality_prediction(5, measurement_age=0.6),
    ]

    filtered = _filter(message)

    assert [item.track_id for item in filtered.objects] == [1]


def test_control_filter_accounts_for_message_age():
    message = MultiModalObjectPredictionArray()
    message.objects = [_quality_prediction(1, semantic_age=0.8)]

    assert len(_filter(message, age_offset=0.1).objects) == 1
    assert len(_filter(message, age_offset=0.3).objects) == 0


def test_recent_lost_person_is_a_fixed_disc_then_expires():
    message = MultiModalObjectPredictionArray()
    lost = _quality_prediction(
        6, lifecycle=2, motion=2, measurement_age=.2, semantic_age=.2)
    message.objects = [lost]

    kept = filter_predictions_for_control(
        message, ['person'], max_semantic_age=1., max_measurement_age=.5,
        allow_recent_lost=True, lost_grace_time=.3)
    assert len(kept.objects) == 1
    assert [mode.model_type for mode in kept.objects[0].modes] == [2]
    assert len({point.mean_x for point in kept.objects[0].modes[0].points}) == 1

    expired = filter_predictions_for_control(
        message, ['person'], max_semantic_age=1., max_measurement_age=.5,
        age_offset=.11, allow_recent_lost=True, lost_grace_time=.3)
    assert expired.objects == []


def test_recent_lost_expires_again_at_control_time():
    message = MultiModalObjectPredictionArray()
    message.header.frame_id = 'odom'
    message.objects = [_quality_prediction(
        6, lifecycle=2, motion=2, measurement_age=.2, semantic_age=.2)]
    filtered = filter_predictions_for_control(
        message, ['person'], max_semantic_age=1., max_measurement_age=.5,
        allow_recent_lost=True, lost_grace_time=.3)
    node = SimpleNamespace(
        _prediction_message=filtered,
        _last_prediction_monotonic=time.monotonic() - .11,
        prediction_timeout=.5, prediction_fail_safe=True,
        planning_frame='odom', prediction_class_names=['person'],
        prediction_max_semantic_age=1., prediction_max_measurement_age=.5,
        prediction_require_confirmed=True, prediction_allow_recent_lost=True,
        prediction_lost_grace_time=.3,
        _lookup_2d=lambda *args: (0., 0., 0.),
        controller=SimpleNamespace(config=SimpleNamespace(
            horizon_steps=2, dt=.1,
            dynamic_risk=SimpleNamespace(obstacle_radius=.25))))

    with pytest.raises(PredictionChainUnavailable):
        DraMppiNode._aligned_predictions(node)


@pytest.mark.parametrize('motion', [0, 1])
def test_nondynamic_person_uses_fixed_current_posterior_safety_disc(motion):
    message = MultiModalObjectPredictionArray()
    original = _quality_prediction(7, motion=motion)
    original.modes[0].points[0].weight = 0.75
    original.modes[1].points[0].weight = 0.25
    original.modes[0].points[0].covariance = [0.01, 0.0, 0.0, 0.02]
    original.modes[1].points[0].covariance = [0.03, 0.0, 0.0, 0.04]
    message.objects = [original]

    filtered = _filter(message)

    assert len(filtered.objects) == 1
    points = [point for mode in filtered.objects[0].modes for point in mode.points]
    assert len({(point.mean_x, point.mean_y) for point in points}) == 1
    assert points[0].mean_x == pytest.approx(0.825)
    assert len({tuple(point.covariance) for point in points}) == 1
    assert points[0].covariance == pytest.approx(
        [0.016875, 0.0, 0.0, 0.025])
    assert original.modes[0].points[2].mean_x == 1.2


@pytest.mark.parametrize('field', ['semantic_age', 'measurement_age'])
@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -1.0])
def test_control_filter_rejects_invalid_observation_age(field, invalid):
    message = MultiModalObjectPredictionArray()
    predicted = _quality_prediction(8)
    setattr(predicted, field, invalid)
    message.objects = [predicted]

    assert len(_filter(message).objects) == 0


def test_control_filter_rejects_nonfinite_age_offset():
    message = MultiModalObjectPredictionArray()
    message.objects = [_quality_prediction(8)]

    assert len(_filter(message, age_offset=float('nan')).objects) == 0


def test_dynamic_person_keeps_its_future_path():
    message = MultiModalObjectPredictionArray()
    message.objects = [_quality_prediction(9, motion=2)]

    filtered = _filter(message)

    cv_x = [point.mean_x for point in filtered.objects[0].modes[0].points]
    assert cv_x == pytest.approx([0.8, 1.0, 1.2])


def _three_mode(motion):
    predicted = _quality_prediction(motion=motion)
    static = copy.deepcopy(predicted.modes[0])
    static.model_type = MotionModePrediction.MODEL_STATIC
    predicted.modes.append(static)
    for mode, weight, x in zip(predicted.modes, [.1, .2, .7], [0., 1., 2.]):
        for k, point in enumerate(mode.points):
            point.weight = weight
            point.mean_x = x if mode.model_type == 2 else x+k*.1
    return predicted


@pytest.mark.parametrize('motion', [0, 1])
def test_three_mode_static_anchor_uses_all_current_posteriors(motion):
    message = MultiModalObjectPredictionArray(objects=[_three_mode(motion)])
    obj = _filter(message).objects[0]
    assert len(obj.modes) == 1
    for point in obj.modes[0].points:
        assert point.mean_x == pytest.approx(1.6)
        assert point.covariance[0] == pytest.approx(.45)
        assert point.weight == 1.
    assert len(message.objects[0].modes) == 3


def test_dynamic_three_mode_conditions_weights_without_averaging_paths():
    message = MultiModalObjectPredictionArray(objects=[_three_mode(2)])
    obj = _filter(message).objects[0]
    assert [m.model_type for m in obj.modes] == [0, 1]
    assert obj.modes[0].points[-1].mean_x == pytest.approx(.2)
    assert obj.modes[1].points[-1].mean_x == pytest.approx(1.2)
    for cv, ct in zip(obj.modes[0].points, obj.modes[1].points):
        assert cv.weight == pytest.approx(1/3)
        assert ct.weight == pytest.approx(2/3)


def test_dynamic_tiny_moving_mass_falls_back_to_current_disc():
    obj = _three_mode(2)
    for mode in obj.modes:
        for point in mode.points:
            point.weight = 1. if mode.model_type == 2 else 0.
    message = MultiModalObjectPredictionArray(objects=[obj])
    result = _filter(message).objects[0]
    assert len(result.modes) == 1
    assert all(p.mean_x == 2. and p.weight == 1. for p in result.modes[0].points)


@pytest.mark.parametrize('motion', [0, 1, 2])
def test_filtered_three_mode_message_reaches_controller_adapter(motion):
    message = MultiModalObjectPredictionArray(objects=[_three_mode(motion)])
    node = SimpleNamespace(
        _prediction_message=_filter(message),
        _last_prediction_monotonic=time.monotonic(),
        prediction_timeout=.5, prediction_fail_safe=True, planning_frame='odom',
        prediction_class_names=['person'], prediction_max_semantic_age=1.,
        prediction_max_measurement_age=.5, prediction_require_confirmed=True,
        _lookup_2d=lambda *args: (0., 0., 0.),
        controller=SimpleNamespace(config=SimpleNamespace(
            horizon_steps=2, dt=.1,
            dynamic_risk=SimpleNamespace(obstacle_radius=.25))))
    result = DraMppiNode._aligned_predictions(node)
    assert result.means.shape == (1, 2, 2, 2)
    if motion != 2:
        assert result.means[0, 0, :, 0] == pytest.approx([1.6, 1.6])
        assert result.mode_weights[0, 0] == pytest.approx([1., 1.])
    else:
        assert result.mode_weights[0, 0] == pytest.approx([1/3, 1/3])


def test_corrupt_person_geometry_is_not_silently_an_empty_scene():
    obj = _three_mode(2)
    obj.modes[0].points[1].mean_x = float('nan')
    message = MultiModalObjectPredictionArray(objects=[obj])
    with pytest.raises(ValueError):
        _filter(message)


def test_missing_prediction_message_is_not_treated_as_empty_scene():
    node = SimpleNamespace(
        _prediction_message=None,
        prediction_fail_safe=True,
    )

    with pytest.raises(PredictionChainUnavailable):
        DraMppiNode._aligned_predictions(node)


def test_stale_prediction_message_is_not_treated_as_empty_scene():
    node = SimpleNamespace(
        _prediction_message=MultiModalObjectPredictionArray(),
        _last_prediction_monotonic=time.monotonic() - 1.0,
        prediction_timeout=0.5,
        prediction_fail_safe=True,
    )

    with pytest.raises(PredictionChainUnavailable):
        DraMppiNode._aligned_predictions(node)


def test_fresh_empty_prediction_message_still_means_no_person():
    message = MultiModalObjectPredictionArray()
    message.header.frame_id = 'odom'
    node = SimpleNamespace(
        _prediction_message=message,
        _last_prediction_monotonic=time.monotonic(),
        prediction_timeout=0.5,
        prediction_fail_safe=True,
        planning_frame='odom',
        _lookup_2d=lambda _target, _source: (0.0, 0.0, 0.0),
        controller=SimpleNamespace(
            config=SimpleNamespace(horizon_steps=10, dt=0.1),
        ),
    )

    assert DraMppiNode._aligned_predictions(node) is None
