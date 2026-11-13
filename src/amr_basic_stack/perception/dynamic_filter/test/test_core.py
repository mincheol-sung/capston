"""Geometry and causal state selection tests requiring no ROS installation."""

from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace as NS

from dynamic_filter.core import (FilterConfig, mask_scan, rigid_matrix, stamp_ns,
                                 TrackHistory, TransformUnavailable)
import numpy as np
import pytest

# Deliberately arbitrary injected values: the core does not hardcode ROS enums.
CONFIRMED, DYNAMIC = 17, 23


def header(seconds=10., frame='odom'):
    ns = round(seconds * 1e9)
    return NS(stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9), frame_id=frame)


def track(x=1., y=0., vx=0., vy=0., **kwargs):
    result = NS(track_id=1, lifecycle_state=CONFIRMED, motion_state=0,
                class_name='person', class_id=999, measurement_age=0., semantic_age=0.,
                radius=100., modes=[NS(probability=1., state=[x, y, vx, vy, 0.])])
    result.__dict__.update(kwargs)
    return result


def snapshot(*objects, seconds=10., frame='odom'):
    return NS(header=header(seconds, frame), objects=list(objects))


def scan(ranges=(1.,), seconds=10., increment=0., angle_increment=0.):
    return NS(header=header(seconds, 'laser'), ranges=list(ranges),
              angle_min=0., angle_max=1., angle_increment=angle_increment,
              time_increment=increment, scan_time=.1, range_min=.1, range_max=10.,
              intensities=[float(i + 1) for i in range(len(ranges))])


def pose(x=0., y=0., yaw=0.):
    return rigid_matrix((x, y, 0.), (0., 0., math.sin(yaw/2), math.cos(yaw/2)))


def run(msg=None, tracks=None, config=None, tf=None):
    # These synthetic fixtures explicitly use chronological (or instantaneous)
    # measurements, unlike the bundled angle-compensated physical driver.
    config = replace(config or FilterConfig(), scan_timing_mode='ros')
    return mask_scan(msg if msg is not None else scan(),
                     tracks if tracks is not None else snapshot(track()),
                     config, tf or (lambda *_: pose()),
                     confirmed=CONFIRMED, dynamic=DYNAMIC)


def test_strict_radius_boundary_and_original_metadata():
    msg = scan([.5, .75, 1., 1.249, 1.25, 1.5])
    original = deepcopy(msg)
    out, stats = run(msg, config=FilterConfig(filter_radius=.25))
    assert np.isnan(out.ranges).tolist() == [False, False, True, True, False, False]
    assert stats['removed_points'] == 2
    assert msg == original
    assert {k: v for k, v in vars(out).items() if k != 'ranges'} == {
        k: v for k, v in vars(original).items() if k != 'ranges'}
    assert out.header is not msg.header


def test_multiple_tracks_union_not_radius_field():
    out, stats = run(scan([1., 2., 3.]), snapshot(track(), track(3.)))
    assert np.isnan(out.ranges).tolist() == [True, False, True]
    assert stats['used_tracks'] == 2


@pytest.mark.parametrize('transform, centre', [
    (pose(.5, .2, 0.), (1.5, .2)),
    (pose(-.4, .3, math.pi), (-1.4, .3)),
    (pose(.2, -.4, math.pi/2), (.2, .6)),
])
def test_front_rear_mount_translation_rotation(transform, centre):
    out, _ = run(tracks=snapshot(track(*centre)), tf=lambda *_: transform)
    assert math.isnan(out.ranges[0])


def test_beam_time_robot_motion_and_cv_extrapolation():
    # Track at x=1 at t=10, moving 1m/s. Robot also translates 1m/s.
    msg = scan([1., 1., 1.], seconds=10.1, increment=.05)
    requested = []

    def tf(frame, ns):
        assert frame == 'laser'
        requested.append(ns)
        return pose((ns - 10_000_000_000) * 1e-9)

    out, stats = run(msg, snapshot(track(vx=1.)),
                     FilterConfig(filter_radius=.01), tf)
    assert all(math.isnan(r) for r in out.ranges)
    assert set(requested) == {10_100_000_000, 10_150_000_000, 10_200_000_000}
    assert stats['used_tracks'] == 1


def test_weighted_imm_mean_is_short_cv_not_mode_trajectory():
    obj = track(modes=[NS(probability=.25, state=[0., 0., 0., 0., 20.]),
                       NS(probability=.75, state=[1., 0., 2., 0., -20.])])
    out, _ = run(scan([.9], seconds=10.1), snapshot(obj),
                 FilterConfig(filter_radius=.01))
    assert math.isnan(out.ranges[0])


def test_stationary_person_and_dynamic_unknown_class_policy():
    assert math.isnan(run()[0].ranges[0])
    dynamic_config = FilterConfig(filter_policy='dynamic_only')
    assert run(config=dynamic_config)[0].ranges == [1.]
    unknown = snapshot(track(class_name='', class_id=-1, semantic_age=-1., motion_state=DYNAMIC))
    assert math.isnan(run(tracks=unknown, config=dynamic_config)[0].ranges[0])
    assert run(tracks=unknown)[0].ranges == [1.]


@pytest.mark.parametrize('state', [0, 1, 2, 3])
def test_all_nonconfirmed_states_excluded(state):
    # None equals our injected confirmed constant, regardless of numeric ordering.
    out, stats = run(tracks=snapshot(track(lifecycle_state=state)))
    assert out.ranges == [1.]
    assert stats['reason'] == 'no_eligible_tracks'


@pytest.mark.parametrize('changes', [
    {'measurement_age': .25}, {'semantic_age': .95},
])
def test_actual_age_includes_beam_offset(changes):
    out, stats = run(scan([1., 1.], increment=.1), snapshot(track(**changes)))
    assert math.isnan(out.ranges[0])
    assert out.ranges[1] == 1.
    assert stats['removed_points'] == 1


def test_extrapolation_cap_no_clamping():
    out, stats = run(scan([1., 1.], seconds=10.2, increment=.2))
    assert math.isnan(out.ranges[0]) and out.ranges[1] == 1.
    assert stats['extrapolation_limit_beams'] == 1
    out, stats = run(scan(seconds=11.))
    assert out.ranges == [1.] and stats['reason'] == 'tracks_expired'


@pytest.mark.parametrize('obj', [
    track(semantic_age=-1.), track(measurement_age=math.nan),
    track(modes=[]), track(modes=[NS(probability=-1., state=[1., 0., 0., 0., 0.])]),
    track(modes=[NS(probability=1., state=[math.nan, 0., 0., 0., 0.])]),
])
def test_invalid_tracks_do_not_mask(obj):
    assert run(tracks=snapshot(obj))[0].ranges == [1.]


def test_history_causal_empty_reordered_and_bounded():
    history = TrackHistory(FilterConfig(history_size=2))
    assert history.select(10**10)[1] == 'tracks_missing'
    first = snapshot(track(), seconds=10.)
    clear = snapshot(seconds=10.1)
    future = snapshot(track(9.), seconds=10.2)
    history.add(clear)
    history.add(first)  # Transport reordering is not a clock reset.
    assert history.select(10_050_000_000)[0] is first
    assert history.select(9_000_000_000)[1] == 'no_causal_tracks'
    history.add(future)
    selected, _ = history.select(10_150_000_000)
    assert selected is clear
    assert run(scan(seconds=10.15), selected)[1]['reason'] == 'empty_tracks'
    assert history.select(10_000_000_000)[0] is None  # size limit
    history.add(snapshot(seconds=20.))
    assert history.select(10_200_000_000)[0] is None  # duration limit
    history.clear()
    assert history.select(20_000_000_000)[1] == 'tracks_missing'


def test_future_state_and_non_odom_frame_pass_through():
    assert run(tracks=snapshot(track(), seconds=11.))[1]['reason'] == 'no_causal_tracks'
    assert run(tracks=snapshot(track(), frame='map'))[1]['reason'] == 'unsupported_track_frame'


def test_tf_failure_rolls_back_entire_scan():
    def tf(frame, ns):
        if ns == 10_050_000_000:
            raise TransformUnavailable('missing interior transform')
        return pose()
    out, stats = run(scan([1., 1., 1.], increment=.05), tf=tf)
    assert out.ranges == [1., 1., 1.]
    assert stats['reason'] == 'tf_unavailable' and stats['removed_points'] == 0


def test_nan_inf_out_of_range_empty_and_all_masked():
    out, stats = run(scan([math.nan, math.inf, -math.inf, 0., 11., 1.]))
    np.testing.assert_equal(out.ranges, [math.nan, math.inf, -math.inf, 0., 11., math.nan])
    assert stats['valid_points'] == 1
    assert run(scan([]))[1]['reason'] == 'empty_scan'
    assert all(math.isnan(r) for r in run(scan([1., 1.]))[0].ranges)


def test_zero_increment_never_invents_time():
    times = []
    out, stats = run(scan([1., 1., 1.]), tf=lambda _, ns: times.append(ns) or pose())
    assert set(times) == {10**10}
    assert stats['zero_time_increment'] == 1


@pytest.mark.parametrize('method', ['radius', 'cluster'])
def test_unknown_driver_timing_never_removes_points(method):
    msg = scan([1., 1., 1.], increment=.05)
    out, stats = mask_scan(
        msg, snapshot(track()), FilterConfig(mask_method=method),
        lambda *_: pytest.fail('unknown timing must not query invented beam epochs'),
        confirmed=CONFIRMED, dynamic=DYNAMIC)
    assert out == msg and out is not msg
    assert stats['reason'] == 'scan_timing_unknown' and stats['removed_points'] == 0


def test_verified_reversed_scan_uses_original_index_times_for_ego_motion():
    # Static world target x=2; robot x=0, .05, .1 at t=10, 10.05, 10.1.
    # Chronologically reversed ranges must stay in their original array order.
    msg = scan([1.9, 1.95, 2.], increment=.05)
    requested = []

    def tf(_, ns):
        requested.append(ns)
        return pose((ns-10_000_000_000)*1e-9)

    out, stats = mask_scan(
        msg, snapshot(track(2.)), FilterConfig(scan_timing_mode='reversed', filter_radius=.001),
        tf, confirmed=CONFIRMED, dynamic=DYNAMIC)
    assert all(math.isnan(r) for r in out.ranges) and stats['removed_points'] == 3
    assert set(requested) == {10_000_000_000, 10_050_000_000, 10_100_000_000}
    assert msg.ranges == [1.9, 1.95, 2.]


@pytest.mark.parametrize('values', [
    {'filter_radius': 0.}, {'max_extrapolation': -1.}, {'filter_radius': math.inf},
    {'filter_policy': 'unknown'}, {'pending_queue_size': 0}, {'history_size': 1.2},
    {'scan_timing_mode': 'guessed'},
    {'max_tracks': True}, {'max_scan_beams': 100001},
    {'history_duration': .1}, {'tf_cache_duration': 1.}, {'retry_period': 1.},
])
def test_invalid_configuration(values):
    with pytest.raises(ValueError):
        replace(FilterConfig(), **values)


def test_work_limits_and_negative_increment():
    assert run(scan([1., 1.]), config=FilterConfig(max_scan_beams=1))[1]['reason'] == 'scan_limit'
    assert run(tracks=snapshot(track(), track()), config=FilterConfig(max_tracks=1))[
        1]['reason'] == 'track_limit'
    assert run(scan(increment=-.1))[1]['reason'] == 'invalid_scan_metadata'
    assert stamp_ns(header(1.000000001).stamp) == 1_000_000_001
