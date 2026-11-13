"""Independent geometry cases for conservative, whole-component masking."""

from copy import deepcopy
from dataclasses import replace
import math

from dynamic_filter.cluster import segment_scan
from dynamic_filter.core import FilterConfig, TransformUnavailable
import numpy as np
import pytest
from test_core import DYNAMIC, pose, run, scan, snapshot, track

CLUSTER = FilterConfig(mask_method='cluster')


def rays(ranges, angle_min=0., step=.01, **kwargs):
    msg = scan(ranges, angle_increment=step, **kwargs)
    msg.angle_min = angle_min
    msg.angle_max = angle_min + (len(ranges)-1)*step
    return msg


def wall_and_person():
    angles = np.linspace(-.9, .9, 181)
    person = np.abs(angles) < .065
    ranges = 1.25/np.cos(angles)  # A vertical wall only 25cm behind the person.
    ranges[person] = 1.
    return rays(ranges, angle_min=-.9), person


def test_radius_baseline_and_cluster_preserve_distinct_wall():
    msg, person = wall_and_person()
    before = deepcopy(msg)
    radius, _ = run(msg)
    clustered, stats = run(msg, config=CLUSTER)
    assert np.count_nonzero(np.isnan(radius.ranges) & ~person) > 0
    np.testing.assert_array_equal(np.isnan(clustered.ranges), person)
    assert stats['clusters_matched'] == 1 and stats['clusters_oversized'] == 2
    assert msg == before
    clustered.ranges = before.ranges
    assert clustered == before  # Headers, timing, intensities and all metadata preserved.


def test_cluster_does_not_use_filter_radius():
    msg, _ = wall_and_person()
    a, _ = run(msg, config=replace(CLUSTER, filter_radius=.001))
    b, _ = run(msg, config=replace(CLUSTER, filter_radius=100.))
    np.testing.assert_equal(a.ranges, b.ranges)


def test_small_component_has_no_radius_fallback():
    msg = rays([1., 1.])
    assert all(math.isnan(r) for r in run(msg)[0].ranges)
    out, stats = run(msg, config=CLUSTER)
    assert out.ranges == msg.ranges
    assert stats['clusters_small'] == 1 and stats['reason'] == 'no_cluster_match'


def test_two_legs_one_track_and_invalid_beams_unchanged():
    msg = rays([1.]*3 + [math.inf]*34 + [1.]*3, angle_min=-.2)
    out, stats = run(msg, config=CLUSTER)
    assert stats['clusters_matched'] == 2 and stats['used_tracks'] == 1
    assert all(math.isnan(out.ranges[i]) for i in (0, 1, 2, 37, 38, 39))
    assert all(math.isinf(r) for r in out.ranges[3:37])


@pytest.mark.parametrize('other', [track(1.04, track_id=2),
                                   track(1.04, track_id=2, class_name='chair')])
def test_competing_tracks_are_ambiguous_even_when_non_person(other):
    msg = rays([1.]*3, angle_min=-.01)
    out, stats = run(msg, snapshot(track(), other), CLUSTER)
    assert out.ranges == msg.ranges
    assert stats['clusters_ambiguous'] == 1


def test_nearest_non_target_track_prevents_person_stealing_cluster():
    msg = rays([1.]*3, angle_min=-.01)
    tracks = snapshot(track(1.25), track(1., track_id=2, class_name='chair'))
    out, stats = run(msg, tracks, CLUSTER)
    assert out.ranges == msg.ranges
    assert stats['clusters_non_target_or_expired'] == 1


def test_unknown_dynamic_policy_and_stationary_person_policy():
    msg = rays([1.]*3, angle_min=-.01)
    assert all(math.isnan(r) for r in run(msg, config=CLUSTER)[0].ranges)
    obj = track(class_name='', semantic_age=-1., motion_state=DYNAMIC)
    out, _ = run(msg, snapshot(obj), replace(CLUSTER, filter_policy='dynamic_only'))
    assert all(math.isnan(r) for r in out.ranges)
    assert run(msg, snapshot(obj), CLUSTER)[0].ranges == msg.ranges
    assert run(msg, config=replace(CLUSTER, filter_policy='dynamic_only'))[
        0].ranges == msg.ranges


@pytest.mark.parametrize('changes', [{'measurement_age': .25}, {'semantic_age': .95}])
def test_expiry_in_middle_preserves_entire_component(changes):
    msg = rays([1.]*3, angle_min=-.01, increment=.05)
    out, _ = run(msg, snapshot(track(**changes)), CLUSTER)
    assert out.ranges == msg.ranges


def test_expired_tail_does_not_crop_wall_into_person_cluster():
    angles = np.linspace(-.6, .6, 121)
    msg = rays(1./np.cos(angles), angle_min=-.6, increment=.01)
    out, stats = run(msg, snapshot(track(1., -.6)), CLUSTER)
    assert out.ranges == msg.ranges
    assert stats['clusters_oversized'] == 1


def test_extent_guard_rejects_cluster_reaching_far_from_track():
    angles = np.linspace(-.34, .34, 69)
    msg = rays(1./np.cos(angles), angle_min=-.34)
    out, stats = run(msg, snapshot(track(1., .35)), CLUSTER)
    assert out.ranges == msg.ranges
    assert stats['clusters_extent_rejected'] == 1


def test_too_many_candidate_components_are_not_arbitrarily_selected():
    msg = rays([1.]*3 + [math.inf] + [1.]*3 + [math.inf] + [1.]*3, angle_min=-.05)
    out, stats = run(msg, config=CLUSTER)
    np.testing.assert_equal(out.ranges, msg.ranges)
    assert stats['clusters_track_limit_rejected'] == 3


def test_combined_components_must_fit_object_extent():
    msg = rays([1.]*3 + [math.inf]*74 + [1.]*3, angle_min=-.395)
    config = replace(CLUSTER, cluster_max_span=.6)
    out, stats = run(msg, config=config)
    np.testing.assert_equal(out.ranges, msg.ranges)
    assert stats['clusters_group_extent_rejected'] == 2


@pytest.mark.parametrize('step', [2*math.pi/360, -2*math.pi/360])
def test_full_circle_seam_preserves_original_indices(step):
    values = np.full(360, math.inf)
    values[[0, 1, 359]] = 1.
    msg = rays(values, step=step)
    groups = segment_scan(msg, values, np.isfinite(values), CLUSTER)
    assert len(groups) == 1 and set(groups[0]) == {0, 1, 359}
    out, stats = run(msg, config=CLUSTER)
    assert stats['removed_points'] == 3
    assert all(math.isnan(out.ranges[i]) for i in (0, 1, 359))


def test_partial_fov_edges_and_invalid_gaps_do_not_merge():
    values = np.array([1., math.nan, 1.])
    msg = rays(values, step=.01)
    groups = segment_scan(msg, values, np.isfinite(values), CLUSTER)
    assert [g.tolist() for g in groups] == [[0], [2]]
    msg.angle_increment = 0.
    out, stats = run(msg, config=CLUSTER)
    np.testing.assert_equal(out.ranges, msg.ranges)
    assert stats['cluster_invalid_geometry'] == 1


def test_each_sensor_and_beam_uses_own_transform_and_prediction():
    msg = rays([1.]*3, increment=.05)
    times = []

    def tf(frame, ns):
        times.append(ns)
        return pose((ns-10_000_000_000)*1e-9, yaw=math.pi)

    out, _ = run(msg, snapshot(track(-1., vx=1.)), CLUSTER, tf)
    assert all(math.isnan(r) for r in out.ranges)
    assert set(times) == {10_000_000_000, 10_050_000_000, 10_100_000_000}


def test_tf_failure_rolls_back_cluster_scan():
    msg = rays([1.]*3, increment=.05)

    def tf(frame, ns):
        if ns == 10_050_000_000:
            raise TransformUnavailable('interior failure')
        return pose()

    out, stats = run(msg, config=CLUSTER, tf=tf)
    assert out.ranges == msg.ranges and stats['reason'] == 'tf_unavailable'


@pytest.mark.parametrize('changes', [
    {'mask_method': 'unknown'}, {'cluster_min_points': 0}, {'cluster_max_per_track': True},
    {'cluster_base_gap': math.nan}, {'cluster_gap_scale': -1.}, {'cluster_max_gap': .01},
    {'cluster_max_span': 0.}, {'cluster_ambiguity_margin': 0.},
    {'cluster_match_distance': .7}, {'cluster_max_point_distance': math.inf},
])
def test_cluster_configuration_validation(changes):
    with pytest.raises(ValueError):
        replace(CLUSTER, **changes)
