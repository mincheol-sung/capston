"""Exact sensor/epoch membership, receipt deadlines and conservative rejection."""

from copy import deepcopy
from dataclasses import replace
import math
from types import SimpleNamespace as NS

from dynamic_filter.core import FilterConfig
from dynamic_filter.observations import mask_observations, ObservationHistory
import numpy as np
import pytest
from test_core import CONFIRMED, DYNAMIC, header, scan

CONFIG = FilterConfig(mask_method='tracked_cluster')


def link(indices=(0, 2), sensor='/scan_front', frame='laser', seconds=10., **changes):
    observation = NS(sensor_id=sensor, header=header(seconds, frame),
                     observation_id='original cluster', beam_indices=list(indices),
                     beam_time_offsets=[])
    result = NS(track_id=7, lifecycle_state=CONFIRMED, motion_state=0,
                class_name='person', semantic_age=0., observation=observation)
    result.__dict__.update(changes)
    return result


def message(*links, seconds=10.1):
    return NS(header=header(seconds), links=list(links))


def run(records, msg=None, config=CONFIG):
    return mask_observations(msg if msg is not None else scan([1., 2., 3.]),
                             records, '/scan_front', config, confirmed=CONFIRMED, dynamic=DYNAMIC)


def test_membership_only_no_radius_or_tf_and_metadata_unchanged():
    msg = scan([1., 5., 9.])
    before = deepcopy(msg)
    out, stats = run([(10_100_000_000, link())], msg)
    assert np.isnan(out.ranges).tolist() == [True, False, True]
    assert stats['removed_points'] == 2 and stats['used_tracks'] == 1
    out.ranges = msg.ranges
    assert out == before and msg == before


@pytest.mark.parametrize('changes', [
    {'sensor': '/scan_rear'}, {'frame': 'rear_laser'}, {'seconds': 10.000000001},
])
def test_no_sensor_frame_or_timestamp_fallback(changes):
    out, stats = run([(10_100_000_000, link(**changes))])
    assert out.ranges == [1., 2., 3.]
    assert stats['observation_identity_rejected'] == 1


def test_arrival_deadline_including_delayed_timer_and_multiple_events():
    history = ObservationHistory(CONFIG)
    first, second, late = link([0]), link([1]), link([2])
    history.add(message(first), received=1.05)
    history.add(message(second), received=1.2)
    history.add(message(late), received=1.31)
    records = history.select(scan([1., 2., 3.]), '/scan_front', deadline=1.3)
    out, stats = run(records)
    assert np.isnan(out.ranges).tolist() == [True, True, False]
    assert stats['observation_links'] == 2
    assert not history.select(scan(seconds=10.1), '/scan_front', deadline=2.)


def test_history_bounds_and_oversized_payloads():
    history = ObservationHistory(replace(CONFIG, max_observation_links=2))
    assert not history.add(message(link(), link(), link()), 0.)
    history.add(message(link([0])), 0.)
    history.add(message(link([1])), .1)
    history.add(message(link([2])), .2)
    assert len(history.entries) == 2
    history.prune(3.)
    assert not history.entries
    history.add(message(link()), 3.)
    history.clear()
    assert not history.entries
    huge = link(list(range(CONFIG.max_scan_beams + 1)))
    assert not history.add(message(huge), 4.)


@pytest.mark.parametrize('indices', [[], [0, 0], [0, 3], [-1, 1]])
def test_bad_membership_rejects_entire_link(indices):
    out, stats = run([(10_100_000_000, link(indices))])
    assert out.ranges == [1., 2., 3.]
    assert stats['observation_membership_rejected'] == 1


@pytest.mark.parametrize('changes,epoch', [
    ({'lifecycle_state': 0}, 10_100_000_000),
    ({'class_name': 'chair'}, 10_100_000_000),
    ({'semantic_age': -1.}, 10_100_000_000),
    ({'semantic_age': 1.01}, 10_100_000_000),
    ({'semantic_age': math.nan}, 10_100_000_000),
    ({}, 9_999_999_999), ({}, 10_750_000_001),
])
def test_policy_and_update_epoch_are_checked(changes, epoch):
    assert run([(epoch, link(**changes))])[0].ranges == [1., 2., 3.]


def test_unknown_moving_track_and_stopped_person_policies():
    obj = link(class_name='', semantic_age=-1., motion_state=DYNAMIC)
    assert run([(10_100_000_000, obj)])[0].ranges == [1., 2., 3.]
    config = replace(CONFIG, filter_policy='dynamic_only')
    out, _ = run([(10_100_000_000, obj)], config=config)
    assert np.isnan(out.ranges).tolist() == [True, False, True]
    assert run([(10_100_000_000, link())], config=config)[0].ranges == [1., 2., 3.]


def test_duplicate_links_idempotent_and_conflicting_ownership_vetoes_component():
    a = link()
    out, stats = run([(10_100_000_000, a)]*2)
    assert stats['removed_points'] == 2
    b = link([2], track_id=8, class_name='chair')
    out, stats = run([(10_100_000_000, a), (10_100_000_000, b)])
    assert out.ranges == [1., 2., 3.]
    assert stats['observation_conflicts'] == 1


def test_inconsistent_same_track_decision_is_conservative():
    out, _ = run([(10_100_000_000, link()),
                  (10_150_000_000, link([2], class_name='chair'))])
    assert out.ranges == [1., 2., 3.]


def test_known_offsets_are_bounded_and_invalid_original_ranges_are_preserved():
    msg = scan([math.inf, 0., 1., math.nan, 11.], increment=.01)
    obj = link([0, 1, 2, 3, 4])
    obj.observation.beam_time_offsets = [0., .01, .02, .03, .04]
    out, stats = run([(10_100_000_000, obj)], msg)
    np.testing.assert_equal(out.ranges, [math.inf, 0., math.nan, math.nan, 11.])
    assert stats['removed_points'] == 1
    obj.observation.beam_time_offsets[2] = .05
    out, stats = run([(10_100_000_000, obj)], msg)
    np.testing.assert_equal(out.ranges, msg.ranges)
    assert stats['observation_timing_rejected'] == 1


def test_verified_reversed_beam_times_are_supported_without_reordering_ranges():
    msg = scan([1., 2., 3.], increment=.01)
    obj = link([0, 2])
    obj.observation.timing_status = 1
    obj.observation.beam_time_offsets = [.02, 0.]
    out, _ = run([(10_100_000_000, obj)], msg)
    assert np.isnan(out.ranges).tolist() == [True, False, True]
    obj.observation.beam_time_offsets = []
    assert run([(10_100_000_000, obj)], msg)[0].ranges == msg.ranges


def test_missing_link_never_falls_back_to_geometric_mask():
    assert run([])[1]['reason'] == 'observations_missing'


@pytest.mark.parametrize('changes', [
    {'observation_wait_timeout': 0.}, {'max_observation_lag': math.nan},
    {'max_observation_links': 0}, {'observation_wait_timeout': 3.},
])
def test_new_configuration_validation(changes):
    with pytest.raises(ValueError):
        replace(CONFIG, **changes)
