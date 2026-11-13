"""ROS observation messages, strict receipt deadlines, FIFO and clock reset."""

from copy import deepcopy
from types import SimpleNamespace

from amr_interfaces.msg import TrackedObservation, TrackedObservationArray
import dynamic_filter.dynamic_filter_node as node_module
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from test_node import inputs


@pytest.fixture
def fixture(monkeypatch, request):
    clock = SimpleNamespace(value=1.)
    monkeypatch.setattr(node_module, 'time', SimpleNamespace(monotonic=lambda: clock.value))
    rclpy.init()
    node = node_module.DynamicFilterNode(parameter_overrides=[
        Parameter('mask_method', value='tracked_cluster'),
        Parameter('observation_sensor_id', value=getattr(request, 'param', '/scan_front'))])
    outputs = []
    monkeypatch.setattr(node.pub, 'publish', outputs.append)
    # Membership masking must never depend on a nearest-position/TF fallback.
    monkeypatch.setattr(node, '_transform_at', lambda *_: pytest.fail('unexpected TF lookup'))
    yield node, outputs, clock
    node.destroy_node()
    rclpy.shutdown()


def associations(scan, indices=(0, 2), track_id=7):
    msg = TrackedObservationArray()
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 150_000_000
    link = TrackedObservation()
    link.track_id = track_id
    link.lifecycle_state = 1
    link.class_name = 'person'
    obs = link.observation
    obs.header = deepcopy(scan.header)
    obs.sensor_id = '/scan_front'
    obs.observation_id = f'cluster-{track_id}'
    obs.beam_indices = list(indices)
    msg.links = [link]
    return msg


def test_wait_for_actual_link_and_preserve_metadata_without_imm_or_tf(fixture):
    node, outputs, clock = fixture
    scan, _ = inputs()
    original = serialize_message(scan)
    node._on_scan(scan)
    node._drain()
    assert not outputs
    clock.value = 1.1
    node._on_observations(associations(scan))
    node._drain()
    assert not outputs  # More components/conflicts may still arrive.
    clock.value = 1.301
    node._drain()
    assert np.isnan(outputs[0].ranges).tolist() == [True, False, True]
    restored = deepcopy(outputs[0])
    restored.ranges = scan.ranges
    assert serialize_message(restored) == original
    assert serialize_message(scan) == original


def test_late_event_cannot_mask_even_if_timer_is_delayed(fixture):
    node, outputs, clock = fixture
    scan, _ = inputs()
    node._on_scan(scan)
    clock.value = 1.31
    node._on_observations(associations(scan))
    node._drain()
    assert serialize_message(outputs[0]) == serialize_message(scan)
    assert node.latest['reason'] == 'observations_missing'


def test_multiple_arrivals_and_conflicts_before_deadline(fixture):
    node, outputs, clock = fixture
    scan, _ = inputs()
    node._on_scan(scan)
    clock.value = 1.1
    node._on_observations(associations(scan, [0, 1], 7))
    clock.value = 1.2
    node._on_observations(associations(scan, [1], 8))
    clock.value = 1.301
    node._drain()
    assert serialize_message(outputs[0]) == serialize_message(scan)
    assert node.latest['observation_conflicts'] == 2


def test_next_scan_does_not_reuse_previous_indices(fixture):
    node, outputs, clock = fixture
    scan, _ = inputs()
    node._on_observations(associations(scan))
    later = deepcopy(scan)
    later.header.stamp.nanosec = 100_000_000
    node._on_scan(later)
    clock.value = 1.4
    node._drain()
    assert serialize_message(outputs[0]) == serialize_message(later)


@pytest.mark.parametrize('fixture', ['/scan_front', '/scan_rear'], indirect=True)
def test_same_indices_and_capture_header_are_isolated_by_sensor(fixture):
    node, outputs, clock = fixture
    scan, _ = inputs()
    scan.ranges = [1., 2., 3.]
    original = serialize_message(scan)
    # Deliberately identical frame, stamp and indices: only sensor ID differs.
    front = associations(scan, [0, 2], 7)
    rear = associations(scan, [0, 2], 8)
    rear.links[0].observation.sensor_id = '/scan_rear'
    rear.links[0].class_name = 'chair'
    stale = associations(scan, [1], 9)
    stale.links[0].observation.sensor_id = node.observation_sensor_id
    stale.links[0].observation.header.stamp.nanosec = 1
    wrong_frame = associations(scan, [1], 10)
    wrong_frame.links[0].observation.sensor_id = node.observation_sensor_id
    wrong_frame.links[0].observation.header.frame_id = 'another_laser'
    node._on_scan(scan)
    for event in (front, rear, stale, wrong_frame):
        node._on_observations(event)
    clock.value = 1.301
    node._drain()
    assert len(outputs) == 1
    assert node.latest['observation_links'] == 1
    expected = ([np.nan, 2., np.nan]
                if node.observation_sensor_id == '/scan_front' else scan.ranges)
    np.testing.assert_array_equal(outputs[0].ranges, expected)
    assert len(outputs[0].ranges) == len(scan.ranges)
    restored = deepcopy(outputs[0])
    restored.ranges = scan.ranges
    assert serialize_message(restored) == original
    assert serialize_message(scan) == original


def test_overflow_and_reset_preserve_existing_fail_open_contract(fixture):
    node, outputs, _ = fixture
    scan, _ = inputs()
    node._on_observations(associations(scan))
    for _ in range(node.config.pending_queue_size+1):
        node._on_scan(scan)
    assert len(outputs) == 1
    assert node.latest['reason'] == 'queue_overflow'
    assert serialize_message(outputs[0]) == serialize_message(scan)
    node._on_clock_jump(None)
    assert not node.pending and not node.observations.entries
