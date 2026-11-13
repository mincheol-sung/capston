"""ROS message, TF, bounded wait and simulated-time reset integration tests."""

from copy import deepcopy
import math
import time

from amr_interfaces.msg import ImmModeState, ImmTrackedObject, ImmTrackedObjectArray
from dynamic_filter.core import TransformUnavailable
from dynamic_filter.dynamic_filter_node import DynamicFilterNode
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


@pytest.fixture
def node(monkeypatch):
    rclpy.init()
    instance = DynamicFilterNode(parameter_overrides=[
        Parameter('scan_timing_mode', value='ros')])
    outputs = []
    monkeypatch.setattr(instance.pub, 'publish', lambda msg: outputs.append(msg))
    yield instance, outputs
    instance.destroy_node()
    rclpy.shutdown()


def inputs():
    msg = LaserScan()
    msg.header.stamp.sec = 10
    msg.header.frame_id = 'front_laser'
    msg.angle_min, msg.angle_max, msg.angle_increment = -.1, .1, .1
    msg.time_increment, msg.scan_time = .05, .1
    msg.range_min, msg.range_max = .1, 10.
    msg.ranges = [1., 1., 1.]
    msg.intensities = [4., 5., 6.]
    tracks = ImmTrackedObjectArray()
    tracks.header.stamp.sec = 10
    tracks.header.frame_id = 'odom'
    obj = ImmTrackedObject()
    obj.lifecycle_state = obj.LIFECYCLE_CONFIRMED
    obj.class_name = 'person'
    mode = ImmModeState()
    mode.probability = 1.
    mode.state = [1., 0., 0., 0., 0.]
    obj.modes = [mode]
    tracks.objects = [obj]
    return msg, tracks


def set_tf(node, seconds, x=0., frame='front_laser'):
    msg = TransformStamped()
    msg.header.stamp = Time(seconds=seconds).to_msg()
    msg.header.frame_id, msg.child_frame_id = 'odom', frame
    msg.transform.translation.x = x
    msg.transform.rotation.w = 1.
    node.tf_buffer.set_transform(msg, 'test')


def expire(node):
    msg, _ = node.pending[0]
    node.pending[0] = (msg, time.monotonic() - node.config.tf_wait_timeout - 1.)


def test_real_ros_message_tf_and_metadata_preservation(node):
    instance, outputs = node
    scan, tracks = inputs()
    before = serialize_message(scan)
    instance._on_tracks(tracks)
    set_tf(instance, 10.)
    set_tf(instance, 10.2)
    instance._on_scan(scan)
    instance._drain()
    assert len(outputs) == 1
    assert all(math.isnan(r) for r in outputs[0].ranges)
    restored = deepcopy(outputs[0])
    restored.ranges = scan.ranges
    assert serialize_message(restored) == before
    assert serialize_message(scan) == before
    assert instance.latest['removed_points'] == 3


def test_nonblocking_tf_wait_then_original_passthrough(node):
    instance, outputs = node
    scan, tracks = inputs()
    instance._on_tracks(tracks)
    instance._on_scan(scan)
    start = time.monotonic()
    instance._drain()
    assert time.monotonic() - start < .1
    assert not outputs and len(instance.pending) == 1
    expire(instance)
    instance._drain()
    assert serialize_message(outputs[0]) == serialize_message(scan)
    assert instance.latest['reason'] == 'tf_unavailable'


def test_delayed_tf_can_recover_before_deadline(node):
    instance, outputs = node
    scan, tracks = inputs()
    instance._on_tracks(tracks)
    instance._on_scan(scan)
    instance._drain()
    assert not outputs
    set_tf(instance, 10.)
    set_tf(instance, 10.2)
    instance._drain()
    assert all(math.isnan(r) for r in outputs[0].ranges)


def test_no_tracks_and_queue_overflow_are_bounded(node):
    instance, outputs = node
    scan, _ = inputs()
    for _ in range(instance.config.pending_queue_size + 2):
        instance._on_scan(scan)
    assert len(outputs) == 2 and len(instance.pending) == instance.config.pending_queue_size
    assert instance.latest['reason'] == 'queue_overflow'
    expire(instance)
    instance._drain()
    assert instance.latest['reason'] == 'tracks_missing'


def test_clock_rewind_clears_history_pending_and_tf(node):
    instance, _ = node
    instance.set_parameters([Parameter('use_sim_time', value=True)])
    clock = instance.get_clock()
    clock.set_ros_time_override(Time(seconds=20.))
    scan, tracks = inputs()
    instance._on_tracks(tracks)
    instance._on_scan(scan)
    set_tf(instance, 10.)
    clock.set_ros_time_override(Time(seconds=5.))
    assert not instance.pending
    assert instance.history.select(30_000_000_000)[1] == 'tracks_missing'
    assert instance.latest['reason'] == 'clock_reset'
    with pytest.raises(TransformUnavailable):
        instance._transform_at('front_laser', 10_000_000_000)


def test_zero_epoch_never_uses_latest_tf(node):
    instance, _ = node
    set_tf(instance, 10.)
    with pytest.raises(TransformUnavailable):
        instance._transform_at('front_laser', 0)


def test_diagnostics_show_reason_and_counts(node, monkeypatch):
    instance, _ = node
    published = []
    monkeypatch.setattr(instance.diagnostics, 'publish', published.append)
    scan, _ = inputs()
    instance._on_scan(scan)
    expire(instance)
    instance._drain()
    instance._diagnose()
    status = published[-1].status[0]
    values = {item.key: item.value for item in status.values}
    assert status.message == 'tracks_missing'
    assert values['valid_points'] == '3'
    assert values['total_reason_tracks_missing'] == '1'


def test_parameters_cannot_silently_change_validated_config(node):
    instance, _ = node
    result = instance.set_parameters([Parameter('filter_radius', value=-1.)])
    assert not result[0].successful


def test_cluster_parameter_override_real_messages_and_diagnostics(monkeypatch):
    rclpy.init()
    instance = DynamicFilterNode(parameter_overrides=[
        Parameter('mask_method', value='cluster'), Parameter('scan_timing_mode', value='ros')])
    outputs, diagnostic = [], []
    monkeypatch.setattr(instance.pub, 'publish', outputs.append)
    monkeypatch.setattr(instance.diagnostics, 'publish', diagnostic.append)
    try:
        scan, tracks = inputs()
        before = serialize_message(scan)
        instance._on_tracks(tracks)
        set_tf(instance, 10.)
        set_tf(instance, 10.2)
        instance._on_scan(scan)
        instance._drain()
        assert all(math.isnan(r) for r in outputs[0].ranges)
        restored = deepcopy(outputs[0])
        restored.ranges = scan.ranges
        assert serialize_message(restored) == before
        assert instance.latest['clusters_matched'] == 1
        instance._diagnose()
        values = {item.key: item.value for item in diagnostic[-1].status[0].values}
        assert values['mask_method'] == 'cluster'
        assert values['total_clusters_matched'] == '1'
        changed = instance.set_parameters([Parameter('mask_method', value='radius')])
        assert not changed[0].successful
    finally:
        instance.destroy_node()
        rclpy.shutdown()
