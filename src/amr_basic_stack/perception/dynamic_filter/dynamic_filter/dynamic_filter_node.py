#!/usr/bin/env python3
"""One time-preserving LaserScan filter; Cartographer launch owns two instances."""

from collections import Counter, deque
from dataclasses import fields
import math
import time

from amr_interfaces.msg import ImmTrackedObject, ImmTrackedObjectArray, TrackedObject
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

from .core import (FilterConfig, mask_scan, rigid_matrix, stamp_ns, TrackHistory,
                   TransformUnavailable)
from .observations import mask_observations, ObservationHistory


class DynamicFilterNode(Node):
    """Bounded FIFO and zero-timeout TF lookups keep input callbacks responsive."""

    def __init__(self, **kwargs):
        super().__init__('dynamic_filter_node', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        defaults = FilterConfig()
        self.config = FilterConfig(**{
            field.name: self.declare_parameter(
                field.name, getattr(defaults, field.name), descriptor).value
            for field in fields(defaults)})
        topics = {name: self.declare_parameter(name, default, descriptor).value
                  for name, default in (('scan_input_topic', 'scan_in'),
                                        ('scan_output_topic', 'scan_out'),
                                        ('tracks_topic', '/tracked_object_modes'))}
        if any(not name.strip() for name in topics.values()):
            raise ValueError('topic names must not be empty')
        if (self.resolve_topic_name(topics['scan_input_topic'])
                == self.resolve_topic_name(topics['scan_output_topic'])):
            raise ValueError('scan input and output must differ (including remappings)')
        self.history = TrackHistory(self.config)
        self.observations = ObservationHistory(self.config)
        self.observation_sensor_id = self.declare_parameter(
            'observation_sensor_id', '', descriptor).value
        observations_topic = self.declare_parameter(
            'tracked_observations_topic', '/tracked_observations', descriptor).value
        if self.config.mask_method == 'tracked_cluster':
            if not self.observation_sensor_id.strip() or not observations_topic.strip():
                raise ValueError('tracked_cluster requires observation_sensor_id and topic')
            # Radius/cluster can still run against older interface installations.
            from amr_interfaces.msg import TrackedObservationArray
            self.create_subscription(TrackedObservationArray, observations_topic,
                                     self._on_observations, qos_profile_sensor_data)
        self.pending = deque()
        self.totals = Counter()
        self.latest = {'reason': 'waiting_for_scan'}
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=self.config.tf_cache_duration), node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, topics['scan_output_topic'], 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '~/diagnostics', 10)
        self.create_subscription(LaserScan, topics['scan_input_topic'],
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(ImmTrackedObjectArray, topics['tracks_topic'],
                                 self._on_tracks, qos_profile_sensor_data)
        # Steady deadlines still expire if rosbag /clock pauses.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.config.retry_period, self._drain, clock=self.steady_clock)
        self.create_timer(self.config.diagnostic_period, self._diagnose,
                          clock=self.steady_clock)
        self.jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True), post_callback=self._on_clock_jump)
        self.get_logger().info(
            f'method={self.config.mask_method}, policy={self.config.filter_policy}, '
            f'radius_baseline={self.config.filter_radius} m; '
            'Failures pass through. Details: ~/diagnostics')

    def _on_clock_jump(self, _jump):
        self.history.clear()
        self.observations.clear()
        self.pending.clear()
        self.tf_buffer.clear()
        self.totals['clock_resets'] += 1
        self.latest = {'reason': 'clock_reset'}

    def _on_tracks(self, msg):
        self.history.add(msg)

    def _on_observations(self, msg):
        if not self.observations.add(msg, time.monotonic()):
            self.totals['observation_message_limit'] += 1

    def _on_scan(self, msg):
        if len(self.pending) >= self.config.pending_queue_size:
            oldest, _ = self.pending.popleft()
            self._publish(oldest, {'reason': 'queue_overflow', 'removed_points': 0,
                                   'used_tracks': 0, 'valid_points': self._valid_count(oldest)})
        self.pending.append((msg, time.monotonic()))

    @staticmethod
    def _valid_count(msg):
        return sum(math.isfinite(r) and msg.range_min <= r <= msg.range_max
                   for r in msg.ranges)

    def _transform_at(self, frame, nanoseconds):
        # tf2 uses Time(0) as "latest". Never silently apply it to epoch zero.
        if nanoseconds <= 0 or not frame:
            raise TransformUnavailable('missing frame or zero/negative TF epoch')
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', frame, Time(nanoseconds=nanoseconds, clock_type=ClockType.ROS_TIME),
                timeout=Duration()).transform
        except tf2_ros.TransformException as exc:
            raise TransformUnavailable(str(exc)) from exc
        t, q = transform.translation, transform.rotation
        return rigid_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

    def _drain(self):
        if not self.pending:
            return
        msg, received = self.pending[0]
        if self.config.mask_method == 'tracked_cluster':
            deadline = received + self.config.observation_wait_timeout
            # Events are incremental: wait the full bounded window so another
            # cluster/competing owner can still arrive for the same scan.
            if time.monotonic() < deadline:
                return
            records = self.observations.select(msg, self.observation_sensor_id, deadline)
            out, stats = mask_observations(
                msg, records, self.observation_sensor_id, self.config,
                confirmed=ImmTrackedObject.LIFECYCLE_CONFIRMED,
                dynamic=TrackedObject.MOTION_DYNAMIC)
            self.observations.prune(time.monotonic())
            self.pending.popleft()
            self._publish(out, stats)
            return
        snapshot, reason = self.history.select(stamp_ns(msg.header.stamp))
        out, stats = mask_scan(
            msg, snapshot, self.config, self._transform_at,
            confirmed=ImmTrackedObject.LIFECYCLE_CONFIRMED,
            dynamic=TrackedObject.MOTION_DYNAMIC, missing_reason=reason)
        waiting = stats['reason'] in ('tf_unavailable', 'tracks_missing', 'no_causal_tracks')
        if waiting and time.monotonic() - received < self.config.tf_wait_timeout:
            self.totals['wait_attempts'] += 1
            return
        self.pending.popleft()
        self._publish(out, stats)

    def _publish(self, scan, stats):
        self.pub.publish(scan)
        self.latest = stats
        self.totals['scans'] += 1
        self.totals['reason_' + stats['reason']] += 1
        for key, value in stats.items():
            if key != 'reason':
                self.totals[key] += value

    def _diagnose(self):
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name()
        status.hardware_id = 'dynamic_filter'
        status.message = self.latest['reason']
        healthy = ('filtered', 'no_points_in_radius', 'no_cluster_match',
                   'no_observation_match', 'empty_tracks',
                   'no_eligible_tracks', 'empty_scan', 'no_valid_points')
        status.level = (DiagnosticStatus.OK if status.message in healthy
                        else DiagnosticStatus.WARN)
        if self.latest.get('zero_time_increment'):
            status.level = DiagnosticStatus.WARN
        values = {**self.latest, 'mask_method': self.config.mask_method,
                  'filter_policy': self.config.filter_policy, 'pending_scans': len(self.pending),
                  **{'total_' + key: value for key, value in self.totals.items()}}
        status.values = [KeyValue(key=key, value=str(value))
                         for key, value in sorted(values.items())]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DynamicFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
