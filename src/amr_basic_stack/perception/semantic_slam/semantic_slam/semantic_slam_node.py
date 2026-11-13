"""Capture-time object mapping alongside the existing geometric localization."""

from collections import deque
import fcntl
import math
from pathlib import Path
import time

from amr_interfaces.msg import SemanticObject
from amr_interfaces.msg import SemanticObjectArray
from amr_interfaces.msg import SemanticObservationArray
from amr_interfaces.msg import SemanticRelation
from amr_interfaces.msg import TrackedObjectArray
from amr_interfaces.srv import SetObjectAlias

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.time import Time

from semantic_slam.object_map import ObjectMap
from semantic_slam.object_map import Observation

import tf2_ros

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# A track this far outside the observed AABB is beside the object, not on it.
DYNAMIC_FOOTPRINT_MARGIN = 0.1


def rotation_matrix(q):
    """Convert a finite nonzero quaternion to an orthonormal matrix."""
    vector = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(vector)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = vector / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def stamp_seconds(stamp):
    """ROS time in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class SemanticSlamNode(Node):
    """Persistent object memory with optional optimized-trajectory reprojection."""

    def __init__(self):
        super().__init__('semantic_slam_node')
        defaults = {
            'map_id': '',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'database_path': '~/.ros/amr/semantic_objects.json',
            'save_period_sec': 10.0,
            'association_distance': 0.8,
            'ambiguity_margin': 0.2,
            'confirm_count': 3,
            'min_observation_interval': 0.3,
            'stale_after': 60.0,
            'max_objects': 200,
            'max_points': 512,
            'voxel_size': 0.05,
            'observation_max_age': 2.0,
            'odom_association_window': 5.0,
            'duplicate_merge_distance': 1.5,
            'require_localization': True,
            'localization_topic': '/odometry/map',
            'localization_is_odometry': True,
            'localization_timeout': 3.0,
            'max_pose_variance': 0.5,
            'localization_settle_time': 2.0,
            'enable_cartographer_correction': False,
            'cartographer_graph_id': '',
            'cartographer_tracking_frame': 'camera_gyro_frame',
            'cartographer_trajectory_id': -1,
            'cartographer_states_service': '/get_trajectory_states',
            'cartographer_query_service': '/trajectory_query',
            'graph_max_interpolation_gap': 1.0,
            'graph_max_nodes': 20000,
            'graph_max_trajectories': 8,
            'graph_max_anchors_per_object': 16,
            'graph_poll_period': 1.0,
            'graph_service_timeout': 3.0,
            'graph_freshness': 3.0,
            'graph_node_timeout': 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not math.isfinite(self.p['save_period_sec']) or self.p['save_period_sec'] <= 0:
            raise ValueError('save_period_sec must be finite and positive')
        if self.p['enable_cartographer_correction'] and not self.p['cartographer_graph_id']:
            raise ValueError('cartographer_graph_id must identify this saved graph/session')
        path = Path(self.p['database_path']).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = path.with_suffix('.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another semantic mapper owns this database') from exc
        self.memory = ObjectMap(
            self.p['map_id'],
            str(path),
            self.p['association_distance'],
            self.p['ambiguity_margin'],
            self.p['confirm_count'],
            self.p['min_observation_interval'],
            self.p['stale_after'],
            self.p['max_objects'],
            self.p['max_points'],
            self.p['voxel_size'],
            graph_id=(self.p['cartographer_graph_id']
                      if self.p['enable_cartographer_correction'] else ''),
            max_anchors=self.p['graph_max_anchors_per_object'],
            odom_window=self.p['odom_association_window'],
            merge_distance=self.p['duplicate_merge_distance'],
        )
        self.carto = None
        if self.p['enable_cartographer_correction']:
            from semantic_slam.cartographer_adapter import CartographerAdapter
            self.carto = CartographerAdapter(self, self.memory, self.p)
            self.create_timer(0.1, self._graph_tick)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pending = deque(maxlen=10)
        self.pose_received, self.pose_good = -math.inf, False
        self.pose_history = deque(maxlen=100)
        self.pause_until, self.reset_stamp, self.last_clock = 0.0, 0.0, 0.0
        self.dynamic_tracks = deque(maxlen=2000)
        self.create_subscription(TrackedObjectArray, '/tracks', self._tracks, 10)
        self.create_subscription(
            SemanticObservationArray,
            '/semantic/observations',
            self._observe,
            qos_profile_sensor_data,
        )
        localization_type = (
            Odometry if self.p['localization_is_odometry'] else PoseWithCovarianceStamped)
        self.create_subscription(localization_type, self.p['localization_topic'], self._pose, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._relocalize, qos_profile_sensor_data
        )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(SemanticObjectArray, '/semantic/objects', qos)
        self.markers = self.create_publisher(MarkerArray, '/semantic/object_markers', qos)
        self._point_messages = {}
        self.create_service(SetObjectAlias, '/semantic/set_alias', self._alias)
        self.create_timer(0.1, self._process)
        self.create_timer(1.0, self._publish)
        self.create_timer(self.p['save_period_sec'], self._save)
        self.get_logger().info(
            f'Object map {self.memory.map_id}: {len(self.memory.objects)} saved objects'
        )

    def _pose(self, msg):
        cov = np.asarray(msg.pose.covariance).reshape(6, 6)
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(msg.header.stamp)
        self.pose_good = (
            msg.header.frame_id == self.p['map_frame']
            and -0.1 <= age <= self.p['localization_timeout']
            and np.isfinite(cov).all()
            and min(cov.diagonal()) >= 0
            and max(cov[0, 0], cov[1, 1], cov[5, 5]) <= self.p['max_pose_variance']
        )
        self.pose_received = time.monotonic()
        if self.pose_good:
            p = msg.pose.pose.position
            self.pose_history.append((stamp_seconds(msg.header.stamp),
                                      np.array([p.x, p.y, p.z]), cov.copy()))

    def _graph_tick(self):
        """Publish revision/readiness changes without waiting for the marker timer."""
        ready, revision = self.carto.ready, self.memory.revision
        self.carto.tick()
        if ready != self.carto.ready or revision != self.memory.revision:
            self._publish()

    def _relocalize(self, _msg):
        if self.carto:
            self.carto.invalidate()
        self.pending.clear()
        self.pause_until = time.monotonic() + self.p['localization_settle_time']
        self.reset_stamp = self.get_clock().now().nanoseconds / 1e9
        self.pose_good = False
        self.pose_history.clear()

    def _observe(self, msg):
        if msg.header.frame_id and stamp_seconds(msg.header.stamp) > 0:
            self.pending.append((time.monotonic(), msg))

    def _tracks(self, msg):
        """Keep a short DYNAMIC-track history; UNKNOWN never means proven static."""
        now = self.get_clock().now().nanoseconds / 1e9
        # Observations wait up to observation_max_age, then match within 1 s.
        horizon = now - self.p['observation_max_age'] - 1.0
        while self.dynamic_tracks and self.dynamic_tracks[0][0] < horizon:
            self.dynamic_tracks.popleft()
        for track in msg.objects:
            stamp = stamp_seconds(track.header.stamp)
            if track.motion_state != 2 or not 0 <= now - stamp <= 1.0:
                continue
            p, v = track.pose.pose.position, track.velocity.twist.linear
            point, velocity = np.array([p.x, p.y, p.z]), np.array([v.x, v.y])
            pose_cov = np.asarray(track.pose.covariance).reshape(6, 6)[:2, :2]
            velocity_cov = np.asarray(track.velocity.covariance).reshape(6, 6)[:2, :2]
            cross_cov = np.asarray(track.position_velocity_covariance).reshape(2, 2)
            if not all(np.isfinite(a).all()
                       for a in (point, velocity, pose_cov, velocity_cov, cross_cov)):
                continue
            try:
                if track.header.frame_id != self.p['map_frame']:
                    tf = self.tf_buffer.lookup_transform(
                        self.p['map_frame'],
                        track.header.frame_id,
                        Time.from_msg(track.header.stamp),
                    )
                    t = tf.transform.translation
                    rotation = rotation_matrix(tf.transform.rotation)
                    point = rotation @ point + [t.x, t.y, t.z]
                    planar = rotation[:2, :2]
                    velocity = planar @ velocity
                    pose_cov, velocity_cov, cross_cov = (
                        planar @ c @ planar.T for c in (pose_cov, velocity_cov, cross_cov))
            except (tf2_ros.TransformException, ValueError):
                continue
            self.dynamic_tracks.append((stamp, track.class_id, track.track_id, point[:2],
                                        velocity, pose_cov, cross_cov, velocity_cov))

    def _dynamic_predictions(self, stamp, class_id):
        """Yield each matching track predicted to the capture time with its gate."""
        nearest = {}
        for sample in self.dynamic_tracks:
            seen, cls, track_id = sample[:3]
            dt = abs(stamp - seen)
            if (cls == class_id or cls < 0) and dt <= 1.0 and (
                    track_id not in nearest or dt < abs(stamp - nearest[track_id][0])):
                nearest[track_id] = sample
        for seen, _, _, point, velocity, pose_cov, cross_cov, velocity_cov in nearest.values():
            tau = stamp - seen
            # Constant-velocity extrapolation, including the P_pv cross term.
            sigma = pose_cov + tau * (cross_cov + cross_cov.T) + tau * tau * velocity_cov
            spread = math.sqrt(max(float(np.linalg.eigvalsh(sigma).max()), 0.0))
            # Bounded so one diverged track cannot blank out a whole room.
            yield point + velocity * tau, min(0.5 + 2.0 * spread, 2.0)

    def _dynamic_at(self, stamp, class_id, xy):
        """Compare an observation with each track predicted to the capture time."""
        return any(np.linalg.norm(xy - predicted) <= gate
                   for predicted, gate in self._dynamic_predictions(stamp, class_id))

    def _dynamic_inside(self, stamp, class_id, points):
        """
        Whether a dynamic track lies on the observed object itself.

        The exclusion gate above is wide enough that a person walking past a chair
        drops the chair's observation. That is cheap; marking the stored chair as
        moving is not, so it requires the track inside the observed footprint.
        """
        lo, hi = np.quantile(np.asarray(points)[:, :2], [0.05, 0.95], axis=0)
        lo, hi = lo - DYNAMIC_FOOTPRINT_MARGIN, hi + DYNAMIC_FOOTPRINT_MARGIN
        return any(np.all((lo <= predicted) & (predicted <= hi))
                   for predicted, _ in self._dynamic_predictions(stamp, class_id))

    def _odom_transform(self, header):
        """Capture-time sensor pose in odometry, or None to fall back to the map."""
        if not self.p['odom_frame']:
            return None, None
        if header.frame_id == self.p['odom_frame']:
            return np.eye(3), np.zeros(3)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.p['odom_frame'], header.frame_id, Time.from_msg(header.stamp))
            t = transform.transform.translation
            return rotation_matrix(transform.transform.rotation), np.array([t.x, t.y, t.z])
        except (tf2_ros.TransformException, ValueError):
            return None, None

    def _process(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.last_clock:
            self.pending.clear()
            self.dynamic_tracks.clear()
            self.get_logger().error('ROS time moved backward; use a separate DB for replay')
            self.reset_stamp = math.inf
            if self.carto:
                self.carto.invalidate()
        self.last_clock = now
        if self.carto and not self.carto.ready:
            return
        mono = time.monotonic()
        if mono < self.pause_until or (
            self.p['require_localization']
            and (not self.pose_good or mono - self.pose_received > self.p['localization_timeout'])
        ):
            self.pending.clear()
            return
        deferred = deque(maxlen=10)
        while self.pending:
            received, msg = self.pending.popleft()
            stamp = stamp_seconds(msg.header.stamp)
            if (
                stamp <= self.reset_stamp
                or now - stamp > self.p['observation_max_age']
                or stamp > now + 0.1
            ):
                continue
            try:
                if msg.header.frame_id == self.p['map_frame']:
                    rotation, translation = np.eye(3), np.zeros(3)
                else:
                    transform = self.tf_buffer.lookup_transform(
                        self.p['map_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp)
                    )
                    rotation = rotation_matrix(transform.transform.rotation)
                    t = transform.transform.translation
                    translation = np.array([t.x, t.y, t.z])
            except (tf2_ros.TransformException, ValueError):
                if mono - received < self.p['observation_max_age']:
                    deferred.append((received, msg))
                continue
            odom_rotation, odom_translation = self._odom_transform(msg.header)
            observations, retry_batch = [], False
            for obs in msg.observations[: self.p['max_objects']]:
                if (obs.header != msg.header or len(obs.points) > self.p['max_points']
                        or len(obs.points) < 3 or len(obs.covariance) != 9):
                    continue
                points = np.array([[p.x, p.y, p.z] for p in obs.points])
                if points.ndim != 2 or points.shape[1] != 3:
                    continue
                cov = np.asarray(obs.covariance).reshape(3, 3)
                transformed = points @ rotation.T + translation
                representative = np.median(transformed, axis=0)
                odom_position = (
                    None if odom_rotation is None
                    else np.median(points @ odom_rotation.T + odom_translation, axis=0).tolist())
                map_covariance = rotation @ cov @ rotation.T
                if self.p['require_localization']:
                    if not self.pose_history:
                        continue
                    pose_stamp, base_position, pose_cov = min(
                        self.pose_history, key=lambda sample: abs(sample[0] - stamp))
                    if abs(pose_stamp - stamp) > 0.5:
                        continue
                    relative = representative - base_position
                    # Planar map-pose uncertainty includes translation, heading,
                    # and their correlation. Heading error grows with range.
                    jacobian = np.zeros((3, 6))
                    jacobian[:, :3] = np.eye(3)
                    jacobian[:, 5] = [-relative[1], relative[0], 0]
                    map_covariance += jacobian @ pose_cov @ jacobian.T
                anchor = None
                if self.carto:
                    try:
                        tracking_tf = self.tf_buffer.lookup_transform(
                            self.p['cartographer_tracking_frame'], msg.header.frame_id,
                            Time.from_msg(msg.header.stamp))
                        tracking_rotation = rotation_matrix(tracking_tf.transform.rotation)
                        tt = tracking_tf.transform.translation
                        local_points = points @ tracking_rotation.T + [tt.x, tt.y, tt.z]
                        # Rotate the conservative sensor+EKF covariance into tracking axes.
                        local_cov = tracking_rotation @ rotation.T @ map_covariance
                        local_cov = local_cov @ rotation @ tracking_rotation.T
                        anchor, projected, projected_cov = self.carto.graph.bind(
                            stamp, local_points, local_cov)
                    except (LookupError, tf2_ros.TransformException, ValueError):
                        if mono - received < self.p['observation_max_age']:
                            retry_batch = True
                        continue
                    transformed, map_covariance = projected, projected_cov
                    representative = np.median(transformed, axis=0)
                if self._dynamic_at(stamp, obs.class_id, representative[:2]):
                    if self._dynamic_inside(stamp, obs.class_id, transformed):
                        for record in self.memory.objects.values():
                            if (
                                record.class_id == obs.class_id
                                and np.linalg.norm(
                                    representative[:2] - np.array(record.position)[:2])
                                <= 0.5
                            ):
                                record.status, record.movement_votes = 3, 3
                                self.memory.dirty = True
                    continue
                observations.append(
                    Observation(
                        obs.class_id,
                        obs.class_name,
                        stamp,
                        transformed.tolist(),
                        map_covariance.ravel().tolist(),
                        obs.confidence,
                        obs.valid_depth_ratio,
                        obs.support_method,
                        anchor=anchor,
                        odom_position=odom_position,
                    )
                )
            if retry_batch:
                deferred.append((received, msg))
            self.memory.update(observations)
            if self.carto:
                self.memory.reproject(self.carto.graph)
        self.pending = deferred

    def _alias(self, request, response):
        try:
            self.memory.set_alias(request.object_id, request.alias)
        except ValueError as exc:
            response.success, response.message = False, f'alias not set: {exc}'
            return response
        try:
            self.memory.save()
            response.success, response.message = True, 'Alias saved'
        except (ValueError, OSError) as exc:
            response.success, response.message = False, f'alias set, not persisted: {exc}'
            self.get_logger().error(response.message)
        return response

    def _save(self):
        try:
            self.memory.save()
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Cannot persist semantic objects: {exc}')

    def _publish(self):
        if not hasattr(self, '_point_messages'):
            self._point_messages = {}
        self._point_messages = {
            key: points for key, points in self._point_messages.items()
            if key in self.memory.objects
        }
        now = self.get_clock().now()
        self.memory.age(now.nanoseconds / 1e9)
        out = SemanticObjectArray()
        out.header.stamp, out.header.frame_id = now.to_msg(), self.p['map_frame']
        out.map_id = self.memory.map_id
        out.revision = self.memory.revision
        out.pose_graph_ready = not self.carto or self.carto.ready
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for index, record in enumerate(self.memory.objects.values()):
            obj = SemanticObject()
            obj.object_id, obj.class_id = record.object_id, record.class_id
            obj.class_name, obj.alias = record.class_name, record.alias
            obj.status, obj.support_method = record.status, record.method
            obj.observation_count = record.count
            for field, values in (
                (obj.position, record.position),
                (obj.box_center, record.center),
                (obj.extent, record.extent),
            ):
                field.x, field.y, field.z = map(float, values)
            points = self._point_messages.get(record.object_id)
            if points is None or len(points) != len(record.points):
                points = [Point32() for _ in record.points]
                self._point_messages[record.object_id] = points
            # rclpy serializes publish() synchronously. Reuse only the owned
            # point messages, updating every coordinate before each publication.
            for point, values in zip(points, record.points):
                point.x, point.y, point.z = map(float, values)
            obj.points = points
            obj.covariance = record.covariance
            obj.last_seen = Time(seconds=record.last_seen).to_msg()
            out.objects.append(obj)
            box = Marker()
            box.header, box.ns, box.id = out.header, 'objects', index
            box.type, box.action = Marker.CUBE, Marker.ADD
            box.pose.position, box.pose.orientation.w = obj.box_center, 1.0
            box.scale = obj.extent
            rgba = (0.1, 0.8, 0.3, 0.35) if record.status == 1 else (0.8, 0.5, 0.1, 0.35)
            box.color.r, box.color.g, box.color.b, box.color.a = rgba
            markers.markers.append(box)
            label = Marker()
            label.header, label.ns, label.id = out.header, 'labels', index
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y = record.position[:2]
            label.pose.position.z = record.center[2] + record.extent[2] / 2 + 0.2
            label.pose.orientation.w, label.scale.z = 1.0, 0.15
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = (
                f'{record.alias or record.class_name} {record.object_id[:8]} [{record.status}]'
            )
            markers.markers.append(label)
        for first, second, relation, distance in self.memory.relations():
            out.relations.append(
                SemanticRelation(
                    source_id=first, target_id=second, relation=relation, distance=distance
                )
            )
        self.pub.publish(out)
        self.markers.publish(markers)

    def destroy_node(self):
        if self.carto:
            self.carto.invalidate()
        self._save()
        if hasattr(self, 'lock'):
            self.lock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
