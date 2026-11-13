"""
Synthetic RGB-D -> persisted objects -> real Theta* -> navigation action.

Run only on an isolated ROS domain (never the vehicle domain 13).
No wheel/controller process is started; the base is a deterministic TF/odom fixture.
"""

import math
import os
import time

from amr_interfaces.action import NavigateToObject
from amr_interfaces.msg import TraversableMap
from amr_interfaces.srv import PlanObjectApproach

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

from global_planner.rviz_theta_star_node import RvizThetaStarNode

from nav_msgs.msg import Odometry

import numpy as np

from PIL import Image as PILImage

import pytest

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile

from semantic_slam.object_map import ObjectMap
from semantic_slam.semantic_navigation_node import SemanticNavigationNode
from semantic_slam.semantic_navigation_node import yaw
from semantic_slam.semantic_slam_node import SemanticSlamNode

from sensor_fusion.rgbd_detection_node import RgbdDetectionNode

from sensor_msgs.msg import CameraInfo

from std_msgs.msg import Bool

from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster


class Environment(Node):
    """Synthetic map, pose and image source, with no controller or motor node."""

    def __init__(self, detector, carto=False):
        super().__init__('semantic_test_environment')
        self.detector = detector
        self.pose, self.move, self.observe = [1.0, 2.0, 0.0], True, True
        self.armed, self.goals, self.arm_commands = True, [], []
        self.localization_good = True
        self.graph_shift, self.graph_error = 0.0, False
        self.graph_start = int(self.get_clock().now().nanoseconds // 100000000) - 30
        self.graph_prune = False
        self.graph_deleted = False
        if carto:
            from cartographer_ros_msgs.srv import GetTrajectoryStates
            from cartographer_ros_msgs.srv import TrajectoryQuery
            self.create_service(GetTrajectoryStates, '/get_trajectory_states', self._states)
            self.create_service(TrajectoryQuery, '/trajectory_query', self._trajectory)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(TraversableMap, '/traversable_map', qos)
        self.armed_pub = self.create_publisher(Bool, '/mpc/armed', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.localization_pub = self.create_publisher(Odometry, '/odometry/map', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal, 10)
        self.create_subscription(Bool, '/mpc/arm', self._arm, 10)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        camera = TransformStamped()
        camera.header.frame_id, camera.child_frame_id = 'map', 'test_camera'
        camera.header.stamp = self.get_clock().now().to_msg()
        camera.transform.translation.x, camera.transform.translation.y = 2.0, 2.0
        camera.transform.translation.z = 1.0
        # optical z -> map x, optical x -> -map y, optical y -> -map z
        camera.transform.rotation.x, camera.transform.rotation.y = -0.5, 0.5
        camera.transform.rotation.z, camera.transform.rotation.w = -0.5, 0.5
        self.static.sendTransform(camera)
        info = CameraInfo()
        info.header.frame_id = 'test_camera'
        info.height, info.width = 10, 10
        info.k = [100.0, 0.0, 5.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0]
        detector._on_info(info)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.4, self._image)

    def _states(self, _, response):
        response.status.code = 13 if self.graph_error else 0
        response.trajectory_states.trajectory_id = [0, 1]
        response.trajectory_states.trajectory_state = [2, 3 if self.graph_deleted else 0]
        return response

    def _trajectory(self, _, response):
        response.status.code = 13 if self.graph_error else 0
        now_tick = int(self.get_clock().now().nanoseconds // 100000000)
        start = now_tick - 3 if self.graph_prune else self.graph_start
        for tick in range(start, now_tick + 1):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp.sec = tick // 10
            pose.header.stamp.nanosec = (tick % 10) * 100000000
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                2.0 + self.graph_shift, 2.0, 1.0)
            q = pose.pose.orientation
            q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
            response.trajectory.append(pose)
        return response

    def _goal(self, msg):
        self.goals.append(msg)
        if self.move:
            self.pose = [msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation)]

    def _arm(self, msg):
        self.arm_commands.append(msg.data)
        self.armed = msg.data

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'map', 'base_footprint', stamp
        tf.transform.translation.x, tf.transform.translation.y = self.pose[:2]
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.pose[2] / 2), math.cos(
            self.pose[2] / 2
        )
        self.dynamic.sendTransform(tf)
        # Existing legacy detection needs base_link<-camera; semantic uses map<-camera.
        base = TransformStamped()
        base.header.frame_id, base.child_frame_id, base.header.stamp = (
            'base_footprint',
            'base_link',
            stamp,
        )
        base.transform.rotation.w = 1.0
        self.dynamic.sendTransform(base)
        grid = TraversableMap()
        grid.header.frame_id, grid.header.stamp, grid.oldest_observation = 'map', stamp, stamp
        grid.sensor_watermark = stamp
        grid.grid.header = grid.header
        grid.grid.info.resolution, grid.grid.info.width, grid.grid.info.height = 0.1, 80, 80
        grid.grid.info.origin.orientation.w = 1.0
        values = np.zeros((80, 80), dtype=np.int8)
        values[19:22, 39:42] = 100
        grid.grid.data = values.ravel().tolist()
        self.map_pub.publish(grid)
        self.armed_pub.publish(Bool(data=self.armed))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        self.odom_pub.publish(odom)
        localization = Odometry()
        localization.header.stamp, localization.header.frame_id = stamp, 'map'
        localization.pose.pose.position.x, localization.pose.pose.position.y = self.pose[:2]
        localization.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        localization.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        localization.pose.covariance = (
            np.eye(6) * (0.03 if self.localization_good else 5.0)).ravel().tolist()
        self.localization_pub.publish(localization)

    def _image(self):
        if not self.observe:
            return
        stamp = self.get_clock().now().to_msg()
        color = self.detector.bridge.cv2_to_imgmsg(np.zeros((10, 10, 3), dtype=np.uint8), 'bgr8')
        depth = self.detector.bridge.cv2_to_imgmsg(
            np.full((10, 10), 2000, dtype=np.uint16), '16UC1'
        )
        color.header.frame_id, color.header.stamp = 'test_camera', stamp
        depth.header = color.header
        self.detector._on_pair(color, depth)


@pytest.fixture
def pipeline(tmp_path, request, monkeypatch):
    # colcon test bypasses tools/run_tests.sh: isolate here, never on vehicle domain 13.
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '13'):
        monkeypatch.setenv('ROS_DOMAIN_ID', '83')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('CYCLONEDDS_URI', '')
    PILImage.fromarray(np.full((80, 80), 255, dtype=np.uint8)).save(tmp_path / 'fixture.pgm')
    (tmp_path / 'fixture.yaml').write_text(
        'image: fixture.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
    )
    arguments = [
        '--ros-args',
        '-p',
        'map:=fixture',
        '-p',
        f'maps_dir:={tmp_path}',
        '-p',
        'map_id:=fixture-v1',
        '-p',
        f'database_path:={tmp_path}/objects.json',
        '-p',
        'require_localization:=true',
        '-p',
        'require_initial_pose:=false',
        '-p',
        'publish_semantic:=true',
        '-p',
        'enable_goal_dispatch:=true',
    ]
    carto = getattr(request, 'param', False)
    if carto:
        arguments.extend([
            '-p', 'enable_cartographer_correction:=true',
            '-p', 'cartographer_graph_id:=fixture-carto',
            '-p', 'cartographer_tracking_frame:=test_camera',
            '-p', 'graph_poll_period:=0.15',
            '-p', 'require_pose_graph:=true',
        ])
    rclpy.init(args=arguments)
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        detector = RgbdDetectionNode()
        detector.detector.detect = lambda _: [
            {
                'class_id': 56,
                'class_name': 'chair',
                'confidence': 0.9,
                'cx': 5.0,
                'cy': 5.0,
                'w': 8.0,
                'h': 8.0,
                'mask': np.ones((10, 10), dtype=bool),
            }
        ]
        mapper, planner, navigator = (
            SemanticSlamNode(),
            RvizThetaStarNode(),
            SemanticNavigationNode(),
        )
        env = Environment(detector, carto=carto)
        nodes = [detector, mapper, planner, navigator, env]
        for node in nodes:
            executor.add_node(node)

        def until(predicate, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if predicate():
                    return
            raise AssertionError('Timed out waiting for synthetic ROS pipeline')

        until.executor = executor

        until(lambda: navigator.objects and any(o.status == 1 for o in navigator.objects.objects))
        yield env, mapper, planner, navigator, until, tmp_path
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def send_action(env, until, request):
    client = ActionClient(env, NavigateToObject, '/semantic/navigate_to_object')
    until(client.server_is_ready)
    future = client.send_goal_async(request)
    until(future.done)
    assert future.result().accepted
    handle = future.result()
    return client, handle, handle.get_result_async()


def test_preview_arrival_reobservation_and_persistence(pipeline):
    env, mapper, planner, navigator, until, path = pipeline
    object_id = navigator.objects.objects[0].object_id
    obj = navigator.objects.objects[0]
    assert abs(obj.position.x - 4.0) < 0.05 and abs(obj.position.y - 2.0) < 0.05
    assert obj.covariance[0] >= 0.03 and obj.covariance[4] > 0.2
    client, _, result = send_action(
        env, until, NavigateToObject.Goal(object_id=object_id, preview_only=True)
    )
    until(result.done)
    assert result.result().result.status == 'PREVIEWED'
    assert not env.goals and not env.arm_commands
    client.destroy()
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(lambda: navigator.mission and navigator.mission['state'] == 'VERIFYING')
    arrival = navigator.mission['arrival_stamp']
    until(result.done)
    assert result.result().result.status == 'SUCCEEDED'
    assert next(iter(mapper.memory.objects.values())).last_seen > arrival
    assert env.goals and env.arm_commands and not any(env.arm_commands)
    assert planner.current_goal is None and planner.current_path == []
    mapper.memory.save()
    assert object_id in ObjectMap('fixture-v1', str(path / 'objects.json')).objects
    client.destroy()


def test_cancel_clears_retained_goal_and_never_arms(pipeline):
    env, mapper, planner, navigator, until, _ = pipeline
    env.move = False
    object_id = navigator.objects.objects[0].object_id
    client, handle, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(lambda: bool(env.goals))
    # The dispatched /goal_pose keeps the approach service's unknown-blocked policy.
    until(lambda: planner.current_goal is not None)
    assert planner.current_goal_verified
    cancellation = handle.cancel_goal_async()
    until(cancellation.done)
    until(result.done)
    assert result.result().result.status == 'CANCELLED'
    assert env.arm_commands and not any(env.arm_commands)
    assert planner.current_goal is None and planner.current_path == []
    client.destroy()


def test_approach_rejects_stale_map_and_blocked_view(pipeline):
    env, mapper, planner, navigator, until, _ = pipeline
    req = PlanObjectApproach.Request()
    req.target_center.x, req.target_center.y = 4.0, 2.0
    req.target_extent.x, req.target_extent.y = 0.1, 0.1
    req.min_standoff, req.max_standoff, req.max_snap_distance = 0.5, 1.5, 0.2
    req.require_fresh_map = True
    candidate = PoseStamped()
    candidate.header.frame_id = 'map'
    candidate.pose.position.x, candidate.pose.position.y = 3.0, 2.0
    req.candidates = [candidate]
    response = planner._plan_object_approach(req, PlanObjectApproach.Response())
    assert response.success
    planner._traversable_message.sensor_watermark.sec -= 10
    response = planner._plan_object_approach(req, PlanObjectApproach.Response())
    assert not response.success and 'stale' in response.message


def test_arrival_without_new_observation_fails(pipeline):
    """Controller arrival alone cannot satisfy an object mission."""
    env, mapper, planner, navigator, until, _ = pipeline
    navigator.p['reverify_timeout'] = 0.4
    object_id = navigator.objects.objects[0].object_id
    env.observe = False
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(result.done)
    assert result.result().result.status == 'REVERIFY_FAILED'
    assert planner.current_goal is None
    assert env.arm_commands and not any(env.arm_commands)
    client.destroy()


def test_bounded_search_then_object_approach(pipeline):
    """An observation waypoint is followed by the final object approach."""
    env, mapper, planner, navigator, until, _ = pipeline
    navigator.views = [[1.0, 2.0, 0.0]]
    alias = 'found during search'
    client, _, result = send_action(
        env, until, NavigateToObject.Goal(alias=alias, allow_search=True)
    )
    until(lambda: bool(env.goals))
    object_id = next(iter(mapper.memory.objects))
    mapper.memory.set_alias(object_id, alias)
    mapper._publish()
    until(result.done)
    assert result.result().result.status == 'SUCCEEDED'
    assert len(env.goals) == 2
    assert result.result().result.object_id == object_id
    client.destroy()


def test_search_exhaustion_and_timeout_stop(pipeline):
    """Search and mission duration have explicit bounds and clear planner state."""
    env, mapper, planner, navigator, until, _ = pipeline
    navigator.views = [[1.0, 2.0, 0.0]]
    navigator.p['reverify_timeout'] = 0.3
    client, _, result = send_action(
        env, until, NavigateToObject.Goal(class_name='bookcase', allow_search=True)
    )
    until(result.done)
    assert result.result().result.status == 'SEARCH_EXHAUSTED'
    assert len(env.goals) == 1 and planner.current_goal is None
    client.destroy()
    env.armed, env.move = True, False
    # Refresh the synthetic gate before requesting the second mission.
    until(lambda: navigator.armed)
    object_id = navigator.objects.objects[0].object_id
    client, _, result = send_action(
        env, until, NavigateToObject.Goal(object_id=object_id, timeout=0.5)
    )
    until(result.done)
    assert result.result().result.status == 'TIMEOUT'
    assert planner.current_goal is None and not any(env.arm_commands)
    client.destroy()


def test_localization_quality_loss_stops_mission(pipeline):
    """Bad map-pose covariance halts dispatch and preserves the saved map."""
    env, mapper, planner, navigator, until, _ = pipeline
    env.move = False
    object_id = navigator.objects.objects[0].object_id
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(lambda: bool(env.goals))
    env.localization_good = False
    until(result.done)
    assert result.result().result.status == 'LOCALIZATION_UNAVAILABLE'
    assert object_id in mapper.memory.objects
    assert planner.current_goal is None
    assert env.arm_commands and not any(env.arm_commands)
    client.destroy()


def test_wall_blocks_view_and_snap_cannot_change_object_approach(pipeline):
    """Path reachability cannot override visibility or resolved-pose constraints."""
    from copy import deepcopy

    env, mapper, planner, navigator, until, _ = pipeline
    req = PlanObjectApproach.Request()
    req.target_center.x, req.target_center.y = 4.0, 2.0
    req.target_extent.x, req.target_extent.y = 0.1, 0.1
    req.min_standoff, req.max_standoff, req.max_snap_distance = 0.5, 1.5, 0.2
    req.require_fresh_map = True
    candidate = PoseStamped()
    candidate.header.frame_id = 'map'
    candidate.pose.position.x, candidate.pose.position.y = 3.0, 2.0
    req.candidates = [candidate]
    message = deepcopy(planner._traversable_message)
    values = np.array(message.grid.data).reshape(80, 80)
    values[19:22, 35] = 100
    message.grid.data = values.ravel().tolist()
    planner._on_traversable_map(message)
    response = planner._plan_object_approach(req, PlanObjectApproach.Response())
    assert not response.success and 'visibility' in response.message
    # A blocked requested pose near the object is snapped, but its resulting
    # standoff still fails. This uses the real endpoint resolution code.
    candidate.pose.position.x = 3.85
    response = planner._plan_object_approach(req, PlanObjectApproach.Response())
    assert not response.success


def test_real_traversability_sensor_watermark_and_reset(pipeline):
    """Fresh publication cannot conceal an old or missing configured sensor."""
    from types import SimpleNamespace
    from unittest.mock import Mock

    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import LaserScan
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from traversability.traversability_node import TraversabilityNode

    env, mapper, planner, navigator, until, _ = pipeline
    producer = TraversabilityNode()
    try:
        producer._on_base_map(planner._traversable_message.grid)
        producer.publisher = Mock()
        producer.debug_publisher = None
        producer._scan_cells = lambda _: (np.array([1]), np.array([], dtype=int))
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        producer.tf_buffer = SimpleNamespace(lookup_transform=lambda *_, **__: transform)
        scan = LaserScan()
        scan.header.stamp = Time(sec=100)
        producer._on_scan(scan)
        producer._publish()
        assert producer.publisher.publish.call_args.args[0].sensor_watermark.sec == 0
        cloud = point_cloud2.create_cloud_xyz32(
            Header(frame_id='map', stamp=Time(sec=101)), [[2.0, 2.0, 0.1]])
        producer._on_band_cloud(cloud)
        producer._publish()
        assert producer.publisher.publish.call_args.args[0].sensor_watermark.sec == 100
        scan.header.stamp = Time(sec=103)
        producer._on_scan(scan)
        producer._publish()
        assert producer.publisher.publish.call_args.args[0].sensor_watermark.sec == 101
        empty = point_cloud2.create_cloud_xyz32(Header(frame_id='map', stamp=Time(sec=104)), [])
        producer._on_band_cloud(empty)
        producer._publish()
        assert producer.publisher.publish.call_args.args[0].sensor_watermark.sec == 103
        producer._reset_memory((80, 80), 'test map revision')
        producer._publish()
        assert producer.publisher.publish.call_args.args[0].sensor_watermark.sec == 0
    finally:
        producer.destroy_node()


def test_uncertain_target_and_shutdown_cleanup(pipeline):
    """Uncertain geometry is refused, and shutdown drains an active mission."""
    env, mapper, planner, navigator, until, _ = pipeline
    env.move, env.observe = False, False
    record = next(iter(mapper.memory.objects.values()))
    object_id = record.object_id
    original = list(record.covariance)
    record.covariance = (np.eye(3) * 2.0).ravel().tolist()
    mapper._publish()
    until(lambda: navigator.objects.objects[0].covariance[0] > 1.0)
    client, _, result = send_action(
        env, until, NavigateToObject.Goal(object_id=object_id, preview_only=True))
    until(result.done)
    assert result.result().result.status == 'TARGET_UNCERTAIN'
    assert not env.goals
    client.destroy()

    record.covariance = original
    mapper._publish()
    until(lambda: navigator.objects.objects[0].covariance[0] < 1.0)
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(lambda: bool(env.goals))
    navigator.shutting_down = True
    navigator._finish(False, 'SHUTDOWN', 'Test shutdown request')
    until(result.done)
    assert result.result().result.status == 'SHUTDOWN'
    assert planner.current_goal is None and env.arm_commands and not any(env.arm_commands)
    rejected = client.send_goal_async(NavigateToObject.Goal(object_id=object_id))
    until(rejected.done)
    assert not rejected.result().accepted
    client.destroy()


@pytest.mark.parametrize('pipeline', [True], indirect=True)
def test_cartographer_service_correction_stops_old_goal_and_preserves_identity(pipeline):
    """Actual ROS service responses correct geometry and invalidate dispatched goals."""
    env, mapper, planner, navigator, until, path = pipeline
    env.move, env.observe = False, False
    record = next(iter(mapper.memory.objects.values()))
    # Drain capture-time observations already queued before disabling the source.
    until(lambda: not mapper.pending and
          env.get_clock().now().nanoseconds / 1e9 - record.last_seen > 2.1)
    object_id, seen, count = record.object_id, record.last_seen, record.count
    position = np.array(record.position)
    mapper.memory.set_alias(object_id, 'window chair')
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(lambda: bool(env.goals))
    env.graph_shift = 0.3
    until(result.done)
    assert result.result().result.status == 'MAP_REPROJECTED'
    assert mapper.memory.revision == 1
    assert np.allclose(record.position, position + [0.3, 0.0, 0.0])
    assert record.object_id == object_id and record.alias == 'window chair'
    assert record.last_seen == seen and record.count == count and record.status == 1
    assert planner.current_goal is None and env.arm_commands and not any(env.arm_commands)
    mapper.memory.save()
    saved = ObjectMap('fixture-v1', str(path / 'objects.json'), graph_id='fixture-carto')
    assert saved.graph_id == 'fixture-carto' and saved.revision == 1
    assert saved.objects[object_id].anchors[0]['points']
    client.destroy()


@pytest.mark.parametrize('pipeline', [True], indirect=True)
def test_small_graph_correction_does_not_stop_mission(pipeline):
    """Routine optimization (mm-cm) bumps the revision but keeps the approach."""
    env, mapper, planner, navigator, until, _ = pipeline
    env.move, env.observe = False, False
    record = next(iter(mapper.memory.objects.values()))
    until(lambda: not mapper.pending)
    client, handle, result = send_action(
        env, until, NavigateToObject.Goal(object_id=record.object_id))
    until(lambda: bool(env.goals))
    revision = mapper.memory.revision
    env.graph_shift = 0.02
    until(lambda: navigator.objects.revision > revision)
    assert navigator.mission and navigator.mission['state'] == 'NAVIGATING'
    assert planner.current_goal is not None and not result.done()
    cancellation = handle.cancel_goal_async()
    until(cancellation.done)
    until(result.done)
    assert result.result().result.status == 'CANCELLED'
    client.destroy()


@pytest.mark.parametrize('pipeline', [True], indirect=True)
def test_cartographer_failure_trimming_and_deletion(pipeline):
    """Query failure and deletion fail closed; trimmed history keeps its final pose."""
    env, mapper, planner, navigator, until, _ = pipeline
    env.move, env.observe = False, False
    record = next(iter(mapper.memory.objects.values()))
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=record.object_id))
    until(lambda: bool(env.goals))
    env.graph_error = True
    until(result.done)
    assert result.result().result.status == 'POSE_GRAPH_UNAVAILABLE'
    assert planner.current_goal is None and not env.armed
    client.destroy()
    env.graph_error = False
    until(lambda: mapper.carto.ready)
    revision, position = mapper.memory.revision, list(record.position)
    env.graph_prune = True
    until(lambda: all(a.get('frozen') for a in record.anchors))
    assert record.support_valid and record.status == 1 and mapper.carto.ready
    assert record.position == position and mapper.memory.revision == revision
    env.graph_deleted = True
    until(lambda: not record.support_valid)
    assert record.status == 2 and not mapper.carto.ready
    assert mapper.memory.revision == revision + 1


def test_target_change_during_planning_is_revalidated(pipeline):
    """A plan reply for a target that changed meanwhile is replanned, never dispatched."""
    from rclpy.task import Future

    env, mapper, planner, navigator, until, _ = pipeline
    env.move, env.observe = False, False
    object_id = navigator.objects.objects[0].object_id
    original, held, requests = navigator.planner.call_async, [], []

    def hold(request):
        requests.append(request)
        held.append((original(request), Future()))
        return held[-1][1]

    def published():
        return next(o for o in navigator.objects.objects if o.object_id == object_id)

    navigator.planner.call_async = hold
    client, _, result = send_action(env, until, NavigateToObject.Goal(object_id=object_id))
    until(lambda: held and held[0][0].done())
    approved = held[0][0].result()
    assert approved.success
    record = mapper.memory.objects[object_id]
    record.center = [record.center[0] + 0.5, *record.center[1:]]
    record.position = [record.position[0] + 0.5, *record.position[1:]]
    mapper._publish()
    until(lambda: abs(published().box_center.x - record.center[0]) < 1e-6)
    held[0][1].set_result(approved)
    until(lambda: len(requests) == 2)
    assert not env.goals and navigator.mission['replans'] == 1
    assert abs(requests[1].target_center.x - record.center[0]) < 1e-6
    # The target turns MOVING while the replan is pending: stop without dispatch.
    record.status = 3
    mapper._publish()
    until(lambda: published().status == 3)
    held[1][1].set_result(approved)
    until(result.done)
    assert result.result().result.status == 'TARGET_CHANGED'
    assert not env.goals and not env.arm_commands
    client.destroy()


def test_unanswered_planner_requests_are_released(pipeline):
    """A discovered but silent planner must not accumulate pending client futures."""
    env, mapper, planner, navigator, until, _ = pipeline
    env.move = False
    object_id = navigator.objects.objects[0].object_id
    until.executor.remove_node(planner)
    try:
        client, _, result = send_action(
            env, until, NavigateToObject.Goal(object_id=object_id, preview_only=True))
        until(lambda: navigator.mission and navigator.mission['state'] == 'PLANNING')
        plan = navigator.mission['plan_future']
        navigator.mission['plan_deadline'] = 0.0
        until(result.done)
        assert result.result().result.status == 'PLAN_TIMEOUT'
        assert plan.cancelled() and not navigator.planner._pending_requests
        client.destroy()
        until.executor.add_node(planner)
        client, handle, result = send_action(
            env, until, NavigateToObject.Goal(object_id=object_id))
        until(lambda: bool(env.goals))
        until.executor.remove_node(planner)
        cancellation = handle.cancel_goal_async()
        until(lambda: navigator.mission and navigator.mission['state'] == 'STOPPING')
        stop = navigator.mission['stop_future']
        until(result.done)
        assert result.result().result.status == 'STOP_FAILED'
        assert stop.cancelled() and not navigator.cancel_planner._pending_requests
        until(cancellation.done)
        client.destroy()
    finally:
        until.executor.add_node(planner)
