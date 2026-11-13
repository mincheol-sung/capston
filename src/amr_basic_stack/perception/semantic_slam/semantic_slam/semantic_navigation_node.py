"""Action-driven object navigation using the AMR goal/path and command gate."""

import math
import signal
import time

from amr_interfaces.action import NavigateToObject
from amr_interfaces.msg import SemanticObjectArray
from amr_interfaces.msg import TraversableMap
from amr_interfaces.srv import PlanObjectApproach

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from rclpy.time import Time

from semantic_slam.navigation import approach_candidates
from semantic_slam.navigation import at_approach
from semantic_slam.navigation import select_object
from semantic_slam.semantic_slam_node import stamp_seconds

from std_msgs.msg import Bool

from std_srvs.srv import Trigger

import tf2_ros


def yaw(q):
    """Extract planar yaw."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_message(x, y, angle, frame, stamp):
    """Create a facing pose."""
    result = PoseStamped()
    result.header.frame_id, result.header.stamp = frame, stamp
    result.pose.position.x, result.pose.position.y = float(x), float(y)
    result.pose.orientation.z, result.pose.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
    return result


class SemanticNavigationNode(Node):
    """Nonblocking mission state machine with bounded search and re-observation."""

    def __init__(self):
        super().__init__('semantic_navigation_node')
        defaults = {
            'map_id': '',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'enable_goal_dispatch': False,
            'require_localization': True,
            'require_pose_graph': False,
            'localization_topic': '/odometry/map',
            'localization_is_odometry': True,
            'odometry_topic': '/odometry/filtered',
            'object_freshness': 15.0,
            'max_target_variance': 0.5,
            'target_change_tolerance': 0.1,
            'standoff': 0.8,
            'min_standoff': 0.5,
            'max_standoff': 1.5,
            'max_snap_distance': 0.2,
            'mission_timeout': 120.0,
            'reverify_timeout': 8.0,
            'search_viewpoints': [0.0],
            'max_search_views': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if not self.p['map_id']:
            raise ValueError('map_id is required and must match the semantic mapper')
        views = self.p['search_viewpoints']
        self.views = [] if views == [0.0] else [views[i:i + 3] for i in range(0, len(views), 3)]
        if any(len(view) != 3 or not np.isfinite(view).all() for view in self.views):
            raise ValueError('search_viewpoints must contain x,y,yaw triples')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.objects, self.map_received = None, -math.inf
        self.traversable, self.traversable_received = None, -math.inf
        self.armed, self.odom, self.odom_received = False, None, -math.inf
        self.pose_good, self.pose_received = False, -math.inf
        self.busy, self.mission = False, None
        self.shutting_down = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SemanticObjectArray, '/semantic/objects', self._objects, latched)
        self.create_subscription(Bool, '/mpc/armed', self._armed, latched)
        self.create_subscription(Path, '/global_path', self._path, latched)
        self.create_subscription(TraversableMap, '/traversable_map', self._map, latched)
        self.create_subscription(
            Odometry, self.p['odometry_topic'], self._odom, qos_profile_sensor_data)
        localization_type = (
            Odometry if self.p['localization_is_odometry'] else PoseWithCovarianceStamped)
        self.create_subscription(localization_type, self.p['localization_topic'], self._pose, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._initial_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, '/goal_pose', self._external_goal, qos_profile_sensor_data
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.preview_pub = self.create_publisher(PoseStamped, '/semantic/approach_goal', latched)
        self.preview_path_pub = self.create_publisher(Path, '/semantic/approach_path', latched)
        self.arm_pub = self.create_publisher(Bool, '/mpc/arm', 10)
        self.planner = self.create_client(
            PlanObjectApproach, '/global_planner/plan_object_approach'
        )
        self.cancel_planner = self.create_client(Trigger, '/global_planner/cancel_goal')
        self.action = ActionServer(
            self,
            NavigateToObject,
            '/semantic/navigate_to_object',
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.create_timer(0.1, self._tick)

    def _objects(self, msg):
        if msg.map_id == self.p['map_id'] and msg.header.frame_id == self.p['map_frame']:
            if self.mission and self.objects and msg.revision != self.objects.revision:
                # Normal optimization shifts every live anchor by millimeters; only a
                # target change beyond tolerance invalidates the dispatched approach.
                self._reprojected(msg)
            # _finish ignores a mission that is already stopping.
            if self.mission and self.p['require_pose_graph'] and not msg.pose_graph_ready:
                self._finish(False, 'POSE_GRAPH_UNAVAILABLE', 'Fresh optimized graph is required')
            self.objects, self.map_received = msg, time.monotonic()
        else:
            self.objects = None

    def _armed(self, msg):
        self.armed = msg.data

    def _map(self, msg):
        self.traversable, self.traversable_received = msg, time.monotonic()

    def _odom(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def _pose(self, msg):
        cov = np.asarray(msg.pose.covariance).reshape(6, 6)
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(msg.header.stamp)
        self.pose_good = (
            msg.header.frame_id == self.p['map_frame']
            and -0.1 <= age <= 3.0
            and np.isfinite(cov).all()
            and min(cov.diagonal()) >= 0
            and max(cov[0, 0], cov[1, 1], cov[5, 5]) <= 0.5
        )
        self.pose_received = time.monotonic()

    def _initial_pose(self, _msg):
        self.pose_good = False
        if self.mission:
            self._finish(False, 'LOCALIZATION_RESET', 'Localization was reset')

    def _external_goal(self, msg):
        m = self.mission
        if (
            m
            and m['dispatched']
            and m['state'] != 'STOPPING'
            and msg.header.stamp != m['goal'].header.stamp
        ):
            self._finish(False, 'INTERRUPTED', 'Another goal replaced the semantic mission')

    def _path(self, msg):
        m = self.mission
        if not m or not m['dispatched'] or m['state'] == 'STOPPING':
            return
        if (
            stamp_seconds(msg.header.stamp) < m['dispatch_stamp']
            or msg.header.frame_id != self.p['map_frame']
        ):
            return
        if not msg.poses:
            m['path_valid'] = False
            m['path_wait_until'] = time.monotonic() + 3.0
            return
        end, requested = msg.poses[-1].pose.position, m['goal'].pose.position
        if math.hypot(end.x - requested.x, end.y - requested.y) > 0.02:
            self._finish(
                False, 'PATH_CHANGED', 'Planner resolved a different endpoint; retry approach'
            )
            return
        m['path_valid'] = True

    def _accept(self, request):
        if (
            self.busy or self.shutting_down
            or not any((request.object_id, request.class_name, request.alias))
            or not math.isfinite(request.timeout)
            or request.timeout < 0
        ):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def _execute(self, handle):
        timeout = handle.request.timeout or self.p['mission_timeout']
        self.mission = {
            'handle': handle,
            'future': Future(),
            'state': 'SELECTING',
            'deadline': time.monotonic() + timeout,
            'object': None,
            'goal': PoseStamped(),
            'plan_future': None,
            'dispatched': False,
            'search_index': 0,
            'replans': 0,
            'searching': False,
            'path_valid': False,
            'path_wait_until': 0.0,
            'arrival_stamp': 0.0,
            'result': None,
        }
        future = self.mission['future']
        return await future

    def _robot_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.p['map_frame'], self.p['base_frame'], Time()
        )
        age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        if not -0.1 <= age <= 0.5:
            raise ValueError('Robot TF is stale')
        t = transform.transform.translation
        return (t.x, t.y, yaw(transform.transform.rotation))

    def _query(self):
        request = self.mission['handle'].request
        if self.mission['object'] is not None:
            return select_object(
                self.objects.objects, object_id=self.mission['object'].object_id)
        return select_object(
            self.objects.objects,
            request.object_id,
            request.class_name,
            request.alias,
            request.near_object_id,
        )

    @staticmethod
    def _geometry_change(current, planned):
        """Largest planar change of the target's position, box center or extent [m]."""
        return max(
            abs(current.box_center.x - planned.box_center.x),
            abs(current.box_center.y - planned.box_center.y),
            abs(current.extent.x - planned.extent.x),
            abs(current.extent.y - planned.extent.y),
            math.hypot(current.position.x - planned.position.x,
                       current.position.y - planned.position.y),
        )

    def _reprojected(self, msg):
        """Judge a graph correction by its effect on this mission's target only."""
        m = self.mission
        if m['state'] == 'STOPPING' or not m['object'] or m['searching'] or not m['dispatched']:
            # Before dispatch, PLANNING revalidates against the latest map anyway.
            return
        current = next((o for o in msg.objects if o.object_id == m['object'].object_id), None)
        if (current is None
                or self._geometry_change(current, m['object'])
                > self.p['target_change_tolerance']):
            self._finish(False, 'MAP_REPROJECTED',
                         'Graph correction moved the target; replan with the new revision')

    def _target_unchanged(self, planned, ros_now, max_replans=3):
        """Revalidate the planned target before its approach is previewed or dispatched."""
        m = self.mission
        current = next(
            (o for o in self.objects.objects if o.object_id == planned.object_id), None)
        if current is None or current.status != 1:
            self._finish(False, 'TARGET_CHANGED', 'Target disappeared or moved while planning')
            return False
        cov = np.asarray(current.covariance).reshape(3, 3)
        moved = self._geometry_change(current, planned)
        if (moved <= self.p['target_change_tolerance'] and np.isfinite(cov).all()
                and max(cov[0, 0], cov[1, 1]) <= self.p['max_target_variance']
                and 0 <= ros_now - stamp_seconds(current.last_seen) <= self.p['object_freshness']):
            return True
        # Geometry, uncertainty or freshness changed: SELECTING re-checks and replans.
        m['replans'] += 1
        if m['replans'] > max_replans:
            self._finish(False, 'TARGET_UNSTABLE', 'Target kept changing while planning')
        else:
            m['state'] = 'SELECTING'
        return False

    def _request_plan(self, target, candidates, searching=False):
        if not self.planner.service_is_ready():
            self._finish(False, 'PLANNER_UNAVAILABLE', 'Approach planning service is unavailable')
            return
        req = PlanObjectApproach.Request()
        req.target_center, req.target_extent = target.box_center, target.extent
        req.candidates = [
            pose_message(*p, self.p['map_frame'], self.get_clock().now().to_msg())
            for p in candidates
        ]
        req.min_standoff, req.max_standoff = float(self.p['min_standoff']), float(
            self.p['max_standoff']
        )
        req.max_snap_distance, req.require_fresh_map = float(self.p['max_snap_distance']), True
        m = self.mission
        m['plan_future'], m['state'], m['searching'] = (
            self.planner.call_async(req),
            'PLANNING',
            searching,
        )
        m['plan_deadline'] = time.monotonic() + 5.0

    def _search(self, target=None):
        m = self.mission
        if m['handle'].request.preview_only:
            self._finish(False, 'NOT_OBSERVED', 'Preview cannot perform an observation search')
            return
        if not m['handle'].request.allow_search:
            self._finish(
                False, 'NOT_OBSERVED', 'Object needs a fresh observation; enable bounded search'
            )
            return
        index = m['search_index']
        if index >= min(len(self.views), self.p['max_search_views']):
            self._finish(False, 'SEARCH_EXHAUSTED', 'No configured search viewpoint remains')
            return
        m['search_index'] += 1
        x, y, angle = self.views[index]
        # A search pose is planned by the same approach service. A small
        # virtual target 0.8 m ahead validates the viewing corridor as well.
        from amr_interfaces.msg import SemanticObject

        view_target = SemanticObject()
        view_target.box_center.x = x + self.p['standoff'] * math.cos(angle)
        view_target.box_center.y = y + self.p['standoff'] * math.sin(angle)
        self._request_plan(view_target, [(x, y, angle)], searching=True)

    def _dispatch(self, goal):
        m = self.mission
        if (
            not self.p['enable_goal_dispatch']
            or not self.armed
            or self.count_publishers('/mpc/armed') < 1
        ):
            self._finish(
                False, 'DISPATCH_DISABLED', 'Enable dispatch and arm the existing gate manually'
            )
            return
        if not self.cancel_planner.service_is_ready():
            self._finish(False, 'CANCEL_UNAVAILABLE', 'Planner cancellation service is required')
            return
        goal.header.stamp = self.get_clock().now().to_msg()
        m['goal'], m['dispatched'], m['state'] = goal, True, 'NAVIGATING'
        m['dispatch_stamp'], m['path_wait_until'] = (
            stamp_seconds(goal.header.stamp),
            time.monotonic() + 3,
        )
        m['path_valid'] = False
        self.goal_pub.publish(goal)

    def _finish(self, success, status, message):
        m = self.mission
        if not m or m['state'] == 'STOPPING':
            return
        # A plan reply can never be used after this point; release it from the client.
        self._drop(self.planner, m['plan_future'])
        m['plan_future'] = None
        result = NavigateToObject.Result(success=success, status=status, message=message)
        result.object_id = m['object'].object_id if m['object'] else ''
        result.goal = m['goal']
        m['result'], m['state'] = result, 'STOPPING'
        if m['dispatched']:
            self.arm_pub.publish(Bool(data=False))
            m['stop_future'] = (
                self.cancel_planner.call_async(Trigger.Request())
                if self.cancel_planner.service_is_ready()
                else None
            )
            m['stop_deadline'] = time.monotonic() + 2.0
        else:
            self._complete()

    @staticmethod
    def _drop(client, future):
        """Forget an unanswered request so a silent server cannot accumulate futures."""
        if future is not None and not future.done():
            client.remove_pending_request(future)
            future.cancel()

    def _complete(self):
        m = self.mission
        if m['handle'].is_cancel_requested:
            m['handle'].canceled()
        elif m['result'].success:
            m['handle'].succeed()
        else:
            m['handle'].abort()
        m['future'].set_result(m['result'])
        self.mission, self.busy = None, False

    def _tick(self):
        m = self.mission
        if not m:
            return
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        if m['state'] == 'STOPPING':
            self.arm_pub.publish(Bool(data=False))
            future = m['stop_future']
            if future and future.done():
                if future.exception() or not future.result().success:
                    m['result'].success, m['result'].status = False, 'STOP_FAILED'
                self._complete()
            elif now > m['stop_deadline']:
                self._drop(self.cancel_planner, future)
                m['result'].success, m['result'].status = False, 'STOP_FAILED'
                m['result'].message += '; command gate disarmed, planner clear unconfirmed'
                self._complete()
            return
        if m['handle'].is_cancel_requested:
            self._finish(False, 'CANCELLED', 'Mission cancelled')
            return
        if now > m['deadline']:
            self._finish(False, 'TIMEOUT', 'Mission timeout')
            return
        if m['dispatched'] and (not self.armed or self.count_publishers('/mpc/armed') < 1):
            self._finish(False, 'DISARMED', 'Existing command gate is disarmed or unavailable')
            return
        if (
            self.objects is None
            or now - self.map_received > 3
            or (self.p['require_pose_graph'] and not self.objects.pose_graph_ready)
            or (
                self.p['require_localization']
                and (not self.pose_good or now - self.pose_received > 3)
            )
        ):
            self._finish(
                False, 'LOCALIZATION_UNAVAILABLE', 'Fresh object map and localization are required'
            )
            return
        if m['dispatched']:
            oldest = stamp_seconds(self.traversable.sensor_watermark) if self.traversable else 0
            if (
                not self.traversable
                or self.traversable.header.frame_id != self.p['map_frame']
                or now - self.traversable_received > 2
                or not 0 <= ros_now - oldest <= 2
            ):
                self._finish(False, 'MAP_STALE', 'Traversable sensor data is stale')
                return
        feedback = NavigateToObject.Feedback(state=m['state'], goal=m['goal'])
        feedback.object_id = m['object'].object_id if m['object'] else ''
        m['handle'].publish_feedback(feedback)
        try:
            if m['state'] == 'SELECTING':
                try:
                    obj = self._query()
                except LookupError:
                    self._search()
                    return
                m['object'] = obj
                cov = np.asarray(obj.covariance).reshape(3, 3)
                if (not np.isfinite(cov).all() or min(cov.diagonal()) < 0
                        or max(cov[0, 0], cov[1, 1]) > self.p['max_target_variance']):
                    self._finish(
                        False, 'TARGET_UNCERTAIN', 'Object position covariance is too large')
                    return
                age = ros_now - stamp_seconds(obj.last_seen)
                if not 0 <= age <= self.p['object_freshness'] or obj.status != 1:
                    self._search(obj)
                    return
                candidates = approach_candidates(
                    [obj.box_center.x, obj.box_center.y, obj.box_center.z],
                    [obj.extent.x, obj.extent.y, obj.extent.z],
                    self.p['standoff'],
                )
                self._request_plan(obj, candidates)
            elif m['state'] == 'PLANNING':
                future = m['plan_future']
                if now > m['plan_deadline']:
                    self._finish(False, 'PLAN_TIMEOUT', 'Approach planning timed out')
                elif future.done():
                    if future.exception() or not future.result().success:
                        reason = (
                            str(future.exception())
                            if future.exception()
                            else future.result().message
                        )
                        self._finish(False, 'UNREACHABLE', reason)
                        return
                    result = future.result()
                    if m['object'] and not m['searching'] and not self._target_unchanged(
                            m['object'], ros_now):
                        return
                    self.preview_pub.publish(result.goal)
                    self.preview_path_pub.publish(result.path)
                    m['goal'] = result.goal
                    if m['handle'].request.preview_only:
                        self._finish(
                            True, 'PREVIEWED', 'Reachable approach preview; no motion dispatched'
                        )
                    else:
                        self._dispatch(result.goal)
            elif m['state'] == 'NAVIGATING':
                pose = self._robot_pose()
                if not m['path_valid']:
                    if now > m['path_wait_until']:
                        self._finish(False, 'NO_PATH', 'Planner did not provide a valid path')
                    return
                odom_age = (
                    ros_now - stamp_seconds(self.odom.header.stamp) if self.odom else math.inf
                )
                if not self.odom or now - self.odom_received > 0.5 or not -0.1 <= odom_age <= 0.5:
                    self._finish(False, 'ODOMETRY_STALE', 'Fresh odometry is required')
                    return
                if m['object'] and not m['searching']:
                    current = next(
                        (o for o in self.objects.objects if o.object_id == m['object'].object_id),
                        None,
                    )
                    if (
                        current is None
                        or current.status != 1
                        or math.hypot(
                            current.position.x - m['object'].position.x,
                            current.position.y - m['object'].position.y,
                        )
                        > 0.4
                    ):
                        self._finish(
                            False, 'TARGET_CHANGED', 'Target disappeared or moved; select again'
                        )
                        return
                goal = m['goal'].pose
                twist = self.odom.twist.twist
                if at_approach(
                    pose,
                    (goal.position.x, goal.position.y, yaw(goal.orientation)),
                    math.hypot(twist.linear.x, twist.linear.y),
                    twist.angular.z,
                ):
                    m['arrival_stamp'], m['verify_until'], m['state'] = (
                        ros_now,
                        now + self.p['reverify_timeout'],
                        'VERIFYING',
                    )
            elif m['state'] == 'VERIFYING':
                # Require a later image at the destination, never a latched map
                # observation or merely controller distance-to-goal.
                pose = self._robot_pose()
                goal = m['goal'].pose
                odom_age = (
                    ros_now - stamp_seconds(self.odom.header.stamp) if self.odom else math.inf
                )
                if not self.odom or not -0.1 <= odom_age <= 0.5:
                    self._finish(
                        False, 'ODOMETRY_STALE', 'Fresh odometry is required for re-observation'
                    )
                    return
                twist = self.odom.twist.twist
                if not at_approach(
                    pose,
                    (goal.position.x, goal.position.y, yaw(goal.orientation)),
                    math.hypot(twist.linear.x, twist.linear.y),
                    twist.angular.z,
                ):
                    self._finish(
                        False, 'ARRIVAL_LOST', 'Base left the approach during re-observation'
                    )
                    return
                obj = self._query()
                fresh = stamp_seconds(obj.last_seen) > m['arrival_stamp'] and obj.status == 1
                same = (
                    m['searching']
                    or not m['object']
                    or (
                        obj.object_id == m['object'].object_id
                        and math.hypot(
                            obj.position.x - m['object'].position.x,
                            obj.position.y - m['object'].position.y,
                        )
                        <= 0.4
                    )
                )
                if fresh and same:
                    if m['searching']:
                        m['object'], m['state'] = obj, 'SELECTING'
                    else:
                        self._finish(
                            True, 'SUCCEEDED', 'Stopped at approach and re-observed the target'
                        )
                elif now > m['verify_until']:
                    if m['searching']:
                        self._search()
                    elif m['handle'].request.allow_search:
                        self._search(m['object'])
                    else:
                        self._finish(
                            False, 'REVERIFY_FAILED', 'Target was not re-observed at destination'
                        )
        except LookupError:
            if m['state'] == 'VERIFYING' and now <= m['verify_until']:
                return
            self._search()
        except (ValueError, tf2_ros.TransformException) as exc:
            self._finish(False, 'INVALID_STATE', str(exc))

    def destroy_node(self):
        if self.mission and self.mission['dispatched']:
            self.arm_pub.publish(Bool(data=False))
            if self.cancel_planner.service_is_ready():
                self.cancel_planner.call_async(Trigger.Request())
        self.action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SemanticNavigationNode()
    requested = False
    drained_since = None

    def request_shutdown(_signal, _frame):
        nonlocal requested
        requested = True

    old_handlers = {number: signal.signal(number, request_shutdown)
                    for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if requested:
                node.shutting_down = True
                if node.mission:
                    node._finish(False, 'SHUTDOWN', 'Semantic navigation is stopping')
                else:
                    if drained_since is None:
                        drained_since = time.monotonic()
                    elif time.monotonic() - drained_since >= 0.2:
                        break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for number, handler in old_handlers.items():
            signal.signal(number, handler)
