#!/usr/bin/env python3
"""Plan a Theta* path from RViz 2D Pose Estimate and 2D Goal Pose."""

import math
import time

from amr_interfaces.msg import TraversableMap
from amr_interfaces.srv import PlanObjectApproach
from std_srvs.srv import Trigger

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

from global_planner.common.ros_map_loader import (
    find_ros_map_yaml,
    load_ros_map,
)
from global_planner.common.traversable_map import (
    geometry_matches,
    inflate_layer,
    traversable_obstacle_mask,
)
from global_planner.theta_star.costmap import Costmap
from global_planner.theta_star.theta_star import theta_star
from global_planner.theta_star.utils import is_line_of_sight_clear

from nav_msgs.msg import Path

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

import tf2_ros

from visualization_msgs.msg import Marker


def densify_grid_path(path, interval):
    """Interpolate a grid path without changing its straight-line segments."""
    if not path:
        return []

    dense_path = []
    interval = max(float(interval), 1.0e-6)
    for start, end in zip(path[:-1], path[1:]):
        start = np.asarray(start, dtype=np.float64)
        end = np.asarray(end, dtype=np.float64)
        distance = float(np.linalg.norm(end - start))
        segment_count = max(1, int(math.ceil(distance / interval)))
        for step in range(segment_count):
            point = start + (end - start) * (step / segment_count)
            dense_path.append((float(point[0]), float(point[1])))
    dense_path.append((float(path[-1][0]), float(path[-1][1])))
    return dense_path


def nearest_free_cell(position, costmap, maximum_radius_cells):
    """Return the closest collision-free cell inside a bounded search disk."""
    x, y = (int(position[0]), int(position[1]))
    if costmap.is_out_of_bounds(x, y):
        return None
    if not costmap.is_collision(x, y):
        return x, y

    radius = max(0, int(maximum_radius_cells))
    if radius == 0:
        return None
    x_min = max(0, x - radius)
    x_max = min(costmap.width - 1, x + radius)
    y_min = max(0, y - radius)
    y_max = min(costmap.height - 1, y + radius)
    yy, xx = np.mgrid[y_min:y_max + 1, x_min:x_max + 1]
    squared = (xx - x) ** 2 + (yy - y) ** 2
    eligible = squared <= radius * radius
    eligible &= costmap.inflated_map[yy, xx] != 1
    if not np.any(eligible):
        return None
    distances = np.where(eligible, squared, np.inf)
    index = int(np.argmin(distances))
    return int(xx.flat[index]), int(yy.flat[index])


def remaining_path_has_collision(path, current_world, ros_map, costmap):
    """Check every grid cell along the untraversed part of a world path."""
    if not path:
        return False
    points = np.asarray(path, dtype=np.float64)
    current = np.asarray(current_world, dtype=np.float64)
    nearest = int(np.argmin(np.linalg.norm(points - current[None, :], axis=1)))
    # Only validate segments which are actually part of the published path.
    # A straight connector from the robot to the nearest waypoint is not part
    # of that path and can cross a wall when the local controller is slightly
    # off-track, causing a false replan on every validation cycle.
    remaining = [tuple(point) for point in points[nearest:]]
    grid_points = [ros_map.world_to_grid(point) for point in remaining]
    if len(grid_points) == 1:
        return costmap.is_collision(*grid_points[0])
    return any(
        not is_line_of_sight_clear(start, end, costmap)
        for start, end in zip(grid_points[:-1], grid_points[1:])
    )


class RvizThetaStarNode(Node):
    """Plan and periodically validate a Theta* path with a traversable map."""

    def __init__(self):
        """Initialize map, subscriptions, publishers, and validation timer."""
        super().__init__('rviz_theta_star_node')

        self.declare_parameter('map', '')
        self.declare_parameter('maps_dir', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter(
            'waypoint_marker_topic',
            '/global_path_waypoints',
        )
        self.declare_parameter('waypoint_marker_size', 0.06)
        self.declare_parameter('path_validation_frequency', 1.0)
        self.declare_parameter('robot_size', 0.30)
        self.declare_parameter('endpoint_snap_radius', 0.50)
        self.declare_parameter('waypoint_interval', 0.10)
        self.declare_parameter('require_initial_pose', True)
        self.declare_parameter('initial_pose_grace_period', 2.0)
        self.declare_parameter('tf_timeout', 0.20)
        self.declare_parameter('transform_max_age', 0.5)
        self.declare_parameter('use_traversable_map', True)
        self.declare_parameter('traversable_map_topic', '/traversable_map')
        self.declare_parameter('traversable_map_timeout', 2.0)
        self.declare_parameter('traversable_unknown_is_obstacle', False)
        self.declare_parameter('verified_goal_ttl', 10.0)

        self.map_name = str(self.get_parameter('map').value)
        maps_dir = str(self.get_parameter('maps_dir').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.require_initial_pose = bool(
            self.get_parameter('require_initial_pose').value
        )
        self.initial_pose_grace_period = float(
            self.get_parameter('initial_pose_grace_period').value
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.transform_max_age = float(self.get_parameter('transform_max_age').value)
        if not math.isfinite(self.transform_max_age) or self.transform_max_age <= 0:
            raise ValueError('transform_max_age must be finite and positive')
        self.path_validation_frequency = float(
            self.get_parameter('path_validation_frequency').value
        )
        self.use_traversable_map = bool(
            self.get_parameter('use_traversable_map').value
        )
        self.traversable_map_timeout = float(
            self.get_parameter('traversable_map_timeout').value
        )
        # Global planning may cross unverified space; the local planner may
        # not.  Design document section 4 splits the policy that way, so this
        # node defaults to treating unknown cells as passable.
        self.traversable_unknown_is_obstacle = bool(
            self.get_parameter('traversable_unknown_is_obstacle').value
        )

        self.verified_goal_ttl = float(self.get_parameter('verified_goal_ttl').value)

        if self.traversable_map_timeout <= 0.0:
            raise ValueError('traversable_map_timeout must be positive')
        if self.path_validation_frequency <= 0.0:
            raise ValueError('path_validation_frequency must be positive')
        if not self.map_name:
            raise ValueError('The map parameter is required.')
        if not maps_dir:
            raise ValueError('The maps_dir parameter is required.')

        map_yaml = find_ros_map_yaml(maps_dir, self.map_name)
        self.ros_map = load_ros_map(map_yaml)
        self.robot_size = float(self.get_parameter('robot_size').value)
        self.endpoint_snap_radius = float(
            self.get_parameter('endpoint_snap_radius').value
        )
        if self.endpoint_snap_radius < 0.0:
            raise ValueError('endpoint_snap_radius must be non-negative')
        robot_size_cells = max(
            1,
            int(math.ceil(self.robot_size / self.ros_map.resolution)),
        )
        self.costmap = Costmap(
            self.ros_map.grid_map,
            robot_size=robot_size_cells,
        )
        self.waypoint_interval = float(
            self.get_parameter('waypoint_interval').value
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rviz_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        initial_pose_topic = str(
            self.get_parameter('initial_pose_topic').value
        )
        goal_topic = str(self.get_parameter('goal_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.waypoint_marker_topic = str(
            self.get_parameter('waypoint_marker_topic').value
        )
        self.waypoint_marker_size = float(
            self.get_parameter('waypoint_marker_size').value
        )
        self.path_publisher = self.create_publisher(
            Path,
            self.path_topic,
            path_qos,
        )
        self.waypoint_marker_publisher = self.create_publisher(
            Marker,
            self.waypoint_marker_topic,
            path_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            initial_pose_topic,
            self._on_initial_pose,
            rviz_qos,
        )
        self.create_subscription(
            PoseStamped,
            goal_topic,
            self._on_goal,
            rviz_qos,
        )
        self.initial_pose = None
        self.initial_pose_received_at = 0.0
        self.current_goal = None
        self.current_goal_yaw = 0.0
        # Goals validated by plan_object_approach keep its unknown-is-obstacle
        # policy through dispatch and every replan.
        self.current_goal_verified = False
        self._verified_goal = None
        self.current_path = []
        self._replan_pending = False
        self._traversable_mask = None
        self._verified_mask = None
        self._traversable_message = None
        self._traversable_received_at = 0.0
        if self.use_traversable_map:
            self.create_subscription(
                TraversableMap,
                str(self.get_parameter('traversable_map_topic').value),
                self._on_traversable_map,
                path_qos,
            )
            self.create_timer(
                1.0 / self.path_validation_frequency,
                self._validate_current_path,
            )
        self._publish_empty_path()
        self.create_service(PlanObjectApproach, '/global_planner/plan_object_approach',
                            self._plan_object_approach)
        self.create_service(Trigger, '/global_planner/cancel_goal', self._cancel_goal)
        traversable_status = (
            'traversable map disabled'
            if not self.use_traversable_map
            else (
                'traversable map on '
                f'{self.get_parameter("traversable_map_topic").value} '
                '(unknown = '
                f'{"blocked" if self.traversable_unknown_is_obstacle else "passable"})'
                f'; path validation at {self.path_validation_frequency:.1f} Hz'
            )
        )
        self.get_logger().info(
            f'Loaded {map_yaml}; waiting for RViz 2D Pose Estimate on '
            f'{initial_pose_topic}, then 2D Goal Pose on {goal_topic}; '
            f'{traversable_status}.'
        )

    def _on_initial_pose(self, message):
        try:
            self.initial_pose = self._point_in_map(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.header.frame_id or self.map_frame,
            )
        except Exception as exc:
            self.initial_pose = None
            self.get_logger().error(f'Cannot transform initial pose: {exc}')
            return

        self.initial_pose_received_at = time.monotonic()
        # Re-localizing must cancel any path that could still be driving MPC.
        self.current_goal = None
        self.current_goal_verified, self._verified_goal = False, None
        self.current_path = []
        self._replan_pending = False
        self._publish_empty_path()

        self.get_logger().info(
            f'Initial pose received at ({self.initial_pose[0]:.3f}, '
            f'{self.initial_pose[1]:.3f}); previous global path cleared.'
        )

    def _cancel_goal(self, _request, response):
        """Clear the retained goal as well as the path so replanning cannot resume it."""
        self.current_goal, self.current_path, self._replan_pending = None, [], False
        self.current_goal_verified, self._verified_goal = False, None
        self._publish_empty_path()
        response.success, response.message = True, 'Goal and path cleared'
        return response

    def _plan_object_approach(self, request, response):
        """Validate candidate paths and snapped endpoints without publishing a goal."""
        try:
            if self.require_initial_pose and self.initial_pose is None:
                raise ValueError('Initialize localization before object navigation')
            center = np.array([request.target_center.x, request.target_center.y])
            extent = np.array([request.target_extent.x, request.target_extent.y])
            limits = [request.min_standoff, request.max_standoff, request.max_snap_distance]
            if (not np.isfinite(np.r_[center, extent, limits]).all() or np.any(extent < 0)
                    or np.any(extent > 10) or not 0 < limits[0] <= limits[1] <= 5
                    or not 0 <= limits[2] <= self.endpoint_snap_radius
                    or not 1 <= len(request.candidates) <= 32):
                raise ValueError('Invalid object approach geometry or candidate count')
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            transform_stamp = Time.from_msg(transform.header.stamp)
            age = (self.get_clock().now() - transform_stamp).nanoseconds / 1e9
            if not -0.1 <= age <= 0.5:
                raise ValueError('Robot localization TF is stale')
            start = (transform.transform.translation.x, transform.transform.translation.y)
            costmap = self._planning_costmap(require_fresh_traversable=request.require_fresh_map)
            if costmap is None:
                raise ValueError('Fresh traversable map is required')
            if request.require_fresh_map:
                msg = self._traversable_message
                if msg is None or msg.header.frame_id != self.map_frame:
                    raise ValueError('Traversable map frame mismatch')
                oldest = Time.from_msg(msg.sensor_watermark)
                oldest_age = (self.get_clock().now() - oldest).nanoseconds / 1e9
                if (oldest.nanoseconds == 0
                        or not -0.1 <= oldest_age <= self.traversable_map_timeout):
                    raise ValueError('Traversable sensor observations are stale')
                q = msg.grid.info.origin.orientation
                if abs(q.x) + abs(q.y) + abs(q.z) > 1e-5 or abs(abs(q.w) - 1) > 1e-5:
                    raise ValueError('Rotated occupancy grid is unsupported')
                # Semantic approaches use verified space, even when RViz global
                # planning is configured to permit unknown cells.
                raw = traversable_obstacle_mask(msg.grid, unknown_is_obstacle=True)
                mask = inflate_layer(raw, self.robot_size, float(msg.inflation_radius),
                                     self.ros_map.resolution)
                costmap = Costmap(np.asarray(mask, dtype=np.uint8), robot_size=1)
                view_map = Costmap(np.asarray(raw, dtype=np.uint8), robot_size=1)
            else:
                view_map = costmap
            start_grid = self.ros_map.world_to_grid(start)
            if costmap.is_collision(*start_grid):
                raise ValueError(
                    'Robot start is not verified footprint-free; approach cannot snap it')
            best = None
            for candidate in request.candidates:
                if candidate.header.frame_id != self.map_frame:
                    continue
                xy = np.array([candidate.pose.position.x, candidate.pose.position.y])
                if not np.isfinite(xy).all():
                    continue
                try:
                    path = self._plan(start, tuple(xy), costmap)
                except (RuntimeError, ValueError):
                    continue
                resolved = np.array(path[-1])
                if np.linalg.norm(resolved - xy) > request.max_snap_distance + 1e-6:
                    continue
                offset = resolved - center
                surface_distance = np.linalg.norm(np.maximum(np.abs(offset) - extent/2, 0))
                if not request.min_standoff <= surface_distance <= request.max_standoff:
                    continue
                # Visibility stops outside the observed box, since the object
                # itself is expected to occupy the end of a viewing ray.
                ratio = min((extent[i]/2) / abs(offset[i]) for i in range(2)
                            if abs(offset[i]) > 1e-9)
                near_face = center + offset * min(1.0, ratio)
                ray = resolved - near_face
                if np.linalg.norm(ray) <= 2*self.ros_map.resolution:
                    continue
                view_end = near_face + ray / np.linalg.norm(ray) * (2*self.ros_map.resolution)
                if not is_line_of_sight_clear(
                    self.ros_map.world_to_grid(resolved),
                    self.ros_map.world_to_grid(view_end), view_map,
                ):
                    continue
                length = sum(math.dist(a, b) for a, b in zip(path[:-1], path[1:]))
                if best is None or length < best[0]:
                    best = (length, resolved, path)
            if best is None:
                raise ValueError('No reachable approach with standoff and visibility')
            _, resolved, path = best
            response.goal.header.frame_id = self.map_frame
            response.goal.header.stamp = self.get_clock().now().to_msg()
            response.goal.pose.position.x, response.goal.pose.position.y = map(float, resolved)
            yaw = math.atan2(center[1] - resolved[1], center[0] - resolved[0])
            response.goal.pose.orientation.z = math.sin(yaw/2)
            response.goal.pose.orientation.w = math.cos(yaw/2)
            response.path.header = response.goal.header
            for xy in path:
                pose = PoseStamped()
                pose.header = response.goal.header
                pose.pose.position.x, pose.pose.position.y = map(float, xy)
                pose.pose.orientation.w = 1.0
                response.path.poses.append(pose)
            # A later /goal_pose at exactly this endpoint inherits the verified policy.
            self._verified_goal = (time.monotonic(), tuple(map(float, resolved)))
            response.success, response.message = True, 'Validated object approach'
        except (RuntimeError, ValueError, tf2_ros.TransformException) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _on_goal(self, message):
        # Stop downstream path following while a replacement path is computed.
        self.current_goal = None
        self.current_goal_verified = False
        self.current_path = []
        self._replan_pending = False
        self._publish_empty_path()

        if self.require_initial_pose and self.initial_pose is None:
            self.get_logger().warning(
                'Set RViz 2D Pose Estimate before sending 2D Goal Pose.'
            )
            return

        try:
            goal_world = self._point_in_map(
                message.pose.position.x,
                message.pose.position.y,
                message.header.frame_id or self.map_frame,
            )
            verified = self._consume_verified_goal(goal_world)
            start_world, start_source = self._planning_start()
            planning_costmap = self._planning_costmap(
                require_fresh_traversable=verified, verified=verified)
            if planning_costmap is None:
                raise ValueError('Verified object goal requires a fresh traversable map')
            world_path = self._plan(start_world, goal_world, planning_costmap)
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            self.get_logger().error(f'Theta* planning rejected: {exc}')
            return
        except Exception as exc:
            self.get_logger().error(f'Theta* planning failed: {exc}')
            return

        goal_yaw = self._goal_yaw_in_map(message)
        self.current_goal = tuple(goal_world)
        self.current_goal_yaw = goal_yaw
        self.current_goal_verified = verified
        self._publish_path(world_path, goal_yaw)
        self.get_logger().info(
            f'Theta* path published: {len(world_path)} poses, '
            f'start={start_world} ({start_source}), goal={goal_world}'
            f'{" (verified: unknown blocked)" if verified else ""}.'
        )

    def _consume_verified_goal(self, goal_world):
        """Match a goal to the last approach this node validated, at most once."""
        record, self._verified_goal = self._verified_goal, None
        if record is None:
            return False
        validated_at, xy = record
        return (time.monotonic() - validated_at <= self.verified_goal_ttl
                and math.dist(xy, goal_world) <= 1e-6)

    def _planning_start(self):
        initial_pose_is_recent = (
            self.initial_pose is not None
            and time.monotonic() - self.initial_pose_received_at
            < self.initial_pose_grace_period
        )
        if initial_pose_is_recent:
            return self.initial_pose, 'recent /initialpose'

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
            stamp = Time.from_msg(transform.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or not -0.1 <= age <= self.transform_max_age:
                raise ValueError('Robot localization transform is stale')
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            ), 'TF'
        except Exception:
            raise RuntimeError(
                f'TF {self.map_frame}->{self.base_frame} is unavailable'
            )

    def _plan(self, start_world, goal_world, costmap=None):
        planning_costmap = self.costmap if costmap is None else costmap
        requested_start_grid = self.ros_map.world_to_grid(start_world)
        requested_goal_grid = self.ros_map.world_to_grid(goal_world)
        start_grid, start_snapped = self._resolve_endpoint(
            'start',
            requested_start_grid,
            planning_costmap,
        )
        goal_grid, goal_snapped = self._resolve_endpoint(
            'goal',
            requested_goal_grid,
            planning_costmap,
        )

        grid_path = theta_star(start_grid, goal_grid, planning_costmap)
        if not grid_path:
            raise RuntimeError('no collision-free path was found')

        interval_cells = self.waypoint_interval / self.ros_map.resolution
        dense_grid_path = densify_grid_path(grid_path, interval_cells)
        world_path = [
            self.ros_map.grid_to_world(point) for point in dense_grid_path
        ]

        # Preserve exact RViz coordinates for already-safe endpoints. A
        # snapped endpoint stays at the safe cell center; restoring the
        # requested coordinate would put it back in the inflated obstacle.
        resolved_start = (
            self.ros_map.grid_to_world(start_grid)
            if start_snapped else tuple(start_world)
        )
        resolved_goal = (
            self.ros_map.grid_to_world(goal_grid)
            if goal_snapped else tuple(goal_world)
        )
        if len(world_path) == 1:
            world_path = [resolved_start, resolved_goal]
        else:
            world_path[0] = resolved_start
            world_path[-1] = resolved_goal
        return world_path

    def _resolve_endpoint(self, label, position, costmap):
        if costmap.is_out_of_bounds(*position):
            raise ValueError(f'{label} is outside the map: {position}')
        radius_cells = int(math.ceil(
            self.endpoint_snap_radius / self.ros_map.resolution
        ))
        resolved = nearest_free_cell(position, costmap, radius_cells)
        if resolved is None:
            raise ValueError(
                f'{label} is blocked and no free cell exists within '
                f'{self.endpoint_snap_radius:.2f} m: {position}'
            )
        snapped = resolved != tuple(position)
        if snapped:
            offset = self.ros_map.resolution * math.hypot(
                resolved[0] - position[0],
                resolved[1] - position[1],
            )
            self.get_logger().warning(
                f'{label} {position} is inside an inflated obstacle; '
                f'snapped {offset:.2f} m to safe cell {resolved}.'
            )
        return resolved, snapped

    def _on_traversable_map(self, message):
        """Store the traversable map as an inflated image-order obstacle mask."""
        info = message.grid.info
        if not geometry_matches(info, self.ros_map):
            self.get_logger().warning(
                f'Ignoring traversable map {info.width}x{info.height} @ '
                f'{info.resolution:.4f} origin '
                f'({info.origin.position.x:.4f}, {info.origin.position.y:.4f}); '
                f'this node loaded {self.ros_map.width}x{self.ros_map.height} @ '
                f'{self.ros_map.resolution:.4f} origin '
                f'({self.ros_map.origin[0]:.4f}, {self.ros_map.origin[1]:.4f}).',
                throttle_duration_sec=5.0,
            )
            return

        def inflated(unknown_is_obstacle):
            return inflate_layer(
                traversable_obstacle_mask(message.grid, unknown_is_obstacle),
                self.robot_size,
                float(message.inflation_radius),
                self.ros_map.resolution,
            )

        self._traversable_mask = inflated(self.traversable_unknown_is_obstacle)
        self._verified_mask = (
            self._traversable_mask if self.traversable_unknown_is_obstacle else inflated(True))
        self._traversable_received_at = time.monotonic()
        self._traversable_message = message

    def _fresh_traversable_layer(self, now, verified=False):
        mask = self._verified_mask if verified else self._traversable_mask
        if mask is None:
            return None
        if now - self._traversable_received_at > self.traversable_map_timeout:
            return None
        return mask

    def _planning_costmap(self, require_fresh_traversable=False, verified=False):
        traversable = self._fresh_traversable_layer(time.monotonic(), verified)
        if traversable is None:
            # The static map cannot express the verified-space policy.
            return None if require_fresh_traversable or verified else self.costmap
        return Costmap(
            np.asarray(traversable, dtype=np.uint8),
            robot_size=1,
        )

    def _validate_current_path(self):
        if self.current_goal is None:
            return
        planning_costmap = self._planning_costmap(
            require_fresh_traversable=True,
            verified=self.current_goal_verified,
        )
        if planning_costmap is None:
            return

        try:
            current_world, _ = self._planning_start()
            blocked = remaining_path_has_collision(
                self.current_path,
                current_world,
                self.ros_map,
                planning_costmap,
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Cannot validate global path: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        if not blocked and not self._replan_pending:
            return
        if blocked and not self._replan_pending:
            self.get_logger().warning(
                'Traversable map blocks the remaining global path; '
                'replanning.'
            )
            self._replan_pending = True
        self._attempt_replan(current_world, planning_costmap)

    def _attempt_replan(self, current_world, planning_costmap):
        try:
            world_path = self._plan(
                current_world,
                self.current_goal,
                planning_costmap,
            )
        except (RuntimeError, ValueError) as exc:
            self._publish_empty_path()
            self.get_logger().warning(
                f'Traversable-map replan unavailable; remaining stopped: {exc}',
                throttle_duration_sec=2.0,
            )
            return
        except Exception as exc:
            self._publish_empty_path()
            self.get_logger().error(
                f'Traversable-map replan failed; remaining stopped: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        self._replan_pending = False
        self._publish_path(world_path, self.current_goal_yaw)
        self.get_logger().info(
            f'Theta* replanned around traversable-map obstacle: '
            f'{len(world_path)} poses.'
        )

    def _point_in_map(self, x, y, source_frame):
        if source_frame == self.map_frame:
            return float(x), float(y)
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout),
        ).transform
        transform_yaw = self._quaternion_yaw(transform.rotation)
        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)
        return (
            float(transform.translation.x + cos_yaw * x - sin_yaw * y),
            float(transform.translation.y + sin_yaw * x + cos_yaw * y),
        )

    def _goal_yaw_in_map(self, message):
        yaw = self._quaternion_yaw(message.pose.orientation)
        source_frame = message.header.frame_id or self.map_frame
        if source_frame == self.map_frame:
            return yaw
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
            return yaw + self._quaternion_yaw(transform.rotation)
        except Exception:
            return yaw

    @staticmethod
    def _quaternion_yaw(quaternion):
        return math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    def _publish_empty_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.path_publisher.publish(message)
        self._publish_waypoints([], message.header.stamp)

    def _publish_path(self, world_path, goal_yaw):
        self.current_path = [tuple(point) for point in world_path]
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame

        for index, point in enumerate(world_path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            if index + 1 < len(world_path):
                next_point = world_path[index + 1]
                yaw = math.atan2(
                    next_point[1] - point[1],
                    next_point[0] - point[0],
                )
            else:
                yaw = goal_yaw
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)

        self.path_publisher.publish(message)
        self._publish_waypoints(world_path, message.header.stamp)

    def _publish_waypoints(self, world_path, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = 'global_path_waypoints'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD if world_path else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.waypoint_marker_size
        marker.scale.y = self.waypoint_marker_size
        marker.scale.z = self.waypoint_marker_size
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            Point(x=float(point[0]), y=float(point[1]), z=0.03)
            for point in world_path
        ]
        self.waypoint_marker_publisher.publish(marker)


def main(args=None):
    """Run the RViz Theta* global planner node."""
    rclpy.init(args=args)
    node = RvizThetaStarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
