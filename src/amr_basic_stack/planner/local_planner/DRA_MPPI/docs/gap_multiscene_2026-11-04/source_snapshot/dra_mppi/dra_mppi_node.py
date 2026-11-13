#!/usr/bin/env python3
"""ROS 2 adapter for the dynamic risk-aware MPPI controller."""

import copy
import math
import time

from amr_interfaces.msg import (
    MotionModePrediction,
    MultiModalObjectPredictionArray,
    PlanarReference,
    PlanarReferenceArray,
    TraversableMap,
)
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray, Int32MultiArray, MultiArrayDimension
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from .dynamic_risk import DynamicRiskConfig, PredictionSet
from .geometry import (
    quaternion_to_yaw,
    rotate_covariances,
    transform_points,
    transform_states,
)
from .mppi_controller import MppiConfig, MppiController
from .grid_costmap import DistanceField, GridMetadata
from .cycle_timing import CycleRecorder
from .path_handler import (
    build_terminal_pose_reference,
    GoalCompletionLatch,
    heading_error,
)


LIFECYCLE_CONFIRMED = 1
LIFECYCLE_LOST = 2
MOTION_UNKNOWN = 0
MOTION_STATIC = 1
MOTION_DYNAMIC = 2


class PredictionChainUnavailable(RuntimeError):
    """Raised when dynamic-risk input is missing or stale, not merely empty."""


class TraversabilityUnavailable(RuntimeError):
    """The guide ensemble cannot check the current robot footprint."""


def _prediction_is_usable(predicted, allowed_class_names,
                          max_semantic_age, max_measurement_age,
                          require_confirmed=True, age_offset=0.0,
                          allow_recent_lost=False, lost_grace_time=0.0):
    """Return whether one prediction is safe to pass to dynamic-risk control."""
    allowed = {
        str(name).strip().casefold()
        for name in allowed_class_names
        if str(name).strip()
    }
    class_name = str(predicted.class_name).strip().casefold()
    if allowed and '*' not in allowed and class_name not in allowed:
        return False
    semantic_age = float(predicted.semantic_age)
    measurement_age = float(predicted.measurement_age)
    age_offset = float(age_offset)
    if not all(math.isfinite(value) for value in (
            semantic_age, measurement_age, age_offset)):
        return False
    if age_offset < 0.0:
        return False
    if semantic_age < 0.0 or semantic_age + age_offset > max_semantic_age:
        return False
    if (measurement_age < 0.0
            or measurement_age + age_offset > max_measurement_age):
        return False
    if require_confirmed:
        lifecycle = int(predicted.lifecycle_state)
        recent_lost = (
            allow_recent_lost
            and lifecycle == LIFECYCLE_LOST
            and measurement_age + age_offset <= lost_grace_time
        )
        if lifecycle != LIFECYCLE_CONFIRMED and not recent_lost:
            return False
    return True


def _make_nondynamic_prediction_stationary(predicted, force=False):
    """Collapse a STATIC/UNKNOWN person to its current posterior safety disc."""
    if not force and int(predicted.motion_state) not in (MOTION_UNKNOWN, MOTION_STATIC):
        return predicted

    offsets = np.asarray(predicted.time_offsets, dtype=np.float64)
    if offsets.size == 0 or not np.all(np.isfinite(offsets)):
        return predicted
    # The motion predictor publishes the posterior epoch at offset zero. Use
    # the nearest entry defensively for older recorded messages.
    current_index = int(np.argmin(np.abs(offsets)))
    candidates = []
    for mode in predicted.modes:
        if current_index >= len(mode.points):
            continue
        point = mode.points[current_index]
        covariance = np.asarray(point.covariance, dtype=np.float64)
        values = (float(point.weight), float(point.mean_x), float(point.mean_y))
        if (covariance.size != 4
                or not np.all(np.isfinite(covariance))
                or not all(math.isfinite(value) for value in values)):
            continue
        covariance = covariance.reshape(2, 2)
        covariance = 0.5 * (covariance + covariance.T)
        candidates.append((max(0.0, values[0]),
                           np.array(values[1:], dtype=np.float64), covariance))
    if not candidates:
        return predicted

    weights = np.asarray([candidate[0] for candidate in candidates])
    if float(np.sum(weights)) <= 0.0:
        weights = np.full(len(candidates), 1.0 / len(candidates))
    else:
        weights = weights / np.sum(weights)
    anchor = sum(weight * candidate[1]
                 for weight, candidate in zip(weights, candidates))
    anchor_covariance = np.zeros((2, 2), dtype=np.float64)
    for weight, (_, mean, covariance) in zip(weights, candidates):
        residual = mean - anchor
        anchor_covariance += weight * (
            covariance + np.outer(residual, residual))
    covariance_values = [float(value) for value in anchor_covariance.flatten()]
    # One unit-weight stationary Gaussian with ALL IMM modes included at t=0.
    fixed = copy.deepcopy(predicted.modes[0])
    fixed.model_type = MotionModePrediction.MODEL_STATIC
    for point in fixed.points:
        point.mean_x = float(anchor[0])
        point.mean_y = float(anchor[1])
        point.covariance = covariance_values
        point.weight = 1.0
    predicted.modes = [fixed]
    return predicted


def _dynamic_prediction_modes(predicted):
    """Condition on CV/CT at every future time; tiny mass uses a fixed anchor."""
    moving = [mode for mode in predicted.modes if mode.model_type in (0, 1)]
    count = len(predicted.time_offsets)
    if not moving or not count or any(len(mode.points) != count for mode in moving):
        return _make_nondynamic_prediction_stationary(predicted, force=True)
    weights = np.array([[point.weight for point in mode.points] for mode in moving])
    totals = weights.sum(axis=0)
    if (not np.all(np.isfinite(weights)) or np.any(weights < 0)
            or np.any(totals <= 1e-8)):
        return _make_nondynamic_prediction_stationary(predicted, force=True)
    for mode, normalized in zip(moving, weights / totals):
        for point, weight in zip(mode.points, normalized):
            point.weight = float(weight)
    predicted.modes = moving
    return predicted


def _validate_prediction_geometry(predicted):
    """Reject corrupt accepted geometry instead of treating it as an empty scene."""
    offsets = np.asarray(predicted.time_offsets, dtype=float)
    if (not len(offsets) or not np.all(np.isfinite(offsets))
            or np.any(np.diff(offsets) <= 0) or offsets[0] < 0 or not predicted.modes):
        raise ValueError('prediction times/modes are invalid')
    seen = set()
    total = np.zeros(len(offsets))
    for mode in predicted.modes:
        if mode.model_type not in (0, 1, 2) or mode.model_type in seen:
            raise ValueError('unknown or duplicate prediction mode')
        seen.add(mode.model_type)
        if len(mode.points) != len(offsets):
            raise ValueError('prediction point count differs from time count')
        for i, point in enumerate(mode.points):
            values = [point.mean_x, point.mean_y, point.weight]
            P = np.asarray(point.covariance, dtype=float).reshape(2, 2)
            if (not np.all(np.isfinite(values)) or point.weight < 0
                    or not np.all(np.isfinite(P)) or not np.allclose(P, P.T)
                    or np.linalg.eigvalsh(P).min() < -1e-10):
                raise ValueError('prediction mean/weight/covariance is invalid')
            total[i] += point.weight
    if np.any(total <= 0):
        raise ValueError('prediction mode mass must be positive at every time')


def filter_predictions_for_control(message, allowed_class_names,
                                   max_semantic_age,
                                   max_measurement_age,
                                   require_confirmed=True,
                                   stationary_nondynamic=True,
                                   age_offset=0.0,
                                   allow_recent_lost=False,
                                   lost_grace_time=0.0):
    """Filter quality and turn STATIC/UNKNOWN people into fixed safety discs."""
    output = MultiModalObjectPredictionArray()
    output.header = message.header
    for predicted in message.objects:
        if not _prediction_is_usable(
                predicted,
                allowed_class_names,
                max_semantic_age,
                max_measurement_age,
                require_confirmed=require_confirmed,
                age_offset=age_offset,
                allow_recent_lost=allow_recent_lost,
                lost_grace_time=lost_grace_time):
            continue
        accepted = copy.deepcopy(predicted)
        _validate_prediction_geometry(accepted)
        is_lost = int(accepted.lifecycle_state) == LIFECYCLE_LOST
        if is_lost:
            # A coasted trajectory must not be presented as measured motion.
            accepted = _make_nondynamic_prediction_stationary(
                accepted, force=True)
        elif stationary_nondynamic:
            accepted = _make_nondynamic_prediction_stationary(accepted)
        if not is_lost and int(accepted.motion_state) == MOTION_DYNAMIC:
            accepted = _dynamic_prediction_modes(accepted)
        output.objects.append(accepted)
    return output


def filter_predictions_by_class(message, allowed_class_names):
    """Copy only predictions whose normalized class name is allowed."""
    allowed = {
        str(name).strip().casefold()
        for name in allowed_class_names
        if str(name).strip()
    }
    output = MultiModalObjectPredictionArray()
    output.header = message.header
    accept_all = not allowed or '*' in allowed
    for predicted in message.objects:
        class_name = str(predicted.class_name).strip().casefold()
        if accept_all or class_name in allowed:
            output.objects.append(predicted)
    return output


class DraMppiNode(Node):
    """Generate planar pose/velocity references from path, state, and risk."""

    def __init__(self):
        super().__init__('dra_mppi_node')

        self.planning_frame = self._declare('planning_frame', 'odom')
        self.global_path_topic = self._declare('global_path_topic', '/global_path')
        self.goal_pose_topic = self._declare('goal_pose_topic', '/goal_pose')
        self.odometry_topic = self._declare('odometry_topic', '/odometry/filtered')
        self.predictions_topic = self._declare('predictions_topic', '/predictions')
        self.predictions_used_topic = self._declare(
            'prediction_filter.output_topic',
            '/dra_mppi/predictions_used',
        )
        self.prediction_class_names = tuple(self._declare(
            'prediction_filter.class_names', ['person']))
        self.prediction_max_semantic_age = float(self._declare(
            'prediction_filter.max_semantic_age', 1.0))
        self.prediction_max_measurement_age = float(self._declare(
            'prediction_filter.max_measurement_age', 0.5))
        self.prediction_require_confirmed = bool(self._declare(
            'prediction_filter.require_confirmed', True))
        self.prediction_allow_recent_lost = bool(self._declare(
            'prediction_filter.allow_recent_lost', True))
        self.prediction_lost_grace_time = float(self._declare(
            'prediction_filter.lost_grace_time', 0.30))
        self.stationary_nondynamic_predictions = bool(self._declare(
            'prediction_filter.stationary_nondynamic', True))
        self.prediction_fail_safe = bool(self._declare(
            'prediction_fail_safe', True))
        if self.prediction_max_semantic_age <= 0.0:
            raise ValueError('prediction_filter.max_semantic_age must be positive')
        if self.prediction_max_measurement_age < 0.0:
            raise ValueError(
                'prediction_filter.max_measurement_age must be non-negative')
        if not (0.0 <= self.prediction_lost_grace_time
                <= self.prediction_max_measurement_age):
            raise ValueError(
                'prediction_filter.lost_grace_time must be within measurement age')
        self.reference_topic = self._declare(
            'reference_topic',
            '/dra_mppi/reference',
        )
        self.reference_trajectory_topic = self._declare(
            'reference_trajectory_topic',
            '/dra_mppi/reference_trajectory',
        )
        self.cmd_vel_topic = self._declare('cmd_vel_topic', '/dra_mppi/cmd_vel')
        self.publish_cmd_vel = bool(self._declare('publish_cmd_vel', False))
        self.selected_path_topic = self._declare(
            'selected_path_topic',
            '/dra_mppi/selected_path',
        )
        self.rollouts_topic = self._declare(
            'rollouts_topic',
            '/dra_mppi/rollouts',
        )
        self.guide_mc_region_areas_topic = self._declare(
            'guide_mc_region_areas_topic',
            '/dra_mppi/guide_mc_region_areas',
        )
        self.guide_mc_sample_counts_topic = self._declare(
            'guide_mc_sample_counts_topic',
            '/dra_mppi/guide_mc_sample_counts',
        )

        controller_frequency = float(self._declare('controller_frequency', 10.0))
        self.controller_period = 1.0 / controller_frequency
        self.odom_timeout = float(self._declare('odom_timeout', 0.25))
        self.prediction_timeout = float(self._declare('prediction_timeout', 0.50))
        self.path_lookahead = float(self._declare('path_lookahead', 1.2))
        self.max_path_points = int(self._declare('max_path_points', 80))
        self.goal_tolerance = float(self._declare('goal_tolerance', 0.15))
        self.goal_yaw_tolerance = float(
            self._declare('goal_yaw_tolerance', math.radians(5.0))
        )
        self.goal_heading_gain = float(
            self._declare('goal_heading.gain', 1.5)
        )
        self.goal_heading_max_yaw_rate = float(
            self._declare('goal_heading.max_yaw_rate', 0.60)
        )
        self.goal_terminal_exit_tolerance = float(
            self._declare('goal_terminal.exit_tolerance', 0.25)
        )
        self.goal_terminal_completion_tolerance = float(
            self._declare('goal_terminal.completion_tolerance', 0.08)
        )
        self.goal_terminal_max_linear_speed = float(
            self._declare('goal_terminal.max_linear_speed', 0.10)
        )
        self.goal_terminal_linear_gain = float(
            self._declare('goal_terminal.linear_gain', 1.0)
        )
        self.goal_terminal_settle_linear_speed = float(
            self._declare('goal_terminal.settle_linear_speed', 0.03)
        )
        self.goal_terminal_settle_yaw_rate = float(
            self._declare('goal_terminal.settle_yaw_rate', 0.05)
        )
        self.goal_terminal_settle_cycles = int(
            self._declare('goal_terminal.settle_cycles', 3)
        )
        if self.goal_tolerance <= 0.0:
            raise ValueError('goal_tolerance must be positive')
        if not 0.0 < self.goal_yaw_tolerance <= math.pi:
            raise ValueError('goal_yaw_tolerance must be in (0, pi]')
        if self.goal_heading_gain <= 0.0:
            raise ValueError('goal_heading.gain must be positive')
        if self.goal_heading_max_yaw_rate <= 0.0:
            raise ValueError('goal_heading.max_yaw_rate must be positive')
        if not (
            0.0 < self.goal_terminal_completion_tolerance
            <= self.goal_tolerance
            < self.goal_terminal_exit_tolerance
        ):
            raise ValueError(
                'terminal distances must satisfy 0 < completion <= '
                'goal_tolerance < exit'
            )
        if (
            self.goal_terminal_max_linear_speed <= 0.0
            or self.goal_terminal_linear_gain <= 0.0
        ):
            raise ValueError('terminal linear speed and gain must be positive')
        if (
            self.goal_terminal_settle_linear_speed < 0.0
            or self.goal_terminal_settle_yaw_rate < 0.0
            or self.goal_terminal_settle_cycles < 1
        ):
            raise ValueError('terminal settle thresholds/cycles are invalid')
        self.visualization_rollouts = int(
            self._declare('visualization_rollouts', 32)
        )
        robot_radius = float(self._declare('robot_radius', 0.22))
        dynamic_config = DynamicRiskConfig(
            mode=str(self._declare('dynamic_risk.mode', 'monte_carlo')),
            soft_weight=float(
                self._declare('dynamic_risk.soft_weight', 18.0)
            ),
            robot_radius=robot_radius,
            obstacle_radius=float(
                self._declare('dynamic_risk.obstacle_radius', 0.25)
            ),
            minimum_sigma=float(
                self._declare('dynamic_risk.minimum_sigma', 0.08)
            ),
            mc_samples=int(self._declare('dynamic_risk.mc_samples', 2048)),
            adaptive_halton_mc=bool(
                self._declare('dynamic_risk.adaptive_halton_mc', False)
            ),
            mc_sample_density=float(
                self._declare('dynamic_risk.mc_sample_density', 124.0)
            ),
            mc_samples_min=int(
                self._declare('dynamic_risk.mc_samples_min', 512)
            ),
            mc_samples_max=int(
                self._declare('dynamic_risk.mc_samples_max', 4096)
            ),
            guide_parallel_workers=int(
                self._declare('dynamic_risk.guide_parallel_workers', 2)
            ),
            guide_mc_sample_density=float(
                self._declare('dynamic_risk.guide_mc_sample_density', 124.0)
            ),
            guide_mc_samples_min=int(
                self._declare('dynamic_risk.guide_mc_samples_min', 128)
            ),
            guide_mc_samples_max=int(
                self._declare('dynamic_risk.guide_mc_samples_max', 1024)
            ),
            final_mc_sample_density=float(
                self._declare('dynamic_risk.final_mc_sample_density', 124.0)
            ),
            final_mc_samples_min=int(
                self._declare('dynamic_risk.final_mc_samples_min', 256)
            ),
            final_mc_samples_max=int(
                self._declare('dynamic_risk.final_mc_samples_max', 4096)
            ),
            mc_reject_underresolved=bool(self._declare('dynamic_risk.mc_reject_underresolved', True)),
            hard_threshold=float(
                self._declare('dynamic_risk.hard_threshold', 0.05)
            ),
            hard_penalty=float(
                self._declare('dynamic_risk.hard_penalty', 100000.0)
            ),
            seed=int(self._declare('dynamic_risk.seed', 17)),
        )
        # Mecanum wheel-speed limit. A single wheel carries vx, vy and yaw_rate
        # at once, so the feasible set is the L1 polytope below, not the axis
        # box. load_budget reserves authority for the downstream MPC.
        wheel_radius = float(self._declare('limits.wheel_radius', 0.04))
        max_wheel_rad_per_sec = float(
            self._declare('limits.max_wheel_rad_per_sec', 30.0)
        )
        load_budget = float(self._declare('limits.load_budget', 0.65))
        yaw_lever = float(self._declare('limits.yaw_lever', 0.19835))
        speed_limit_l1 = wheel_radius * max_wheel_rad_per_sec * load_budget
        self.get_logger().info(
            f'wheel-speed limit: |vx|+|vy|+{yaw_lever:.5f}|yaw_rate| <= '
            f'{speed_limit_l1:.3f} m/s '
            f'({wheel_radius:.3f} m x {max_wheel_rad_per_sec:.1f} rad/s x '
            f'{load_budget:.2f} budget)'
        )
        config = MppiConfig(
            dt=float(self._declare('mppi.dt', 0.1)),
            horizon_steps=int(self._declare('mppi.horizon_steps', 30)),
            num_rollouts=int(self._declare('mppi.num_rollouts', 512)),
            strategy=str(self._declare('mppi.strategy', 'standard')),
            temperature=float(self._declare('mppi.temperature', 2.0)),
            adaptive_temperature=bool(
                self._declare('mppi.adaptive_temperature', True)
            ),
            temperature_eta_lower=float(
                self._declare('mppi.temperature_eta_lower', 5.0)
            ),
            temperature_eta_upper=float(
                self._declare('mppi.temperature_eta_upper', 10.0)
            ),
            temperature_decrease_factor=float(
                self._declare('mppi.temperature_decrease_factor', 0.9)
            ),
            temperature_increase_factor=float(
                self._declare('mppi.temperature_increase_factor', 1.2)
            ),
            noise_std=tuple(self._declare('mppi.noise_std', [0.10, 0.10, 0.30])),
            spline_knots=int(self._declare('mppi.spline_knots', 4)),
            halton_scramble=bool(self._declare('mppi.halton_scramble', True)),
            control_min=tuple(
                self._declare('limits.control_min', [-0.10, -0.15, -0.80])
            ),
            control_max=tuple(
                self._declare('limits.control_max', [0.25, 0.15, 0.80])
            ),
            acceleration_limit=tuple(
                self._declare('limits.acceleration', [0.60, 0.60, 1.80])
            ),
            yaw_lever=yaw_lever,
            speed_limit_l1=speed_limit_l1,
            desired_speed=float(self._declare('cost.desired_speed', 0.20)),
            tracking_path_weight=float(
                self._declare('cost.tracking_path_weight', 8.0)
            ),
            tracking_goal_weight=float(
                self._declare('cost.tracking_goal_weight', 24.0)
            ),
            heading_weight=float(self._declare('cost.heading_weight', 8.0)),
            heading_lookahead=float(
                self._declare('cost.heading_lookahead', 0.20)
            ),
            speed_weight=float(self._declare('cost.speed_weight', 1.0)),
            progress_weight=float(self._declare('cost.progress_weight', 8.0)),
            projection_window=float(self._declare('cost.projection_window', 0.5)),
            goal_activation_distance=float(self._declare('cost.goal_activation_distance', 0.5)),
            input_delta_weight=tuple(self._declare('cost.input_delta_weight', [0.5, 0.5, 0.055])),
            seed=int(self._declare('mppi.seed', 7)),
            guide_num_rollouts=int(self._declare('mppi.guide_num_rollouts', 500)),
            guide_angles_deg=tuple(self._declare(
                'mppi.guide_angles_deg', [0.0, 45.0, 90.0, -45.0, -90.0])),
            guide_noise_std=tuple(self._declare(
                'mppi.guide_noise_std', [0.15, 0.12, 0.45])),
            guide_anchor_blend=float(self._declare('mppi.guide_anchor_blend', 0.25)),
            guide_switch_penalty=float(self._declare('mppi.guide_switch_penalty', 0.25)),
            guide_hysteresis=float(self._declare('mppi.guide_hysteresis', 0.10)),
            guide_evaluation=str(self._declare('mppi.guide_evaluation', 'split')),
            guide_timeout=float(self._declare('mppi.guide_timeout', 0.18)),
            static_cost_enabled=bool(self._declare(
                'static_cost.enabled', False)),
            static_cost_soft_weight=float(self._declare(
                'static_cost.soft_weight', 8.0)),
            static_cost_decay=float(self._declare(
                'static_cost.decay', 0.10)),
            static_cost_robot_radius=robot_radius,
            dynamic_risk=dynamic_config,
        )
        self.controller = MppiController(config)
        self.goal_heading_max_yaw_rate = min(
            self.goal_heading_max_yaw_rate,
            float(config.control_max[2]),
            abs(float(config.control_min[2])),
        )
        if self.goal_heading_max_yaw_rate <= 0.0:
            raise ValueError('yaw control limits must permit terminal alignment')
        expected_frequency = 1.0 / config.dt
        if not math.isclose(controller_frequency, expected_frequency, rel_tol=1e-6):
            self.get_logger().warning(
                f'controller_frequency={controller_frequency:.3f} Hz does not '
                f'match 1/mppi.dt={expected_frequency:.3f} Hz; one-step warm '
                'start shifting will be time-misaligned.'
            )

        self._path_message = None
        self._state = None
        self._current_control = np.zeros(3, dtype=np.float64)
        self._last_odom_monotonic = 0.0
        self._prediction_message = None
        self._last_prediction_monotonic = 0.0
        self._goal_completion = GoalCompletionLatch()
        self._goal_path_pending = False
        self._last_nonempty_path_goal = None
        self._aligning_goal_heading = False
        self._goal_settle_count = 0
        self._active = False
        self._last_status_log = 0.0

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._distance_field = None
        self._traversable_received_at = 0.0
        self._traversable_stamp_ns = 0
        self._traversable_sensor_stamp_ns = 0
        self.transform_max_age = float(self._declare('transform_max_age', 0.5))
        if not math.isfinite(self.transform_max_age) or self.transform_max_age <= 0:
            raise ValueError('transform_max_age must be finite and positive')
        self.traversable_timeout = float(self._declare('traversability.timeout', 1.0))
        self.traversable_unknown_is_obstacle = bool(self._declare(
            'traversability.unknown_is_obstacle', True))
        traversable_topic = str(self._declare('traversability.topic', '/traversable_map'))
        if config.strategy == 'guide_conditioned' or config.static_cost_enabled:
            if not math.isfinite(self.traversable_timeout) or self.traversable_timeout <= 0:
                raise ValueError('traversability.timeout must be finite and positive')

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            TraversableMap, traversable_topic, self._on_traversable_map, latched_qos)
        self.create_subscription(Path, self.global_path_topic, self._on_path, latched_qos)
        self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self._on_goal_pose,
            10,
        )
        self.create_subscription(Odometry, self.odometry_topic, self._on_odometry, 1)
        self.predictions_used_publisher = self.create_publisher(
            MultiModalObjectPredictionArray,
            self.predictions_used_topic,
            10,
        )
        self.create_subscription(
            MultiModalObjectPredictionArray,
            self.predictions_topic,
            self._on_predictions,
            1,
        )

        self.reference_publisher = self.create_publisher(
            PlanarReference,
            self.reference_topic,
            10,
        )
        self.reference_trajectory_publisher = self.create_publisher(
            PlanarReferenceArray,
            self.reference_trajectory_topic,
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_publisher = self.create_publisher(
            Path,
            self.selected_path_topic,
            10,
        )
        self.rollout_publisher = self.create_publisher(
            MarkerArray,
            self.rollouts_topic,
            10,
        )
        self.guide_mc_region_areas_publisher = self.create_publisher(
            Float64MultiArray,
            self.guide_mc_region_areas_topic,
            10,
        )
        self.guide_mc_sample_counts_publisher = self.create_publisher(
            Int32MultiArray,
            self.guide_mc_sample_counts_topic,
            10,
        )
        self.add_on_set_parameters_callback(self._validate_runtime_parameters)
        self.add_post_set_parameters_callback(self._apply_runtime_parameters)
        timing_file = str(self._declare('timing.jsonl_path', ''))
        self._cycle_recorder = CycleRecorder(timing_file) if timing_file else None
        self._last_reference_publish = None
        self._cycle_index = 0
        self.timer = self.create_timer(self.controller_period, self._control_cycle)
        self._timing_wall_schedule = not self.get_parameter('use_sim_time').value
        self._timer_expected = time.monotonic()+self.timer.time_until_next_call()/1e9
        self.get_logger().info(
            f'DRA-MPPI ready: strategy={config.strategy}, '
            f'path={self.global_path_topic}, '
            f'odom={self.odometry_topic}, predictions={self.predictions_topic}, '
            f'prediction_classes={list(self.prediction_class_names)}, '
            f'reference_trajectory={self.reference_trajectory_topic} '
            f'({self.planning_frame} frame)'
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _validate_runtime_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'static_cost.enabled' and type(parameter.value) is not bool:
                return SetParametersResult(successful=False, reason='static_cost.enabled must be bool')
        return SetParametersResult(successful=True)

    def _apply_runtime_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'static_cost.enabled':
                self.controller.config.static_cost_enabled = parameter.value
                self.controller.reset()
                self.get_logger().info(f'Static obstacle cost enabled={parameter.value}')

    def destroy_node(self):
        if getattr(self, '_cycle_recorder', None) is not None:
            self._cycle_recorder.close()
        self.controller.close()
        return super().destroy_node()

    def _on_traversable_map(self, message):
        """Build the distance field only on map updates, never per rollout."""
        try:
            grid = message.grid
            info = grid.info
            frame = grid.header.frame_id or message.header.frame_id
            if not frame or (grid.header.frame_id and message.header.frame_id
                             and grid.header.frame_id != message.header.frame_id):
                raise ValueError('missing or inconsistent traversability frame')
            if not math.isfinite(message.inflation_radius) or message.inflation_radius < 0:
                raise ValueError('invalid traversability inflation radius')
            if message.inflation_radius > 0:
                raise ValueError('expected uninflated traversability map; refusing double inflation')
            q = info.origin.orientation
            metadata = GridMetadata(
                info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y,
                quaternion_to_yaw(q.x, q.y, q.z, q.w))
            field = DistanceField.from_occupancy(
                grid.data, metadata, frame,
                unknown_is_occupied=self.traversable_unknown_is_obstacle)
            self._distance_field = field
            self._traversable_received_at = time.monotonic()
            self._traversable_stamp_ns = Time.from_msg(grid.header.stamp).nanoseconds
            self._traversable_sensor_stamp_ns = Time.from_msg(
                message.sensor_watermark).nanoseconds
        except (ValueError, TypeError, OverflowError) as exc:
            self._distance_field = None
            self.get_logger().warning(
                f'Invalid traversability map: {exc}', throttle_duration_sec=1.0)

    def _traversability_for_control(self):
        field = self._distance_field
        if field is None:
            raise TraversabilityUnavailable('no valid map')
        receive_age = time.monotonic() - self._traversable_received_at
        stamp_age = (self.get_clock().now().nanoseconds - self._traversable_stamp_ns) / 1e9
        sensor_age = (self.get_clock().now().nanoseconds
                      - self._traversable_sensor_stamp_ns) / 1e9
        if (receive_age > self.traversable_timeout or stamp_age > self.traversable_timeout
                or stamp_age < -0.1 or self._traversable_sensor_stamp_ns <= 0
                or not -0.1 <= sensor_age <= self.traversable_timeout):
            raise TraversabilityUnavailable('stale map or clock mismatch')
        try:
            transform = self._lookup_2d(field.frame_id, self.planning_frame)
            if np.asarray(transform).shape != (3,) or not np.all(np.isfinite(transform)):
                raise ValueError('invalid planar map transform')
        except Exception as exc:
            raise TraversabilityUnavailable(f'map transform unavailable: {exc}') from exc
        return field, transform

    def _on_path(self, message):
        path_goal = self._path_goal_identity(message)
        completed_goal_replaced = (
            self._goal_completion.completed
            and path_goal is not None
            and self._last_nonempty_path_goal is not None
            and not self._same_path_goal(
                path_goal,
                self._last_nonempty_path_goal,
            )
        )
        self._path_message = message
        new_goal_path = (
            (self._goal_path_pending and path_goal is not None)
            or completed_goal_replaced
        )
        if new_goal_path:
            self._goal_path_pending = False
            self._goal_completion.reset()
            self._aligning_goal_heading = False
            self._goal_settle_count = 0
            if completed_goal_replaced:
                self.get_logger().info(
                    'Completion latch released by a replacement global-path '
                    'terminal pose.'
                )
        if path_goal is not None:
            # Keep this across an empty path: the global planner clears its
            # previous path before publishing the replacement for a new goal.
            self._last_nonempty_path_goal = path_goal
        self.controller.reset()
        self.get_logger().info(
            f'Received global path with {len(message.poses)} poses '
            f'in {message.header.frame_id or "map"}'
        )

    @staticmethod
    def _path_goal_identity(path_message):
        """Return the source-frame terminal pose used to identify one goal."""
        if path_message is None or not path_message.poses:
            return None
        endpoint = path_message.poses[-1]
        pose = endpoint.pose
        orientation = pose.orientation
        identity = (
            path_message.header.frame_id or endpoint.header.frame_id or 'map',
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        return (
            identity
            if all(math.isfinite(value) for value in identity[1:])
            else None
        )

    @staticmethod
    def _same_path_goal(first, second):
        """Compare terminal position and yaw while tolerating angle wrapping."""
        return (
            first[0] == second[0]
            and math.hypot(first[1] - second[1], first[2] - second[2]) <= 1e-6
            and abs(heading_error(first[3], second[3])) <= 1e-6
        )

    def _on_goal_pose(self, message):
        """Release the completion latch only for an explicit new RViz goal."""
        self._goal_completion.reset()
        self._aligning_goal_heading = False
        self._goal_settle_count = 0
        # The goal and its resulting global path travel on different topics,
        # so DDS does not guarantee which callback is observed first. If the
        # matching path is already here, use it; otherwise hold until the
        # global planner supplies a non-empty replacement path.
        self._goal_path_pending = not self._path_matches_goal(
            self._path_message,
            message,
        )
        self.controller.reset()
        stamp = self.get_clock().now().to_msg()
        self._publish_stop()
        self._clear_local_visualization(stamp)
        path_status = (
            'waiting for its global path'
            if self._goal_path_pending
            else 'using the matching global path already received'
        )
        self.get_logger().info(
            'New goal received; goal-completion latch released and '
            f'{path_status}.'
        )

    @staticmethod
    def _path_matches_goal(path_message, goal_message):
        if path_message is None or not path_message.poses:
            return False
        path_frame = path_message.header.frame_id or 'map'
        goal_frame = goal_message.header.frame_id or 'map'
        if path_frame != goal_frame:
            return False
        endpoint = path_message.poses[-1].pose.position
        requested = goal_message.pose.position
        return math.hypot(
            endpoint.x - requested.x,
            endpoint.y - requested.y,
        ) <= 1e-6

    def _on_odometry(self, message):
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        if stamp_ns <= 0 or not -0.1 <= age <= self.odom_timeout:
            self.get_logger().warning('Odometry source stamp is missing/stale/future',
                                      throttle_duration_sec=2.0)
            return
        self._odom_source_age_at_receipt = max(0., age)
        pose = message.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        state = np.array([pose.position.x, pose.position.y, yaw], dtype=np.float64)
        source_frame = message.header.frame_id or self.planning_frame
        try:
            transform = self._lookup_2d(self.planning_frame, source_frame)
            self._state = transform_states(state, transform)
        except Exception as exc:
            self.get_logger().warning(
                f'Cannot transform odometry {source_frame}->{self.planning_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return
        twist = message.twist.twist
        self._current_control[:] = (
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        )
        self._last_odom_monotonic = time.monotonic()

    def _on_predictions(self, message):
        try:
            filtered = filter_predictions_for_control(
                message,
                self.prediction_class_names,
                self.prediction_max_semantic_age,
                self.prediction_max_measurement_age,
                require_confirmed=self.prediction_require_confirmed,
                stationary_nondynamic=self.stationary_nondynamic_predictions,
                allow_recent_lost=self.prediction_allow_recent_lost,
                lost_grace_time=self.prediction_lost_grace_time,
            )
        except (ValueError, TypeError) as exc:
            self._prediction_message = None
            self.get_logger().warning(f'Invalid person prediction: {exc}',
                                      throttle_duration_sec=1.0)
            return
        self._prediction_message = filtered
        self._last_prediction_monotonic = time.monotonic()
        self.predictions_used_publisher.publish(filtered)
        rejected = len(message.objects) - len(filtered.objects)
        if rejected:
            self.get_logger().info(
                f'prediction filter: {len(filtered.objects)} used, '
                f'{rejected} rejected',
                throttle_duration_sec=2.0,
            )

    def _control_cycle(self):
        started = time.monotonic()
        self._cycle_status = 'exception'
        self._cycle_result = None
        self._cycle_publish_interval_ms = None
        self._cycle_reference_published = None
        self._cycle_obstacles = self._cycle_modes = 0
        self._cycle_error = ''
        expected = getattr(self, '_timer_expected', started)
        skipped = max(0, int((started-expected)/self.controller_period))
        self._timer_expected = expected+(skipped+1)*self.controller_period
        try:
            DraMppiNode._run_control_cycle(self)
        finally:
            finished = time.monotonic()
            recorder = getattr(self, '_cycle_recorder', None)
            if recorder is not None:
                result = self._cycle_result
                record_started = time.monotonic()
                row = {
                    'cycle': self._cycle_index, 'started': started,
                    'K': self.controller.config.guide_num_rollouts if self.controller.config.strategy == 'guide_conditioned' else self.controller.config.num_rollouts,
                    'N': self.controller.config.horizon_steps, 'dt': self.controller.config.dt,
                    'backend': self.controller.config.guide_evaluation,
                    'workers': self.controller.config.dynamic_risk.guide_parallel_workers,
                    'obstacles': self._cycle_obstacles, 'modes': self._cycle_modes,
                    'failure_reason': self._cycle_error,
                    'odom_source_age_ms': 1000*(started-getattr(self, '_last_odom_monotonic', started)+getattr(self, '_odom_source_age_at_receipt', 0.)),
                    'odom_receive_age_ms': 1000*(started-getattr(self, '_last_odom_monotonic', started)),
                    'prediction_receive_age_ms': 1000*(started-getattr(self, '_last_prediction_monotonic', started)),
                    'map_receive_age_ms': 1000*(started-getattr(self, '_traversable_received_at', started)),
                    'reference_publish_ms': None if self._cycle_reference_published is None else 1000*(self._cycle_reference_published-started),
                    'scheduled_response_ms': None if self._cycle_reference_published is None else 1000*(self._cycle_reference_published-expected),
                    'stage_ms': None if result is None else result.stage_ms,
                    'cost_raw': None if result is None else result.selected_cost_raw,
                    'cost_weighted': None if result is None else result.selected_cost_weighted,

                    'expected': expected, 'finished': finished,
                    'schedule_source': ('monotonic_initial_timer_phase'
                                        if self._timing_wall_schedule else 'sim_time_unmeasured'),
                    'skipped_periods': skipped,
                    'status': self._cycle_status,
                    'callback_ms': 1000*(finished-started),
                    'start_delay_ms': 1000*max(0., started-expected),
                    'budget_overrun': finished-started > self.controller_period,
                    'deadline_lateness_ms': 1000*max(
                        0., finished-expected-self.controller_period),
                    'publish_interval_ms': self._cycle_publish_interval_ms,
                    'optimizer_ms': None if result is None else result.compute_time_ms,
                    'static_ms': None if result is None else self.controller.static_time_ms,
                    'primary_mc_ms': None if result is None else result.primary_risk_time_ms,
                    'final_mc_ms': None if result is None else result.final_risk_time_ms,
                    'representative_ms': None if result is None else self.controller.representative_time_ms,
                    'selected': None if result is None else result.selected_mode,
                    'max_step_cp': None if result is None else result.max_collision_probability,
                    'static_enabled': self.controller.config.static_cost_enabled,
                    'risk_integration_mode': self.controller.config.dynamic_risk.mode,
                    'primary_analytic_evaluations': None if result is None else self.controller.risk_diagnostics.analytic_evaluations,
                    'primary_quadrature_evaluations': None if result is None else self.controller.risk_diagnostics.quadrature_evaluations,
                    'primary_integration_error_estimate': None if result is None else self.controller.risk_diagnostics.error_estimate_max,
                    'final_analytic_evaluations': None if result is None or result.final_risk_diagnostics is None else result.final_risk_diagnostics.analytic_evaluations,
                    'primary_unresolved_integrals': None if result is None else self.controller.risk_diagnostics.unresolved_integrals,
                    'final_quadrature_evaluations': None if result is None or result.final_risk_diagnostics is None else result.final_risk_diagnostics.quadrature_evaluations,
                    'final_unresolved_integrals': None if result is None or result.final_risk_diagnostics is None else result.final_risk_diagnostics.unresolved_integrals,
                    'actual_primary_mc_samples': None if result is None else self.controller.risk_diagnostics.total_samples,
                    'candidate_sample_comparisons': None if result is None else self.controller.risk_diagnostics.candidate_sample_comparisons,
                    'underresolved_steps': None if result is None else self.controller.risk_diagnostics.underresolved_steps,
                    'peak_pairwise_bytes_bound': None if result is None else self.controller.risk_diagnostics.peak_pairwise_bytes,
                    'primary_empty_circles': None if result is None else self.controller.risk_diagnostics.empty_circle_count,
                    'primary_clipped_marginals': None if result is None else self.controller.risk_diagnostics.clipped_marginal_count,
                    'mc_samples': None if result is None or result.guide_mc_sample_counts is None
                    else result.guide_mc_sample_counts.tolist(),
                    'final_mc_samples': None if result is None or result.final_mc_sample_counts is None
                    else result.final_mc_sample_counts.tolist(),
                }
                if not self._timing_wall_schedule:
                    for key in ('expected', 'start_delay_ms', 'deadline_lateness_ms', 'skipped_periods'):
                        row[key] = None
                row['record_prepare_ms'] = 1000*(time.monotonic()-record_started)
                recorder.record(row)
            self._cycle_index = getattr(self, '_cycle_index', 0)+1

    def _run_control_cycle(self):
        cycle_started = time.perf_counter()
        if self._goal_completion.completed:
            self._cycle_status = 'completed_hold'
            self._publish_stop(self._goal_completion.hold_state)
            return
        if self._goal_path_pending:
            self._cycle_status = 'pending_path'
            self._publish_stop()
            return
        if self._state is None or self._path_message is None:
            self._cycle_status = 'missing_input'
            self._publish_stop()
            return
        if (time.monotonic() - self._last_odom_monotonic
                + getattr(self, '_odom_source_age_at_receipt', 0.) > self.odom_timeout):
            self._cycle_status = 'stale_odom'
            self.get_logger().warning(
                'Odometry is stale; publishing a hold reference.',
                throttle_duration_sec=2.0,
            )
            self._publish_stop()
            return

        try:
            global_path = self._path_in_planning_frame()
            if global_path.shape[0] == 0:
                self._cycle_status = 'empty_path'
                self._publish_stop()
                return
            # Terminal XY/yaw alignment is motion too: apply the same input
            # freshness gates before either terminal control or MPPI computation.
            predictions = self._aligned_predictions()
            self._cycle_obstacles = 0 if predictions is None else predictions.means.shape[0]
            self._cycle_modes = 0 if predictions is None else int(np.count_nonzero(np.any(predictions.mode_weights > 0, axis=-1) if predictions.mode_weights.ndim == 3 else predictions.mode_weights > 0))
            distance_field = None
            source_to_grid = None
            if self.controller.config.static_cost_enabled:
                distance_field, source_to_grid = self._traversability_for_control()
            goal_state = self._path_goal_in_planning_frame()
            position_error = float(np.linalg.norm(
                goal_state[:2] - self._state[:2]
            ))
            yaw_error = heading_error(goal_state[2], self._state[2])
            if (
                self._aligning_goal_heading
                and position_error > self.goal_terminal_exit_tolerance
            ):
                self.controller.reset()
                self._aligning_goal_heading = False
                self._goal_settle_count = 0
                self.get_logger().warning(
                    'Robot left the terminal region; resuming MPPI '
                    f'({position_error:.3f} m from goal).'
                )

            terminal_active = (
                self._aligning_goal_heading
                or position_error <= self.goal_tolerance
            )
            if terminal_active:
                if not self._aligning_goal_heading:
                    self.controller.reset()
                    self._aligning_goal_heading = True
                    self._goal_settle_count = 0
                    self.get_logger().info(
                        'Entered terminal pose alignment: correcting XY and yaw '
                        f'({position_error:.3f} m, '
                        f'{math.degrees(yaw_error):+.1f} deg remaining).'
                    )

                linear_speed = float(np.linalg.norm(self._current_control[:2]))
                yaw_rate = abs(float(self._current_control[2]))
                settled = (
                    position_error <= self.goal_terminal_completion_tolerance
                    and abs(yaw_error) <= self.goal_yaw_tolerance
                    and linear_speed <= self.goal_terminal_settle_linear_speed
                    and yaw_rate <= self.goal_terminal_settle_yaw_rate
                )
                self._goal_settle_count = (
                    self._goal_settle_count + 1 if settled else 0
                )
                if (
                    self._goal_settle_count
                    >= self.goal_terminal_settle_cycles
                ):
                    completed = self._goal_completion.complete_if_reached(
                        global_path,
                        self._state,
                        self.goal_terminal_completion_tolerance,
                        goal_yaw=goal_state[2],
                        yaw_tolerance=self.goal_yaw_tolerance,
                        hold_state=goal_state,
                    )
                    if not completed:
                        raise RuntimeError(
                            'terminal settle conditions and goal latch disagree'
                        )
                    self.controller.reset()
                    self._aligning_goal_heading = False
                    self._cycle_status = 'completed_hold'
                    stamp = self.get_clock().now().to_msg()
                    self._publish_stop(self._goal_completion.hold_state)
                    self._clear_local_visualization(stamp)
                    self.get_logger().info(
                        f'Goal completed at {position_error:.3f} m and '
                        f'{math.degrees(abs(yaw_error)):.1f} deg after '
                        f'{self._goal_settle_count} settled cycles; holding '
                        'the requested goal pose until a new goal is received.'
                    )
                    return

                alignment = build_terminal_pose_reference(
                    self._state,
                    goal_state,
                    self._current_control,
                    self.controller.config.dt,
                    self.controller.config.horizon_steps,
                    self.goal_terminal_max_linear_speed,
                    self.goal_terminal_linear_gain,
                    self.goal_heading_max_yaw_rate,
                    self.goal_heading_gain,
                    self.controller.config.acceleration_limit,
                )
                # Terminal translation/rotation uses the same hard constraints
                # and collision evaluation as every optimizer representative.
                from .mecanum_model import rollout
                controls = self.controller._constrain_controls(
                    alignment[None, :, 3:], self._current_control)
                poses = rollout(self._state, controls, self.controller.config.dt)
                static, _ = self.controller._static_obstacle_cost(
                    self._state, poses, distance_field, source_to_grid)
                risk = self.controller._evaluate_dynamic_risk(poses, predictions)
                if (not np.isfinite(static[0]) or np.any(
                        risk.collision_probability > self.controller.config.dynamic_risk.hard_threshold)):
                    self._cycle_status = 'terminal_unsafe_hold'
                    self._publish_stop()
                    return
                alignment = np.concatenate((poses[0], controls[0]), axis=1)
                stamp = self.get_clock().now().to_msg()
                self._publish_reference(
                    alignment[0, :3],
                    alignment[0, 3:],
                    stamp,
                )
                self._publish_reference_trajectory(alignment, stamp)
                self._publish_selected_path(alignment[:, :3], stamp)
                self._clear_rollout_markers(stamp)
                self._active = True
                self._cycle_status = 'terminal'
                return
            # Continuous projection and goal gating need the actual global
            # endpoint, not the end of a cropped local window.
            result = self.controller.compute(
                self._state,
                global_path,
                current_control=self._current_control,
                dynamic_predictions=predictions,
                visualization_rollouts=self.visualization_rollouts,
                guide_path=global_path,
                distance_field=distance_field,
                source_to_grid=source_to_grid,
            )
        except TraversabilityUnavailable as exc:
            self._cycle_status = 'stale_map'
            self.get_logger().warning(
                f'Traversability unavailable; holding position: {exc}',
                throttle_duration_sec=1.0)
            self.controller.reset()
            self._publish_stop()
            self._clear_rollout_markers(self.get_clock().now().to_msg())
            return
        except PredictionChainUnavailable as exc:
            self._cycle_status = 'stale_prediction'
            self.get_logger().warning(
                f'Prediction chain unavailable; holding position: {exc}',
                throttle_duration_sec=1.0,
            )
            self.controller.reset()
            self._publish_stop()
            return
        except Exception as exc:
            self._cycle_status = 'timeout' if isinstance(exc, TimeoutError) else 'exception'
            self._cycle_error = str(exc)
            self.controller.reset()
            self.get_logger().warning(
                f'DRA-MPPI cycle failed; holding position: {exc}',
                throttle_duration_sec=1.0,
            )
            self._publish_stop()
            return

        self._cycle_result = result
        if (time.monotonic()-self._last_odom_monotonic
                + getattr(self, '_odom_source_age_at_receipt', 0.) > self.odom_timeout):
            self._cycle_status = 'expired_input'
            self._cycle_error = 'odometry expired while planning; result discarded'
            self.controller.reset()
            self._publish_stop()
            return
        self._cycle_status = (result.safety_status if result.safety_status != 'ok'
                              else 'stop' if result.selected_mode == 'stop' else 'normal')
        stamp = self.get_clock().now().to_msg()
        self._publish_reference(result.reference_state, result.control, stamp)
        self._publish_reference_trajectory(result.reference_sequence, stamp)
        self._publish_selected_path(result.trajectory, stamp)
        self._publish_rollouts(result, stamp)
        self._publish_guide_mc_diagnostics(result)
        self._active = True
        cycle_time_ms = (time.perf_counter() - cycle_started) * 1000.0
        deadline_ms = self.controller_period * 1000.0
        if cycle_time_ms > deadline_ms:
            self.get_logger().warning(
                f'DRA-MPPI cycle missed its {deadline_ms:.1f} ms deadline: '
                f'{cycle_time_ms:.1f} ms',
                throttle_duration_sec=2.0,
            )

        now = time.monotonic()
        if now - self._last_status_log >= 2.0:
            self._last_status_log = now
            mc = self.controller.risk_diagnostics
            mc_summary = ''
            if mc.sample_counts.size:
                area_label = (
                    'A_sum' if mc.sampling_mode.startswith('per_guide_') else 'A'
                )
                final_risk = (
                    f'+{result.final_risk_time_ms:.1f} ms final'
                    if result.strategy == 'guide_conditioned'
                    else ''
                )
                mc_summary = (
                    f', mc={mc.sampling_mode} {mc.compute_time_ms:.1f}'
                    f'{final_risk} '
                    f'N_total={mc.total_samples} '
                    f'{area_label}={np.min(mc.region_areas):.2f}..'
                    f'{np.max(mc.region_areas):.2f} m^2 '
                    f'N={np.min(mc.sample_counts)}..'
                    f'{np.max(mc.sample_counts)} '
                    f'cp_clip={mc.clipped_marginal_count} '
                    f'empty_circle={mc.empty_circle_count}'
                )
            risk_label = (
                'selected_cp'
                if result.strategy == 'guide_conditioned'
                else 'weighted_rollout_cp'
            )
            self.get_logger().info(
                f'MPPI[{result.selected_mode}] cycle={cycle_time_ms:.1f} ms '
                f'(optimizer={result.compute_time_ms:.1f} ms, '
                f'budget={deadline_ms:.1f} ms), '
                f'cost={result.cost:.2f}, '
                f'C_static={result.selected_static_cost:.2f}, '
                f'ESS={result.effective_sample_size:.1f}, '
                f'{risk_label}={result.max_collision_probability:.3f}, '
                f'u=[{result.control[0]:.2f}, {result.control[1]:.2f}, '
                f'{result.control[2]:.2f}]{mc_summary}'
            )
            self.get_logger().debug(
                f'MPPI weights: beta={result.temperature:.6g} -> '
                f'{result.next_temperature:.6g}, eta={result.weight_normalizer:.3f}, '
                f'ESS={result.effective_sample_size:.3f}, '
                f'cost={result.minimum_cost:.6g}..{result.maximum_cost:.6g}, '
                f'weight={result.minimum_weight:.6g}..{result.maximum_weight:.6g}'
            )
            if result.strategy == 'guide_conditioned':
                self.get_logger().info(
                    f'guides active={result.guide_active_mask.tolist()} '
                    f'counts={result.guide_rollout_counts.tolist()} '
                    f'rejected={result.guide_rejection_reasons} '
                    f'safety={result.safety_status} '
                    f'C_static_candidates='
                    f'{np.round(result.mode_static_costs, 2).tolist()}')
            if mc.sampling_mode.endswith('adaptive_halton') and mc.sample_counts.size:
                self.get_logger().debug(
                    'adaptive MC per step (t: area[m^2] / samples): '
                    + ', '.join(
                        f'{step}: {area:.3f}/{count}'
                        for step, (area, count) in enumerate(
                            zip(mc.region_areas, mc.sample_counts)
                        )
                    )
                )
            if mc.clipped_marginal_count:
                self.get_logger().warning(
                    'MC marginal CP left [0,1] before safety clipping: '
                    f'count={mc.clipped_marginal_count}, '
                    f'raw={mc.raw_marginal_minimum:.6g}..'
                    f'{mc.raw_marginal_maximum:.6g}. Increase or retune MC '
                    'samples if this persists.',
                    throttle_duration_sec=2.0,
                )
            if mc.empty_circle_count:
                self.get_logger().warning(
                    'MC integration had rollout circles with no interior '
                    f'samples: count={mc.empty_circle_count}.',
                    throttle_duration_sec=2.0,
                )

    def _path_in_planning_frame(self):
        message = self._path_message
        if message is None or not message.poses:
            return np.empty((0, 2), dtype=np.float64)
        points = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in message.poses
            ],
            dtype=np.float64,
        )
        source_frame = message.header.frame_id or 'map'
        transform = self._lookup_2d(self.planning_frame, source_frame)
        return transform_points(points, transform)

    def _path_goal_in_planning_frame(self):
        """Return the final Path pose, including RViz goal yaw."""
        message = self._path_message
        if message is None or not message.poses:
            raise ValueError('global path is empty')
        pose = message.poses[-1].pose
        goal = np.array(
            [
                pose.position.x,
                pose.position.y,
                quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            ],
            dtype=np.float64,
        )
        source_frame = message.header.frame_id or 'map'
        transform = self._lookup_2d(self.planning_frame, source_frame)
        return transform_states(goal, transform)

    def _aligned_predictions(self):
        message = self._prediction_message
        if message is None:
            if self.prediction_fail_safe:
                raise PredictionChainUnavailable(
                    'no prediction message received')
            return None
        reception_age = time.monotonic() - self._last_prediction_monotonic
        stamp_is_set = bool(
            message.header.stamp.sec or message.header.stamp.nanosec)
        if stamp_is_set:
            source_age = (
                self.get_clock().now()
                - Time.from_msg(message.header.stamp)
            ).nanoseconds * 1e-9
            if source_age < -0.1:
                raise PredictionChainUnavailable('prediction timestamp is in the future')
            source_age = max(source_age, 0.0)
        else:
            source_age = max(reception_age, 0.0)
        if max(reception_age, source_age) > self.prediction_timeout:
            if self.prediction_fail_safe:
                raise PredictionChainUnavailable(
                    f'prediction age {max(reception_age, source_age):.3f}s '
                    f'exceeds {self.prediction_timeout:.3f}s timeout')
            return None

        source_frame = message.header.frame_id or self.planning_frame
        transform = self._lookup_2d(self.planning_frame, source_frame)
        query_times = (
            np.arange(1, self.controller.config.horizon_steps + 1)
            * self.controller.config.dt
        )
        prediction_times = query_times + source_age
        all_means = []
        all_covariances = []
        all_weights = []
        all_radii = []

        for obstacle in message.objects:
            # The ages in the object are measured at message.header.stamp.
            # Re-check them at the actual control cycle, not only on receipt.
            if not _prediction_is_usable(
                    obstacle,
                    self.prediction_class_names,
                    self.prediction_max_semantic_age,
                    self.prediction_max_measurement_age,
                    require_confirmed=self.prediction_require_confirmed,
                    age_offset=source_age,
                    allow_recent_lost=getattr(
                        self, 'prediction_allow_recent_lost', False),
                    lost_grace_time=getattr(
                        self, 'prediction_lost_grace_time', 0.0)):
                continue
            offsets = np.asarray(obstacle.time_offsets, dtype=np.float64)
            if offsets.size == 0 or not obstacle.modes:
                continue
            order = np.argsort(offsets)
            offsets = offsets[order]
            mode_means = np.zeros((2, query_times.size, 2), dtype=np.float64)
            mode_covariances = np.zeros(
                (2, query_times.size, 2, 2), dtype=np.float64)
            mode_weights = np.zeros((2, query_times.size), dtype=np.float64)
            found_modes = set()
            for mode in obstacle.modes:
                mode_index = int(mode.model_type)
                # Fixed safety discs occupy one internal mixture slot.
                if mode_index == MotionModePrediction.MODEL_STATIC:
                    mode_index = MotionModePrediction.MODEL_CV
                if mode_index not in (
                        MotionModePrediction.MODEL_CV,
                        MotionModePrediction.MODEL_CT):
                    continue
                count = min(offsets.size, len(mode.points))
                if count == 0 or mode_index in found_modes:
                    continue
                mode_offsets = np.asarray(
                    obstacle.time_offsets[:count], dtype=np.float64)
                mode_order = np.argsort(mode_offsets)
                mode_offsets = mode_offsets[mode_order]
                if (stamp_is_set and mode.model_type != MotionModePrediction.MODEL_STATIC
                        and (prediction_times[0] < mode_offsets[0]-1e-9
                             or prediction_times[-1] > mode_offsets[-1]+1e-9)):
                    raise PredictionChainUnavailable('moving prediction horizon does not cover rollout')
                raw_means = np.array([
                    [point.mean_x, point.mean_y]
                    for point in mode.points[:count]
                ], dtype=np.float64)[mode_order]
                raw_covariances = np.array([
                    np.asarray(point.covariance, dtype=np.float64).reshape(2, 2)
                    for point in mode.points[:count]
                ])[mode_order]
                raw_weights = np.asarray([
                    point.weight for point in mode.points[:count]
                ], dtype=np.float64)[mode_order]
                raw_means = transform_points(raw_means, transform)
                raw_covariances = rotate_covariances(
                    raw_covariances, transform[2])

                mode_means[mode_index, :, 0] = np.interp(
                    prediction_times, mode_offsets, raw_means[:, 0])
                mode_means[mode_index, :, 1] = np.interp(
                    prediction_times, mode_offsets, raw_means[:, 1])
                mode_weights[mode_index] = np.interp(
                    prediction_times, mode_offsets, raw_weights)
                for row in range(2):
                    for column in range(2):
                        mode_covariances[mode_index, :, row, column] = np.interp(
                            prediction_times,
                            mode_offsets,
                            raw_covariances[:, row, column],
                        )
                found_modes.add(mode_index)

            if not found_modes:
                continue
            # PredictionSet requires finite tensors even for a zero-weight mode.
            template = next(iter(found_modes))
            for missing in {0, 1} - found_modes:
                mode_means[missing] = mode_means[template]
                mode_covariances[missing] = mode_covariances[template]
            if np.any(np.sum(mode_weights, axis=0) <= 0.0):
                continue
            all_means.append(mode_means)
            all_covariances.append(mode_covariances)
            all_weights.append(mode_weights)
            fallback_radius = self.controller.config.dynamic_risk.obstacle_radius
            all_radii.append(
                float(obstacle.radius)
                if obstacle.radius > 0.0 else fallback_radius)

        if not all_means:
            if message.objects and self.prediction_fail_safe:
                raise PredictionChainUnavailable(
                    'all accepted person predictions became stale or invalid')
            return None
        return PredictionSet(
            means=np.asarray(all_means),
            covariances=np.asarray(all_covariances),
            times=query_times,
            mode_weights=np.asarray(all_weights),
            obstacle_radii=np.asarray(all_radii),
        )

    def _lookup_2d(self, target_frame, source_frame):
        if not source_frame or target_frame == source_frame:
            return 0.0, 0.0, 0.0
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        stamp = Time.from_msg(transform.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -0.1 <= age <= self.transform_max_age:
            raise ValueError('Localization transform is stale or has a clock mismatch')
        transform = transform.transform
        yaw = quaternion_to_yaw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        return transform.translation.x, transform.translation.y, yaw

    def _publish_reference(self, state, control, stamp):
        message = PlanarReference()
        message.header.stamp = stamp
        message.header.frame_id = self.planning_frame
        message.x = float(state[0])
        message.y = float(state[1])
        message.yaw = float(state[2])
        message.vx = float(control[0])
        message.vy = float(control[1])
        message.yaw_rate = float(control[2])
        self.reference_publisher.publish(message)

        if self.publish_cmd_vel:
            command = Twist()
            command.linear.x = message.vx
            command.linear.y = message.vy
            command.angular.z = message.yaw_rate
            self.cmd_vel_publisher.publish(command)

    def _publish_reference_trajectory(self, reference_sequence, stamp):
        sequence = np.asarray(reference_sequence, dtype=np.float64)
        if sequence.ndim != 2 or sequence.shape[1] != 6:
            raise ValueError('reference_sequence must have shape (T, 6)')

        message = PlanarReferenceArray()
        message.header.stamp = stamp
        message.header.frame_id = self.planning_frame
        message.dt = float(self.controller.config.dt)
        for state_control in sequence:
            reference = PlanarReference()
            reference.header = message.header
            reference.x = float(state_control[0])
            reference.y = float(state_control[1])
            reference.yaw = float(state_control[2])
            reference.vx = float(state_control[3])
            reference.vy = float(state_control[4])
            reference.yaw_rate = float(state_control[5])
            message.references.append(reference)
        self.reference_trajectory_publisher.publish(message)
        published = time.monotonic()
        self._cycle_reference_published = published
        previous = getattr(self, '_last_reference_publish', None)
        self._cycle_publish_interval_ms = None if previous is None else 1000*(published-previous)
        self._last_reference_publish = published

    def _publish_stop(self, hold_state=None):
        reference_state = self._state if hold_state is None else np.asarray(
            hold_state,
            dtype=np.float64,
        )
        if reference_state is not None:
            stamp = self.get_clock().now().to_msg()
            self._publish_reference(
                reference_state,
                np.zeros(3),
                stamp,
            )
            stop_sequence = np.zeros(
                (self.controller.config.horizon_steps, 6),
                dtype=np.float64,
            )
            stop_sequence[:, :3] = reference_state
            self._publish_reference_trajectory(stop_sequence, stamp)
        elif self.publish_cmd_vel and self._active:
            self.cmd_vel_publisher.publish(Twist())
        self._active = False

    def _clear_local_visualization(self, stamp):
        self._publish_selected_path(np.empty((0, 3), dtype=np.float64), stamp)
        self._clear_rollout_markers(stamp)

    def _clear_rollout_markers(self, stamp):
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.planning_frame
        clear.action = Marker.DELETEALL
        self.rollout_publisher.publish(MarkerArray(markers=[clear]))

    def _publish_selected_path(self, trajectory, stamp):
        message = Path()
        message.header.stamp = stamp
        message.header.frame_id = self.planning_frame
        for state in trajectory:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(state[0])
            pose.pose.position.y = float(state[1])
            pose.pose.orientation.z = math.sin(float(state[2]) / 2.0)
            pose.pose.orientation.w = math.cos(float(state[2]) / 2.0)
            message.poses.append(pose)
        self.path_publisher.publish(message)

    @staticmethod
    def _guide_mc_layout(guide_count, horizon):
        return [
            MultiArrayDimension(
                label='guide',
                size=int(guide_count),
                stride=int(guide_count * horizon),
            ),
            MultiArrayDimension(
                label='timestep',
                size=int(horizon),
                stride=int(horizon),
            ),
        ]

    def _publish_guide_mc_diagnostics(self, result):
        areas = result.guide_mc_region_areas
        counts = result.guide_mc_sample_counts
        if areas is None or counts is None:
            return
        areas = np.asarray(areas, dtype=np.float64)
        counts = np.asarray(counts, dtype=np.int32)
        if areas.ndim != 2 or counts.shape != areas.shape:
            raise ValueError('guide MC diagnostics must have shape (guides, horizon)')
        layout = self._guide_mc_layout(*areas.shape)
        area_message = Float64MultiArray()
        area_message.layout.dim = layout
        area_message.data = areas.ravel().tolist()
        count_message = Int32MultiArray()
        count_message.layout.dim = self._guide_mc_layout(*counts.shape)
        count_message.data = counts.ravel().tolist()
        self.guide_mc_region_areas_publisher.publish(area_message)
        self.guide_mc_sample_counts_publisher.publish(count_message)

    def _publish_rollouts(self, result, stamp):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        mode_names = result.candidate_names
        guide_visualization = result.strategy == 'guide_conditioned'
        if guide_visualization:
            # Fixed colors make each robot-relative guide recognizable across
            # cycles. Stop is included for result indexing but is not drawn as
            # a guide path or rollout group.
            guide_palette = (
                (0.10, 0.85, 0.20),  #   0 deg: green
                (0.05, 0.75, 0.95),  # +45 deg: cyan
                (0.15, 0.35, 1.00),  # +90 deg: blue
                (0.95, 0.80, 0.10),  # -45 deg: yellow
                (0.90, 0.20, 0.80),  # -90 deg: magenta
            )
            mode_colors = tuple(
                guide_palette[i % len(guide_palette)]
                for i in range(len(mode_names) - 1)
            ) + ((0.75, 0.75, 0.75),)  # stop: selection data only
        grouped_visualization = (
            result.strategy == 'guide_conditioned'
            and result.control_mode_ids is not None
            and result.mode_trajectories is not None
        )
        rollout_indices = result.sampled_rollout_indices
        if rollout_indices is None:
            rollout_indices = np.arange(result.sampled_rollouts.shape[0])

        mode_peak_weights = None
        if grouped_visualization:
            mode_peak_weights = np.zeros(len(mode_names))
            for mode_index in range(len(mode_names)):
                mode_weights = result.weights[
                    result.control_mode_ids == mode_index
                ]
                if mode_weights.size:
                    mode_peak_weights[mode_index] = np.max(mode_weights)

        for marker_id, (trajectory, rollout_index) in enumerate(zip(
            result.sampled_rollouts,
            rollout_indices,
        )):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.planning_frame
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            if grouped_visualization:
                mode_index = int(result.control_mode_ids[rollout_index])
                color = mode_colors[mode_index]
                peak = mode_peak_weights[mode_index]
                normalized_strength = (
                    float(result.weights[rollout_index]) / peak
                    if peak > 0.0 else 0.0
                )
                visual_strength = math.sqrt(normalized_strength)
                marker.ns = f'guide_rollouts_{mode_names[mode_index]}'
                marker.scale.x = 0.004 + 0.012 * visual_strength
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 0.08 + 0.62 * visual_strength
            else:
                marker.ns = 'sampled_rollouts'
                marker.scale.x = 0.008
                marker.color.r = 0.25
                marker.color.g = 0.55
                marker.color.b = 1.0
                marker.color.a = 0.20
            marker.points = [
                Point(x=float(state[0]), y=float(state[1]), z=0.02)
                for state in trajectory
            ]
            array.markers.append(marker)

        if guide_visualization and result.guide_anchors is not None:
            for g, trajectory in enumerate(result.guide_anchors):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = self.planning_frame
                marker.ns, marker.id = 'guide_paths', g
                marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
                marker.scale.x = 0.025
                color = mode_colors[g] if result.guide_active_mask[g] else (1.0, 0.0, 0.0)
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 0.95
                marker.points = [Point(x=float(s[0]), y=float(s[1]), z=0.06)
                                 for s in trajectory]
                array.markers.append(marker)

        selected_marker = Marker()
        selected_marker.header.stamp = stamp
        selected_marker.header.frame_id = self.planning_frame
        selected_marker.ns = (
            'selected_path' if guide_visualization else 'selected_trajectory'
        )
        selected_marker.id = 0
        selected_marker.type = Marker.LINE_STRIP
        selected_marker.action = Marker.ADD
        selected_marker.scale.x = 0.040
        selected_marker.color.r = 1.0
        selected_marker.color.g = 0.35
        selected_marker.color.b = 0.05
        selected_marker.color.a = 1.0
        selected_marker.points = [
            Point(x=float(state[0]), y=float(state[1]), z=0.10)
            for state in result.trajectory
        ]
        array.markers.append(selected_marker)
        self.rollout_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DraMppiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_vel_publisher.publish(Twist())
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass


if __name__ == '__main__':
    main()
