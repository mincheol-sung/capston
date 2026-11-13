"""Vectorised DRA-MPPI optimiser for a planar holonomic robot."""

from dataclasses import dataclass, field
import time
import threading

import numpy as np

from .control_sampling import HaltonSplineSampler
from .common_cost import common_costs, project_path
from .dynamic_risk import (
    DynamicRiskConfig,
    make_probability_estimator,
    dynamic_risk_cost,
    check_prediction_budget,
)
from .geometry import wrap_angle
from .mecanum_model import rollout
from .observation_cost import observation_cost, prepare_target
from .gap_geometry import GapConfig


@dataclass
class MppiConfig:
    """Numerical, motion-limit, and shared application cost parameters."""

    dt: float = 0.1
    horizon_steps: int = 30
    num_rollouts: int = 512
    temperature: float = 2.0
    adaptive_temperature: bool = True
    temperature_eta_lower: float = 5.0
    temperature_eta_upper: float = 10.0
    temperature_decrease_factor: float = 0.9
    temperature_increase_factor: float = 1.2
    noise_std: tuple = (0.10, 0.10, 0.30)
    spline_knots: int = 4
    halton_scramble: bool = True
    control_min: tuple = (-0.10, -0.15, -0.80)
    control_max: tuple = (0.25, 0.15, 0.80)
    acceleration_limit: tuple = (0.60, 0.60, 1.80)
    # Mecanum inverse kinematics puts all three axes on one wheel, so the real
    # motor limit is the L1 polytope |vx| + |vy| + yaw_lever*|yaw_rate| <=
    # speed_limit_l1, which the axis box above cannot express.
    yaw_lever: float = 0.19835
    speed_limit_l1: float = 0.78
    desired_speed: float = 0.20
    tracking_path_weight: float = 8.0
    tracking_goal_weight: float = 24.0
    heading_weight: float = 8.0
    heading_lookahead: float = 0.20
    speed_weight: float = 1.78
    progress_weight: float = 8.0
    projection_window: float = 0.5
    goal_activation_distance: float = 0.5
    input_delta_weight: tuple = (0.5, 0.5, 0.055)
    observation_cost_enabled: bool = False
    observation_cost_weight: float = 3.0
    observation_half_fov: float = np.deg2rad(35.)
    observation_camera_offset: tuple = (0., 0.)
    observation_camera_yaw: float = 0.
    seed: int = 7
    strategy: str = 'standard'
    gap: GapConfig = field(default_factory=GapConfig)
    # Independent moving-sample budget; standard uses num_rollouts.
    guide_num_rollouts: int = 500
    guide_angles_deg: tuple = (0.0, 45.0, 90.0, -45.0, -90.0)
    guide_noise_std: tuple = (0.15, 0.12, 0.45)
    guide_anchor_blend: float = 0.25
    guide_switch_penalty: float = 0.25
    guide_hysteresis: float = 0.10
    guide_evaluation: str = 'split'
    guide_timeout: float = 0.18
    static_cost_enabled: bool = False
    static_cost_soft_weight: float = 8.0
    static_cost_decay: float = 0.10
    static_cost_robot_radius: float = 0.20
    dynamic_risk: DynamicRiskConfig = field(default_factory=DynamicRiskConfig)


@dataclass(frozen=True)
class MppiCostTerms:
    """Per-rollout costs, including holonomic path-heading alignment."""

    tracking: np.ndarray
    heading: np.ndarray
    speed: np.ndarray
    input_delta: np.ndarray
    risk: np.ndarray
    static: np.ndarray = None
    raw: dict = None
    weighted: dict = None
    observation: np.ndarray = None

    @property
    def total(self):
        return (
            self.tracking
            + self.heading
            + self.speed
            + self.input_delta
            + self.risk
            + (0.0 if self.static is None else self.static)
            + (0.0 if self.observation is None else self.observation)
        )


@dataclass(frozen=True)
class MppiResult:
    """Updated reference sequence and diagnostics from one MPPI iteration."""

    control: np.ndarray
    control_sequence: np.ndarray
    reference_state: np.ndarray
    reference_sequence: np.ndarray
    trajectory: np.ndarray
    sampled_controls: np.ndarray
    sampled_rollouts: np.ndarray
    rollout_costs: np.ndarray
    weights: np.ndarray
    cost_terms: MppiCostTerms
    collision_probabilities: np.ndarray
    cost: float
    max_collision_probability: float
    effective_sample_size: float
    temperature: float
    next_temperature: float
    weight_normalizer: float
    minimum_cost: float
    maximum_cost: float
    minimum_weight: float
    maximum_weight: float
    compute_time_ms: float
    strategy: str = 'standard'
    selected_mode: str = 'standard'
    control_mode_ids: np.ndarray = None
    mode_control_sequences: np.ndarray = None
    mode_trajectories: np.ndarray = None
    mode_final_costs: np.ndarray = None
    mode_selection_costs: np.ndarray = None
    mode_collision_probabilities: np.ndarray = None
    mode_temperatures: np.ndarray = None
    mode_next_temperatures: np.ndarray = None
    mode_effective_sample_sizes: np.ndarray = None
    mode_static_costs: np.ndarray = None
    sampled_rollout_indices: np.ndarray = None
    primary_risk_time_ms: float = 0.0
    final_risk_time_ms: float = 0.0
    # Guide ensemble labels, including the independent Stop candidate.
    candidate_names: tuple = ()
    guide_anchors: np.ndarray = None
    guide_active_mask: np.ndarray = None
    guide_rollout_counts: np.ndarray = None
    guide_rejection_reasons: tuple = ()
    final_static_collisions: np.ndarray = None
    safety_status: str = 'ok'
    selected_static_cost: float = 0.0
    guide_mc_region_areas: np.ndarray = None
    guide_mc_sample_counts: np.ndarray = None
    final_mc_region_areas: np.ndarray = None
    final_mc_sample_counts: np.ndarray = None
    selected_cost_raw: dict = None
    selected_cost_weighted: dict = None
    stage_ms: dict = None
    final_risk_diagnostics: object = None
    gap_diagnostics: dict = None
    gap_regions: tuple = ()


class MppiController:
    """Stateful receding-horizon DRA-MPPI controller."""

    def __init__(self, config=None):
        self.config = config or MppiConfig()
        if self.config.horizon_steps < 1 or (
            self.config.strategy == 'standard' and self.config.num_rollouts < 4
        ):
            raise ValueError('horizon_steps >= 1 and num_rollouts >= 4 are required')
        if self.config.dt <= 0.0 or self.config.temperature <= 0.0:
            raise ValueError('dt and temperature must be positive')
        if self.config.strategy not in (
            'standard', 'guide_conditioned', 'gap'
        ):
            raise ValueError('unknown MPPI strategy')
        if not (
            0.0 < self.config.temperature_eta_lower
            < self.config.temperature_eta_upper
        ):
            raise ValueError('temperature eta bounds must satisfy 0 < lower < upper')
        if not 0.0 < self.config.temperature_decrease_factor < 1.0:
            raise ValueError('temperature_decrease_factor must be in (0, 1)')
        if self.config.temperature_increase_factor <= 1.0:
            raise ValueError('temperature_increase_factor must exceed 1')
        self._nominal = np.zeros((self.config.horizon_steps, 3), dtype=np.float64)
        self._evaluation_current = np.zeros(3)
        self._observation_target = None
        self._projection_progress = None
        self._projection_path = None
        self._projection_position = None
        self.static_time_ms = 0.0
        self.representative_time_ms = 0.0
        self._compute_lock = threading.Lock()
        if self.config.guide_evaluation not in ('batch', 'split'):
            raise ValueError('guide_evaluation must be batch or split')
        if not np.isfinite(self.config.guide_timeout) or self.config.guide_timeout <= 0:
            raise ValueError('guide_timeout must be finite and positive')
        for name in ('projection_window', 'goal_activation_distance', 'heading_lookahead'):
            if not np.isfinite(getattr(self.config, name)) or getattr(self.config, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        value = np.asarray(self.config.input_delta_weight)
        if value.shape != (3,) or not np.all(np.isfinite(value)) or np.any(value < 0):
            raise ValueError('invalid input_delta_weight')
        if type(self.config.observation_cost_enabled) is not bool:
            raise ValueError('observation_cost_enabled must be bool')
        if (not np.isfinite(self.config.observation_cost_weight)
                or self.config.observation_cost_weight < 0):
            raise ValueError('observation_cost_weight must be finite and non-negative')
        if (not np.isfinite(self.config.observation_half_fov)
                or not 0 < self.config.observation_half_fov <= np.pi):
            raise ValueError('observation_half_fov must be in (0, pi]')
        offset = np.asarray(self.config.observation_camera_offset)
        if (offset.shape != (2,) or not np.all(np.isfinite(offset))
                or not np.isfinite(self.config.observation_camera_yaw)):
            raise ValueError('observation camera pose must be finite XY and yaw')
        self._temperature = float(self.config.temperature)
        self._control_min = np.asarray(self.config.control_min, dtype=np.float64)
        self._control_max = np.asarray(self.config.control_max, dtype=np.float64)
        self._acceleration_limit = np.asarray(
            self.config.acceleration_limit,
            dtype=np.float64,
        )
        if (
            self._control_min.shape != (3,)
            or self._control_max.shape != (3,)
            or self._acceleration_limit.shape != (3,)
        ):
            raise ValueError('control and acceleration limits must have three values')
        if np.any(self._control_min > self._control_max):
            raise ValueError('control_min must not exceed control_max')
        if np.any(self._control_min > 0.0) or np.any(self._control_max < 0.0):
            raise ValueError('control limits must include the all-zero stop input')
        if np.any(self._acceleration_limit < 0.0):
            raise ValueError('acceleration limits must be non-negative')
        if self.config.speed_limit_l1 <= 0.0 or self.config.yaw_lever < 0.0:
            raise ValueError('speed_limit_l1 > 0 and yaw_lever >= 0 are required')
        if self.config.heading_weight < 0.0:
            raise ValueError('heading_weight must be non-negative')
        if self.config.heading_lookahead <= 0.0:
            raise ValueError('heading_lookahead must be positive')
        if self.config.speed_weight < 0.0:
            raise ValueError('speed_weight must be non-negative')
        if self.config.static_cost_soft_weight < 0.0:
            raise ValueError('static_cost_soft_weight must be non-negative')
        if self.config.static_cost_decay <= 0.0:
            raise ValueError('static_cost_decay must be positive')
        if self.config.static_cost_robot_radius < 0.0:
            raise ValueError('static_cost_robot_radius must be non-negative')
        self._sample_noise = None
        if self.config.strategy == 'standard':
            sampler = HaltonSplineSampler(
                horizon_steps=self.config.horizon_steps,
                control_dimension=3,
                knot_points=self.config.spline_knots,
                noise_std=self.config.noise_std,
                scramble=self.config.halton_scramble,
                seed=self.config.seed,
            )
            # The same low-discrepancy basis is reused each cycle around the
            # shifted nominal sequence, as in the reference DRA-MPPI structure.
            self._sample_noise = sampler.sample(self.config.num_rollouts - 3)
        self._guide_optimizer = None
        if self.config.strategy == 'guide_conditioned':
            from .guide_conditioned import GuideConditionedOptimizer
            self._guide_optimizer = GuideConditionedOptimizer(self)
        elif self.config.strategy == 'gap':
            from .gap_conditioned import GapConditionedOptimizer
            self._guide_optimizer = GapConditionedOptimizer(self)
        self._risk_estimator = make_probability_estimator(self.config.dynamic_risk,
            reject_underresolved=self.config.dynamic_risk.mc_reject_underresolved,
            num_samples=self.config.dynamic_risk.mc_samples,
            minimum_sigma=self.config.dynamic_risk.minimum_sigma,
            seed=self.config.dynamic_risk.seed,
            adaptive_halton=self.config.dynamic_risk.adaptive_halton_mc,
            sample_density=self.config.dynamic_risk.mc_sample_density,
            min_samples=self.config.dynamic_risk.mc_samples_min,
            max_samples=self.config.dynamic_risk.mc_samples_max,
        )
        # Standard representatives and terminal alignment use this estimator.
        # Keep standard's historical MC budget when final_mode is inherited.
        self._final_risk_estimator = make_probability_estimator(
            self.config.dynamic_risk, final=True,
            reject_underresolved=self.config.dynamic_risk.mc_reject_underresolved,
            num_samples=self.config.dynamic_risk.mc_samples,
            minimum_sigma=self.config.dynamic_risk.minimum_sigma,
            seed=self.config.dynamic_risk.seed,
            adaptive_halton=self.config.dynamic_risk.adaptive_halton_mc,
            sample_density=self.config.dynamic_risk.mc_sample_density,
            min_samples=self.config.dynamic_risk.mc_samples_min,
            max_samples=self.config.dynamic_risk.mc_samples_max)

    @property
    def risk_diagnostics(self):
        """Diagnostics from the most recent Monte Carlo CP evaluation."""
        return self._risk_estimator.last_diagnostics

    def reset(self):
        """Clear the warm-start control sequence."""
        self._nominal.fill(0.0)
        self._observation_target = None
        self._projection_progress = None
        self._projection_path = None
        self._projection_position = None
        self._temperature = float(self.config.temperature)
        if self._guide_optimizer is not None:
            self._guide_optimizer.reset()

    def close(self):
        """Release persistent guide-risk workers."""
        if self._guide_optimizer is not None:
            self._guide_optimizer.shutdown()


    def compute(self, *args, **kwargs):
        # Single owner, including on exceptions/timeouts. Never overlap cycles.
        if not self._compute_lock.acquire(blocking=False):
            raise RuntimeError('planner cycle already running')
        try:
            return self._compute_impl(*args, **kwargs)
        finally:
            self._compute_lock.release()

    def _compute_impl(
        self,
        state,
        reference_path,
        current_control=None,
        dynamic_predictions=None,
        visualization_rollouts=40,
        guide_path=None,
        distance_field=None,
        source_to_grid=None,
        observation_target=None,
        gap_scans=None,
    ):
        """Optimise and return the next planar pose/velocity reference."""
        started = time.perf_counter()
        try:
            check_prediction_budget(self.config.dynamic_risk, dynamic_predictions)
        except ValueError:
            self.reset()
            raise
        state = np.array(state, dtype=np.float64, copy=True)
        path = np.array(reference_path, dtype=np.float64, copy=True)
        if state.shape != (3,):
            raise ValueError('state must be [x, y, yaw]')
        if path.ndim != 2 or path.shape[1] < 2 or path.shape[0] == 0:
            raise ValueError('reference_path must contain at least one x/y point')
        path = path[:, :2]
        guide = path if guide_path is None else np.array(
            guide_path,
            dtype=np.float64,
        )
        if guide.ndim != 2 or guide.shape[1] < 2 or guide.shape[0] == 0:
            raise ValueError('guide_path must contain at least one x/y point')
        guide = guide[:, :2]
        current = np.zeros(3, dtype=np.float64) if current_control is None else (
            np.array(current_control, dtype=np.float64, copy=True)
        )
        if current.shape != (3,):
            raise ValueError('current_control must be [vx, vy, yaw_rate]')
        if not np.all(np.isfinite(np.r_[state, current, path.ravel(), guide.ravel()])):
            raise ValueError('controller inputs must be finite')
        # Use the complete forward/global path for every strategy's objective;
        # a cropped local endpoint is not the actual goal.
        path = guide
        self._evaluation_current = current.copy()
        self._observation_target = (
            prepare_target(observation_target, self.config.horizon_steps)
            if self.config.observation_cost_enabled else None)
        self.static_time_ms = 0.0
        self.representative_time_ms = 0.0
        same_path = (self._projection_path is not None
                     and np.array_equal(path, self._projection_path))
        window = self.config.projection_window
        if same_path and self._projection_position is not None:
            window = max(window, 2*np.linalg.norm(state[:2]-self._projection_position))
        self._projection_progress = float(project_path(
            state[None, None, :2], path, self.config.heading_lookahead, window,
            self._projection_progress if same_path else None)[1][0, 0])
        self._projection_path = path.copy()
        self._projection_position = state[:2].copy()

        if self.config.strategy == 'gap':
            return self._guide_optimizer.compute(
                state, path, current, dynamic_predictions, visualization_rollouts,
                distance_field, source_to_grid, started, guide[-1], scans=gap_scans)
        if self.config.strategy == 'guide_conditioned':
            return self._guide_optimizer.compute(
                state, path, current, dynamic_predictions,
                visualization_rollouts, distance_field, source_to_grid, started,
                guide[-1],
            )
        return self._compute_standard(
            state,
            path,
            guide,
            current,
            dynamic_predictions,
            visualization_rollouts,
            distance_field,
            source_to_grid,
            started,
        )

    def _compute_standard(
        self,
        state,
        path,
        guide,
        current,
        dynamic_predictions,
        visualization_rollouts,
        distance_field,
        source_to_grid,
        started,
    ):
        """Run the unchanged single-distribution reference MPPI update."""
        cfg = self.config
        controls = np.empty(
            (cfg.num_rollouts, cfg.horizon_steps, 3),
            dtype=np.float64,
        )
        controls[0] = self._nominal
        controls[1] = self._make_stopping_control(current)
        controls[2] = self._make_path_guide(state, guide, current)
        controls[3:] = self._nominal[None, ...] + self._sample_noise
        controls = self._constrain_controls(controls, current)
        # Keep one deterministic, acceleration-limited stopping candidate.
        # Reapply it after projection so noise/nominal handling cannot alter
        # the braking profile derived from the measured current velocity.
        controls[1] = self._make_stopping_control(current)
        trajectories = rollout(state, controls, cfg.dt)
        terms, risk_evaluation = self._evaluate_rollouts(
            state,
            path,
            controls,
            trajectories,
            dynamic_predictions,
            distance_field=distance_field,
            source_to_grid=source_to_grid,
        )
        costs = terms.total
        primary_diagnostics = self._risk_estimator.last_diagnostics

        weight_data = self._importance_weights(costs, self._temperature)
        if weight_data is None:
            self.reset()
            raise RuntimeError('all MPPI rollout costs are non-finite')
        finite, minimum_cost, maximum_cost, weights, weight_normalizer = (
            weight_data
        )
        temperature = self._temperature
        self._temperature = self._adapt_temperature(
            self._temperature,
            weight_normalizer,
        )

        # Paper MPPI update: weighted average of complete candidate control
        # sequences, followed by input/rate projection.
        updated = np.sum(weights[:, None, None] * controls, axis=0)
        updated = self._constrain_controls(updated[None, ...], current)[0]
        final_started = time.perf_counter()
        final_controls = np.stack((updated, self._make_stopping_control(current)))
        final_paths = rollout(state, final_controls, cfg.dt)
        final_terms, final_risk = self._evaluate_rollouts(
            state, path, final_controls, final_paths, dynamic_predictions,
            distance_field=distance_field, source_to_grid=source_to_grid,
            risk_estimator=self._final_risk_estimator)
        safe = (np.isfinite(final_terms.total)
                & (final_risk.maximum_probability <= cfg.dynamic_risk.hard_threshold))
        selected = 0 if safe[0] else 1
        updated = final_controls[selected]
        selected_trajectory = final_paths[selected]
        self.representative_time_ms = (time.perf_counter()-final_started)*1000.
        first_control = updated[0].copy()
        first_state = selected_trajectory[0].copy()

        # Receding-horizon warm start for the next iteration.
        self._nominal[:-1] = updated[1:]
        self._nominal[-1] = updated[-1]

        visual_count = max(0, min(int(visualization_rollouts), cfg.num_rollouts))
        indices = np.argsort(costs)[:visual_count]
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        final_diagnostics = self._final_risk_estimator.last_diagnostics
        self._risk_estimator.last_diagnostics = primary_diagnostics
        return MppiResult(
            control=first_control,
            control_sequence=updated.copy(),
            reference_state=first_state,
            reference_sequence=np.concatenate(
                (selected_trajectory, updated),
                axis=1,
            ),
            trajectory=selected_trajectory,
            sampled_controls=controls,
            sampled_rollouts=trajectories[indices],
            rollout_costs=costs,
            weights=weights,
            cost_terms=terms,
            collision_probabilities=risk_evaluation.collision_probability,
            # Avoid the IEEE 0 * inf -> NaN case when only a subset of
            # rollouts has a finite cost and therefore a non-zero weight.
            cost=float(final_terms.total[selected]),
            max_collision_probability=float(final_risk.maximum_probability[selected]),
            effective_sample_size=float(1.0 / np.sum(weights * weights)),
            temperature=float(temperature),
            next_temperature=float(self._temperature),
            weight_normalizer=weight_normalizer,
            minimum_cost=minimum_cost,
            maximum_cost=maximum_cost,
            minimum_weight=float(np.min(weights)),
            maximum_weight=float(np.max(weights)),
            compute_time_ms=elapsed_ms,
            sampled_rollout_indices=indices.copy(),
            safety_status='ok' if safe[selected] else 'no_safe_candidate_hold',
            selected_mode='standard' if selected == 0 else 'stop',
            selected_static_cost=float(final_terms.static[selected]),
            primary_risk_time_ms=float(
                primary_diagnostics.compute_time_ms
            ),
            final_risk_diagnostics=final_diagnostics,
            final_risk_time_ms=final_diagnostics.compute_time_ms,
            final_mc_region_areas=final_diagnostics.region_areas.copy(),
            final_mc_sample_counts=final_diagnostics.sample_counts.copy(),
            selected_cost_raw={key: float(value[selected]) for key, value in final_terms.raw.items()},
            selected_cost_weighted={key: float(value[selected]) for key, value in final_terms.weighted.items()},
        )


    def _evaluate_rollouts(
        self,
        state,
        path,
        controls,
        trajectories,
        dynamic_predictions,
        distance_field=None,
        source_to_grid=None,
        cache_risk_samples=False,
        reuse_risk_samples=False,
        risk_estimator=None,
    ):
        tracking_cost, heading_cost, speed_cost, input_delta_cost, raw, weighted = common_costs(
            self.config, state, path, controls, trajectories,
            self._evaluation_current, self._projection_progress, return_components=True)
        observation_raw, observation_weighted = observation_cost(
            self.config, trajectories, self._observation_target)
        raw['observation'] = observation_raw
        weighted['observation'] = observation_weighted
        risk_evaluation = self._evaluate_dynamic_risk(
            trajectories,
            dynamic_predictions,
            estimator=risk_estimator,
            cache_samples=cache_risk_samples,
            reuse_last_samples=reuse_risk_samples,
        )
        static_cost, static_collision = self._static_obstacle_cost(
            state,
            trajectories,
            distance_field,
            source_to_grid,
        )
        return MppiCostTerms(
            tracking=tracking_cost,
            heading=heading_cost,
            speed=speed_cost,
            input_delta=input_delta_cost,
            risk=risk_evaluation.cost,
            static=static_cost,
            observation=observation_weighted,
            raw={**raw, 'dynamic_probability': np.mean(risk_evaluation.collision_probability, axis=1),
                 'dynamic_exceedances': np.sum(risk_evaluation.collision_probability >
                     self.config.dynamic_risk.hard_threshold, axis=1),
                 'static': self._last_static_raw, 'static_collision': static_collision.astype(float)},
            weighted={**weighted, 'dynamic_probability': self.config.dynamic_risk.soft_weight*np.mean(risk_evaluation.collision_probability, axis=1),
                      'dynamic_exceedances': self.config.dynamic_risk.hard_penalty*np.sum(risk_evaluation.collision_probability > self.config.dynamic_risk.hard_threshold, axis=1),
                      'static': static_cost},
        ), risk_evaluation

    def _evaluate_dynamic_risk(
        self,
        trajectories,
        dynamic_predictions,
        estimator=None,
        cache_samples=False,
        reuse_last_samples=False,
    ):
        """Evaluate only dynamic risk with an optional isolated MC sampler."""
        return dynamic_risk_cost(
            trajectories[..., :2],
            dynamic_predictions,
            self.config.dynamic_risk,
            estimator=self._risk_estimator if estimator is None else estimator,
            cache_samples=cache_samples,
            reuse_last_samples=reuse_last_samples,
        )

    def _static_obstacle_cost(
        self,
        state,
        trajectories,
        distance_field,
        source_to_grid,
    ):
        """Return per-rollout static cost and hard-collision flags.

        The hinge-square mean uses a conservative footprint-clearance bound.
        Swept disc collisions with occupied/unknown/out-of-map space are invalid
        (infinite cost). Grid/sample padding covers the discrete distance query.
        """
        rollout_count = trajectories.shape[0]
        self._last_static_raw = np.zeros(rollout_count)
        if not self.config.static_cost_enabled:
            return (
                np.zeros(rollout_count, dtype=np.float64),
                np.zeros(rollout_count, dtype=bool),
            )
        if distance_field is None:
            raise ValueError('static cost is enabled but no distance field was provided')

        started = time.perf_counter()
        map_clearance = distance_field.state_clearances(
            trajectories[..., :2],
            source_to_grid=source_to_grid,
        )
        footprint_clearance = (
            map_clearance - self.config.static_cost_robot_radius
        )
        hard_collision = distance_field.trajectory_collisions(
            trajectories[..., :2], np.asarray(state)[:2],
            self.config.static_cost_robot_radius, source_to_grid=source_to_grid)
        self._last_static_raw = np.mean(
            np.maximum(0., 1.-footprint_clearance/self.config.static_cost_decay)**2,
            axis=1,
        )
        soft_cost = self.config.static_cost_soft_weight*self._last_static_raw
        soft_cost[hard_collision] = np.inf
        self.static_time_ms += (time.perf_counter()-started)*1000.
        return soft_cost, hard_collision

    def _importance_weights(self, costs, temperature):
        finite = np.isfinite(costs)
        if not np.any(finite):
            return None
        minimum_cost = float(np.min(costs[finite]))
        maximum_cost = float(np.max(costs[finite]))
        scaled = np.full_like(costs, -np.inf)
        scaled[finite] = -(costs[finite] - minimum_cost) / max(
            float(temperature),
            1e-6,
        )
        unnormalized = np.exp(scaled)
        normalizer = float(np.sum(unnormalized))
        weights = unnormalized / max(normalizer, 1e-12)
        return finite, minimum_cost, maximum_cost, weights, normalizer

    def _adapt_temperature(self, temperature, normalizer):
        value = float(temperature)
        if not self.config.adaptive_temperature:
            return value
        if normalizer > self.config.temperature_eta_upper:
            return value * self.config.temperature_decrease_factor
        if normalizer < self.config.temperature_eta_lower:
            return value * self.config.temperature_increase_factor
        return value


    def _constrain_controls(self, controls, current_control):
        values = np.clip(controls, self._control_min, self._control_max).copy()
        previous = self._project_wheel_limit(np.broadcast_to(
            np.clip(current_control, self._control_min, self._control_max),
            (values.shape[0], 3)).copy())
        max_delta = self._acceleration_limit * self.config.dt
        for step in range(values.shape[1]):
            target = self._project_wheel_limit(values[:, step])
            delta = target - previous
            fractions = np.divide(max_delta, np.abs(delta),
                                  out=np.full_like(delta, np.inf),
                                  where=np.abs(delta) > 1e-12)
            scale = np.minimum(1., np.min(fractions, axis=1))
            values[:, step] = previous + scale[:, None]*delta
            previous = values[:, step]
        return values

    def _make_stopping_control(self, current_control):
        """Return the feasible velocity sequence for braking to rest.

        Stop is evaluated with the same kinematic rollout as every moving
        candidate. Each body-velocity component approaches zero by at most
        ``acceleration_limit * dt`` per step, so dynamic-risk evaluation sees
        the distance and rotation travelled during the braking transient.
        """
        zeros = np.zeros(
            (1, self.config.horizon_steps, 3),
            dtype=np.float64,
        )
        return self._constrain_controls(zeros, current_control)[0]

    def _project_wheel_limit(self, values):
        """Scale [vx, vy, yaw_rate] onto the mecanum wheel-speed polytope.

        The four wheel speeds are ``(vx -+ vy -+ k*yaw_rate) / r``, so every
        axis loads the same motor and the binding constraint is the L1 sum
        ``|vx| + |vy| + k*|yaw_rate| <= speed_limit_l1``. The axis box cannot
        express that coupling: its corner asks for wheel speeds far past the
        motor limit, and the base driver then scales the command down on its
        own, leaving the rollout model predicting a trajectory the robot never
        drives. Scaling uniformly here keeps the command direction and matches
        what the driver does when it saturates.

        The constraint routine projects its target here before interpolating
        from the previous feasible input to satisfy rate limits as well.
        An initially infeasible measured input is projected first; only that
        initial correction can exceed a rate limit.
        """
        load = (
            np.abs(values[:, 0])
            + np.abs(values[:, 1])
            + self.config.yaw_lever * np.abs(values[:, 2])
        )
        scale = np.minimum(
            1.0,
            self.config.speed_limit_l1 / np.maximum(load, 1e-12),
        )
        return values * scale[:, None]

    def _make_path_guide(self, state, path, current_control):
        """Build one feasible deterministic control sequence along the path."""
        points = np.asarray(path, dtype=np.float64)[:, :2]
        # Consecutive duplicates have zero arc length and no tangent. Removing
        # them affects only the guide construction, not tracking/heading costs.
        if points.shape[0] > 1:
            keep = np.concatenate((
                np.array([True]),
                np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9,
            ))
            points = points[keep]

        controls = np.zeros((self.config.horizon_steps, 3), dtype=np.float64)
        guide_state = np.asarray(state, dtype=np.float64).copy()
        previous = np.array(current_control, dtype=np.float64, copy=True).copy()
        dt = self.config.dt

        if points.shape[0] == 1:
            for step in range(self.config.horizon_steps):
                world_delta = points[0] - guide_state[:2]
                distance = float(np.linalg.norm(world_delta))
                desired_speed = self._guide_braking_speed(distance)
                world_velocity = self._bounded_guide_velocity(
                    world_delta,
                    desired_speed,
                    dt,
                )
                raw = self._guide_body_control(
                    world_velocity,
                    guide_state[2],
                    guide_state[2],
                    dt,
                )
                control = self._constrain_controls(
                    raw[None, None, :],
                    previous,
                )[0, 0]
                controls[step] = control
                guide_state = rollout(guide_state, control[None, :], dt)[0]
                previous = control
            return controls

        segments = np.diff(points, axis=0)
        segment_lengths = np.linalg.norm(segments, axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        progress = self._project_path_progress(
            guide_state[:2],
            points,
            segments,
            segment_lengths,
            cumulative,
        )
        total_length = float(cumulative[-1])

        for step in range(self.config.horizon_steps):
            goal_distance = float(np.linalg.norm(points[-1] - guide_state[:2]))
            remaining = max(total_length - progress, goal_distance)
            desired_speed = self._guide_braking_speed(remaining)
            target_progress = min(total_length, progress + desired_speed * dt)
            target, target_heading = self._path_target_at_progress(
                target_progress,
                points,
                segments,
                segment_lengths,
                cumulative,
            )
            world_velocity = self._bounded_guide_velocity(
                target - guide_state[:2],
                desired_speed,
                dt,
            )
            raw = self._guide_body_control(
                world_velocity,
                guide_state[2],
                target_heading,
                dt,
            )
            control = self._constrain_controls(
                raw[None, None, :],
                previous,
            )[0, 0]
            controls[step] = control
            guide_state = rollout(guide_state, control[None, :], dt)[0]
            previous = control
            progress = target_progress
        return controls

    def _guide_braking_speed(self, remaining_distance):
        translational_acceleration = float(np.min(self._acceleration_limit[:2]))
        if translational_acceleration <= 0.0:
            return 0.0
        braking_speed = np.sqrt(
            2.0 * translational_acceleration * max(remaining_distance, 0.0)
        )
        return min(float(self.config.desired_speed), float(braking_speed))

    @staticmethod
    def _bounded_guide_velocity(world_delta, desired_speed, dt):
        distance = float(np.linalg.norm(world_delta))
        if distance <= 1e-12 or desired_speed <= 0.0:
            return np.zeros(2, dtype=np.float64)
        speed = min(float(desired_speed), distance / dt)
        return np.asarray(world_delta, dtype=np.float64) * (speed / distance)

    @staticmethod
    def _guide_body_control(world_velocity, yaw, target_heading, dt):
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        yaw_error = float(wrap_angle(target_heading - yaw))
        return np.array(
            [
                cos_yaw * world_velocity[0] + sin_yaw * world_velocity[1],
                -sin_yaw * world_velocity[0] + cos_yaw * world_velocity[1],
                yaw_error / dt,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _project_path_progress(
        position,
        points,
        segments,
        segment_lengths,
        cumulative,
    ):
        relative = position[None, :] - points[:-1]
        fractions = np.clip(
            np.sum(relative * segments, axis=1) / (segment_lengths ** 2),
            0.0,
            1.0,
        )
        projections = points[:-1] + fractions[:, None] * segments
        differences = projections - position[None, :]
        segment = int(np.argmin(np.sum(differences * differences, axis=1)))
        return float(
            cumulative[segment] + fractions[segment] * segment_lengths[segment]
        )

    @staticmethod
    def _path_target_at_progress(
        progress,
        points,
        segments,
        segment_lengths,
        cumulative,
    ):
        segment = int(np.searchsorted(cumulative, progress, side='right') - 1)
        segment = int(np.clip(segment, 0, segments.shape[0] - 1))
        fraction = np.clip(
            (progress - cumulative[segment]) / segment_lengths[segment],
            0.0,
            1.0,
        )
        target = points[segment] + fraction * segments[segment]
        heading = float(np.arctan2(segments[segment, 1], segments[segment, 0]))
        return target, heading


    def _nearest_path_geometry(self, trajectories, path):
        """Project positions onto path segments.

        Returns squared distance, original segment index, closest point and a
        distance-lookahead path heading for every queried position.  Using the
        continuous polyline avoids the artificial error created between sparse
        waypoints by a nearest-waypoint query.
        """
        positions = np.asarray(trajectories, dtype=np.float64)[..., :2]
        path_points = np.asarray(path, dtype=np.float64)
        query_shape = positions.shape[:-1]
        flat_positions = positions.reshape(-1, 2)

        segments = np.diff(path_points, axis=0)
        segment_squared = np.sum(segments * segments, axis=1)
        valid_indices = np.flatnonzero(segment_squared > 1e-18)
        if valid_indices.size == 0:
            closest = np.broadcast_to(
                path_points[0],
                (flat_positions.shape[0], 2),
            ).copy()
            difference = flat_positions - closest
            squared = np.sum(difference * difference, axis=1)
            indices = np.zeros(flat_positions.shape[0], dtype=np.int64)
            headings = np.zeros(flat_positions.shape[0], dtype=np.float64)
        else:
            valid_segments = segments[valid_indices]
            valid_squared = segment_squared[valid_indices]
            starts = path_points[valid_indices]
            projection_numerator = (
                flat_positions @ valid_segments.T
                - np.sum(starts * valid_segments, axis=1)[None, :]
            )
            fractions = np.clip(
                projection_numerator / valid_squared[None, :],
                0.0,
                1.0,
            )
            # Squared point-to-projection distance without constructing the
            # large (rollout, horizon, segment, xy) tensor.
            relative_squared = (
                np.sum(flat_positions * flat_positions, axis=1)[:, None]
                + np.sum(starts * starts, axis=1)[None, :]
                - 2.0 * (flat_positions @ starts.T)
            )
            squared_all = (
                relative_squared
                - 2.0 * fractions * projection_numerator
                + fractions * fractions * valid_squared[None, :]
            )
            np.maximum(squared_all, 0.0, out=squared_all)
            selected_valid = np.argmin(squared_all, axis=1)
            rows = np.arange(flat_positions.shape[0])
            closest = (
                starts[selected_valid]
                + fractions[rows, selected_valid, None]
                * valid_segments[selected_valid]
            )
            squared = squared_all[rows, selected_valid]
            indices = valid_indices[selected_valid]
            headings = self._path_headings(path_points)[indices]

        return (
            squared.reshape(query_shape),
            indices.reshape(query_shape),
            closest.reshape((*query_shape, 2)),
            headings.reshape(query_shape),
        )

    def _path_headings(self, path):
        """Return a distance-lookahead tangent heading for every path point."""
        segments = np.diff(path, axis=0)
        segment_lengths = np.linalg.norm(segments, axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        target_distances = np.minimum(
            cumulative + self.config.heading_lookahead,
            cumulative[-1],
        )
        segment_indices = np.searchsorted(
            cumulative,
            target_distances,
            side='right',
        ) - 1
        segment_indices = np.clip(segment_indices, 0, path.shape[0] - 2)
        selected_lengths = segment_lengths[segment_indices]
        fractions = np.divide(
            target_distances - cumulative[segment_indices],
            selected_lengths,
            out=np.zeros_like(target_distances),
            where=selected_lengths > 1e-9,
        )
        targets = (
            path[segment_indices]
            + fractions[:, None] * segments[segment_indices]
        )
        tangents = targets - path

        # The final point and repeated points can have a zero tangent. Borrow
        # the closest valid path direction instead of using atan2(0, 0).
        valid = np.linalg.norm(tangents, axis=1) > 1e-9
        valid_indices = np.flatnonzero(valid)
        if valid_indices.size == 0:
            return np.zeros(path.shape[0], dtype=np.float64)
        invalid_indices = np.flatnonzero(~valid)
        for index in invalid_indices:
            nearest_valid = valid_indices[
                np.argmin(np.abs(valid_indices - index))
            ]
            tangents[index] = tangents[nearest_valid]
        return np.arctan2(tangents[:, 1], tangents[:, 0])
