"""Monte Carlo joint collision probability for DRA-MPPI rollouts."""

from dataclasses import dataclass, fields
import time

import numpy as np
from scipy.stats import qmc


@dataclass(frozen=True)
class PredictionSet:
    """Multi-modal obstacle predictions aligned to the MPPI horizon.

    Canonical shapes are ``means=(O,M,T,2)``,
    ``covariances=(O,M,T,2,2)``, mode weights ``(O,M)`` or ``(O,M,T)``,
    and ``times=(T,)``.
    Older single-mode arrays are promoted to one mode for compatibility with
    core callers and recorded inputs created before the multi-modal message.
    """

    means: np.ndarray
    covariances: np.ndarray
    times: np.ndarray
    mode_weights: np.ndarray = None
    obstacle_radii: np.ndarray = None

    def __post_init__(self):
        means = np.asarray(self.means, dtype=np.float64)
        covariances = np.asarray(self.covariances, dtype=np.float64)
        times = np.asarray(self.times, dtype=np.float64)
        if means.ndim == 3:
            means = means[:, None, :, :]
        if covariances.ndim == 4:
            covariances = covariances[:, None, :, :, :]
        if means.ndim != 4 or means.shape[-1] != 2:
            raise ValueError('means must have shape (O, M, T, 2)')
        if covariances.shape != (*means.shape[:-1], 2, 2):
            raise ValueError('covariances must have shape (O, M, T, 2, 2)')
        if times.shape != (means.shape[2],):
            raise ValueError('times must have one value per prediction step')
        if not np.all(np.isfinite(times)) or np.any(np.diff(times) < 0.0):
            raise ValueError('times must be finite and non-decreasing')
        if not np.all(np.isfinite(means)) or not np.all(np.isfinite(covariances)):
            raise ValueError('prediction values must be finite')
        if (not np.allclose(covariances, np.swapaxes(covariances, -1, -2),
                            atol=1e-10, rtol=1e-7)
                or np.any(np.linalg.eigvalsh(covariances) < -1e-10)):
            raise ValueError('prediction covariance must be symmetric positive semidefinite')

        obstacle_count, mode_count = means.shape[:2]
        if self.mode_weights is None:
            weights = np.ones((obstacle_count, mode_count), dtype=np.float64)
        else:
            weights = np.asarray(self.mode_weights, dtype=np.float64)
        if weights.shape not in (
                (obstacle_count, mode_count),
                (obstacle_count, mode_count, means.shape[2])):
            raise ValueError('mode_weights must have shape (O,M) or (O,M,T)')
        if np.any(weights < 0.0) or not np.all(np.isfinite(weights)):
            raise ValueError('mode_weights must be finite and non-negative')
        totals = np.sum(weights, axis=1, keepdims=True)
        if obstacle_count and (np.any(totals <= 0.0) or not np.all(np.isfinite(totals))):
            raise ValueError('each obstacle needs at least one positive mode weight')
        if obstacle_count:
            weights = weights / totals

        radii = self.obstacle_radii
        if radii is not None:
            radii = np.asarray(radii, dtype=np.float64)
            if radii.shape != (obstacle_count,):
                raise ValueError('obstacle_radii must have shape (O,)')
            if np.any(radii <= 0.0) or not np.all(np.isfinite(radii)):
                raise ValueError('obstacle_radii must be finite and positive')

        means, covariances, times, weights = [np.array(a, copy=True) for a in (means, covariances, times, weights)]
        radii = None if radii is None else np.array(radii, copy=True)
        for array in (means, covariances, times, weights, radii):
            if array is not None:
                array.setflags(write=False)
        object.__setattr__(self, 'means', means)
        object.__setattr__(self, 'covariances', covariances)
        object.__setattr__(self, 'times', times)
        object.__setattr__(self, 'mode_weights', weights)
        object.__setattr__(self, 'obstacle_radii', radii)

    @property
    def empty(self):
        return self.means.shape[0] == 0


@dataclass(frozen=True)
class DynamicRiskConfig:
    """Shared collision model, primary/final integration and input budget."""

    mode: str = 'monte_carlo'
    # inherit preserves the primary integration method for existing callers.
    final_mode: str = 'inherit'
    max_obstacles: int = 0  # 0 disables the budget guard; never truncate tracks.
    soft_weight: float = 18.0
    robot_radius: float = 0.22
    obstacle_radius: float = 0.25
    minimum_sigma: float = 0.08
    mc_samples: int = 2048
    adaptive_halton_mc: bool = False
    mc_sample_density: float = 124.0
    mc_samples_min: int = 512
    mc_samples_max: int = 4096
    guide_parallel_workers: int = 2
    guide_mc_sample_density: float = 124.0
    guide_mc_samples_min: int = 128
    guide_mc_samples_max: int = 1024
    final_mc_sample_density: float = 124.0
    final_mc_samples_min: int = 256
    final_mc_samples_max: int = 4096
    mc_reject_underresolved: bool = True
    hard_threshold: float = 0.05
    hard_penalty: float = 100000.0
    seed: int = 17

    def __post_init__(self):
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, (int, float)) and not np.isfinite(value):
                raise ValueError(f'{item.name} must be finite')
        if self.mode not in ('off', 'monte_carlo', 'gaussian_disk'):
            raise ValueError('dynamic risk mode must be off, monte_carlo or gaussian_disk')
        if self.final_mode not in ('inherit', 'monte_carlo', 'gaussian_disk'):
            raise ValueError('final_mode must be inherit, monte_carlo or gaussian_disk')
        if (isinstance(self.max_obstacles, bool)
                or not isinstance(self.max_obstacles, (int, np.integer))
                or self.max_obstacles < 0):
            raise ValueError('max_obstacles must be a non-negative integer')
        if self.soft_weight < 0.0 or self.hard_penalty < 0.0:
            raise ValueError('risk cost weights must be non-negative')
        if self.robot_radius < 0.0 or self.obstacle_radius < 0.0:
            raise ValueError('collision radii must be non-negative')
        if self.robot_radius + self.obstacle_radius <= 0.0:
            raise ValueError('combined collision radius must be positive')
        if self.minimum_sigma <= 0.0 or self.mc_samples < 1:
            raise ValueError('minimum_sigma and mc_samples must be positive')
        if self.mc_sample_density <= 0.0:
            raise ValueError('mc_sample_density must be positive')
        if self.mc_samples_min < 1 or self.mc_samples_max < self.mc_samples_min:
            raise ValueError(
                'mc_samples_min >= 1 and mc_samples_max >= mc_samples_min '
                'are required'
            )
        if self.guide_parallel_workers < 1:
            raise ValueError('guide_parallel_workers must be positive')
        if self.guide_mc_sample_density <= 0.0:
            raise ValueError('guide_mc_sample_density must be positive')
        if (self.guide_mc_samples_min < 1
                or self.guide_mc_samples_max < self.guide_mc_samples_min):
            raise ValueError(
                'guide_mc_samples_min >= 1 and guide_mc_samples_max >= '
                'guide_mc_samples_min are required'
            )
        if self.final_mc_sample_density <= 0.0:
            raise ValueError('final_mc_sample_density must be positive')
        if (self.final_mc_samples_min < 1
                or self.final_mc_samples_max < self.final_mc_samples_min):
            raise ValueError(
                'final_mc_samples_min >= 1 and final_mc_samples_max >= '
                'final_mc_samples_min are required'
            )
        if not 0.0 <= self.hard_threshold <= 1.0:
            raise ValueError('hard_threshold must be between zero and one')


@dataclass(frozen=True)
class RiskEvaluation:
    """Risk cost and time-resolved joint collision probabilities."""

    cost: np.ndarray
    collision_probability: np.ndarray

    @property
    def maximum_probability(self):
        if self.collision_probability.shape[1] == 0:
            return np.zeros(self.collision_probability.shape[0])
        return np.max(self.collision_probability, axis=1)


@dataclass(frozen=True)
class MonteCarloDiagnostics:
    """Sampling work performed by the most recent probability evaluation."""

    sampling_mode: str
    region_areas: np.ndarray
    sample_counts: np.ndarray
    total_samples: int
    compute_time_ms: float
    empty_circle_count: int
    clipped_marginal_count: int
    raw_marginal_minimum: float
    raw_marginal_maximum: float
    candidate_sample_comparisons: int = 0
    underresolved_steps: int = 0
    peak_pairwise_bytes: int = 0
    analytic_evaluations: int = 0
    quadrature_evaluations: int = 0
    unresolved_integrals: int = 0
    error_estimate_max: float = 0.0


class MonteCarloJointCollisionProbability:
    """Approximate joint CP using samples shared by all rollouts per step."""

    def __init__(
        self,
        num_samples=2048,
        minimum_sigma=0.08,
        seed=17,
        adaptive_halton=False,
        sample_density=2048.0,
        min_samples=512,
        max_samples=4096,
        reject_underresolved=False,
    ):
        self.reject_underresolved = bool(reject_underresolved)
        self.num_samples = int(num_samples)
        self.minimum_sigma = float(minimum_sigma)
        self.adaptive_halton = bool(adaptive_halton)
        self.sample_density = float(sample_density)
        self.min_samples = int(min_samples)
        self.max_samples = int(max_samples)
        if self.num_samples < 1:
            raise ValueError('num_samples must be positive')
        if self.minimum_sigma <= 0.0:
            raise ValueError('minimum_sigma must be positive')
        if self.sample_density <= 0.0:
            raise ValueError('sample_density must be positive')
        if self.min_samples < 1 or self.max_samples < self.min_samples:
            raise ValueError(
                'min_samples >= 1 and max_samples >= min_samples are required'
            )
        self._rng = np.random.default_rng(int(seed))
        self._halton = (
            qmc.Halton(d=2, scramble=True, seed=int(seed))
            if self.adaptive_halton
            else None
        )
        self._quadrature_cache = None
        self.reset_diagnostics()

    def reset_diagnostics(self):
        self.last_diagnostics = MonteCarloDiagnostics(
            sampling_mode=self.sampling_mode,
            region_areas=np.empty(0, dtype=np.float64),
            sample_counts=np.empty(0, dtype=np.int64),
            total_samples=0,
            compute_time_ms=0.0,
            empty_circle_count=0,
            clipped_marginal_count=0,
            raw_marginal_minimum=0.0,
            raw_marginal_maximum=0.0,
        )

    @property
    def sampling_mode(self):
        return 'adaptive_halton' if self.adaptive_halton else 'fixed_random'

    def _sample_count(self, region_area):
        if not self.adaptive_halton:
            return self.num_samples
        requested = int(np.ceil(self.sample_density * region_area))
        return int(np.clip(requested, self.min_samples, self.max_samples))

    def _draw_samples(self, lower, upper, sample_count):
        if not self.adaptive_halton:
            return self._rng.uniform(lower, upper, size=(sample_count, 2))
        unit_samples = self._halton.random(sample_count)
        return lower + unit_samples * (upper - lower)

    def evaluate(
        self,
        positions,
        predictions,
        collision_radius,
        cache_samples=False,
        reuse_last_samples=False,
    ):
        """Return joint collision probabilities shaped ``(rollouts, time)``.

        At each time step, one sample set covers every rollout's collision
        circle.  Each obstacle density is integrated over every circle first,
        using the equation-(10) shared samples and equation-(12) quadrature.
        The resulting marginal probabilities are then combined with equation
        (9).  This deliberately avoids treating Gaussian PDF values as
        pointwise probabilities in equation (11).
        """
        points = np.asarray(positions, dtype=np.float64)
        if points.ndim != 3 or points.shape[-1] != 2:
            raise ValueError('positions must have shape (K, T, 2)')
        if not np.all(np.isfinite(points)):
            raise ValueError('positions must be finite')
        rollout_count, horizon, _ = points.shape
        if predictions is None or predictions.empty:
            self._quadrature_cache = None
            self.reset_diagnostics()
            return np.zeros((rollout_count, horizon), dtype=np.float64)
        if predictions.means.shape[2] != horizon:
            raise ValueError('prediction and rollout horizons must match')
        radii = np.asarray(collision_radius, dtype=np.float64)
        if radii.ndim == 0:
            radii = np.full(predictions.means.shape[0], float(radii))
        if radii.shape != (predictions.means.shape[0],):
            raise ValueError('collision_radius must be scalar or have shape (O,)')
        if np.any(radii <= 0.0) or not np.all(np.isfinite(radii)):
            raise ValueError('collision radii must be finite and positive')
        maximum_radius = float(np.max(radii))

        started = time.perf_counter()
        result = np.zeros((rollout_count, horizon), dtype=np.float64)
        region_areas = np.empty(horizon, dtype=np.float64)
        sample_counts = np.empty(horizon, dtype=np.int64)
        cached = self._quadrature_cache
        can_reuse = bool(
            reuse_last_samples
            and cached is not None
            and cached['predictions'] is predictions
            and cached['horizon'] == horizon
            and np.allclose(cached['collision_radii'], radii)
        )
        if can_reuse:
            inverse_covariances = cached['inverse_covariances']
            density_normalizers = cached['density_normalizers']
        else:
            inverse_covariances, density_normalizers = (
                self._gaussian_parameters(predictions)
            )
        cached_steps = [] if cache_samples else None
        empty_circle_count = 0
        clipped_marginal_count = 0
        underresolved_steps = 0
        raw_marginal_minimum = np.inf
        raw_marginal_maximum = -np.inf
        for step in range(horizon):
            centers = points[:, step, :]
            lower = np.min(centers, axis=0) - maximum_radius
            upper = np.max(centers, axis=0) + maximum_radius
            region_area = float(np.prod(upper - lower))
            cached_step = cached['steps'][step] if can_reuse else None
            reuse_step = bool(
                cached_step is not None
                and np.all(lower >= cached_step['lower'] - 1e-12)
                and np.all(upper <= cached_step['upper'] + 1e-12)
            )
            if reuse_step:
                samples = cached_step['samples']
                obstacle_density = cached_step['obstacle_density']
                sample_count = samples.shape[0]
                region_area = float(np.prod(cached_step['upper']-cached_step['lower']))
            else:
                sample_count = self._sample_count(region_area)
                samples = self._draw_samples(lower, upper, sample_count)
                obstacle_density = self._mixture_densities(
                    samples,
                    predictions,
                    step,
                    inverse_covariances,
                    density_normalizers,
                )
            region_areas[step] = region_area
            sample_counts[step] = sample_count
            if cache_samples:
                cached_steps.append({
                    'lower': lower.copy(),
                    'upper': upper.copy(),
                    'samples': samples,
                    'obstacle_density': obstacle_density,
                })

            # Pairwise squared distances via a matrix product avoids allocating
            # a (rollouts, samples, 2) displacement tensor at every timestep.
            squared_distance = (
                np.sum(centers * centers, axis=1)[:, None]
                + np.sum(samples * samples, axis=1)[None, :]
                - 2.0 * centers @ samples.T
            )
            raw_marginals = np.zeros(
                (rollout_count, obstacle_density.shape[0]), dtype=np.float64)
            if np.all(radii == radii[0]):
                inside = squared_distance <= radii[0] * radii[0]
                counts = np.sum(inside, axis=1)
                empty_circle_count += int(np.count_nonzero(counts == 0))
                marginal_sums = inside @ obstacle_density.T
                # Samples are uniform on the shared rectangle B, not C.
                # Use area(B)/M * sum(1_C * density), never a conditional
                # in-circle mean. No hits means insufficient evaluation.
                raw_marginals = region_area/sample_count*marginal_sums
                raw_marginals[counts == 0] = 1.0
            else:
                # An obstacle's physical radius changes its integration disk.
                # Process one obstacle at a time to avoid a large (O,K,N) mask.
                for obstacle, radius in enumerate(radii):
                    inside = squared_distance <= radius * radius
                    counts = np.sum(inside, axis=1)
                    empty_circle_count += int(np.count_nonzero(counts == 0))
                    marginal_sums = inside @ obstacle_density[obstacle]
                    raw_marginals[:, obstacle] = region_area/sample_count*marginal_sums
                    raw_marginals[counts == 0, obstacle] = 1.0

            if (self.reject_underresolved and self.adaptive_halton
                    and sample_count+1e-9 < self.sample_density*region_area):
                # Cap was reached: this evaluation has not met its density
                # contract. Mark every candidate invalid for this time step.
                raw_marginals[:] = 1.
                underresolved_steps += 1
            finite_marginals = raw_marginals[np.isfinite(raw_marginals)]
            if finite_marginals.size:
                raw_marginal_minimum = min(
                    raw_marginal_minimum,
                    float(np.min(finite_marginals)),
                )
                raw_marginal_maximum = max(
                    raw_marginal_maximum,
                    float(np.max(finite_marginals)),
                )
            outside_probability_range = (
                ~np.isfinite(raw_marginals)
                | (raw_marginals < 0.0)
                | (raw_marginals > 1.0)
            )
            clipped_marginal_count += int(
                np.count_nonzero(outside_probability_range)
            )

            # PDF values are densities and are never clipped.  A finite-sample
            # quadrature estimate can nevertheless overshoot the mathematical
            # [0,1] range.  Clamp only the integrated marginals so equation (9)
            # always receives valid probabilities, and expose every correction
            # through diagnostics instead of silently hiding sampling error.
            marginals = np.nan_to_num(
                raw_marginals,
                nan=1.0,
                posinf=1.0,
                neginf=1.0,
            )
            marginals = np.clip(marginals, 0.0, 1.0)
            joint_probability = 1.0 - np.prod(1.0 - marginals, axis=1)
            result[:, step] = np.clip(joint_probability, 0.0, 1.0)
        if cache_samples:
            self._quadrature_cache = {
                'predictions': predictions,
                'horizon': horizon,
                'collision_radii': radii.copy(),
                'inverse_covariances': inverse_covariances,
                'density_normalizers': density_normalizers,
                'steps': cached_steps,
            }
        self.last_diagnostics = MonteCarloDiagnostics(
            sampling_mode=self.sampling_mode,
            region_areas=region_areas,
            sample_counts=sample_counts,
            total_samples=int(np.sum(sample_counts)),
            candidate_sample_comparisons=int(rollout_count*np.sum(sample_counts)),
            underresolved_steps=underresolved_steps,
            peak_pairwise_bytes=int(rollout_count*np.max(sample_counts)*8),
            compute_time_ms=(time.perf_counter() - started) * 1000.0,
            empty_circle_count=empty_circle_count,
            clipped_marginal_count=clipped_marginal_count,
            raw_marginal_minimum=(
                float(raw_marginal_minimum)
                if np.isfinite(raw_marginal_minimum)
                else float('nan')
            ),
            raw_marginal_maximum=(
                float(raw_marginal_maximum)
                if np.isfinite(raw_marginal_maximum)
                else float('nan')
            ),
        )
        return result

    def _gaussian_parameters(self, predictions):
        """Stabilize all covariances and factor their PDF terms in one batch."""
        min_variance = self.minimum_sigma * self.minimum_sigma
        covariance = predictions.covariances
        covariance = 0.5 * (covariance + np.swapaxes(covariance, -1, -2))
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, min_variance)
        covariance = np.einsum(
            '...ik,...k,...jk->...ij',
            eigenvectors,
            eigenvalues,
            eigenvectors,
            optimize=True,
        )
        covariance[..., 0, 0] += 1e-12
        covariance[..., 1, 1] += 1e-12

        # Explicit batched 2x2 inverse avoids a Python obstacle/mode loop and
        # is faster than repeatedly calling np.linalg.inv and det.
        xx = covariance[..., 0, 0]
        xy = covariance[..., 0, 1]
        yx = covariance[..., 1, 0]
        yy = covariance[..., 1, 1]
        determinant = np.maximum(xx * yy - xy * yx, 1e-18)
        inverse = np.empty_like(covariance)
        inverse[..., 0, 0] = yy / determinant
        inverse[..., 0, 1] = -xy / determinant
        inverse[..., 1, 0] = -yx / determinant
        inverse[..., 1, 1] = xx / determinant
        normalizer = 1.0 / (2.0 * np.pi * np.sqrt(determinant))
        return inverse, normalizer

    def _mixture_densities(
        self,
        samples,
        predictions,
        step,
        inverse_covariances=None,
        density_normalizers=None,
    ):
        """Evaluate all obstacle/mode Gaussian PDFs with batched NumPy math."""
        if inverse_covariances is None or density_normalizers is None:
            inverse_covariances, density_normalizers = (
                self._gaussian_parameters(predictions)
            )
        # Explicit 2-D quadratic form avoids generic einsum and a trailing
        # dimension-2 displacement allocation. No PDF truncation/approximation.
        dx = samples[None, None, :, 0]-predictions.means[:, :, step, None, 0]
        dy = samples[None, None, :, 1]-predictions.means[:, :, step, None, 1]
        inv = inverse_covariances[:, :, step]
        mahalanobis = (inv[:, :, None, 0, 0]*dx*dx
                       + (inv[:, :, None, 0, 1]+inv[:, :, None, 1, 0])*dx*dy
                       + inv[:, :, None, 1, 1]*dy*dy)
        mode_densities = (
            density_normalizers[:, :, step, None]
            * np.exp(-0.5 * np.maximum(mahalanobis, 0.0))
        )
        return np.sum(
            (
                predictions.mode_weights[:, :, None]
                if predictions.mode_weights.ndim == 2
                else predictions.mode_weights[:, :, step, None]
            ) * mode_densities,
            axis=1,
        )


class PredictionBudgetExceeded(ValueError):
    """Too many prediction objects to evaluate within the configured budget."""


def check_prediction_budget(config, predictions):
    """Reject an oversized input as a whole, without dropping any obstacles."""
    if (config.mode != 'off' and config.max_obstacles and predictions is not None
            and predictions.means.shape[0] > config.max_obstacles):
        raise PredictionBudgetExceeded(
            f'prediction obstacle budget exceeded: {predictions.means.shape[0]} > '
            f'{config.max_obstacles}; no tracks discarded')


def dynamic_risk_cost(
    positions,
    predictions,
    config,
    estimator=None,
    cache_samples=False,
    reuse_last_samples=False,
):
    """Evaluate paper-style soft and hard costs using joint CP."""
    rollout_positions = np.asarray(positions, dtype=np.float64)
    if rollout_positions.ndim != 3 or rollout_positions.shape[-1] != 2:
        raise ValueError('positions must have shape (K, T, 2)')
    rollout_count, horizon = rollout_positions.shape[:2]
    check_prediction_budget(config, predictions)
    if predictions is None or predictions.empty or config.mode == 'off':
        if estimator is not None:
            estimator.reset_diagnostics()
        return RiskEvaluation(
            cost=np.zeros(rollout_count, dtype=np.float64),
            collision_probability=np.zeros((rollout_count, horizon), dtype=np.float64),
        )
    if config.mode not in ('monte_carlo', 'gaussian_disk'):
        raise ValueError(f'unsupported dynamic risk mode: {config.mode}')

    calculator = estimator or make_probability_estimator(config,
        num_samples=config.mc_samples,
        minimum_sigma=config.minimum_sigma,
        seed=config.seed,
        adaptive_halton=config.adaptive_halton_mc,
        sample_density=config.mc_sample_density,
        min_samples=config.mc_samples_min,
        max_samples=config.mc_samples_max,
    )
    collision_radius = config.robot_radius + config.obstacle_radius
    if predictions.obstacle_radii is not None:
        collision_radius = config.robot_radius + predictions.obstacle_radii
    probability = calculator.evaluate(
        rollout_positions,
        predictions,
        collision_radius=collision_radius,
        cache_samples=cache_samples,
        reuse_last_samples=reuse_last_samples,
    )
    soft_cost = config.soft_weight * np.mean(probability, axis=1)
    hard_cost = config.hard_penalty * np.sum(
        probability > config.hard_threshold,
        axis=1,
    )
    return RiskEvaluation(
        cost=soft_cost + hard_cost,
        collision_probability=probability,
    )


def make_probability_estimator(config, *, final=False, **kwargs):
    """Use the same probability model with either shared MC or disk integration."""
    mode = config.mode
    if final and mode != 'off' and config.final_mode != 'inherit':
        mode = config.final_mode
    if mode == 'gaussian_disk':
        from .gaussian_disk import GaussianDiskCollisionProbability
        return GaussianDiskCollisionProbability(**kwargs)
    return MonteCarloJointCollisionProbability(**kwargs)
