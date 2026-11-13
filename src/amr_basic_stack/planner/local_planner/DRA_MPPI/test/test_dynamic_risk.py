import numpy as np
from scipy.stats import qmc

from dra_mppi.dynamic_risk import (
    DynamicRiskConfig,
    MonteCarloJointCollisionProbability,
    PredictionSet,
    dynamic_risk_cost,
)


def _single_obstacle_prediction(means, variance=0.02):
    means = np.asarray(means, dtype=np.float64)[None, ...]
    covariance = np.zeros((*means.shape[:2], 2, 2), dtype=np.float64)
    covariance[..., 0, 0] = variance
    covariance[..., 1, 1] = variance
    return PredictionSet(
        means,
        covariance,
        np.arange(1, means.shape[1] + 1, dtype=np.float64) * 0.1,
    )


def test_prediction_set_promotes_single_mode_and_normalises_weights():
    means = np.zeros((2, 3, 4, 2))
    covariances = np.broadcast_to(np.eye(2), (2, 3, 4, 2, 2)).copy()
    predictions = PredictionSet(
        means,
        covariances,
        np.arange(4),
        mode_weights=np.array([[2.0, 1.0, 1.0], [1.0, 0.0, 3.0]]),
    )

    assert predictions.means.shape == (2, 3, 4, 2)
    assert predictions.covariances.shape == (2, 3, 4, 2, 2)
    assert np.allclose(np.sum(predictions.mode_weights, axis=1), 1.0)


def test_prediction_set_accepts_and_normalises_time_varying_weights():
    means = np.zeros((1, 2, 3, 2))
    covariances = np.broadcast_to(np.eye(2), (1, 2, 3, 2, 2)).copy()
    predictions = PredictionSet(
        means,
        covariances,
        np.array([0.1, 0.2, 0.3]),
        mode_weights=np.array([[[3.0, 2.0, 1.0], [1.0, 2.0, 3.0]]]),
        obstacle_radii=np.array([0.35]),
    )
    assert predictions.mode_weights.shape == (1, 2, 3)
    assert np.allclose(np.sum(predictions.mode_weights, axis=1), 1.0)
    assert np.allclose(predictions.obstacle_radii, [0.35])


def test_time_varying_weight_is_selected_for_requested_step():
    means = np.array([[[[0.0, 0.0], [0.0, 0.0]],
                       [[4.0, 0.0], [4.0, 0.0]]]])
    covariances = np.broadcast_to(
        np.eye(2) * 0.1, (1, 2, 2, 2, 2)).copy()
    predictions = PredictionSet(
        means,
        covariances,
        np.array([0.1, 0.2]),
        mode_weights=np.array([[[1.0, 0.0], [0.0, 1.0]]]),
    )
    calculator = MonteCarloJointCollisionProbability(minimum_sigma=0.01)
    sample = np.array([[0.0, 0.0]])
    at_cv_step = calculator._mixture_densities(sample, predictions, step=0)
    at_ct_step = calculator._mixture_densities(sample, predictions, step=1)
    assert at_cv_step[0, 0] > 1.0
    assert at_ct_step[0, 0] < 1e-20


def test_per_obstacle_radius_changes_collision_probability():
    positions = np.zeros((1, 1, 2))
    base = _single_obstacle_prediction([[0.0, 0.0]], variance=0.2)
    small = PredictionSet(
        base.means, base.covariances, base.times,
        obstacle_radii=np.array([0.05]))
    large = PredictionSet(
        base.means, base.covariances, base.times,
        obstacle_radii=np.array([0.40]))
    config = DynamicRiskConfig(
        robot_radius=0.1,
        obstacle_radius=0.25,
        mc_samples=30000,
        seed=41,
    )
    small_probability = dynamic_risk_cost(
        positions, small, config).collision_probability[0, 0]
    large_probability = dynamic_risk_cost(
        positions, large, config).collision_probability[0, 0]
    assert large_probability > small_probability


def test_crossing_trajectory_has_more_cp_and_cost_than_distant_trajectory():
    positions = np.array([
        [[0.5, 0.0], [1.0, 0.0], [1.5, 0.0]],
        [[0.5, 2.0], [1.0, 2.0], [1.5, 2.0]],
    ])
    predictions = _single_obstacle_prediction(
        [[0.5, 0.0], [1.0, 0.0], [1.5, 0.0]]
    )

    evaluation = dynamic_risk_cost(
        positions,
        predictions,
        DynamicRiskConfig(
            robot_radius=0.20,
            obstacle_radius=0.20,
            mc_samples=16000,
            hard_threshold=0.05,
            seed=3,
        ),
    )

    assert evaluation.collision_probability.shape == (2, 3)
    assert np.all((evaluation.collision_probability >= 0.0))
    assert np.all((evaluation.collision_probability <= 1.0))
    assert evaluation.maximum_probability[0] > evaluation.maximum_probability[1]
    assert evaluation.cost[0] > evaluation.cost[1]
    assert evaluation.cost[0] >= 100000.0


def test_joint_probability_increases_when_a_second_obstacle_is_added():
    variance = 0.50
    radius = 0.25
    positions = np.zeros((1, 1, 2))
    one = _single_obstacle_prediction([[0.0, 0.0]], variance=variance)
    means = np.repeat(one.means, 2, axis=0)
    covariances = np.repeat(one.covariances, 2, axis=0)
    two = PredictionSet(means, covariances, one.times)
    calculator_one = MonteCarloJointCollisionProbability(
        num_samples=40000,
        seed=11,
    )
    calculator_two = MonteCarloJointCollisionProbability(
        num_samples=40000,
        seed=11,
    )

    cp_one = calculator_one.evaluate(positions, one, collision_radius=radius)
    cp_two = calculator_two.evaluate(positions, two, collision_radius=radius)

    # Integrate each obstacle first, then combine the two probabilities with
    # equation (9).  Gaussian densities themselves are not probabilities.
    marginal = 1.0 - np.exp(-(radius * radius) / (2.0 * variance))
    expected = 1.0 - (1.0 - marginal) ** 2

    assert cp_two[0, 0] > cp_one[0, 0]
    assert np.isclose(cp_two[0, 0], expected, atol=0.002)


def test_narrow_gaussian_pdf_is_not_clipped_as_a_probability():
    sigma = 0.05
    radius = 0.10
    positions = np.zeros((1, 1, 2))
    predictions = _single_obstacle_prediction(
        [[0.0, 0.0]],
        variance=sigma * sigma,
    )
    calculator = MonteCarloJointCollisionProbability(
        num_samples=100000,
        minimum_sigma=0.01,
        seed=21,
    )

    probability = calculator.evaluate(
        positions,
        predictions,
        collision_radius=radius,
    )[0, 0]
    analytic = 1.0 - np.exp(-(radius * radius) / (2.0 * sigma * sigma))

    # The Gaussian peak is about 63.7 m^-2.  Treating that density as a
    # pointwise [0,1] probability would underestimate this integral severely.
    assert 1.0 / (2.0 * np.pi * sigma * sigma) > 1.0
    assert np.isclose(probability, analytic, atol=0.01)
    assert 0.0 <= probability <= 1.0


def test_integrating_before_equation_9_fixes_narrow_overlapping_gaussians():
    sigma = 0.05
    radius = 0.10
    positions = np.zeros((1, 1, 2))
    one = _single_obstacle_prediction(
        [[0.0, 0.0]],
        variance=sigma * sigma,
    )
    two = PredictionSet(
        np.repeat(one.means, 2, axis=0),
        np.repeat(one.covariances, 2, axis=0),
        one.times,
    )
    calculator = MonteCarloJointCollisionProbability(
        num_samples=100000,
        minimum_sigma=0.01,
        seed=21,
    )

    probability = calculator.evaluate(
        positions,
        two,
        collision_radius=radius,
    )[0, 0]

    marginal = 1.0 - np.exp(-(radius * radius) / (2.0 * sigma * sigma))
    expected = 1.0 - (1.0 - marginal) ** 2

    # The Gaussian peak is much greater than one, but only integrated
    # probabilities enter equation (9), so overlapping obstacles stay valid.
    assert 1.0 / (2.0 * np.pi * sigma * sigma) > 1.0
    assert np.isclose(probability, expected, atol=0.01)
    assert 0.0 <= probability <= 1.0


def test_batched_multimode_density_matches_scalar_reference():
    samples = np.array([
        [-0.4, 0.2],
        [0.1, -0.3],
        [0.7, 0.6],
    ])
    means = np.array([
        [
            [[0.0, 0.0]],
            [[0.5, 0.2]],
        ],
        [
            [[-0.2, 0.4]],
            [[0.8, -0.1]],
        ],
    ])
    covariances = np.array([
        [
            [[[0.12, 0.03], [0.03, 0.08]]],
            [[[0.20, -0.02], [-0.02, 0.10]]],
        ],
        [
            [[[0.09, 0.01], [0.01, 0.16]]],
            [[[0.14, 0.04], [0.04, 0.18]]],
        ],
    ])
    predictions = PredictionSet(
        means,
        covariances,
        np.array([0.1]),
        mode_weights=np.array([[0.7, 0.3], [0.25, 0.75]]),
    )
    calculator = MonteCarloJointCollisionProbability(minimum_sigma=0.01)

    actual = calculator._mixture_densities(samples, predictions, step=0)
    expected = np.zeros_like(actual)
    for obstacle in range(2):
        for mode in range(2):
            covariance = covariances[obstacle, mode, 0]
            inverse = np.linalg.inv(covariance)
            determinant = np.linalg.det(covariance)
            delta = samples - means[obstacle, mode, 0]
            mahalanobis = np.einsum('ni,ij,nj->n', delta, inverse, delta)
            normalizer = 1.0 / (2.0 * np.pi * np.sqrt(determinant))
            expected[obstacle] += (
                predictions.mode_weights[obstacle, mode]
                * normalizer
                * np.exp(-0.5 * mahalanobis)
            )

    assert np.allclose(actual, expected, rtol=1e-10, atol=1e-10)


def test_modes_of_one_obstacle_are_mixture_alternatives_not_obstacles():
    radius = 0.25
    positions = np.zeros((2, 1, 2))
    positions[1, 0] = [0.2, 0.0]
    one = _single_obstacle_prediction([[0.0, 0.0]], variance=0.08)
    multimode = PredictionSet(
        np.repeat(one.means, 3, axis=1),
        np.repeat(one.covariances, 3, axis=1),
        one.times,
        mode_weights=np.array([[0.2, 0.3, 0.5]]),
    )
    calculator_one = MonteCarloJointCollisionProbability(
        num_samples=20000,
        seed=31,
    )
    calculator_multimode = MonteCarloJointCollisionProbability(
        num_samples=20000,
        seed=31,
    )

    cp_one = calculator_one.evaluate(positions, one, collision_radius=radius)
    cp_multimode = calculator_multimode.evaluate(
        positions,
        multimode,
        collision_radius=radius,
    )

    # Repeating an identical hypothesis with weights summing to one must not
    # count one physical obstacle three times.
    assert np.allclose(cp_multimode, cp_one, rtol=1e-12, atol=1e-12)


def test_integrated_probability_clipping_is_visible_in_diagnostics():
    sigma = 0.05
    radius = 0.10
    positions = np.zeros((1, 1, 2))
    predictions = _single_obstacle_prediction(
        [[0.0, 0.0]],
        variance=sigma * sigma,
    )
    calculator = MonteCarloJointCollisionProbability(
        num_samples=1,
        minimum_sigma=0.01,
        seed=21,
    )
    # Deliberately force a pathological one-point quadrature at the Gaussian
    # peak.  The PDF remains untouched; only its invalid integral estimate is
    # clamped before equation (9), and the correction must remain observable.
    calculator._draw_samples = lambda lower, upper, count: np.zeros((count, 2))

    probability = calculator.evaluate(
        positions,
        predictions,
        collision_radius=radius,
    )[0, 0]
    diagnostics = calculator.last_diagnostics

    assert probability == 1.0
    assert diagnostics.raw_marginal_maximum > 1.0
    assert diagnostics.clipped_marginal_count == 1
    assert diagnostics.empty_circle_count == 0


def test_soft_and_hard_costs_are_both_applied_to_joint_cp():
    positions = np.zeros((1, 2, 2))
    predictions = _single_obstacle_prediction(
        [[0.0, 0.0], [0.0, 0.0]],
        variance=0.05,
    )
    config = DynamicRiskConfig(
        soft_weight=7.0,
        robot_radius=0.15,
        obstacle_radius=0.15,
        mc_samples=30000,
        hard_threshold=0.01,
        hard_penalty=1234.0,
        seed=5,
    )
    evaluation = dynamic_risk_cost(positions, predictions, config)
    cp = evaluation.collision_probability[0]
    expected = 7.0 * np.mean(cp) + 1234.0 * np.sum(cp > 0.01)

    assert np.isclose(evaluation.cost[0], expected)


def test_off_mode_returns_zero_cost_and_probability():
    empty = PredictionSet(
        np.empty((0, 2, 2)),
        np.empty((0, 2, 2, 2)),
        np.array([0.1, 0.2]),
    )
    evaluation = dynamic_risk_cost(
        np.zeros((3, 2, 2)),
        empty,
        DynamicRiskConfig(mode='off'),
    )
    assert np.all(evaluation.cost == 0.0)
    assert np.all(evaluation.collision_probability == 0.0)


def test_fixed_random_mode_remains_the_default_and_uses_fixed_sample_count():
    positions = np.zeros((2, 2, 2))
    predictions = _single_obstacle_prediction([[0.0, 0.0], [0.0, 0.0]])
    calculator = MonteCarloJointCollisionProbability(num_samples=37, seed=8)

    calculator.evaluate(positions, predictions, collision_radius=0.25)
    diagnostics = calculator.last_diagnostics

    assert diagnostics.sampling_mode == 'fixed_random'
    assert np.array_equal(diagnostics.sample_counts, [37, 37])
    assert diagnostics.total_samples == 74


def test_fixed_random_mode_preserves_the_legacy_numpy_rng_samples():
    lower = np.array([-2.0, 3.0])
    upper = np.array([4.0, 5.0])
    calculator = MonteCarloJointCollisionProbability(
        num_samples=16,
        seed=12,
    )
    expected = np.random.default_rng(12).uniform(lower, upper, size=(16, 2))

    actual = calculator._draw_samples(lower, upper, 16)

    assert np.array_equal(actual, expected)


def test_adaptive_sample_count_uses_region_area_not_timestep_index():
    positions = np.array(
        [
            [[0.0, 0.0], [0.0, 0.0]],
            [[0.5, 0.0], [3.0, 2.0]],
        ]
    )
    predictions = _single_obstacle_prediction([[0.0, 0.0], [0.0, 0.0]])
    calculator = MonteCarloJointCollisionProbability(
        adaptive_halton=True,
        sample_density=100.0,
        min_samples=20,
        max_samples=200,
        seed=9,
    )

    calculator.evaluate(positions, predictions, collision_radius=0.25)
    diagnostics = calculator.last_diagnostics

    assert diagnostics.sampling_mode == 'adaptive_halton'
    assert np.allclose(diagnostics.region_areas, [0.5, 8.75])
    assert np.array_equal(diagnostics.sample_counts, [50, 200])
    assert diagnostics.total_samples == 250
    assert diagnostics.compute_time_ms > 0.0


def test_adaptive_mode_maps_two_dimensional_halton_points_into_region():
    lower = np.array([-2.0, 3.0])
    upper = np.array([4.0, 5.0])
    calculator = MonteCarloJointCollisionProbability(
        adaptive_halton=True,
        seed=12,
    )
    expected_engine = qmc.Halton(d=2, scramble=True, seed=12)
    expected = lower + expected_engine.random(16) * (upper - lower)

    actual = calculator._draw_samples(lower, upper, 16)

    assert np.allclose(actual, expected)
    assert np.all(actual >= lower)
    assert np.all(actual <= upper)


def test_final_subset_can_reuse_cached_quadrature_without_changing_cp():
    positions = np.array([
        [[0.0, 0.0], [0.2, 0.0]],
        [[0.0, 0.4], [0.2, 0.4]],
    ])
    predictions = _single_obstacle_prediction([[0.0, 0.0], [0.2, 0.0]])
    calculator = MonteCarloJointCollisionProbability(
        adaptive_halton=True,
        sample_density=400.0,
        min_samples=128,
        max_samples=512,
        seed=21,
    )

    primary = calculator.evaluate(
        positions,
        predictions,
        collision_radius=0.25,
        cache_samples=True,
    )
    reused = calculator.evaluate(
        positions[:1],
        predictions,
        collision_radius=0.25,
        reuse_last_samples=True,
    )

    assert np.allclose(reused[0], primary[0], rtol=0.0, atol=1e-15)
